// Detector de frutas por color y forma (OpenCV en C++: en la Pi gasta ~25 % menos CPU que Python, ver
// docs/mision-frutas-plan.md, fase 0). Un solo proceso y una sola cámara abierta a la vez: la que pide la misión en
// el tópico `camara` (std_msgs/String, transient_local): "frutas", "garra" o "" (ninguna, no gasta CPU).
//   frutas  cam_frutas (al frente): todas las frutas en el suelo -> frutas/detecciones
//           {"frutas": [{"tipo", "x", "y", "d", "px"}]}  x, y en base_link (m); d = diámetro estimado
//   garra   cam_garra (mira hacia abajo entre los dedos): la fruta más grande -> garra/fruta
//           {"visible", "x", "y", "tipo", "area"}
// Clasificación por tono (medido 2026-11-02): el limón es verde (H ~43-49), la naranja amarilla (H ~22). La lima de
// las pruebas es oscura (V de 17 a 62 en cam_garra): con v_min = 50 casi no pasaba y la detección parpadeaba.
// Posición en el suelo:
//   frutas  la fila donde la fruta toca el suelo da la distancia (cámara a ~8 cm, casi horizontal):
//           z = k_suelo / (fila - fila_horizonte); si la fruta toca el borde de abajo, z = f · diámetro / ancho
//   garra   afín píxel -> suelo ajustada con 4 puntos (error ±0.6 cm)
// Con depuracion:=true guarda cada segundo el último cuadro con las manchas en ~/.cache/wallens/detector/<cámara>.jpg.
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace {

struct Mancha {
  std::string tipo;
  double u, v;          // centroide (px)
  double fondo;         // fila más baja (px)
  double ancho;         // px
  double area;
  cv::Rect caja;
};

std::string num(double x) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(4) << x;
  return s.str();
}

}  // namespace

class DetectorFrutas : public rclcpp::Node {
 public:
  DetectorFrutas() : Node("detector_frutas") {
    pedida_ = declare_parameter<std::string>("camara", "");          // al arrancar; luego manda el tópico
    disp_frutas_ = declare_parameter<std::string>("dispositivo_frutas", "/dev/cam_frutas");
    disp_garra_ = declare_parameter<std::string>("dispositivo_garra", "/dev/cam_garra");
    // la C270 en MJPG llena el log de "Corrupt JPEG"; una sola cámara abierta: YUYV cabe en el USB
    formato_frutas_ = declare_parameter<std::string>("formato_frutas", "YUYV");
    formato_garra_ = declare_parameter<std::string>("formato_garra", "MJPG");
    ancho_ = declare_parameter<int>("ancho", 640);
    alto_ = declare_parameter<int>("alto", 480);
    fps_ = declare_parameter<int>("fps", 15);
    descartar_ = declare_parameter<int>("descartar", 5);            // cuadros tras abrir: exposición sin ajustar
    h_min_ = declare_parameter<int>("h_min", 10);
    h_max_ = declare_parameter<int>("h_max", 75);
    s_min_ = declare_parameter<int>("s_min", 60);                    // suelo: S <= 30; lima: S mediana 118
    v_min_ = declare_parameter<int>("v_min", 20);                    // lima: V de 17 a 62
    h_corte_ = declare_parameter<double>("h_corte", 32.0);         // tono mediano: debajo naranja, encima limón
    area_min_ = declare_parameter<double>("area_min", 150.0);
    circularidad_min_ = declare_parameter<double>("circularidad_min", 0.55);
    // de cerca (cam_garra) la sombra de la fruta se pega a la mancha y la deja poco redonda (0.49 medido): vale igual
    // si el círculo inscrito ocupa al menos esto de la mancha, y la fruta es ese círculo (limón con sombra: 0.76)
    llenado_min_ = declare_parameter<double>("llenado_min", 0.5);
    depuracion_ = declare_parameter<bool>("depuracion", false);
    // el pool de hilos de OpenCV reparte cada operación pequeña en 4 núcleos y espera girando: ~1.9 núcleos con una
    // cámara a 7.5 Hz (medido 2026-11-02); con 1 hilo hace lo mismo
    cv::setNumThreads(static_cast<int>(declare_parameter<int>("hilos_opencv", 1)));
    // cam_frutas
    f_ = declare_parameter<double>("f", 770.0);
    cx_ = declare_parameter<double>("cx", 320.0);
    fila_horizonte_ = declare_parameter<double>("fila_horizonte", 256.6);
    k_suelo_ = declare_parameter<double>("k_suelo", 63.4);        // f · altura de la cámara (px · m)
    cam_x_ = declare_parameter<double>("cam_x", 0.10);
    d_limon_ = declare_parameter<double>("diametro_limon", 0.049);
    d_naranja_ = declare_parameter<double>("diametro_naranja", 0.061);
    // diámetro aparente (ancho en px a la distancia del suelo) / diámetro de la fruta; frutas reales: 0.87-1.0
    tam_min_ = declare_parameter<double>("tam_min", 0.6);
    tam_max_ = declare_parameter<double>("tam_max", 1.6);
    aspecto_max_ = declare_parameter<double>("aspecto_max", 1.5);   // alto / ancho; una fruta en el suelo, <= 1
    // cam_garra: x = gx[0] + gx[1] u + gx[2] v, y = gy[0] + gy[1] u + gy[2] v (m)
    gx_ = declare_parameter<std::vector<double>>("garra_x", {-0.12581, -0.000019, -0.000403});
    gy_ = declare_parameter<std::vector<double>>("garra_y", {0.075515, -0.000277, 0.000008});
    const char* home = std::getenv("HOME");
    dir_depuracion_ = std::string(home ? home : "/tmp") + "/.cache/wallens/detector";

    pub_frutas_ = create_publisher<std_msgs::msg::String>("frutas/detecciones", 5);
    pub_garra_ = create_publisher<std_msgs::msg::String>("garra/fruta", 5);
    sub_camara_ = create_subscription<std_msgs::msg::String>(
        "camara", rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::String::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex_);
          pedida_ = m->data;
        });
    hilo_ = std::thread([this] { bucle(); });
    RCLCPP_INFO(get_logger(), "detector_frutas: espera el tópico camara (frutas | garra | \"\")");
  }

  ~DetectorFrutas() override {
    fin_ = true;
    if (hilo_.joinable()) hilo_.join();
  }

 private:
  bool abrir(cv::VideoCapture& cap, const std::string& camara) {
    const std::string& dispositivo = camara == "garra" ? disp_garra_ : disp_frutas_;
    const std::string& formato = camara == "garra" ? formato_garra_ : formato_frutas_;
    cap.open(dispositivo, cv::CAP_V4L2);
    if (!cap.isOpened()) return false;
    if (formato.size() == 4)
      cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(formato[0], formato[1], formato[2], formato[3]));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, ancho_);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, alto_);
    cap.set(cv::CAP_PROP_FPS, fps_);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    return true;
  }

  void bucle() {
    cv::VideoCapture cap;
    cv::Mat cuadro;
    std::string activa;
    int saltar = 0;
    auto ultima_foto = std::chrono::steady_clock::now();
    while (!fin_ && rclcpp::ok()) {
      std::string pedida;
      {
        std::lock_guard<std::mutex> l(mutex_);
        pedida = pedida_;
      }
      if (pedida != "frutas" && pedida != "garra") pedida.clear();
      if (pedida != activa) {                         // cierra la que había antes de abrir otra: nunca las dos
        cap.release();
        activa = pedida;
        saltar = descartar_;
        RCLCPP_INFO(get_logger(), "cámara: %s", activa.empty() ? "ninguna" : activa.c_str());
      }
      if (activa.empty()) {
        std::this_thread::sleep_for(100ms);
        continue;
      }
      if (!cap.isOpened() && !abrir(cap, activa)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No puedo abrir la cámara %s", activa.c_str());
        std::this_thread::sleep_for(1s);
        continue;
      }
      if (!cap.read(cuadro) || cuadro.empty()) {
        RCLCPP_WARN(get_logger(), "cámara %s sin cuadros: la reabro", activa.c_str());
        cap.release();
        std::this_thread::sleep_for(500ms);
        continue;
      }
      if (saltar > 0) {
        --saltar;
        continue;
      }
      auto manchas = buscar(cuadro, activa == "frutas");
      std_msgs::msg::String m;
      m.data = activa == "garra" ? json_garra(manchas) : json_frutas(manchas);
      (activa == "garra" ? pub_garra_ : pub_frutas_)->publish(m);
      if (depuracion_ && std::chrono::steady_clock::now() - ultima_foto > 1s) {
        ultima_foto = std::chrono::steady_clock::now();
        guardar(cuadro, manchas, activa);
      }
    }
  }

  std::vector<Mancha> buscar(const cv::Mat& bgr, bool frutas) {
    cv::Mat hsv, mascara;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, 255, 255), mascara);
    cv::morphologyEx(mascara, mascara, cv::MORPH_OPEN, cv::Mat::ones(5, 5, CV_8U));
    cv::morphologyEx(mascara, mascara, cv::MORPH_CLOSE, cv::Mat::ones(7, 7, CV_8U));
    std::vector<std::vector<cv::Point>> contornos;
    cv::findContours(mascara, contornos, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<Mancha> salida;
    for (const auto& c : contornos) {
      double area = cv::contourArea(c);
      if (area < area_min_) continue;
      cv::Rect r = cv::boundingRect(c);
      // cam_frutas mira casi horizontal: una fruta en el suelo queda bajo el horizonte. Lo que llega al borde de arriba
      // es alto (marcos de puerta de madera, muebles) y además se saltaría el filtro de forma por estar cortado
      if (frutas && r.y <= 1) continue;
      bool cortada = r.x <= 1 || r.y <= 1 || r.x + r.width >= bgr.cols - 1 || r.y + r.height >= bgr.rows - 1;
      double perimetro = cv::arcLength(c, true);
      double circ = perimetro > 0 ? 4 * M_PI * area / (perimetro * perimetro) : 0;
      cv::Mat m1 = cv::Mat::zeros(mascara.size(), CV_8U);
      cv::drawContours(m1, std::vector<std::vector<cv::Point>>{c}, 0, 255, cv::FILLED);
      cv::Moments mo = cv::moments(c);
      double u = mo.m10 / mo.m00, v = mo.m01 / mo.m00, fondo = r.y + r.height, ancho = r.width;
      if (!cortada && circ < circularidad_min_) {   // forma: redonda (cortada por el borde no se puede saber)
        cv::Mat dist;
        cv::distanceTransform(m1(r), dist, cv::DIST_L2, 5);
        double radio;
        cv::Point centro;
        cv::minMaxLoc(dist, nullptr, &radio, nullptr, &centro);
        if (M_PI * radio * radio < llenado_min_ * area) continue;
        u = r.x + centro.x, v = r.y + centro.y, fondo = v + radio, ancho = 2 * radio;   // la fruta, sin la sombra
        r = cv::Rect(cv::Point(u - radio, v - radio), cv::Point(u + radio, v + radio)) &
            cv::Rect(0, 0, bgr.cols, bgr.rows);
        m1.setTo(0);
        cv::circle(m1, cv::Point(u, v), static_cast<int>(radio), 255, cv::FILLED);
      }
      cv::bitwise_and(m1, mascara, m1);
      std::vector<uchar> tonos;
      for (int y = r.y; y < r.y + r.height; y += 2) {
        const uchar* mk = m1.ptr<uchar>(y);
        const cv::Vec3b* px = hsv.ptr<cv::Vec3b>(y);
        for (int x = r.x; x < r.x + r.width; x += 2)
          if (mk[x]) tonos.push_back(px[x][0]);
      }
      if (tonos.empty()) continue;
      std::nth_element(tonos.begin(), tonos.begin() + tonos.size() / 2, tonos.end());
      double tono = tonos[tonos.size() / 2];
      salida.push_back({tono < h_corte_ ? "naranja" : "limon", u, v, fondo, ancho, area, r});
    }
    return salida;
  }

  std::string json_frutas(const std::vector<Mancha>& manchas) {
    std::ostringstream s;
    s << "{\"frutas\": [";
    bool primera = true;
    for (const auto& m : manchas) {
      double diam = m.tipo == "naranja" ? d_naranja_ : d_limon_;
      double z;
      if (m.fondo >= alto_ - 2) {
        z = f_ * diam / std::max(m.ancho, 1.0);              // toca el borde de abajo: por el tamaño
      } else {
        if (m.fondo <= fila_horizonte_ + 3) continue;          // sobre el horizonte: no está en el suelo
        z = k_suelo_ / (m.fondo - fila_horizonte_);
      }
      // cortada por un lado, el ancho miente y el alto sigue entero
      bool lateral = m.caja.x <= 1 || m.caja.x + m.caja.width >= ancho_ - 1;
      // alta y estrecha: pata de silla o de mesa, canto de una puerta que no llega arriba
      if (!lateral && m.caja.height > aspecto_max_ * m.caja.width) continue;
      double y = -(m.u - cx_) * z / f_;
      double d = m.ancho * z / f_;
      // tamaño imposible para esa distancia: no es la fruta (pie de un marco de puerta, una papelera al fondo)
      double tam = std::max(d, m.caja.height * z / f_);
      if (tam < tam_min_ * diam || tam > tam_max_ * diam) continue;
      s << (primera ? "" : ", ") << "{\"tipo\": \"" << m.tipo << "\", \"x\": " << num(cam_x_ + z) << ", \"y\": "
        << num(y) << ", \"d\": " << num(d) << ", \"px\": " << static_cast<int>(m.ancho) << "}";
      primera = false;
    }
    s << "]}";
    return s.str();
  }

  std::string json_garra(const std::vector<Mancha>& manchas) {
    if (manchas.empty()) return "{\"visible\": false, \"x\": 0, \"y\": 0, \"tipo\": \"\", \"area\": 0}";
    const auto& m = *std::max_element(manchas.begin(), manchas.end(),
                                      [](const Mancha& a, const Mancha& b) { return a.area < b.area; });
    double x = gx_[0] + gx_[1] * m.u + gx_[2] * m.v;
    double y = gy_[0] + gy_[1] * m.u + gy_[2] * m.v;
    return "{\"visible\": true, \"x\": " + num(x) + ", \"y\": " + num(y) + ", \"tipo\": \"" + m.tipo +
           "\", \"area\": " + std::to_string(static_cast<int>(m.area)) + "}";
  }

  void guardar(const cv::Mat& bgr, const std::vector<Mancha>& manchas, const std::string& camara) {
    cv::Mat foto = bgr.clone();
    for (const auto& m : manchas) {
      cv::rectangle(foto, m.caja, m.tipo == "limon" ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255), 2);
      cv::putText(foto, m.tipo + " " + std::to_string(static_cast<int>(m.area)), m.caja.tl() + cv::Point(0, -4),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    std::error_code e;
    std::filesystem::create_directories(dir_depuracion_, e);
    cv::imwrite(dir_depuracion_ + "/" + camara + ".jpg", foto);
  }

  std::string pedida_, disp_frutas_, disp_garra_, formato_frutas_, formato_garra_, dir_depuracion_;
  int ancho_, alto_, fps_, descartar_, h_min_, h_max_, s_min_, v_min_;
  double tam_min_, tam_max_, aspecto_max_, h_corte_, area_min_, circularidad_min_, llenado_min_, f_, cx_, fila_horizonte_, k_suelo_, cam_x_, d_limon_,
      d_naranja_;
  bool depuracion_;
  std::vector<double> gx_, gy_;
  std::mutex mutex_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_frutas_, pub_garra_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_camara_;
  std::atomic<bool> fin_{false};
  std::thread hilo_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectorFrutas>());
  rclcpp::shutdown();
  return 0;
}
