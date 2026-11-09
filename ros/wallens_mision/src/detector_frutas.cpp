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
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_destinos.hpp"

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
  explicit DetectorFrutas(bool camaras=true) : Node("detector_frutas") {
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
    d_limon_ = declare_parameter<double>("diametro_limon", 0.050);
    d_naranja_ = declare_parameter<double>("diametro_naranja", 0.070);
    d_mandarina_ = declare_parameter<double>("diametro_mandarina", 0.0); // POR MEDIR; baldosa bloqueada para este tipo
    ancho_marco_ = declare_parameter<double>("ancho_marco", 0.0);       // POR MEDIR en metros
    costo_max_ = declare_parameter<double>("costo_clase_max", 3.0);
    margen_clase_ = declare_parameter<double>("margen_clase", 0.25);
    // Mismas cinco características y escalas de voz_fruta_pi4/fruta.py; muestras en config/frutas.yaml.
    ref_limon_ = declare_parameter<std::vector<double>>("referencias_limon", {38,4,160,0.3223968742,0.9682539683,38,4,156,0.3199901558,0.9836065574,37,7,149,0.3845241741,0.9219858701});
    ref_naranja_ = declare_parameter<std::vector<double>>("referencias_naranja", {25,5,199,0.2516708237,1,25,5,198,0.2469782906,0.9841270521,25,5,196,0.2524993630,1});
    ref_mandarina_ = declare_parameter<std::vector<double>>("referencias_mandarina", {31,13,180,0.3910305946,0.6985294118,36,11,152,0.3750828398,0.7253640510,35,7,151,0.3877671683,0.6932118306});
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
    pub_cajas_ = create_publisher<std_msgs::msg::String>("cajas/detecciones", 5);
    pub_baldosas_ = create_publisher<std_msgs::msg::String>("baldosas/detecciones", 5);
    sub_camara_ = create_subscription<std_msgs::msg::String>(
        "camara", rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::String::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mutex_);
          pedida_ = m->data;
        });
    if(camaras) hilo_ = std::thread([this] { bucle(); });
    RCLCPP_INFO(get_logger(), "detector_frutas: espera el tópico camara (frutas | garra | \"\")");
  }

  ~DetectorFrutas() override {
    fin_ = true;
    if (hilo_.joinable()) hilo_.join();
  }

  std::string evaluar(const cv::Mat& bgr, const std::string& modo) {
    if(modo=="cajas")return json_cajas(bgr);
    auto manchas=buscar(bgr,modo!="garra");
    if(modo=="baldosas")return json_baldosas(bgr,manchas);
    return modo=="garra" ? json_garra(manchas) : json_frutas(bgr,manchas);
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
      if (pedida != "frutas" && pedida != "garra" && pedida != "cajas" && pedida != "baldosas") pedida.clear();
      if (pedida != activa) {                         // cierra la que había antes de abrir otra: nunca las dos
        if (activa.empty() || pedida.empty() || activa == "garra" || pedida == "garra") cap.release();
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
      auto manchas = activa == "cajas" ? std::vector<Mancha>{} : buscar(cuadro, activa != "garra");
      std_msgs::msg::String m;
      if (activa == "cajas") {m.data=json_cajas(cuadro);pub_cajas_->publish(m);}
      else if (activa == "baldosas") {m.data=json_baldosas(cuadro,manchas);pub_baldosas_->publish(m);}
      else {m.data = activa == "garra" ? json_garra(manchas) : json_frutas(cuadro, manchas);
        (activa == "garra" ? pub_garra_ : pub_frutas_)->publish(m);}
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
      std::vector<uchar> tonos, saturaciones;
      double suma_v=0, suma_v2=0;
      for (int y = r.y; y < r.y + r.height; y += 2) {
        const uchar* mk = m1.ptr<uchar>(y);
        const cv::Vec3b* px = hsv.ptr<cv::Vec3b>(y);
        for (int x = r.x; x < r.x + r.width; x += 2)
          if (mk[x]) {tonos.push_back(px[x][0]);saturaciones.push_back(px[x][1]);
            suma_v+=px[x][2];suma_v2+=double(px[x][2])*px[x][2];}
      }
      if (tonos.empty()) continue;
      std::sort(tonos.begin(),tonos.end());
      std::sort(saturaciones.begin(),saturaciones.end());
      double tono = tonos[tonos.size() / 2];
      auto lados=cv::minAreaRect(c).size;
      double media_v=suma_v/tonos.size();
      std::array<double,5> medidas={tono, double(tonos[(tonos.size()-1)*9/10]-tonos[(tonos.size()-1)/10]),
        double(saturaciones[saturaciones.size()/2]),std::sqrt(std::max(0.0,suma_v2/tonos.size()-media_v*media_v))/std::max(1.0,media_v),
        std::min(lados.width,lados.height)/std::max(1.0f,std::max(lados.width,lados.height))};
      std::string tipo=clasificar(medidas);
      if(tipo.empty() && frutas)continue;  // de cerca solo interesa visible, no el tipo
      salida.push_back({tipo, u, v, fondo, ancho, area, r});
    }
    return salida;
  }

  std::string json_frutas(const cv::Mat& bgr, const std::vector<Mancha>& manchas) {
    std::ostringstream s;
    s << "{\"frutas\": [";
    bool primera = true;
    // 2026-11-04: la foto impresa de las tarjetas no es fruta. Se descarta lo que cae dentro de un marco negro leído
    // (de cerca) y lo que queda ENCIMA de una caja turquesa (la tarjeta va en la tapa del fondo): una fruta real en
    // el piso junto a la caja queda más abajo que el frente turquesa en la imagen.
    std::vector<cv::Rect> zonas_tarjeta;
    for (auto& mc : vision_destinos::marcos(bgr)) {
      cv::Rect r = cv::boundingRect(mc.esquinas);
      zonas_tarjeta.push_back(cv::Rect(r.x - r.width / 4, r.y - r.height / 4, r.width * 3 / 2, r.height * 3 / 2));
    }
    auto cajas_turquesa = vision_destinos::turquesas(bgr);
    for (const auto& m : manchas) {
      bool en_tarjeta = false;
      for (const auto& z : zonas_tarjeta) en_tarjeta = en_tarjeta || z.contains(cv::Point2d(m.u, m.v));
      for (const auto& t : cajas_turquesa)
        en_tarjeta = en_tarjeta || (m.u >= t.x && m.u <= t.x + t.width && m.fondo <= t.y + t.height);
      if (en_tarjeta) continue;
      double diam = diametro(m.tipo);
      if(diam>0 && m.fondo<alto_-2 && m.caja.x>1 && m.caja.x+m.caja.width<ancho_-1) {
        double por_diametro=f_*diam/std::max(1.0,m.ancho);
        double altura=(k_suelo_-por_diametro*(m.fondo-fila_horizonte_))/f_;
        if(altura>0.03)continue; // 2026-11-04: 0.012 descartaba el limón en el piso (focal y horizonte recalibrados); elevada: no recogerla como fruta del piso aunque el cuadrado esté oculto
      }
      double rumbo_baldosa=0;
      if(diam>0 && vision_destinos::sobre_baldosa(bgr,m.u,m.fondo,f_*diam/std::max(1.0,m.ancho),
                  f_,cx_,fila_horizonte_,k_suelo_,rumbo_baldosa))continue;
      double z;
      if (m.fondo >= alto_ - 2) {
        if(diam<=0)continue;
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
      if (diam>0 && (tam < tam_min_ * diam || tam > tam_max_ * diam)) continue;
      s << (primera ? "" : ", ") << "{\"tipo\": \"" << m.tipo << "\", \"x\": " << num(cam_x_ + z) << ", \"y\": "
        << num(y) << ", \"d\": " << num(d) << ", \"px\": " << static_cast<int>(m.ancho) << "}";
      primera = false;
    }
    s << "]}";
    return s.str();
  }

  double diametro(const std::string& tipo) const {
    return tipo=="limon" ? d_limon_ : (tipo=="naranja" ? d_naranja_ : d_mandarina_);
  }

  std::string clasificar(const std::array<double,5>& medidas) const {
    const std::array<double,5> escalas={4,6,50,0.15,0.25};
    std::vector<std::pair<double,std::string>> costos;
    for(auto ref : {std::make_pair(&ref_limon_,"limon"),std::make_pair(&ref_naranja_,"naranja"),std::make_pair(&ref_mandarina_,"mandarina")}){
      double mejor=1e9;
      for(size_t i=0;i+4<ref.first->size();i+=5){double costo=0;
        for(int k=0;k<5;++k)costo+=std::abs(medidas[k]-(*ref.first)[i+k])/escalas[k];
        mejor=std::min(mejor,costo);}
      costos.emplace_back(mejor,ref.second);
    }
    std::sort(costos.begin(),costos.end());
    return costos[0].first<costo_max_ && costos[1].first-costos[0].first>=margen_clase_ ? costos[0].second : "";
  }

  std::string json_cajas(const cv::Mat& bgr) {
    std::ostringstream s; s<<"{\"cajas\":[";bool primera=true;
    std::vector<cv::Point2f> leidas;
    for(auto& m:vision_destinos::marcos(bgr)){
      vision_destinos::PoseCaja p;
      if(!vision_destinos::pose_caja(m,ancho_marco_,f_,cx_,alto_/2.0,cam_x_,p))continue;
      cv::Point2f centro;for(auto e:m.esquinas)centro+=e*0.25f;leidas.push_back(centro);
      s<<(primera?"":",")<<"{\"tipo\":\""<<m.tipo<<"\",\"x\":"<<num(p.x)<<",\"y\":"<<num(p.y)
       <<",\"ancho\":0.22,\"rumbo\":"<<num(p.rumbo)<<"}";primera=false;
    }
    for(auto r:vision_destinos::turquesas(bgr)){
      bool leida=false;
      for(auto p:leidas)if(p.x>=r.x-r.width/10.0 && p.x<=r.x+r.width*1.1 &&
                          p.y>=r.y-r.width*1.5 && p.y<=r.y+r.height*0.7)leida=true;
      if(leida)continue;
      double u=r.x+r.width/2.0;
      double a=std::atan2(cx_-u,f_);
      double mitad=std::abs(std::atan2(cx_-r.x,f_)-std::atan2(cx_-r.x-r.width,f_))/2;
      // Sin escala inventada: la lógica asocia este rayo con el láser del frente.
      s<<(primera?"":",")<<"{\"tipo\":\"\",\"u\":"<<num(u)<<",\"angulo\":"<<num(a)
       <<",\"semiancho_angular\":"<<num(mitad)<<",\"ancho_px\":"<<r.width<<"}";primera=false;
    }
    s<<"]}";return s.str();
  }

  std::string json_baldosas(const cv::Mat& bgr, const std::vector<Mancha>& manchas) {
    std::ostringstream s;s<<"{\"baldosas\":[";bool primera=true;
    for(auto& m:manchas){
      double diam=diametro(m.tipo), rumbo=0;
      if(diam<=0)continue;
      double z=f_*diam/std::max(1.0,m.ancho);
      if(!vision_destinos::sobre_baldosa(bgr,m.u,m.fondo,z,f_,cx_,fila_horizonte_,k_suelo_,rumbo))continue;
      s<<(primera?"":",")<<"{\"tipo\":\""<<m.tipo<<"\",\"x\":"<<num(cam_x_+z)
       <<",\"y\":"<<num(-(m.u-cx_)*z/f_)<<",\"rumbo\":"<<num(rumbo)<<"}";primera=false;
    }
    s<<"]}";return s.str();
  }

  std::string json_garra(const std::vector<Mancha>& manchas) {
    if (manchas.empty()) return "{\"visible\": false, \"x\": 0, \"y\": 0, \"tipo\": \"\", \"area\": 0, \"candidatas\":[]}";
    const auto& m = *std::max_element(manchas.begin(), manchas.end(),
                                      [](const Mancha& a, const Mancha& b) { return a.area < b.area; });
    double x = gx_[0] + gx_[1] * m.u + gx_[2] * m.v;
    double y = gy_[0] + gy_[1] * m.u + gy_[2] * m.v;
    std::ostringstream candidatas;candidatas<<"[";bool primera=true;
    for(const auto& c:manchas){
      candidatas<<(primera?"":",")<<"{\"x\":"<<num(gx_[0]+gx_[1]*c.u+gx_[2]*c.v)
                <<",\"y\":"<<num(gy_[0]+gy_[1]*c.u+gy_[2]*c.v)<<"}";
      primera=false;
    }
    candidatas<<"]";
    return "{\"visible\": true, \"x\": " + num(x) + ", \"y\": " + num(y) + ", \"tipo\": \"" + m.tipo +
           "\", \"area\": " + std::to_string(static_cast<int>(m.area)) + ",\"candidatas\":"+candidatas.str()+"}";
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
    auto cruda=dir_depuracion_ + "/" + camara + "-cruda.jpg";
    cv::imwrite(cruda+".tmp.jpg", bgr);
    std::filesystem::rename(cruda+".tmp.jpg",cruda,e);
  }

  std::string pedida_, disp_frutas_, disp_garra_, formato_frutas_, formato_garra_, dir_depuracion_;
  int ancho_, alto_, fps_, descartar_, h_min_, h_max_, s_min_, v_min_;
  double tam_min_, tam_max_, aspecto_max_, h_corte_, area_min_, circularidad_min_, llenado_min_, f_, cx_, fila_horizonte_, k_suelo_, cam_x_, d_limon_,
      d_naranja_, d_mandarina_, ancho_marco_, costo_max_, margen_clase_;
  bool depuracion_;
  std::vector<double> gx_, gy_;
  std::vector<double> ref_limon_, ref_naranja_, ref_mandarina_;
  std::mutex mutex_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_frutas_, pub_garra_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cajas_, pub_baldosas_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_camara_;
  std::atomic<bool> fin_{false};
  std::thread hilo_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto args=rclcpp::remove_ros_arguments(argc,argv);
  auto evaluacion=std::find(args.begin(),args.end(),"--evaluar");
  if(evaluacion!=args.end() && evaluacion+1!=args.end()) {
    auto nodo=std::make_shared<DetectorFrutas>(false);
    auto modo=std::find(args.begin(),args.end(),"--modo");
    std::string seleccion=modo!=args.end() && modo+1!=args.end() ? *(modo+1) : "frutas";
    if(seleccion!="frutas" && seleccion!="garra" && seleccion!="cajas" && seleccion!="baldosas")return 2;
    std::vector<std::filesystem::path> archivos;
    for(auto& e:std::filesystem::directory_iterator(*(evaluacion+1)))
      if(e.path().extension()==".jpg" || e.path().extension()==".png")archivos.push_back(e.path());
    std::sort(archivos.begin(),archivos.end());
    for(auto& p:archivos){auto img=cv::imread(p.string());if(!img.empty())
      std::cout<<p.filename().string()<<"\t"<<nodo->evaluar(img,seleccion)<<std::endl;}
  } else rclcpp::spin(std::make_shared<DetectorFrutas>());
  rclcpp::shutdown();
  return 0;
}
