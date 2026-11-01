// Driver de las ruedas de Wallens: dueño único del bus Dynamixel (XM430-W210, protocolo 2.0).
// Escucha /cmd_vel, mueve las ruedas en modo velocidad y publica /odom y el TF odom -> base_link.
// Si pierde /cmd_vel o la comunicación frena, y al salir apaga el torque (verificándolo).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {

// Tabla de control del XM430-W210
constexpr uint16_t MODELO_XM430_W210 = 1030;
constexpr uint16_t DIR_MODO = 11;
constexpr uint16_t DIR_TORQUE = 64;
constexpr uint16_t DIR_ERROR_HW = 70;
constexpr uint16_t DIR_WATCHDOG = 98;
constexpr uint16_t DIR_VELOCIDAD_META = 104;
constexpr uint16_t DIR_ACELERACION = 108;
constexpr uint16_t DIR_LECTURA = 126;    // corriente (2), velocidad (4) y posición (4) seguidas
constexpr uint16_t LARGO_LECTURA = 10;
constexpr uint16_t DIR_POSICION = 132;
constexpr uint16_t DIR_VOLTAJE = 144;
constexpr uint16_t DIR_TEMPERATURA = 146;
constexpr uint8_t MODO_VELOCIDAD = 1;
constexpr uint8_t ALERTA_HW = 0x80;      // bit de error de hardware en el paquete de estado
constexpr double RPM_POR_UNIDAD = 0.229;
constexpr double TICKS_VUELTA = 4096.0;
constexpr double DOS_PI = 2.0 * M_PI;

// Fallo que no se reintenta: se frena y se apaga el nodo.
struct Fatal : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct Rueda {
  uint8_t id;
  int signo;
  double k;                // corrección de velocidad
  int32_t pos_previa = 0;
  int32_t pos = 0;
};

}  // namespace

class WallensDriver : public rclcpp::Node {
 public:
  WallensDriver() : Node("wallens_driver") {
    puerto_ = declare_parameter("port", "/dev/dynamixel");
    baudrate_ = declare_parameter("baudrate", 57600);
    loop_hz_ = declare_parameter("loop_hz", 30.0);
    izq_ = {static_cast<uint8_t>(declare_parameter("id_left", 2)),
            static_cast<int>(declare_parameter("sign_left", 1)), declare_parameter("k_left", 1.0)};
    der_ = {static_cast<uint8_t>(declare_parameter("id_right", 4)),
            static_cast<int>(declare_parameter("sign_right", -1)), 1.0};
    radio_ = declare_parameter("wheel_radius", 0.03625);
    separacion_ = declare_parameter("wheel_separation", 0.238);
    max_lineal_ = declare_parameter("max_linear", 0.15);
    max_angular_ = declare_parameter("max_angular", 1.0);
    max_rpm_ = declare_parameter("max_wheel_rpm", 60.0);
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.25);
    aceleracion_ = declare_parameter("profile_acceleration", 20);
    watchdog_ = declare_parameter("bus_watchdog", 10);
    voltaje_min_ = declare_parameter("voltage_min", 10.0);
    voltaje_max_ = declare_parameter("voltage_max", 14.8);
    temp_max_ = declare_parameter("temp_max", 55);
    max_fallos_ = declare_parameter("max_failures", 5);
    marco_odom_ = declare_parameter("odom_frame", "odom");
    marco_base_ = declare_parameter("base_frame", "base_link");
    publicar_tf_ = declare_parameter("publish_tf", true);

    if (radio_ <= 0 || separacion_ <= 0 || loop_hz_ <= 0 || max_rpm_ <= 0) {
      throw std::invalid_argument("wheel_radius, wheel_separation, loop_hz y max_wheel_rpm deben ser > 0");
    }

    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          cmd_v_ = m->linear.x;
          cmd_w_ = m->angular.z;
          ultimo_cmd_ = std::chrono::steady_clock::now();
          hay_cmd_ = true;
        });
    // La alarma de wallens_power llega aunque el driver arranque después (transient_local)
    sub_bateria_ = create_subscription<std_msgs::msg::Bool>(
        "battery_critical", rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data != bateria_critica_) {
            RCLCPP_WARN(get_logger(), m->data ? "Batería crítica: las ruedas se frenan" : "Batería recuperada");
          }
          bateria_critica_ = m->data;
        });
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    if (publicar_tf_) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // Abre el puerto y deja los motores listos. Se llama aparte del constructor para que, si algo
  // falla a medias, main pueda apagar el torque que ya se hubiera encendido.
  void iniciar() {
    port_ = dynamixel::PortHandler::getPortHandler(puerto_.c_str());
    ph_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_->openPort() || !port_->setBaudRate(baudrate_)) {
      throw Fatal("No pude abrir " + puerto_ + " a " + std::to_string(baudrate_));
    }
    for (Rueda* r : {&izq_, &der_}) {
      uint16_t modelo = 0;
      uint8_t err = 0;
      verificar(ph_->ping(port_, r->id, &modelo, &err), err, r->id, "ping");
      if (modelo != MODELO_XM430_W210) {
        throw Fatal("El ID " + std::to_string(r->id) + " no es un XM430-W210 (modelo " + std::to_string(modelo) + ")");
      }
      escribir1(r->id, DIR_TORQUE, 0);           // torque off para cambiar el modo
      escribir1(r->id, DIR_WATCHDOG, 0);         // limpia un watchdog disparado antes: con él activo (255) el motor
                                                 // rechaza las demás escrituras (tras reiniciar la Pi pasa siempre)
      escribir1(r->id, DIR_MODO, MODO_VELOCIDAD);
      escribir4(r->id, DIR_ACELERACION, static_cast<uint32_t>(aceleracion_));
      escribir4(r->id, DIR_VELOCIDAD_META, 0);
      motores_activos_ = true;
      escribir1(r->id, DIR_TORQUE, 1);
      escribir1(r->id, DIR_WATCHDOG, static_cast<uint8_t>(watchdog_));
    }
    sync_escritura_ = std::make_unique<dynamixel::GroupSyncWrite>(port_, ph_, DIR_VELOCIDAD_META, 4);
    sync_lectura_ = std::make_unique<dynamixel::GroupSyncRead>(port_, ph_, DIR_LECTURA, LARGO_LECTURA);
    for (Rueda* r : {&izq_, &der_}) sync_lectura_->addParam(r->id);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / loop_hz_), [this] { ciclo(); });
    RCLCPP_INFO(get_logger(), "Listo: ruedas IDs %d (izq) y %d (der), %.0f Hz, r=%.4f m, L=%.3f m",
                izq_.id, der_.id, loop_hz_, radio_, separacion_);
  }

  // Frena, apaga el torque y comprueba con una lectura que quedó apagado. Se puede llamar varias veces.
  void apagar() {
    if (!motores_activos_) return;
    motores_activos_ = false;
    timer_.reset();
    for (Rueda* r : {&izq_, &der_}) intentar([&] { escribir4(r->id, DIR_VELOCIDAD_META, 0); });
    for (int i = 0; i < 8; ++i) {                // deja que termine la rampa sin soltar el watchdog
      for (Rueda* r : {&izq_, &der_}) intentar([&] { leer1(r->id, DIR_ERROR_HW); });
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    bool todo_apagado = true;
    for (Rueda* r : {&izq_, &der_}) {
      intentar([&] { escribir1(r->id, DIR_WATCHDOG, 0); });  // un watchdog disparado rechaza otras escrituras
      bool apagado = false;
      intentar([&] {
        escribir1(r->id, DIR_TORQUE, 0);
        apagado = leer1(r->id, DIR_TORQUE) == 0;
      });
      todo_apagado = todo_apagado && apagado;
      if (!apagado) RCLCPP_ERROR(get_logger(), "No pude apagar el motor %d: desconecta la alimentación", r->id);
    }
    if (todo_apagado) RCLCPP_INFO(get_logger(), "Torque apagado");
    if (port_) port_->closePort();
  }

 private:
  void ciclo() {
    const auto ahora = std::chrono::steady_clock::now();
    try {
      mandar_velocidades(ahora);
      leer_y_publicar(ahora);
      if (++ciclos_ % std::max(1, static_cast<int>(loop_hz_)) == 0) revisar_salud();
      fallos_ = 0;
    } catch (const Fatal& e) {
      RCLCPP_FATAL(get_logger(), "%s: se apaga el driver", e.what());
      apagar();
      rclcpp::shutdown();
    } catch (const std::exception& e) {
      if (++fallos_ >= max_fallos_) {
        RCLCPP_FATAL(get_logger(), "%d fallos seguidos (%s): se apaga el driver", fallos_, e.what());
        apagar();
        rclcpp::shutdown();
      } else {
        RCLCPP_WARN(get_logger(), "Fallo de comunicación %d/%d: %s", fallos_, max_fallos_, e.what());
      }
    }
  }

  void mandar_velocidades(std::chrono::steady_clock::time_point ahora) {
    double v = 0.0, w = 0.0;
    if (!bateria_critica_ && hay_cmd_ && std::chrono::duration<double>(ahora - ultimo_cmd_).count() <= cmd_timeout_) {
      v = std::clamp(cmd_v_, -max_lineal_, max_lineal_);
      w = std::clamp(cmd_w_, -max_angular_, max_angular_);
    }
    // m/s de cada rueda -> rpm
    double rpm_izq = (v - w * separacion_ / 2.0) / radio_ * 60.0 / DOS_PI;
    double rpm_der = (v + w * separacion_ / 2.0) / radio_ * 60.0 / DOS_PI;
    const double pico = std::max(std::abs(rpm_izq), std::abs(rpm_der));
    if (pico > max_rpm_) {                       // se escalan las dos para conservar la curva
      rpm_izq *= max_rpm_ / pico;
      rpm_der *= max_rpm_ / pico;
    }
    sync_escritura_->clearParam();
    for (auto [r, rpm] : {std::pair{&izq_, rpm_izq}, std::pair{&der_, rpm_der}}) {
      const auto unidades = static_cast<int32_t>(std::lround(r->signo * rpm * r->k / RPM_POR_UNIDAD));
      const auto u = static_cast<uint32_t>(unidades);
      uint8_t datos[4] = {static_cast<uint8_t>(u), static_cast<uint8_t>(u >> 8),
                          static_cast<uint8_t>(u >> 16), static_cast<uint8_t>(u >> 24)};
      if (!sync_escritura_->addParam(r->id, datos)) throw Fatal("No pude armar el paquete de velocidades");
    }
    const int res = sync_escritura_->txPacket();
    if (res != COMM_SUCCESS) throw std::runtime_error(ph_->getTxRxResult(res));
  }

  void leer_y_publicar(std::chrono::steady_clock::time_point ahora) {
    const int res = sync_lectura_->txRxPacket();
    if (res != COMM_SUCCESS) throw std::runtime_error(ph_->getTxRxResult(res));
    for (Rueda* r : {&izq_, &der_}) {
      uint8_t err = 0;
      if (sync_lectura_->getError(r->id, &err) && (err & ALERTA_HW)) {
        char hex[8];
        std::snprintf(hex, sizeof hex, "0x%02x", leer1(r->id, DIR_ERROR_HW));
        throw Fatal(std::string("Error de hardware ") + hex + " en el motor " + std::to_string(r->id) +
                    " (1=voltaje, 4=sobrecalent., 32=sobrecarga)");
      }
      if (!sync_lectura_->isAvailable(r->id, DIR_POSICION, 4)) {
        throw std::runtime_error("El motor " + std::to_string(r->id) + " no entregó la posición");
      }
      r->pos = static_cast<int32_t>(sync_lectura_->getData(r->id, DIR_POSICION, 4));
    }
    if (!odom_iniciada_) {                       // primera lectura: solo fija el punto de partida
      izq_.pos_previa = izq_.pos;
      der_.pos_previa = der_.pos;
      t_previo_ = ahora;
      odom_iniciada_ = true;
      return;
    }
    const double dt = std::chrono::duration<double>(ahora - t_previo_).count();
    t_previo_ = ahora;
    // metros que avanzó cada rueda (el signo la lleva a "hacia adelante")
    const double d_izq = izq_.signo * ticks_entre(izq_.pos_previa, izq_.pos) / TICKS_VUELTA * DOS_PI * radio_;
    const double d_der = der_.signo * ticks_entre(der_.pos_previa, der_.pos) / TICKS_VUELTA * DOS_PI * radio_;
    izq_.pos_previa = izq_.pos;
    der_.pos_previa = der_.pos;

    const double ds = (d_izq + d_der) / 2.0;
    const double dth = (d_der - d_izq) / separacion_;
    x_ += ds * std::cos(th_ + dth / 2.0);
    y_ += ds * std::sin(th_ + dth / 2.0);
    th_ += dth;
    publicar(dt > 0 ? ds / dt : 0.0, dt > 0 ? dth / dt : 0.0);
  }

  // Diferencia con signo entre dos posiciones; la resta sin signo evita el desborde de int32.
  static int32_t ticks_entre(int32_t antes, int32_t despues) {
    return static_cast<int32_t>(static_cast<uint32_t>(despues) - static_cast<uint32_t>(antes));
  }

  void publicar(double v, double w) {
    const auto stamp = now();
    const double qz = std::sin(th_ / 2.0), qw = std::cos(th_ / 2.0);
    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = marco_odom_;
    o.child_frame_id = marco_base_;
    o.pose.pose.position.x = x_;
    o.pose.pose.position.y = y_;
    o.pose.pose.orientation.z = qz;
    o.pose.pose.orientation.w = qw;
    o.twist.twist.linear.x = v;
    o.twist.twist.angular.z = w;
    // covarianzas provisionales hasta validar la odometría (Fase 4)
    o.pose.covariance[0] = o.pose.covariance[7] = 1e-3;
    o.pose.covariance[14] = o.pose.covariance[21] = o.pose.covariance[28] = 1e6;
    o.pose.covariance[35] = 1e-2;
    o.twist.covariance[0] = 1e-3;
    o.twist.covariance[7] = o.twist.covariance[14] = o.twist.covariance[21] = o.twist.covariance[28] = 1e6;
    o.twist.covariance[35] = 1e-2;
    pub_odom_->publish(o);

    if (tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header = o.header;
      t.child_frame_id = marco_base_;
      t.transform.translation.x = x_;
      t.transform.translation.y = y_;
      t.transform.rotation = o.pose.pose.orientation;
      tf_->sendTransform(t);
    }
  }

  void revisar_salud() {
    for (Rueda* r : {&izq_, &der_}) {
      const double voltaje = leer2(r->id, DIR_VOLTAJE) / 10.0;
      const int temp = leer1(r->id, DIR_TEMPERATURA);
      if (voltaje < voltaje_min_ || voltaje > voltaje_max_) {
        throw Fatal("Motor " + std::to_string(r->id) + ": voltaje " + std::to_string(voltaje) + " V fuera de rango");
      }
      if (temp > temp_max_) {
        throw Fatal("Motor " + std::to_string(r->id) + ": temperatura " + std::to_string(temp) + " °C");
      }
    }
  }

  void verificar(int res, uint8_t err, uint8_t id, const std::string& que) {
    if (res != COMM_SUCCESS) throw std::runtime_error("motor " + std::to_string(id) + " " + que + ": " + ph_->getTxRxResult(res));
    if (err != 0) throw std::runtime_error("motor " + std::to_string(id) + " " + que + ": " + ph_->getRxPacketError(err));
  }
  void escribir1(uint8_t id, uint16_t dir, uint8_t v) {
    uint8_t err = 0;
    verificar(ph_->write1ByteTxRx(port_, id, dir, v, &err), err, id, "escribir " + std::to_string(dir));
  }
  void escribir4(uint8_t id, uint16_t dir, uint32_t v) {
    uint8_t err = 0;
    verificar(ph_->write4ByteTxRx(port_, id, dir, v, &err), err, id, "escribir " + std::to_string(dir));
  }
  uint8_t leer1(uint8_t id, uint16_t dir) {
    uint8_t v = 0, err = 0;
    verificar(ph_->read1ByteTxRx(port_, id, dir, &v, &err), err, id, "leer " + std::to_string(dir));
    return v;
  }
  uint16_t leer2(uint8_t id, uint16_t dir) {
    uint16_t v = 0;
    uint8_t err = 0;
    verificar(ph_->read2ByteTxRx(port_, id, dir, &v, &err), err, id, "leer " + std::to_string(dir));
    return v;
  }
  template <typename F>
  void intentar(F&& f) {                          // en el apagado cada paso se intenta aunque falle otro
    try {
      f();
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Apagado: %s", e.what());
    }
  }

  // parámetros
  std::string puerto_, marco_odom_, marco_base_;
  int baudrate_, aceleracion_, watchdog_, temp_max_, max_fallos_;
  double loop_hz_, radio_, separacion_, max_lineal_, max_angular_, max_rpm_, cmd_timeout_, voltaje_min_, voltaje_max_;
  bool publicar_tf_;

  // hardware
  dynamixel::PortHandler* port_ = nullptr;
  dynamixel::PacketHandler* ph_ = nullptr;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_escritura_;
  std::unique_ptr<dynamixel::GroupSyncRead> sync_lectura_;
  Rueda izq_, der_;
  bool motores_activos_ = false;

  // estado
  double cmd_v_ = 0.0, cmd_w_ = 0.0;
  bool hay_cmd_ = false;
  bool bateria_critica_ = false;
  std::chrono::steady_clock::time_point ultimo_cmd_, t_previo_;
  bool odom_iniciada_ = false;
  double x_ = 0.0, y_ = 0.0, th_ = 0.0;
  int fallos_ = 0;
  long ciclos_ = 0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bateria_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<WallensDriver> nodo;
  int codigo = 0;
  try {
    nodo = std::make_shared<WallensDriver>();
    nodo->iniciar();
    rclcpp::spin(nodo);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("wallens_driver"), "%s", e.what());
    codigo = 1;
  }
  if (nodo) nodo->apagar();                      // Ctrl+C, fallo al iniciar o fallo en marcha
  rclcpp::shutdown();
  return codigo;
}
