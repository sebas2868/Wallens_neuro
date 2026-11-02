// Lee la batería de Wallens con un INA226 (I2C) y publica sensor_msgs/BatteryState a rate_hz.
// battery_critical (std_msgs/Bool, transient_local) se activa cuando el voltaje se mantiene bajo
// el corte; el driver de las ruedas lo escucha y frena.
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace {
// Registros del INA226 (16 bits, el byte alto primero)
constexpr uint8_t REG_CONFIG = 0x00;
constexpr uint8_t REG_SHUNT = 0x01;
constexpr uint8_t REG_BUS = 0x02;
constexpr uint8_t REG_CORRIENTE = 0x04;
constexpr uint8_t REG_CALIBRACION = 0x05;
constexpr uint8_t REG_FABRICANTE = 0xFE;
constexpr uint8_t REG_DIE = 0xFF;
constexpr uint16_t ID_FABRICANTE = 0x5449;  // "TI"
constexpr uint16_t ID_DIE = 0x2260;         // los 4 bits bajos son la revisión
// Promedio de 16 muestras, 1.1 ms por conversión, shunt y bus continuos
constexpr uint16_t CONFIG_PROMEDIADO = 0x4527;
constexpr double LSB_BUS = 1.25e-3;         // V por paso
}  // namespace

class Ina226 {
 public:
  Ina226(int bus, int direccion) {
    const std::string ruta = "/dev/i2c-" + std::to_string(bus);
    fd_ = open(ruta.c_str(), O_RDWR);
    if (fd_ < 0) throw std::runtime_error("No pude abrir " + ruta);
    direccion_ = direccion;
  }
  ~Ina226() {
    if (fd_ >= 0) close(fd_);
  }
  Ina226(const Ina226&) = delete;
  Ina226& operator=(const Ina226&) = delete;

  uint16_t leer(uint8_t reg) {
    uint8_t dato[2] = {0, 0};
    i2c_msg msgs[2] = {{static_cast<uint16_t>(direccion_), 0, 1, &reg},
                       {static_cast<uint16_t>(direccion_), I2C_M_RD, 2, dato}};
    i2c_rdwr_ioctl_data tx{msgs, 2};
    if (ioctl(fd_, I2C_RDWR, &tx) < 0) throw std::runtime_error("Fallo leyendo el registro " + std::to_string(reg));
    return static_cast<uint16_t>((dato[0] << 8) | dato[1]);
  }
  void escribir(uint8_t reg, uint16_t valor) {
    uint8_t dato[3] = {reg, static_cast<uint8_t>(valor >> 8), static_cast<uint8_t>(valor)};
    i2c_msg msg{static_cast<uint16_t>(direccion_), 0, 3, dato};
    i2c_rdwr_ioctl_data tx{&msg, 1};
    if (ioctl(fd_, I2C_RDWR, &tx) < 0) throw std::runtime_error("Fallo escribiendo el registro " + std::to_string(reg));
  }

 private:
  int fd_ = -1;
  int direccion_ = 0;
};

namespace wallens_power {

class WallensPower : public rclcpp::Node {
 public:
  explicit WallensPower(const rclcpp::NodeOptions& options) : Node("wallens_power", options) {
    const int bus = declare_parameter("i2c_bus", 1);
    const int direccion = declare_parameter("i2c_address", 0x40);
    const double shunt = declare_parameter("shunt_ohms", 0.002);
    lsb_corriente_ = declare_parameter("current_lsb", 0.0005);
    const double rate = declare_parameter("rate_hz", 2.0);
    celdas_ = declare_parameter("cells", 4);
    aviso_ = declare_parameter("warn_cell_voltage", 3.6) * celdas_;
    corte_ = declare_parameter("cut_cell_voltage", 3.4) * celdas_;
    muestras_corte_ = declare_parameter("cut_samples", 5);
    margen_ = declare_parameter("resume_margin", 0.3);
    if (shunt <= 0 || lsb_corriente_ <= 0 || rate <= 0 || celdas_ <= 0) {
      throw std::invalid_argument("shunt_ohms, current_lsb, rate_hz y cells deben ser > 0");
    }

    ina_ = std::make_unique<Ina226>(bus, direccion);
    if (ina_->leer(REG_FABRICANTE) != ID_FABRICANTE || (ina_->leer(REG_DIE) & 0xFFF0) != ID_DIE) {
      throw std::runtime_error("El dispositivo en 0x" + std::to_string(direccion) + " no es un INA226");
    }
    const double cal = std::trunc(0.00512 / (lsb_corriente_ * shunt));
    if (cal < 1 || cal > 0xFFFF) throw std::invalid_argument("current_lsb y shunt_ohms dan un CAL fuera de rango");
    ina_->escribir(REG_CONFIG, CONFIG_PROMEDIADO);
    ina_->escribir(REG_CALIBRACION, static_cast<uint16_t>(cal));

    pub_bateria_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    pub_critica_ = create_publisher<std_msgs::msg::Bool>("battery_critical", rclcpp::QoS(1).transient_local());
    publicar_critica(false);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { ciclo(); });
    RCLCPP_INFO(get_logger(), "INA226 listo: CAL=%.0f, aviso %.1f V, corte %.1f V (%d celdas)", cal, aviso_, corte_, celdas_);
  }

 private:
  void ciclo() {
    double voltaje, corriente;
    try {
      voltaje = ina_->leer(REG_BUS) * LSB_BUS;
      corriente = static_cast<int16_t>(ina_->leer(REG_CORRIENTE)) * lsb_corriente_;
      fallos_ = 0;
    } catch (const std::exception& e) {
      // sin datos no se puede afirmar que la batería esté bien: a los 5 fallos seguidos se declara crítica
      if (++fallos_ == 5) {
        RCLCPP_ERROR(get_logger(), "Sin lectura del INA226 (%s): se marca batería crítica", e.what());
        publicar_critica(true);
      } else {
        RCLCPP_WARN(get_logger(), "Fallo leyendo el INA226: %s", e.what());
      }
      return;
    }

    sensor_msgs::msg::BatteryState m;
    m.header.stamp = now();
    m.voltage = static_cast<float>(voltaje);
    m.current = static_cast<float>(-corriente);          // convención ROS: negativa al descargar
    m.charge = m.capacity = m.design_capacity = m.percentage = std::numeric_limits<float>::quiet_NaN();
    m.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    m.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    m.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    m.present = true;
    pub_bateria_->publish(m);

    if (voltaje < aviso_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Batería baja: %.2f V (aviso %.1f V)", voltaje, aviso_);
    }
    bajas_ = voltaje < corte_ ? bajas_ + 1 : 0;
    if (!critica_ && bajas_ >= muestras_corte_) {
      RCLCPP_ERROR(get_logger(), "Batería crítica: %.2f V bajo %.1f V; las ruedas se frenan", voltaje, corte_);
      publicar_critica(true);
    } else if (critica_ && voltaje > corte_ + margen_ && fallos_ == 0) {
      RCLCPP_INFO(get_logger(), "Batería recuperada: %.2f V", voltaje);
      publicar_critica(false);
    }
  }

  void publicar_critica(bool valor) {
    critica_ = valor;
    std_msgs::msg::Bool m;
    m.data = valor;
    pub_critica_->publish(m);
  }

  std::unique_ptr<Ina226> ina_;
  double lsb_corriente_, aviso_, corte_, margen_;
  int celdas_, muestras_corte_;
  int bajas_ = 0, fallos_ = 0;
  bool critica_ = false;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_bateria_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_critica_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wallens_power

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wallens_power::WallensPower)
