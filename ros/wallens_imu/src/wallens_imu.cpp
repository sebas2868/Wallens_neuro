// Driver mínimo del BNO055 por I2C. Reemplaza al paquete bno055 (Python), que leía 45 bytes por ciclo en dos
// transacciones; aquí es un solo bloque de 26 bytes (giroscopio, Euler, cuaternión y aceleración lineal), lo que
// importa porque el bus es por software (i2c-gpio) y cada byte cuesta tiempo de CPU en el kernel.
// Publica (mismos nombres que el driver anterior, para el EKF y los scripts):
//   bno055/imu          sensor_msgs/Imu  orientación (cuaternión), velocidad angular (rad/s), aceleración lineal (m/s²)
//   bno055/calib_status std_msgs/String  JSON {"sys":0-3,"gyro":0-3,"accel":0-3,"mag":0-3}
// No resetea el chip al arrancar (conserva la calibración). Si el chip pierde el modo, lo vuelve a configurar.
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
constexpr uint8_t REG_CHIP_ID = 0x00, REG_PAGE_ID = 0x07, REG_GYR = 0x14, REG_CALIB = 0x35, REG_UNIT = 0x3B,
                  REG_MODE = 0x3D, REG_PWR = 0x3E, REG_TRIGGER = 0x3F, REG_AXIS_CFG = 0x41, REG_AXIS_SIGN = 0x42;
constexpr uint8_t CHIP_ID = 0xA0, MODO_CONFIG = 0x00;
constexpr uint8_t UNIDADES = 0x82;          // aceleración m/s², giroscopio rad/s, Euler en grados
constexpr size_t LARGO_BLOQUE = 26;         // 0x14..0x2D: gyr(6) euler(6) quat(8) lia(6)
constexpr double LSB_CUATERNION = 16384.0, LSB_GIRO = 900.0, LSB_ACEL = 100.0;
constexpr int FALLOS_REINICIO = 30;         // lecturas fallidas seguidas antes de reconfigurar el chip

// Remapeo de ejes (AXIS_MAP_CONFIG, AXIS_MAP_SIGN) según la hoja de datos, secc. 3.4
const std::map<std::string, std::array<uint8_t, 2>> PLACEMENTS = {
    {"P0", {0x21, 0x04}}, {"P1", {0x24, 0x00}}, {"P2", {0x24, 0x06}}, {"P3", {0x21, 0x02}},
    {"P4", {0x24, 0x03}}, {"P5", {0x21, 0x01}}, {"P6", {0x21, 0x07}}, {"P7", {0x24, 0x05}}};

int16_t i16(const uint8_t* p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); }
void dormir_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
}  // namespace

class WallensImu : public rclcpp::Node {
 public:
  WallensImu() : Node("wallens_imu") {
    dispositivo_ = declare_parameter("i2c_device", "/dev/i2c-1");
    direccion_ = static_cast<uint8_t>(declare_parameter("i2c_address", 0x28));
    const double rate = declare_parameter("rate_hz", 30.0);
    frame_ = declare_parameter("frame_id", "imu_link");
    modo_ = static_cast<uint8_t>(declare_parameter("operation_mode", 0x08));
    placement_ = declare_parameter("placement_axis_remap", "P6");
    const double calib_rate = declare_parameter("calib_status_rate_hz", 1.0);
    if (!PLACEMENTS.count(placement_)) throw std::runtime_error("placement_axis_remap desconocido: " + placement_);

    fd_ = ::open(dispositivo_.c_str(), O_RDWR);
    if (fd_ < 0) throw std::runtime_error("No pude abrir " + dispositivo_ + ": " + std::strerror(errno));
    configurar();

    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("bno055/imu", 10);
    pub_calib_ = create_publisher<std_msgs::msg::String>("bno055/calib_status", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { ciclo(); });
    timer_calib_ = create_wall_timer(std::chrono::duration<double>(1.0 / calib_rate), [this] { calibracion(); });
    RCLCPP_INFO(get_logger(), "BNO055 en %s (0x%02x), modo 0x%02x, placement %s, %.0f Hz", dispositivo_.c_str(),
                direccion_, modo_, placement_.c_str(), rate);
  }

  ~WallensImu() override {
    if (fd_ >= 0) ::close(fd_);
  }

 private:
  // Escribe el registro y lee n bytes en una sola transacción (repeated start)
  bool leer(uint8_t reg, uint8_t* destino, uint16_t n) {
    i2c_msg msgs[2] = {{direccion_, 0, 1, &reg}, {direccion_, I2C_M_RD, n, destino}};
    i2c_rdwr_ioctl_data datos{msgs, 2};
    return ::ioctl(fd_, I2C_RDWR, &datos) >= 0;
  }

  bool escribir(uint8_t reg, uint8_t valor) {
    uint8_t buf[2] = {reg, valor};
    i2c_msg msg = {direccion_, 0, 2, buf};
    i2c_rdwr_ioctl_data datos{&msg, 1};
    return ::ioctl(fd_, I2C_RDWR, &datos) >= 0;
  }

  void escribir_o_fallar(uint8_t reg, uint8_t valor) {
    if (!escribir(reg, valor)) throw std::runtime_error("Fallo escribiendo el registro 0x" + std::to_string(reg));
  }

  // Misma secuencia que el driver bno055, sin resetear el chip (así conserva la calibración)
  void configurar() {
    uint8_t id = 0;
    for (int i = 0; i < 20 && !(leer(REG_CHIP_ID, &id, 1) && id == CHIP_ID); ++i) dormir_ms(100);
    if (id != CHIP_ID) throw std::runtime_error("El BNO055 no responde (chip id distinto de 0xA0)");
    escribir_o_fallar(REG_MODE, MODO_CONFIG);
    dormir_ms(30);
    escribir_o_fallar(REG_PWR, 0x00);
    escribir_o_fallar(REG_PAGE_ID, 0x00);
    escribir_o_fallar(REG_TRIGGER, 0x00);
    escribir_o_fallar(REG_UNIT, UNIDADES);
    const auto& p = PLACEMENTS.at(placement_);
    escribir_o_fallar(REG_AXIS_CFG, p[0]);
    escribir_o_fallar(REG_AXIS_SIGN, p[1]);
    escribir_o_fallar(REG_MODE, modo_);
    dormir_ms(30);
  }

  void ciclo() {
    uint8_t b[LARGO_BLOQUE];
    if (!leer(REG_GYR, b, LARGO_BLOQUE)) return registrar_fallo("error de I2C");
    // Un bloque corrupto (todo 0xFF o ceros) o con el cuaternión sin norma 1 se descarta
    const double w = i16(b + 12) / LSB_CUATERNION, x = i16(b + 14) / LSB_CUATERNION, y = i16(b + 16) / LSB_CUATERNION,
                 z = i16(b + 18) / LSB_CUATERNION;
    const double norma = std::sqrt(w * w + x * x + y * y + z * z);
    if (norma < 0.9 || norma > 1.1) return registrar_fallo("cuaternión con norma " + std::to_string(norma));
    fallos_ = 0;

    sensor_msgs::msg::Imu m;
    m.header.stamp = now();
    m.header.frame_id = frame_;
    m.orientation.w = w / norma;
    m.orientation.x = x / norma;
    m.orientation.y = y / norma;
    m.orientation.z = z / norma;
    m.angular_velocity.x = i16(b + 0) / LSB_GIRO;
    m.angular_velocity.y = i16(b + 2) / LSB_GIRO;
    m.angular_velocity.z = i16(b + 4) / LSB_GIRO;
    m.linear_acceleration.x = i16(b + 20) / LSB_ACEL;
    m.linear_acceleration.y = i16(b + 22) / LSB_ACEL;
    m.linear_acceleration.z = i16(b + 24) / LSB_ACEL;
    m.orientation_covariance = {0.0159, 0, 0, 0, 0.0159, 0, 0, 0, 0.0159};
    m.angular_velocity_covariance = {0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04};
    m.linear_acceleration_covariance = {0.017, 0, 0, 0, 0.017, 0, 0, 0, 0.017};
    pub_imu_->publish(m);
  }

  void calibracion() {
    uint8_t c = 0, modo = 0;
    if (!leer(REG_CALIB, &c, 1) || !leer(REG_MODE, &modo, 1)) return registrar_fallo("error de I2C leyendo la calibración");
    if ((modo & 0x0F) != (modo_ & 0x0F)) {      // el chip se reinició o salió del modo
      RCLCPP_WARN(get_logger(), "El BNO055 está en modo 0x%02x en vez de 0x%02x: reconfigurando", modo, modo_);
      reconfigurar();
      return;
    }
    std_msgs::msg::String s;
    s.data = "{\"sys\": " + std::to_string((c >> 6) & 3) + ", \"gyro\": " + std::to_string((c >> 4) & 3) +
             ", \"accel\": " + std::to_string((c >> 2) & 3) + ", \"mag\": " + std::to_string(c & 3) + "}";
    pub_calib_->publish(s);
  }

  void registrar_fallo(const std::string& motivo) {
    ++fallos_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Lectura descartada (%s); %d seguidas", motivo.c_str(), fallos_);
    if (fallos_ % FALLOS_REINICIO == 0) reconfigurar();
  }

  void reconfigurar() {
    try {
      configurar();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "No pude reconfigurar el BNO055: %s", e.what());
    }
  }

  std::string dispositivo_, frame_, placement_;
  uint8_t direccion_, modo_;
  int fd_ = -1, fallos_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_calib_;
  rclcpp::TimerBase::SharedPtr timer_, timer_calib_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WallensImu>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("wallens_imu"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
