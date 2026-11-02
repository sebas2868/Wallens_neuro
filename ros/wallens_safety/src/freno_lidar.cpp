// Freno por obstáculo en C++ (mismo comportamiento que scripts/freno_lidar.py + vigilante_lidar.py, con la lógica de
// include/wallens_safety/freno_logica.hpp). Lee /cmd_vel, mira /scan y publica /cmd_vel_safe, que es lo que escucha el
// driver. Además hace de vigilante: si /scan falta scan_lost_s segundos reinicia el nodo del LiDAR (SIGINT con pkill;
// el respawn del launch lo vuelve a lanzar), esperando cooldown_s entre reinicios.
// Interbloqueo del brazo: mientras /brazo/ocupado sea true (el brazo se mueve) publica velocidad 0.
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "wallens_safety/freno_logica.hpp"

extern char** environ;

using namespace wallens_safety;

namespace {

double ahora() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ruta completa al inicio de la línea de comandos: no coincide con este mismo proceso
const char* PATRON = "^/.*/ldlidar_stl_ros2_node( |$)";

// Ejecuta `pkill -INT -f PATRON` sin shell y espera a que termine (como subprocess.run). Devuelve su código de
// salida (0 = mató algo), 127 si no se pudo lanzar, -1 si salió por señal.
int pkill_lidar() {
  char arg0[] = "pkill", arg1[] = "-INT", arg2[] = "-f";
  std::string patron = PATRON;
  char* argv[] = {arg0, arg1, arg2, patron.data(), nullptr};
  pid_t pid;
  if (posix_spawnp(&pid, "pkill", nullptr, nullptr, argv, environ) != 0) return 127;
  int estado = 0;
  while (waitpid(pid, &estado, 0) < 0) {
    if (errno != EINTR) return -1;
  }
  return WIFEXITED(estado) ? WEXITSTATUS(estado) : -1;
}

}  // namespace

class FrenoLidar : public rclcpp::Node {
 public:
  FrenoLidar() : Node("freno_lidar") {
    auto dbl = [this](const char* n, double d) { return declare_parameter<double>(n, d); };
    Poligono huella = parsear_huella(declare_parameter<std::string>("footprint", ""));
    double laser_x = dbl("laser_x", 0.089);
    double margen_lateral = dbl("lateral_margin", 0.03);
    double margen_giro = dbl("turn_margin", 0.03);
    double horizonte_giro = dbl("turn_horizon", 0.5);
    double retencion_giro = dbl("turn_hold", 1.0);
    double dist = dbl("stop_distance", 0.30);
    double semiancho = dbl("half_angle_deg", 40.0) * (PI / 180.0);   // math.radians
    double yaw = dbl("laser_yaw_deg", -90.0) * (PI / 180.0);
    double minimo = dbl("min_range", 0.06);
    double scan_timeout = dbl("scan_timeout", 0.3);
    bool requiere_scan = declare_parameter<bool>("require_scan", true);
    double cmd_timeout = dbl("cmd_timeout", 0.25);
    double histeresis = dbl("hysteresis", 0.05);
    int min_lecturas = static_cast<int>(declare_parameter<int64_t>("min_valid_readings", 10));
    freno_ = std::make_unique<Freno>(dist, semiancho, yaw, minimo, scan_timeout, requiere_scan, cmd_timeout, histeresis,
                                     min_lecturas, huella, laser_x, margen_lateral, margen_giro, horizonte_giro,
                                     retencion_giro);
    vigilante_ = std::make_unique<Vigilante>(dbl("scan_lost_s", 5.0), dbl("cooldown_s", 15.0), ahora());

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { on_scan(*m); });
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10), [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { on_cmd(*m); });
    // transient_local: llega aunque este nodo arranque después. Si wallens_brazo muere con true, las ruedas siguen paradas.
    sub_brazo_ = create_subscription<std_msgs::msg::Bool>(
        "brazo/ocupado", rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data != brazo_ocupado_) {
            RCLCPP_WARN(get_logger(), m->data ? "Brazo ocupado: las ruedas se frenan" : "Brazo libre: las ruedas obedecen");
          }
          brazo_ocupado_ = m->data;
        });
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", rclcpp::QoS(10));
    pub_freno_ = create_publisher<std_msgs::msg::Bool>("freno_obstaculo", rclcpp::QoS(1));
    timer_ = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / 30)), [this] { ciclo(); });
    timer_vig_ = create_wall_timer(std::chrono::seconds(1), [this] { revisar(); });
    if (!huella.empty()) {
      RCLCPP_INFO(get_logger(), "Freno por huella de %zu puntos; frena también el giro", huella.size());
    } else {
      RCLCPP_WARN(get_logger(), "Sin huella (footprint): cono desde el LiDAR y el giro no se frena");
    }
  }

 private:
  void on_scan(const sensor_msgs::msg::LaserScan& m) {
    double t = ahora();
    freno_->on_scan(m.ranges, m.angle_min, m.angle_increment, t);
    vigilante_->on_scan(t);
  }

  void on_cmd(const geometry_msgs::msg::Twist& m) {
    if (!freno_->on_cmd(m.linear.x, m.angular.z, ahora())) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cmd_vel no finito: se descarta");
    }
  }

  void ciclo() {
    auto r = freno_->ciclo(ahora());
    if (!r) {
      set_freno(false);
      return;   // sin órdenes recientes: el driver frena solo
    }
    if (r->bloqueado) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Avance anulado: %s", r->motivo.c_str());
    }
    set_freno(r->bloqueado);
    geometry_msgs::msg::Twist out;
    if (!brazo_ocupado_) {
      out.linear.x = r->v;
      out.angular.z = r->w;
    }
    pub_->publish(out);
  }

  void set_freno(bool estado) {
    if (estado != estado_publicado_) {
      estado_publicado_ = estado;
      std_msgs::msg::Bool b;
      b.data = estado;
      pub_freno_->publish(b);
    }
  }

  void revisar() {
    if (vigilante_->debe_reiniciar(ahora())) {
      int rc = pkill_lidar();
      if (rc == 0) {
        RCLCPP_ERROR(get_logger(), "Sin /scan: se reinicia el nodo del LiDAR");
      } else if (rc == 127 || rc < 0) {
        RCLCPP_ERROR(get_logger(), "Sin /scan y no pude ejecutar pkill (código %d)", rc);
      } else {
        RCLCPP_ERROR(get_logger(), "Sin /scan y no encuentro el proceso del LiDAR (¿ya se está relanzando?)");
      }
    }
  }

  std::unique_ptr<Freno> freno_;
  std::unique_ptr<Vigilante> vigilante_;
  bool estado_publicado_ = false;
  bool brazo_ocupado_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_brazo_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_freno_;
  rclcpp::TimerBase::SharedPtr timer_, timer_vig_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrenoLidar>());
  rclcpp::shutdown();
  return 0;
}
