// Supervisor de la misión de mapeo (lógica en include/wallens_nav/supervisor_logica.hpp). Vigila explore_lite, la batería, la odometría y el freno; al terminar o fallar
// detiene la exploración, guarda el mapa con slam_toolbox y sale (mapeo.launch.py cierra entonces todo: on_exit=Shutdown).
// En un fallo además publica /parada true (el driver la mantiene hasta /parada false). Nunca reintenta.
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "slam_toolbox/srv/save_map.hpp"
#include "std_msgs/msg/bool.hpp"
#include "wallens_nav/supervisor_logica.hpp"

#ifdef WALLENS_HAS_EXPLORE_LITE_MSGS
#include "explore_lite_msgs/msg/explore_status.hpp"
#endif

using namespace std::chrono_literals;

class SupervisorMision : public rclcpp::Node
{
public:
  SupervisorMision()
  : Node("supervisor_mision")
  {
    auto p = [this](const std::string & n, double d) {return declare_parameter<double>(n, d);};
    // orden de argumentos de la clase: max, bateria_min, ventana, sin_progreso, progreso_min, bloqueo, dato, arranque
    const double max_duracion = p("max_duration", 1200.0);
    const double bateria_min = p("battery_min_voltage", 14.6);
    const double sin_progreso = p("no_progress_timeout", 120.0);
    const double progreso_min = p("progress_distance", 0.20);
    const double bloqueo_max = p("blocked_timeout", 20.0);
    const double dato_timeout = p("data_timeout", 3.0);
    const double arranque = p("startup_grace", 30.0);
    sup_ = wallens_nav::Supervisor(max_duracion, bateria_min, 10.0, sin_progreso, progreso_min, bloqueo_max,
        dato_timeout, arranque);
    std::string dir = declare_parameter<std::string>("map_dir", "~/mapas");
    if (!dir.empty() && dir[0] == '~') {
      const char * home = std::getenv("HOME");
      dir = std::string(home ? home : "") + dir.substr(1);
    }
    carpeta_ = dir;

    auto latched = rclcpp::QoS(1).transient_local();
    pub_parada_ = create_publisher<std_msgs::msg::Bool>("parada", latched);
    pub_explorar_ = create_publisher<std_msgs::msg::Bool>("explore/resume", 10);
#ifdef WALLENS_HAS_EXPLORE_LITE_MSGS
    sub_estado_ = create_subscription<explore_lite_msgs::msg::ExploreStatus>(
      "explore/status", latched,
      [this](explore_lite_msgs::msg::ExploreStatus::ConstSharedPtr m) {sup_.on_estado(m->status);});
#endif
    sub_bat_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state", 10,
      [this](sensor_msgs::msg::BatteryState::ConstSharedPtr m) {sup_.on_bateria(m->voltage, ahora());});
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        sup_.on_pose(m->pose.pose.position.x, m->pose.pose.position.y, ahora());
      });
    sub_freno_ = create_subscription<std_msgs::msg::Bool>(
      "freno_obstaculo", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {sup_.on_freno(m->data, ahora());});
    cli_guardar_ = create_client<slam_toolbox::srv::SaveMap>("slam_toolbox/save_map");
    timer_ = create_wall_timer(1s, [this]() {ciclo();});
    RCLCPP_INFO(get_logger(), "Supervisor de la misión en marcha");
  }

private:
  static double ahora()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void ciclo()
  {
    if (terminado_) {return;}
    auto r = sup_.evaluar(ahora());
    if (!r) {return;}
    terminado_ = true;
    timer_->cancel();
    std_msgs::msg::Bool b;
    b.data = false;
    pub_explorar_->publish(b);                 // explore_lite cancela su meta en Nav2
    if (r->tipo == "fallo") {
      b.data = true;
      pub_parada_->publish(b);
      RCLCPP_ERROR(get_logger(), "Misión abortada: %s. /parada publicada; para seguir: /parada false",
        r->motivo.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Misión terminada: %s", r->motivo.c_str());
    }
    RCLCPP_INFO(get_logger(), "Recorrido según el EKF: %.2f m", sup_.recorrido());
    guardar_mapa();
  }

  // Asíncrono: el executor sigue girando (y publicando); salir() cierra tras ~0.5 s.
  void guardar_mapa()
  {
    std::error_code ec;
    std::filesystem::create_directories(carpeta_, ec);
    char marca[64];
    std::time_t t = std::time(nullptr);
    std::strftime(marca, sizeof marca, "mapa_%Y%m%d_%H%M%S", std::localtime(&t));
    ruta_ = (std::filesystem::path(carpeta_) / marca).string();
    t_inicio_ = this->now();
    espera_ = create_wall_timer(100ms, [this]() {esperar_servicio();});
  }

  void esperar_servicio()
  {
    if (cli_guardar_->service_is_ready()) {
      espera_->cancel();
      auto pedido = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
      pedido->name.data = ruta_;
      futuro_ = cli_guardar_->async_send_request(pedido).future.share();
      t_inicio_ = this->now();
      espera_ = create_wall_timer(100ms, [this]() {esperar_resultado();});
    } else if ((this->now() - t_inicio_).seconds() >= 5.0) {
      espera_->cancel();
      RCLCPP_ERROR(get_logger(), "slam_toolbox/save_map no responde: el mapa no se guardó");
      salir();
    }
  }

  void esperar_resultado()
  {
    if (futuro_.valid() && futuro_.wait_for(0s) == std::future_status::ready) {
      espera_->cancel();
      bool ok = false;
      try {ok = futuro_.get()->result == 0;} catch (...) {}
      if (ok) {
        RCLCPP_INFO(get_logger(), "Mapa guardado: %s.pgm y %s.yaml", ruta_.c_str(), ruta_.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "No se pudo guardar el mapa en %s", ruta_.c_str());
      }
      salir();
    } else if ((this->now() - t_inicio_).seconds() >= 15.0) {
      espera_->cancel();
      RCLCPP_ERROR(get_logger(), "No se pudo guardar el mapa en %s", ruta_.c_str());
      salir();
    }
  }

  void salir()
  {
    // que salgan /parada y explore/resume (latched) antes de cerrar
    espera_ = create_wall_timer(500ms, []() {rclcpp::shutdown();});
  }

  wallens_nav::Supervisor sup_;
  std::string carpeta_, ruta_;
  bool terminado_ = false;
  rclcpp::Time t_inicio_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_parada_, pub_explorar_;
#ifdef WALLENS_HAS_EXPLORE_LITE_MSGS
  rclcpp::Subscription<explore_lite_msgs::msg::ExploreStatus>::SharedPtr sub_estado_;
#endif
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_bat_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_freno_;
  rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr cli_guardar_;
  std::shared_future<slam_toolbox::srv::SaveMap::Response::SharedPtr> futuro_;
  rclcpp::TimerBase::SharedPtr timer_, espera_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SupervisorMision>());
  rclcpp::shutdown();
  return 0;
}
