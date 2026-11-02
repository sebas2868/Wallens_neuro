// Lógica del supervisor de la misión de mapeo, sin ROS: traducción fiel de test/supervisor_logica.py.
// La usa src/supervisor_mision.cpp y se prueba en test/test_supervisor_cpp.cpp contra el oráculo de Python.
// Decide cuándo se termina la misión ("fin": se guarda el mapa) o se aborta ("fallo": además /parada). Nunca reintenta.
#pragma once
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace wallens_nav {

inline const char * COMPLETA = "exploration_complete";  // explore_lite_msgs/ExploreStatus

struct Resultado {
  std::string tipo;    // "fin" | "fallo"
  std::string motivo;
};

class Supervisor {
public:
  // Los tiempos (t) son segundos de un reloj monótono que pasa el que llama.
  // sin_progreso <= 0 desactiva la comprobación de progreso.
  explicit Supervisor(
    double max_duracion = 1200.0, double bateria_min = 14.6, double ventana_bateria = 10.0,
    double sin_progreso = 120.0, double progreso_min = 0.20, double bloqueo_max = 20.0,
    double dato_timeout = 3.0, double arranque = 30.0)
  : max_duracion_(max_duracion), bateria_min_(bateria_min), ventana_bateria_(ventana_bateria),
    sin_progreso_(sin_progreso), progreso_min_(progreso_min), bloqueo_max_(bloqueo_max),
    dato_timeout_(dato_timeout), arranque_(arranque) {}

  void on_estado(const std::string & estado) {estado_ = estado;}

  void on_bateria(double voltaje, double t)
  {
    if (!std::isfinite(voltaje)) {return;}
    voltajes_.emplace_back(t, voltaje);
    while (!voltajes_.empty() && t - voltajes_.front().first > ventana_bateria_) {voltajes_.pop_front();}
    t_bateria_ = t;
  }

  void on_pose(double x, double y, double t)
  {
    if (pose_) {recorrido_ += std::hypot(x - pose_->first, y - pose_->second);}
    pose_ = std::make_pair(x, y);
    t_pose_ = t;
    if (!ancla_ || std::hypot(x - ancla_->x, y - ancla_->y) >= progreso_min_) {ancla_ = Ancla{x, y, t};}
  }

  void on_freno(bool frenado, double t)
  {
    if (!frenado) {t_freno_.reset();} else if (!t_freno_) {t_freno_ = t;}
  }

  // nullopt si se sigue; si no, {"fin" | "fallo", motivo}.
  std::optional<Resultado> evaluar(double t)
  {
    if (!t0_) {t0_ = t;}
    const double transcurrido = t - *t0_;
    if (estado_ && *estado_ == COMPLETA) {
      return Resultado{"fin", "exploración completa: no quedan fronteras alcanzables"};
    }
    if (transcurrido > max_duracion_) {
      return Resultado{"fin", "tiempo máximo de " + fmt("%.0f", max_duracion_) + " s"};
    }
    // media de la ventana: un pico de corriente al arrancar baja el voltaje un momento sin que la batería esté baja
    if (!voltajes_.empty() && t - voltajes_.front().first >= 0.5 * ventana_bateria_) {
      double suma = 0.0;
      for (const auto & tv : voltajes_) {suma += tv.second;}
      const double media = suma / static_cast<double>(voltajes_.size());
      if (media < bateria_min_) {
        return Resultado{"fin", "batería baja: " + fmt("%.2f", media) + " V de media (mínimo " +
                         fmt("%.2f", bateria_min_) + " V)"};
      }
    }
    if (transcurrido > arranque_) {
      if (t - t_bateria_ > dato_timeout_) {return Resultado{"fallo", "sin /battery_state reciente"};}
      if (t - t_pose_ > dato_timeout_) {return Resultado{"fallo", "sin /odometry/filtered reciente"};}
      if (sin_progreso_ > 0 && ancla_ && t - ancla_->t > sin_progreso_) {
        return Resultado{"fallo", "sin avanzar " + fmt("%.2f", progreso_min_) + " m en " +
                         fmt("%.0f", sin_progreso_) + " s"};
      }
    }
    if (t_freno_ && t - *t_freno_ > bloqueo_max_) {
      return Resultado{"fallo", "frenado por obstáculo más de " + fmt("%.0f", bloqueo_max_) + " s seguidos"};
    }
    return std::nullopt;
  }

  double recorrido() const {return recorrido_;}

private:
  struct Ancla {double x, y, t;};

  static std::string fmt(const char * f, double v)
  {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
  }

  double max_duracion_, bateria_min_, ventana_bateria_, sin_progreso_, progreso_min_, bloqueo_max_,
    dato_timeout_, arranque_;
  std::optional<double> t0_;
  std::optional<std::string> estado_;
  std::deque<std::pair<double, double>> voltajes_;
  double t_bateria_ = -std::numeric_limits<double>::infinity();
  double t_pose_ = -std::numeric_limits<double>::infinity();
  std::optional<Ancla> ancla_;
  std::optional<double> t_freno_;
  double recorrido_ = 0.0;
  std::optional<std::pair<double, double>> pose_;
};

}  // namespace wallens_nav
