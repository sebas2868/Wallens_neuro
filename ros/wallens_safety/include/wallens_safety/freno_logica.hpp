// Lógica del freno LiDAR y del vigilante sin ROS: traducción fiel de scripts/freno_logica.py y vigilante_logica.py.
// Mismo orden de operaciones y mismos casos límite; test/test_freno_cpp.cpp la compara con el oráculo de Python
// (test/casos_freno.json). Si cambias una, cambia la otra y regenera: python3 test/generar_casos.py
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace wallens_safety {

constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double PI = 3.14159265358979323846;  // == math.pi

using Punto = std::pair<double, double>;
using Poligono = std::vector<Punto>;

// a % b de Python para floats (el resultado toma el signo de b)
inline double py_mod(double a, double b) {
  double m = std::fmod(a, b);
  if (m != 0.0) {
    if ((b < 0.0) != (m < 0.0)) m += b;
  } else {
    m = std::copysign(0.0, b);
  }
  return m;
}
// min(a, b) y max(a, b) de Python: devuelven el primero salvo que el segundo gane estrictamente
inline double py_min(double a, double b) { return (b < a) ? b : a; }
inline double py_max(double a, double b) { return (b > a) ? b : a; }

// Menor distancia con ángulo (marco del robot) dentro de centro ± semiancho, o nullopt si hay menos de min_lecturas
// lecturas válidas (0, NaN, inf o menores que minimo no cuentan).
template <class R>
std::optional<double> distancia_minima(const R& rangos, double angulo_min, double incremento, double yaw, double centro,
                                       double semiancho, double minimo, int min_lecturas = 1) {
  std::optional<double> mejor;
  int validas = 0;
  const std::size_t n = rangos.size();
  for (std::size_t i = 0; i < n; ++i) {
    double r = static_cast<double>(rangos[i]);
    if (!(minimo <= r && r < INF)) continue;
    double a = angulo_min + static_cast<double>(i) * incremento + yaw;
    if (std::fabs(py_mod(a - centro + PI, 2 * PI) - PI) <= semiancho) {
      validas += 1;
      if (!mejor.has_value() || r < *mejor) mejor = r;
    }
  }
  if (validas >= min_lecturas) return mejor;
  return std::nullopt;
}

// Lecturas válidas como puntos (x, y) en base_link; el LiDAR está en (laser_x, 0) con su yaw.
template <class R>
std::vector<Punto> puntos_base(const R& rangos, double angulo_min, double incremento, double yaw, double laser_x,
                               double minimo) {
  std::vector<Punto> pts;
  const std::size_t n = rangos.size();
  for (std::size_t i = 0; i < n; ++i) {
    double r = static_cast<double>(rangos[i]);
    if (minimo <= r && r < INF) {
      double a = angulo_min + static_cast<double>(i) * incremento + yaw;
      pts.emplace_back(laser_x + r * std::cos(a), r * std::sin(a));
    }
  }
  return pts;
}

// Polígono desde el texto de `footprint` (mismo formato que Nav2: "[[x, y], ...]"). "" = sin huella (vacío).
inline Poligono parsear_huella(const std::string& texto) {
  auto invalido = [&texto]() { return std::invalid_argument("footprint inválido: " + texto); };
  const char* c = texto.c_str();
  auto espacios = [&c]() { while (std::isspace(static_cast<unsigned char>(*c))) ++c; };
  auto simbolo = [&](char s) { espacios(); if (*c != s) throw invalido(); ++c; };
  auto numero = [&]() {
    espacios();
    char* fin = nullptr;
    double v = std::strtod(c, &fin);
    if (fin == c || !std::isfinite(v)) throw invalido();
    c = fin;
    return v;
  };
  Poligono pts;
  espacios();
  if (*c == '\0') return pts;
  simbolo('[');
  do {
    simbolo('[');
    double x = numero();
    simbolo(',');
    double y = numero();
    simbolo(']');
    pts.emplace_back(x, y);
    espacios();
  } while (*c == ',' && ++c);
  simbolo(']');
  espacios();
  if (*c != '\0' || pts.size() < 3) throw invalido();
  return pts;
}

inline Poligono huella_wallens(double radio, double largo_gripper, double semiancho_gripper, int lados = 16) {
  double r = radio / std::cos(PI / lados);
  double xg = -std::sqrt(py_max(r * r - semiancho_gripper * semiancho_gripper, 0.0));
  double cola = -(radio + largo_gripper);
  Poligono pts;
  for (int k = 0; k < lados; ++k) {
    double a = 2 * PI * k / lados;
    if (k == lados / 2) {
      pts.emplace_back(xg, semiancho_gripper);
      pts.emplace_back(cola, semiancho_gripper);
      pts.emplace_back(cola, -semiancho_gripper);
      pts.emplace_back(xg, -semiancho_gripper);
    } else {
      pts.emplace_back(r * std::cos(a), r * std::sin(a));
    }
  }
  return pts;
}

inline bool dentro(const Poligono& poligono, double x, double y) {
  bool d = false;
  const std::size_t n = poligono.size();
  for (std::size_t i = 0; i < n; ++i) {
    double x1 = poligono[i].first, y1 = poligono[i].second;
    double x2 = poligono[(i + 1) % n].first, y2 = poligono[(i + 1) % n].second;
    if ((y1 > y) != (y2 > y) && x < x1 + (y - y1) * (x2 - x1) / (y2 - y1)) d = !d;
  }
  return d;
}

inline double distancia_al_borde(const Poligono& poligono, double x, double y) {
  double mejor = INF;
  const std::size_t n = poligono.size();
  for (std::size_t i = 0; i < n; ++i) {
    double x1 = poligono[i].first, y1 = poligono[i].second;
    double x2 = poligono[(i + 1) % n].first, y2 = poligono[(i + 1) % n].second;
    double dx = x2 - x1, dy = y2 - y1;
    double t = py_max(0.0, py_min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)));
    mejor = py_min(mejor, std::hypot(x - x1 - t * dx, y - y1 - t * dy));
  }
  return mejor;
}

// x del borde delantero (o trasero) a la altura lateral y, o nullopt si esa fila no lo corta.
inline std::optional<double> borde_x(const Poligono& poligono, double y, bool adelante) {
  std::vector<double> xs;
  const std::size_t n = poligono.size();
  for (std::size_t i = 0; i < n; ++i) {
    double x1 = poligono[i].first, y1 = poligono[i].second;
    double x2 = poligono[(i + 1) % n].first, y2 = poligono[(i + 1) % n].second;
    if ((y1 > y) != (y2 > y)) xs.push_back(x1 + (y - y1) * (x2 - x1) / (y2 - y1));
  }
  if (xs.empty()) return std::nullopt;
  // max()/min() de Python: el primero salvo que otro gane estrictamente
  double m = xs[0];
  for (std::size_t i = 1; i < xs.size(); ++i) m = adelante ? py_max(m, xs[i]) : py_min(m, xs[i]);
  return m;
}

inline std::optional<double> distancia_pasillo(const std::vector<Punto>& puntos, const Poligono& huella,
                                               double margen_lateral, bool adelante = true) {
  double semiancho = std::fabs(huella[0].second);
  for (std::size_t i = 1; i < huella.size(); ++i) semiancho = py_max(semiancho, std::fabs(huella[i].second));
  std::optional<double> mejor;
  for (const auto& p : puntos) {
    double x = p.first, y = p.second;
    if (std::fabs(y) > semiancho + margen_lateral) continue;
    double yb = py_max(-0.999 * semiancho, py_min(0.999 * semiancho, y));
    auto b = borde_x(huella, yb, adelante);
    if (!b.has_value()) continue;
    double d = adelante ? x - *b : *b - x;
    if (d >= 0 && (!mejor.has_value() || d < *mejor)) mejor = d;
  }
  return mejor;
}

constexpr double ACERCAMIENTO_MIN = 0.005;

inline bool choque_al_girar(const std::vector<Punto>& puntos, double w, const Poligono& huella, double margen,
                            double horizonte, int pasos = 6) {
  double giro = w * horizonte;
  double mx = std::hypot(huella[0].first, huella[0].second);
  for (std::size_t i = 1; i < huella.size(); ++i) mx = py_max(mx, std::hypot(huella[i].first, huella[i].second));
  double radio2 = (mx + margen) * (mx + margen);
  for (const auto& p : puntos) {
    double x = p.first, y = p.second;
    if (x * x + y * y > radio2 || dentro(huella, x, y)) continue;
    double inicial = distancia_al_borde(huella, x, y);
    for (int k = 1; k <= pasos; ++k) {
      double th = giro * k / pasos;
      double xr = x * std::cos(th) + y * std::sin(th);
      double yr = -x * std::sin(th) + y * std::cos(th);
      if (dentro(huella, xr, yr)) return true;
      double dist = distancia_al_borde(huella, xr, yr);
      if (dist < margen && dist < inicial - ACERCAMIENTO_MIN) return true;
    }
  }
  return false;
}

struct Resultado {
  double v, w;
  bool bloqueado;
  std::string motivo;
};

class Freno {
 public:
  // Los tiempos (`ahora`) son segundos de un reloj monótono que pasa el que llama. `huella` vacía = sin huella.
  Freno(double dist, double semiancho, double yaw, double minimo, double scan_timeout, bool requiere_scan,
        double cmd_timeout, double histeresis, int min_lecturas, Poligono huella = {}, double laser_x = 0.0,
        double margen_lateral = 0.03, double margen_giro = 0.03, double horizonte_giro = 0.5,
        double retencion_giro = 1.0)
      : dist_(dist), semiancho_(semiancho), yaw_(yaw), minimo_(minimo), scan_timeout_(scan_timeout),
        requiere_scan_(requiere_scan), cmd_timeout_(cmd_timeout), histeresis_(histeresis),
        min_lecturas_(min_lecturas), huella_(std::move(huella)), laser_x_(laser_x), margen_lateral_(margen_lateral),
        margen_giro_(margen_giro), horizonte_giro_(horizonte_giro), retencion_giro_(retencion_giro) {}

  template <class R>
  void on_scan(const R& rangos, double angulo_min, double incremento, double ahora) {
    frente = distancia_minima(rangos, angulo_min, incremento, yaw_, 0.0, semiancho_, minimo_, min_lecturas_);
    atras = distancia_minima(rangos, angulo_min, incremento, yaw_, PI, semiancho_, minimo_, min_lecturas_);
    cono_ok[1] = frente.has_value();
    cono_ok[0] = atras.has_value();
    if (!huella_.empty()) {
      puntos = puntos_base(rangos, angulo_min, incremento, yaw_, laser_x_, minimo_);
      frente = distancia_pasillo(puntos, huella_, margen_lateral_, true);
      atras = distancia_pasillo(puntos, huella_, margen_lateral_, false);
    }
    t_scan = ahora;
  }

  // Guarda el comando; devuelve false (y lo descarta) si no es finito.
  bool on_cmd(double v_, double w_, double ahora) {
    if (!(std::isfinite(v_) && std::isfinite(w_))) return false;
    v = v_;
    w = w_;
    t_cmd = ahora;
    return true;
  }

  // nullopt si no hay comando reciente (no se publica nada)
  std::optional<Resultado> ciclo(double ahora) {
    if (ahora - t_cmd > cmd_timeout_) {
      frenando = false;
      return std::nullopt;
    }
    double vv = v, ww = w;
    bool bloqueado = false;
    std::string motivo;
    bool sin_scan = requiere_scan_ && ahora - t_scan > scan_timeout_;
    if (vv != 0.0) {
      if (sin_scan) {
        bloqueado = true;
        motivo = "sin /scan reciente";
      } else {
        const std::optional<double>& d = (vv > 0) ? frente : atras;
        if (requiere_scan_ && !cono_ok[vv > 0 ? 1 : 0]) {
          bloqueado = true;
          motivo = "sector sin lecturas válidas suficientes (LiDAR tapado o cegado)";
        } else if (d.has_value() && *d < dist_ + (frenando ? histeresis_ : 0.0)) {
          bloqueado = true;
          char buf[64];
          std::snprintf(buf, sizeof buf, "obstáculo a %.2f m ", *d);
          motivo = std::string(buf) + (vv > 0 ? "al frente" : "atrás");
        }
      }
    }
    frenando = bloqueado;
    if (!huella_.empty() && ww != 0.0) {
      if (sin_scan) {
        frenando_giro = true;
        if (motivo.empty()) motivo = "sin /scan reciente (giro)";
      } else {
        double margen = margen_giro_ + ((frenando_giro && ww * w_frenado > 0) ? histeresis_ : 0.0);
        if (!(cache_valida && cache_t_scan == t_scan && cache_w == ww && cache_margen == margen)) {
          bool choque = choque_al_girar(puntos, ww, huella_, margen, horizonte_giro_);
          cache_valida = true;
          cache_t_scan = t_scan;
          cache_w = ww;
          cache_margen = margen;
          cache_choque = choque;
        }
        bool mismo_sentido = ww * w_frenado > 0;
        if (cache_choque) {
          t_choque_giro = ahora;
          w_frenado = ww;
        }
        frenando_giro = cache_choque || (mismo_sentido && ahora - t_choque_giro < retencion_giro_);
        if (!frenando_giro) w_frenado = 0.0;
        if (frenando_giro) motivo = (motivo.empty() ? std::string() : motivo + "; ") + "la huella barrería un obstáculo al girar";
      }
      if (frenando_giro) {
        ww = 0.0;
        bloqueado = true;
      }
    }
    return Resultado{frenando ? 0.0 : vv, ww, bloqueado, motivo};
  }

  // Estado (público, como en Python)
  std::optional<double> frente, atras;
  std::vector<Punto> puntos;
  bool cono_ok[2] = {false, false};  // [1] delantero, [0] trasero
  bool frenando_giro = false;
  double w_frenado = 0.0;
  double t_choque_giro = -INF;
  bool cache_valida = false;
  double cache_t_scan = 0.0, cache_w = 0.0, cache_margen = 0.0;
  bool cache_choque = false;
  double t_scan = -INF;
  double v = 0.0, w = 0.0;
  double t_cmd = -INF;
  bool frenando = false;

 private:
  double dist_, semiancho_, yaw_, minimo_, scan_timeout_;
  bool requiere_scan_;
  double cmd_timeout_, histeresis_;
  int min_lecturas_;
  Poligono huella_;
  double laser_x_, margen_lateral_, margen_giro_, horizonte_giro_, retencion_giro_;
};

// Decide cuándo reiniciar el nodo del LiDAR. `ahora` son segundos de un reloj monótono.
class Vigilante {
 public:
  Vigilante(double scan_perdido, double espera, double inicio)
      : scan_perdido_(scan_perdido), espera_(espera), t_scan_(inicio) {}
  void on_scan(double ahora) { t_scan_ = ahora; }
  bool debe_reiniciar(double ahora) {
    if (ahora - t_scan_ > scan_perdido_ && ahora - t_reinicio_ > espera_) {
      t_reinicio_ = ahora;
      return true;
    }
    return false;
  }

 private:
  double scan_perdido_, espera_, t_scan_, t_reinicio_ = -INF;
};

}  // namespace wallens_safety
