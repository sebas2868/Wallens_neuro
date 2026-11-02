// Compara freno_logica.hpp con el oráculo de Python (test/casos_freno.json, generado por test/generar_casos.py):
// bools y textos iguales, floats con tolerancia 1e-9.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "wallens_safety/freno_logica.hpp"

using namespace wallens_safety;

// --- JSON mínimo (acepta NaN, Infinity y -Infinity como escribe Python) ---
struct J {
  enum T { NUL, BOOL, NUM, STR, ARR, OBJ } t = NUL;
  bool b = false;
  double n = 0.0;
  std::string s;
  std::vector<J> a;
  std::map<std::string, J> o;
  const J& operator[](const std::string& k) const { return o.at(k); }
  const J& operator[](std::size_t i) const { return a.at(i); }
  bool nulo() const { return t == NUL; }
  double num() const { return n; }
  std::size_t size() const { return a.size(); }
};

class Parser {
 public:
  explicit Parser(const std::string& s) : s_(s) {}
  J leer() {
    J j = valor();
    blancos();
    if (p_ != s_.size()) throw std::runtime_error("JSON: basura al final");
    return j;
  }

 private:
  const std::string& s_;
  std::size_t p_ = 0;
  void blancos() {
    while (p_ < s_.size() && (s_[p_] == ' ' || s_[p_] == '\n' || s_[p_] == '\t' || s_[p_] == '\r')) ++p_;
  }
  bool prefijo(const char* w) {
    std::size_t n = std::string(w).size();
    if (s_.compare(p_, n, w) == 0) {
      p_ += n;
      return true;
    }
    return false;
  }
  J valor() {
    blancos();
    J j;
    char c = s_.at(p_);
    if (c == '{') {
      ++p_;
      j.t = J::OBJ;
      blancos();
      if (s_[p_] == '}') { ++p_; return j; }
      for (;;) {
        blancos();
        std::string k = cadena();
        blancos();
        if (s_[p_++] != ':') throw std::runtime_error("JSON: falta :");
        j.o[k] = valor();
        blancos();
        if (s_[p_] == ',') { ++p_; continue; }
        if (s_[p_++] != '}') throw std::runtime_error("JSON: falta }");
        return j;
      }
    }
    if (c == '[') {
      ++p_;
      j.t = J::ARR;
      blancos();
      if (s_[p_] == ']') { ++p_; return j; }
      for (;;) {
        j.a.push_back(valor());
        blancos();
        if (s_[p_] == ',') { ++p_; continue; }
        if (s_[p_++] != ']') throw std::runtime_error("JSON: falta ]");
        return j;
      }
    }
    if (c == '"') { j.t = J::STR; j.s = cadena(); return j; }
    if (prefijo("true")) { j.t = J::BOOL; j.b = true; return j; }
    if (prefijo("false")) { j.t = J::BOOL; return j; }
    if (prefijo("null")) return j;
    j.t = J::NUM;
    if (prefijo("NaN")) { j.n = std::nan(""); return j; }
    if (prefijo("Infinity")) { j.n = INF; return j; }
    if (prefijo("-Infinity")) { j.n = -INF; return j; }
    char* fin = nullptr;
    j.n = std::strtod(s_.c_str() + p_, &fin);
    if (fin == s_.c_str() + p_) throw std::runtime_error("JSON: número inválido");
    p_ = static_cast<std::size_t>(fin - s_.c_str());
    return j;
  }
  std::string cadena() {
    if (s_[p_++] != '"') throw std::runtime_error("JSON: falta \"");
    std::string r;
    while (s_[p_] != '"') {
      char c = s_[p_++];
      if (c == '\\') {
        char e = s_[p_++];
        switch (e) {
          case 'n': r += '\n'; break;
          case 't': r += '\t'; break;
          case 'u': {   // json.dump con ensure_ascii escribe \uXXXX; solo hay BMP (sin pares sustitutos)
            unsigned cp = static_cast<unsigned>(std::stoul(s_.substr(p_, 4), nullptr, 16));
            p_ += 4;
            if (cp < 0x80) r += static_cast<char>(cp);
            else if (cp < 0x800) { r += static_cast<char>(0xC0 | (cp >> 6)); r += static_cast<char>(0x80 | (cp & 0x3F)); }
            else { r += static_cast<char>(0xE0 | (cp >> 12)); r += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); r += static_cast<char>(0x80 | (cp & 0x3F)); }
            break;
          }
          default: r += e;
        }
      } else {
        r += c;
      }
    }
    ++p_;
    return r;
  }
};

static bool casi(double a, double b) {
  if (std::isnan(a) || std::isnan(b)) return std::isnan(a) && std::isnan(b);
  if (std::isinf(a) || std::isinf(b)) return a == b;
  return std::fabs(a - b) <= 1e-9;
}
#define CASI(a, b) EXPECT_TRUE(casi((a), (b))) << ctx << ": " << (a) << " != " << (b)

static void opt(const std::optional<double>& c, const J& e, const std::string& ctx) {
  ASSERT_EQ(c.has_value(), !e.nulo()) << ctx;
  if (c) {
    CASI(*c, e.num());
  }
}

static std::vector<double> nums(const J& j) {
  std::vector<double> v;
  for (const auto& x : j.a) v.push_back(x.num());
  return v;
}
static std::vector<Punto> pares(const J& j) {
  std::vector<Punto> v;
  for (const auto& x : j.a) v.emplace_back(x[0].num(), x[1].num());
  return v;
}
static Poligono huella_de(const J& h) { return huella_wallens(h[0].num(), h[1].num(), h[2].num()); }

static std::string leer_archivo(const std::string& ruta) {
  std::ifstream f(ruta, std::ios::binary);
  if (!f) throw std::runtime_error("no puedo abrir " + ruta);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static void caso_freno(const J& c, const std::string& ctx) {
  const J& p = c["params"];
  Poligono huella;
  double laser_x = 0.0, ml = 0.03, mg = 0.03, hz = 0.5, rg = 1.0;
  if (!c["huella"].nulo()) {
    huella = huella_de(c["huella"]);
    laser_x = p["laser_x"].num();
    ml = p["margen_lateral"].num();
    mg = p["margen_giro"].num();
    hz = p["horizonte_giro"].num();
    rg = p["retencion_giro"].num();
  }
  Freno f(p["dist"].num(), p["semiancho"].num(), p["yaw"].num(), p["minimo"].num(), p["scan_timeout"].num(),
          p["requiere_scan"].b, p["cmd_timeout"].num(), p["histeresis"].num(), static_cast<int>(p["min_lecturas"].num()),
          huella, laser_x, ml, mg, hz, rg);
  std::vector<std::vector<double>> scans;
  for (const auto& s : c["scans"].a) scans.push_back(nums(s["rangos"]));
  std::size_t k = 0;
  for (const auto& op : c["ops"].a) {
    std::string cx = ctx + " op " + std::to_string(k++);
    const std::string& o = op["op"].s;
    if (o == "scan") {
      const J& s = c["scans"][static_cast<std::size_t>(op["i"].num())];
      f.on_scan(scans[static_cast<std::size_t>(op["i"].num())], s["amin"].num(), s["inc"].num(), op["t"].num());
      opt(f.frente, op["frente"], cx + " frente");
      opt(f.atras, op["atras"], cx + " atras");
      EXPECT_EQ(f.cono_ok[1], op["cono"][0].b) << cx;
      EXPECT_EQ(f.cono_ok[0], op["cono"][1].b) << cx;
      EXPECT_EQ(f.puntos.size(), static_cast<std::size_t>(op["np"].num())) << cx;
    } else if (o == "cmd") {
      EXPECT_EQ(f.on_cmd(op["v"].num(), op["w"].num(), op["t"].num()), op["ok"].b) << cx;
    } else {
      auto r = f.ciclo(op["t"].num());
      ASSERT_EQ(r.has_value(), !op["r"].nulo()) << cx;
      if (r) {
        const J& e = op["r"];
        CASI(r->v, e[0].num());
        CASI(r->w, e[1].num());
        EXPECT_EQ(r->bloqueado, e[2].b) << cx;
        EXPECT_EQ(r->motivo, e[3].s) << cx;
      }
      EXPECT_EQ(f.frenando, op["frenando"].b) << cx;
      EXPECT_EQ(f.frenando_giro, op["frenando_giro"].b) << cx;
      CASI(f.w_frenado, op["w_frenado"].num());
    }
  }
}

TEST(FrenoCpp, CoincideConElOraculoDePython) {
  const char* dir = std::getenv("WALLENS_CASOS_DIR");
  std::string ruta = std::string(dir ? dir : CASOS_DIR) + "/casos_freno.json";
  std::string texto = leer_archivo(ruta);
  J raiz = Parser(texto).leer();
  std::map<std::string, int> cuenta;
  std::size_t idx = 0;
  for (const auto& c : raiz["casos"].a) {
    const std::string tipo = c["tipo"].s;
    std::string ctx = tipo + " #" + std::to_string(idx++) + (c.o.count("nombre") ? " (" + c["nombre"].s + ")" : "");
    cuenta[tipo]++;
    if (tipo == "huella") {
      Poligono p = huella_wallens(c["args"][0].num(), c["args"][1].num(), c["args"][2].num(),
                                  static_cast<int>(c["args"][3].num()));
      ASSERT_EQ(p.size(), c["esp"].size()) << ctx;
      for (std::size_t i = 0; i < p.size(); ++i) {
        CASI(p[i].first, c["esp"][i][0].num());
        CASI(p[i].second, c["esp"][i][1].num());
      }
    } else if (tipo == "geom") {
      Poligono P = huella_de(c["huella"]);
      auto pts = pares(c["puntos"]);
      for (std::size_t i = 0; i < pts.size(); ++i) {
        std::string cx = ctx + " pt " + std::to_string(i);
        EXPECT_EQ(dentro(P, pts[i].first, pts[i].second), c["dentro"][i].b) << cx;
        CASI(distancia_al_borde(P, pts[i].first, pts[i].second), c["borde"][i].num());
        opt(borde_x(P, pts[i].second, true), c["bx_adelante"][i], cx + " bx_adelante");
        opt(borde_x(P, pts[i].second, false), c["bx_atras"][i], cx + " bx_atras");
      }
    } else if (tipo == "dmin") {
      auto r = nums(c["rangos"]);
      opt(distancia_minima(r, c["amin"].num(), c["inc"].num(), c["yaw"].num(), c["centro"].num(), c["semi"].num(),
                           c["minimo"].num(), static_cast<int>(c["minl"].num())), c["esp"], ctx);
      auto pb = puntos_base(r, c["amin"].num(), c["inc"].num(), c["yaw"].num(), c["lx"].num(), c["minimo"].num());
      ASSERT_EQ(pb.size(), c["pb"].size()) << ctx;
      for (std::size_t i = 0; i < pb.size(); ++i) {
        CASI(pb[i].first, c["pb"][i][0].num());
        CASI(pb[i].second, c["pb"][i][1].num());
      }
    } else if (tipo == "pasillo") {
      Poligono P = huella_de(c["huella"]);
      auto pts = pares(c["puntos"]);
      opt(distancia_pasillo(pts, P, c["margen"].num(), true), c["adelante"], ctx + " adelante");
      opt(distancia_pasillo(pts, P, c["margen"].num(), false), c["atras"], ctx + " atras");
    } else if (tipo == "choque") {
      Poligono P = huella_de(c["huella"]);
      EXPECT_EQ(choque_al_girar(pares(c["puntos"]), c["w"].num(), P, c["margen"].num(), c["hor"].num(),
                                static_cast<int>(c["pasos"].num())), c["esp"].b) << ctx;
    } else if (tipo == "freno") {
      caso_freno(c, ctx);
    } else if (tipo == "vigilante") {
      Vigilante v(c["args"][0].num(), c["args"][1].num(), c["args"][2].num());
      std::size_t k = 0;
      for (const auto& op : c["ops"].a) {
        if (op["op"].s == "scan") v.on_scan(op["t"].num());
        else EXPECT_EQ(v.debe_reiniciar(op["t"].num()), op["esp"].b) << ctx << " op " << k;
        ++k;
      }
    } else {
      FAIL() << "tipo desconocido " << tipo;
    }
  }
  EXPECT_GE(idx, 500u);
  EXPECT_GT(cuenta["freno"], 300);
  EXPECT_GT(cuenta["vigilante"], 30);
}

TEST(FrenoCpp, EjemploDeLaDocumentacion) {   // sanidad sin JSON: un obstáculo delante bloquea el avance
  std::vector<double> r(450, 3.0);
  Freno f(0.30, 40 * PI / 180, -90 * PI / 180, 0.06, 0.3, true, 0.5, 0.05, 10);
  f.on_cmd(0.05, 0.0, 1.0);
  f.on_scan(std::vector<double>(450, 0.2), 0.0, 2 * PI / 450, 1.0);
  auto x = f.ciclo(1.0);
  ASSERT_TRUE(x.has_value());
  EXPECT_TRUE(x->bloqueado);
  EXPECT_EQ(x->v, 0.0);
}

TEST(FrenoCpp, ParseaLaHuellaComoNav2) {
  Poligono p = parsear_huella(" [[0.1, 0.2], [-0.3,0.4] ,[0.5, -0.6]] ");
  ASSERT_EQ(p.size(), 3u);
  EXPECT_EQ(p[1], Punto(-0.3, 0.4));
  EXPECT_TRUE(parsear_huella("").empty());
  for (const char* malo : {"[[0, 0], [1, 1]]", "[[0, 0], [1, 1], [2]]", "nada", "[[0, 0], [1, 1], [2, 2]", "[[a, 0]]",
                           "[[0, 0], [1, 1], [2, 2]] x"}) {
    EXPECT_THROW(parsear_huella(malo), std::invalid_argument) << malo;
  }
}
