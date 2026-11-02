// Exige que wallens_nav::Supervisor (C++) dé lo mismo que supervisor_logica.py (casos de generar_casos_supervisor.py).
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "wallens_nav/supervisor_logica.hpp"

namespace {

// JSON mínimo (solo lo que escribe json.dump): sin dependencias externas.
struct J {
  enum T {NUL, BOOL, NUM, STR, ARR, OBJ} t = NUL;
  bool b = false;
  double n = 0;
  std::string s;
  std::vector<J> a;
  std::map<std::string, J> o;
  const J & at(const std::string & k) const {return o.at(k);}
  bool has(const std::string & k) const {return o.count(k) > 0;}
};

class Parser {
public:
  explicit Parser(const std::string & s) : s_(s) {}
  J parse() {return val();}

private:
  const std::string & s_;
  size_t i_ = 0;
  void ws() {while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) {++i_;}}
  void expect(char c)
  {
    ws();
    if (i_ >= s_.size() || s_[i_] != c) {throw std::runtime_error(std::string("JSON: se esperaba ") + c);}
    ++i_;
  }
  std::string str()
  {
    expect('"');
    std::string r;
    while (s_[i_] != '"') {
      if (s_[i_] == '\\') {
        ++i_;
        switch (s_[i_]) {
          case 'n': r += '\n'; break;
          case 't': r += '\t'; break;
          case 'u': {
              unsigned c = std::stoul(s_.substr(i_ + 1, 4), nullptr, 16);
              i_ += 4;
              if (c < 0x80) {r += static_cast<char>(c);}
              else if (c < 0x800) {r += static_cast<char>(0xC0 | (c >> 6)); r += static_cast<char>(0x80 | (c & 63));}
              else {
                r += static_cast<char>(0xE0 | (c >> 12)); r += static_cast<char>(0x80 | ((c >> 6) & 63));
                r += static_cast<char>(0x80 | (c & 63));
              }
              break;
            }
          default: r += s_[i_];
        }
        ++i_;
      } else {
        r += s_[i_++];
      }
    }
    ++i_;
    return r;
  }
  J val()
  {
    ws();
    J j;
    char c = s_[i_];
    if (c == '{') {
      j.t = J::OBJ; ++i_; ws();
      if (s_[i_] == '}') {++i_; return j;}
      for (;;) {
        ws(); std::string k = str(); expect(':'); j.o[k] = val(); ws();
        if (s_[i_] == ',') {++i_; continue;}
        expect('}'); return j;
      }
    }
    if (c == '[') {
      j.t = J::ARR; ++i_; ws();
      if (s_[i_] == ']') {++i_; return j;}
      for (;;) {
        j.a.push_back(val()); ws();
        if (s_[i_] == ',') {++i_; continue;}
        expect(']'); return j;
      }
    }
    if (c == '"') {j.t = J::STR; j.s = str(); return j;}
    if (s_.compare(i_, 4, "true") == 0) {j.t = J::BOOL; j.b = true; i_ += 4; return j;}
    if (s_.compare(i_, 5, "false") == 0) {j.t = J::BOOL; i_ += 5; return j;}
    if (s_.compare(i_, 4, "null") == 0) {i_ += 4; return j;}
    size_t used = 0;
    j.t = J::NUM; j.n = std::stod(s_.substr(i_), &used); i_ += used;
    return j;
  }
};

double numero(const J & j)
{
  return j.t == J::STR ? std::nan("") : j.n;    // "nan" -> NaN
}

double param(const J & p, const char * k, double def)
{
  return p.has(k) ? p.at(k).n : def;
}

}  // namespace

TEST(SupervisorCpp, IgualQueElOraculoDePython)
{
  std::ifstream f(CASOS_JSON);
  ASSERT_TRUE(f.good()) << "falta " << CASOS_JSON << ": python3 test/generar_casos_supervisor.py";
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string txt = ss.str();
  const J casos = Parser(txt).parse();
  ASSERT_FALSE(casos.a.empty());
  for (const J & c : casos.a) {
    SCOPED_TRACE(c.at("nombre").s);
    const J & p = c.at("params");
    wallens_nav::Supervisor s(
      param(p, "max_duracion", 1200.0), param(p, "bateria_min", 14.6), param(p, "ventana_bateria", 10.0),
      param(p, "sin_progreso", 120.0), param(p, "progreso_min", 0.20), param(p, "bloqueo_max", 20.0),
      param(p, "dato_timeout", 3.0), param(p, "arranque", 30.0));
    size_t k = 0;
    for (const J & e : c.at("eventos").a) {
      ++k;
      const std::string & op = e.at("op").s;
      if (op == "estado") {
        s.on_estado(e.at("v").s);
      } else if (op == "bateria") {
        s.on_bateria(numero(e.at("v")), e.at("t").n);
      } else if (op == "pose") {
        s.on_pose(e.at("x").n, e.at("y").n, e.at("t").n);
      } else if (op == "freno") {
        s.on_freno(e.at("f").b, e.at("t").n);
      } else if (op == "eval") {
        auto r = s.evaluar(e.at("t").n);
        const J & esp = e.at("esperado");
        if (esp.t == J::NUL) {
          ASSERT_FALSE(r.has_value()) << "evento " << k << ": " << r->tipo << " " << r->motivo;
        } else {
          ASSERT_TRUE(r.has_value()) << "evento " << k << ": esperado " << esp.a[1].s;
          EXPECT_EQ(r->tipo, esp.a[0].s) << "evento " << k;
          EXPECT_EQ(r->motivo, esp.a[1].s) << "evento " << k;
        }
      }
    }
    EXPECT_NEAR(s.recorrido(), c.at("recorrido").n, 1e-9);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
