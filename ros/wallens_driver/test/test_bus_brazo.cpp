#include <gtest/gtest.h>

#include <atomic>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>

#include "wallens_driver/bus_brazo.hpp"

using namespace wallens;

namespace {

// Puerto falso: anota cada llamada y desde qué hilo llegó. `fallos` y `errores` se programan por clave "id:dir".
class PuertoFalso : public PuertoDxl {
 public:
  struct Llamada { char tipo; uint8_t id; uint16_t dir; uint8_t largo; uint32_t valor; };

  RespuestaPuerto ping(uint8_t id, uint16_t* modelo) override {
    registrar({'p', id, 0, 0, 0});
    *modelo = 1030;
    return {0, 0};
  }
  RespuestaPuerto reboot(uint8_t id) override {
    registrar({'b', id, 0, 0, 0});
    return {0, ALERTA_HW_BRAZO};          // el estado sale antes del reinicio: con la alerta todavía puesta
  }
  RespuestaPuerto leer(uint8_t id, uint16_t dir, uint8_t largo, std::vector<uint8_t>* datos) override {
    registrar({'r', id, dir, largo, 0});
    if (lanzar) throw std::runtime_error("puerto roto");
    datos->clear();
    for (int i = 0; i < largo; ++i) datos->push_back(static_cast<uint8_t>(base + i));
    return respuesta(id, dir);
  }
  RespuestaPuerto escribir(uint8_t id, uint16_t dir, uint8_t largo, uint32_t valor) override {
    registrar({'w', id, dir, largo, valor});
    if (lanzar) throw std::runtime_error("puerto roto");
    return respuesta(id, dir);
  }

  std::vector<Llamada> llamadas;
  std::set<std::thread::id> hilos;
  std::map<std::pair<int, int>, RespuestaPuerto> programadas;
  uint8_t base = 0x10;
  bool lanzar = false;

 private:
  void registrar(Llamada c) {
    llamadas.push_back(c);
    hilos.insert(std::this_thread::get_id());
  }
  RespuestaPuerto respuesta(uint8_t id, uint16_t dir) {
    auto it = programadas.find({id, dir});
    return it == programadas.end() ? RespuestaPuerto{0, 0} : it->second;
  }
};

// Con 57600 baudios cada operación cuesta ~5 ms y el tope por llamada es 12 ms: las pruebas de orden y de cortes
// necesitan llamadas de 3 operaciones, así que usan un presupuesto holgado.
ConfigBrazo holgado() {
  ConfigBrazo c;
  c.max_ms = 100.0;
  return c;
}

OperacionDxl leer(uint8_t id, uint16_t dir, uint8_t largo) { return {TipoOp::Leer, id, dir, largo, 0}; }
OperacionDxl escribir(uint8_t id, uint16_t dir, uint8_t largo, uint32_t v) { return {TipoOp::Escribir, id, dir, largo, v}; }

struct Recogida {
  int veces = 0;
  RespuestaDxl r;
  BusBrazo::Finalizador fin() { return [this](RespuestaDxl&& x) { ++veces; r = std::move(x); }; }
};

}  // namespace

TEST(BusBrazo, EjecutaEnOrdenYDevuelveLecturas) {
  PuertoFalso p;
  BusBrazo b(&p, holgado());
  Recogida rc;
  ASSERT_TRUE(b.encolar({escribir(5, 112, 4, 87), escribir(5, 108, 4, 10), leer(5, 132, 4)}, rc.fin()));
  EXPECT_EQ(rc.veces, 0);                                   // encolar no toca el bus
  EXPECT_TRUE(p.llamadas.empty());
  b.atender(12.0);
  ASSERT_EQ(rc.veces, 1);
  EXPECT_TRUE(rc.r.aceptada);
  ASSERT_EQ(p.llamadas.size(), 3u);
  EXPECT_EQ(p.llamadas[0].dir, 112);                        // la velocidad antes que la aceleración
  EXPECT_EQ(p.llamadas[1].dir, 108);
  EXPECT_EQ(rc.r.resultados[2].valor, 0x13121110u);         // little endian
}

TEST(BusBrazo, LecturaEnBloqueVaEnDatos) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  Recogida rc;
  ASSERT_TRUE(b.encolar({leer(3, 126, 21)}, rc.fin()));
  b.atender(12.0);
  ASSERT_EQ(rc.r.resultados[0].datos.size(), 21u);
  EXPECT_EQ(rc.r.resultados[0].datos[20], 0x10 + 20);
}

TEST(BusBrazo, ListaBlancaNuncaLlegaAlBus) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  const std::vector<OperacionDxl> prohibidas = {
      leer(2, 132, 4),            // rueda izquierda
      escribir(4, 104, 4, 0),     // rueda derecha
      escribir(3, 7, 1, 9),       // cambiar el ID
      escribir(3, 8, 1, 1),       // cambiar el baudrate
      escribir(3, 6, 4, 0),       // pisa 7 y 8 desde 6
      leer(3, 200, 1),            // fuera de los rangos
      leer(3, 125, 2),            // empieza antes del bloque 126..146
      leer(3, 146, 2),            // termina después del bloque
      escribir(3, 64, 3, 0),      // largo de escritura inválido
      leer(3, 126, 33),           // bloque demasiado largo
      leer(3, 126, 0),
  };
  for (const auto& op : prohibidas) {
    Recogida rc;
    ASSERT_TRUE(b.encolar({op}, rc.fin()));
    b.atender(12.0);
    ASSERT_EQ(rc.veces, 1);
    EXPECT_EQ(rc.r.resultados[0].estado, Estado::Rechazada) << "dir " << op.direccion << " id " << int(op.id);
  }
  EXPECT_TRUE(p.llamadas.empty());
}

TEST(BusBrazo, RechazadaCortaLaLlamada) {
  PuertoFalso p;
  BusBrazo b(&p, holgado());
  Recogida rc;
  b.encolar({escribir(5, 64, 1, 1), leer(2, 132, 4), escribir(5, 64, 1, 0)}, rc.fin());
  b.atender(12.0);
  EXPECT_EQ(rc.r.resultados[0].estado, Estado::Ok);
  EXPECT_EQ(rc.r.resultados[1].estado, Estado::Rechazada);
  EXPECT_EQ(rc.r.resultados[2].estado, Estado::NoEjecutada);
  EXPECT_EQ(p.llamadas.size(), 1u);
}

TEST(BusBrazo, ParaEnLaPrimeraQueFalla) {
  PuertoFalso p;
  p.programadas[{5, 108}] = {-3001, 0};                     // sin respuesta del motor
  BusBrazo b(&p, holgado());
  Recogida rc;
  b.encolar({escribir(5, 112, 4, 87), escribir(5, 108, 4, 10), escribir(5, 116, 4, 3000)}, rc.fin());
  b.atender(12.0);
  EXPECT_EQ(rc.r.resultados[0].estado, Estado::Ok);
  EXPECT_EQ(rc.r.resultados[1].estado, Estado::ErrorComunicacion);
  EXPECT_EQ(rc.r.resultados[1].comunicacion, -3001);
  EXPECT_EQ(rc.r.resultados[2].estado, Estado::NoEjecutada);
  EXPECT_EQ(p.llamadas.size(), 2u);
}

TEST(BusBrazo, ErrorDelMotorCorta) {
  PuertoFalso p;
  p.programadas[{5, 64}] = {0, 0x01};                       // error de resultado, no solo alerta
  BusBrazo b(&p, holgado());
  Recogida rc;
  b.encolar({escribir(5, 64, 1, 1), escribir(5, 116, 4, 3000)}, rc.fin());
  b.atender(12.0);
  EXPECT_EQ(rc.r.resultados[0].estado, Estado::ErrorMotor);
  EXPECT_EQ(rc.r.resultados[0].error_motor, 0x01);
  EXPECT_EQ(rc.r.resultados[1].estado, Estado::NoEjecutada);
}

TEST(BusBrazo, AlertaDeHardwareSeDevuelveSinCortar) {
  PuertoFalso p;
  p.programadas[{3, 126}] = {0, ALERTA_HW_BRAZO};
  BusBrazo b(&p, holgado());
  Recogida rc;
  b.encolar({leer(3, 126, 21), leer(3, 70, 1)}, rc.fin());   // se puede leer el error después de la alerta
  b.atender(12.0);
  EXPECT_EQ(rc.r.resultados[0].estado, Estado::Ok);
  EXPECT_EQ(rc.r.resultados[0].error_motor, ALERTA_HW_BRAZO);
  EXPECT_EQ(rc.r.resultados[1].estado, Estado::Ok);
}

TEST(BusBrazo, ExcepcionDelPuertoNoSaleDeAtender) {
  PuertoFalso p;
  p.lanzar = true;
  BusBrazo b(&p, {});
  Recogida rc;
  b.encolar({leer(3, 132, 4)}, rc.fin());
  EXPECT_NO_THROW(b.atender(12.0));
  ASSERT_EQ(rc.veces, 1);
  EXPECT_EQ(rc.r.resultados[0].estado, Estado::ErrorComunicacion);
  EXPECT_NE(rc.r.motivo.find("puerto roto"), std::string::npos);
}

TEST(BusBrazo, DemasiadasOperacionesSeRechazanDeInmediato) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  Recogida rc;
  std::vector<OperacionDxl> ops(9, leer(3, 70, 1));
  EXPECT_FALSE(b.encolar(ops, rc.fin()));
  EXPECT_EQ(rc.veces, 1);
  EXPECT_FALSE(rc.r.aceptada);
  EXPECT_EQ(b.en_cola(), 0u);
  Recogida vacia;
  EXPECT_FALSE(b.encolar({}, vacia.fin()));
  EXPECT_FALSE(vacia.r.aceptada);
}

TEST(BusBrazo, PresupuestoRechazaSinTocarElBus) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  // una lectura suelta ~5 ms; un bloque de 21 bytes ~8.5 ms; dos bloques ~17 ms > 12
  EXPECT_NEAR(b.estimar_ms({leer(3, 132, 4)}), 5.5, 0.3);
  EXPECT_NEAR(b.estimar_ms({leer(3, 126, 21)}), 8.5, 0.3);
  EXPECT_NEAR(b.estimar_ms({escribir(5, 116, 4, 0)}), 5.2, 0.3);
  Recogida dos;
  EXPECT_FALSE(b.encolar({leer(3, 126, 21), leer(5, 126, 21)}, dos.fin()));
  EXPECT_FALSE(dos.r.aceptada);
  EXPECT_NE(dos.r.motivo.find("PRESUPUESTO"), std::string::npos);
  Recogida uno;
  EXPECT_TRUE(b.encolar({leer(3, 126, 21)}, uno.fin()));
  EXPECT_TRUE(p.llamadas.empty());
}

TEST(BusBrazo, AtenderRespetaElPresupuestoPorCiclo) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  Recogida a, c, d;
  b.encolar({leer(3, 132, 4)}, a.fin());                    // ~5 ms cada una
  b.encolar({leer(3, 132, 4)}, c.fin());
  b.encolar({leer(3, 132, 4)}, d.fin());
  b.atender(12.0);                                          // 5 + 5 caben, la tercera (15) no
  EXPECT_EQ(a.veces + c.veces + d.veces, 2);
  EXPECT_EQ(b.en_cola(), 1u);
  b.atender(12.0);
  EXPECT_EQ(d.veces, 1);
}

TEST(BusBrazo, SiempreEjecutaAlMenosUnaPorCiclo) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  Recogida a;
  b.encolar({leer(3, 126, 21)}, a.fin());                   // ~8.5 ms
  b.atender(1.0);                                           // presupuesto menor que la llamada
  EXPECT_EQ(a.veces, 1);
}

TEST(BusBrazo, ColaLlena) {
  PuertoFalso p;
  ConfigBrazo cfg;
  cfg.max_cola = 2;
  BusBrazo b(&p, cfg);
  Recogida a, c, d;
  EXPECT_TRUE(b.encolar({leer(3, 70, 1)}, a.fin()));
  EXPECT_TRUE(b.encolar({leer(3, 70, 1)}, c.fin()));
  EXPECT_FALSE(b.encolar({leer(3, 70, 1)}, d.fin()));
  EXPECT_EQ(d.veces, 1);
  EXPECT_FALSE(d.r.aceptada);
  EXPECT_EQ(d.r.motivo, "cola llena");
  EXPECT_EQ(b.diagnostico().rechazadas, 1u);
}

TEST(BusBrazo, CerrarVaciaLaColaYRechazaLoNuevo) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  Recogida a, c, tarde;
  b.encolar({leer(3, 70, 1)}, a.fin());
  b.encolar({leer(3, 70, 1)}, c.fin());
  b.cerrar();
  EXPECT_EQ(a.veces, 1);
  EXPECT_EQ(c.veces, 1);
  EXPECT_FALSE(a.r.aceptada);
  EXPECT_EQ(a.r.motivo, "bus apagado");
  EXPECT_FALSE(b.encolar({leer(3, 70, 1)}, tarde.fin()));
  EXPECT_EQ(tarde.r.motivo, "bus apagado");
  b.atender(12.0);
  EXPECT_TRUE(p.llamadas.empty());
}

TEST(BusBrazo, DiagnosticoCuenta) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  Recogida a;
  b.encolar({leer(3, 70, 1)}, a.fin());
  b.atender(12.0);
  const auto d = b.diagnostico();
  EXPECT_EQ(d.transacciones, 1u);
  EXPECT_GT(d.tiempo_bus_ms_total, 0.0);
  EXPECT_EQ(d.en_cola, 0u);
}

// Varios hilos encolan mientras otro atiende: cada llamada termina exactamente una vez y el puerto solo lo toca
// el hilo que atiende.
TEST(BusBrazo, VariosHilosEncolanYSoloUnoTocaElPuerto) {
  PuertoFalso p;
  ConfigBrazo cfg;
  cfg.max_cola = 1000;
  BusBrazo b(&p, cfg);
  constexpr int HILOS = 4, POR_HILO = 200;
  std::atomic<int> terminadas{0};
  std::atomic<bool> fin{false};
  std::thread motor([&] {
    while (!fin) b.atender(12.0);
    b.atender(1e9);
  });
  std::vector<std::thread> clientes;
  for (int h = 0; h < HILOS; ++h) {
    clientes.emplace_back([&] {
      for (int i = 0; i < POR_HILO; ++i) b.encolar({leer(3, 70, 1)}, [&](RespuestaDxl&&) { ++terminadas; });
    });
  }
  for (auto& t : clientes) t.join();
  while (terminadas < HILOS * POR_HILO) std::this_thread::yield();
  fin = true;
  motor.join();
  EXPECT_EQ(terminadas.load(), HILOS * POR_HILO);
  EXPECT_EQ(p.hilos.size(), 1u);
  EXPECT_EQ(p.llamadas.size(), static_cast<size_t>(HILOS * POR_HILO));
}

TEST(BusBrazo, CerrarMientrasOtrosEncolanNoDejaNadieColgado) {
  PuertoFalso p;
  ConfigBrazo cfg;
  cfg.max_cola = 100000;
  BusBrazo b(&p, cfg);
  std::atomic<int> enviadas{0}, terminadas{0};
  std::vector<std::thread> clientes;
  for (int h = 0; h < 4; ++h) {
    clientes.emplace_back([&] {
      for (int i = 0; i < 500; ++i) {
        ++enviadas;
        b.encolar({leer(3, 70, 1)}, [&](RespuestaDxl&&) { ++terminadas; });
      }
    });
  }
  b.cerrar();
  for (auto& t : clientes) t.join();
  b.cerrar();
  EXPECT_EQ(enviadas.load(), terminadas.load());            // ninguna llamada se pierde
}

TEST(BusBrazo, PingYModelo) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  Recogida rc;
  b.encolar({{TipoOp::Ping, 5, 0, 0, 0}}, rc.fin());
  b.atender(12.0);
  EXPECT_EQ(rc.r.resultados[0].valor, 1030u);
}

TEST(BusBrazo, RebootDeUnIdPermitidoConLaAlertaEnElEstado) {
  PuertoFalso p;
  BusBrazo b(&p, {});
  Recogida rc;
  b.encolar({{TipoOp::Reboot, 5, 0, 0, 0}}, rc.fin());
  b.atender(12.0);
  ASSERT_EQ(p.llamadas.size(), 1u);
  EXPECT_EQ(p.llamadas[0].tipo, 'b');
  EXPECT_EQ(rc.r.resultados[0].estado, Estado::Ok);         // solo la alerta: no es error del motor
  Recogida otro;
  b.encolar({{TipoOp::Reboot, 9, 0, 0, 0}}, otro.fin());    // ID fuera de la lista
  b.atender(12.0);
  EXPECT_EQ(otro.r.resultados[0].estado, Estado::Rechazada);
  EXPECT_EQ(p.llamadas.size(), 1u);
}
