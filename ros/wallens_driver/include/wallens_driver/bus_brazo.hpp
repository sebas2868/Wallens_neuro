// Operaciones del brazo (tornillo ID 3 y gripper ID 5) sobre el bus que tiene abierto el driver de las ruedas.
// Sin ROS ni SDK: el puerto entra por la interfaz PuertoDxl (docs/brazo-bus-plan.md).
//
// Hilos: `encolar` se llama desde cualquier hilo (el callback del servicio); `atender` y `cerrar` los llama el hilo de
// motores, que es el único que toca el puerto. Un fallo del brazo se devuelve en el resultado y nunca sale como
// excepción de `atender`, para que no cuente como fallo de las ruedas.
#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace wallens {

enum class TipoOp : uint8_t { Leer = 0, Escribir = 1, Ping = 2, Reboot = 3 };   // Reboot: borra la alerta de hardware

struct OperacionDxl {
  TipoOp tipo = TipoOp::Leer;
  uint8_t id = 0;
  uint16_t direccion = 0;
  uint8_t largo = 0;       // leer: 1..32; escribir: 1, 2 o 4
  uint32_t valor = 0;      // solo escribir
};

enum class Estado : uint8_t { Ok = 0, NoEjecutada = 1, Rechazada = 2, ErrorComunicacion = 3, ErrorMotor = 4 };

struct ResultadoDxl {
  Estado estado = Estado::NoEjecutada;
  int16_t comunicacion = 0;      // código COMM del SDK (0, -1000..-9000)
  uint8_t error_motor = 0;       // byte de error del paquete de estado (bit 7 = alerta de hardware)
  uint32_t valor = 0;            // lectura de 1, 2 o 4 bytes (ping: modelo)
  std::vector<uint8_t> datos;    // lecturas en bloque (más de 4 bytes)
};

struct RespuestaDxl {
  bool aceptada = false;
  std::string motivo;
  std::vector<ResultadoDxl> resultados;
  float tiempo_bus_ms = 0.0f;
};

// Lo que devuelve el puerto: código COMM del SDK (0 = éxito) y byte de error del paquete de estado.
struct RespuestaPuerto {
  int comunicacion = 0;
  uint8_t error = 0;
};

class PuertoDxl {
 public:
  virtual ~PuertoDxl() = default;
  virtual RespuestaPuerto ping(uint8_t id, uint16_t* modelo) = 0;
  virtual RespuestaPuerto reboot(uint8_t id) = 0;
  virtual RespuestaPuerto leer(uint8_t id, uint16_t dir, uint8_t largo, std::vector<uint8_t>* datos) = 0;
  virtual RespuestaPuerto escribir(uint8_t id, uint16_t dir, uint8_t largo, uint32_t valor) = 0;
};

constexpr uint8_t ALERTA_HW_BRAZO = 0x80;
constexpr uint16_t DIR_ID = 7;
constexpr uint16_t DIR_BAUDRATE = 8;

struct ConfigBrazo {
  std::vector<uint8_t> ids{3, 5};
  // Rangos inclusivos de direcciones permitidas: los registros que usa scripts/brazo/brazo.py
  std::vector<std::pair<uint16_t, uint16_t>> rangos{{11, 11},   {38, 39},   {48, 51},   {52, 55},   {64, 64},
                                                     {70, 70},   {98, 98},   {102, 103}, {108, 111}, {112, 115},
                                                     {116, 119}, {126, 146}, {222, 223}};
  int baudrate = 57600;
  double max_ms = 12.0;           // tiempo de bus máximo de una llamada y de lo que el brazo gasta por ciclo
  double retorno_ms = 0.5;        // tiempo de retorno del motor por operación
  size_t max_operaciones = 8;
  size_t max_cola = 8;
};

struct DiagnosticoBrazo {
  uint64_t transacciones = 0;
  uint64_t rechazadas = 0;
  double tiempo_bus_ms_total = 0.0;
  double tiempo_bus_ms_max = 0.0;
  size_t en_cola = 0;
};

class BusBrazo {
 public:
  using Finalizador = std::function<void(RespuestaDxl&&)>;

  BusBrazo(PuertoDxl* puerto, ConfigBrazo cfg) : puerto_(puerto), cfg_(std::move(cfg)) {}

  // Bytes en el cable de una operación (instrucción + respuesta), protocolo 2.0 con CRC.
  static int bytes_operacion(const OperacionDxl& op) {
    switch (op.tipo) {
      case TipoOp::Ping: return 10 + 14;
      case TipoOp::Leer: return 14 + 11 + op.largo;
      case TipoOp::Escribir: return 12 + op.largo + 11;
    }
    return 0;
  }

  // Tiempo de bus estimado de una llamada, en ms (10 bits por byte, como espera_lectura_ del driver).
  double estimar_ms(const std::vector<OperacionDxl>& ops) const {
    double ms = 0.0;
    for (const auto& op : ops) ms += bytes_operacion(op) * 10.0 * 1000.0 / cfg_.baudrate + cfg_.retorno_ms;
    return ms;
  }

  // Valida lo que se puede sin tocar el bus y deja la llamada en la cola. Si no entra, `fin` se llama aquí mismo con
  // aceptada = false. Devuelve si quedó en la cola.
  bool encolar(std::vector<OperacionDxl> ops, Finalizador fin) {
    std::string motivo;
    if (ops.empty()) {
      motivo = "sin operaciones";
    } else if (ops.size() > cfg_.max_operaciones) {
      motivo = "demasiadas operaciones (" + std::to_string(ops.size()) + " > " + std::to_string(cfg_.max_operaciones) + ")";
    } else if (const double ms = estimar_ms(ops); ms > cfg_.max_ms) {
      motivo = "PRESUPUESTO: " + std::to_string(ms) + " ms de bus > " + std::to_string(cfg_.max_ms) + " ms";
    }
    if (motivo.empty()) {
      std::lock_guard<std::mutex> l(mtx_);
      if (cerrado_) {
        motivo = "bus apagado";
      } else if (cola_.size() >= cfg_.max_cola) {
        motivo = "cola llena";
      } else {
        cola_.push_back({std::move(ops), std::move(fin)});
        return true;
      }
    }
    {
      std::lock_guard<std::mutex> l(mtx_);
      ++diag_.rechazadas;
    }
    RespuestaDxl r;
    r.motivo = motivo;
    fin(std::move(r));
    return false;
  }

  // Hilo de motores, tras el ciclo de las ruedas: ejecuta llamadas mientras quepan en `presupuesto_ms` de bus.
  // Con la cola no vacía siempre ejecuta al menos una (cada llamada ya cabe en max_ms). Devuelve los ms usados.
  double atender(double presupuesto_ms) {
    double usado = 0.0;
    for (;;) {
      Pendiente p;
      {
        std::lock_guard<std::mutex> l(mtx_);
        if (cola_.empty()) break;
        if (usado > 0.0 && usado + estimar_ms(cola_.front().ops) > presupuesto_ms) break;
        p = std::move(cola_.front());
        cola_.pop_front();
      }
      const double estimado = estimar_ms(p.ops);
      const auto t0 = std::chrono::steady_clock::now();
      RespuestaDxl r = ejecutar(p.ops);
      r.tiempo_bus_ms = static_cast<float>(
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
      usado += std::max(static_cast<double>(r.tiempo_bus_ms), estimado);   // lo medido nunca baja de lo que cuesta el cable
      {
        std::lock_guard<std::mutex> l(mtx_);
        ++diag_.transacciones;
        diag_.tiempo_bus_ms_total += r.tiempo_bus_ms;
        if (r.tiempo_bus_ms > diag_.tiempo_bus_ms_max) diag_.tiempo_bus_ms_max = r.tiempo_bus_ms;
      }
      p.fin(std::move(r));
    }
    return usado;
  }

  // Cierra la cola: lo pendiente y lo que llegue después se rechaza con "bus apagado". Se llama antes de hacer
  // join del hilo de motores, para que ningún cliente se quede esperando.
  void cerrar() {
    std::deque<Pendiente> pendientes;
    {
      std::lock_guard<std::mutex> l(mtx_);
      cerrado_ = true;
      pendientes.swap(cola_);
      diag_.rechazadas += pendientes.size();
    }
    for (auto& p : pendientes) {
      RespuestaDxl r;
      r.motivo = "bus apagado";
      p.fin(std::move(r));
    }
  }

  DiagnosticoBrazo diagnostico() const {
    std::lock_guard<std::mutex> l(mtx_);
    DiagnosticoBrazo d = diag_;
    d.en_cola = cola_.size();
    return d;
  }

  size_t en_cola() const {
    std::lock_guard<std::mutex> l(mtx_);
    return cola_.size();
  }

  // ¿Puede esta operación llegar al bus? ID de la lista, rango completo dentro de un rango permitido y sin tocar
  // el ID (7) ni el baudrate (8) aunque la configuración los incluyera.
  bool permitida(const OperacionDxl& op) const {
    bool id_ok = false;
    for (uint8_t i : cfg_.ids) id_ok = id_ok || i == op.id;
    if (!id_ok) return false;
    if (op.tipo == TipoOp::Ping || op.tipo == TipoOp::Reboot) return true;
    if (op.largo == 0) return false;
    if (op.tipo == TipoOp::Escribir && op.largo != 1 && op.largo != 2 && op.largo != 4) return false;
    if (op.tipo == TipoOp::Leer && op.largo > 32) return false;
    const uint32_t ini = op.direccion, fin = ini + op.largo - 1;
    if (ini <= DIR_BAUDRATE && fin >= DIR_ID) return false;
    for (const auto& [a, b] : cfg_.rangos) {
      if (ini >= a && fin <= b) return true;
    }
    return false;
  }

 private:
  struct Pendiente {
    std::vector<OperacionDxl> ops;
    Finalizador fin;
  };

  RespuestaDxl ejecutar(const std::vector<OperacionDxl>& ops) noexcept {
    RespuestaDxl r;
    r.aceptada = true;
    r.resultados.assign(ops.size(), ResultadoDxl{});
    for (size_t i = 0; i < ops.size(); ++i) {
      ResultadoDxl& res = r.resultados[i];
      if (!permitida(ops[i])) {
        res.estado = Estado::Rechazada;
        break;
      }
      try {
        ejecutar_una(ops[i], &res);
      } catch (const std::exception& e) {
        res = ResultadoDxl{};
        res.estado = Estado::ErrorComunicacion;
        r.motivo = e.what();
      } catch (...) {
        res = ResultadoDxl{};
        res.estado = Estado::ErrorComunicacion;
        r.motivo = "excepción desconocida";
      }
      if (res.estado != Estado::Ok) break;
    }
    return r;
  }

  // El byte de error solo con la alerta de hardware (bit 7) no corta: el dato es válido y el cliente ve la alerta.
  void ejecutar_una(const OperacionDxl& op, ResultadoDxl* res) {
    RespuestaPuerto rp;
    std::vector<uint8_t> datos;
    uint16_t modelo = 0;
    switch (op.tipo) {
      case TipoOp::Ping: rp = puerto_->ping(op.id, &modelo); break;
      case TipoOp::Reboot: rp = puerto_->reboot(op.id); break;
      case TipoOp::Leer: rp = puerto_->leer(op.id, op.direccion, op.largo, &datos); break;
      case TipoOp::Escribir: rp = puerto_->escribir(op.id, op.direccion, op.largo, op.valor); break;
    }
    res->comunicacion = static_cast<int16_t>(rp.comunicacion);
    res->error_motor = rp.error;
    if (rp.comunicacion != 0) {
      res->estado = Estado::ErrorComunicacion;
      return;
    }
    res->estado = (rp.error & ~ALERTA_HW_BRAZO) != 0 ? Estado::ErrorMotor : Estado::Ok;
    if (op.tipo == TipoOp::Ping) {
      res->valor = modelo;
    } else if (op.tipo == TipoOp::Leer) {
      if (datos.size() < op.largo) {
        res->estado = Estado::ErrorComunicacion;
        return;
      }
      if (op.largo <= 4) {
        for (int i = op.largo - 1; i >= 0; --i) res->valor = (res->valor << 8) | datos[i];   // little endian
      } else {
        res->datos.assign(datos.begin(), datos.begin() + op.largo);
      }
    }
  }

  PuertoDxl* puerto_;
  ConfigBrazo cfg_;
  mutable std::mutex mtx_;
  std::deque<Pendiente> cola_;
  bool cerrado_ = false;
  DiagnosticoBrazo diag_;
};

}  // namespace wallens
