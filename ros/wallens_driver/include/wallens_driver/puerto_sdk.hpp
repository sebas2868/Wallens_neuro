// Adaptador de PuertoDxl al Dynamixel SDK (el puerto y el PacketHandler que ya tiene abiertos el driver).
#pragma once

#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "wallens_driver/bus_brazo.hpp"

namespace wallens {

class PuertoSdk : public PuertoDxl {
 public:
  PuertoSdk(dynamixel::PortHandler* port, dynamixel::PacketHandler* ph) : port_(port), ph_(ph) {}

  RespuestaPuerto ping(uint8_t id, uint16_t* modelo) override {
    uint8_t err = 0;
    const int res = ph_->ping(port_, id, modelo, &err);
    return {res, err};
  }
  RespuestaPuerto leer(uint8_t id, uint16_t dir, uint8_t largo, std::vector<uint8_t>* datos) override {
    datos->assign(largo, 0);
    uint8_t err = 0;
    const int res = ph_->readTxRx(port_, id, dir, largo, datos->data(), &err);
    return {res, err};
  }
  RespuestaPuerto escribir(uint8_t id, uint16_t dir, uint8_t largo, uint32_t valor) override {
    uint8_t err = 0;
    int res = COMM_NOT_AVAILABLE;
    switch (largo) {
      case 1: res = ph_->write1ByteTxRx(port_, id, dir, static_cast<uint8_t>(valor), &err); break;
      case 2: res = ph_->write2ByteTxRx(port_, id, dir, static_cast<uint16_t>(valor), &err); break;
      case 4: res = ph_->write4ByteTxRx(port_, id, dir, valor, &err); break;
    }
    return {res, err};
  }

 private:
  dynamixel::PortHandler* port_;
  dynamixel::PacketHandler* ph_;
};

}  // namespace wallens
