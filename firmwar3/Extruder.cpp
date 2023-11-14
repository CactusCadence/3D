#include "Extruder.hpp"

Extruder::Extruder() {

}

Extruder::Extruder(uint8_t hlfb, uint8_t pulse, uint8_t direction, uint8_t enable)
: HLFB(hlfb), PULSE(pulse), DIRECTION(direction), ENABLE(enable) {

  pinMode(HLFB, INPUT);
  pinMode(PULSE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(ENABLE, OUTPUT);
}