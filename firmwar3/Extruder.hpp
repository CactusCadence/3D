#ifndef EXTRUDER_HPP
#define EXTRUDER_HPP

class Extruder {
  
  public:
    Extruder();
    Extruder(uint8_t, uint8_t, uint8_t, uint8_t);

  private:
    // Pin Definitions
    const uint8_t HLFB;
    const uint8_t PULSE;
    const uint8_t DIRECTION;
    const uint8_t ENABLE;
}

#endif //EXTRUDER_HPP
