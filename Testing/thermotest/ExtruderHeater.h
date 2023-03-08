/*
    ExtruderHeater.h

    Include file for Extruder Heater class
    Intended to read and control the extruder heater of the MDPH2,
    by reading a Type K thermocouple from the MAX6675.
*/

#ifndef INCLUDED_EXTRUDER_HEATER_H
#define INCLUDED_EXTRUDER_HEATER_H

#include <SPI.h>

class ExtruderHeater {

public:
  // Default Constructor
  ExtruderHeater();
  // Pass default target temperature
  ExtruderHeater(double);

  void setTarget(double);
  void setHeater(unsigned int);
  void setCS(unsigned int);

private:
  void HeaterOn();
  void HeaterOff();

  double target;

  SPISettings thermocouple;

  unsigned int heaterRelay;
  unsigned int thermocoupleCS;

  const unsigned int DELAY;
  const unsigned int BUFFER_SIZE;
};

#endif  //INCLUDED_EXTRUDER_HEATER_H
