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

  void Update();

private:
  bool checkTime();

  void heaterOn();
  void heaterOff();

  double target;

  SPISettings thermocouple;

  unsigned int heaterRelay;
  unsigned int thermocoupleCS;

  const unsigned int BUFFER_SIZE;
  const unsigned long DELAY;
  unsigned long lastTime;
};

#endif  //INCLUDED_EXTRUDER_HEATER_H
