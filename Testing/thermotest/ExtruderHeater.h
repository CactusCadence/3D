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

  double getTemperature();

  void Update();

private:
  bool checkTime();

  void heaterOn();
  void heaterOff();

  bool readThermocouple();
  bool isThermocoupleDataValid(uint16_t);
  bool isAmplifierConnected(uint16_t);
  bool isThermocoupleConnected(uint16_t);

  void UpdatePID();
  bool clamp(double&);
  bool isPIDIOEqual(double, double);

  SPISettings thermocouple;

  unsigned int heaterRelay;
  unsigned int thermocoupleCS;

  static const unsigned int BUFFER_SIZE = 12;
  const unsigned long DELAY = 250;
  const double deltaT = DELAY / 1000.0;
  unsigned long lastTime;

  // PID Characteristics
  double pidResult;

  // points
  double desiredPoint; // set point
  double measuredPoint; // plant measurement

  // gains
  const double kProportional = 0.69;  // proportional gain
  const double kIntegral = 0.04;    // integral gain
  const double kDerivative = 3.2;  // derivative gain

  // clamping limits
  const double MIN = -0.5;
  const double MAX = 0.5;
  bool isClamped;

  // storage variables
  double measurements[BUFFER_SIZE];
  double integral = 0.0;
  double derivative = 0.0;

  double resultProportional;
  double resultIntegral;
  double resultDerivative;

  double output;
};

#endif  //INCLUDED_EXTRUDER_HEATER_H
