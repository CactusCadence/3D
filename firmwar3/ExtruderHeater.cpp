/*
    ExtruderHeater.cpp

    Implementation file for ExtruderHeater class
    Intended to read and control the extruder heater of the MDPH2,
    by reading a Type K thermocouple from the MAX6675.
*/

#include "ExtruderHeater.h"

#define NOOP 0x00

/**
 * Default Constructor
*/
ExtruderHeater::ExtruderHeater()
    : DELAY(250), BUFFER_SIZE(12)
{
    thermocouple.speedMaximum = 4300000;
    thermocouple.dataOrder = MSBFIRST;
    thermocouple.dataMode = SPI_MODE0;

    setHeater(9);
    setCS(10);
}

void ExtruderHeater::setHeater(unsigned int pin) {
    heaterRelay = pin;
    pinMode(heaterRelay, OUTPUT);

    return;
}

void ExtruderHeater::setCS(unsigned int pin) {
    thermocoupleCS = pin;
    pinMode(thermocoupleCS, OUTPUT);

    return;
}
