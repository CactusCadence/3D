/*
    ExtruderHeater.cpp

    Implementation file for ExtruderHeater class
    Intended to read and control the extruder heater of the MDPH2,
    by reading a Type K thermocouple from the MAX6675.
*/

#include "ExtruderHeater.h"
#include "helpers.h"

#define NOOP 0x00

/**
 * Default Constructor
*/
ExtruderHeater::ExtruderHeater()
    : target(-1.0), BUFFER_SIZE(12), DELAY(250), lastTime(0)
{
    thermocouple = SPISettings(4300000, MSBFIRST, SPI_MODE0);

    setHeater(9);
    setCS(10);

    SPI.begin();
}

/**
 * Pass default target temperature
*/
ExtruderHeater::ExtruderHeater(double newTarget)
    : target(newTarget), BUFFER_SIZE(12), DELAY(250), lastTime(0)
{
    thermocouple = SPISettings(4300000, MSBFIRST, SPI_MODE0);

    setHeater(9);
    setCS(10);

    SPI.begin();
}

/**
 * Set the target temperature in degrees C for the PID
 * 
 * @param[in] newTarget Target temperature in degrees C
*/
void ExtruderHeater::setTarget(double newTarget) {

    target = newTarget;

    return;
}

/**
 * Set the pin that the heater relay is connected to
 * 
 * @param[in] pin The pin number to set
*/
void ExtruderHeater::setHeater(unsigned int pin) {

    heaterRelay = pin;
    pinMode(heaterRelay, OUTPUT);

    return;
}

/**
 * Set the pin that the thermocouple amplifier CS is connected to
 * 
 * @param[in] pin The pin number to set
*/
void ExtruderHeater::setCS(unsigned int pin) {

    thermocoupleCS = pin;
    pinMode(thermocoupleCS, OUTPUT);

    return;
}

/**
 * Called once per loop; read the thermocouple
 * and run the PID controller
*/
void ExtruderHeater::Update() {
    
    // check the elapsed time (non-blocking)
    if(checkTime()) {

        // read the thermocouple
        

        // run the PID controller

    }
}

/**
 * Check the elapsed time since last call. If enough time has
 * passed according to @param DELAY, update the last recorded time
 * 
 * @returns Whether enough time has passed
 * @retval true Enough time has passed; last recorded time was updated
 * @retval false Not enough time has passed
*/
bool ExtruderHeater::checkTime() {

    unsigned long currentTime = millis();
    if(currentTime - DELAY > lastTime) {
        lastTime = currentTime;
        
        return true;
    }

    return false;
}

/**
 * Turn the heater on
*/
void ExtruderHeater::heaterOn() {

    digitalWrite(heaterRelay, LOW);

    return;
}

/**
 * Turn the heater off
*/
void ExtruderHeater::heaterOff() {

    digitalWrite(heaterRelay, HIGH);

    return;
}
