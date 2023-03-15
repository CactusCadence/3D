/*
    ExtruderHeater.cpp

    Implementation file for ExtruderHeater class
    Intended to read and control the extruder heater of the MDPH2,
    by reading a Type K thermocouple from the MAX6675.
*/

#include "ExtruderHeater.h"
#include "helpers.h"

#define NO_OP 0x00

const uint16_t INPUTMASK = 0b100;

/**
 * Default Constructor
*/
ExtruderHeater::ExtruderHeater()
    : temperature(0.0), target(-1.0), BUFFER_SIZE(12), DELAY(250), lastTime(0)
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
 * @returns The last read temperature of the thermocouple
*/
double ExtruderHeater::getTemperature() {
    
    return temperature;
}

/**
 * Called once per loop; read the thermocouple
 * and run the PID controller
*/
void ExtruderHeater::Update() {
    
    // check the elapsed time (non-blocking)
    if(checkTime()) {

        if(readThermocouple()) {
            
            // run the PID controller

        }
    }
}

/**
 * Read from the thermocouple amplifier connected to the SPI bus.
 * 
 * @returns Whether or not the thermocouple read successfully.
 * @retval true The thermocouple was read successfully.
 * @retval false The thermocouple was not read successfully.
 * (Either because the MAX was disconnected, or thermocouple is disconnected)
*/
bool ExtruderHeater::readThermocouple() {

    // basic readout test, just print the current temp
    uint16_t byteBuffer = 0x0000;

    //drive CS low & start a clock signal at 4.3MHz
    digitalWrite(thermocoupleCS, LOW);
    SPI.beginTransaction(thermocouple);

    byteBuffer = SPI.transfer16(NO_OP);

    SPI.endTransaction();
    digitalWrite(thermocoupleCS, HIGH);

    if(!isThermocoupleDataValid(byteBuffer)) {
        return false;
    }

    temperature = (byteBuffer >> 3) *  0.25;

    return true;
}

/**
 * Verify incoming data from the SPI bus
 * 
 * @param [in] buffer Byte buffer from the SPI bus
 * 
 * @returns Whether or not the data is valid
 * @retval true Data is valid
 * @retval false Data is invalid (either amp is not connected, or thermocouple is not connected)
*/
bool ExtruderHeater::isThermocoupleDataValid(uint16_t buffer) {

    // Is the MAX6675 connected?
    return isAmplifierConnected(buffer) || isThermocoupleConnected(buffer);

    if(buffer == 0) {
        Serial.println("MAX6675 not connected.");
        return false;
    }

    // Is a thermocouple connected to MAX6675?
    uint16_t result = buffer & INPUTMASK;
    if(result > 0) {
        Serial.println("Thermocouple not connected to MAX6675");
        return false;
    }

    return true;
}

/**
 * Determine whether or not the amplifier is connected
 * 
 * @param [in] buffer Byte buffer from the SPI bus
 * 
 * @returns Whether or not amp is connected
 * @retval true Amp is connected
 * @retval false Amp is not connected
*/
bool ExtruderHeater::isAmplifierConnected(uint16_t buffer) {

    return buffer != 0;
}

/**
 * Determine whether or not a thermocouple is connected to the amp
 * 
 * @param [in] buffer Byte buffer from the SPI bus
 * 
 * @returns Whether or not a thermocouple is connected
 * @retval true Thermocouple is connected
 * @retval false Thermocouple is not connected
*/
bool ExtruderHeater::isThermocoupleConnected(uint16_t buffer) {
    
    uint16_t result = buffer & INPUTMASK;

    return result != 0b100;
}

/**
 * Check the elapsed time since last call. If enough time has
 * passed according to DELAY, update the last recorded time
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
