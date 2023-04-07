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
    : lastTime(0),
    pidResult(0.0), desiredPoint(260.0),
    measuredPoint(0.0), isClamped(false),
    integral(0.0), derivative(0.0)
{
    thermocouple = SPISettings(4300000, MSBFIRST, SPI_MODE0);

    setHeater(9);
    setCS(10);

    SPI.begin();
}

/**
 * Pass default target temperature
 * 
 * @param[in] newTarget Target temperature in degrees C
*/
ExtruderHeater::ExtruderHeater(double newTarget)
    : lastTime(0),
    pidResult(0.0), desiredPoint(newTarget),
    measuredPoint(0.0), isClamped(false),
    integral(0.0), derivative(0.0)
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

    desiredPoint = newTarget;

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
    
    return measuredPoint;
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
          UpdatePID();

          if(pidResult < 0.0) {
            heaterOn();
          } else {
            heaterOff();
          }

        } else {

            // turn off the heater if thermocouple data is invalid
            heaterOff();
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

    measuredPoint = (byteBuffer >> 3) *  0.25;

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
    if(!isAmplifierConnected(buffer)) {

      Serial.println("Amp not connected");
      return false;
    }
    else if (!isThermocoupleConnected(buffer)) {

      Serial.println("Thermocouple not connected");
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
 * Update the PID controller
 */
void ExtruderHeater::UpdatePID() {

  // calculate the error
  Prepend(measuredPoint, measurements, BUFFER_SIZE);
  double error = measuredPoint - desiredPoint;

  // recalculate proportional
  resultProportional = error * kProportional;

  // recalculate integral
  if(!isClamped) {
    integral += error;
    resultIntegral = integral * kIntegral;
  }

  // recalculate derivative
  if(measurements[BUFFER_SIZE-1] != 0.0) {
    double sum = 0.0;
    const int AVG = 3;

    // take an average of AVG error points
    for(uint i = 1; i <= AVG; ++i)
    {
      double lastError = measurements[BUFFER_SIZE-i] - desiredPoint;

      sum += (error - lastError) / (deltaT * BUFFER_SIZE);      
    }
    derivative = sum / AVG;
  }
  resultDerivative = derivative * kDerivative;

  pidResult = resultProportional + resultIntegral + resultDerivative;

  isClamped = isPIDIOEqual(error, pidResult) & clamp(pidResult);

  return;
}

/**
 * Clamp the final result from the PID controller.
 *
 * @param[in,out] sum The result from the PID controller
 * @returns Whether or not the result was clamped.
 * @retval 1 Result was clamped
 * @retval 0 Result was NOT clamped
 */
bool ExtruderHeater::clamp(double& sum) {
  if(sum > MAX) {
    sum = MAX;
  } else if (sum < MIN) {
    sum = MIN;
  } else {
    return false;
  }

  return true;
}

/**
 * Check whether the signs on the Error and Sum are the same
 * 
 * @param[in] error The error input into the PID controller
 * @param[in] sum The (unclamped) result from the PID controller.
 */
bool ExtruderHeater::isPIDIOEqual(double error, double sum) {
  return sgn(error) == sgn(sum);
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
