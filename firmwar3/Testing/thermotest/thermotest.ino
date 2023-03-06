// this example is public domain. enjoy!
// https://learn.adafruit.com/thermocouple/

#include <SPI.h>

// MAX6675 Pins
#define thermocoupleDO 12
#define thermocoupleCS 10
#define thermocoupleCLK 13

// Heater Pins
#define heaterRelay 9

#define NO_OP 0x00

const int DELAY = 250;

//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
SPISettings thermocouple(4300000, MSBFIRST, SPI_MODE0);

void setup() {
  Serial.begin(9600);
  SPI.begin();

  pinMode(thermocoupleCS, OUTPUT);

  Serial.println("MAX6675 test");

  // wait for MAX chip to stabilize
  delay(DELAY);
}

void HeaterON() { digitalWrite(heaterRelay, LOW); }
void HeaterOFF() { digitalWrite(heaterRelay, HIGH); }

void loop() {
  // basic readout test, just print the current temp
  uint16_t byteBuffer = 0x0000;

  //drive CS low & start a clock signal at 4.3MHz
  digitalWrite(thermocoupleCS, LOW);
  SPI.beginTransaction(thermocouple);

  byteBuffer = SPI.transfer16(NO_OP);

  SPI.endTransaction();
  digitalWrite(thermocoupleCS, HIGH);

  float tempC = (byteBuffer >> 3) *  0.25;

  Serial.printf("Actual:%f\n",tempC);

  auto result = UpdatePID(tempC);

  if(result > 1.0) {
    HeaterON();
  } else {
    HeaterOFF();
  }

  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(DELAY);
}

// points
float desiredPoint = 60.0; // set point
float measuredPoint; // plant measurement
float error = -9999;  // set to some absurd default value

// gains
float kProportional = 1.0;  // proportional gain
float kIntegral = 1.0;    // integral gain
float kDerivative = 1.0;  // derivative gain

// storage variables
float integral = 0.0;
float derivative = 0.0;
float lastError = -9999;  // set to some absurd default value

float resultProportional;
float resultIntegral;
float resultDerivative;

float output;

float UpdatePID(float newMeasuredPoint) {
  
  // calculate the error
  measuredPoint = newMeasuredPoint;
  lastError = error;
  error = desiredPoint - measuredPoint;
  float deltaT = DELAY / 1000.0;

  // recalculate proportional
  resultProportional = error * kProportional;

  // recalculate integral
  integral += error;
  resultIntegral = integral * kIntegral;

  // recalculate derivative
  if(lastError != -9999) {
    derivative = (error - lastError) / deltaT;
  }
  resultDerivative = derivative * kDerivative;

  //float sum = resultProportional + resultIntegral + resultDerivative;
  Serial.printf("Proportional:%f\n", resultProportional);
  Serial.printf("Integral:%f\n", resultIntegral);
  Serial.printf("Derivative:%f\n", resultDerivative);
  Serial.printf("Set_Point:%f\n", desiredPoint);

  Serial.println();

  return resultProportional;
}
