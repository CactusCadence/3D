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
const int SIZE = 10;

//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
SPISettings thermocouple(4300000, MSBFIRST, SPI_MODE0);

// Helper functions
//Sliiiiiiiiiiiiide to the left
// <--<--<--
// [1, 2, 3] -> [2, 3, 0]
void SlideLeft(float* arr, unsigned int size)
{
  memmove(arr, arr + 1, sizeof(float) * size);
  arr[size - 1] = 0.0;

  return;
}

//Sliiiiiiiiiiiiide to the right
// -->-->-->
// [1, 2, 3] -> [0, 1, 2]
void SlideRight(float* arr, unsigned int size)
{
  memmove(arr + 1, arr, sizeof(float) * size);
  arr[0] = 0.0;

  return;
}

// (...chris-cross...?)
void Prepend(float val, float* arr, unsigned int size)
{
  SlideRight(arr, size);
  arr[0] = val;

  return;
}

// print the contents of a given array
void printArray(float* arr, unsigned int size)
{
  Serial.print("[");
  for(uint i = 0; i < size-1; ++i)
  {
    Serial.print(arr[i]);
    Serial.print(", ");    
  }
  Serial.print(arr[size-1]);

  Serial.print("]\n");

  return;
}

void setup() {
  Serial.begin(9600);
  SPI.begin();

  pinMode(thermocoupleCS, OUTPUT);
  pinMode(heaterRelay, OUTPUT);

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

  if(result < 0.0) {
    HeaterON();
    Serial.println("ON");
  } else {
    HeaterOFF();
    Serial.println("OFF");
  }

  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(DELAY);
}

// points
float desiredPoint = 260.0; // set point
float measuredPoint; // plant measurement
float error = -9999;  // set to some absurd default value

// gains
float kProportional = 1.0;  // proportional gain
float kIntegral = 1.0;    // integral gain
float kDerivative = 1.0;  // derivative gain

// storage variables
float measurements[SIZE] = {69,69,69,69,4,3,3,2};
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
  Prepend(measuredPoint, measurements, SIZE);
  lastError = error;
  error = measuredPoint - desiredPoint;
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
  printArray(measurements, SIZE);

  Serial.println();

  return resultProportional;
}
