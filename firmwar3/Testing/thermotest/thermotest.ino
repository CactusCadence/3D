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
const int SIZE = 12;

//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
SPISettings thermocouple(4300000, MSBFIRST, SPI_MODE0);

// Helper functions
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

//Sliiiiiiiiiiiiide to the left
// <--<--<--
// [1, 2, 3] -> [2, 3, 0]
void SlideLeft(double* arr, unsigned int size)
{
  memmove(arr, arr + 1, sizeof(double) * size);
  arr[size - 1] = 0.0;

  return;
}

//Sliiiiiiiiiiiiide to the right
// -->-->-->
// [1, 2, 3] -> [0, 1, 2]
void SlideRight(double* arr, unsigned int size)
{
  memmove(arr + 1, arr, sizeof(double) * size);
  arr[0] = 0.0;

  return;
}

// (...chris-cross...?)
void Prepend(double val, double* arr, unsigned int size)
{
  SlideRight(arr, size);
  arr[0] = val;

  return;
}

// print the contents of a given array
void printArray(double* arr, unsigned int size)
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

  double tempC = (byteBuffer >> 3) *  0.25;

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
double desiredPoint = 260.0; // set point
double measuredPoint; // plant measurement
double error = -9999;  // set to some absurd default value

// gains
double kProportional = 1.0;  // proportional gain
double kIntegral = 1.0;    // integral gain
double kDerivative = 1.0;  // derivative gain

// clamping limits
const double MIN = -500;
const double MAX = 500;
bool isClamped = false;

// storage variables
double measurements[SIZE];
double integral = 0.0;
double derivative = 0.0;
double lastError = -9999;  // set to some absurd default value

double resultProportional;
double resultIntegral;
double resultDerivative;

double output;

double UpdatePID(double newMeasuredPoint) {
  
  // calculate the error
  measuredPoint = newMeasuredPoint;
  Prepend(measuredPoint, measurements, SIZE);
  error = measuredPoint - desiredPoint;
  auto deltaT = DELAY / 1000.0;

  // recalculate proportional
  resultProportional = error * kProportional;

  // recalculate integral
  if(!isClamped) {
    integral += error;
    resultIntegral = integral * kIntegral;
  }

  // recalculate derivative
  if(measurements[SIZE-1] != 0.0) {
    lastError = measurements[SIZE-1] - desiredPoint;

    derivative = (error - lastError) / (deltaT * SIZE);
    Serial.println(deltaT * SIZE);
  }
  resultDerivative = derivative * kDerivative;

  double sum = resultProportional + resultIntegral; //+ resultDerivative;
  Serial.printf("Proportional:%f\n", resultProportional);
  Serial.printf("Integral:%f\n", resultIntegral);
  Serial.printf("Derivative:%f\n", resultDerivative);
  Serial.printf("Set_Point:%f\n", desiredPoint);
  printArray(measurements, SIZE);

  Serial.println();

  isClamped = isPIDIOEqual(error, sum) & clamp(sum);

  return sum;
}

/**
 * Clamp the final result from the PID controller.
 *
 * @param[in,out] sum The result from the PID controller
 * @returns Whether or not the result was clamped.
 * @retval 1 Result was clamped
 * @retval 0 Result was NOT clamped
 */
bool clamp(double& sum) {
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
bool isPIDIOEqual(double error, double sum) {
  return sgn(error) == sgn(sum);
}
