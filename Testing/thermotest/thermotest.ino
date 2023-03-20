// this example is public domain. enjoy!
// https://learn.adafruit.com/thermocouple/

#include <SPI.h>
#include "ExtruderHeater.h"

ExtruderHeater heater;

#define DELAY 250

void setup() {
  Serial.begin(9600);

  // wait for MAX chip to stabilize
  delay(DELAY);
}

void loop() {
  heater.Update();

  double tempC = heater.getTemperature();

  Serial.printf("Actual:%f\n", tempC);

  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(DELAY);
}
