//#include "extruder_heater.h"
#include <SPI.h>
#include <MAX6675.h>

#define HEATER_PIN 9

int thermoDO = 12;
int thermoCS = 10;
int thermoCLK = 13;

MAX6675 thermocouple;

void setup() {
  // put your setup code here, to run once:
  //setupExtruderHeater();
  SPI.begin();

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(thermoCS, OUTPUT);

  thermocouple.begin(thermoCS);

  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("HIGH");
  digitalWrite(HEATER_PIN, HIGH);
  delay(500);

  Serial.println("LOW");
  digitalWrite(HEATER_PIN, LOW);
  delay(500);

  Serial.println(thermocouple.readCelsius());
}

