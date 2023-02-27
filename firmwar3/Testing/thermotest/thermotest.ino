// this example is public domain. enjoy!
// https://learn.adafruit.com/thermocouple/

#include <SPI.h>

int thermoDO = 12;
int thermoCS = 10;
int thermoCLK = 13;

#define NO_OP 0x00

//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
SPISettings thermocouple(4300000, MSBFIRST, SPI_MODE0);

void setup() {
  Serial.begin(9600);
  SPI.begin();

  pinMode(thermoCS, OUTPUT);

  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // basic readout test, just print the current temp
  uint16_t byteBuffer = 0x0000;

  //drive CS low & start a clock signal at 4.3MHz
  digitalWrite(thermoCS, LOW);
  SPI.beginTransaction(thermocouple);

  byteBuffer = SPI.transfer16(NO_OP);

  SPI.endTransaction();
  digitalWrite(thermoCS, HIGH);

  Serial.println((byteBuffer >> 3) *  0.25);

  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(1000);
}