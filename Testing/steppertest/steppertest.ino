/**
 * steppertest.ino
 * 
 * Basic program for testing stepper motor functionality
*/

#define ENABLE_PIN 5
#define PULSE 6
#define DIRECTION 7
#define HLFB 8

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(PULSE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(HLFB, INPUT);

  digitalWrite(ENABLE_PIN, HIGH);

  Serial.println("Balls");
}

void loop() {
  // put your main code here, to run repeatedly:

}
