/**
 * steppertest.ino
 * 
 * Basic program for testing stepper motor functionality
*/

#define HLFB 5
#define PULSE 6
#define DIRECTION 7
#define ENABLE_PIN 8

// Keep this at 800 according to the MDPH2 guide
const unsigned int INPUT_RESOLUTION = 800;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(PULSE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(HLFB, INPUT);

  digitalWrite(ENABLE_PIN, HIGH);

  // digitalWrite(PULSE, HIGH);

  // in order to change the frequency, have to change the frequency THEN analogWrite.
  // might be able to analog write at the 50% duty cycle & change frequency when applicable?
  analogWriteFrequency(PULSE, INPUT_RESOLUTION * 1);
  analogWrite(PULSE, 128);

  Serial.println("Initalization Complete");
}

void loop() {
  // put your main code here, to run repeatedly:

}

