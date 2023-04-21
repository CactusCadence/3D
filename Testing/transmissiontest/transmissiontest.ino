// Note that we can't use the internal LED outside of this example because it is linked to CLK
#define LEDPIN 13
unsigned long ledOnTime = 0;
const unsigned long ONTIME = 50;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);
}

void loop() {
  UpdateLED();

  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    digitalWrite(LEDPIN, HIGH);
    ledOnTime = millis();
    char c = Serial.read();
    Serial.print(c);
  }
}

void UpdateLED()
{
  if(ledOnTime + ONTIME < millis())
  {
    digitalWrite(LEDPIN, LOW);
  }
}