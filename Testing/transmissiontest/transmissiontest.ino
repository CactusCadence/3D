#include <ArduinoJson.h>

// Note that we can't use the internal LED outside of this example because it is linked to CLK
#define LEDPIN 13
unsigned long ledOnTime = 0;
const unsigned long ONTIME = 50;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);
}

StaticJsonDocument<128> cmd;
String commandString = "";

// Commanded distance to travel
float distance;
// Commanded length of material to extrude
float materialLength;
// Robot Movement Speed
float speed;

void loop() {
  UpdateLED();

  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    digitalWrite(LEDPIN, HIGH);
    ledOnTime = millis();
    commandString = Serial.readStringUntil('\n');
    Serial.print("New Command String: ");
    Serial.println(commandString);

    DeserializationError error = deserializeJson(cmd, commandString.c_str());
    if(error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }
    else
    {
      Serial.println("New Command Variables:");
      float distance = cmd["distance"];
      float materialLength = cmd["materialLength"];
      float newSpeed = cmd["newSpeed"];

      Serial.printf("%s: %f\n", "distance", distance);
      Serial.printf("%s: %f\n", "materialLength", materialLength);
      Serial.printf("%s: %f\n", "newSpeed", newSpeed);

      Serial.println();
    }
  }

}

void UpdateLED()
{
  if(ledOnTime + ONTIME < millis())
  {
    digitalWrite(LEDPIN, LOW);
  }
}