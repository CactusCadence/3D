#include <ArduinoJson.h>

// Note that we can't use the internal LED outside of this example because it is linked to CLK
#define LEDPIN 13
unsigned long ledOnTime = 0;
const unsigned long ONTIME = 50;

// amount of material extruded per revolution (mm)
const float MATERIAL_PER_REV = 17.4625;

// Keep this at 800 according to the MDPH2 guide
const unsigned int INPUT_RESOLUTION = 800;

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
float materialToExtrude;
float commandedMaterialLength;
float lastMaterialLength;
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
      distance = cmd["distance"];
      commandedMaterialLength = cmd["materialLength"];
      float newSpeed = cmd["newSpeed"];

      Serial.printf("%s: %f\n", "distance", distance);
      Serial.printf("%s: %f\n", "materialLength", commandedMaterialLength);
      Serial.printf("%s: %f\n", "newSpeed", newSpeed);

      if(commandedMaterialLength != 0.0)
      {
        materialToExtrude = commandedMaterialLength - lastMaterialLength;
        lastMaterialLength = commandedMaterialLength;
      }
      else
      {
        materialToExtrude = commandedMaterialLength;
        lastMaterialLength = commandedMaterialLength;
      }

      if(newSpeed != -1.0)
      {
        speed = newSpeed;
      }

      Serial.println();

      calculateExtrusionInstruction();
    }
  }

}

void calculateExtrusionInstruction() {

  float instructionTime = distance / speed;
  float numberRevolutions = materialToExtrude / MATERIAL_PER_REV;
  float revolutionRate = numberRevolutions / instructionTime;

  float analogFrequency = revolutionRate * INPUT_RESOLUTION;
  Serial.printf("Analog Freq: %f\n", analogFrequency);
}

void UpdateLED()
{
  if(ledOnTime + ONTIME < millis())
  {
    digitalWrite(LEDPIN, LOW);
  }
}