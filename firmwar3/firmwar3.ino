/*
  firmwar3.ino

  Large format 3D printer controller.
  Interfaces robot controller software with the Extruder
*/


#include <ArduinoJson.h>

// Pin Definitions
#define HLFB 5
#define PULSE 6
#define DIRECTION 7
#define ENABLE_PIN 8
#define LEDPIN 13     //Note: 13 is linked to CLK

unsigned long ledOnTimestamp = 0;
const unsigned long ONTIME = 50;

// Pulses per Revolution for MDPH2
const unsigned int INPUT_RESOLUTION = 800;
// amount of material extruded per revolution (mm)
const float MATERIAL_PER_REV = 17.4625;
// amount of material to be extruded per E value (mm)
const float MATERIAL_PER_E = 32.32;

// Parsed command
StaticJsonDocument<128> cmd;

// Per-Instruction Variables
float materialToExtrude;
float movementDistance;
float movementSpeed;

float lastCommandedMaterialLength = 0.0;

bool isPerformingExtrusion = false;
unsigned long extrusionStartTime = 0.0;
unsigned long extrusionDuration = 0.0;

void setup() {
  // Initalize Serial Port for communication
  Serial.begin(9600);

  // Initalize Extruder Pins
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(PULSE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(HLFB, INPUT);

  // Initalize Communication Status Led
  pinMode(LEDPIN, OUTPUT);

  Serial.println("Initalization Complete");
}

void loop() {

  UpdateCommand();
  UpdateExtrusionTime();
}

void UpdateCommand() {
  
  if(Serial.available()) {

    auto commandString = Serial.readStringUntil('\n');
    parseCommand(commandString);
  }
}

void parseCommand(String& commandString) {

  DeserializationError error = deserializeJson(cmd, commandString.c_str());

  if(error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());

    return;
  }

  movementDistance = cmd["distance"];
  float commandedMaterialLength = cmd["materialLength"];

  float newSpeed = cmd["newSpeed"];
  if(newSpeed != -1.0)
  {
    movementSpeed = newSpeed;
  }

  if(commandedMaterialLength != 0.0)
  {
    materialToExtrude = commandedMaterialLength - lastCommandedMaterialLength;
    lastCommandedMaterialLength = commandedMaterialLength;

    if(materialToExtrude != 0.0) {
      commandExtruder();
    }
  }
  else
  {
    materialToExtrude = commandedMaterialLength;
    lastCommandedMaterialLength = commandedMaterialLength;
  }
}

// It's commanding the extruder... but BETTER!
void commandExtruder() {

  // The amount of material needed to be extruded for this instruction
  float instructionMaterial = abs(materialToExtrude) * MATERIAL_PER_E;
  // Number of revolutions required to extrude the material
  float numberRevolutions = instructionMaterial / MATERIAL_PER_REV;
  // The amount of time it will take the extruder/arm to complete the instruction
  float instructionTime;
  // The extrusion rate to print properly (in rev/s)
  float revolutionsPerSecond;

  // Calculate based on the instruction type (moving or stationary)
  if(movementDistance > 0.1) {
    // the instruction is a moving instruction
    instructionTime = movementDistance / movementSpeed;
    revolutionsPerSecond = numberRevolutions / instructionTime;
  } else {
    // the instruction can happen at an arbitrary set speed
    revolutionsPerSecond = 2.0;
    instructionTime = numberRevolutions / revolutionsPerSecond;
  }

  // Analog frequency to achieve the given rev/s
  float analogFrequency = revolutionsPerSecond * INPUT_RESOLUTION;

  Serial.println();
  Serial.println(instructionMaterial);
  Serial.println(numberRevolutions);
  Serial.println(instructionTime);
  Serial.println(revolutionsPerSecond);
  Serial.println(analogFrequency);
  Serial.println();

  if(materialToExtrude > 0.0) {
    // CW; push material
    digitalWrite(DIRECTION, HIGH);
  } else {
    // CCW; pull material
    digitalWrite(DIRECTION, LOW);
  }

  analogWriteFrequency(PULSE, abs(analogFrequency));
  analogWrite(PULSE, 128);

  digitalWrite(ENABLE_PIN, HIGH);

  extrusionStartTime = millis();
  extrusionDuration = instructionTime * 1000.0;
  isPerformingExtrusion = true;
}

void UpdateExtrusionTime() {

  if(isPerformingExtrusion && millis() > extrusionStartTime + extrusionDuration) {
    digitalWrite(ENABLE_PIN, LOW);
    Serial.println("Instruction done");
    isPerformingExtrusion = false;
  }
}
