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

// Parsed command
StaticJsonDocument<128> cmd;

// Per-Instruction Variables
float materialToExtrude;
float movementDistance;
float movementSpeed;

float lastCommandedMaterialLength = 0.0;

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

  digitalWrite(ENABLE_PIN, HIGH);

  Serial.println("Initalization Complete");
}

void loop() {

  UpdateCommand();
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
  }
  else {
    movementDistance = cmd["distance"];
    float commandedMaterialLength = cmd["materialLength"];
    
    if(commandedMaterialLength != 0.0)
    {
      materialToExtrude = commandedMaterialLength - lastCommandedMaterialLength;
      lastCommandedMaterialLength = commandedMaterialLength;
    }
    else
    {
      materialToExtrude = commandedMaterialLength;
      lastCommandedMaterialLength = commandedMaterialLength;
    }

    float newSpeed = cmd["newSpeed"];
    if(newSpeed != -1.0)
    {
      movementSpeed = newSpeed;
    }

    commandExtruder();
  }
}

void commandExtruder() {

  Serial.println(movementDistance);
  Serial.println(movementSpeed);
  Serial.println(materialToExtrude);

  float instructionTime = movementDistance / movementSpeed;
  float numberRevolutions = materialToExtrude; //* MATERIAL_PER_REV;
  float revolutionRate = numberRevolutions; /// instructionTime;

  float analogFrequency = revolutionRate * INPUT_RESOLUTION;

  // Clockwise
  if(analogFrequency > 0.0) {
    digitalWrite(DIRECTION, HIGH);
  }
  // Counterclockwise
  else {
    digitalWrite(DIRECTION, LOW);
  }

  Serial.printf("Raw Analog Freq: %f\n", analogFrequency);

  if(abs(analogFrequency) > 4000.0) {
    digitalWrite(LEDPIN, HIGH);
    analogFrequency = 0.0;
  }
  //analogFrequency = min(abs(analogFrequency), 4000.0);

  Serial.printf("Analog Freq: %f\n", analogFrequency);

  analogWriteFrequency(PULSE, abs(analogFrequency));
  analogWrite(PULSE, 128);
}
