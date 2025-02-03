#include <Arduino.h>

/*
  firmwar3.ino

  Large format 3D printer controller.
  Interfaces robot controller software with the Extruder
*/

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "UDPCommunication.h"
#include <ArduinoJson.h>

// Pin Definitions
#define HLFB 3
#define PULSE 4
#define DIRECTION 5
#define ENABLE_PIN 6
#define LEDPIN 13     //Note: 13 is linked to CLK

#define MAX_PACKET_SIZE 128

unsigned long ledOnTimestamp = 0;
const unsigned long ONTIME = 50;

// Pulses per Revolution for MDPH2
const unsigned int INPUT_RESOLUTION = 800;
// amount of material extruded per revolution (mm)
const float MATERIAL_PER_REV = 17.4625;
// amount of material to be extruded per E value (mm)
const float MATERIAL_PER_E = 32.32;

// Parsed command
StaticJsonDocument<MAX_PACKET_SIZE> cmd;

// Per-Instruction Variables
float materialToExtrude;
float movementDistance;
float movementSpeed;

float lastCommandedMaterialLength = 0.0;

bool isPerformingExtrusion = false;
unsigned long extrusionStartTime = 0.0;
unsigned long extrusionDuration = 0.0;

uint8_t mac[6];
IPAddress ip(10, 0, 0, 111);
unsigned int localPort = 8888;

// buffers for receiving and sending data
char packetBuffer[MAX_PACKET_SIZE];  // buffer to hold incoming packet

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

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

  // Initalize Ethernet/UDP
  SetupUDP();

  Serial.println("Initalization Complete");
}

void loop() {

  UpdateCommand();
  UpdateExtrusionTime();
}

// Reads streams to see if there is any content
// Command is in JSON format
// Format: { distance: %f, materialLength: %f, newSpeed: %f }
void UpdateCommand() {

  String commandString;
  int packetSize = Udp.parsePacket();

  if(packetSize) {
    Udp.read(packetBuffer, MAX_PACKET_SIZE);
    commandString = packetBuffer;
  }
  else if(Serial.available()) {

    commandString = Serial.readStringUntil('\n');
  }

  if(commandString != "")
  {
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

  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(LEDPIN, LOW);

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
    revolutionsPerSecond = 4.0;
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
  digitalWrite(LEDPIN, HIGH);

  extrusionStartTime = millis();
  extrusionDuration = (instructionTime * 1000.0) * 2;
  isPerformingExtrusion = true;
}

void UpdateExtrusionTime() {

  if(isPerformingExtrusion && millis() > extrusionStartTime + extrusionDuration) {
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(LEDPIN, LOW);
    Serial.println("Instruction done");
    isPerformingExtrusion = false;
  }
}

void SetupUDP()
{
  teensyMAC(mac);
  Ethernet.begin(mac, ip);  //Note: this WILL pause the program if no connection exists.

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  Udp.begin(localPort);
  Serial.println("UDP setup complete");
}
