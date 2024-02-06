/*
  firmwar3.ino

  Large format 3D printer controller.
  Interfaces robot controller software with the Extruder
*/

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "UDPCommunication.h"

#include <ArduinoJson.h>

#include <SPI.h>
#include <SD.h>

// Pin Definitions

// Extruder Pins
#define HLFB 3
#define PULSE 4
#define DIRECTION 5
#define ENABLE_PIN 6

// Other
#define LEDPIN 13     //Note: 13 is linked to CLK

#define MAX_PACKET_SIZE 128

unsigned long ledOnTimestamp = 0;
const unsigned long ONTIME = 50;

// Pulses per Revolution for MDPH2
const unsigned int INPUT_RESOLUTION = 800;
// amount of material extruded per revolution (mm)
const float MATERIAL_PER_REV = 17.4625;
// amount of material to be extruded per E value (mm)
const float MATERIAL_PER_E = 1.83669400925;

// Parsed command
StaticJsonDocument<MAX_PACKET_SIZE> cmd;

float lastCommandedMaterialLength = 0.0;

bool isPerformingExtrusion = false;
unsigned long extrusionStartTime = 0.0;
unsigned long extrusionDuration = 0.0;

unsigned long extrusionIndex = 0;

uint8_t mac[6];
IPAddress ip(10, 0, 0, 111);
unsigned int localPort = 8888;

// buffers for receiving and sending data
char packetBuffer[MAX_PACKET_SIZE];  // buffer to hold incoming packet

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// Sd Card Object
Sd2Card card;
File extrusionInstructions;

struct {
  bool toExtrude = false;
  bool isBypass = false;
  float movementSpeed;
  float extrusionValue;
} ExtrusionInstruction;

float CHECK_BYPASS = -9999.0;

void setup() {
  // Initalize Serial Port for communication
  Serial.begin(9600);

  Serial.println("- Initalization Begin -\n");

  // Initalize Extruder Pins
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(PULSE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(HLFB, INPUT);

  // Initalize Communication Status Led
  pinMode(LEDPIN, OUTPUT);

  // Initalize Ethernet/UDP
  SetupUDP();

  // Initialize SD Card
  SdVolume volume;
  if(!card.init(SPI_HALF_SPEED, BUILTIN_SDCARD)) {
    Serial.println("SD could not be found.");
  } else {
    Serial.println("SD initialized.");
    volume.init(card);

    // print the type and size of the first FAT-type volume
    Serial.print("Volume type is:    FAT");
    Serial.println(volume.fatType(), DEC);

    Serial.print("Volume size (Gb):  ");

    uint32_t volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= volume.clusterCount();       // we'll have a lot of clusters
    volumesize /= 2048;
    Serial.println((float)volumesize / 1024.0);

    if(!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("SD card failed to initialize.");
    }

    extrusionInstructions = SD.open("instructions.txt", FILE_READ);
  }

  Serial.println("\n- Initalization Complete -");
}

void loop() {
  UpdateCommand(cmd);
  UpdateExtruder();
 
}

/**
 * Sets up a UDP connection
 *
 * Precondition: An ethernet cable is plugged into the microcontroller.
 *               (Note: the function will freeze if no cable is plugged in)
 *
 * Postcondition: The Udp object is initalized and can be used to send/receive messages from
 *                another device.
 */
void SetupUDP()
{
  teensyMAC(mac);
  Ethernet.begin(mac, ip);  //Note: this WILL pause the program if no connection exists.
  Serial.print("Ethernet found\n");

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

/**
 * Reads input stream options to see if any content is available.
 *
 * Precondition: At least one communication stream is open and available.
 *               The text is sent in JSON format reflecting this structure:
 *               { distance: %f, materialLength: %f, newSpeed: %f }
 *
 * Postcondition: The cmd JSON object is populated with the content from the stream.
 */
void UpdateCommand(JsonDocument& udpCommand) {

  DeserializationError error = DeserializationError::Ok;
  int packetSize = Udp.parsePacket();

  if(packetSize) {
    // Read command from UDP and deserialize it
    auto readSize = Udp.read(packetBuffer, MAX_PACKET_SIZE);
    error = deserializeJson(udpCommand, packetBuffer);

    // Read the command from SD card and deserialize it
    auto instructionString = extrusionInstructions.readStringUntil('\n');
    StaticJsonDocument<MAX_PACKET_SIZE> sdCommand;
    error = deserializeJson(sdCommand, instructionString.c_str());

    // Ensure that the two commands are the same
    if (udpCommand["extrude"] == sdCommand["extrude"] || udpCommand["extrude"] == CHECK_BYPASS) {
      // Serial.printf("Extruding at: %f\n", sdCommand["extrude"]);
      ExtrusionInstruction.extrusionValue = sdCommand["extrude"];
      ExtrusionInstruction.movementSpeed = sdCommand["speed"];
      ExtrusionInstruction.toExtrude = true;

      if (udpCommand["extrude"] == CHECK_BYPASS) {
        ExtrusionInstruction.isBypass = true;
      }
    } else {
      Serial.println("UDP and SD command mismatch!");
      Serial.printf("UDP: %f, SD: %f\n\n", (float)udpCommand["extrude"], (float)sdCommand["extrude"]);
    }
  }
  else if(Serial.available()) {

    error = deserializeJson(udpCommand, Serial.readStringUntil('\n').c_str());
  }

  if(error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());

    return;
  }
}

void UpdateExtruder()
{
  if (ExtrusionInstruction.toExtrude)
  {
    ExtrusionInstruction.toExtrude = false;
    //Serial.println(abs(ExtrusionInstruction.extrusionValue)*MATERIAL_PER_E / ExtrusionInstruction.movementSpeed);
    if (isPerformingExtrusion) {
      Serial.printf("Instruction interrupted, %i milliseconds remaining.\n", (extrusionStartTime + extrusionDuration) - millis());
    }

    commandExtruder();
  }

  UpdateExtrusionTime();
}

// It's commanding the extruder... but BETTER!
void commandExtruder() {

  // The amount of material needed to be extruded for this instruction
  float instructionMaterial = abs(ExtrusionInstruction.extrusionValue) * MATERIAL_PER_E;
  // Number of revolutions required to extrude the material
  float numberRevolutions = instructionMaterial / MATERIAL_PER_REV;
  // The amount of time it will take the extruder/arm to complete the instruction
  float instructionTime;
  // The extrusion rate to print properly (in rev/s)
  float revolutionsPerSecond;

  // Calculate based on the instruction type (moving or stationary)
  if(instructionMaterial > 0.1) {
    // the instruction is a moving instruction
    instructionTime = instructionMaterial / abs(ExtrusionInstruction.movementSpeed);
    revolutionsPerSecond = numberRevolutions / instructionTime;
  } else {
    // the instruction can happen at an arbitrary set speed
    revolutionsPerSecond = 2.0;
    instructionTime = numberRevolutions / revolutionsPerSecond;
  }

  if (ExtrusionInstruction.isBypass) {
    instructionTime = 9999;
  }

  // Analog frequency to achieve the given rev/s
  float analogFrequency = revolutionsPerSecond * INPUT_RESOLUTION;

  // Serial.println();
  // Serial.println(instructionMaterial);
  // Serial.println(numberRevolutions);
  // Serial.println(instructionTime);
  // Serial.println(revolutionsPerSecond);
  // Serial.println(analogFrequency);
  // Serial.println();

  Serial.printf("DEBUG\nE: %f\nRevs: %f\nRevs/s: %f\nTime: %f\nAnalogFreq: %f\n\n",
                instructionMaterial,
                numberRevolutions,
                revolutionsPerSecond,
                instructionTime,
                analogFrequency);

  if(instructionMaterial > 0.0) {
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
  extrusionDuration = (instructionTime * 1000.0);
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
