#ifndef INCLUDED_EXTRUDER_HEATER_H
#define INCLUDED_EXTRUDER_HEATER_H

class ExtruderHeater = {
  private:
    uint8_t heaterPin;
    uint8_t DO, CS, SCK;

  public:
    /**
    * Default Constructor (with default arguments)
    * @param{in} _heaterPin Digital pin to control the heater state
    * @param{in} _SDA SDA pin for thermocouple control
    * @param{in} _SCL SCL pin for thermocouple control
    */
    ExtruderHeater(uint8_t _heaterPin = 9, );



};



//Default Configuration
#ifndef EXTRUDER_HEATER_PIN
#define EXTRUDER_HEATER_PIN 9
#endif

/**
* Set up all of the extruder heater concerns
*/
void setupExtruderHeater()
{
  pinMode(EXTRUDER_HEATER_PIN, OUTPUT);
}





#endif  //INCLUDED_EXTRUDER_HEATER_H
