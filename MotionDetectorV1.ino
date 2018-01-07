/*********************************

   DESCRIPTION

   Connect button or REED/window reed switch between
   digitial I/O pin 2  and GND.

   http://www.mysensors.org/build/binary

*/

#define SKETCH_NAME "Motion Sensor (I2C)"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"

//#define MY_NODE_ID 13 (uncomment this if you want to control the NODE_ID yourself
//#define MY_PARENT_ID 0
#define NODE_ID auto

// Enable and select radio type attached
#define MY_RADIO_NRF24

// possible values: RF24_PA_LOW (is default on gateway), RF24_PA_MED, RF24_PA_HIGH, RF24_PA_MAX (is default on nodes)
#define RF24_PA_LEVEL RF24_PA_MAX

// Enable OTA fireware update feature
// Angeblich bei MYSBootloader nicht notwendig, da OTA da nur offline möglich ist
#define MY_OTA_FIRMWARE_FEATURE

#include <SPI.h>
#include <MySensors.h>
#include <Vcc.h>
//#include <Wire.h>

// Child declarations
#define CHILD_ID_GENERAL  0
#define CHILD_ID_PIR     1
#define CHILD_ID_VOLTAGE  2
#define CHILD_ID_POWER    3
#define CHILD_ID_TEMPERATURE 4

// pin definitions
#define DIGITAL_INPUT_SENSOR   2  // Arduino Digital I/O pin for button/reed switch
#define LEDPIN     6  // on the very narrow node, this is my only available led.

// constant definitions
#define MESSAGEWAIT  500
#define NORMALONTIME 5
#define BLIPINTERVAL 75
//#define REPORTINTERVAL 82800000 // every 23 hours
//#define REPORTINTERVAL 43200000 // every 12 hours
#define REPORTINTERVAL 3600000 // every 1 hour
//#define REPORTINTERVAL 1800000 // every 30 minutes
//#define REPORTINTERVAL 15000 // every 15 seconds

int oldValue = -1;
int SleepResult = -1;

const float VccMin   = 1.9;           // Minimum expected Vcc level, in Volts.
const float VccMax   = 3.3;           // Maximum expected Vcc level, in Volts.
const float VccCorrection = 3.0 / 3.0 ; // Measured Vcc by multimeter divided by reported Vcc, use this to calibrate
float BatteryVoltage = 0.0;

Vcc vcc(VccCorrection);

MyMessage msgGeneral(CHILD_ID_GENERAL, V_VAR1);
MyMessage msgMotion(CHILD_ID_PIR, V_TRIPPED);
MyMessage msgVoltage(CHILD_ID_VOLTAGE, V_VOLTAGE);
MyMessage msgPower(CHILD_ID_POWER, V_VOLTAGE);
MyMessage msgTemperature(CHILD_ID_TEMPERATURE, V_TEMP);

////////////////////////////////////////////////////////
void setup() {
  // Setup the led
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);

  // Setup the button
  pinMode(DIGITAL_INPUT_SENSOR, INPUT);
  // Activate internal pull-up
  // digitalWrite(DIGITAL_INPUT_SENSOR, HIGH);
}


////////////////////////////////////////////////////////
void BlipLed() {
  digitalWrite(LEDPIN, HIGH);
  wait(NORMALONTIME);
  digitalWrite(LEDPIN, LOW);
}

void presentation() {

  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  present(CHILD_ID_GENERAL, S_ARDUINO_NODE, SKETCH_NAME); 
  present(CHILD_ID_VOLTAGE, S_MULTIMETER, "Battery");
  present(CHILD_ID_PIR, S_MOTION);
  present(CHILD_ID_POWER, S_MULTIMETER, "ADC power", true); 
  present(CHILD_ID_TEMPERATURE, S_TEMP, "Chip temp"); 

  // the next calls to BlipLed are optional (I like blinky lights)
  BlipLed();
  sleep(BLIPINTERVAL);
  BlipLed();
  sleep(BLIPINTERVAL);
}

long readMUX(uint8_t aControl) {
  long result;
  
  ADMUX = aControl;
  delay(20); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  return result;
}

long readVcc() {
  // Read 1.1V reference against AVcc
  return (1126400L / readMUX(_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1))) / 1000;  
}

float readTemp() {
  // Read 1.1V reference against MUX3  
  return (readMUX(_BV(REFS1) | _BV(REFS0) | _BV(MUX3)) - 125) * 0.1075; 
}

void loop() {
  // optional BlipLed :-)
  BlipLed();

  // Read digital motion value
  bool tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH;
  
  if (SleepResult == -1) {
    
    // we get here if the processor wakes up after the reporting interval (so not when a switch is open/closed).
    float BatteryVoltage = vcc.Read_Volts();
    send(msgVoltage.set(BatteryVoltage, 2));
    send(msgTemperature.set(readTemp(), 1));
    send(msgPower.set(readVcc()), 2);
    sendHeartbeat();
    
  } else {
    
    // we get here if one of the swicthes was opened or closed.
    float BatteryVoltage = vcc.Read_Volts();
    send(msgVoltage.set(BatteryVoltage, 2));
    send(msgTemperature.set(readTemp(), 1));
    send(msgPower.set(readVcc()), 2);

    // read the switch, it should be settled by now after sending the battery voltage
    if (tripped != oldValue) {
      // Send in the new value
      send(msgMotion.set(tripped?"1":"0"));  // Send tripped value to gw
      oldValue = tripped;
      BlipLed();
    }
  }
  
  // Sleep until interrupt comes in on motion sensor. Send update every ### minute.
  // Sleep until something happens with the switches, or wakeup after interval
  SleepResult = sleep(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), CHANGE, REPORTINTERVAL);

}





