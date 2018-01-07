/**
   Leakage Detector w/ HTU21d temp humidity sensor

 *******************************
    HTU21D 3V3V ONLY  --  https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library
    xxx
    Node
   
*/

#define SKETCH_NAME "Leakage Detector Temp Humi Sensor"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "1.21"

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
#include <Wire.h>
#include "SparkFunHTU21D.h"

// Child declarations
// wichtig für fhem https://fhem.de/commandref.html#MYSENSORS_DEVICE
#define CHILD_ID_GENERAL  0
#define CHILD_ID_REED     1
#define CHILD_ID_TEMP     2
#define CHILD_ID_HUM      3
#define CHILD_ID_VOLTAGE  4
#define CHILD_ID_TEMPERATURE 6

// pin definitions
#define DIGITAL_INPUT_SENSOR   2  // Arduino Digital I/O pin for button/reed switch
#define DIGITAL_OUTPUT_BUZZER  8

// constant definitions
#define MESSAGEWAIT  500
#define NORMALONTIME 5
#define BLIPINTERVAL 75
//#define REPORTINTERVAL 82800000 // every 23 hours
//#define REPORTINTERVAL 43200000 // every 12 hours
//#define REPORTINTERVAL 3600000 // every 1 hour
//#define REPORTINTERVAL 1800000 // every 30 minutes
//#define REPORTINTERVAL 15000 // every 15 seconds
#define REPORTINTERVAL 600000 // every 10 minutes

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

int16_t oldVoltage  = -1;
int16_t oldHtuTemp  = -1;
int16_t oldHtuHumi  = -1;
int16_t oldValue    = -1;
boolean buzzerState = false;      
int SleepResult     = -1;
int measureCount    = 0;

const float VccMin   = 1.9;           // Minimum expected Vcc level, in Volts.
const float VccMax   = 3.3;           // Maximum expected Vcc level, in Volts.
const float VccCorrection = 3.0 / 3.0 ; // Measured Vcc by multimeter divided by reported Vcc, use this to calibrate
float BatteryVoltage = 0.0;

Vcc vcc(VccCorrection);

// https://github.com/mysensors/MySensors/blob/development/core/MyMessage.h
MyMessage msgGeneral(CHILD_ID_GENERAL, V_VAR1);
MyMessage msgReed(CHILD_ID_REED, V_TRIPPED);
MyMessage msgHtuTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHtuHum(CHILD_ID_HUM, V_HUM);
MyMessage msgVoltage(CHILD_ID_VOLTAGE, V_VOLTAGE);
MyMessage msgTemperature(CHILD_ID_TEMPERATURE, V_TEMP);

//Create a instances of the objects
HTU21D myHumidity;

////////////////////////////////////////////////////////
void setup(){
  // If you want to set the aref to something other than 5v
  // analogReference(EXTERNAL); // ++ 10nF zwischen AREF und GND ???
  
  // Setup the button
  pinMode(DIGITAL_INPUT_SENSOR, INPUT);
  // Activate internal pull-up
  // digitalWrite(DIGITAL_INPUT_SENSOR, HIGH);

  buzzerState = false; 

  // HTU21D
  myHumidity.begin();
  // can be left ...
  byte reg = myHumidity.readUserRegister();
 
}

////////////////////////////////////////////////////////
void presentation() {

  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);
  sleep(MESSAGEWAIT); // only needed when using Domoticz and direct connected NRF24 - needs some time between messages

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_GENERAL, S_ARDUINO_NODE, SKETCH_NAME); 
  present(CHILD_ID_REED, S_DOOR, "LeakageDetector D2");
  present(CHILD_ID_HUM, S_HUM, "HTU21D humidity I2C");
  present(CHILD_ID_TEMP, S_TEMP, "HTU21D temperature I2C");
  present(CHILD_ID_VOLTAGE, S_MULTIMETER, "Batteriespannung (V)");  
  present(CHILD_ID_TEMPERATURE, S_TEMP, "Chip temp"); 
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
  return 1126400L / readMUX(_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1));  
}

float readTemp() {
  // Read 1.1V reference against MUX3  
  return (readMUX(_BV(REFS1) | _BV(REFS0) | _BV(MUX3)) - 125) * 0.1075; 
}

void sendHTU21D(){

  uint8_t nNoUpdatesTemp;
  uint8_t nNoUpdatesHum;

  // Get temperature from DHT library
  float temperature = myHumidity.readTemperature();
  if (isnan(temperature)) {
//    Serial.println("Failed reading temperature from DHT!");
  } else if (temperature != oldHtuTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {
    // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
    oldHtuTemp = temperature;
    // Reset no updates counter
    nNoUpdatesTemp = 0;
    temperature += SENSOR_TEMP_OFFSET;
    send(msgHtuTemp.set(temperature, 1));
  } else {
    // Increase no update counter if the temperature stayed the same
    nNoUpdatesTemp++;
  }

  // Get humidity from DHT library
  float humidity = myHumidity.readHumidity();
  if (isnan(humidity)) {
//    Serial.println("Failed reading humidity from DHT");
  } else if (humidity != oldHtuHumi || nNoUpdatesHum == FORCE_UPDATE_N_READS) {
    // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
    oldHtuHumi = humidity;
    // Reset no updates counter
    nNoUpdatesHum = 0;
    send(msgHtuHum.set(humidity, 1));
  } else {
    // Increase no update counter if the humidity stayed the same
    nNoUpdatesHum++;
  }
}

void loop(){

  //wdt_enable(WDTO_8S);  // https://forum.mysensors.org/post/67574

  // Read digital motion value
  bool tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH;
  
  if (buzzerState == true){
    tone(DIGITAL_OUTPUT_BUZZER, 2000, 1000); // pin, frequency, duration
    delay(500);        
  } else {
    noTone(DIGITAL_OUTPUT_BUZZER);
  }

  if (SleepResult == -1) {
    
    sendHTU21D();
    // we get here if the processor wakes up after the reporting interval (so not when a switch is open/closed).
    float BatteryVoltage = vcc.Read_Volts();
    send(msgVoltage.set(BatteryVoltage, 2));
    send(msgTemperature.set(readTemp(), 1));
    sendHeartbeat();
    
  } else {

    // read the switch, it should be settled by now after sending the battery voltage
    if (tripped != oldValue) {
      // Send in the new value
      send(msgReed.set(tripped?"1":"0"));  // Send tripped value to gw
      oldValue = tripped;
      buzzerState = tripped ? true: false;

      sendHTU21D();
      // we get here if one of the swicthes was opened or closed.
      float BatteryVoltage = vcc.Read_Volts();
      send(msgVoltage.set(BatteryVoltage, 2)); // , decimals);
      send(msgTemperature.set(readTemp(), 1));
    }  
  }

  //wdt_disable();     // https://forum.mysensors.org/post/67574

  // Sleep until something happens with the switches, or wakeup after interval
  // https://www.mysensors.org/download/sensor_api_20#sleeping
  SleepResult = smartSleep(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), CHANGE, REPORTINTERVAL);
}


