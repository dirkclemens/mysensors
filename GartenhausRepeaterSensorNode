/**


  I2C:
  SCL <-> A5    gelb
  SDA <-> A4    weiß

*/

// Enable debug
#define MY_DEBUG
#define MY_SPECIAL_DEBUG
//#define MY_DEBUG_VERBOSE_RF24

#ifdef MY_DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif


#define MY_SIGNAL_REPORT_ENABLED //most likely only needed for nRF-Type nodes to get a pseudo-value - https://forum.mysensors.org/topic/9073/new-2-2-0-signal-report-function
#define MY_SMART_SLEEP_WAIT_DURATION_MS 1000


#define SKETCH_NAME "Gartenhaus Repeater Node"
#define SKETCH_VERSION "2.2.3c 02.05.19"

#define MY_NODE_ID 89

// Enable and select radio type attached
#define MY_RADIO_RF24

#define MY_RF24_PA_LEVEL RF24_PA_MAX

// Enabled repeater feature for this node
#define MY_REPEATER_FEATURE

//#define MY_TRANSPORT_WAIT_READY_MS (30000) // Don't stay awake for more than 30s if communication is broken
//#define MY_TRANSPORT_UPLINK_CHECK_DISABLED  //Don't check for uplink
//#define MY_PARENT_NODE_ID  1
//#define MY_PARENT_NODE_IS_STATIC

// Set blinking period (in milliseconds)
//#define MY_DEFAULT_LED_BLINK_PERIOD 300

// if needed !!!
//#define MY_WITH_LEDS_BLINKING_INVERSE

////////////////////////////////////////////////////////////////////////
// pin definitions LED müssen vor dem include definiert werden !!!!
// Each LED is connected by its anode (long leg) to +5V. 
// The cathode (short leg) is connected through a resistor to one of the 
// following digital pins of the Arduino:
#define MY_DEFAULT_TX_LED_PIN 5
#define MY_DEFAULT_RX_LED_PIN 6   // min setup !
#define MY_DEFAULT_ERR_LED_PIN 7

////////////////////////////////////////////////////////////////////////
// includes
#include <SPI.h>
#include <MySensors.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>

#if !defined(__time_t_defined) // avoid conflict with newlib or other posix libc
typedef unsigned long time_t;
#endif

////////////////////////////////////////////////////////////////////////
// Child declarations
// wichtig für fhem https://fhem.de/commandref.html#MYSENSORS_DEVICE
// https://tecdox.adcore.de/edit/wiki/iot/mysensors-sensoren
#define CHILD_ID_GENERAL  0
#define CHILD_ID_TEMP     3
#define CHILD_ID_HUM      4
#define CHILD_ID_POWER    102
#define CHILD_ID_CPUTEMP  103
#define CHILD_ID_MEMORY   104
#define CHILD_ID_HBFREQ   196   // Heartbeat Frequency
#define CHILD_ID_UPTIME   197
#define CHILD_ID_ERR      198
#define CHILD_ID_TEXT     199

////////////////////////////////////////////////////////////////////////
// constant definitions
#define SECS_PER_MIN  ((time_t)(60UL))
#define SECS_PER_HOUR ((time_t)(3600UL))
#define SECS_PER_DAY  ((time_t)(SECS_PER_HOUR * 24UL))
const unsigned long SECOND = 1000UL;
const unsigned long MINUTE = SECOND * 60;

int16_t SendDelay         = 500; //500 ms
int16_t oldValue          = -1;

////////////////////////////////////////////////////////////////////////
// instantiate classes

MyMessage msgGeneral(CHILD_ID_GENERAL, V_VAR1);
MyMessage msgHtuTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHtuHum(CHILD_ID_HUM, V_HUM);
MyMessage msgPower(CHILD_ID_POWER, V_VOLTAGE);
MyMessage msgTemperature(CHILD_ID_CPUTEMP, V_TEMP);
MyMessage msgMemory(CHILD_ID_MEMORY, V_TEXT);
MyMessage msgHbFreq(CHILD_ID_HBFREQ, V_TEXT);
MyMessage msgUptime(CHILD_ID_UPTIME, V_TEXT);
MyMessage msgError(CHILD_ID_ERR, V_TEXT);

//Create a instances of the objects
HTU21D myHumidity;
// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

////////////////////////////////////////////////////////
//  timer for recurrent broadcasting of information
unsigned long previous_millis = 0;
unsigned long interval_millis = 1*60*1000UL; // every 1 minutes // unsigned long ms

uint8_t checkTimer(){
  unsigned long now = millis();
  if ( interval_millis == 0 ){ previous_millis = now; return 1; }
  if ( (now - previous_millis) >= interval_millis) { previous_millis += interval_millis ; return 1; }  
  return 0;
}
////////////////////////////////////////////////////////////////////////
// uptime
volatile unsigned int millisOverflow = 0;
long Day=0;
int Hour =0;
int Minute=0;
int Second=0;
int HighMillis=0;
int Rollover=0;


////////////////////////////////////////////////////////
void before() {
/*
  interval_millis = loadState(0);
  // wenn wert < 5 Sek ODER > 60 Min dann setzen DEFAULT setzen 
  if ((interval_millis < SECOND * 5 ) || (interval_millis > MINUTE * 60)){
    interval_millis = SECOND * 30;
  }
*/  
}

////////////////////////////////////////////////////////
void setup() {
    // HTU21D
    myHumidity.begin();
    // can be left ...
    byte reg = myHumidity.readUserRegister();

    send(msgError.set(""));
    sendStatusMessage();
}

////////////////////////////////////////////////////////
void presentation() {
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo(SKETCH_NAME, __DATE__"-"__TIME__); wait(200);

    present(CHILD_ID_GENERAL, S_ARDUINO_NODE, SKETCH_NAME);
    present(CHILD_ID_HUM, S_HUM, "HTU21D humidity I2C");
    present(CHILD_ID_TEMP, S_TEMP, "HTU21D temperature I2C");  
    present(CHILD_ID_CPUTEMP, S_TEMP, "Chip_temp");
    present(CHILD_ID_HBFREQ, S_INFO, "heartbeat frequency");
    present(CHILD_ID_UPTIME, S_INFO, "uptime");
    present(CHILD_ID_ERR, S_INFO, "error messages");
    present(CHILD_ID_TEXT, S_INFO, "relay interval");
}

////////////////////////////////////////////////////////
int free() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

////////////////////////////////////////////////////////
long readMUX(uint8_t aControl) {
    long result;
    ADMUX = aControl;
    delay(20); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA, ADSC));
    result = ADCL;
    result |= ADCH << 8;
    return result;
}

////////////////////////////////////////////////////////
long readVcc() {
    // Read 1.1V reference against AVcc
    return 1126400L / readMUX(_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1));
}

////////////////////////////////////////////////////////
float readTemp() {
    // Read 1.1V reference against MUX3
    return (readMUX(_BV(REFS1) | _BV(REFS0) | _BV(MUX3)) - 125) * 0.1075;
}

/*
 * Function made to millis() be an optional parameter
 * https://forum.arduino.cc/index.php?topic=71212.0
 */
char *uptime() { 
    return (char *)uptime(millis()); // call original uptime function with unsigned long millis() value
}
static unsigned int  _rolloverCount   = 0;    // Number of 0xFFFFFFFF rollover we had in millis()
static unsigned long _lastMillis      = 0;    // Value of the last millis() 
static unsigned long _incMillis      = 0;    // Value of the last millis() 
char *uptime(unsigned long milli){
    // If we had a rollover we count that.
    if (milli < _lastMillis) {
        _rolloverCount++;
        _incMillis++;
    }
    // Now store the current number of milliseconds for the next round.
    _lastMillis = milli;    
    // Based on the current milliseconds and the number of rollovers
    // we had in total we calculate here the uptime in seconds since 
    // poweron or reset.
    // Caution: Because we shorten millis to seconds we may miss one 
    // second for every rollover (1 second every 50 days).
    unsigned long secs = (0xFFFFFFFF / 1000 ) * _rolloverCount + (_lastMillis / 1000)  + _incMillis;  
    
    static char _result[24];
    //unsigned long secs = milli / 1000, 
    unsigned long mins = secs / 60;
    unsigned int hours = mins / 60, days = hours / 24 ;
    milli -= secs * 1000;
    secs -= mins * 60;
    mins -= hours * 60;
    hours -= days * 24;
    sprintf(_result, "%d days %2.2d:%2.2d:%2.2d", (int)days, (byte)hours, (byte)mins, (byte)secs);
    send(msgUptime.set(_result));
    return _result;
}

////////////////////////////////////////////////////////
void sendHTU21DSimple(){

  // Get temperature from DHT library
  float temperature = myHumidity.readTemperature();
  if (! isnan(temperature)) {
    temperature += SENSOR_TEMP_OFFSET;
    send(msgHtuTemp.set(temperature, 1));
  }
  
  // Get humidity from DHT library
  float humidity = myHumidity.readHumidity();
  if (! isnan(humidity)) {
    send(msgHtuHum.set(humidity, 1));
  }
}

void sendStatusMessage(){
    sendHeartbeat();                                    wait(SendDelay);
    send(msgTemperature.set(readTemp(), 1));            wait(SendDelay);
    send(msgPower.set(readVcc()/1000.0, 2));            wait(SendDelay);
    send(msgMemory.set(hwFreeMem()));                   wait(SendDelay);
    send(msgHbFreq.set(interval_millis));               wait(SendDelay);
    uptime();
}

////////////////////////////////////////////////////////
void receive(const MyMessage &message){
    // We only expect one type of message from controller. But we better check anyway.
    //  V_TEXT          = 47, //!< S_INFO. Text message to display on LCD or controller device
    if (message.type==V_TEXT) {
        // Change interval_millis
        unsigned long msgVal = message.getULong();
        // wenn wert > 5 Sek UND < 60 Min dann setzen
        if ((msgVal > SECOND * 5) && (msgVal < MINUTE * 60)) {
          interval_millis = msgVal;
          //saveState(0, msgVal);
          previous_millis = millis();
        }
    }

    if (message.type == V_TEXT && message.sensor == CHILD_ID_HBFREQ) {
        DEBUG_PRINT("receive: V_TEXT -> ");
        DEBUG_PRINTLN(message.getULong());
        // Change interval_millis
        unsigned long msgVal = message.getULong();
        // wenn wert > 5 Sek UND < 60 Min dann setzen
        if ((msgVal > SECOND * 5) && (msgVal < MINUTE * 60)) {
            interval_millis = msgVal;
            previous_millis = millis();
        }
    }
}

////////////////////////////////////////////////////////
void loop() {

  if (checkTimer() == 1) {
    sendStatusMessage();
    sendHTU21DSimple();                       //wait(SendDelay);
  }
}
