/*

    arduino-cli compile --fqbn MySensors:avr:MYSBL ~/Arduino/mysensors/GartenhausMotionLight/GartenhausMotionLight.ino
    set  MYSENSOR_6 flash
    

  Gartenhaus Control V1

  VERSION 2.30 (June 7 2019)

    Taster      D2,D3           interupts
    ????        D4              ? DHT22 ?
    RCWL        A0              RCWL radar motion sensor 
    HTU21       A4,A5 (I2C)     
                                
    Relay       D5,D6,D7,D8

    I2C:
        SCL <-> A5    weiß
        SDA <-> A4    gelb
  
  FHEM
    setCommands     on:status51:on off:status51:off on52:status52:on off52:status52:off on53:status53:on off53:status53:off on54:status54:on off54:status54:off
    webCmd          on:off:on52:off52:on53:off53:on54:off54


  https://forum.mysensors.org/topic/2106/reinventing-the-motion-controlled-outside-lamp/2

  HTU21:
    SparkFunHTU21D.h

  PIR:
  https://www.mysensors.org/build/motion

  Relay:
  https://github.com/mysensors/MySensorsArduinoExamples/blob/master/examples/RelayWithButtonActuator/RelayWithButtonActuator.ino
  https://forum.fhem.de/index.php?topic=31663.msg244533#msg244533
  https://forum.fhem.de/index.php/topic,31663.msg263514.html#msg263514
  https://forum.mysensors.org/topic/938/multisensor-multiactuator-sketch-testboard-tested-with-fhem-controller
  
  Uptime:
  https://github.com/YiannisBourkelis/Uptime-Library
  https://hackaday.io/project/7008-fly-wars-a-hackers-solution-to-world-hunger/log/25043-updated-uptime-counter
  
*/

// Enable debug prints
// #define MY_DEBUG
//#define MY_SPECIAL_DEBUG
//#define MY_DEBUG_VERBOSE_RF24

//#define MY_DISABLED_SERIAL // if you want to use the UART TX/RX pins as normal I/O pins.

#ifdef MY_DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

#define MY_SIGNAL_REPORT_ENABLED //most likely only needed for nRF-Type nodes to get a pseudo-value - https://forum.mysensors.org/topic/9073/new-2-2-0-signal-report-function
#define MY_SMART_SLEEP_WAIT_DURATION_MS 1000

// Enable and select radio type attached
#define MY_RADIO_RF24
#define RF24_PA_LEVEL RF24_PA_MAX

// https://www.mysensors.org/download/sensor_api_20
#define MY_NODE_ID AUTO

#define MY_SPLASH_SCREEN_DISABLED
#define MY_TRANSPORT_WAIT_READY_MS (30000) // Don't stay awake for more than 30s if communication is broken
//#define MY_TRANSPORT_WAIT_READY_MS 1      // uncomment this to enter the loop() and setup()-function even when the node cannot be registered to gw

// as this node is battery powered, it cannot work as a repeater, so do not load all the code
#define MY_REPEATER_FEATURE

#define RELAIS
#define HTU21
#define RCWL
#define SWITCH

#define SKETCH_NAME "Gartenhaus Control Center"
#define SKETCH_VERSION "2.3.2"
// const char SKETCH_DATE[] = __DATE__ "." __TIME__;

const unsigned long SECOND = 1000UL;
const unsigned long MINUTE = SECOND * 60;
const unsigned long HOUR = MINUTE * 60;

#if !defined(__time_t_defined) // avoid conflict with newlib or other posix libc
typedef unsigned long time_t;
#endif

//----------------------- Relay Configuration -----------------------
#define RELAY_PIN           5   // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS    4         // Total number of attached relays. Must be equal to total number of elements in array below!
const int RELAYS[]          = {5, 6, 7, 8};     // digital pins of attached relays
#define RELAY_ON            1                   // GPIO value to write to turn on attached relay
#define RELAY_OFF           0                   // GPIO value to write to turn off attached relay
bool ack = 1;                                   // set this to 1 if you want destination node to send ack back to this node

////////////////////////////////////////////////////////////////////////
// Child declarations
// wichtig für fhem https://fhem.de/commandref.html#MYSENSORS_DEVICE
// https://tecdox.adcore.de/edit/wiki/iot/mysensors-sensoren
#define CHILD_ID_GENERAL  0
#define CHILD_ID_BTN1     1
#define CHILD_ID_BTN2     2
#define CHILD_ID_TEMP     3
#define CHILD_ID_HUM      4
#define CHILD_ID_RCWL     9
#define CHILD_ID_RELAY    50
#define CHILD_ID_VOLTAGE  101
#define CHILD_ID_POWER    102
#define CHILD_ID_CPUTEMP  103
#define CHILD_ID_MEMORY   104
#define CHILD_ID_CPUFREQ  105
#define CHILD_ID_HBFREQ   196   // Heartbeat Frequency
#define CHILD_ID_UPTIME   197
#define CHILD_ID_ERR      198
#define CHILD_ID_TEXT     199

////////////////////////////////////////////////////////////////////////
// includes
#include <MySensors.h>
#include "TimerClass.h"

#ifdef HTU21
#include <Wire.h>
#include "SparkFunHTU21D.h"
//Create a instances of the objects
HTU21D myHumidity;
int16_t oldHtuTemp        = -1;
int16_t oldHtuHumi        = -1;
// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0
#endif

#ifdef SWITCH
#include <Bounce2.h>
#endif

#ifdef MY_DEBUG
uint32_t interval_millis              = 1 * 60 * 1000UL; // every 1 minutes // unsigned long ms
#else
uint32_t interval_millis              = 15 * 60 * 1000UL; // every 15 minutes // unsigned long ms
#endif
TimerClass sensorTimerInterval        = TimerClass(interval_millis);
////////////////////////////////////////////////////////////////////////
// uptime
volatile unsigned int millisOverflow = 0;
long Day=0;
int Hour =0;
int Minute=0;
int Second=0;
int HighMillis=0;
int Rollover=0;

////////////////////////////////////////////////////////////////////////
// pin definitions
#define DIGITAL_INPUT_SENSOR_2      2   // Arduino Digital irq I/O pin for button/reed switch (Only 2 and 3 generates interrupt!)
#define DIGITAL_INPUT_SENSOR_3      3   // Arduino Digital irq I/O pin for button/reed switch (Only 2 and 3 generates interrupt!)
#define DIGITAL_INPUT_SENSOR_RCWL   A0

int16_t SendDelay                 = 200; //500 ms

#ifdef SWITCH
Bounce debouncer2  = Bounce();
boolean btn2State                = LOW; 
Bounce debouncer3  = Bounce();
boolean btn3State                = LOW; 
#endif

// /////// /////////////////////////////////////////////////////////////////////
// Initialize message types
// https://www.mysensors.org/download/serial_api_20#variable-types
// https://github.com/mysensors/MySensors/blob/development/core/MyMessage.h
MyMessage msgGeneral(CHILD_ID_GENERAL, V_VAR1);

#ifdef SWITCH
MyMessage msgButton1(CHILD_ID_BTN1, V_TRIPPED);
MyMessage msgButton2(CHILD_ID_BTN2, V_TRIPPED);
#endif

#ifdef RCWL
MyMessage msgMotion(CHILD_ID_RCWL, V_TRIPPED);
boolean oldValueRcwl            = LOW;
#endif

#ifdef HTU21
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
#endif

MyMessage msgPower(CHILD_ID_POWER, V_VOLTAGE);
MyMessage msgTemperature(CHILD_ID_CPUTEMP, V_TEMP);
MyMessage msgMemory(CHILD_ID_MEMORY, V_TEXT);
MyMessage msgHbFreq(CHILD_ID_HBFREQ, V_TEXT);
MyMessage msgUptime(CHILD_ID_UPTIME, V_TEXT);
MyMessage msgError(CHILD_ID_ERR, V_TEXT);

#ifdef RELAIS
MyMessage msgRelay; // Initialize relay message
#endif


void presentation() {
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo(SKETCH_NAME, __DATE__"-"__TIME__); wait(200);

#ifdef SWITCH
    present(CHILD_ID_BTN1, S_DOOR, "push button 1");
    present(CHILD_ID_BTN2, S_DOOR, "push button 2");
#endif
  
#ifdef RCWL
  present(CHILD_ID_RCWL, S_MOTION, "motion");
#endif

#ifdef HTU21
  present(CHILD_ID_TEMP, S_TEMP, "temperature (°C)");
  present(CHILD_ID_HUM, S_HUM, "humidity (%)");
#endif

#ifdef RELAIS
    // Register all sensors to gw (they will be created as child devices)
    for (int sensor = 1; sensor <= NUMBER_OF_RELAYS; sensor++) {
        present(CHILD_ID_RELAY+sensor, S_BINARY, "relay", ack);
    }
#endif

   present(CHILD_ID_POWER, S_MULTIMETER, "adc power", true);
   present(CHILD_ID_CPUTEMP, S_TEMP, "chip temp");
   present(CHILD_ID_MEMORY, S_INFO, "chip memory");
   present(CHILD_ID_CPUFREQ, S_INFO, "chip frequency");
   present(CHILD_ID_HBFREQ, S_INFO, "heartbeat frequency");
   present(CHILD_ID_UPTIME, S_INFO, "uptime");
   present(CHILD_ID_ERR, S_INFO, "error messages");
   present(CHILD_ID_TEXT, S_INFO, "relay interval");
}

void before() {
#ifdef HTU21
#endif    
}

void setup() {

#ifdef MY_DEBUG
    Serial.begin(115200);
    while (!Serial) {} // Wait
#endif
    
    // If you want to set the aref to something other than 5v
    analogReference(EXTERNAL); // ++ 10nF zwischen AREF und GND ???

#ifdef SWITCH
    // Use the internal pullup to be able to hook up this sketch directly to a button/switch/reed/...
    // If no pullup is used, the reported usage will be too high because of the floating pin
    pinMode(DIGITAL_INPUT_SENSOR_2, INPUT);//, INPUT_PULLUP);
    pinMode(DIGITAL_INPUT_SENSOR_3, INPUT);//, INPUT_PULLUP);
    // Activate internal pull-up (optional)
    digitalWrite(DIGITAL_INPUT_SENSOR_2, HIGH);
    digitalWrite(DIGITAL_INPUT_SENSOR_3, HIGH);
    // After setting up the button, setup debouncer
    debouncer2.attach(DIGITAL_INPUT_SENSOR_2);
    debouncer2.interval(50);

    debouncer3.attach(DIGITAL_INPUT_SENSOR_3);
    debouncer3.interval(50);
#endif

#ifdef RCWL
    pinMode(DIGITAL_INPUT_SENSOR_RCWL, INPUT_PULLUP);
    // Activate internal pull-up
    // digitalWrite(DIGITAL_INPUT_SENSOR_RCWL, HIGH);
#endif

#ifdef RELAIS
    int i;
    for (int sensor = 1, i = 0; sensor <= NUMBER_OF_RELAYS; sensor++, i++) {
        // set relay pins to output mode
        pinMode(RELAYS[i], OUTPUT);
        // power off when starting
        digitalWrite(RELAYS[i], RELAY_OFF);
        // otherwise restore relay to last known state (using eeprom storage)
//         digitalWrite(RELAYS[i], loadState(sensor) ? RELAY_ON : RELAY_OFF);
    }
#endif

#ifdef HTU21
    myHumidity.begin();
    // can be left ...
    byte reg = myHumidity.readUserRegister();
#endif

    send(msgError.set(""));
    sendStatusMessage();
}


// https://arduino.stackexchange.com/questions/12587/how-can-i-handle-the-millis-rollover
uint64_t millis64() {
    static uint32_t low32, high32;
    uint32_t new_low32 = millis();
    if (new_low32 < low32) high32++;
    low32 = new_low32;
    return (uint64_t) high32 << 32 | low32;
}

int free() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

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

long readVcc() {
    // Read 1.1V reference against AVcc
    return 1126400L / readMUX(_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1));
}

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

///////////////////////////////////////////////////////
void receive(const MyMessage &message) {

    // Handle incoming relay commands
    if (message.type == V_STATUS) {
        // Change relay state
        if (RELAYS[message.sensor - CHILD_ID_RELAY - 1]) {
            digitalWrite(RELAYS[message.sensor  - CHILD_ID_RELAY - 1], message.getBool() ? RELAY_ON : RELAY_OFF);
            // Store state in eeprom
            saveState(message.sensor - CHILD_ID_RELAY - 1, message.getBool());
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
            sensorTimerInterval.setInterval(msgVal);
        }
    }
}

void sendHTU21D() {
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
    if (temperature < 100) {
      send(msgTemp.set(temperature, 1));
    }
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
    if (humidity < 100) {
      send(msgHum.set(humidity, 1));
    }
  } else {
    // Increase no update counter if the humidity stayed the same
    nNoUpdatesHum++;
  }

}

void sendStatusMessage(){
    sendHeartbeat();                                    wait(SendDelay);
    send(msgTemperature.set(readTemp(), 1));            wait(SendDelay);
    send(msgPower.set(readVcc()/1000.0, 2));            wait(SendDelay);
    send(msgMemory.set(hwFreeMem()));                   wait(SendDelay);
    send(msgHbFreq.set(interval_millis));               wait(SendDelay);
#ifdef HTU21
    sendHTU21D();                                       wait(SendDelay);
#endif
    uptime();
}

////////////////////////////////////////////////////////
void loop() {

    debouncer2.update(); // Update the Bounce instance
    if ( debouncer2.fell() ) {  // Call code if button transitions from HIGH to LOW
        btn2State = !btn2State ; // toggle state
        send(msgButton1.set(btn2State?"1":"0"));  // Send tripped value to gw
    }

    debouncer3.update(); // Update the Bounce instance
    if ( debouncer3.fell() ) {  // Call code if button transitions from HIGH to LOW
        btn3State = !btn3State ; // toggle state
        send(msgButton2.set(btn3State?"1":"0"));  // Send tripped value to gw
    }

    boolean rcwl = digitalRead(DIGITAL_INPUT_SENSOR_RCWL) == HIGH;
    if (rcwl != oldValueRcwl) {
        send(msgMotion.set(rcwl?"1":"0"));  // Send tripped value to gw
        oldValueRcwl = rcwl;
    }

    if (sensorTimerInterval.check() == 1) {
        sendStatusMessage();
    } // end of sensorTimerInterval.check()

}
