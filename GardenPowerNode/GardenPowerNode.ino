/*
  Garden Multi Power Node V2
  VERSION 2.14

	https://www.mysensors.org/build/connect_radio
  nRF24L01+   D10,D11,D12,D13			--> 3v3
  Relay       D5,D6,D7,D8					--> 5v    
  RFM433      D2									--> 5v
  DS18B20     D4							    --> 5v	  mit 4K7 zw.  D3 und 5v
  Moisture    A1 / A2 						--> 5V
  
  Relay:
  https://forum.fhem.de/index.php?topic=31663.msg244533#msg244533
  https://forum.fhem.de/index.php/topic,31663.msg263514.html#msg263514
  https://forum.mysensors.org/topic/938/multisensor-multiactuator-sketch-testboard-tested-with-fhem-controller

  RFM433: http://funduino.de/nr-03-433mhz-funkverbindung
  Codes der Fernsteuerung 3
  *  Kanal 1 an: 1049937   aus: 1049940
  *  Kanal 2 an: 1053009   aus: 1053012
  *  Kanal 3 an: 1053777   aus: 1053780
  *  Kanal 4 an: 1053969   aus: 1053972

	DS18B20:
	https://github.com/mysensors/MySensorsArduinoExamples/blob/master/libraries/DallasTemperature/examples/Single/Single.pde
	https://forum.mysensors.org/topic/5046/dallas-ds18b20-compiling-error
	https://forum.mysensors.org/topic/2902/dallas-temp-failure-to-compile/21
	https://forum.mysensors.org/topic/4828/temperature-sensor/3
	https://forum.mysensors.org/topic/2434/dallas-temperature-sensor-compiling-error/18

	DY-69 SoilMoist - vergleichbar, nicht 1:1 nutzbar:
	https://github.com/mysensors/MySensors/blob/development/examples/SoilMoistSensor/SoilMoistSensor.ino

	Device 0 Address: 28E5B48605000069
	Device 1 Address: 28FF6AD171160557 //wasserfest,aussen
	DeviceAddress Probe01 = { 0x28, 0xE5, 0xB4, 0x86, 0x05, 0x00, 0x00, 0x69 };
	DeviceAddress Probe02 = { 0x28, 0xFF, 0x6A, 0xD1, 0x71, 0x16, 0x05, 0x57 };
 */

// Enable debug prints
// #define MY_DEBUG
// #define MY_DEBUG_VERBOSE_RF24
// #define MY_SPECIAL_DEBUG

// Use a bit lower baudrate for serial prints than default in MyConfig.h
// #define MY_BAUD_RATE 9600
// Enable and select radio type attached
#define MY_RADIO_NRF24
#define RF24_PA_LEVEL RF24_PA_MAX

#define MY_NODE_ID 2

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE

#define SKETCH_NAME "garden power-multi-sensor node"
#define SKETCH_MAJOR_VER "2"
#define SKETCH_MINOR_VER "1.4"
// const char SKETCH_DATE[] = __DATE__ "." __TIME__;

const unsigned long SECOND = 1000UL;
const unsigned long MINUTE = SECOND * 60;
const unsigned long HOUR = MINUTE * 60;

// wichtig für fhem https://fhem.de/commandref.html#MYSENSORS_DEVICE
#define CHILD_ID_MOISTURE 0
#define CHILD_ID_RELAY1 1 // 2,3,4
//First Child-ID to be used by Dallas Bus;
// set this to be higher than other Child-ID's who need EEPROM storage to avoid conflicts
#define CHILD_ID_TEMP 5 //+6
#define CHILD_ID_VOLTAGE  7
#define CHILD_ID_CPUTEMP  8

#define SEND_ID // Send also Dallas-Addresses?

#include <MySensors.h>
#include "TimerClass.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include "RCSwitch.h"

// Wait times
#define LONG_WAIT 500
#define SHORT_WAIT 50

// RCSWITCH ////////////////////////////////////////////////////////////////////
RCSwitch mySwitch = RCSwitch();
// Receiver on interrupt 0 => that is pin D2
#define SWITCH_PIN 0

// RELAY ///////////////////////////////////////////////////////////////////////
#define RELAY_1  5  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
#define NUMBER_OF_RELAYS 4 // Total number of attached relays
#define RELAY_ON    LOW  // GPIO value to write to turn on attached relay
#define RELAY_OFF   HIGH // GPIO value to write to turn off attached relay
bool ack = 1;
int16_t oldVoltage = -1;
int16_t oldCpuTemp = -1;

// DS18B20 /////////////////////////////////////////////////////////////////////
// Set this offset if the sensor has a permanent small offset to the real temperatures
#define TEMPERATURE_PRECISION 9
#define SENSOR_TEMP_OFFSET 0
// Data wire is plugged into port D3 (example: 2) on the Arduino
#define ONE_WIRE_BUS 4 // D4 - Pin where dallas sensor is connected
#define MAX_ATTACHED_DS18B20 4

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature dallas(&oneWire); // Pass the oneWire reference to Dallas Temperature.

int numSensors=0;
int  conversionTime = 750;

// DeviceAddress Probe01 = { 0x28, 0xE5, 0xB4, 0x86, 0x05, 0x00, 0x00, 0x69 };
// DeviceAddress Probe02 = { 0x28, 0xFF, 0x6A, 0xD1, 0x71, 0x16, 0x05, 0x57 };

// DY-69 ///////////////////////////////////////////////////////////////////////
#define MOISTURE A1
// Because of this electrolysis occurs so it can destroy the probe pretty fast in high-moisture soils. 
// To bypass this, instead of directly linking the VCC to the Arduino's VCC we simply link it to a 
// digital pin and power it (digital pin goes HIGH) only before we do a readout (see the code for this).
#define MOISTURE_VCC A2 // power on/off the sensor
 
// /////// /////////////////////////////////////////////////////////////////////
// sensor data publishing timer// how often will the node publish data?
#ifdef MY_DEBUG
	TimerClass sensorTimerInterval = TimerClass(SECOND * 20);
#else
	TimerClass sensorTimerInterval = TimerClass(MINUTE * 5);
#endif

// /////// /////////////////////////////////////////////////////////////////////
// Initialize message types
// https://www.mysensors.org/download/serial_api_20#variable-types
MyMessage msgMoisture(CHILD_ID_MOISTURE, V_LEVEL);
MyMessage msgRelay(CHILD_ID_RELAY1, V_STATUS); //V_TRIPPED);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
#ifdef SEND_ID
MyMessage msgId(CHILD_ID_TEMP, V_ID);
#endif
MyMessage msgVoltage(CHILD_ID_VOLTAGE, V_VOLTAGE);
MyMessage msgCpuTemp(CHILD_ID_CPUTEMP, V_TEMP);


// /////// /////////////////////////////////////////////////////////////////////
char* charAddr = (char *)"Check for faults";
char* addrToChar(uint8_t* data) {
  String strAddr = String(data[0], HEX); //Chip Version; should be higher than 16
  byte first ;
  int j = 0;
  for (uint8_t i = 1; i < 8; i++) {
    if (data[i] < 16) strAddr = strAddr + 0;
    strAddr = strAddr + String(data[i], HEX);
    strAddr.toUpperCase();
  }
  for (int j = 0; j < 16; j++) {
    charAddr[j] = strAddr[j];
  }
  return charAddr;
}

void presentation(){
  // Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER " (" __DATE__ ")");
  wait(LONG_WAIT);
    
	// Bodensensor YL-69
	present(CHILD_ID_MOISTURE, S_MOISTURE, "Bodenfeuchtigkeit (%)");
  wait(SHORT_WAIT);
    
	// 4 channel relay
  for (int relays=1, pin=RELAY_1; relays<=NUMBER_OF_RELAYS; relays++, pin++) {
      // Register all sensors to gateway (they will be created as child devices)
      present(relays, S_BINARY, "Relais (1/0)", ack);
      wait(SHORT_WAIT);
  }
  
	// DS18B20: Fetch the number of attached temperature sensors
	DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
	numSensors = dallas.getDeviceCount();
	// Present all sensors to controller
	for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
			if (dallas.getAddress(tempDeviceAddress, i)) {
				// set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
		 		dallas.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
				// https://github.com/rejoe2/MySensors-Dallas-Address-ChildID-Consistency/blob/master/DallasTemperatureSensor_Stored_ID/DallasTemperatureSensor_Stored_ID.ino
				charAddr = addrToChar(tempDeviceAddress);
    		Serial.println(charAddr);
		 		present(CHILD_ID_TEMP+i, S_TEMP, charAddr);
        wait(SHORT_WAIT);
 		}
	}
  
  present(CHILD_ID_VOLTAGE, S_MULTIMETER, "Spannungsversorgung (V)");  
  wait(SHORT_WAIT);
  present(CHILD_ID_CPUTEMP, S_TEMP, "CPU Temperatur (°C)");  
  wait(SHORT_WAIT);
}

void before(){
		conversionTime = 750 / (1 << (12 - TEMPERATURE_PRECISION));

  	// Startup up the OneWire library
		dallas.begin();
		// requestTemperatures() will not block current thread
		dallas.setWaitForConversion(false);
}

void setup(){
    mySwitch.enableReceive(SWITCH_PIN);     
    for (int relays=1, pin=RELAY_1; relays<=NUMBER_OF_RELAYS; relays++, pin++) {
        // Then set relay pins in output mode
        pinMode(pin, OUTPUT);
        // Set relay to last known state (using eeprom storage)
        // nach Stromausfall immer alle aus, daher nichts speichern
        // digitalWrite(pin, loadState(relays)?RELAY_ON:RELAY_OFF);
        // statt dessen alles OFF
        digitalWrite(pin, RELAY_OFF);
    }

		// DY-69
		pinMode(MOISTURE, INPUT);
		pinMode(MOISTURE_VCC, OUTPUT);
    digitalWrite(MOISTURE_VCC, LOW);

#ifdef SEND_ID
		DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
		for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
		  dallas.getAddress(tempDeviceAddress, i);
		  //8 sorgt dafür, dass alle 16 Stellen übermittelt werden
		  send(msgId.setSensor(CHILD_ID_TEMP+i).set(tempDeviceAddress, 8));
			// Serial.print(CHILD_ID_TEMP+i);
			// Serial.print("Adress:");
			// charAddr = addrToChar(tempDeviceAddress);
			// Serial.println(charAddr);
		}
#endif
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

void requestTemperature(){

#ifdef MY_DEBUG
  Serial.print("Requesting temperatures...");
#endif
  // Fetch temperatures from Dallas sensors
  dallas.requestTemperatures();
#ifdef MY_DEBUG
  Serial.println("DONE");
#endif

  // query conversion time and sleep until conversion completed
  // int16_t conversionTime = dallas.millisToWaitForConversion(dallas.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
  wait(conversionTime);

  // Read temperatures and send them to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
    // float temperature = static_cast<float>(static_cast<int>(dallas.getTempCByIndex(i)) * 10.) / 10.;
    float temperature = dallas.getTempCByIndex(i);
#ifdef MY_DEBUG
    Serial.print(CHILD_ID_TEMP+i);
    Serial.print("Temperature:");
    Serial.println(temperature);
#endif
    if (temperature != -127.00 && temperature != 85.00) {
      // Send in the new temperature
      send(msgTemp.setSensor(CHILD_ID_TEMP+i).set(temperature,1));
    }
  }
}

void requestMoisture(){
  digitalWrite(MOISTURE_VCC, HIGH); // power sensor
  delay(500);                       // wait before reading analog value
  uint8_t value = 255 - analogRead(MOISTURE);
  digitalWrite(MOISTURE_VCC, LOW);  // set power off for sensor
  
  uint8_t percent = map(value, 20, 255, 0, 99);
  if (percent > 0 && percent < 100) {
    send(msgMoisture.setSensor(CHILD_ID_MOISTURE).set(percent));
  }
  #ifdef MY_DEBUG
    Serial.print("MOISTURE: ");
    Serial.println(percent);
  #endif
}

void receive(const MyMessage &message){
    // We only expect one type of message from controller. But we better check anyway.
    if (message.type==V_STATUS) {
        // Change relay state
        digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
        // Store state in eeprom
        // saveState(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
				// saveState(message.sensor, message.getBool());
        // Write some debug info
#ifdef MY_DEBUG
        Serial.print("Incoming change for sensor:");
        Serial.print(message.sensor);
        Serial.print(", New status: ");
        Serial.println(message.getBool());
#endif
    }
}

void checkRFM433(){
  unsigned long switchesOn[]  = {1049937, 1053009, 1053777, 1053969};
  unsigned long switchesOff[] = {1049940, 1053012, 1053780, 1053972};

  if (mySwitch.available()) { // Wenn ein Code Empfangen wird...

    int value = mySwitch.getReceivedValue(); // Empfangene Daten werden unter der Variable "value" gespeichert.
#ifdef MY_DEBUG
      Serial.print("Empfangen: ");
      Serial.println( value );
#endif

    if (value != 0) { // Wenn der Empfangene Code brauchbar ist, wird er ausgewertet:
			uint8_t id = 0; // id of relay
			uint8_t bValue = -1;
/*
      for (int relays=0, pin=RELAY_1; relays<=NUMBER_OF_RELAYS-1; relays++, pin++) {
        if (value == switchesOn[relays]) {
          bValue = RELAY_ON;
          id = relays;
        }
        if (value == switchesOff[relays]) {
          bValue = RELAY_OFF;
          id = relays;
        }
      }
*/     
      switch (value) {
        case 1049937:
          bValue = RELAY_ON;
					id = 0;
        break;
        case 1049940:
          bValue = RELAY_OFF;
					id = 0;
        break;

        case 1053009:
          bValue = RELAY_ON;
					id = 1;
        break;
        case 1053012:
          bValue = RELAY_OFF;
					id = 1;
        break;

        case 1053777:
          bValue = RELAY_ON;
					id = 2;
        break;
        case 1053780:
          bValue = RELAY_OFF;
					id = 2;
        break;

        case 1053969:
          bValue = RELAY_ON;
					id = 3;
        break;
        case 1053972:
          bValue = RELAY_OFF;
					id = 3;
        break;
      }

			if (bValue == RELAY_OFF || bValue == RELAY_ON ){ // and not -1 (default)
				// switch relay
				digitalWrite(RELAY_1+id, (bool)bValue);
				// send Message back to gateway in order to keep in sync
				send(msgRelay.setSensor(CHILD_ID_RELAY1+id).set((bool)bValue));
				// send(msgRelay.set(tripped ? "1" : "0"));
			}
     
    } else { // Wenn die Empfangenen Daten "0" sind, wird "Unbekannter Code" angezeigt.
#ifdef MY_DEBUG
      Serial.println("Unbekannter Code");
#endif
    }

    mySwitch.resetAvailable(); // Hier wird der Empfänger "resettet"
  }
}


void loop(){
  checkRFM433();

  if (sensorTimerInterval.check() == 1) {
    // ==== todo: add infinite loop tasks here ====
    sendHeartbeat();

    requestTemperature();
    wait(SHORT_WAIT);
    requestMoisture();
    
    // internal battery monitor
    long voltage =  readVcc();
    if (oldVoltage != voltage) {
        // Power up radio after sleep
        send(msgVoltage.set((voltage/1000.0), 2), true);
        oldVoltage = voltage;
    }  
  
    // internal cpu temperature monitor
    long cpuTemp =  readTemp();
    if (oldCpuTemp != cpuTemp) {
        send(msgCpuTemp.set(cpuTemp, 2));
        oldCpuTemp = cpuTemp;
    }
    
  } // end of sensorTimerInterval.check()

}

