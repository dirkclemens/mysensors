/**
 * GatewayWemosD1Mini 
 * 
 * The GatewayWemosD1Mini sends data received from sensors to the WiFi link.
 * The gateway also accepts input on ethernet interface, which is then sent out to the radio network.
 *
 * LED purposes:
 * - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs in your sketch, only the LEDs that is defined is used.
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 * See http://www.mysensors.org/build/esp8266_gateway for wiring instructions.
 * https://www.wemos.cc/product/d1-mini.html
 * nRF24L01+  ESP8266     Wemos D1 mini
 * VCC        VCC         VCC
 * CE         GPIO4       D2
 * CSN/CS     GPIO15      D8
 * SCK        GPIO14      D5
 * MISO       GPIO12      D6
 * MOSI       GPIO13      D7
 * GND        GND         GND
 *
 * Inclusion mode button:
 * - Connect GPIO5 via switch to GND ('inclusion switch')
 */

// Enable debug prints to serial monitor
//#define MY_DEBUG

// dic: extended debugging
//#define MY_DEBUG_VERBOSE

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
#define MY_BAUD_RATE 9600

// Enables and select radio type (if attached)
#define MY_RADIO_NRF24
#define RF24_PA_LEVEL RF24_PA_MAX

// dic: repeater, already included when using as controller
#define MY_REPEATER_FEATURE

#define MY_GATEWAY_ESP8266

#define MY_ESP8266_SSID "netzbox"
#define MY_ESP8266_PASSWORD "FG#3?cB92&"

// Enable UDP communication
//#define MY_USE_UDP

// Set the hostname for the WiFi Client. This is the hostname
// it will pass to the DHCP server if not static.
#define MY_ESP8266_HOSTNAME "MYSensorsWemosD1MiniGateway"

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
//#define MY_IP_ADDRESS 192,168,178,87

// If using static ip you need to define Gateway and Subnet address as well
#define MY_IP_GATEWAY_ADDRESS 192,168,2,23    /// IMMER DIE IP ADRESSE DES CONTROLLERS: smarthome !!!!
#define MY_IP_SUBNET_ADDRESS 255,255,255,0

// The port to keep open on node server mode
#define MY_PORT 5003

// How many clients should be able to connect to this gateway (default 1)
#define MY_GATEWAY_MAX_CLIENTS 10

// Controller ip address. Enables client mode (default is "server" mode).
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere.
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 178, 68

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
#define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
#define MY_INCLUSION_MODE_BUTTON_PIN   5 // D1 to GND alt: 3 // (GIPO0 D3)

// Set blinking period
// #define MY_DEFAULT_LED_BLINK_PERIOD 300

// Flash leds on rx/tx/err
// Led pins used if blinking feature is enabled above
// Wemos D1 Mini Pins !!!
#define MY_DEFAULT_ERR_LED_PIN D3  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  D4  // Receive led pin, the esp onboard LED
#define MY_DEFAULT_TX_LED_PIN  D0  // 

#if defined(MY_USE_UDP)
#include <WiFiUdp.h>
#endif

#include <ESP8266WiFi.h>

#include <MySensors.h>

void presentation() {
  sendSketchInfo(MY_ESP8266_HOSTNAME, "1.1");
}

void setup() {}

void loop() {}


