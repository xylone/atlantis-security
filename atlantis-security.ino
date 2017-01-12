/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik EKblad
 * Contribution by a-lurker and Anticimex,
 * Contribution by Norbert Truchsess <norbert.truchsess@t-online.de>
 * Contribution by Ivo Pullens (ESP8266 support)
 *
 * DESCRIPTION
 * The EthernetGateway sends data received from sensors to the WiFi link.
 * The gateway also accepts input on ethernet interface, which is then sent out to the radio network.
 *
 * VERA CONFIGURATION:
 * Enter "ip-number:port" in the ip-field of the Arduino GW device. This will temporarily override any serial configuration for the Vera plugin.
 * E.g. If you want to use the defualt values in this sketch enter: 192.168.178.66:5003
 *
 * LED purposes:
 * - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs in your sketch, only the LEDs that is defined is used.
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 * See http://www.mysensors.org/build/esp8266_gateway for wiring instructions.
 * nRF24L01+  ESP8266
 * VCC        VCC
 * CE         GPIO4
 * CSN/CS     GPIO15
 * SCK        GPIO14
 * MISO       GPIO12
 * MOSI       GPIO13
 * GND        GND
 *
 * Not all ESP8266 modules have all pins available on their external interface.
 * This code has been tested on an ESP-12 module.
 * The ESP8266 requires a certain pin configuration to download code, and another one to run code:
 * - Connect REST (reset) via 10K pullup resistor to VCC, and via switch to GND ('reset switch')
 * - Connect GPIO15 via 10K pulldown resistor to GND
 * - Connect CH_PD via 10K resistor to VCC
 * - Connect GPIO2 via 10K resistor to VCC
 * - Connect GPIO0 via 10K resistor to VCC, and via switch to GND ('bootload switch')
 *
  * Inclusion mode button:
 * - Connect GPIO5 via switch to GND ('inclusion switch')
 *
 * Hardware SHA204 signing is currently not supported!
 *
 * Make sure to fill in your ssid and WiFi password below for ssid & pass.
 */


// Enable debug prints to serial monitor
#define MY_DEBUG

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
#define MY_BAUD_RATE 9600

// Enables and select radio type (if attached)
#define MY_RADIO_NRF24

// To use native I2C pins CE should be moved. Declare the new GPIO used here.
// GPIO 3 is pin D9 (RX next to D8)
#define MY_RF24_CE_PIN 3
#define MY_RF24_CHANNEL 83

//#define MY_RADIO_RFM69

#define MY_GATEWAY_ESP8266

//#define MY_ESP8266_SSID "Atlantis"
//#define MY_ESP8266_PASSWORD "changeme"

// Enable UDP communication
//#define MY_USE_UDP

// Set the hostname for the WiFi Client. This is the hostname
// it will pass to the DHCP server if not static.
// #define MY_ESP8266_HOSTNAME "sensor-gateway"

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
//#define MY_IP_ADDRESS 192,168,178,87

// If using static ip you need to define Gateway and Subnet address as well
//#define MY_IP_GATEWAY_ADDRESS 192,168,178,1
//#define MY_IP_SUBNET_ADDRESS 255,255,255,0

// The port to keep open on node server mode
#define MY_PORT 5003

// How many clients should be able to connect to this gateway (default 1)
#define MY_GATEWAY_MAX_CLIENTS 2

// Controller ip address. Enables client mode (default is "server" mode).
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere.
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 178, 68

// Enable inclusion mode
//#define MY_INCLUSION_MODE_FEATURE

// Enable Inclusion mode button on gateway
// #define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
//#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3


// Set blinking period
// #define MY_DEFAULT_LED_BLINK_PERIOD 300

// Flash leds on rx/tx/err
// Led pins used if blinking feature is enabled above
#define MY_DEFAULT_ERR_LED_PIN 16  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  16  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  16  // the PCB, on board LED

#if defined(MY_USE_UDP)
#include <WiFiUdp.h>
#endif

#include <ESP8266WiFi.h>
#include <Adafruit_MCP23008.h>
#include <MySensors.h>

// ----------------------------------------------------------------------------
// I2C setup: SDA and SCL pin selection
// ----------------------------------------------------------------------------
#include <Wire.h>

// If using the standard MySensor wiring direction we can't use default I2C ports. I2C ports moved next to the second GND and 3.3v ports for convenient wiring of I2c modules.
//#define SDA 3  //default to GPIO 4 (D2) but NRF24L01+ CE is plugged on D2 by default for the ESP gateway. SDA changed to GPIO 3 (RX)
//#define SCL 1  //default to GPIO 5 (D1). SDA changed to GPIO 1 (TX)

#define ROOMNODE_NAME "Alarm"
#define ROOMNODE_VERSION "1.0"

#define SENSOR_COUNT 4
#define TAMPERED_ID 10
#define TAMPERED_STATE_ID 11

typedef struct {
  int id;
  char *name;
  int alarm_pin;
  int tamper_pin;
  bool tripped = false;
  bool armed = true;
  bool tampered = false;
  int type;
} sensor;

sensor sensors[SENSOR_COUNT];

MyMessage msgSensorTripped(0, V_TRIPPED);
MyMessage msgSensorArmed(0, V_ARMED);

MyMessage msgSensorTampered(0, V_STATUS);
MyMessage msgSensorTamperedState(0, V_TEXT);

Adafruit_MCP23008 mcp;

bool alarmArmed = false;

void before() {
    // Define alarm sensors
  sensors[0].id = 1;
  sensors[0].name = "Woonkamer beweging";
  sensors[0].alarm_pin = 0;
  sensors[0].tamper_pin = 1;
  sensors[0].type = S_MOTION;

  sensors[1].id = 2;
  sensors[1].name = "Gang beweging";
  sensors[1].alarm_pin = 2;
  sensors[1].tamper_pin = 3;
  sensors[1].type = S_MOTION;

  sensors[2].id = 3;
  sensors[2].name = "Schuur beweging";
  sensors[2].alarm_pin = 4;
  sensors[2].tamper_pin = 5;
  sensors[2].type = S_MOTION;

  sensors[3].id = 4;
  sensors[3].name = "Woonkamer rookmelder";
  sensors[3].alarm_pin = 6;
  sensors[3].tamper_pin = 7;
  sensors[3].type = S_SMOKE;

//  sensors[4].id = 5;
//  sensors[4].name = "Overloop rookmelder";
//  sensors[4].alarm_pin = 8;
//  sensors[4].tamper_pin = 9;
//  sensors[4].type = S_SMOKE; 
}

void setup()
{
  alarmArmed = (bool)loadState(0);
  
  // MCP23008 Multiplex
  mcp.begin();           // use default address 0

  for (int i=0; i<SENSOR_COUNT; i++) {
    mcp.pinMode(sensors[i].alarm_pin, INPUT);
    mcp.pinMode(sensors[i].tamper_pin, INPUT);
    mcp.pullUp(sensors[i].alarm_pin, LOW);
    mcp.pullUp(sensors[i].tamper_pin, LOW);
    //mcp.pullUp(sensors[i].alarm_pin, HIGH);   // turn on a 100K pullup internally
  }

  //presentation();
}

void presentation()
{
  // Send the Sketch Version Information to the Gateway
  
  sendSketchInfo(ROOMNODE_NAME, ROOMNODE_VERSION);

  present(1, S_BINARY, "Armed");
  present(2, S_INFO, "Alarm state");
  
  present(TAMPERED_ID, S_BINARY, "Tampered");
  present(TAMPERED_STATE_ID, S_INFO, "Tampered state");

  Serial.println("Present sensors");
  for (int i=0; i<SENSOR_COUNT; i++) {
    int childId = 100 + sensors[i].id;
    present(childId, sensors[i].type, sensors[i].name);
  }

  for (int i=0; i<SENSOR_COUNT; i++) {
    int childId = 100 + sensors[i].id;
    send(msgSensorTripped.setSensor(childId).set(sensors[i].tripped));
    send(msgSensorArmed.setSensor(childId).set(sensors[i].armed));
  }
}

unsigned long lastSendTime = 0;
unsigned long now = 0;

void loop()
{
//  now = millis();
	// Send locally attached sensors data here
//  if ((now - lastSendTime) > 500) {
    for (int i=0; i<SENSOR_COUNT; i++) {
      if (alarmArmed) {
        bool tripped = mcp.digitalRead(sensors[i].alarm_pin);
        if (sensors[i].tripped != tripped) {
          sensors[i].tripped = tripped;
          int childId = 100 + sensors[i].id;
          send(msgSensorTripped.setSensor(childId).set(sensors[i].tripped));
          Serial.print("Send status of sensor ");
          Serial.println(sensors[i].name);
        }
      }
      bool tampered = !mcp.digitalRead(sensors[i].tamper_pin); // Normaly closed (1), so tampered when 0
      if (sensors[i].tampered != tampered) {
        sensors[i].tampered = tampered;
        send(msgSensorTampered.setSensor(TAMPERED_ID).set(tampered));
        Serial.print("Send tampered of sensor ");
        Serial.println(sensors[i].name);
        if (tampered) {
          send(msgSensorTamperedState.setSensor(TAMPERED_STATE_ID).set(sensors[i].name));
        } else {
          send(msgSensorTamperedState.setSensor(TAMPERED_STATE_ID).set("Geen problemen"));
        }
      }
    }
//    lastSendTime = now;
//  }
}

// This is called when a message is received 
void receive(const MyMessage &message) {
  alarmArmed = message.getBool();
  saveState(0, alarmArmed);
  if (alarmArmed) { 
    // Write some debug info
    Serial.println("Alarm armed");
  } else {
    Serial.println("Alarm disarmed");
  }
}
