/*
  Config File
  -------------------------------------
  Author: Christoph Himmler, Kamilo Knezevic, David Sommer, FH JOANNEUM, IMA
  Created: 19.11.2024

  Description: Settings/Config file that holds several data such as libs and defines
  for "iot_cress_project.ino".
  FH Joanneum IoT Project

  Last update: 25.11.2024
*/

#ifndef _CONFIG_H
#define _CONFIG_H


/******************************************************************************
* Libs 
******************************************************************************/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "Seeed_TMG3993.h"
#include "Grove_Temperature_And_Humidity_Sensor.h"


/******************************************************************************
* WIFI and MQTT
******************************************************************************/
const char* SSID = "SSID";
const char* PSK = "PSK";

const char* mqtt_server = "mqtt.medien-transparenz.at";
const char* mqttClient = "sensor5";
const char* mqttUser = "sensor5";
const char* mqttPassword = "sensor5";
const int mqtt_port = 8883;


/******************************************************************************
* Send and Sample Intervals
******************************************************************************/
#define SEND_INTERVAL 600000    // Time interval before sending data to the broker
#define SAMPLE_ARRAY_SIZES 20  // Amount of samples taken per interval

typedef struct Interval_S {
  unsigned long previousMillis;   
  unsigned long interval;   
} Interval_T;


/******************************************************************************
* Analog Soil Moisture Sensor
******************************************************************************/
const int analogSoilSensor = A0;
#define DRY 780
#define WET 350


/******************************************************************************
* Soil Temperature Sensor
******************************************************************************/
#define ONE_WIRE_BUS D3 // Data wire plug


/******************************************************************************
* Sensor Array Type
******************************************************************************/
typedef struct SensorValues_S {
  int idx = 0;
  float values[SAMPLE_ARRAY_SIZES];  
} SensorValues_T;

/******************************************************************************
* Temp and Humidity (DHT)
******************************************************************************/
#define DHTTYPE DHT20 
#define DHTPIN 2


/******************************************************************************
* Button
******************************************************************************/
const int pushButtonPin = D5;


/******************************************************************************
* LEDs
******************************************************************************/
#define LED_BLINK_INTERVAL 5000 // Time in ms how long the LED should blink
#define LED_BLINK_FREQUENCY 5 // Amount of how often the LED should blink (in the interval)
const int onBoardLED = D0; //Build in LED
const int externalLED = D7; 

typedef struct LED_S {
  unsigned long previousMillis;   
  unsigned long interval;
  unsigned long frequency;
  unsigned long startMillis;
  bool state;   
} LED_T;


/******************************************************************************
* Color LED
******************************************************************************/
const int colorRedLED = D6; // D6 because D10 caused issues/failure
const int colorBlueLED = D8;
const int colorGreenLED = D9;

/******************************************************************************
* Sleep Mode
******************************************************************************/
typedef struct SleepMode_S {
  bool isSleepMode;
  int lastLEDColors[3]; 
} SleepMode_T;


#endif
