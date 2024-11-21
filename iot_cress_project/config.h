/*
  Config File
  -------------------------------------
  Author: Christoph Himmler, Kamilo Knezevic, David Sommer, FH JOANNEUM, IMA
  Created: 19.11.2024

  Description: Settings/Config file that holds several data such as libs and defines
  for "iot_cress_project.ino".
  FH Joanneum IoT Project

  Last update: 20.11.2024
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
#define SEND_INTERVAL 10000    // Time interval before sending data to the broker
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
typedef struct sensorValues_S {
  int idx = 0;
  float values[SAMPLE_ARRAY_SIZES];  
} sensorValues_T;


#endif
