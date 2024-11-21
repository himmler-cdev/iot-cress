/*
  IoT Cress Project
  -------------------------------------
  Author: Christoph Himmler, Kamilo Knezevic, David Sommer, FH JOANNEUM, IMA
  Created: 18.11.2024

  Description: Smart monitoring and control system utilizing IoT 
  technology to optimize environmental conditions for growing cress.
  FH Joanneum IoT Project

  Last update: 20.11.2024
*/
#include "config.h"


/******************************************************************************
* Variables 
******************************************************************************/
/* -----------------------------------  
*  WiFiClient Setup
-------------------------------------- */
// WiFiClient espClient;
WiFiClientSecure espClient;
PubSubClient client(espClient);

/* -----------------------------------  
*  Message Instance
-------------------------------------- */
#define MSG_BUFFER_SIZE 512
char msg[MSG_BUFFER_SIZE];

/* -----------------------------------  
*  Time and Sample Data
-------------------------------------- */
Interval_T sendInterval;
Interval_T sampleInterval;

/* -----------------------------------  
*  Soil moisture
-------------------------------------- */
sensorValues_T soilMoisture;

/* -----------------------------------  
*  Soil Temerature
-------------------------------------- */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
sensorValues_T soilTemperature;

/* -----------------------------------  
*  Define Next Sensor here
-------------------------------------- */

/******************************************************************************
* Setup 
******************************************************************************/

/* -----------------------------------  
*  Initialize
--------------------------------------*/
void setup() {
  Serial.begin(115200);           // Baud rate microcontroller connection
  Serial.println("Setup begin");  //Debug message

  //WiFi
  WiFi.mode(WIFI_STA);
  setup_wifi();
  delay(1000);

  //MQTT
  espClient.setInsecure();

  //Client MQTT Setup
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Soil Temperature Sensor Init
  sensors.begin();

  // Struct Inits (sets values to 0)
  memset((void*)&sendInterval, 0, sizeof(Interval_S));
  memset((void*)&sampleInterval, 0, sizeof(Interval_S));
  memset((void*)&soilMoisture, 0, sizeof(sensorValues_S));
  memset((void*)&soilTemperature, 0, sizeof(sensorValues_S));
  
  // Struct data inits
  sendInterval.interval = SEND_INTERVAL;
  sampleInterval.interval = SEND_INTERVAL / SAMPLE_ARRAY_SIZES;

  Serial.println("Setup done");
  Serial.println("================");
}

/* -----------------------------------  
*  WiFi
--------------------------------------*/
void setup_wifi() {
  delay(10);
  //Debug output
  Serial.println();
  Serial.println("Hostname");
  Serial.println(WiFi.getHostname());
  String mac = WiFi.macAddress();
  Serial.print("MAC Address");
  Serial.println(mac);
  Serial.print("Connecting to ");
  Serial.println(SSID);

  //Connect to WiFi
  WiFi.begin(SSID, PSK);

  Serial.print("Try to connect in progress ");

  //Show connection attempts for network troubleshooting
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("--- Connection status ---");
    Serial.printf("Connection status: %d\n", WiFi.status());
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("--- WiFi.printDiag ---");
    WiFi.printDiag(Serial);
    Serial.println("---  ---");
  }

  //After connected show IP
  Serial.println("==============");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("==============");
}

/* -----------------------------------  
*  Soil Moisture Calibration 
*  -> Prints Soil value to console for calibration to define DRY and WET 
--------------------------------------*/
void checkSoilMoistureCalibration() {
  int val = analogRead(analogSoilSensor);
  Serial.print(val);
  Serial.println(" - Function: checkSoilMoistureCalibration");
}

/******************************************************************************
* Program 
******************************************************************************/

/* -----------------------------------  
*  Loop
--------------------------------------*/
void loop() {
  if (!client.connected()) reconnect();  // Check and establish the connection
  client.loop();                         // Maintain connection and check for incomming messsages

  DynamicJsonDocument doc(1024);

  // Calibration
  // checkSoilMoistureCalibration();   // If soil mositure sensor is not yet set:

  // Set current millis for time calc
  unsigned long currentMillis = millis();

  // Interval for getting samples
  if (currentMillis - sampleInterval.previousMillis >= sampleInterval.interval) {
    sampleInterval.previousMillis = currentMillis;
    logSoilMoistureSamples();
    logSoilTemperatureSamples();
  }

  // Interval for sending data
  if (currentMillis - sendInterval.previousMillis >= sendInterval.interval) {
    sendInterval.previousMillis = currentMillis;

    doc["soilMoistureMean"] = calcMean(soilMoisture.values);
    doc["soilTemperatureMean"] = calcMean(soilTemperature.values);

    sendSensorData(doc);  // Takes JSON and sends message
    resetData();
  }
}

/* -----------------------------------  
*  Soil Moisture Data Logger
*  -> Reads and normalizes soil moisture values and sets in Array
--------------------------------------*/
void logSoilMoistureSamples() {
  int analogSoilVal = analogRead(analogSoilSensor);

  // Handle unexpected values
  if (analogSoilVal < 10 || analogSoilVal > 950) {
    Serial.println("Warning: Sensor might be disconnected or malfunctioning. - Function: logSoilMoistureSamples");
    return;
  }

  // Map and normalize the sensor value
  int mappedValue = map(analogSoilVal, WET, DRY, 1000, 0);
  float normalizedSoilVal = mappedValue / 1000.0;

  // Store the normalized value in the array
  soilMoisture.values[soilMoisture.idx] = normalizedSoilVal;
  soilMoisture.idx++;
}

/* -----------------------------------  
*  Soil Temperature Data Logger
*  -> Reads values and sets in Array
--------------------------------------*/
void logSoilTemperatureSamples() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  
  if(tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data - Function: logSoilTemperatureSamples");
    return;
  }

  // Check if reading was successful
  soilTemperature.values[soilTemperature.idx] = tempC;
  soilTemperature.idx++;
}

/* -----------------------------------
 *  Mean Calculator
 *  -> Calculates mean from an array
 --------------------------------------*/
float calcMean(float values[]) {
  float sum = 0.0;

  for (int i = 0; i < SAMPLE_ARRAY_SIZES; i++) {
    sum += values[i];
  }

  return sum / SAMPLE_ARRAY_SIZES;
}

/* -----------------------------------  
*  Sends Collected Sensor Data to Broker
*  -> Serialize and publish JSON data to broker
--------------------------------------*/
void sendSensorData(DynamicJsonDocument& doc) {
  serializeJson(doc, msg);                        // Serialize the JSON to a string
  publishMessage("sensor/5/sandbox", msg, true);  // Publish the JSON string to the MQTT broker

  Serial.println("---------- Send Sensor Data ---------- - Function: sendSensorData");
}

/* -----------------------------------  
*  Reset Data 
*  -> Resets index of arrays
--------------------------------------*/
void resetData() {
  soilMoisture.idx = 0;
  soilTemperature.idx = 0;
}

/******************************************************************************
* MQTT and WIFI Logic from Lecture 
******************************************************************************/

/* -----------------------------------  
  # MQTT connect to broker
--------------------------------------
  Connection 
  Subscription
--------------------------------------*/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    //if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) {
    if (client.connect(mqttClient, mqttUser, mqttPassword)) {
      Serial.println("connected");

      client.subscribe("sensor/5/sandbox/feedback");  // subscribe the topics here

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");  // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/* -----------------------------------  
  # MQTT callback
--------------------------------------
  Receiving messages from the broker
--------------------------------------*/
void callback(char* topic, byte* payload, unsigned int length) {
  (void)topic;
  (void)payload;
  (void)length;
}

/* -----------------------------------  
  # Publish messages
--------------------------------------
  Publish MQTT messages 
--------------------------------------*/
void publishMessage(const char* topic, String payload, boolean retained) {
  if (client.publish(topic, payload.c_str(), true))
    Serial.println("Message published [" + String(topic) + "]: " + payload);
}
