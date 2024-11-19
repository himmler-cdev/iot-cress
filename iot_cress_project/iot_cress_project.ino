/*
  IoT Cress Project
  -------------------------------------
  
  Author: Christoph Himmler, Kamilo Knezevic, David Sommer, FH JOANNEUM, IMA
  Last update: 18.11.2024
*/

// # Libraries ##############################
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// # Variables ######################################################
/* -----------------------------------  
  Setup Variables for WIFI and MQTT
-------------------------------------- */
const char* SSID = "SSID";
const char* PSK = "PSK";

// MQTT Broker for external use
const char* mqtt_server = "mqtt.medien-transparenz.at";
const char* mqttClient = "sensor5";
const char* mqttUser = "sensor5";
const char* mqttPassword = "sensor5";
const int mqtt_port = 8883;
const int analogSoilSensor = A0;

// WiFiClient espClient;
WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

/* -----------------------------------  
  Time and Sample Data
-------------------------------------- */
unsigned long previousIntervalMillis = 0;                   // Last time sensor data was sent
unsigned long previousSampleMillis = 0;                     // Last time sensor sample was taken
#define SEND_INTERVAL 10000                                 // Send Interval
#define SAMPLE_ARRAY_SIZES 20                               // Amount of Samples taken per interval
#define SAMPLE_INTERVAL SEND_INTERVAL / SAMPLE_ARRAY_SIZES  // Interval for getting Sample data

/* -----------------------------------  
  Soil moisture
-------------------------------------- */
static float soilMoistureArray[SAMPLE_ARRAY_SIZES];  // soil data array
static int soilMoistureIdx = 0;
#define DRY 780
#define WET 350

/* -----------------------------------  
  Define Next Sensor here
-------------------------------------- */

// # End of Variables ######################################################

// # Setup ######################################################

/* -----------------------------------  
  # Setup
--------------------------------------
    Initialize
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

  Serial.println("Setup done");
  Serial.println("================");
}

/* -----------------------------------  
  # Setup 
--------------------------------------
    WiFi
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

  // DO NOT USE INSIDE FH JOANNEUM NETWORK
  //Static IP address configuration
  //WiFi.config(local_IP, gateway, subnet); //Sonst hier IP Adressen angeben

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
  # Setup 
--------------------------------------
    Analog Soil Moisture Calibration 
--------------------------------------*/
void checksoilMoistureCalibration() {
  int analogSoilVal = analogRead(analogSoilSensor);
  Serial.print(analogSoilVal);
}

// # End of Setup ######################################################

/* -----------------------------------  
  # Program
--------------------------------------
    Loop
--------------------------------------*/
void loop() {
  if (!client.connected()) reconnect();  // Check and establish the connection
  client.loop();                         // Maintain connection and check for incomming messsages

  DynamicJsonDocument doc(1024);

  // Calibration
  // analogSoilMoistureCalibration();   // If soil mositure sensor is not yet set:

  // Interval for getting Samples
  unsigned long currentSampleMillis = millis();
  if (currentSampleMillis - previousSampleMillis >= SAMPLE_INTERVAL) {
    previousSampleMillis = currentSampleMillis;
    logSoilMoistureSamples();
  }

  // Interval for sending data
  unsigned long currentIntervalMillis = millis();
  if (currentIntervalMillis - previousIntervalMillis >= SEND_INTERVAL) {
    previousIntervalMillis = currentIntervalMillis;

    doc["soilMoistureMean"] = calcSoilMoistureMean();

    sendSensorData(doc);  // Takes JSON and sends Message
    resetData();
  }
}

/* -----------------------------------  
  # Program
--------------------------------------
   Soil Moisture Data Logger
   -> Reads and normalizes soil moisture values and sets in Array
--------------------------------------*/
void logSoilMoistureSamples() {
  int analogSoilVal = analogRead(analogSoilSensor);
  Serial.println(analogSoilVal);

  // Handle unexpected values
  if (analogSoilVal < 10 || analogSoilVal > 950) {
    Serial.println("Warning: Sensor might be disconnected or malfunctioning.");
    return;  
  }

  // Map and normalize the sensor value
  int mappedValue = map(analogSoilVal, WET, DRY, 1000, 0);
  float normalizedSoilVal = mappedValue / 1000.0;

  // Store the normalized value in the array
  soilMoistureArray[soilMoistureIdx] = normalizedSoilVal;
  soilMoistureIdx++;
}

/* -----------------------------------  
  # Program
--------------------------------------
  Calc Soil Mean of Array
--------------------------------------*/
float calcSoilMoistureMean() {
  float sum = 0.0;

  for (int i = 0; i < SAMPLE_ARRAY_SIZES; i++) {
    sum += soilMoistureArray[i];
  }

  return sum / SAMPLE_ARRAY_SIZES;
}

/* -----------------------------------  
  # Program
--------------------------------------
   Sends Collected Sensor Data to Broker
   -> Serialize and publish JSON data
--------------------------------------*/
void sendSensorData(DynamicJsonDocument& doc) {
  // Serialize the JSON to a string
  char mqtt_message[128];
  serializeJson(doc, mqtt_message);

  // Publish the JSON string to the MQTT broker
  publishMessage("sensor/5/sandbox", mqtt_message, true);

  Serial.println("sendData");
}

/* -----------------------------------  
  # Program
--------------------------------------
   Reset Data (Resets Index of Arrays)
--------------------------------------*/
void resetData() {
  soilMoistureIdx = 0;
}

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
