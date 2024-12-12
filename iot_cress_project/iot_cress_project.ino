/*
  IoT Cress Project
  -------------------------------------
  Author: Christoph Himmler, Kamilo Knezevic, David Sommer, FH JOANNEUM, IMA
  Created: 18.11.2024

  Description: Smart monitoring and control system utilizing IoT 
  technology to optimize environmental conditions for growing cress.
  FH Joanneum IoT Project

  Last update: 25.11.2024
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
SensorValues_T soilMoisture;

/* -----------------------------------  
*  Soil Temerature
-------------------------------------- */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
SensorValues_T soilTemperature;

/* -----------------------------------  
*  Light/Brigthness 
-------------------------------------- */
TMG3993 tmg3993;
SensorValues_T light;  // Lux
SensorValues_T cct;    // Color Temperature

/* -----------------------------------  
*  Temp and Humidity (DHT)
-------------------------------------- */
DHT dht(DHTTYPE);
SensorValues_T airHumidity;
SensorValues_T airTemperature;

/* -----------------------------------  
*  Button
-------------------------------------- */
volatile unsigned int wateringCnt;
unsigned int wateringLoopCnt;
volatile bool isButtonPressed = false;
//Debouncing:
volatile unsigned long buttonChangedTime = 0;  //Volatile: Varialbes used inside Interrupt functions should be marked by volatile. Its a compiler directive which tells the microcontroller to load this variable from RAM
volatile unsigned long lastButtonChangedTime = 0;
volatile int bounceDelay = 50;  //Value in ms

/* -----------------------------------  
*  LEDs
-------------------------------------- */
LED_T led;

/* -----------------------------------  
*  OLED
-------------------------------------- */
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

/* -----------------------------------  
*  Sleep Mode
-------------------------------------- */
SleepMode_T sleepMode;

/******************************************************************************
* Button Interrupt 
******************************************************************************/
/* -----------------------------------  
  # Interrupt handling - Button state changed
--------------------------------------
    !!!!! Attention: This code has to be written before void Setup() because of ICACHE_RAM_ATTR attribute. 
      Placing it afterwards, will cause an error. 
    
    ICACHE_RAM_ATTR attribute places the code in ESP8266’s Internal RAM (IRAM), which works faster than flash memory on ESP8266. 
      Placing the code in flash memory can cause exceptions and crashes 

    Within Interrupts there should be as less code as possible. For some reasons, it might be good, 
    if complete debouncing is done insige the interrupt code. MAybe this code dont work in every project. 
--------------------------------------*/
void ICACHE_RAM_ATTR buttonStateInterruptChanged() {

  buttonChangedTime = millis();  //There are sources which means that within the routines millis(), delay() and micros() dont work, becaue they rely on interrupts as well. This Code works pretty well.

  //If the button was not pressed before, and it is the first high signal, the button poress is accepted.
  //If within some milliseconds another signal change is recognized, it might come from bouncing of the
  //mechanical parts of the button. Bouncing will be ignored (= Debouncing).
  if (!isButtonPressed && (buttonChangedTime - lastButtonChangedTime > bounceDelay)) {  //responsible for the pressed (e.g. clicked) state of the button
    isButtonPressed = true;
    wateringCnt++;
    lastButtonChangedTime = buttonChangedTime;
  }

  if (isButtonPressed && (buttonChangedTime - lastButtonChangedTime > bounceDelay)) {  //responsible for the released state of the button
    isButtonPressed = false;
    lastButtonChangedTime = buttonChangedTime;
  }
}


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

  // Light Init
  Wire.begin();

  if (tmg3993.initialize() == false) {
    Serial.println("Device not found. Check wiring.");
    while (1)
      ;
  }
  tmg3993.setADCIntegrationTime(0xdb);
  tmg3993.enableEngines(ENABLE_PON | ENABLE_AEN | ENABLE_AIEN);

  // DHT Init
  dht.begin();

  // Button
  pinMode(pushButtonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pushButtonPin), buttonStateInterruptChanged, CHANGE);
  wateringCnt, wateringLoopCnt = 0;

  // LEDs
  pinMode(onBoardLED, OUTPUT);
  pinMode(externalLED, OUTPUT);
  pinMode(colorBlueLED, OUTPUT);
  pinMode(colorGreenLED, OUTPUT);
  pinMode(colorRedLED, OUTPUT);

  digitalWrite(onBoardLED, HIGH);
  digitalWrite(externalLED, LOW);

  // OLED
  u8g2.begin();        //Connect to display
  u8g2.clearBuffer();  //Clear buffer
  u8g2.setFont(u8g2_font_profont11_mf);
  u8g2.drawStr(0, 10, "Started Project of");
  u8g2.drawStr(0, 25, "C. Himmler");
  u8g2.drawStr(0, 40, "D. Sommer");
  u8g2.drawStr(0, 55, "K. Knezevic");
  u8g2.sendBuffer();

  // Struct Inits (sets values to 0)
  memset((void*)&sendInterval, 0, sizeof(Interval_S));
  memset((void*)&sampleInterval, 0, sizeof(Interval_S));
  memset((void*)&led, 0, sizeof(LED_T));
  memset((void*)&soilMoisture, 0, sizeof(SensorValues_S));
  memset((void*)&soilTemperature, 0, sizeof(SensorValues_S));
  memset((void*)&light, 0, sizeof(SensorValues_S));
  memset((void*)&cct, 0, sizeof(SensorValues_S));
  memset((void*)&airHumidity, 0, sizeof(SensorValues_S));
  memset((void*)&airTemperature, 0, sizeof(SensorValues_S));
  memset((void*)&sleepMode, 0, sizeof(SleepMode_S));

  // Struct data inits
  sendInterval.interval = SEND_INTERVAL;
  sampleInterval.interval = SEND_INTERVAL / SAMPLE_ARRAY_SIZES;
  led.interval = LED_BLINK_INTERVAL;
  led.frequency = LED_BLINK_INTERVAL / (LED_BLINK_FREQUENCY * 2);
  led.state = LOW;
  sleepMode.isSleepMode = false;

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

  // Watering Button Handling
  if (wateringLoopCnt != wateringCnt) {
    wateringLoopCnt = wateringCnt;
    sendWatering(wateringCnt);
    led.startMillis = millis();  // Comment this out if you do not want blinking of the leds when clicking the button
    printWateringMsg(wateringCnt);
  }

  // Interval for getting samples
  if (currentMillis - sampleInterval.previousMillis >= sampleInterval.interval) {
    sampleInterval.previousMillis = currentMillis;
    logSoilMoistureSamples();
    logSoilTemperatureSamples();
    logLightSamples();
    logDHTSamples();  // Air - Temp and Humidity
  }

  // Interval for sending data
  if (currentMillis - sendInterval.previousMillis >= sendInterval.interval) {
    sendInterval.previousMillis = currentMillis;

    doc["soilMoisture"] = (calcMean(soilMoisture.values) * 100);
    doc["soilTemperature"] = calcMean(soilTemperature.values);
    doc["lightVisible"] = calcMean(light.values);
    doc["colorTemperature"] = calcMean(cct.values);
    doc["airHumidity"] = calcMean(airHumidity.values);
    doc["airTemperature"] = calcMean(airTemperature.values);
    doc["upTimeSec"] = (currentMillis / 1000);

    sendSensorData(doc);  // Takes JSON and sends message
    //led.startMillis = millis(); // Comment this out if you do not want blinking of the leds when sending
    printDocToOLED(doc, wateringCnt);
    resetData();
  }

  // Handle LED
  if (currentMillis - led.startMillis < led.interval) {
    blinkLED();
  } else {
    digitalWrite(onBoardLED, HIGH);
    digitalWrite(externalLED, LOW);
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

  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data - Function: logSoilTemperatureSamples");
    return;
  }

  // Check if reading was successful
  soilTemperature.values[soilTemperature.idx] = tempC;
  soilTemperature.idx++;
}

/* -----------------------------------  
*  Light Data Logger
*  -> Reads values and sets in Array
--------------------------------------*/
void logLightSamples() {
  if (tmg3993.getSTATUS() & STATUS_AVALID) {
    uint16_t r, g, b, c;
    int32_t luxVal;
    int32_t cctVal;
    tmg3993.getRGBCRaw(&r, &g, &b, &c);
    luxVal = tmg3993.getLux(r, g, b, c);
    cctVal = tmg3993.getCCT(r, g, b, c);

    light.values[light.idx] = luxVal;
    cct.values[cct.idx] = cctVal;
    light.idx++;
    cct.idx++;

    tmg3993.clearALSInterrupts();
  } else {
    Serial.println("Error: Could not read light data - Function: logLightSamples");
  }
}

/* -----------------------------------  
*  DHT Data Logger
*  -> Reads air temperature and air humidity values and sets in Array
--------------------------------------*/
void logDHTSamples() {
  float hum_temp_val[20] = { 0 };

  if (!dht.readTempAndHumidity(hum_temp_val)) {
    airHumidity.values[airHumidity.idx] = hum_temp_val[0];
    airTemperature.values[airTemperature.idx] = hum_temp_val[1];

    airHumidity.idx++;
    airTemperature.idx++;
  }
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
  serializeJson(doc, msg);                                 // Serialize the JSON to a string
  publishMessage("sensor/5/environment", msg, true);  // Publish the JSON string to the MQTT broker

  Serial.println("---------- Send Sensor Data ---------- - Function: sendSensorData");
}

/* -----------------------------------  
*  Sends Collected Sensor Data to Broker
*  -> Serialize and publish JSON data to broker
--------------------------------------*/
void sendWatering(int btnCnt) {
  DynamicJsonDocument doc(1024);
  doc["wateringCnt"] = btnCnt;
  serializeJson(doc, msg);                            // Serialize the JSON to a string
  publishMessage("sensor/5/urgent", msg, true);  // Publish the JSON string to the MQTT broker

  Serial.println("---------- Send Watering ---------- - Function: sendWatering");
}

/* -----------------------------------  
*  Print Watering Message
*  -> Prints message and button count to the OLED 
--------------------------------------*/
void printWateringMsg(int btnCnt) {
  char countStr[20];
  sprintf(countStr, "Button Count: %d", btnCnt);

  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "Water Button Pressed");
  u8g2.drawStr(0, 30, countStr);
  u8g2.drawStr(0, 50, "Don't forget Pictures");
  u8g2.sendBuffer();
}

/* -----------------------------------  
*  Print Documentation Message
*  -> Prints sensor data via OLED-display
--------------------------------------*/
void printDocToOLED(DynamicJsonDocument& doc, int btnCnt) {
  u8g2.clearBuffer();

  // First line: SM (Soil Moisture as percentage) and ST (Soil Temperature in °C)
  char firstLine[30];                                                 // Buffer for the line
  float soilMoisturePercent = doc["soilMoisture"].as<float>() * 100;  // Convert to percentage
  sprintf(firstLine, "SM:%.2f%% | ST:%.2fC", soilMoisturePercent, doc["soilTemperature"].as<float>());
  u8g2.drawStr(0, 10, firstLine);

  // Second line: BR (Brightness) and CT (Color Temperature)
  char secondLine[30];
  const char* colorTemperature = formatWithSI(doc["colorTemperature"].as<float>());
  sprintf(secondLine, "BR:%.2f | CT:%s", doc["lightVisible"].as<float>(), colorTemperature);
  u8g2.drawStr(0, 22, secondLine);

  // Third line: AH (Air Humidity as percentage) and AT (Air Temperature in °C)
  char thirdLine[30];
  sprintf(thirdLine, "AH:%.2f%% | AT:%.2fC", doc["airHumidity"].as<float>(), doc["airTemperature"].as<float>());
  u8g2.drawStr(0, 34, thirdLine);

  // Fourth Line: Watering Button Count
  char wateringCountLine[30];
  sprintf(wateringCountLine, "Button Count: %d", btnCnt);
  u8g2.drawStr(0, 46, wateringCountLine);

  // Last line: Uptime in days, hours, minutes
  unsigned long totalSeconds = doc["upTimeSec"].as<unsigned long>();
  unsigned long days = totalSeconds / 86400;
  unsigned long hours = (totalSeconds % 86400) / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;

  char uptimeStr[30];
  sprintf(uptimeStr, "Uptime: %lud %luh %lum", days, hours, minutes);
  u8g2.drawStr(0, 58, uptimeStr);

  // Send the buffer to the OLED
  u8g2.sendBuffer();
}

/* -----------------------------------  
*  Format Number/Value into SI
*  -> Turns a value into corresponding SI value (from Kilo to Tera)
--------------------------------------*/
const char* formatWithSI(float value) {
  if (value >= 1e12) {
    // Tera (T)
    static char formatted[10];
    sprintf(formatted, "%.2fT", value / 1e12);
    return formatted;
  } else if (value >= 1e9) {
    // Giga (G)
    static char formatted[10];
    sprintf(formatted, "%.2fG", value / 1e9);
    return formatted;
  } else if (value >= 1e6) {
    // Mega (M)
    static char formatted[10];
    sprintf(formatted, "%.2fM", value / 1e6);
    return formatted;
  } else if (value >= 1e3) {
    // Kilo (K)
    static char formatted[10];
    sprintf(formatted, "%.2fK", value / 1e3);
    return formatted;
  } else {
    // Normal Number
    static char formatted[10];
    sprintf(formatted, "%.2f", value);
    return formatted;
  }
}

/* -----------------------------------  
*  LED Blink Controller
*  -> Turns the LEDs on and off for LED_BLINK_INTERVAL time in LED_BLINK_FREQUENCY (see config.h) 
--------------------------------------*/
void blinkLED() {
  unsigned long currentMillis = millis();

  if (currentMillis - led.previousMillis >= led.frequency) {
    led.previousMillis = currentMillis;
    led.state = !led.state;
    digitalWrite(onBoardLED, !led.state);
    digitalWrite(externalLED, led.state);
  }
}

/* -----------------------------------  
*  Reset Data 
*  -> Resets index of arrays
--------------------------------------*/
void resetData() {
  soilMoisture.idx = 0;
  soilTemperature.idx = 0;
  light.idx = 0;
  cct.idx = 0;
  airHumidity.idx = 0;
  airTemperature.idx = 0;
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

      client.subscribe("sensor/5/feedback");  // subscribe the topics here

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
  if (length > 0) {
    char jsonCallback[length + 1];          // Create a char array for the incoming JSON
    memcpy(jsonCallback, payload, length);  // Copy payload to the array
    jsonCallback[length] = '\0';            // Null-terminate the string

    Serial.println("Message arrived [" + String(topic) + "]: " + String(jsonCallback));

    // JSON deserialization
    StaticJsonDocument<200> docFeedback;  // Adjust size based on expected payload
    DeserializationError error = deserializeJson(docFeedback, jsonCallback);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.f_str());
      return;  // Exit if JSON parsing fails
    }

    // Check if colorFeedback exists in the JSON
    if (docFeedback.containsKey("colorFeedback")) {
      JsonArray colorFeedback = docFeedback["colorFeedback"];

      int rgbColors[3];
      for (int i = 0; i < 3; i++) {
        // Check if the value is a valid number (int)
        int colorValue = colorFeedback[i].as<int>();

        // If the value is outside the range or invalid, set it to 0
        if (colorValue < 0 || colorValue > 255) {
          rgbColors[i] = 0;
        } else {
          rgbColors[i] = colorValue;
        }
      }

      setColorLED(rgbColors);  // Set the LED color
    }

    // Check if sleep mode is toggled
    if (docFeedback.containsKey("setSleepMode")) {
      // Check if the value is explicitly boolean true
      if (docFeedback["setSleepMode"].as<bool>() == true) {
        sleepMode.isSleepMode = true;
      } else {
        sleepMode.isSleepMode = false;
      }
      
      setSleepMode();
    }
  }
}

/* -----------------------------------
 *  Set LED Colors
 *  -> Helper function to set RGB values to LEDs
--------------------------------------*/
void setLEDColors(int red, int green, int blue) {
  analogWrite(colorRedLED, red);
  analogWrite(colorGreenLED, green);
  analogWrite(colorBlueLED, blue);
}

/* -----------------------------------
 *  Set Color LED based on Feedback
 *  -> Sets RGB color based on feedback value
--------------------------------------*/
void setColorLED(int rgbColors[]) {
  // Save the current color to sleepMode
  for (int i = 0; i < 3; i++) {
    sleepMode.lastLEDColors[i] = rgbColors[i];
  }

  // Update the LEDs only if not in sleep mode
  if (!sleepMode.isSleepMode) {
    setLEDColors(sleepMode.lastLEDColors[0], sleepMode.lastLEDColors[1], sleepMode.lastLEDColors[2]);
  }
}

/* -----------------------------------
 *  Set Sleep Mode
 *  -> Turns on/off the LEDs based on sleepMode state
--------------------------------------*/
void setSleepMode() {
  if (sleepMode.isSleepMode) {
    setLEDColors(0, 0, 0);
    u8g2.setPowerSave(true);  // Turn off the OLED
  } else {
    setLEDColors(sleepMode.lastLEDColors[0], sleepMode.lastLEDColors[1], sleepMode.lastLEDColors[2]);
    u8g2.setPowerSave(false);  // Turn on the OLED
  }
}

/* -----------------------------------  
  # Publish messages
--------------------------------------
  Publish MQTT messages 
--------------------------------------*/
void publishMessage(const char* topic, String payload, boolean retained) {
  if (client.publish(topic, payload.c_str(), true))
    Serial.println("Message published [" + String(topic) + "]: " + payload + " - Function: logSoilMoistureSamples");
}
