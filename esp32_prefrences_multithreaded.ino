#ifdef ESP8266
 #include <ESP8266WiFi.h>
 #else
 #include <WiFi.h>
#endif

//#include "DHTesp.h"
#include <ArduinoJson.h>
#include<SPI.h>
#include<Wire.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_I2C.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Preferences.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


/**** LED Settings *******/
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
const int led = 2;
const long utcOffsetInSeconds = 19800; // UTC offset for Indian Standard Time (IST) in seconds (5 hours + 30 minutes)

//Set LED pin as GPIO5
/****** WiFi Connection Details *******/
const char* ssid = "ankit";
const char* password = "anki1234";
unsigned int globalCount;
unsigned int count;
unsigned int mqqtLastCount = 0;
unsigned int nvsLastCount = 0;

unsigned long epochTime;

int sensorPin = 5;
String machineSiteId = "HDR-01";
const char* machineTopic = "PMP_data_test_HDR-01";



Preferences preferences;


LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/******* MQTT Broker Connection Details *******/
const char* mqtt_server = " d08900cfdef5463098201f44a1532917.s2.eu.hivemq.cloud";
const char* mqtt_username = "ankit1";
const char* mqtt_password = "Ankit@123";
const int mqtt_port =8883;

bool wiFiConnected = false;
bool mqqtConnected = false;

WiFiClientSecure espClient;


/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient client(espClient);

int lastSensorValue = LOW;

#define MQTT_KEEP_ALIVE_INTERVAL 86400 // 24 hours
const unsigned long mqtt_send_interval = 5000;

DynamicJsonDocument doc(1024);


#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];


void setup_wifi() {
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());
  wiFiConnected = true;
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");

      client.subscribe(machineTopic);   // subscribe the topics here
      mqqtConnected = true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void publishMessage(const char* topic, String payload , boolean retained){
  if(!client.connected())
    reconnect();
  if (client.publish(topic, payload.c_str(), true))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}
void setup() {

  pinMode(led, OUTPUT); //set up LED
  Serial.begin(115200);
  while (!Serial) delay(1);
  setup_wifi();

  espClient.setInsecure();
 
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);
  reconnect();
  
  timeClient.begin();
  timeClient.setTimeOffset(utcOffsetInSeconds);
  // client.setCallback(callback);
  client.subscribe(machineTopic);
  count = 0 ;

  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(machineSiteId);
  preferences.begin("my-app", false);   // initialize perfernces

  xTaskCreatePinnedToCore(
      publishMqttMessage, // Task function
      "publishMqttMessage", // Task name
      10000, // Stack size
      NULL, // Task parameters (pointer to the dynamically allocated memory)
      1, // Priority (higher priority than loop task)
      NULL, // Task handle
      0 // Core to run the task on (0 or 1)
    );
}

void loop() {
   timeClient.update();
  
 // Get current day
  int currentDay = timeClient.getDay();
  // Get current hour
  int currentHour = timeClient.getHours();

  epochTime = timeClient.getEpochTime();
  String formattedTime = timeClient.getFormattedTime();

  lcd.setCursor(0,0);
  lcd.print(machineSiteId);
  lcd.setCursor(7,0);
  lcd.print(formattedTime);
  
  int sensorValue = digitalRead(sensorPin);
  
  if(sensorValue==HIGH  && lastSensorValue == LOW)
  {
      if(count==0){
        count = preferences.getUInt("counter", 0);
      }

    count++;    
    Serial.println(count);
    
  }
  lastSensorValue = sensorValue;

  lcd.setCursor(0,3);
  lcd.print(count);
    
}

void publishMqttMessage(void * parameter) {
  while(true){

    Serial.println(" WiFi status: " + String(WiFi.status()));
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Not connected to Wi-Fi. Reconnecting...");
      setup_wifi(); // Reconnect to Wi-Fi
      reconnect(); // Reconnect to MQTT broker
    }
    Serial.println(" count from another thread is "+ String(count)+" mqqtLastCount is "+String(mqqtLastCount));
    if(mqqtLastCount!=count){
      preferences.putUInt("counter", count);
      doc.clear();
      doc["deviceId"] = "nodemcu";
      doc["siteId"] = machineSiteId;
      doc["count"] = count;
      doc["publishedTime"] = epochTime;
      
      char mqtt_message[400];
      serializeJson(doc, mqtt_message);
      mqqtLastCount = count;
      publishMessage(machineTopic, mqtt_message, true);}
    vTaskDelay(pdMS_TO_TICKS(mqtt_send_interval)); 
  }
}



