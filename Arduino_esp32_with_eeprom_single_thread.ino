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
#include <EEPROM.h>
//#include<time.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
//#define DHTpin 2   //Set DHT pin as GPIO2
//DHTesp dht;

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
bool publishMessageToMqtt = false;

int sensorPin = 5;
String machineSiteId = "HDR-01";
const char* machineTopic = "PMP_data_test_HDR-01";
int rsetPin=6;
const unsigned long interval = 60000;  // 1 minute in milliseconds
unsigned long lastMinuteMillis = 0; // Time of the last minute
int count1=0;

Preferences preferences;


LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/******* MQTT Broker Connection Details *******/
const char* mqtt_server = " d08900cfdef5463098201f44a1532917.s2.eu.hivemq.cloud";
const char* mqtt_username = "ankit1";
const char* mqtt_password = "Ankit@123";
const int mqtt_port =8883;

/******* EEPROM Details *******/
int eepromAddress = 0;
int eepromValueRead;

WiFiClientSecure espClient;
int packetCount = 10;

/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient client(espClient);

int lastSensorValue = LOW;

#define MQTT_KEEP_ALIVE_INTERVAL 86400 // 24 hours
const unsigned long mqtt_send_interval = 5000;
unsigned long last_mqtt_send_time = 0;
long feed = 0;


#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
static const char *root_ca PROGMEM = R"EOF(

-----BEGIN CERTIFICATE-----
MIIFnjCCA4YCCQDm7/FV5GARZDANBgkqhkiG9w0BAQ0FADCBqTELMAkGA1UEBhMC
REUxEDAOBgNVBAgMB0JhdmFyaWExETAPBgNVBAcMCEVybGFuZ2VuMRcwFQYDVQQK
DA5Tb2Z0YmxhZGUgR21iSDEZMBcGA1UECwwQd3d3LnNvZnRibGFkZS5kZTEcMBoG
A1UEAwwTU29mdGJsYWRlIFJvb3QgQ0EgMTEjMCEGCSqGSIb3DQEJARYUY29udGFj
dEBzb2Z0YmxhZGUuZGUwHhcNMjQwNDA2MTczNDUyWhcNMjQwNjA1MTczNDUyWjB4
MQswCQYDVQQGEwJERTEMMAoGA1UECAwDTlJXMREwDwYDVQQHDAhFcmxhbmdlbjES
MBAGA1UECgwJU29mdGJsYWRlMRIwEAYDVQQDDAlTb2Z0YmxhZGUxIDAeBgkqhkiG
9w0BCQEWEW5pa29Ac29mdGJsYWRlLmRlMIICIjANBgkqhkiG9w0BAQEFAAOCAg8A
MIICCgKCAgEAqUE9p71j0zzCIQJlacOrA/k7EvNO98jdVUS77opS2ONUg6mwWZ8H
QJrJbce00xN8RPTPfKG4gmpWYY9JEIl9U4baYGAjMALiEaInxODZ8TF1IqyKdmyL
9Q7t+Fd+goGZy1rZUeLDxU40OXZ0okJu1vc5lrXkVDQtscPeEvB/HORsV9+9LM0v
hUIx+S+SmF0CCh/oNWlmIW5lL0kViD49YC8jaULLzEgljh4rV3rQwKp40MBmFJkt
E3eMa3vfjHwDTlsTY84gLhAY5gYPzlTJPCXFrMlmiSGc5/tl3TTyT1+TJAEP3pGJ
ktRbEmlQi5FwOaGgSfbUj+5Sq1t+ZLp0stmB2wYh0izN/zklTB9FJMN93o/dbqVm
TS4ygxiUD3wuxL/yQryMoSHkPgZQfEEd4fHc22eetCrBwJRgs4wP528WfZ/Kup8o
SpAXexJlptFfiLY00XzXNxDWfzwEKBbJ5JUnNRIvzZ2Pul1ZYZ+/FKjjLUG7hmBW
nkkmUcu3758bbCRccwqiIlLcQrzx48TnOkiIpEoIEPKpBFkwpURLYMyrMfK9CSFO
l28/xYqf7WNdywABCmqCIiQIyvoIHDvYF3HR3fcEJ/4kqAA+Zt0A6X3uK2Bvsr9h
LuwaZnKPC2r3ghvqKIvTzvztj7isL4RRTrkf/0hZ/Xg5MIxsZCsRZ3cCAwEAATAN
BgkqhkiG9w0BAQ0FAAOCAgEAgulmMp81NZJbGeFUeQ5rBoPWtCQ7jLVR/8ZuJ6gd
+9QJESqeLuhCvv/qvNZh7Mfb2opICymgCPOyQEq7lDEcpVUC3H6BcbVuoNifnx+y
oc8Mzb5WZHSeGm6CxsMxkePkMbe+ZOkxEC7H8Sag5KtIUhAp7Zk5GYWzhjF2GglR
kjCtv+RD8XIizkl0Ni+FCYfU26N+7xNxkFUjl4mQqacoZIhO2+rBtex4pmDauAsl
v8egAG0ViYIbViLteFrrLHDwNVzLYP5mqYba8vOEUv4OQaY/p0ey2hlqeGHYeeeQ
Vm7mNTZSOEvSKxgvZBPHZohXjAzgYwhXGJ8MoCu3cUstds3CoAfRe039Vb3HqGkt
nOdi/5srBf0KjBP+sgXSJRFN5pH4RSZmN9UDwylxnPPfCIyqxv/T5qREpW5OL5uR
sfSdLJxGDioIPcyXX/8pLi4aZE/4yvv4/lrSXayloDhmnm/vQ6ObJT5SroL+LABL
nD7fHyBuGArv5KIaqM/R++xhwgCSVHAsT+y1bkJBkG0LbC4Wul87UUvCJ1i/5KjZ
1v2eiUDrzKRd+xWnv3VjMVMcXI6m8tQn7vR3Yf+qN/nwUF1vKxwalouks7Dy+jCC
st34AZPO6i4CUn8c9M6K7b8aefie0c2x6WozXd1VNg8Um3hv/b0ug73TdsSciXZP
/ec=
-----END CERTIFICATE-----
)EOF";

void setup_wifi() {
  delay(10);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());
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

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// void callback(char* topic, byte* payload, unsigned int length) {
//   Serial.println("calling callback");
//   String incommingMessage = "";
//   for (int i = 0; i < length; i++) incommingMessage+=(char)payload[i];

//   Serial.println("Message arrived ["+String(topic)+"]"+incommingMessage);

//   DynamicJsonDocument doc(1024);
//   deserializeJson(doc, incommingMessage);
//   String siteId = doc["siteId"];
//   int count = doc["count"];

//   // Use the extracted count value in your code
//   if(machineSiteId==siteId){
//     Serial.println("Count: " + String(count));
//     globalCount = count;
//   }
// }

void publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str(), true))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}
void setup() {

  pinMode(led, OUTPUT); //set up LED
  Serial.begin(115200);
  while (!Serial) delay(1);
  setup_wifi();

  // #ifdef ESP8266
    espClient.setInsecure();
  // #else
  //   espClient.setCACert(root_ca);      // enable this line and the the "certificate" code for secure connection
  // #endif

  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(MQTT_KEEP_ALIVE_INTERVAL);
  reconnect();
  
  timeClient.begin();
  timeClient.setTimeOffset(utcOffsetInSeconds);
  // client.setCallback(callback);
  client.subscribe(machineTopic);
  count =0 ;

  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(machineSiteId);
 
  // EEPROM.begin(512);      // initialize EEPROM
  preferences.begin("my-app", false);   // initialize perfernces

  

}
void loop() {
   timeClient.update();
  
 // Get current day
  int currentDay = timeClient.getDay();
  // Get current hour
  int currentHour = timeClient.getHours();

  unsigned long epochTime = timeClient.getEpochTime();
  String formattedTime = timeClient.getFormattedTime();

  lcd.setCursor(0,0);
  lcd.print(machineSiteId);
  lcd.setCursor(7,0);
  lcd.print(formattedTime);
  
  int sensorValue = digitalRead(sensorPin);
  int rset=digitalRead(rsetPin);
  if(sensorValue==HIGH  && lastSensorValue == LOW)
  {
      if(count==0){
        count = preferences.getUInt("counter", 0);
        // count=readEEPROM();
        // count = globalCount;
        // count = 0;
      }
      if(count==1){
          // count=globalCount+1; 
          count = 1;
      }
      count++;    
      preferences.putUInt("counter", count);
      // writeEEPROM(count);
      count1++;
      
      unsigned long milli = millis();

      // Check if a minute has passed

      // if (milli - lastMinuteMillis >= interval) {
      //   // Calculate events per minute
      //   float eventsPerMinute = count1 / ((milli - lastMinuteMillis) / 60000.0);
      //   lcd.setCursor(12,3);
      //   lcd.print("F");
      //   lcd.setCursor(13,3);
      //   lcd.print(count1);
      //   feed = count1;
        
      //   // Reset event count and update last minute time
      //   count1 = 0;
      //   lastMinuteMillis = milli;
      // }

    publishMessageToMqtt = true;
    Serial.println(count);
    Serial.println("feed is ");
    Serial.println(feed);
  
  }
  lastSensorValue = sensorValue;

  lcd.setCursor(0,3);
  lcd.print(count);
   
//  if(count!=0 && count%packetCount==0 && publishMessageToMqtt)
//     {
      if (millis() - last_mqtt_send_time >= mqtt_send_interval && publishMessageToMqtt) {
      if (!client.connected()) {
      reconnect();}
      publishMqttMessage(count,epochTime);
      publishMessageToMqtt=false;
      last_mqtt_send_time = millis();
      }
    // }
}

void publishMqttMessage(int count, long epochTime) {
  DynamicJsonDocument doc(1024);
  doc["deviceId"] = "nodemcu";
  doc["siteId"] = machineSiteId;
  doc["count"] = count;
  doc["publishedTime"] = epochTime;
  Serial.print(epochTime);

  char mqtt_message[400];
  serializeJson(doc, mqtt_message);

  publishMessage(machineTopic, mqtt_message, true);
  publishMessageToMqtt = false;
}

int readEEPROM(){
  int eepromValueRead;
  // EEPROM.begin(512); // Initialize EEPROM with size
  EEPROM.get(eepromAddress, eepromValueRead); // Read value from EEPROM
  EEPROM.end();
  return eepromValueRead;
}

void writeEEPROM(int count){
  // EEPROM.begin(512); // Initialize EEPROM with size
  EEPROM.put(eepromAddress, count); // Write value to EEPROM
  EEPROM.commit(); // Commit changes
  EEPROM.end(); // Free EEPROM memory
}
