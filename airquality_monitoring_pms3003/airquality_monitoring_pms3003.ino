/*This project include BME280 and pms3003 sensor module.
   conection details
   bme280----------Sda=33   VCC=3.3V
                   SCL=D2   GND = GND

   CCS811 --------- same as Bme280 pins

   PMS3003--------  PIN 1 = VCC = 5.0V
                    PIN 2 = GND = GND
                    PIN3 = 12 OR, D6
                          (SET when 0 = standby and 1 = operating,)
                    PIN 4 = RX = ESp32 TX
                    PIN 5 = TX = ESP32 RX

                    Analog 34


*/

/****************************************
   Include Libraries
 ****************************************/
#include <WiFi.h>
#include <PubSubClient.h>
#include <pms3003.h>
#include "Adafruit_CCS811.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <Ticker.h>

#define I2C_SDA 33
#define I2C_SCL 32

#define fan 25
#define light 34

int LED_BUILTIN = 2;

#define SEALEVELPRESSURE_HPA (1015.25)
Adafruit_BME280 bme; // I2C 32=SCL, 33=SDA
Adafruit_CCS811 ccs;
Ticker secondTick;
const int timeout = 200;
//int watchdogCount = 0;
volatile int watchdogCount = 0;
//*************************Wifi******************************

#define WIFISSID "" // Put your WifiSSID here
#define PASSWORD "" // Put your wifi password here
#define TOKEN "" // Put your Ubidots' TOKEN
#define MQTT_CLIENT_NAME "sdg" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
//it should be a random and unique ascii string and different from all other devices
/**************************************************
   Deepsleep in seconds
 **************************************************/
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

/****************************************
   Define Constants
 ****************************************/

#define DEVICE_LABEL "esp32" // Assig the device label


char mqttBroker[]  = "industrial.api.ubidots.com";
char payload[100];
char topic[150];
// Space to store values to send
char str_sensor[10];

/****************************************
   Auxiliar Functions
 ****************************************/
WiFiClient ubidots;
PubSubClient client(ubidots);

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");

    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}
void clientPub(char* devise, char* variable_name, unsigned int val) {

  sprintf(topic, "%s%s", "/v1.6/devices/", devise);
  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"%s\":", variable_name); // Adds the variable label

  float value = val;

  /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  dtostrf(value, 4, 2, str_sensor);

  sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor); // Adds the value
  Serial.println("Publishing data to Ubidots Cloud");
  Serial.println(topic);
  Serial.println(payload);
  client.publish(topic, payload);
}
/****************************************
   Main Functions
 ****************************************/
void setup() {
  delay(100);
  WiFi.mode(WIFI_STA);
  delay(100);
  Serial.begin(115200);
  pinMode(fan, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(2000);
  Serial2.begin(9600); //serial for receiving data from PMS3003 16, 17
  pms3003.begin(Serial2);
  delay(1000);
  Wire.begin(I2C_SDA, I2C_SCL); //Start the I2c in those pins
  bme.begin();
  delay(100);
  ccs.begin();
  
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
  WiFi.begin(WIFISSID, PASSWORD);
  // Assign the pin as INPUT
  pinMode(fan, OUTPUT);

  Serial.println();
  Serial.print("Wait for WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    blinkLed(1);
    delay(500);
  }
  
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
  blinkLed(2);
}
 void blinkLed(unsigned int times){
  for (int t=0; t < times; t++ ){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(500); 
  }
 }
 void deepSleep() {
  //https://github.com/esp8266/Arduino/issues/644
  WiFi.mode(WIFI_OFF);
  delay(1);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                 " Seconds");
  esp_deep_sleep_start();
}
void loop() {
  Serial.printf("Watchdog counter= %d\n", watchdogCount);
  watchdogCount = 0;
  digitalWrite(fan, HIGH);
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(20000);
  digitalWrite(LED_BUILTIN, LOW); 
  if (!client.connected()) {
    reconnect();
  }
  if (pms3003.measure() == true)
  {

    float pm25 = pms3003.get_pm2_5();
    float pm10 = pms3003.get_pm10();

    clientPub(DEVICE_LABEL, "pm25", pm25);
    delay(1000);
    clientPub(DEVICE_LABEL, "pm10", pm10);
    delay(1000);

    float pressure = bme.readPressure() / 100.0F;
    float tempC = bme.readTemperature() * 0.90;
    float hum1 = bme.readHumidity();
    float hight1 = bme.readAltitude(SEALEVELPRESSURE_HPA) + 20;
    float lightValue = analogRead(light);
    
    clientPub(DEVICE_LABEL, "Temp", tempC);
    delay(1500);
    blinkLed(1);
    clientPub(DEVICE_LABEL, "Humidity", hum1);
    blinkLed(1);
    delay(1500);
    clientPub(DEVICE_LABEL, "pressure", pressure);
    blinkLed(1);
    delay(1500);
    clientPub(DEVICE_LABEL, "altitude", hight1);
    blinkLed(1);
    delay(1500);
    clientPub(DEVICE_LABEL, "light", lightValue);
    blinkLed(1);
    delay(1500);

    if(ccs.available()){
    float temp = ccs.calculateTemperature();
    if(!ccs.readData()){
      Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.print(ccs.getTVOC());
      Serial.print("ppb   Temp:");
      Serial.println(temp);
    }
    float co2 = ccs.geteCO2();
    float voc = ccs.getTVOC();
    clientPub(DEVICE_LABEL, "co2", co2);
    blinkLed(1);
    delay(1500);
    clientPub(DEVICE_LABEL, "voc", voc);
    blinkLed(1);
    delay(1500);
    client.loop();
    delay(1000);
    digitalWrite(fan, LOW);
    deepSleep();
  }
  Serial.println("No Data");
  delay(2000);
  digitalWrite(fan, LOW);
  deepSleep();
}

}
void ISRwatchdog() {
  watchdogCount++;
  if (watchdogCount == 100) {
    
    //Serial.println();
    Serial.println("the Watchdog bites......!!!!!!!!!!");
    esp_restart();    
    }
  
  }



