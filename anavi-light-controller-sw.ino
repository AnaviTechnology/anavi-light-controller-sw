#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <Wire.h>
#include "Adafruit_HTU21DF.h"

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

//Configure supported I2C sensors
const int sensorHTU21D =  0x40;

// Configure pins
const int pinAlarm = 15;
const int pinButton = 0;
const int pinLedRed = 12;
const int pinLedGreen = 13;
const int pinLedBlue = 14;

int lightRed = 0;
int lightGreen = 0;
int lightBlue = 0;

unsigned long sensorPreviousMillis = 0;
const long sensorInterval = 5000; 

float sensorTemperature = 0;
float sensorHumidity = 0;

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6] = "8080";
char blynk_token[34] = "YOUR_BLYNK_TOKEN";

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();

  //LED
  pinMode(pinAlarm, OUTPUT);
  //Button
  pinMode(pinButton, INPUT);

  //RGB LED Strip
  pinMode(pinLedRed, OUTPUT);
  pinMode(pinLedGreen, OUTPUT);
  pinMode(pinLedBlue, OUTPUT);

  digitalWrite(pinAlarm, HIGH);

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(blynk_token, json["blynk_token"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read



  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_blynk_token);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("ANAVI Light Controller", "")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  digitalWrite(pinAlarm, LOW);

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  // Sensors
  htu.begin();
}

void factoryReset()
{
  if (false == digitalRead(pinButton))
  {
    Serial.println("Hold the button to reset to factory defaults...");
    for (int iter=0; iter<30; iter++)
    {
      digitalWrite(pinAlarm, HIGH);
      delay(100);
      digitalWrite(pinAlarm, LOW);
      delay(100);
    }
    if (false == digitalRead(pinButton))
    {
      Serial.println("Disconnecting...");
      WiFi.disconnect();

      // NOTE: the boot mode:(1,7) problem is known and only happens at the first restart after serial flashing.
      
      Serial.println("Restarting...");
      ESP.restart();
    }
    else
    {
      // Cancel reset to factory defaults
      Serial.println("Reset tp factory defaults cancelled.");
      digitalWrite(pinAlarm, LOW);
    }
  }
}

void handleHTU21D()
{
  Wire.beginTransmission(sensorHTU21D);
  if (0 == Wire.endTransmission())
  {
    float tempTemperature = htu.readTemperature();
    if (1 <= abs(tempTemperature - sensorTemperature))
    {
      sensorTemperature = tempTemperature;
      Serial.print("Temperature: "); 
      Serial.print(sensorTemperature);
      Serial.println("C");
    }

    float tempHumidity = htu.readHumidity();
    if (1 <= abs(tempHumidity - sensorHumidity))
    {
      sensorHumidity = tempHumidity;
      Serial.print("Humidity: "); 
      Serial.print(sensorHumidity);
      Serial.println("%");
    }
  }  
}

void handleSensors()
{
  handleHTU21D();
}

void loop()
{
  // put your main code here, to run repeatedly:

  // Set colors of RGB LED strip
  analogWrite(pinLedRed, lightRed);
  analogWrite(pinLedGreen, lightGreen);
  analogWrite(pinLedBlue, lightBlue);

  unsigned long currentMillis = millis();
  if (sensorInterval <= (currentMillis - sensorPreviousMillis))
  {
    sensorPreviousMillis = currentMillis;
    handleSensors();
  }
  
  // Press and hold the button to reset to factory defaults
  factoryReset();
}
