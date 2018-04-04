#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <PubSubClient.h>

#include <Wire.h>
#include "Adafruit_HTU21DF.h"

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

//Configure supported I2C sensors
const int sensorHTU21D =  0x40;

// Configure pins
const int pinAlarm = 16;
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
char mqtt_server[40] = "iot.eclipse.org";
char mqtt_port[6] = "1883";
char workgroup[32] = "workgroup";

//flag for saving data
bool shouldSaveConfig = false;

// MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

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
          strcpy(workgroup, json["workgroup"]);

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
  WiFiManagerParameter custom_workgroup("workgroup", "workgroup", workgroup, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_workgroup);

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
  strcpy(workgroup, custom_workgroup.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["workgroup"] = workgroup;

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

  // MQTT
  Serial.print("MQTT Server: ");
  Serial.println(mqtt_server);
  Serial.print("MQTT Port: ");
  Serial.println(mqtt_port);

  int mqttPort = atoi(mqtt_port);
  mqttClient.setServer(mqtt_server, mqttPort);
  mqttClient.setCallback(mqttCallback);

  mqttReconnect();

  Serial.println("");
  Serial.println("-----");
  Serial.print("Device ID: ");
  Serial.println(ESP.getChipId());
  Serial.println("-----");
  Serial.println("");
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
      // Clean the file system with configurations
      SPIFFS.format();
      // Restart the board
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

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  // Convert received bytes to a string
  char text[length];
  for (int i = 0; i < length; i++) {
    text[i] = (char)payload[i];
  }
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(text);

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject(text);
  lightRed = ((0 <= data["red"]) && (255 >= data["red"])) ? data["red"] : 0;
  lightGreen = ((0 <= data["green"]) && (255 >= data["green"])) ? data["green"] : 0;
  lightBlue = ((0 <= data["blue"]) && (255 >= data["blue"])) ? data["blue"] : 0;

  Serial.print("Red: ");
  Serial.println(lightRed);
  Serial.print("Green: ");
  Serial.println(lightGreen);
  Serial.print("Blue: ");
  Serial.println(lightBlue);

  // Set colors of RGB LED strip
  analogWrite(pinLedRed, lightRed);
  analogWrite(pinLedGreen, lightGreen);
  analogWrite(pinLedBlue, lightBlue);
}

void mqttReconnect()
{
  // Loop until we're reconnected
  for (int attempt = 0; attempt < 3; ++attempt)
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    //String clientId = "ESP8266Client-";
    //clientId += String(random(0xffff), HEX);
    String clientId = "light-controller-1";
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str()))
    {
      Serial.println("connected");

      // Subscribe to MQTT topic
      char topic[200];
      sprintf(topic,"%s/%d/led", workgroup, ESP.getChipId());
      mqttClient.subscribe(topic);
      break;
      
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishSensorData(const char* subTopic, const char* key, const float value)
{
  StaticJsonBuffer<100> jsonBuffer;
  char payload[100];
  JsonObject& json = jsonBuffer.createObject();
  json[key] = value;
  json.printTo((char*)payload, json.measureLength() + 1);
  char topic[200];
  sprintf(topic,"%s/%d/%s", workgroup, ESP.getChipId() ,subTopic);
  mqttClient.publish(topic, payload, true);
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

      publishSensorData("temperature", "temperature", sensorTemperature);
    }

    float tempHumidity = htu.readHumidity();
    if (1 <= abs(tempHumidity - sensorHumidity))
    {
      sensorHumidity = tempHumidity;
      Serial.print("Humidity: "); 
      Serial.print(sensorHumidity);
      Serial.println("%");

      publishSensorData("humidity", "humidity", sensorHumidity);
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
  mqttClient.loop();

  // Reconnect if there is an issue with the MQTT connection
  if (false == mqttClient.connected())
  {
    mqttReconnect();
  }
  
  /*unsigned long currentMillis = millis();
  if (sensorInterval <= (currentMillis - sensorPreviousMillis))
  {
    sensorPreviousMillis = currentMillis;
    handleSensors();
  }*/
  
  // Press and hold the button to reset to factory defaults
  factoryReset();
}
