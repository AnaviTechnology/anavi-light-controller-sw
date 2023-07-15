#include <FS.h>                   //this needs to be first, or it all crashes and burns...

// If HOME_ASSISTANT_DISCOVERY is defined, the ANAVI Light Controller will
// publish MQTT messages that makes Home Assistant auto-discover the
// device. See:p https://www.home-assistant.io/docs/mqtt/discovery/
//
// This feature requires PubSubClient 2.7.

#define HOME_ASSISTANT_DISCOVERY 1

// If DEBUG is defined additional message will be printed in serial console
#undef DEBUG

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ArduinoOTA.h>

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <PubSubClient.h>        //https://github.com/knolleary/pubsubclient

#include <MD5Builder.h>

#include <Wire.h>
#include "Adafruit_HTU21DF.h"
#include "Adafruit_APDS9960.h"

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

Adafruit_APDS9960 apds;

//Configure supported I2C sensors
const int sensorHTU21D =  0x40;
const int sensorBH1750 = 0x23;

// Configure pins
const int pinAlarm = 16;
const int pinButton = 0;
const int pinLedRed = 12;
const int pinLedGreen = 13;
const int pinLedBlue = 14;

bool power = false;

int tempRed = 255;
int tempGreen = 255;
int tempBlue = 255;
int lightRed = 255;
int lightGreen = 255;
int lightBlue = 255;
int currentRed = 255;
int currentGreen = 255;
int currentBlue = 255;
int brightnessLevel = 255;
char effect[32] = "";
int effectPos = 0;
unsigned long effectPreviousMillis = millis();
const long effectsInterval = 30;

unsigned long sensorPreviousMillis = 0;
const long sensorInterval = 5000;

unsigned long mqttConnectionPreviousMillis = millis();
const long mqttConnectionInterval = 60000;

float sensorTemperature = 0;
float sensorHumidity = 0;
uint16_t sensorAmbientLight = 0;

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "mqtt.eclipse.org";
char mqtt_port[6] = "1883";
char workgroup[32] = "workgroup";
// MQTT username and password
char username[20] = "";
char password[20] = "";
#ifdef HOME_ASSISTANT_DISCOVERY
char ha_name[32+1] = "";        // Make sure the machineId fits.
#endif

//MD5 of chip ID
char machineId[32] = "";

//flag for saving data
bool shouldSaveConfig = false;

// MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

char cmnd_power_topic[44];
char cmnd_color_topic[44];

char stat_power_topic[44];
char stat_color_topic[44];

void apWiFiCallback(WiFiManager *myWiFiManager)
{
    String configPortalSSID = myWiFiManager->getConfigPortalSSID();
    // Print information in the serial output
    Serial.print("Created access point for configuration: ");
    Serial.println(configPortalSSID);
}

//callback notifying us of the need to save config
void saveConfigCallback ()
{
    Serial.println("Should save config");
    shouldSaveConfig = true;
}

void otaStarted()
{
    Serial.println("OTA update started!");
}

void otaFinished()
{
    Serial.println("OTA update finished!");
}

void otaProgress(unsigned int progress, unsigned int total)
{
    Serial.printf("OTA progress: %u%%\r", (progress / (total / 100)));
}

void otaError(ota_error_t error)
{
    Serial.printf("Error [%u]: ", error);
    switch (error)
    {
        case OTA_AUTH_ERROR:
            Serial.println("auth failed!");
            break;
        case OTA_BEGIN_ERROR:
            Serial.println("begin failed!");
            break;
        case OTA_CONNECT_ERROR:
            Serial.println("connect failed!");
            break;
        case OTA_RECEIVE_ERROR:
            Serial.println("receive failed!");
            break;
        case OTA_END_ERROR:
            Serial.println("end failed!");
            break;
    }
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println();

    // Set PWM range 0 to 255
    // According to the documentation the default range is to 1023:
    // https://arduino-esp8266.readthedocs.io/en/latest/reference.html#analog-output
    analogWriteRange(255);

    //LED
    pinMode(pinAlarm, OUTPUT);
    //Button
    pinMode(pinButton, INPUT);

    //RGB LED Strip
    pinMode(pinLedRed, OUTPUT);
    pinMode(pinLedGreen, OUTPUT);
    pinMode(pinLedBlue, OUTPUT);

    digitalWrite(pinAlarm, HIGH);

    // Lowest priority for resuming state: 100% brightness white
    strcpy(effect, "none");
    currentRed      = 255;
    currentGreen    = 255;
    currentBlue     = 255;
    brightnessLevel = 255;

    //read configuration from FS json
    Serial.println("mounting FS...");

    if (SPIFFS.begin())
    {
        Serial.println("mounted file system");
        if (SPIFFS.exists("/config.json")) {
            //file exists, reading and loading
            Serial.println("reading config file");
            File configFile = SPIFFS.open("/config.json", "r");
            if (configFile)
            {
                Serial.println("opened config file");
                const size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes(buf.get(), size);
                DynamicJsonDocument json(1024);
                if (DeserializationError::Ok == deserializeJson(json, buf.get()))
                {
#ifdef DEBUG
                    // Content stored in the memory of the microcontroller contains
                    // sensitive data such as username and passwords therefore
                    // should be printed only during debugging
                    serializeJson(json, Serial);
                    Serial.println("\nparsed json");
#endif

                    strcpy(mqtt_server, json["mqtt_server"]);
                    strcpy(mqtt_port, json["mqtt_port"]);
                    strcpy(workgroup, json["workgroup"]);
                    strcpy(username, json["username"]);
                    strcpy(password, json["password"]);
#ifdef HOME_ASSISTANT_DISCOVERY
                    {
                        const char *s = json["ha_name"];
                        if (!s)
                            s = machineId;
                        snprintf(ha_name, sizeof(ha_name), "%s", s);
                    }
#endif
                }
                else
                {
                    Serial.println("failed to load json config");
                }
            }
        }
        if (SPIFFS.exists("/state.json")) {
            //file exists, reading and loading
            Serial.println("reading state file");
            File stateFile = SPIFFS.open("/state.json", "r");
            if (stateFile)
            {
                Serial.println("opened state file");
                const size_t size = stateFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                stateFile.readBytes(buf.get(), size);
                DynamicJsonDocument json(1024);
                if (DeserializationError::Ok == deserializeJson(json, buf.get()))
                {
                    serializeJson(json, Serial);
                    Serial.println("\nparsed json");

                    strcpy(effect, json["effect"]);
                    currentRed      = json["color"]["r"];
                    currentGreen    = json["color"]["g"];
                    currentBlue     = json["color"]["b"];
                    brightnessLevel = json["brightness"];
                    //since this device just received power assume it should be on
                    power = true;
                }
                else
                {
                    Serial.println("failed to load json state");
                }
            }
        }
    }
    else
    {
        Serial.println("failed to mount FS");
    }
    setColorPins();
    //end read

    // Give a chance to the user to reset wrong configurations
    // right after turning on the board
    waitForFactoryReset();

    // Machine ID
    calculateMachineId();

    // Set MQTT topics
    sprintf(cmnd_power_topic, "cmnd/%s/power", machineId);
    sprintf(cmnd_color_topic, "cmnd/%s/color", machineId);
    sprintf(stat_power_topic, "stat/%s/power", machineId);
    sprintf(stat_color_topic, "stat/%s/color", machineId);

    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length
    WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
    WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
    WiFiManagerParameter custom_workgroup("workgroup", "workgroup", workgroup, 32);
    WiFiManagerParameter custom_mqtt_user("user", "MQTT username", username, 20);
    WiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", password, 20);
#ifdef HOME_ASSISTANT_DISCOVERY
    WiFiManagerParameter custom_mqtt_ha_name("ha_name", "Device name for Home Assistant", ha_name, sizeof(ha_name));
#endif

    char htmlMachineId[200];
    sprintf(htmlMachineId,"<p style=\"color: red;\">Machine ID:</p><p><b>%s</b></p><p>Copy and save the machine ID because you will need it to control the device.</p>", machineId);
    WiFiManagerParameter custom_text_machine_id(htmlMachineId);

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    //add all your parameters here
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_workgroup);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);
    wifiManager.addParameter(&custom_text_machine_id);
#ifdef HOME_ASSISTANT_DISCOVERY
    wifiManager.addParameter(&custom_mqtt_ha_name);
#endif

    //reset settings - for testing
    //wifiManager.resetSettings();

    //set minimu quality of signal so it ignores AP's under that quality
    //defaults to 8%
    //wifiManager.setMinimumSignalQuality();

    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    wifiManager.setTimeout(300);
    digitalWrite(pinAlarm, HIGH);

    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point
    //and goes into a blocking loop awaiting configuration
    wifiManager.setAPCallback(apWiFiCallback);
    // Append the last 5 character of the machine id to the access point name
    String apId(machineId);
    apId = apId.substring(apId.length() - 5);
    String accessPointName = "ANAVI Light Controller " + apId;
    if (!wifiManager.autoConnect(accessPointName.c_str(), ""))
    {
        digitalWrite(pinAlarm, LOW);
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
    strcpy(username, custom_mqtt_user.getValue());
    strcpy(password, custom_mqtt_pass.getValue());
#ifdef HOME_ASSISTANT_DISCOVERY
    strcpy(ha_name, custom_mqtt_ha_name.getValue());
#endif

    //save the custom parameters to FS
    if (shouldSaveConfig)
    {
        Serial.println("saving config");
        DynamicJsonDocument json(1024);
        json["mqtt_server"] = mqtt_server;
        json["mqtt_port"] = mqtt_port;
        json["workgroup"] = workgroup;
        json["username"] = username;
        json["password"] = password;
        #ifdef HOME_ASSISTANT_DISCOVERY
            json["ha_name"] = ha_name;
        #endif

        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile)
        {
            Serial.println("failed to open config file for writing");
        }

        serializeJson(json, Serial);
        serializeJson(json, configFile);
        configFile.close();
        //end save
    }

    Serial.println("local ip");
    Serial.println(WiFi.localIP());

    ArduinoOTA.onStart(otaStarted);
    ArduinoOTA.onEnd(otaFinished);
    ArduinoOTA.onProgress(otaProgress);
    ArduinoOTA.onError(otaError);
    ArduinoOTA.begin();

    // Sensors
    htu.begin();

    // MQTT
    Serial.print("MQTT Server: ");
    Serial.println(mqtt_server);
    Serial.print("MQTT Port: ");
    Serial.println(mqtt_port);
    // Print MQTT Username
    Serial.print("MQTT Username: ");
    Serial.println(username);
    // Hide password from the log and show * instead
    char hiddenpass[20] = "";
    for (size_t charP=0; charP < strlen(password); charP++)
    {
        hiddenpass[charP] = '*';
    }
    hiddenpass[strlen(password)] = '\0';
    Serial.print("MQTT Password: ");
    Serial.println(hiddenpass);

#ifdef HOME_ASSISTANT_DISCOVERY
    Serial.print("Home Assistant sensor name: ");
    Serial.println(ha_name);
#endif

    const int mqttPort = atoi(mqtt_port);
    mqttClient.setServer(mqtt_server, mqttPort);
    mqttClient.setCallback(mqttCallback);

    mqttReconnect();

    Serial.println("");
    Serial.println("-----");
    Serial.print("Machine ID: ");
    Serial.println(machineId);
    Serial.println("-----");
    Serial.println("");

    setupADPS9960();
}

void setupADPS9960()
{
    if(apds.begin())
    {
        //gesture mode will be entered once proximity mode senses something close
        apds.enableProximity(true);
        apds.enableGesture(true);
    }
}

void waitForFactoryReset()
{
    Serial.println("Press button within 2 seconds for factory reset...");
    for (int iter = 0; iter < 20; iter++)
    {
        digitalWrite(pinAlarm, HIGH);
        delay(50);
        if (false == digitalRead(pinButton))
        {
            factoryReset();
            return;
        }
        digitalWrite(pinAlarm, LOW);
        delay(50);
        if (false == digitalRead(pinButton))
        {
            factoryReset();
            return;
        }
    }
}

void factoryReset()
{
    if (false == digitalRead(pinButton))
    {
        Serial.println("Hold the button to reset to factory defaults...");
        bool cancel = false;
        for (int iter=0; iter<30; iter++)
        {
            digitalWrite(pinAlarm, HIGH);
            delay(100);
            if (true == digitalRead(pinButton))
            {
                cancel = true;
                break;
            }
            digitalWrite(pinAlarm, LOW);
            delay(100);
            if (true == digitalRead(pinButton))
            {
                cancel = true;
                break;
            }
        }
        if (false == digitalRead(pinButton) && !cancel)
        {
            digitalWrite(pinAlarm, HIGH);
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
            Serial.println("Reset to factory defaults cancelled.");
            digitalWrite(pinAlarm, LOW);
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    // Convert received bytes to a string
    char text[length + 1];
    snprintf(text, length + 1, "%s", payload);

    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.println(text);

    // Reset effects
    strcpy(effect, "none");

    if (strcmp(topic, cmnd_power_topic) == 0)
    {
        power = strcmp(text, "ON") == 0;
    }
    else if (strcmp(topic, cmnd_color_topic) == 0)
    {
        StaticJsonDocument<200> data;
        deserializeJson(data, text);

        if (data.containsKey("color"))
        {
            const int r = data["color"]["r"];
            const int g = data["color"]["g"];
            const int b = data["color"]["b"];
            currentRed = ((0 <= r) && (255 >= r)) ? r : 0;
            currentGreen = ((0 <= g) && (255 >= g)) ? g : 0;
            currentBlue = ((0 <= b) && (255 >= b)) ? b : 0;
        }
        else if (data.containsKey("brightness"))
        {
            const int brightness = data["brightness"];
            if ( (0 <= brightness) && (255 >= brightness) )
            {
                brightnessLevel = brightness;
            }
        }
        else if (data.containsKey("effect"))
        {
            if (strcmp(effect, data["effect"]) != 0)
            {
                strcpy(effect, data["effect"]);
                effectPos = 0;
                effectPreviousMillis = millis();
            }
           
        }
        saveColors();
        calculateBrightness();
        if (data.containsKey("state"))
        {
            // Set variable power to true or false depending on the state
            power = (data["state"] == "ON");
        }
        else if (data.containsKey("effect"))
        {
            // Turn on the power if any effect has been specified
            power = true;
        }
        else if (data.containsKey("brightness") || data.containsKey("color"))
        {
            // Turn on if any of the colors is greater than 0
            // Only if *either* color or brightness have been set.
            power = ( (0 < lightRed) || (0 < lightGreen) || (0 < lightBlue) );
        }
    }

    saveState();

    publishState();

    setColorPins();
}

void setColorPins()
{
    calculateBrightness();

    Serial.print("Red: ");
    Serial.println(lightRed);
    Serial.print("Green: ");
    Serial.println(lightGreen);
    Serial.print("Blue: ");
    Serial.println(lightBlue);
    Serial.print("Power: ");
    Serial.println(power);

    // Set colors of RGB LED strip
    if (power)
    {
        if (strcmp(effect, "switch_transition") == 0)
        {
            effectPos = 0;
            return;
        }
        analogWrite(pinLedRed, lightRed);
        analogWrite(pinLedGreen, lightGreen);
        analogWrite(pinLedBlue, lightBlue);
    }
    else
    {
        analogWrite(pinLedRed, 0);
        analogWrite(pinLedGreen, 0);
        analogWrite(pinLedBlue, 0);
    }
}

void saveColors()
{
    tempRed = lightRed;
    tempGreen = lightGreen;
    tempBlue = lightBlue;
}

void calculateBrightness()
{
    unsigned int maximumBrightness = 255;
    lightRed = (currentRed * brightnessLevel) / maximumBrightness;
    lightBlue = (currentBlue * brightnessLevel) / maximumBrightness;
    lightGreen = (currentGreen * brightnessLevel) / maximumBrightness;
}

void processEffects()
{
    if (!power)
    {
        return;
    }

    if (strcmp(effect, "none") == 0)
    {
        return;
    }
    else if (strcmp(effect, "switch_transition") == 0)
    {
        if (effectPos == 256)
        {
            return;
        }

        if (lightRed > tempRed)
        {
            tempRed++;
        }
        else if (lightRed < tempRed) {
            tempRed--;
        }

        if (lightGreen > tempGreen)
        {
            tempGreen++;
        }
        else if (lightGreen < tempGreen) {
            tempGreen--;
        }

        if (lightBlue > tempBlue)
        {
            tempBlue++;
        }
        else if (lightBlue < tempBlue) {
            tempBlue--;
        }

        effectPos += 1;
    
        analogWrite(pinLedRed, tempRed);
        analogWrite(pinLedGreen, tempGreen);
        analogWrite(pinLedBlue, tempBlue);

        return;
    }
    else if (strcmp(effect, "rainbow1") == 0)
    {
        int pos = effectPos;
        if  (pos < 256)
        {
            currentRed = pos;
            currentGreen = 255 - pos;
            currentBlue = 0;
        }
        else if (pos < 512)
        {
            currentRed = 255 - (pos - 256);
            currentGreen = 0;
            currentBlue = pos - 256;
        }
        else
        {
            currentRed = 0;
            currentGreen = pos - 512;
            currentBlue = 255 - (pos - 512);
        }

        effectPos += 1;
        if (effectPos == 766) // 255 * 3 + 1
        {
            effectPos = 0;
        }
    }
    else if (strcmp(effect, "rainbow2") == 0)
    {
        int pos = effectPos;
        if  (pos < 256)
        {
            currentRed = 0;
            currentGreen = 255 - pos;
            currentBlue = pos;
        }
        else if (pos < 512)
        {
            currentRed = pos - 256;
            currentGreen = 0;
            currentBlue = 255 - (pos - 256);
        }
        else
        {
            currentRed = 255 - (pos - 512);
            currentGreen = pos - 512;
            currentBlue = 0;
        }

        effectPos += 1;
        if (effectPos == 766)
        {
            effectPos = 0;
        }
    }
    else if (strcmp(effect, "rainbow3") == 0)
    {
        int pos = effectPos;
        if  (pos < 256)
        {
            currentRed = pos;
            currentGreen = 255 - pos;
            currentBlue = 255;
        }
        else if (pos < 512)
        {
            currentRed = 255;
            currentGreen = pos - 256;
            currentBlue = 255 - (pos - 256);
        }
        else
        {
            currentRed = 255 - (pos - 512);
            currentGreen = 255;
            currentBlue = pos - 512;
        }

        effectPos += 1;
        if (effectPos == 766) // 255 * 3 + 1
        {
            effectPos = 0;
        }
    }
    else if (strcmp(effect, "rainbow4") == 0)
    {
        int pos = effectPos;
        if  (pos < 256)
        {
            currentRed = 255 - pos;
            currentGreen = pos;
            currentBlue = 255;
        }
        else if (pos < 512)
        {
            currentRed = pos - 256;
            currentGreen = 255;
            currentBlue = 255 - (pos - 256);
        }
        else
        {
            currentRed = 255;
            currentGreen = 255 - (pos - 512);
            currentBlue = pos - 512;
        }

        effectPos += 1;
        if (effectPos == 766)
        {
            effectPos = 0;
        }
    }
    setColorPins();
}

void calculateMachineId()
{
    MD5Builder md5;
    md5.begin();
    char chipId[25];
    sprintf(chipId,"%d",ESP.getChipId());
    md5.add(chipId);
    md5.calculate();
    md5.toString().toCharArray(machineId, 32);
}

void mqttReconnect()
{
    // Generate unique MQTT client ID based on the machine ID
    char clientId[23 + sizeof(machineId)];
    snprintf(clientId, sizeof(clientId), "anavi-light-controller-%s", machineId);

    // Loop until we're reconnected
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (true == mqttClient.connect(clientId, username, password))
        {
            Serial.println("connected");

            // Subscribe to MQTT topics
            mqttClient.subscribe(cmnd_power_topic);
            mqttClient.subscribe(cmnd_color_topic);
            publishDiscoveryState();
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

#ifdef HOME_ASSISTANT_DISCOVERY
bool publishSensorDiscovery(const char *config_key,
                            const char *device_class,
                            const char *name_suffix,
                            const char *state_topic,
                            const char *unit,
                            const char *value_template,
                            bool binary = false)
{
    DynamicJsonDocument json(1024);

    String sensorType = "sensor";
    if (true == binary)
    {
        sensorType = "binary_sensor";
    }
    else
    {
        // Unit of measurement is supported only by non-binary sensors
        json["unit_of_measurement"] = unit;
        json["value_template"] = value_template;
    }
    static char topic[48 + sizeof(machineId)];
    snprintf(topic, sizeof(topic),
             "homeassistant/%s/%s/%s/config", sensorType.c_str(), machineId, config_key);

    if (0 < strlen(device_class))
    {
      json["device_class"] = device_class;
    }
    json["name"] = String(ha_name) + " " + name_suffix;
    json["unique_id"] = String("anavi-") + machineId + "-" + config_key;
    json["state_topic"] = String(workgroup) + "/" + machineId + "/" + state_topic;

    json["device"]["identifiers"] = machineId;
    json["device"]["manufacturer"] = "ANAVI Technology";
    json["device"]["model"] = "ANAVI Light Controller";
    json["device"]["name"] = ha_name;
    json["device"]["sw_version"] = ESP.getSketchMD5();

    JsonArray connections = json["device"].createNestedArray("connections").createNestedArray();
    connections.add("mac");
    connections.add(WiFi.macAddress());

    Serial.print("Home Assistant discovery topic: ");
    Serial.println(topic);

    int payload_len = measureJson(json);
    if (!mqttClient.beginPublish(topic, payload_len, true))
    {
        Serial.println("beginPublish failed!\n");
        return false;
    }

    if (serializeJson(json, mqttClient) != payload_len)
    {
        Serial.println("writing payload: wrong size!\n");
        return false;
    }

    if (!mqttClient.endPublish())
    {
        Serial.println("endPublish failed!\n");
        return false;
    }

    return true;
}


bool publishLightDiscovery()
{
    DynamicJsonDocument json(1024);

    static char topic[48 + sizeof(machineId)];
    snprintf(topic, sizeof(topic),
             "homeassistant/light/%s/light/config",  machineId);

    json["schema"] = "json";
    json["brightness"] = true;
    json["rgb"] = true;
    json["effect"] = true;
    JsonArray effects = json.createNestedArray("effect_list");
    effects.add("none");
    effects.add("switch_transition");
    effects.add("rainbow1");
    effects.add("rainbow2");
    effects.add("rainbow3");
    effects.add("rainbow4");

    json["name"] = String(ha_name) + String(" ANAVI Light Controller");
    json["unique_id"] = String("anavi-") + machineId + String("-rgb");
    json["command_topic"] = String("cmnd/") + machineId + String("/color");
    json["state_topic"] = String("stat/") + machineId + String("/#");

    json["device"]["identifiers"] = machineId;
    json["device"]["manufacturer"] = "ANAVI Technology";
    json["device"]["model"] = "ANAVI Light Controller";
    json["device"]["name"] = ha_name;
    json["device"]["sw_version"] = ESP.getSketchMD5();

    JsonArray connections = json["device"].createNestedArray("connections").createNestedArray();
    connections.add("mac");
    connections.add(WiFi.macAddress());

    Serial.print("Home Assistant discovery topic: ");
    Serial.println(topic);

    int payload_len = measureJson(json);
    if (!mqttClient.beginPublish(topic, payload_len, true))
    {
        Serial.println("beginPublish failed!\n");
        return false;
    }

    if (serializeJson(json, mqttClient) != payload_len)
    {
        Serial.println("writing payload: wrong size!\n");
        return false;
    }

    if (!mqttClient.endPublish())
    {
        Serial.println("endPublish failed!\n");
        return false;
    }

    return true;
}

#endif

void publishDiscoveryState()
{
#ifdef HOME_ASSISTANT_DISCOVERY

    // Publish discovery information about the core feature
    publishLightDiscovery();

    // Publish discovery information if any I2C sensors are attached
    static char payload[300];
    static char topic[80];

    String homeAssistantTempScale = "°C";

    if (isSensorAvailable(sensorHTU21D))
    {
        publishSensorDiscovery("temp",
                               "temperature",
                               "Temperature",
                               "temperature",
                               homeAssistantTempScale.c_str(),
                               "{{ value_json.temperature | round(1) }}");

        publishSensorDiscovery("humidity",
                               "humidity",
                               "Humidity",
                               "humidity",
                               "%",
                               "{{ value_json.humidity | round(0) }}");
    }

    if (isSensorAvailable(sensorBH1750))
    {
        publishSensorDiscovery("light",
                       "illuminance",
                       "Light",
                       "light",
                       "Lux",
                       "{{ value_json.light }}");
    }

#endif
}

void publishState()
{
    DynamicJsonDocument json(1024);
    const char* state = power ? "ON" : "OFF";
    json["state"] = state;

    char stat_power_payload[150];
    serializeJson(json, stat_power_payload);

    Serial.print("[");
    Serial.print(stat_power_topic);
    Serial.print("] ");
    Serial.println(stat_power_payload);

    mqttClient.publish(stat_power_topic, stat_power_payload, true);

	// Continue with the rest of the data for the colors
    json["brightness"] = brightnessLevel;
    json["effect"] = effect;

    json["color"]["r"] = power ? currentRed : 0;
    json["color"]["g"] = power ? currentGreen : 0;
    json["color"]["b"] = power ? currentBlue : 0;

    int payloadLength = measureJson(json);
    if (mqttClient.beginPublish(stat_color_topic, payloadLength, true))
    {
        if (serializeJson(json, mqttClient) == payloadLength)
        {
            mqttClient.endPublish();
        }
    }

    char stat_color_payload[150];
    serializeJson(json, stat_color_payload);

    Serial.print("[");
    Serial.print(stat_color_topic);
    Serial.print("] ");
    Serial.println(stat_color_payload);

}

void saveState()
{
    Serial.println("saving config");
    DynamicJsonDocument json(1024);
    const char* powerState = power ? "ON" : "OFF";
    json["state"]      = powerState;
    json["brightness"] = brightnessLevel;
    json["effect"]     = effect;

    json["color"]["r"] = power ? currentRed   : 0;
    json["color"]["g"] = power ? currentGreen : 0;
    json["color"]["b"] = power ? currentBlue  : 0;

    File stateFile = SPIFFS.open("/state.json", "w");
    if (!stateFile)
    {
        Serial.println("failed to open state file for writing");
    }

    serializeJson(json, Serial);
    serializeJson(json, stateFile);
    stateFile.close();
}

void publishSensorData(const char* subTopic, const char* key, const float value)
{
    StaticJsonDocument<100> json;
    json[key] = value;
    char payload[100];
    serializeJson(json, payload);
    char topic[200];
    sprintf(topic,"%s/%s/%s", workgroup, machineId, subTopic);
    mqttClient.publish(topic, payload, true);
}

void publishSensorData(const char* subTopic, const char* key, const String& value)
{
    StaticJsonDocument<100> json;
    json[key] = value;
    char payload[100];
    serializeJson(json, payload);
    char topic[200];
    sprintf(topic,"%s/%s/%s", workgroup, machineId, subTopic);
    mqttClient.publish(topic, payload, true);
}

bool isSensorAvailable(int sensorAddress)
{
    // Check if I2C sensor is present
    Wire.beginTransmission(sensorAddress);
    return 0 == Wire.endTransmission();
}

void handleHTU21D()
{
    // Check if temperature has changed
    const float tempTemperature = htu.readTemperature();
    if (0.1 <= fabs(tempTemperature - sensorTemperature))
    {
        // Print new temprature value
        sensorTemperature = tempTemperature;
        Serial.print("Temperature: ");
        Serial.print(sensorTemperature);
        Serial.println("C");

        // Publish new temperature value through MQTT
        publishSensorData("temperature", "temperature", sensorTemperature);
    }

    // Check if humidity has changed
    const float tempHumidity = htu.readHumidity();
    if (1 <= abs(tempHumidity - sensorHumidity))
    {
        // Print new humidity value
        sensorHumidity = tempHumidity;
        Serial.print("Humidity: ");
        Serial.print(sensorHumidity);
        Serial.println("%");

        // Publish new humidity value through MQTT
        publishSensorData("humidity", "humidity", sensorHumidity);
    }
}

void sensorWriteData(int i2cAddress, uint8_t data)
{
    Wire.beginTransmission(i2cAddress);
    Wire.write(data);
    Wire.endTransmission();
}

void handleBH1750()
{
    Wire.begin();
    // Power on sensor
    sensorWriteData(sensorBH1750, 0x01);
    // Set mode continuously high resolution mode
    sensorWriteData(sensorBH1750, 0x10);

    uint16_t tempAmbientLight;

    Wire.requestFrom(sensorBH1750, 2);
    tempAmbientLight = Wire.read();
    tempAmbientLight <<= 8;
    tempAmbientLight |= Wire.read();
    // s. page 7 of datasheet for calculation
    tempAmbientLight = tempAmbientLight/1.2;

    if (1 <= abs(tempAmbientLight - sensorAmbientLight))
    {
        // Print new humidity value
        sensorAmbientLight = tempAmbientLight;
        Serial.print("Light: ");
        Serial.print(tempAmbientLight);
        Serial.println("Lux");

        // Publish new humidity value through MQTT
        publishSensorData("light", "light", sensorAmbientLight);
    }
}

void detectGesture()
{
    //read a gesture from the device
    const uint8_t gestureCode = apds.readGesture();
    // Skip if gesture has not been detected
    if (0 == gestureCode)
    {
        return;
    }
    String gesture = "";
    switch(gestureCode)
    {
    case APDS9960_DOWN:
        gesture = "down";
        break;
    case APDS9960_UP:
        gesture = "up";
        break;
    case APDS9960_LEFT:
        gesture = "left";
        break;
    case APDS9960_RIGHT:
        gesture = "right";
        break;
    }
    Serial.print("Gesture: ");
    Serial.println(gesture);
    // Publish the detected gesture through MQTT
    publishSensorData("gesture", "gesture", gesture);
}

void handleSensors()
{
    if (isSensorAvailable(sensorHTU21D))
    {
        handleHTU21D();
    }
    if (isSensorAvailable(sensorBH1750))
    {
        handleBH1750();
    }
}

void loop()
{
    // put your main code here, to run repeatedly:
    ArduinoOTA.handle();

    mqttClient.loop();

    const unsigned long effectMillis = millis();
    if (effectsInterval <= (effectMillis - effectPreviousMillis))
    {
        effectPreviousMillis = effectMillis;
        processEffects();
    }

    // Reconnect if there is an issue with the MQTT connection
    const unsigned long mqttConnectionMillis = millis();
    if ( (false == mqttClient.connected()) && (mqttConnectionInterval <= (mqttConnectionMillis - mqttConnectionPreviousMillis)) )
    {
        mqttConnectionPreviousMillis = mqttConnectionMillis;
        mqttReconnect();
    }

    const unsigned long currentMillis = millis();
    if (sensorInterval <= (currentMillis - sensorPreviousMillis))
    {
        sensorPreviousMillis = currentMillis;
        handleSensors();
    }

    // Handle gestures at a shorter interval
    if (isSensorAvailable(APDS9960_ADDRESS))
    {
        detectGesture();
    }

    // Press and hold the button to reset to factory defaults
    factoryReset();
}
