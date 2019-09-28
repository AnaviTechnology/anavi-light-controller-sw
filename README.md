# anavi-light-controller-sw

Open source Arduino sketch for the smart WiFi dev board [ANAVI Light Controller](https://anavi.technology/).

ANAVI Light Controlloer is an ESP8266-powered, open source, Wi-Fi dev board for controlling colors of 12V RGB LED strip. It also has 3 slots for attaching I2C sensor modules for detecting temperature, humidity, light, barometric pressure, gestures, etc. 

# User's Manual

[ANAVI Light Controller User's Manual](https://github.com/AnaviTechnology/anavi-docs/blob/master/anavi-light-controller/anavi-light-controller.md)

# Dependencies

The default firmware of ANAVI Light Controller depends on the following Arduino libraries:

* WiFiManager by tzapu (version 0.12.0)
* ArduinoJson by Benoit Blanchon (version 6.11.2)
* PubSubClient by Nick O'Leary (version 2.7.0)
* Adafruit HTU21DF Library by Adafruit (version 1.0.1)
* Adafruit APDS9960 Library by Adafruit (version 1.0.5)
* Adafruit Unified Sensor by Adafruit (version 1.0.2)
* Adafruit BMP085 Unified (version 1.0.0)
* NTPClient by Fabrice Weinberg (version 3.1.0)
