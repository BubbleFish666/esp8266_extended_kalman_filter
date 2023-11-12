# esp8266_extended_kalman_filter

This repo uses [ArduinoEigen](https://github.com/hideakitai/ArduinoEigen) to implement an extended Kalman Filter on ESP8266 for estimating the orientation of the device.

Don't forget to add a `register_wifi_ap.h` file to `include/`, which should look like:
```
#include <ESP8266WiFiMulti.h>

void registerWiFiAp(ESP8266WiFiMulti& wifiMulti) {
  wifiMulti.addAP("SSID0", "password0");
  wifiMulti.addAP("SSID1", "password1");
  wifiMulti.addAP("SSID2", "password2");
}
```