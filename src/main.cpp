// #include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
#include "ArduinoEigen/ArduinoEigenDense.h"
#include "extended_kalman_filter.h"
#include "register_wifi_ap.h"


WiFiUDP udp;
// specify a port to listen to incoming packets
unsigned int localUdpPort = 4210;
// a buffer for incoming packets
char incomingPacket[256];
// define a reply message
char replyPacket[] = "Hi! Got the message :-)";

// wifi multi class
ESP8266WiFiMulti wifiMulti;
// WiFi connect timeout per AP. Increase when connecting takes longer.
const uint32_t connectTimeoutMs = 5000;
IPAddress current_ip;

ExtendedKalmanFilter ekf(Eigen::Vector3f(0.1, 0.1, M_PI / 4));
Eigen::Vector3f x_c_1;

// connect to available wifi
void connect2WiFi() {
  // set in station mode
  WiFi.mode(WIFI_STA);

  // register multi wifi networks
  registerWiFiAp(wifiMulti);

  // WiFi.begin(ssid, password);
  Serial.print("Connecting");

  while (wifiMulti.run(connectTimeoutMs) != WL_CONNECTED)
  {
    // delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  current_ip = WiFi.localIP();
  Serial.println(current_ip);
}

// connect to the wifi with the strongest signal and print ip address if changed
inline void maintainWiFi() {
  // if not connected
  if (wifiMulti.run(connectTimeoutMs) != WL_CONNECTED) {
    return;
  }

  // if ip address is changed e.g. when connected to another wifi
  if (current_ip != WiFi.localIP()) {
    current_ip = WiFi.localIP();
    Serial.print("IP address changed to: ");
    Serial.println(current_ip);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  connect2WiFi();

  // start listening to incoming packets
  udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n",
                WiFi.localIP().toString().c_str(), localUdpPort);

  // // try ekf
  // Eigen::Vector3f z_1(-0.3131, -0.483, 2.1809633);
  // x_c_1 = ekf.estimate(z_1);

  // Serial.printf("z_1 = (%f, %f, %f)", z_1(0), z_1(1), z_1(2));
  // Serial.println();
  // Serial.printf("x_c_1 = (%f, %f, %f)", x_c_1(0), x_c_1(1), x_c_1(2));
}

// send sensor data via UART to esp8266 esp does the ekf calculation and sends
// back the result to PC via UDP
void loop() {
  // Serial.read();
  maintainWiFi();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize,
                  udp.remoteIP().toString().c_str(), udp.remotePort());
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      // set the char next to the last char to 0 (NULL in ascii)
      incomingPacket[len] = 0;
    }
    Serial.printf("UDP packet contents: %s\n", incomingPacket);

    // parse the incoming data into a Eigen vector
    float z_k_0, z_k_1, z_k_2;
    char* strtok_indx;
    char temp_chars[255];
    strcpy(temp_chars, incomingPacket);
    
    strtok_indx = strtok(temp_chars, ",");
    z_k_0 = atof(strtok_indx);
    strtok_indx = strtok(NULL, ",");
    z_k_1 = atof(strtok_indx);
    strtok_indx = strtok(NULL, ",");
    z_k_2 = atof(strtok_indx);

    Eigen::Vector3f z_k(z_k_0, z_k_1, z_k_2);
    Eigen::Vector3f x_c = ekf.estimate(z_k);

    // send back a reply to the IP address and port we got the packet from
    // udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.beginPacket(udp.remoteIP(), 54321);
    char buffer[100];
    
    // sprintf(buffer, " x_c_1 = (%f, %f, %f)", x_c_1(0), x_c_1(1), x_c_1(2));
    // sprintf(buffer, "%.10f,%.10f,%.10f", z_k(0), z_k(1), z_k(2));
    // udp.write(buffer);

    sprintf(buffer, "%f,%f,%f", x_c(0), x_c(1), x_c(2));
    udp.write(buffer);

    udp.endPacket();
  }
}