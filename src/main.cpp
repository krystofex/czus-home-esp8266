#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include "ESPAsyncWebServer.h"

#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>

#include "Configuration.h"

StaticJsonDocument<200> doc;
AsyncWebServer server(80);
String serial;
int lastTime = millis();

class WebServer
{
public:
  void setup()
  {
    server.on("/api", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send_P(200, "application/json", serial.c_str());
    });
    server.begin();
  }
};
class WIFI
{
public:
  void setup()
  {
    IPAddress ip(192, 168, 1, 185);
    IPAddress dns(192, 168, 1, 1);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(ip, dns, gateway, subnet);
    WiFi.begin(SSID, PASSWORD);
    Serial.println("Connecting to: " + (String)SSID);
    Serial.println("connected");
    Serial.println(WiFi.localIP());
  }
  void checkConnection()
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      delay(50);
      digitalWrite(2, LOW);
      delay(50);
      digitalWrite(2, HIGH);
    }
  }
};
class BME280
{
public:
  Adafruit_BME280 sensor;
  float temperature;
  float pressure;
  float humidity;

  void setup()
  {
    if (sensor.begin(0x76))
      Serial.println("Sensor not fond");
  }
  void measure()
  {
    temperature = (0.9 * temperature) + (0.1 * sensor.readTemperature());
    pressure = (0.9 * pressure) + (0.1 * sensor.readPressure() / 100.0f);
    humidity = (0.9 * humidity) + (0.1 * sensor.readHumidity());

    Serial.println(sensor.readTemperature());

    doc["temperature"] = temperature;
    doc["pressure"] = pressure;
    doc["humidity"] = humidity;
  }
};
class BMP280
{
public:
  Adafruit_BMP280 sensor;
  float temperature;
  float pressure;

  void setup()
  {
    Serial.println("Sensor not fond");
  }
  void measure()
  {
    temperature = (0.9 * temperature) + (0.1 * sensor.readTemperature());
    pressure = (0.9 * pressure) + (0.1 * sensor.readPressure() / 100.0f);

    doc["temperature"] = temperature;
    doc["pressure"] = pressure;
  }
};

WIFI wifi;
WebServer webServer;

BME280 sensor; // change sensor here

void setup()
{
  Serial.begin(9600);
  wifi.setup();
  webServer.setup();
  sensor.setup();
  doc["macAdress"] = WiFi.macAddress();
}

void loop()
{
  wifi.checkConnection();

  serial = "";
  serializeJson(doc, serial);

  if (millis() - lastTime > 2)
  {
    sensor.measure();
    lastTime = millis();
  }
}
