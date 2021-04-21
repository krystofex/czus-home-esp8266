#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <config.h>
#define ONBOARD_LED 2

#define WIFIMODE

#ifdef WIFIMODE
#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include <html.h>
AsyncWebServer server(80);
#endif

int controlThrottle = 1000;
int controlYaw, controlRoll, controlPitch = 1500;

double errorSum;

class IMU
{
public:
  int address;
  double x, y;
  int zTmp, z;
  double xCal, yCal, zCal = 0;
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  int lastLoop;

  void setup()
  {
    Wire.beginTransmission(address);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(address); //Start communicating with the MPU-6050
    Wire.write(0x1C);                //Send the requested starting register
    Wire.write(0x10);                //Set the requested starting register
    Wire.endTransmission();          //End the transmission
    //Configure the gyro (500dps full scale)
    Wire.beginTransmission(address); //Start communicating with the MPU-6050
    Wire.write(0x1B);                //Send the requested starting register
    Wire.write(0x08);                //Set the requested starting register
    Wire.endTransmission();
  }

  void calibrate()
  {
    for (int i = 0; i < 5000; i++)
    {
      Wire.beginTransmission(address);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(address, 14);

      AcX = Wire.read() << 8 | Wire.read();
      AcY = Wire.read() << 8 | Wire.read();
      AcZ = Wire.read() << 8 | Wire.read();
      Tmp = Wire.read() << 8 | Wire.read();
      GyX = Wire.read() << 8 | Wire.read();
      GyY = Wire.read() << 8 | Wire.read();
      GyZ = Wire.read() << 8 | Wire.read();
      xCal += GyX;
      yCal += GyY;
      zCal += GyZ;
      delay(1);
    }
    xCal /= 5000;
    yCal /= 5000;
    zCal /= 5000;
  }

  void measure()
  {
    Wire.beginTransmission(address);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(address, 14);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = ((Wire.read() << 8 | Wire.read()) - xCal);
    GyY = ((Wire.read() << 8 | Wire.read()) - yCal);
    GyZ = ((Wire.read() << 8 | Wire.read()) - zCal);

    int xAng = map(AcX, 265, 402, -90, 90);
    int yAng = map(AcY, 265, 402, -90, 90);
    int zAng = map(AcZ, 265, 402, -90, 90);

    AcX = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
    AcY = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);

    AcX = ((AcX >= 0 && AcX <= 180) ? AcX : AcX - 360) + 5;
    AcX = constrain(AcX, -90, 90);

    AcY = ((AcY >= 0 && AcY <= 180) ? AcY : AcY - 360) * -1;
    AcY = constrain(AcY, -90, 90);

    x += (GyX / 65.5) * (millis() - lastLoop);
    x = 0.99 * x + 10 * AcX;

    y += (GyY / 65.5) * (millis() - lastLoop);
    y = 0.99 * y + 10 * AcY;

    zTmp += (GyZ / 65.5) * (millis() - lastLoop);
    z = (((zTmp / 1000) % 360) + 360) % 360;

    //Serial.println((String)(millis() - lastLoop) + " ms");
    //if (millis() - lastLoop > 0)
    //Serial.println("loop time error: " + (String)(millis() - lastLoop) + " ms");

    lastLoop = millis();
  }

  void print()
  {
    Serial.print(x / 1000);
    Serial.print(" | ");
    Serial.print(y / 1000);
    Serial.print(" | ");
    Serial.println(z);
  }
};
class VoltageSensor
{
public:
  int pin;
  double voltage;
  void setup()
  {
    pinMode(pin, INPUT);
  }
  void measure()
  {
    voltage = (0.95 * voltage) + (0.05 * ((analogRead(pin) * 5) / 1024));
  }
};
class Barometer
{
public:
  const double seaLevelPressure = 1013.25;
  Adafruit_BME280 sensor;
  int address;

  double altitude;
  double groundAltitude;
  int lastLoop = 0;

  void setup()
  {
    while (!sensor.begin(address))
      Serial.println("BME280 error");
    for (int x = 0; x < 500; x++)
    {
      groundAltitude += sensor.readAltitude(seaLevelPressure) / 500;
      delay(1);
    }
  }
  void measure()
  {
    altitude = (0.6 * altitude) + (0.4 * sensor.readAltitude(seaLevelPressure));
    lastLoop = millis();
  }
};
class ESCController
{
public:
  Servo esc[4];

  void setup()
  {
    for (int i = 0; i < 4; i++)
      esc[i].attach(escPins[i]);
    for (int i = 0; i < 4; i++)
      esc[i].writeMicroseconds(0);
    delay(5000);
    for (int i = 0; i < 4; i++)
      esc[i].writeMicroseconds(2000);
    delay(5000);
    for (int i = 0; i < 4; i++)
      esc[i].writeMicroseconds(0);
    delay(5000);
  }
  void control(int params[4])
  {
    for (int i = 0; i < 4; i++)
      esc[i]
          .writeMicroseconds(params[i]);
  }
};
class Controller
{
public:
  void setup()
  {
#ifdef WIFIMODE
    WiFi.begin(SSID, PASSWORD); //setup wifi credentials

    while (WiFi.status() != WL_CONNECTED) //loops until wifi is connected
    {
      ;
    }
    Serial.println(WiFi.localIP()); //uncomment if you don't know ESP IP address

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { //web controller html
      request->send_P(200, "text/html", index_html);
    });

    server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
      //get, convert and save values from web controller
      controlThrottle = atof(request->getParam(2)->value().c_str());
      controlYaw = atof(request->getParam(3)->value().c_str());
      controlPitch = atof(request->getParam(1)->value().c_str());
      controlRoll = atof(request->getParam(0)->value().c_str());
      request->send(200, "text/plain", "OK"); //send 200-OK to client
    });

    server.begin();
#endif
  }
};
class PID
{
};

IMU mpu1;
ESCController ESCs;
Barometer barometer;
VoltageSensor voltageSensor;
Controller controller;
PID pid;

void MPUSetup(int address)
{
  mpu1.address = address;
  mpu1.setup();
  mpu1.calibrate();
  mpu1.lastLoop = millis();
}
void Compute();

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);

  MPUSetup(0x68);

  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, HIGH);

  barometer.address = 0x76;
  barometer.setup();
  ESCs.setup();
  controller.setup();
  // voltageSensor.pin = 15;
  // voltageSensor.setup();
}

void loop()
{
  mpu1.measure(); //if true loop time error output
  //mpu1.print();
  if (millis() - barometer.lastLoop > 100)
  {
    // voltageSensor.measure();
    barometer.measure();
    Serial.println(mpu1.x / 1000);
  }
  Compute();
}

void Compute()
{
  double maxAngle = 15;

  if (controlThrottle > 1010)
  {

    int pitchError = -1 * ((mpu1.y / 1000) - map(controlPitch, 1000, 2000, -maxAngle, maxAngle)); //count angle difference
    pitchError *= 3;

    int rollError = -1 * ((mpu1.x / 1000) - map(controlRoll, 1000, 2000, -maxAngle, maxAngle));
    errorSum += rollError;
    rollError *= 2;

    int yawAdjust = map(controlYaw, 1000, 2000, -100, 100);

    int params[4] = {
        controlThrottle - pitchError + rollError + yawAdjust,
        controlThrottle - pitchError - rollError - yawAdjust,
        controlThrottle + pitchError - rollError + yawAdjust,
        controlThrottle + pitchError + rollError - yawAdjust,
    };
    ESCs.control(params);
  }
}