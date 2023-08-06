#include "main.hpp"
#include <cstring>
#include <string>

#include "train_control.hpp"
#include "buttons.hpp"
#include "Lpf2Hub.h"

#ifdef DEBUGWIFI
#include "RemoteDebug.h"
extern const char *secret_ssid;
extern const char *secret_pass;
RemoteDebug Debug;

#endif // ifdef DEBUGWIFI


Lpf2Hub myHub;
byte    motorPort = (byte)DuploTrainHubPort::MOTOR;

void colorSensorCallback(void      *hub,
                         byte       portNumber,
                         DeviceType deviceType,
                         uint8_t   *pData)
{
  Lpf2Hub *myHub = (Lpf2Hub *)hub;

  if (deviceType == DeviceType::DUPLO_TRAIN_BASE_COLOR_SENSOR)
  {
    int color = myHub->parseColor(pData);
    Serial.print("Color: ");
    Serial.println(COLOR_STRING[color]);
    myHub->setLedColor((Color)color);

    if (color == (byte)RED)
    {
      myHub->playSound((byte)DuploTrainBaseSound::BRAKE);
    }
    else if (color == (byte)BLUE)
    {
      myHub->playSound((byte)DuploTrainBaseSound::WATER_REFILL);
    }
    else if (color == (byte)YELLOW)
    {
      myHub->playSound((byte)DuploTrainBaseSound::HORN);
    }
  }
}

void speedometerSensorCallback(void      *hub,
                               byte       portNumber,
                               DeviceType deviceType,
                               uint8_t   *pData)
{
  Lpf2Hub *myHub = (Lpf2Hub *)hub;

  if (deviceType == DeviceType::DUPLO_TRAIN_BASE_SPEEDOMETER)
  {
    int speed = myHub->parseSpeedometer(pData);
    Serial.print("Speed: ");
    Serial.println(speed);

    if (speed > 10)
    {
      Serial.println("Forward");
      myHub->setBasicMotorSpeed(motorPort, 50);
    }
    else if (speed < -10)
    {
      Serial.println("Back");
      myHub->setBasicMotorSpeed(motorPort, -50);
    }
    else
    {
      Serial.println("Stop");
      myHub->stopBasicMotor(motorPort);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  #ifdef DEBUGWIFI

  WiFi.begin(secret_ssid, secret_pass);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);

    Serial.println("Connecting to WiFi...");
  }

  // Set static IP, gateway, subnet, and DNS
  WiFi.config(staticIP, gateway, subnet, dns);

  Serial.println("WiFi connected with static IP: " +
                 WiFi.localIP().toString());

  MDNS.begin(HOST_NAME);

  Debug.begin(HOST_NAME);
  Debug.setResetCmdEnabled(true);

  #endif // ifdef DEBUGWIFI


  myHub.init();
  TrainControl zug(myHub);
}

void loop()
{
  #ifdef DEBUGWIFI

  Debug.handle();
  #endif // ifdef DEBUGWIFI

  // put your main code here, to run repeatedly:
  // connect flow
  if (myHub.isConnecting())
  {
    myHub.connectHub();

    if (myHub.isConnected())
    {
      Serial.println("Connected to Duplo Hub");

      delay(200);

      delay(200);

      // connect color sensor and activate it for updates
      myHub.activatePortDevice((byte)DuploTrainHubPort::SPEEDOMETER,
                               speedometerSensorCallback);
      delay(200);

      // connect speed sensor and activate it for updates
      myHub.activatePortDevice((byte)DuploTrainHubPort::COLOR,
                               colorSensorCallback);
      delay(200);
      myHub.setLedColor(GREEN);
    }
    else
    {
      Serial.println("Failed to connect to Duplo Hub");
    }
  }
  sleep(1);
}
