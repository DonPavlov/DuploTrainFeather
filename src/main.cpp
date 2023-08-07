#include "main.hpp"
#include <cstring>
#include <string>

#include "train_control.hpp"
#include "buttons.hpp"
#include "Lpf2Hub.h"


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

// void speedometerSensorCallback(void      *hub,
//                                byte       portNumber,
//                                DeviceType deviceType,
//                                uint8_t   *pData);

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
  // Never delete this delay, this ensures the device can be reflashed without
  // issues. Else hardreset with esptool and lucky timing while sending factory
  // reset command and reseting the device
  delay(2500);
  Serial.begin(9600);
  myHub.init();
  TrainControl zug(myHub);
}

void loop()
{
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
