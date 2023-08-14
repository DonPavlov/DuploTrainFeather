#include "train_control.hpp"
#include <Arduino.h>
#include <cstdint>

Lpf2Hub mHub;

int8_t g_speed = 0;

TrainControl::TrainControl()
{}


void TrainControl::init()
{
  Serial1.println("Setup Train Control");
  mHub.init();
}

bool TrainControl::SendCommand(Commands::Commands cmd)
{
  // TODO send something with the bluetooth train interface
  bool result = false;

  return result;
}

bool TrainControl::SendSpeed(Commands::Commands cmd,
                             int8_t             speed)
{
  bool result = false;

  return result;
}

void colorSensorCb(void      *hub,
                   byte       portNumber,
                   DeviceType deviceType,
                   uint8_t   *pData)
{
  Lpf2Hub *mHub = (Lpf2Hub *)hub;

  if (deviceType == DeviceType::DUPLO_TRAIN_BASE_COLOR_SENSOR)
  {
    int color = mHub->parseColor(pData);
    Serial1.print("Color: ");
    Serial1.println(COLOR_STRING[color]);
    mHub->setLedColor((Color)color);

    if (color == (byte)RED)
    {
      mHub->playSound((byte)DuploTrainBaseSound::BRAKE);
    }
    else if (color == (byte)BLUE)
    {
      mHub->playSound((byte)DuploTrainBaseSound::WATER_REFILL);
    }
    else if (color == (byte)YELLOW)
    {
      mHub->playSound((byte)DuploTrainBaseSound::HORN);
    }
  }
}

void speedometerSensorCb(void      *hub,
                         byte       portNumber,
                         DeviceType deviceType,
                         uint8_t   *pData)
{
  Lpf2Hub *mHub  = (Lpf2Hub *)hub;
  byte     mPort = (byte)DuploTrainHubPort::MOTOR;

  if (deviceType == DeviceType::DUPLO_TRAIN_BASE_SPEEDOMETER)
  {
    int speed = mHub->parseSpeedometer(pData);
    Serial1.print("Speed: ");
    Serial1.println(speed);

    if (speed > 10)
    {
      Serial1.println("Forward");
      g_speed = 100;
      mHub->setBasicMotorSpeed(mPort, g_speed);
    }
    else if (speed < -10)
    {
      Serial1.println("Back");
      g_speed = -100;
      mHub->setBasicMotorSpeed(mPort, g_speed);
    }
    else
    {
      Serial1.println("Stop");
      g_speed = 0;
      mHub->stopBasicMotor(mPort);
    }
  }
}

void TrainControl::stateMachine()
{
  if (mHub.isConnecting())
  {
    mHub.connectHub();
    Serial1.println("Connecting...");

    if (mHub.isConnected())
    {
      Serial1.println("Connected to Duplo Hub");

      delay(200);
      delay(200);

      // connect speed sensor and activate it for updates
      mHub.activatePortDevice((byte)DuploTrainHubPort::SPEEDOMETER,
                              speedometerSensorCb);
      delay(200);

      // connect color sensor and activate it for updates
      // // connect speed sensor and activate it for updates
      mHub.activatePortDevice((byte)DuploTrainHubPort::COLOR,
                              colorSensorCb);
      delay(200);
      mHub.setLedColor(GREEN);
    }
    else
    {
      Serial1.println("Failed to connect to Duplo Hub");
    }
  }
}

void TrainControl::increase_speed()
{
  g_speed += 10;

  if (g_speed > 100)
  {
    g_speed = 100;
  }
  else if (g_speed < -100)
  {
    g_speed = -100;
  }
}

void TrainControl::decrease_speed()
{
  g_speed -= 10;

  if ((g_speed < 10) && (g_speed > -10))
  {
    g_speed = 0;
  }
}

int8_t TrainControl::get_speed()
{
  return g_speed;
}
