#include "train_control.hpp"
#include <Arduino.h>
#include <cstdint>


std::map<Commands::Commands, std::string> commandNames = {
  { Commands::Commands::Forward,
    "Forward" },
  { Commands::Commands::Backward,
    "Backward" },
  { Commands::Commands::Stop,
    "Stop" },
  { Commands::Commands::Light,
    "Light"   },
  { Commands::Commands::Refill,
    "Refill"    },
  { Commands::Commands::Horn,
    "Horn"  },
  { Commands::Commands::Steam,
    "Steam" },
  { Commands::Commands::Departure,
    "Departure" },
  { Commands::Commands::Faster,
    "Faster" },
  { Commands::Commands::Slower,
    "Slower" },
  { Commands::Commands::None,
    "None"  }
};


TrainControl::TrainControl()
{}


void TrainControl::init()
{
  Serial1.println("Setup Train Control");
  m_Hub.init();
  checkConnectionToTrain();
}

bool TrainControl::SendCommand(Commands::Commands cmd)
{
  bool result = false;

  if (checkConnectionToTrain())
  {
    result = true;
    char cstr[16] = { 0 };
    byte mPort    = (byte)DuploTrainHubPort::MOTOR;

    // TODO Include magic enum and convert enum name of cmd to String
    sprintf(cstr, "Command %d: %s", static_cast<int>(cmd), commandNames[cmd].c_str());
    Serial1.println(cstr);

    unsigned long curMil                     = millis();
    static unsigned long prevMil             = 0;
    static unsigned long executionTimeMillis = 0;

    bool execute = curMil > (prevMil + executionTimeMillis);

    // TODO test all buttons and assign apropriate values so all do something
    switch (cmd)
    {
    case Commands::Commands::Forward:

      if (0 == g_speed)
      {
        g_speed = 50;
      }

      m_Hub.setBasicMotorSpeed(mPort, g_speed);
      break;

    case Commands::Commands::Backward:

      if (0 == g_speed)
      {
        g_speed = -50;
      }
      m_Hub.setBasicMotorSpeed(mPort, g_speed);
      break;

    case Commands::Commands::Faster:
      increase_speed();
      m_Hub.setBasicMotorSpeed(mPort, g_speed);
      break;

    case Commands::Commands::Slower:
      decrease_speed();
      m_Hub.setBasicMotorSpeed(mPort, g_speed);
      break;

    case Commands::Commands::Stop:

      if (execute)
      {
        m_Hub.playSound((byte)DuploTrainBaseSound::BRAKE);
        prevMil             = curMil;
        executionTimeMillis = 500;
      }
      else
      {
        result = false;
      }
      break;

    case Commands::Commands::Horn:

      if (execute)
      {
        m_Hub.playSound((byte)DuploTrainBaseSound::HORN);
        prevMil             = curMil;
        executionTimeMillis = 500;
      }
      else
      {
        result = false;
      }
      break;

    case Commands::Commands::Light:
      static Color test = Color::PINK;
      m_Hub.setLedColor(test);
      break;

    case Commands::Commands::Refill:

      if (execute)
      {
        m_Hub.playSound((byte)DuploTrainBaseSound::WATER_REFILL);
        prevMil             = curMil;
        executionTimeMillis = 500;
      }
      else
      {
        result = false;
      }
      break;

    case Commands::Commands::Steam:

      if (execute)
      {
        m_Hub.playSound((byte)DuploTrainBaseSound::STEAM);
        prevMil             = curMil;
        executionTimeMillis = 500;
      }
      else
      {
        result = false;
      }
      break;

    default:
      executionTimeMillis = 0;
      break;
    }
  }
  return result;
}

bool TrainControl::SendSpeed(Commands::Commands cmd,
                             int8_t             speed)
{
  bool result = false;

  return result;
}

// TODO add a timeout to the traincommands after having received a successfull command
void TrainControl::colorSensorCb(void      *hub,
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

// TODO add a timeout to the traincommands after having received a successfull command
void TrainControl::speedometerSensorCb(void      *hub,
                                       byte       portNumber,
                                       DeviceType deviceType,
                                       uint8_t   *pData)
{
  Lpf2Hub *mHub  = (Lpf2Hub *)hub;
  byte     mPort = (byte)DuploTrainHubPort::MOTOR;

  if (deviceType == DeviceType::DUPLO_TRAIN_BASE_SPEEDOMETER)
  {
    int speed = mHub->parseSpeedometer(pData);

    if (speed > 10)
    {
      if (g_speed <= 0)
      {
        g_speed = 50;
      }

      if (g_lastSpeed <= 0)
      {
        Serial1.println("Forward");
        Serial1.print("Speed: ");
        Serial1.println(speed);
        mHub->setBasicMotorSpeed(mPort, g_speed);
        g_lastSpeed = g_speed;
      }
    }
    else if (speed < -10)
    {
      if (g_speed >= 0)
      {
        g_speed = -50;
      }

      if (g_lastSpeed >= 0)
      {
        Serial1.println("Back");
        Serial1.print("Speed: ");
        Serial1.println(speed);
        mHub->setBasicMotorSpeed(mPort, g_speed);
        g_lastSpeed = g_speed;
      }
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
  if (!checkConnectionToTrain())
  {
    if (m_Hub.isConnecting())
    {
      m_Hub.connectHub();
      Serial1.println("Connecting...");
      delay(100);

      if (m_Hub.isConnected())
      {
        Serial1.println("Connected to Duplo Hub");

        delay(200);
        delay(200);

        // connect speed sensor and activate it for updates
        m_Hub.activatePortDevice((byte)DuploTrainHubPort::SPEEDOMETER,
                                 speedometerSensorCb);
        delay(200);

        // connect color sensor and activate it for updates
        m_Hub.activatePortDevice((byte)DuploTrainHubPort::COLOR,
                                 colorSensorCb);
        delay(200);
        m_Hub.setLedColor(GREEN);
      }
      else
      {
        Serial1.println("Failed to connect to Duplo Hub");
      }
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

bool TrainControl::checkConnectionToTrain()
{
  if (m_Hub.isConnected())
  {
    m_connected = true;
  }
  else
  {
    m_connected = false;
  }
  return m_connected;
}
