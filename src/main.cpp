#include "main.hpp"
#include <cstring>
#include <string>

#include "io.hpp"
#include "Lpf2Hub.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_NeoPixel.h"
#include <Adafruit_GFX.h>
#include "light_strip.hpp"

#include <Adafruit_MCP23X17.h>
#include "secrets.hpp"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// TODO include #include "Adafruit_MAX1704X.h" // for battery voltage monitoring


// UART1 GPIO 18 RX GPIO17 TX
#define RXD1 (18)
#define TXD1 (17)

Lpf2Hub m_Hub;
volatile int8_t g_speed { 0 };
volatile int8_t g_lastSpeed{ 0 };
bool m_connected{  false };


#if (WIFI_MODE == 1)

// Set your Static IP address
IPAddress local_IP(192, 168, 178, 123);

// Set your Gateway IP address
IPAddress gateway(192, 168, 178, 1);

IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);        // optional
IPAddress secondaryDNS(8, 8, 4, 4);      // optional
#endif // if (WIFI_MODE == 1)
unsigned long startMillis;               // some global variables available anywhere in
                                         // the program
unsigned long currentMillis;
unsigned long wifiMillis;
const unsigned long period       { 50 }; // the value is a number of milliseconds
const unsigned long wifi_timeout { 10000 };
bool wifiConSkipped              { false };
bool wifiSetupfinished           { false };

bool powerSaveMode { false };
const unsigned long powerSaveThreshold { 600000 }; // 1 minutes in milliseconds
unsigned long lastActivityTime         { 0 };
unsigned long executionTimeMillis { 0 };

LightStrip myStrip; // Create an instance of the LightStrip class

// Adafruit_7segment matrix = Adafruit_7segment();
IO io_ctrl;

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

static void colorSensorCb(void      *hub,
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
static void speedometerSensorCb(void      *hub,
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

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);

  io_ctrl.init_buttons();

  // Set ESP32 pins 5 and 9 as INPUT_PULLUP
  pinMode(5, INPUT_PULLUP);        // make sure pins don't die, they are connected to the lvl shifter
  pinMode(9, INPUT_PULLUP);        // make sure pins don't die, they are connected to the lvl shifter


  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage
                                   // level)
  // matrix.begin(0x70);              // Init I2C Display


  static uint8_t number = 0;

  // matrix.print("1234");
  // matrix.writeDisplay();

  myStrip.initialize();

  digitalWrite(LED_BUILTIN, LOW);      // turn the LED off
  startMillis = wifiMillis = millis(); // initial start time

  Serial1.println("Setup Train Control");

#if (WIFI_MODE == 1)

  // Init wifi for ota updates
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
  {
    Serial1.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  #endif // if (WIFI_MODE == 1)

  m_Hub.init();
  checkConnectionToTrain();

  // myStrip.rainbow(true);
}

void loop()
{
  currentMillis = millis();
  #if (WIFI_MODE == 1)

  // Skip wifi part if not connected
  if (!wifiConSkipped)
  {
    // Connect to wifi If not already connected
    uint8_t status = WiFi.waitForConnectResult();

    if (status != WL_CONNECTED)
    {
      if (currentMillis - wifiMillis >= wifi_timeout)
      {
        wifiConSkipped = true;
        wifiMillis     = currentMillis;
      }
    }

    // If connected to Wifi and wifi setup not yet finished Setup arduino ota
    else if ((status == WL_CONNECTED)
             && (!wifiSetupfinished))
    {
      ArduinoOTA
      .onStart([]() {
        String type;

        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
        lastActivityTime = millis();

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial1.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial1.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial1.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial1.printf("Error[%u]: ", error);

        if (error == OTA_AUTH_ERROR)
          Serial1.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial1.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial1.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial1.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial1.println("End Failed");
      });

      ArduinoOTA.begin();

      Serial1.println("Ready");
      Serial1.print("IP address: ");
      Serial1.println(WiFi.localIP());
      wifiSetupfinished = true;
    }

    if ((wifiSetupfinished) && (!powerSaveMode))
    {
      ArduinoOTA.handle();
    }
  }
  #endif // if (WIFI_MODE == 1)

  if (currentMillis - startMillis >= period) // test whether the period has elapsed
  {
    // myStrip.rainbow(true );
    // TODO fix rainbow effect, make it only continue if function is reentered and continue were it was before.
    startMillis = currentMillis; // IMPORTANT to save the start
                                 // time of the current LED
                                 // state.
    io_ctrl.read_buttons();
  }

  // TODO only execute if necessary, make sure train can reconnect if connection is lost.
  stateMachine();

  // TODO add timeout if no button press occured for 10 minutes and shutdown most of the functionality, until a button
  // is pressed again or reboot. Or just disable wifi after 5 min to save power but might also be necessary for arduino
  // OTA to have changes....
  // addfunction to stop battery can charge

  if (checkPowerSaveNeeded())
  {
    powerSaving();
  }
}

void powerSaving()
{
  powerSaveMode = true;
#if (WIFI_MODE == 1)
  ArduinoOTA.end(); // Stop OTA server
  Serial1.println("Disable OTA functionality.");
  wifiConSkipped = true;
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  Serial1.println("Disable WiFi");
#endif // if (WIFI_MODE == 1)
  myStrip.rainbow(false);

  // matrix.print("");
  // matrix.writeDisplay();
  Serial1.println("Stop Led Ring and MatrixLeds.");
}

bool checkPowerSaveNeeded()
{
  bool result { false };

  if (!powerSaveMode && (millis() - lastActivityTime >= powerSaveThreshold))
  {
    result = true;
  }
  return result;
}

/*
 #include "Adafruit_MAX1704X.h"

   Adafruit_MAX17048 maxlipo;

   void setup() {
   Serial.begin(115200);
   while (!Serial) delay(10);    // wait until serial monitor opens

   Serial.println(F("\nAdafruit MAX17048 simple demo"));

   if (!maxlipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    while (1) delay(10);
   }
   Serial.print(F("Found MAX17048"));
   Serial.print(F(" with Chip ID: 0x"));
   Serial.println(maxlipo.getChipID(), HEX);
   }

   void loop() {
   Serial.print(F("Batt Voltage: ")); Serial.print(maxlipo.cellVoltage(), 3); Serial.println(" V");
   Serial.print(F("Batt Percent: ")); Serial.print(maxlipo.cellPercent(), 1); Serial.println(" %");
   Serial.println();

   delay(2000);  // dont query too often!
   }
 */
bool SendCommand(Commands::Commands cmd)
{
  bool result = false;

  if (checkConnectionToTrain())
  {
    result = true;
    constexpr size_t buf_size { 12 };
    char cstr[buf_size] = { 0 };
    byte mPort          = (byte)DuploTrainHubPort::MOTOR;

    snprintf(cstr, buf_size, "Command %d: %s", static_cast<int>(cmd), commandNames[cmd].c_str());
    Serial1.println(cstr);

    unsigned long curMil         = millis();
    static unsigned long prevMil = millis();

    bool execute = curMil >= (prevMil + executionTimeMillis);

    // TODO test all buttons and assign apropriate values so all do something
    switch (cmd)
    {
    case Commands::Commands::Forward:
    {
      Serial1.println("Debugging 1");

      if (execute)
      {
        if (0 == g_speed)
        {
          g_speed = 50;
        }

        // m_Hub.setBasicMotorSpeed(mPort, g_speed);
        prevMil             = curMil;
        executionTimeMillis = 100;
      }
      break;
    }

    case Commands::Commands::Backward:
    {
      Serial1.println("Debugging 2");

      if (execute)
      {
        if (0 == g_speed)
        {
          g_speed = -50;
        }

        // m_Hub.setBasicMotorSpeed(mPort, g_speed);
        prevMil             = curMil;
        executionTimeMillis = 100;
      }

      break;
    }


    case Commands::Commands::Stop:
    {
      Serial1.println("Debugging 5");

      if (execute)
      {
        // m_Hub.playSound((byte)DuploTrainBaseSound::BRAKE);
        prevMil             = curMil;
        executionTimeMillis = 500;
      }

      break;
    }

    case Commands::Commands::Light:
    {
      Serial1.println("Debugging 7");

      if (execute)
      {
        Serial1.println("Debugging 71");
        static Color test = Color::PINK;

        // m_Hub.setLedColor(test);
        prevMil             = curMil;
        executionTimeMillis = 200;
      }

      break;
    }

    case Commands::Commands::Refill:
    {
      Serial1.println("Debugging 8");

      if (execute)
      {
        // m_Hub.playSound((byte)DuploTrainBaseSound::WATER_REFILL);
        prevMil             = curMil;
        executionTimeMillis = 500;
      }

      break;
    }

    case Commands::Commands::Horn:
    {
      Serial1.println("Debugging 6");

      if (execute)
      {
        // m_Hub.playSound((byte)DuploTrainBaseSound::HORN);
        prevMil             = curMil;
        executionTimeMillis = 500;
      }

      break;
    }


    case Commands::Commands::Steam:
    {
      Serial1.println("Debugging 9");

      if (execute)
      {
        // m_Hub.playSound((byte)DuploTrainBaseSound::STEAM);
        prevMil             = curMil;
        executionTimeMillis = 500;
      }

      break;
    }

    case Commands::Commands::Departure:
    {
      Serial1.println("Debugging x");

      if (execute)
      {
        // m_Hub.playSound((byte)DuploTrainBaseSound::STEAM);
        prevMil             = curMil;
        executionTimeMillis = 500;
      }

      break;
    }

    case Commands::Commands::Faster: {
      Serial1.println("Debugging 3");

      if (execute)
      {
        increase_speed();

        // m_Hub.setBasicMotorSpeed(mPort, g_speed);
        prevMil             = curMil;
        executionTimeMillis = 100;
      }

      break;
    }

    case Commands::Commands::Slower:
    {
      Serial1.println("Debugging 4");

      if (execute)
      {
        decrease_speed();

        // m_Hub.setBasicMotorSpeed(mPort, g_speed);
        prevMil             = curMil;
        executionTimeMillis = 100;
      }

      break;
    }

    default:
      executionTimeMillis = 0;
      break;
    }

    if (!execute)
      result = false;
  }


  return result;
}

void stateMachine()
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

void increase_speed()
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

void decrease_speed()
{
  g_speed -= 10;

  if ((g_speed < 10) && (g_speed > -10))
  {
    g_speed = 0;
  }
}

int8_t get_speed()
{
  return g_speed;
}

bool checkConnectionToTrain()
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
