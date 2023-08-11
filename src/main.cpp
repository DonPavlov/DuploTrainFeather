#include "main.hpp"
#include <cstring>
#include <string>

#include "train_control.hpp"
#include "buttons.hpp"
#include "Lpf2Hub.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_NeoPixel.h"
#include <Adafruit_GFX.h>

#define ARCADE_N (18)
#define ARCADE_S (16)
#define ARCADE_W (17)
#define ARCADE_E (15)

#define NEOPIXEL_RING (6)
#define N_LEDS (16)

Lpf2Hub myHub;
byte    motorPort = (byte)DuploTrainHubPort::MOTOR;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS,
                                            NEOPIXEL_RING,
                                            NEO_GRB + NEO_KHZ800);
Adafruit_7segment matrix = Adafruit_7segment();

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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage
                                   // level)

  sleep(3);
  Serial.begin(115200);
  matrix.begin(0x70);     // Init I2C Display
  strip.begin();          // Init LED Strip
  strip.setBrightness(4); // lower brightness for toddlers
  uint32_t magenta = strip.Color(255, 0, 255);

  strip.fill(strip.Color(255, 0, 255), 0, N_LEDS);
  strip.show(); // Initialize all pixels to 'off'

  myHub.init();
  TrainControl zug(myHub);
  digitalWrite(LED_BUILTIN, LOW); // turn the LED on (HIGH is the voltage level)
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

  static uint8_t number = 0;
  matrix.print(number, DEC);
  matrix.writeDisplay();
  number++;
}
