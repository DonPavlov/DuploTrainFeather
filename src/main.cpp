#include "main.hpp"
#include <cstring>
#include <string>

#include "train_control.hpp"
#include "buttons.hpp"
#include "Lpf2Hub.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_NeoPixel.h"
#include <Adafruit_GFX.h>
#include "light_strip.hpp"

#include <Adafruit_MCP23X17.h>

// UART1 GPIO 18 RX GPIO17 TX
#define ARCADE_N (16)
#define ARCADE_S (15)
#define ARCADE_W (14)
#define ARCADE_E (8)

unsigned long startMillis;         // some global variables available anywhere in
                                   // the program
unsigned long currentMillis;
const unsigned long period = 2000; // the value is a number of milliseconds

Lpf2Hub myHub;
byte    motorPort = (byte)DuploTrainHubPort::MOTOR;
LightStrip myStrip; // Create an instance of the LightStrip class

Adafruit_7segment matrix = Adafruit_7segment();
Adafruit_MCP23X17 mcp;

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
  pinMode(LED_BUILTIN, OUTPUT);

  // Set ESP32 pins 5 and 9 as INPUT_PULLUP
  pinMode(5,           INPUT_PULLUP);
  pinMode(9,           INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage
                                   // level)
  matrix.begin(0x70);              // Init I2C Display

  myHub.init();
  TrainControl   zug(myHub);
  static uint8_t number = 0;
  matrix.print("1234");
  matrix.writeDisplay();
  myStrip.initialize();

  // Never delete this delay, this ensures the device can be reflashed without
  // issues. Else hardreset with esptool and lucky timing while sending factory
  // reset command and reseting the device
  sleep(2);
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN, LOW); // turn the LED off
  startMillis = millis();         // initial start time

  // configure buttons
  if (!mcp.begin_I2C())
  {
    // if (!mcp.begin_SPI(CS_PIN)) {
    Serial.println("Error.");

    while (1)
      ;
  }

  // Configure Port A pins 0 to 5 as INPUT and enable pull-up resistors
  for (int i = 0; i < 6; i++)
  {
    mcp.pinMode(i, INPUT_PULLUP);
  }

  // Configure Port B pins B5 to B0 as OUTPUT
  for (int i = 13; i >= 8; i--)
  {
    mcp.pinMode(i, OUTPUT);
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
  // connect flow

  currentMillis = millis();

  if (currentMillis - startMillis >= period) // test whether the period has
                                             // elapsed
  {
    myStrip.rainbow(true);
    startMillis = currentMillis;             // IMPORTANT to save the start
                                             // time of the current LED
                                             // state.
  }

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
  test_inputs();
}

void test_inputs()
{
  for (int i = 0; i < 6; i++)
  {
    int inputValue = mcp.digitalRead(i);
    Serial.print("Input ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(inputValue);
  }

  // Read ESP32 pins 5 and 9 and print their values
  int esp32Input5 = digitalRead(5);
  int esp32Input9 = digitalRead(9);

  Serial.print("ESP32 Pin 5: ");
  Serial.println(esp32Input5);

  Serial.print("ESP32 Pin 9: ");
  Serial.println(esp32Input9);

  // Enable and disable LEDs sequentially with a half-second delay
  for (int i = 13; i >= 8; i--)
  {
    mcp.digitalWrite(i, HIGH); // Turn on LED
    delay(500);                // Wait for 500 milliseconds
    mcp.digitalWrite(i, LOW);  // Turn off LED
  }
}
