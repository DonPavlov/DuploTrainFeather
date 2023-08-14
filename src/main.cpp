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
#include "secrets.hpp"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// UART1 GPIO 18 RX GPIO17 TX
#define ARCADE_N (16)
#define ARCADE_S (15)
#define ARCADE_W (14)
#define ARCADE_E (8)
#define RXD1 (18)
#define TXD1 (17)


// Set your Static IP address
IPAddress local_IP(192, 168, 178, 123);

// Set your Gateway IP address
IPAddress gateway(192, 168, 178, 1);

IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);        // optional
IPAddress secondaryDNS(8, 8, 4, 4);      // optional

unsigned long startMillis;               // some global variables available anywhere in
                                         // the program
unsigned long currentMillis;
unsigned long wifiMillis;
const unsigned long period       = 2000; // the value is a number of milliseconds
const unsigned long wifi_timeout = 10000;
bool wifiConSkipped              = false;
bool wifiSetupfinished           = false;


LightStrip myStrip; // Create an instance of the LightStrip class

Adafruit_7segment matrix = Adafruit_7segment();
Adafruit_MCP23X17 mcp;
TrainControl zug;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);

  // Set ESP32 pins 5 and 9 as INPUT_PULLUP
  pinMode(5,        INPUT_PULLUP); // make sure pins don't die, they are connected to the lvl shifter
  pinMode(9,        INPUT_PULLUP); // make sure pins don't die, they are connected to the lvl shifter

  // Set the inputs for
  pinMode(ARCADE_N, INPUT_PULLUP);
  pinMode(ARCADE_S, INPUT_PULLUP);
  pinMode(ARCADE_W, INPUT_PULLUP);
  pinMode(ARCADE_E, INPUT_PULLUP);

  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage
                                   // level)
  matrix.begin(0x70);              // Init I2C Display


  static uint8_t number = 0;
  matrix.print("1234");
  matrix.writeDisplay();
  myStrip.initialize();

  digitalWrite(LED_BUILTIN, LOW);      // turn the LED off
  startMillis = wifiMillis = millis(); // initial start time

  // configure buttons
  if (!mcp.begin_I2C())
  {
    // if (!mcp.begin_SPI(CS_PIN)) {
    Serial1.println("Error.");

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

  // Init wifi for ota updates
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
  {
    Serial1.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  zug.init();
}

void loop()
{
  currentMillis = millis();

  // Skip wifi part if not connected
  if (!wifiConSkipped)
  {
    // Connect to wifi If not already connected
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
      Serial1.print(".");

      if (currentMillis - wifiMillis >= wifi_timeout)
      {
        wifiConSkipped = true;
        wifiMillis     = currentMillis;
      }
    }

    // If connected to Wifi and wifi setup not yet finished Setup arduino ota
    else if ((WiFi.waitForConnectResult() == WL_CONNECTED)
             && (!wifiSetupfinished))
    {
      ArduinoOTA
      .onStart([]() {
        String type;

        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

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

    if (wifiSetupfinished)
    {
      ArduinoOTA.handle();
    }
  }


  if (currentMillis - startMillis >= period) // test whether the period has elapsed
  {
    myStrip.rainbow(true);
    startMillis = currentMillis;             // IMPORTANT to save the start
                                             // time of the current LED
                                             // state.
    // zug.stateMachine();
    Serial1.println(".");
  }
  zug.stateMachine();

  // test_inputs();
}

void test_inputs()
{
  for (int i = 0; i < 6; i++)
  {
    int inputValue = mcp.digitalRead(i);
    Serial1.print("Input ");
    Serial1.print(i);
    Serial1.print(": ");
    Serial1.println(inputValue);
  }

  // Read ESP32 pins 5 and 9 and print their values
  int arcade_n_input = digitalRead(ARCADE_N);
  int arcade_s_input = digitalRead(ARCADE_S);
  int arcade_w_input = digitalRead(ARCADE_W);
  int arcade_e_input = digitalRead(ARCADE_E);

  Serial1.print("ARCADE_N: ");
  Serial1.println(arcade_n_input);
  Serial1.print("ARCADE_S: ");
  Serial1.println(arcade_s_input);
  Serial1.print("ARCADE_W: ");
  Serial1.println(arcade_w_input);
  Serial1.print("ARCADE_E: ");
  Serial1.println(arcade_e_input);

  // Enable and disable LEDs sequentially with a half-second delay
  for (int i = 13; i >= 8; i--)
  {
    mcp.digitalWrite(i, HIGH); // Turn on LED
    delay(100);                // Wait for 100 milliseconds
    mcp.digitalWrite(i, LOW);  // Turn off LED
  }
}
