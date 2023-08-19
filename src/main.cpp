#include "main.hpp"
#include <cstring>
#include <string>

#include "train_control.hpp"
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

// UART1 GPIO 18 RX GPIO17 TX
#define RXD1 (18)
#define TXD1 (17)


// Set your Static IP address
IPAddress local_IP(192, 168, 178, 123);

// Set your Gateway IP address
IPAddress gateway(192, 168, 178, 1);

IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);      // optional
IPAddress secondaryDNS(8, 8, 4, 4);    // optional

unsigned long startMillis;             // some global variables available anywhere in
                                       // the program
unsigned long currentMillis;
unsigned long wifiMillis;
const unsigned long period       = 50; // the value is a number of milliseconds
const unsigned long wifi_timeout = 10000;
bool wifiConSkipped              = false;
bool wifiSetupfinished           = false;


LightStrip myStrip; // Create an instance of the LightStrip class

Adafruit_7segment matrix = Adafruit_7segment();
TrainControl zug;
IO io_ctrl;

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
  matrix.begin(0x70);              // Init I2C Display


  static uint8_t number = 0;

  // TODO move into TrainControl
  matrix.print("1234");
  matrix.writeDisplay();
  myStrip.initialize();

  digitalWrite(LED_BUILTIN, LOW);      // turn the LED off
  startMillis = wifiMillis = millis(); // initial start time


  // Init wifi for ota updates
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
  {
    Serial1.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  zug.init();
  io_ctrl.init_ctrl(zug);
  myStrip.rainbow(true);
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
    // myStrip.rainbow(true );
    // TODO fix rainbow effect, make it only continue if function is reentered and continue were it was before.
    startMillis = currentMillis; // IMPORTANT to save the start
                                 // time of the current LED
                                 // state.
    io_ctrl.read_buttons();
  }

  // TODO only execute if necessary, make sure train can reconnect if connection is lost.
  zug.stateMachine();

  // TODO add timeout if no button press occured for 10 minutes and shutdown most of the functionality, until a button
  // is pressed again or reboot. Or just disable wifi after 5 min to save power but might also be necessary for arduino
  // OTA to have changes....

  // TODO add function to stop after 5 minutes so battery can charge
  // test_inputs();
}
