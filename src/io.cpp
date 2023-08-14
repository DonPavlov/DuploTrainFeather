#include "io.hpp"
#include "commands.hpp"
#include "Arduino.h"
#include "Esp.h"

//
IO::IO()
{
  // give every button used a fitting command
}

void IO::init_buttons()
{
  // Set the inputs for
  pinMode(ARCADE_N, INPUT_PULLUP);
  pinMode(ARCADE_S, INPUT_PULLUP);
  pinMode(ARCADE_W, INPUT_PULLUP);
  pinMode(ARCADE_E, INPUT_PULLUP);

  // configure IO
  if (!mcp.begin_I2C())
  {
    Serial1.println("Error.");
    int i = 0;

    while (1)
    {
      i++;
      delay(1000);
      Serial1.println("Unable to init MCP23017");

      if (i > 10)
      {
        ESP.restart();
      }
    }
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

  // initialize all IO by using the register button function and assign a
  // command to it

  // Setup Pullup and Inputs

  register_button(IO::BtnNr::BTN0,    Commands::Commands::Stop);
  register_button(IO::BtnNr::BTN1,    Commands::Commands::Light);
  register_button(IO::BtnNr::BTN2,    Commands::Commands::Refill);
  register_button(IO::BtnNr::BTN3,    Commands::Commands::Horn);
  register_button(IO::BtnNr::BTN4,    Commands::Commands::Steam);
  register_button(IO::BtnNr::BTN5,    Commands::Commands::Departure);
  register_button(IO::BtnNr::BTN_A_N, Commands::Commands::Forward);
  register_button(IO::BtnNr::BTN_A_S, Commands::Commands::Backward);
}

//
void IO::register_button(BtnNr buttonNr, Commands::Commands command)
{
  m_buttons.insert(std::pair<BtnNr, Commands::Commands>(buttonNr, command));
}

void IO::init_ctrl(TrainControl& ctrl)
{
  m_ctrl = ctrl;
}

//
void IO::read_buttons()
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

  for (int i = 13; i >= 8; i--)
  {
    mcp.digitalWrite(i, HIGH); // Turn on LED
    delay(33);
  }

  for (int i = 13; i >= 8; i--)
  {
    mcp.digitalWrite(i, LOW); // Turn off LED
    delay(33);
  }
}

bool IO::execute_command(BtnNr buttonNr)
{
  // iterate over m_buttons and if the button matches execute the command saved
  // in the map
  for (auto& button : m_buttons)
  {
    if (button.first == buttonNr)
    {
      m_ctrl.SendCommand(button.second);
      return true;
    }
  }
  return false;
}
