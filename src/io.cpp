#include "io.hpp"
#include "commands.hpp"
#include "Arduino.h"
#include "Esp.h"
#include <array>
#include <string>

//
IO::IO()
{
  // give every button used a fitting command
}

void IO::init_buttons()
{
  // Set the inputs for
  pinMode(JOY_N, INPUT_PULLUP);
  pinMode(JOY_S, INPUT_PULLUP);
  pinMode(JOY_W, INPUT_PULLUP);
  pinMode(JOY_E, INPUT_PULLUP);

  // configure IO Expander
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
  char cstr[32] = { 0 };

  // Configure Port A pins 0 to 5 as INPUT and enable pull-up resistors on Expandr
  for (int i = 0; i < 6; i++)
  {
    sprintf(cstr, "Input Pin %d", i);
    Serial1.println(cstr);
    mcp.pinMode(i, INPUT_PULLUP);
  }

  // Configure Port B pins B5 to B0 as OUTPUT on Expandr
  for (int i = 13; i >= 8; i--)
  {
    sprintf(cstr, "Output Pin %d", i);
    Serial1.println(cstr);
    mcp.pinMode(i, OUTPUT);
  }

  // initialize all IO by using the register button function and assign a

  // Setup Pullup and Inputs

  register_button(btn::BtnNr::BTN0,    Commands::Commands::Stop);
  register_button(btn::BtnNr::BTN1,    Commands::Commands::Light);
  register_button(btn::BtnNr::BTN2,    Commands::Commands::Refill);
  register_button(btn::BtnNr::BTN3,    Commands::Commands::Horn);
  register_button(btn::BtnNr::BTN4,    Commands::Commands::Steam);
  register_button(btn::BtnNr::BTN5,    Commands::Commands::Departure);
  register_button(btn::BtnNr::BTN_J_N, Commands::Commands::Forward);
  register_button(btn::BtnNr::BTN_J_S, Commands::Commands::Backward);
  register_button(btn::BtnNr::BTN_J_W, Commands::Commands::Faster);
  register_button(btn::BtnNr::BTN_J_E, Commands::Commands::Slower);
}

//
void IO::register_button(btn::BtnNr buttonNr, Commands::Commands command)
{
  btn::ButtonData buttonData;

  buttonData.command       = command;
  buttonData.currentValue  = 1; // Initialize current value
  buttonData.previousValue = 1; // Initialize previous value

  m_buttonsData[buttonNr] = buttonData;
}

// TODO add button lighting to do something, while each button is pressed enable LED.
void IO::read_buttons()
{
  uint8_t reg_A = mcp.readGPIOA();

  // split up into array
  constexpr uint8_t arcade_button_nr = 6;
  std::array<uint8_t, arcade_button_nr> arcade_buttons;

  // Update arcade button data
  for (size_t i = 0; i < arcade_button_nr; i++)
  {
    arcade_buttons[i] = (reg_A >> i) & 0x1;

    btn::BtnNr buttonNr         = static_cast<btn::BtnNr>(i);
    btn::ButtonData& buttonData = m_buttonsData[buttonNr];
    buttonData.previousValue = buttonData.currentValue;
    buttonData.currentValue  = arcade_buttons[i];

    // Compare previous and current values and execute the command if needed
    if ((buttonData.currentValue == LOW) && (buttonData.previousValue == HIGH))
    {
      execute_command(buttonNr);
    }

    // TODO add button lighting to do something
    // While each button is pressed, enable LED
    if (buttonData.currentValue == LOW)
    {
      // Implement your LED lighting logic here
      // For example:
      // turn_on_led(buttonNr);
    }
    else
    {
      // Implement your LED turn-off logic here
      // For example:
      // turn_off_led(buttonNr);
    }
  }


  // Only one joystick is high at a time.
  uint8_t joy_n_input = digitalRead(JOY_N);
  uint8_t joy_s_input = digitalRead(JOY_S);
  uint8_t joy_w_input = digitalRead(JOY_W);
  uint8_t joy_e_input = digitalRead(JOY_E);

  // Joystick button data
  btn::BtnNr joystickButton = btn::BtnNr::MAX_BTN; // Default value for joystick button

  // Update joystick button data
  if (joy_n_input == LOW)
  {
    joystickButton = btn::BtnNr::BTN_J_N;
  }
  else if (joy_s_input == LOW)
  {
    joystickButton = btn::BtnNr::BTN_J_S;
  }
  else if (joy_w_input == LOW)
  {
    joystickButton = btn::BtnNr::BTN_J_W;
  }
  else if (joy_e_input == LOW)
  {
    joystickButton = btn::BtnNr::BTN_J_E;
  }

  // Reset other joystick button values
  for (size_t i = static_cast<int>(btn::BtnNr::BTN_J_N); i < static_cast<int>(btn::BtnNr::MAX_BTN); i++)
  {
    btn::BtnNr buttonNr         = static_cast<btn::BtnNr>(i);
    btn::ButtonData& buttonData = m_buttonsData[buttonNr];
    buttonData.previousValue = buttonData.currentValue;

    if (buttonNr != joystickButton)
    {
      buttonData.currentValue = HIGH;
    }
    else
    {
      buttonData.currentValue = LOW;
    }

    // Compare previous and current values and execute the command if needed
    if ((buttonData.currentValue == LOW) && (buttonData.previousValue == HIGH))
    {
      execute_command(buttonNr);
    }
  }
}

void IO::execute_command(btn::BtnNr buttonNr)
{
  btn::ButtonData& buttonData = m_buttonsData[buttonNr];

  lastActivityTime = millis();
  SendCommand(buttonData.command);
}
