#include "buttons.hpp"
#include "commands.hpp"

// ButtonList
// IO4 = BTN0 = 4
// IO5 = BTN1 = 5
// IO6 = BTN2 = 6
// IO7 = BTN3 = 7
// IO15 = BTN4 = 8
// BOOT ?
// RESET ?
// Implement buttons.hpp

//
Buttons::Buttons(ControlInterface ctrl) : m_ctrl(ctrl)
{
  // give every button used a fitting command
  init_buttons();
}

void Buttons::init_buttons()
{
  // initialize all buttons by using the register button function and assign a
  // command to it

  register_button(Buttons::BtnNr::BTN0, Commands::Commands::Stop);
  register_button(Buttons::BtnNr::BTN1, Commands::Commands::Light);
  register_button(Buttons::BtnNr::BTN2, Commands::Commands::Refill);
  register_button(Buttons::BtnNr::BTN3, Commands::Commands::Horn);
  register_button(Buttons::BtnNr::BTN4, Commands::Commands::Steam);
}

//
void Buttons::register_button(BtnNr buttonNr, Commands::Commands command)
{
  m_buttons.insert(std::pair<BtnNr, Commands::Commands>(buttonNr, command));
}

//
void Buttons::read_buttons()
{
  // read all buttons and execute a command if the button is pressed,
}

bool Buttons::execute_command(BtnNr buttonNr)
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
