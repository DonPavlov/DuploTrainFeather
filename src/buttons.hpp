#ifndef BUTTONS_HPP
#define BUTTONS_HPP
#include "commands.hpp"
#include "ControlInterface.hpp"
#include <map>
#include <cstdint>
#include <string>

class Buttons {
  /**
     This class is a button factory. With it it is able to register a
        buttonpress to a specific function.
     Buttons use interrupts to register a buttonpress.
     The interrupt executes the registered function with its parameters.
   **/

public:

  Buttons(ControlInterface ctrl);
  ~Buttons() = default;
  enum class BtnNr {
    BTN0,
    BTN1,
    BTN2,
    BTN3,
    BTN4,
    MAX_BTN
  };

  void init_buttons();

  // Function to register a button to a specific function and parameters
  void register_button(BtnNr              buttonNr,
                       Commands::Commands command);

  void read_buttons();

private:

  ControlInterface m_ctrl;
  std::map<BtnNr, Commands::Commands>m_buttons;
  bool execute_command(BtnNr buttonNr);
};

#endif // ifndef BUTTONS_HPP
