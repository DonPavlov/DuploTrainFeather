#ifndef IO_HPP
#define IO_HPP
#include "commands.hpp"
#include "train_control.hpp"
#include <map>
#include <cstdint>
#include <string>
#include <Adafruit_MCP23X17.h>

#define ARCADE_N (16)
#define ARCADE_S (15)
#define ARCADE_W (14)
#define ARCADE_E (8)

class IO {
  /**
     This class is a button factory. With it it is able to register a
        buttonpress to a specific function.
     IO use interrupts to register a buttonpress.
     The interrupt executes the registered function with its parameters.
   **/

public:

  IO();
  ~IO() = default;
  enum class BtnNr {
    BTN0,
    BTN1,
    BTN2,
    BTN3,
    BTN4,
    BTN5,
    BTN_A_N,
    BTN_A_S,
    BTN_A_W,
    BTN_A_E,
    MAX_BTN
  };

  void init_buttons();

  // Function to register a button to a specific function and parameters
  void register_button(BtnNr              buttonNr,
                       Commands::Commands command);

  void read_buttons();

  void init_ctrl(TrainControl& ctrl);

private:

  TrainControl m_ctrl;
  bool ctrl_initalized { false };
  std::map<BtnNr, Commands::Commands>m_buttons;
  bool execute_command(BtnNr buttonNr);
  Adafruit_MCP23X17 mcp;
};

#endif // ifndef IO_HPP
