#ifndef IO_HPP
#define IO_HPP
#include "commands.hpp"
#include "train_control.hpp"
#include <unordered_map>
#include <cstdint>
#include <string>
#include <Adafruit_MCP23X17.h>

#define JOY_N (16)
#define JOY_S (15)
#define JOY_W (14)
#define JOY_E (8)

extern unsigned long lastActivityTime;
namespace btn
{
  struct ButtonData {
    Commands::Commands command;
    uint8_t            currentValue;
    uint8_t            previousValue;

    ButtonData() : command{Commands::Commands::Stop}, currentValue{1}, previousValue{1}
    {}
  };

  enum class BtnNr {
    BTN0,
    BTN1,
    BTN2,
    BTN3,
    BTN4,
    BTN5,
    BTN_J_N,
    BTN_J_S,
    BTN_J_W,
    BTN_J_E,
    MAX_BTN
  };
}

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


  void init_buttons();

  // Function to register a button to a specific function and parameters
  void register_button(btn::BtnNr         buttonNr,
                       Commands::Commands command);

  void read_buttons();

  void init_ctrl(TrainControl& ctrl);

private:

  TrainControl m_ctrl;
  bool ctrl_initalized { false };
  std::unordered_map<btn::BtnNr, btn::ButtonData>m_buttonsData;
  void execute_command(btn::BtnNr buttonNr);
  Adafruit_MCP23X17 mcp;
};

#endif // ifndef IO_HPP
