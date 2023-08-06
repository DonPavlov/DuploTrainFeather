#ifndef CONTROL_INTERFACE_HPP
#define CONTROL_INTERFACE_HPP
#include "commands.hpp"
#include <stdint.h>

class ControlInterface {
public:

  virtual bool SendCommand(Commands::Commands cmd);
  virtual bool SendSpeed(Commands::Commands cmd,
                         int8_t             speed);

private:
};

#endif // ifndef CONTROL_INTERFACE_HPP
