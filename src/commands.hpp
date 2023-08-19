#ifndef COMMANDS_HPP
#define COMMANDS_HPP
#include <map>
#include <string>

namespace Commands
{
  enum class Commands {
    Forward   = 0x0,
    Backward  = 0x1,
    Stop      = 0x2,
    Light     = 0x3,
    Refill    = 0x4,
    Horn      = 0x5,
    Steam     = 0x6,
    Departure = 0x7,
    Faster    = 0x8,
    Slower    = 0x9,
    None      = 0x10
  };
}


#endif // ifndef COMMANDS_HPP
