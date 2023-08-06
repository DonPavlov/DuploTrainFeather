#ifndef COMMANDS_HPP
#define COMMANDS_HPP

namespace Commands
{
  enum class Commands {
    Forward   = 0x0,
    Backwards = 0x1,
    Stop      = 0x2,
    Light     = 0x3,
    Refill    = 0x4,
    Horn      = 0x5,
    Steam     = 0x6,
    Departure = 0x7,
    None      = 0x8
  };
}

#endif // ifndef COMMANDS_HPP
