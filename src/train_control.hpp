#pragma once

// Train Control class implements ControlInterface.hpp
#include "ControlInterface.hpp"
#include "Lpf2Hub.h"
#include <cstdint>

class TrainControl : public ControlInterface {
  /**
     This class is the main class of the train control.
     It implements the main loop of the train control.
   **/

public:

  TrainControl();
  ~TrainControl() = default;

  bool   SendCommand(Commands::Commands cmd) override;
  bool   SendSpeed(Commands::Commands cmd,
                   int8_t             speed) override;

  void   increase_speed();
  void   decrease_speed();
  int8_t get_speed();
  void   init();
  void   stateMachine();

private:
};
