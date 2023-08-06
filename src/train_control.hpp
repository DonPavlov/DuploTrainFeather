// Train Control class implements ControlInterface.hpp
#include "ControlInterface.hpp"
#include "Lpf2Hub.h"

class TrainControl : public ControlInterface {
  /**
     This class is the main class of the train control.
     It implements the main loop of the train control.
   **/

public:

  TrainControl(Lpf2Hub hub);
  ~TrainControl() = default;

  bool SendCommand(Commands::Commands cmd) override;
  bool SendSpeed(Commands::Commands cmd,
                 int8_t             speed) override;

private:

  Lpf2Hub m_hub;
};
