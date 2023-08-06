#include "train_control.hpp"


TrainControl::TrainControl(Lpf2Hub hub) : m_hub(hub)
{
  // TODO init the bluetooth train interface
}

bool TrainControl::SendCommand(Commands::Commands cmd)
{
  // TODO send something with the bluetooth train interface
  bool result = false;

  return result;
}

bool TrainControl::SendSpeed(Commands::Commands cmd,
                             int8_t             speed)
{
  bool result = false;

  return result;
}
