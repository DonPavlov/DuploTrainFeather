#ifndef TRAIN_CONTROL_HPP
#define TRAIN_CONTROL_HPP

// Train Control class implements ControlInterface.hpp
#include "ControlInterface.hpp"
#include "Lpf2Hub.h"
#include <cstdint>
#include "commands.hpp"


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
  bool   checkConnectionToTrain();

  void   speedometerSensorCb(void      *hub,
                             byte       portNumber,
                             DeviceType deviceType,
                             uint8_t   *pData);
  void colorSensorCb(void      *hub,
                     byte       portNumber,
                     DeviceType deviceType,
                     uint8_t   *pData);

private:

  bool m_connected
  {
    false
  };

  Lpf2Hub m_Hub;
  volatile int8_t g_speed     = 0;
  volatile int8_t g_lastSpeed = 0;
};

#endif // ifndef TRAIN_CONTROL_HPP
