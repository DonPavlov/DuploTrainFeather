#ifndef MAIN_HPP
#define MAIN_HPP
#include <Arduino.h>
#include "Lpf2Hub.h"

void        rainbowCycle(int delayTime);

void        test_inputs();
static void speedometerSensorCallback(void      *hub,
                                      byte       portNumber,
                                      DeviceType deviceType,
                                      uint8_t   *pData);


static void colorSensorCallback(void      *hub,
                                byte       portNumber,
                                DeviceType deviceType,
                                uint8_t   *pData);
#endif  // MAIN_HPP
