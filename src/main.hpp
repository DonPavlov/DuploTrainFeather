#ifndef MAIN_HPP
#define MAIN_HPP
#include <Arduino.h>
#include "Lpf2Hub.h"

void rainbowCycle(int delayTime);

void powerSaving();
bool checkPowerSaveNeeded();

#endif  // MAIN_HPP
