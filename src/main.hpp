#ifndef MAIN_HPP
#define MAIN_HPP
#include <Arduino.h>
#include "Lpf2Hub.h"
#include <cstdint>
#include <commands.hpp>
#include <map>


void   rainbowCycle(int delayTime);

void   powerSaving();
bool   checkPowerSaveNeeded();

void   SendCommand(Commands::Commands cmd);
bool   SendSpeed(Commands::Commands cmd,
                 int8_t             speed);

void   increaseSpeed();
void   decreaseSpeed();
int8_t getSpeed();
void   init();
void   stateMachine();
bool   checkConnectionToTrain();


#endif  // MAIN_HPP
