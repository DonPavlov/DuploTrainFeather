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

bool   SendCommand(Commands::Commands cmd);
bool   SendSpeed(Commands::Commands cmd,
                 int8_t             speed);

void   increase_speed();
void   decrease_speed();
int8_t get_speed();
void   init();
void   stateMachine();
bool   checkConnectionToTrain();


#endif  // MAIN_HPP
