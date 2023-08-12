// Header File to create a class to init the NeoPixel Leds and control them

#ifndef LIGHT_STRIP_H_
#define LIGHT_STRIP_H_
#include "Adafruit_NeoPixel.h"
#include <Adafruit_GFX.h>
#include <cstdint>

#define NEOPIXEL_RING (6)
#define N_LEDS (16)


class LightStrip {
private:

  // map represents the pin and the
  Adafruit_NeoPixel m_strip;

public:

  // init neopixel strip and enable default behaviour of rainbow colored output
  LightStrip();

  ~LightStrip() = default;

  void initialize();

  // Function to set some rainbow patterns if enable is true
  void rainbow(bool enable);
};

#endif // ifndef LIGHT_STRIP_H_
