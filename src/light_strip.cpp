#include "Arduino.h"
#include "light_strip.hpp"

// Constructor: Initialize the NeoPixel strip with the specified pin and number
// of LEDs
LightStrip::LightStrip() : m_strip(N_LEDS, NEOPIXEL_RING, NEO_GRB + NEO_KHZ800)
{}

void LightStrip::initialize()
{
  m_strip.begin();
  m_strip.setBrightness(4);
  uint32_t black = m_strip.Color(0, 0, 0);
  m_strip.fill(m_strip.Color(0, 0, 0), 0, N_LEDS);
  m_strip.show(); // Initialize all pixels to 'off'
}

// Function to display a rainbow pattern on the NeoPixel strip
void LightStrip::rainbow(bool enable)
{
  if (enable)
  {
    m_strip.setBrightness(3);

    for (int i = 0; i < m_strip.numPixels(); i++)
    {
      int pixelHue = (i * 65536L / m_strip.numPixels() + m_hueShift) % 65536;
      m_strip.setPixelColor(i, m_strip.gamma32(m_strip.ColorHSV(pixelHue)));
    }
    m_strip.show();

    m_hueShift += 500; // Adjust the hue shift increment for the color variation

    if (m_hueShift >= 65536)
    {
      m_hueShift = 0;
    }
  }
  else
  {
    m_strip.fill(m_strip.Color(0, 0, 0), 0, N_LEDS);
    m_strip.setBrightness(0);
    m_strip.show();
  }
}
