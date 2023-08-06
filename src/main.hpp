#ifndef MAIN_HPP
#define MAIN_HPP
#include <Arduino.h>

#ifdef DEBUGWIFI
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Wifi.h>
#include "secrets.hpp"
#include "RemoteDebug.h" // https://github.com/JoaoLopesF/RemoteDebug
#define HOST_NAME "feather"
#define USE_MDNS true

// Set your Gateway IP address
IPAddress staticIP(192, 168, 178, 123);
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);
#endif // ifdef DEBUGWIFI


#endif  // MAIN_HPP
