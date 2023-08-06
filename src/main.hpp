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

#endif // ifdef DEBUGWIFI


#endif  // MAIN_HPP
