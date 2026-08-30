#pragma once

#if __has_include("secrets.h")
#include "secrets.h"
#else
#define TAMADUPI_WIFI_SSID ""
#define TAMADUPI_WIFI_PASSWORD ""
#define TAMADUPI_LATITUDE 0.0
#define TAMADUPI_LONGITUDE 0.0
#endif

#define TAMADUPI_CONFIGURED (TAMADUPI_WIFI_SSID[0] != '\0')
