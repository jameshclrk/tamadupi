#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    WEATHER_STATUS_NEEDS_CONFIG,
    WEATHER_STATUS_CONNECTING,
    WEATHER_STATUS_UPDATING,
    WEATHER_STATUS_READY,
    WEATHER_STATUS_ERROR,
} weather_status_t;

typedef struct {
    weather_status_t status;
    bool wifi_connected;
    bool is_day;
    float temperature_c;
    int weather_code;
    uint32_t generation;
} weather_snapshot_t;

esp_err_t weather_service_start(void);
weather_snapshot_t weather_service_snapshot(void);
