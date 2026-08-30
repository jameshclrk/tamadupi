#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t health;
    uint8_t social;
    uint16_t nearby_devices;
    float activity_level;
} pet_state_snapshot_t;

esp_err_t pet_state_init(void);
void pet_state_record_activity(float motion_intensity, float elapsed_seconds);
void pet_state_set_nearby_devices(uint16_t device_count);
pet_state_snapshot_t pet_state_snapshot(void);
