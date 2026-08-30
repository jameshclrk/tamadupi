#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t health;
    uint8_t social;
    uint16_t nearby_devices;
    uint16_t cadence_spm;
    uint32_t steps;
    uint8_t social_multiplier;
    float activity_level;
    bool walking;
} pet_state_snapshot_t;

esp_err_t pet_state_init(void);
void pet_state_record_pedometer(uint8_t confirmed_steps, uint16_t cadence_spm,
                                bool walking);
void pet_state_set_nearby_devices(uint16_t device_count);
pet_state_snapshot_t pet_state_snapshot(void);
