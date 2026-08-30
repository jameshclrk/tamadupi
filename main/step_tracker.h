#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool initialized;
    bool walking;
    float gravity_mps2;
    float filtered_accel;
    float previous_accel;
    float valley_accel;
    int64_t last_candidate_ms;
    uint32_t last_interval_ms;
    uint16_t cadence_spm;
    uint8_t pending_candidates;
} step_tracker_t;

typedef struct {
    uint8_t confirmed_steps;
    uint16_t cadence_spm;
    bool walking;
} step_tracker_result_t;

void step_tracker_init(step_tracker_t *tracker);
step_tracker_result_t step_tracker_update(step_tracker_t *tracker,
                                          float accel_magnitude_mps2,
                                          int64_t timestamp_ms);
