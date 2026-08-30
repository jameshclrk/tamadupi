#include "step_tracker.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define GRAVITY_FILTER_ALPHA 0.02f
#define MOTION_FILTER_ALPHA 0.35f
#define PEAK_THRESHOLD_MPS2 0.75f
#define VALLEY_THRESHOLD_MPS2 (-0.35f)
#define MIN_PROMINENCE_MPS2 1.15f
#define MIN_STEP_INTERVAL_MS 280
#define MAX_STEP_INTERVAL_MS 1300
#define CADENCE_TOLERANCE 0.45f
#define WALK_CONFIRMATION_STEPS 3

void step_tracker_init(step_tracker_t *tracker)
{
    if (tracker != NULL) {
        memset(tracker, 0, sizeof(*tracker));
    }
}

static bool interval_is_consistent(uint32_t interval_ms, uint32_t previous_interval_ms)
{
    if (previous_interval_ms == 0) {
        return true;
    }
    const float difference = fabsf((float)interval_ms - (float)previous_interval_ms);
    return difference <= (float)previous_interval_ms * CADENCE_TOLERANCE;
}

static step_tracker_result_t result(const step_tracker_t *tracker, uint8_t steps)
{
    return (step_tracker_result_t) {
        .confirmed_steps = steps,
        .cadence_spm = tracker->cadence_spm,
        .walking = tracker->walking,
    };
}

step_tracker_result_t step_tracker_update(step_tracker_t *tracker,
                                          float accel_magnitude_mps2,
                                          int64_t timestamp_ms)
{
    if (tracker == NULL || !isfinite(accel_magnitude_mps2)) {
        return (step_tracker_result_t) {0};
    }

    if (!tracker->initialized) {
        tracker->initialized = true;
        tracker->gravity_mps2 = accel_magnitude_mps2;
        return result(tracker, 0);
    }

    tracker->gravity_mps2 += (accel_magnitude_mps2 - tracker->gravity_mps2) *
                             GRAVITY_FILTER_ALPHA;
    const float linear_accel = accel_magnitude_mps2 - tracker->gravity_mps2;
    tracker->filtered_accel += (linear_accel - tracker->filtered_accel) *
                               MOTION_FILTER_ALPHA;

    if (tracker->filtered_accel < tracker->valley_accel) {
        tracker->valley_accel = tracker->filtered_accel;
    }

    uint8_t confirmed_steps = 0;
    const bool found_peak = tracker->previous_accel > tracker->filtered_accel &&
                            tracker->previous_accel >= PEAK_THRESHOLD_MPS2 &&
                            tracker->valley_accel <= VALLEY_THRESHOLD_MPS2 &&
                            tracker->previous_accel - tracker->valley_accel >=
                            MIN_PROMINENCE_MPS2;

    if (found_peak) {
        const uint32_t interval_ms = tracker->last_candidate_ms == 0 ? 0 :
            (uint32_t)(timestamp_ms - tracker->last_candidate_ms);
        const bool plausible_interval = interval_ms >= MIN_STEP_INTERVAL_MS &&
                                        interval_ms <= MAX_STEP_INTERVAL_MS;

        if (tracker->last_candidate_ms == 0 || !plausible_interval) {
            tracker->walking = false;
            tracker->cadence_spm = 0;
            tracker->pending_candidates = 1;
            tracker->last_interval_ms = 0;
        } else if (!interval_is_consistent(interval_ms, tracker->last_interval_ms)) {
            tracker->walking = false;
            tracker->cadence_spm = 0;
            tracker->pending_candidates = 1;
            tracker->last_interval_ms = 0;
        } else {
            tracker->cadence_spm = (uint16_t)(60000U / interval_ms);
            tracker->last_interval_ms = interval_ms;
            if (tracker->walking) {
                confirmed_steps = 1;
            } else {
                ++tracker->pending_candidates;
                if (tracker->pending_candidates >= WALK_CONFIRMATION_STEPS) {
                    tracker->walking = true;
                    confirmed_steps = tracker->pending_candidates;
                    tracker->pending_candidates = 0;
                }
            }
        }

        tracker->last_candidate_ms = timestamp_ms;
        tracker->valley_accel = tracker->filtered_accel;
    } else if (tracker->last_candidate_ms != 0 &&
               timestamp_ms - tracker->last_candidate_ms > MAX_STEP_INTERVAL_MS) {
        tracker->walking = false;
        tracker->cadence_spm = 0;
        tracker->pending_candidates = 0;
        tracker->last_interval_ms = 0;
    }

    tracker->previous_accel = tracker->filtered_accel;
    return result(tracker, confirmed_steps);
}
