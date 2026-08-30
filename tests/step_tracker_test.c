#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "step_tracker.h"

#define PI 3.14159265358979323846f
#define GRAVITY_MPS2 9.80665f
#define SAMPLE_MS 40

static uint32_t feed_walking(step_tracker_t *tracker, int steps, int period_ms)
{
    uint32_t detected = 0;
    const int samples = steps * period_ms / SAMPLE_MS;
    for (int sample = 0; sample < samples; ++sample) {
        const int64_t time_ms = (int64_t)sample * SAMPLE_MS;
        const float phase = 2.0f * PI * (float)(time_ms % period_ms) / (float)period_ms;
        const float walking_signal = 1.75f * sinf(phase) + 0.30f * sinf(phase * 2.0f);
        detected += step_tracker_update(tracker, GRAVITY_MPS2 + walking_signal,
                                        time_ms).confirmed_steps;
    }
    return detected;
}

static void detects_walking_cadence(void)
{
    step_tracker_t tracker;
    step_tracker_init(&tracker);
    const uint32_t detected = feed_walking(&tracker, 12, 500);
    assert(detected >= 10 && detected <= 12);
    assert(tracker.walking);
    assert(tracker.cadence_spm >= 110 && tracker.cadence_spm <= 130);
}

static void ignores_tilting_and_sensor_noise(void)
{
    step_tracker_t tracker;
    step_tracker_init(&tracker);
    uint32_t detected = 0;
    for (int sample = 0; sample < 300; ++sample) {
        const float noise = 0.08f * sinf((float)sample * 0.37f);
        detected += step_tracker_update(&tracker, GRAVITY_MPS2 + noise,
                                        (int64_t)sample * SAMPLE_MS).confirmed_steps;
    }
    assert(detected == 0);
}

static void ignores_isolated_bumps(void)
{
    step_tracker_t tracker;
    step_tracker_init(&tracker);
    uint32_t detected = 0;
    for (int sample = 0; sample < 400; ++sample) {
        const int position = sample % 75;
        float signal = 0.0f;
        if (position == 10) signal = -1.1f;
        if (position == 11) signal = 2.5f;
        if (position == 12) signal = 0.8f;
        detected += step_tracker_update(&tracker, GRAVITY_MPS2 + signal,
                                        (int64_t)sample * SAMPLE_MS).confirmed_steps;
    }
    assert(detected == 0);
}

static void ignores_fast_shaking(void)
{
    step_tracker_t tracker;
    step_tracker_init(&tracker);
    uint32_t detected = 0;
    for (int sample = 0; sample < 250; ++sample) {
        const float signal = 2.2f * sinf(2.0f * PI * (float)sample / 4.0f);
        detected += step_tracker_update(&tracker, GRAVITY_MPS2 + signal,
                                        (int64_t)sample * SAMPLE_MS).confirmed_steps;
    }
    assert(detected == 0);
}

int main(void)
{
    detects_walking_cadence();
    ignores_tilting_and_sensor_noise();
    ignores_isolated_bumps();
    ignores_fast_shaking();
    puts("step_tracker tests passed");
    return 0;
}
