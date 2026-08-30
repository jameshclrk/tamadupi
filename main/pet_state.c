#include "pet_state.h"

#include <stdbool.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#define DEFAULT_HEALTH 65.0f
#define DEFAULT_SOCIAL 35.0f
#define NEEDS_UPDATE_PERIOD_MS 60000
#define NEEDS_SAVE_INTERVAL_TICKS 5
#define HEALTH_PER_STEP 0.50f

static const char *TAG = "pet_state";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static nvs_handle_t s_nvs;
static float s_health = DEFAULT_HEALTH;
static float s_social = DEFAULT_SOCIAL;
static float s_activity_level;
static uint16_t s_nearby_devices;
static uint16_t s_cadence_spm;
static uint32_t s_steps;
static bool s_walking;
static uint8_t s_last_saved_health;
static uint8_t s_last_saved_social;
static uint32_t s_last_saved_steps;

static float clamp_score(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 100.0f) {
        return 100.0f;
    }
    return value;
}

static uint8_t rounded_score(float value)
{
    return (uint8_t)(clamp_score(value) + 0.5f);
}

static void save_if_changed(void)
{
    uint8_t health;
    uint8_t social;
    uint32_t steps;

    portENTER_CRITICAL(&s_lock);
    health = rounded_score(s_health);
    social = rounded_score(s_social);
    steps = s_steps;
    portEXIT_CRITICAL(&s_lock);

    if (health == s_last_saved_health && social == s_last_saved_social &&
        steps == s_last_saved_steps) {
        return;
    }

    esp_err_t ret = nvs_set_u8(s_nvs, "health", health);
    if (ret == ESP_OK) {
        ret = nvs_set_u8(s_nvs, "social", social);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u32(s_nvs, "steps", steps);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(s_nvs);
    }
    if (ret == ESP_OK) {
        s_last_saved_health = health;
        s_last_saved_social = social;
        s_last_saved_steps = steps;
        ESP_LOGI(TAG, "Saved needs: health=%u social=%u steps=%lu",
                 health, social, (unsigned long)steps);
    } else {
        ESP_LOGW(TAG, "Could not save needs: %s", esp_err_to_name(ret));
    }
}

static void needs_task(void *arg)
{
    (void)arg;
    uint32_t save_tick = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(NEEDS_UPDATE_PERIOD_MS));

        portENTER_CRITICAL(&s_lock);
        if (s_activity_level < 0.12f) {
            s_health = clamp_score(s_health - 0.08f);
        }
        s_activity_level *= 0.35f;
        s_walking = false;
        s_cadence_spm = 0;

        const float social_target = clamp_score(15.0f + (float)s_nearby_devices * 11.0f);
        s_social = clamp_score(s_social + (social_target - s_social) * 0.16f);
        portEXIT_CRITICAL(&s_lock);

        if (++save_tick >= NEEDS_SAVE_INTERVAL_TICKS) {
            save_tick = 0;
            save_if_changed();
        }
    }
}

esp_err_t pet_state_init(void)
{
    esp_err_t ret = nvs_open("tamadupi", NVS_READWRITE, &s_nvs);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t stored_health = 0;
    uint8_t stored_social = 0;
    uint32_t stored_steps = 0;
    if (nvs_get_u8(s_nvs, "health", &stored_health) == ESP_OK) {
        s_health = stored_health;
    }
    if (nvs_get_u8(s_nvs, "social", &stored_social) == ESP_OK) {
        s_social = stored_social;
    }
    if (nvs_get_u32(s_nvs, "steps", &stored_steps) == ESP_OK) {
        s_steps = stored_steps;
    }
    s_last_saved_health = rounded_score(s_health);
    s_last_saved_social = rounded_score(s_social);
    s_last_saved_steps = s_steps;

    if (xTaskCreate(needs_task, "pet_needs", 3072, NULL, 3, NULL) != pdPASS) {
        nvs_close(s_nvs);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Needs ready: health=%u social=%u steps=%lu",
             rounded_score(s_health), rounded_score(s_social),
             (unsigned long)s_steps);
    return ESP_OK;
}

void pet_state_record_pedometer(uint8_t confirmed_steps, uint16_t cadence_spm,
                                bool walking)
{
    uint32_t total_steps;

    portENTER_CRITICAL(&s_lock);
    s_walking = walking;
    s_cadence_spm = walking ? cadence_spm : 0;
    if (confirmed_steps > 0) {
        s_steps += confirmed_steps;
        s_health = clamp_score(s_health + (float)confirmed_steps * HEALTH_PER_STEP);
        const float cadence_level = clamp_score((float)cadence_spm * 100.0f / 130.0f) / 100.0f;
        if (cadence_level > s_activity_level) {
            s_activity_level = cadence_level;
        }
    }
    total_steps = s_steps;
    portEXIT_CRITICAL(&s_lock);

    if (confirmed_steps > 1 ||
        (confirmed_steps == 1 && total_steps % 25U == 0)) {
        ESP_LOGI(TAG, "Pedometer: %lu steps (%u spm)",
                 (unsigned long)total_steps, cadence_spm);
    }
}

void pet_state_set_nearby_devices(uint16_t device_count)
{
    portENTER_CRITICAL(&s_lock);
    s_nearby_devices = device_count;
    portEXIT_CRITICAL(&s_lock);
}

pet_state_snapshot_t pet_state_snapshot(void)
{
    pet_state_snapshot_t snapshot;
    portENTER_CRITICAL(&s_lock);
    snapshot.health = rounded_score(s_health);
    snapshot.social = rounded_score(s_social);
    snapshot.nearby_devices = s_nearby_devices;
    snapshot.cadence_spm = s_cadence_spm;
    snapshot.steps = s_steps;
    snapshot.activity_level = s_activity_level;
    snapshot.walking = s_walking;
    portEXIT_CRITICAL(&s_lock);
    return snapshot;
}
