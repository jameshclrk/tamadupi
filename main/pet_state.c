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

static const char *TAG = "pet_state";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static nvs_handle_t s_nvs;
static float s_health = DEFAULT_HEALTH;
static float s_social = DEFAULT_SOCIAL;
static float s_activity_level;
static uint16_t s_nearby_devices;
static uint8_t s_last_saved_health;
static uint8_t s_last_saved_social;

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

    portENTER_CRITICAL(&s_lock);
    health = rounded_score(s_health);
    social = rounded_score(s_social);
    portEXIT_CRITICAL(&s_lock);

    if (health == s_last_saved_health && social == s_last_saved_social) {
        return;
    }

    esp_err_t ret = nvs_set_u8(s_nvs, "health", health);
    if (ret == ESP_OK) {
        ret = nvs_set_u8(s_nvs, "social", social);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(s_nvs);
    }
    if (ret == ESP_OK) {
        s_last_saved_health = health;
        s_last_saved_social = social;
        ESP_LOGI(TAG, "Saved needs: health=%u social=%u", health, social);
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
        s_activity_level *= 0.72f;

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
    if (nvs_get_u8(s_nvs, "health", &stored_health) == ESP_OK) {
        s_health = stored_health;
    }
    if (nvs_get_u8(s_nvs, "social", &stored_social) == ESP_OK) {
        s_social = stored_social;
    }
    s_last_saved_health = rounded_score(s_health);
    s_last_saved_social = rounded_score(s_social);

    if (xTaskCreate(needs_task, "pet_needs", 3072, NULL, 3, NULL) != pdPASS) {
        nvs_close(s_nvs);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Needs ready: health=%u social=%u",
             rounded_score(s_health), rounded_score(s_social));
    return ESP_OK;
}

void pet_state_record_activity(float motion_intensity, float elapsed_seconds)
{
    float active_level = (motion_intensity - 0.30f) / 2.7f;
    if (active_level < 0.0f) {
        active_level = 0.0f;
    } else if (active_level > 1.0f) {
        active_level = 1.0f;
    }

    portENTER_CRITICAL(&s_lock);
    s_activity_level = s_activity_level * 0.96f + active_level * 0.04f;
    s_health = clamp_score(s_health + active_level * 0.04f * elapsed_seconds);
    portEXIT_CRITICAL(&s_lock);
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
    snapshot.activity_level = s_activity_level;
    portEXIT_CRITICAL(&s_lock);
    return snapshot;
}
