#include "social_scan.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "pet_state.h"

#define MAX_TRACKED_DEVICES 64
#define DEVICE_RETENTION_MS (10 * 60 * 1000)
#define MAINTENANCE_PERIOD_MS 30000
#define MINIMUM_RSSI_DBM (-85)
#define SCAN_INTERVAL_UNITS 0x0100
#define SCAN_WINDOW_UNITS 0x0030

typedef struct {
    bool occupied;
    uint8_t address_type;
    uint8_t address[6];
    int64_t last_seen_ms;
} tracked_device_t;

static const char *TAG = "social_scan";
static portMUX_TYPE s_devices_lock = portMUX_INITIALIZER_UNLOCKED;
static tracked_device_t s_devices[MAX_TRACKED_DEVICES];
static uint16_t s_published_count;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static uint16_t count_active_devices_locked(int64_t time_ms)
{
    uint16_t count = 0;
    for (size_t i = 0; i < MAX_TRACKED_DEVICES; ++i) {
        if (s_devices[i].occupied &&
            time_ms - s_devices[i].last_seen_ms <= DEVICE_RETENTION_MS) {
            ++count;
        }
    }
    return count;
}

static void publish_count_if_changed(uint16_t count)
{
    if (count == s_published_count) {
        return;
    }
    s_published_count = count;
    pet_state_set_nearby_devices(count);
    ESP_LOGI(TAG, "%u nearby BLE device%s", count, count == 1 ? "" : "s");
}

static void remember_device(const ble_addr_t *address)
{
    const int64_t time_ms = now_ms();
    size_t free_slot = MAX_TRACKED_DEVICES;
    size_t oldest_slot = 0;
    int64_t oldest_seen = INT64_MAX;
    uint16_t count;

    portENTER_CRITICAL(&s_devices_lock);
    for (size_t i = 0; i < MAX_TRACKED_DEVICES; ++i) {
        if (s_devices[i].occupied && s_devices[i].address_type == address->type &&
            memcmp(s_devices[i].address, address->val, sizeof(address->val)) == 0) {
            s_devices[i].last_seen_ms = time_ms;
            count = count_active_devices_locked(time_ms);
            portEXIT_CRITICAL(&s_devices_lock);
            publish_count_if_changed(count);
            return;
        }
        if (!s_devices[i].occupied && free_slot == MAX_TRACKED_DEVICES) {
            free_slot = i;
        } else if (s_devices[i].occupied && s_devices[i].last_seen_ms < oldest_seen) {
            oldest_seen = s_devices[i].last_seen_ms;
            oldest_slot = i;
        }
    }

    const size_t slot = free_slot < MAX_TRACKED_DEVICES ? free_slot : oldest_slot;
    s_devices[slot].occupied = true;
    s_devices[slot].address_type = address->type;
    memcpy(s_devices[slot].address, address->val, sizeof(address->val));
    s_devices[slot].last_seen_ms = time_ms;
    count = count_active_devices_locked(time_ms);
    portEXIT_CRITICAL(&s_devices_lock);
    publish_count_if_changed(count);
}

static int gap_event(struct ble_gap_event *event, void *arg);

static void start_scan(void)
{
    uint8_t own_address_type;
    int rc = ble_hs_id_infer_auto(0, &own_address_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Could not determine BLE address type: %d", rc);
        return;
    }

    const struct ble_gap_disc_params params = {
        .itvl = SCAN_INTERVAL_UNITS,
        .window = SCAN_WINDOW_UNITS,
        .filter_policy = 0,
        .limited = 0,
        .passive = 1,
        .filter_duplicates = 0,
    };
    rc = ble_gap_disc(own_address_type, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Could not start BLE discovery: %d", rc);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        if (event->disc.rssi != 127 && event->disc.rssi >= MINIMUM_RSSI_DBM) {
            remember_device(&event->disc.addr);
        }
        break;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        start_scan();
        break;
    default:
        break;
    }
    return 0;
}

static void on_sync(void)
{
    const int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Could not create BLE identity: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE social sensing ready");
    start_scan();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset: %d", reason);
}

static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void maintenance_task(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(MAINTENANCE_PERIOD_MS));
        const int64_t time_ms = now_ms();
        uint16_t count;

        portENTER_CRITICAL(&s_devices_lock);
        for (size_t i = 0; i < MAX_TRACKED_DEVICES; ++i) {
            if (s_devices[i].occupied &&
                time_ms - s_devices[i].last_seen_ms > DEVICE_RETENTION_MS) {
                s_devices[i].occupied = false;
            }
        }
        count = count_active_devices_locked(time_ms);
        portEXIT_CRITICAL(&s_devices_lock);
        publish_count_if_changed(count);
    }
}

esp_err_t social_scan_start(void)
{
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);

    if (xTaskCreate(maintenance_task, "social_age", 2560, NULL, 3, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
