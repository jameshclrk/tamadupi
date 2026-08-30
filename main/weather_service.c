#include "weather_service.h"

#include <stddef.h>
#include <string.h>

#include "app_config.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#define WIFI_CONNECTED_BIT BIT0
#define WEATHER_REFRESH_MS (30 * 60 * 1000)
#define WEATHER_RETRY_MS (5 * 60 * 1000)
#define WEATHER_RESPONSE_CAPACITY 2048

typedef struct {
    char data[WEATHER_RESPONSE_CAPACITY];
    size_t length;
    bool overflowed;
} http_response_t;

static const char *TAG = "weather";
static EventGroupHandle_t s_wifi_events;
static portMUX_TYPE s_weather_lock = portMUX_INITIALIZER_UNLOCKED;
static weather_snapshot_t s_weather = {
    .status = WEATHER_STATUS_NEEDS_CONFIG,
};

static void set_connection_state(bool connected)
{
    portENTER_CRITICAL(&s_weather_lock);
    s_weather.wifi_connected = connected;
    if (connected && s_weather.status == WEATHER_STATUS_CONNECTING) {
        s_weather.status = WEATHER_STATUS_UPDATING;
    } else if (!connected && s_weather.status != WEATHER_STATUS_NEEDS_CONFIG) {
        s_weather.status = WEATHER_STATUS_CONNECTING;
    }
    ++s_weather.generation;
    portEXIT_CRITICAL(&s_weather_lock);
}

static void set_weather_status(weather_status_t status)
{
    portENTER_CRITICAL(&s_weather_lock);
    s_weather.status = status;
    ++s_weather.generation;
    portEXIT_CRITICAL(&s_weather_lock);
}

static void publish_weather(float temperature_c, int weather_code, bool is_day)
{
    portENTER_CRITICAL(&s_weather_lock);
    s_weather.status = WEATHER_STATUS_READY;
    s_weather.temperature_c = temperature_c;
    s_weather.weather_code = weather_code;
    s_weather.is_day = is_day;
    ++s_weather.generation;
    portEXIT_CRITICAL(&s_weather_lock);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        set_connection_state(false);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        set_connection_state(true);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi connected");
    }
}

static esp_err_t http_event_handler(esp_http_client_event_t *event)
{
    http_response_t *response = event->user_data;
    if (event->event_id != HTTP_EVENT_ON_DATA || response == NULL || event->data_len <= 0) {
        return ESP_OK;
    }

    const size_t remaining = sizeof(response->data) - 1 - response->length;
    if ((size_t)event->data_len > remaining) {
        response->overflowed = true;
        return ESP_FAIL;
    }
    memcpy(response->data + response->length, event->data, event->data_len);
    response->length += event->data_len;
    response->data[response->length] = '\0';
    return ESP_OK;
}

static esp_err_t fetch_weather(void)
{
    char url[320];
    snprintf(url, sizeof(url),
             "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
             "&current=temperature_2m,weather_code,is_day&timezone=auto",
             (double)TAMADUPI_LATITUDE, (double)TAMADUPI_LONGITUDE);

    http_response_t response = {0};
    const esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event_handler,
        .user_data = &response,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 12000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = esp_http_client_perform(client);
    const int status_code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    if (ret != ESP_OK || response.overflowed || status_code != 200) {
        ESP_LOGW(TAG, "Weather request failed: %s (HTTP %d)",
                 esp_err_to_name(ret), status_code);
        return ret == ESP_OK ? ESP_FAIL : ret;
    }

    cJSON *root = cJSON_Parse(response.data);
    cJSON *current = root == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(root, "current");
    cJSON *temperature = current == NULL ? NULL :
                         cJSON_GetObjectItemCaseSensitive(current, "temperature_2m");
    cJSON *code = current == NULL ? NULL :
                  cJSON_GetObjectItemCaseSensitive(current, "weather_code");
    cJSON *day = current == NULL ? NULL :
                 cJSON_GetObjectItemCaseSensitive(current, "is_day");
    if (!cJSON_IsNumber(temperature) || !cJSON_IsNumber(code) || !cJSON_IsNumber(day)) {
        cJSON_Delete(root);
        ESP_LOGW(TAG, "Weather response did not contain current conditions");
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float temperature_c = (float)temperature->valuedouble;
    const int weather_code = code->valueint;
    const bool is_day = day->valueint != 0;
    ESP_LOGI(TAG, "Weather updated: %.1f C, code %d",
             temperature->valuedouble, code->valueint);
    cJSON_Delete(root);
    publish_weather(temperature_c, weather_code, is_day);
    return ESP_OK;
}

static void weather_task(void *arg)
{
    (void)arg;
    while (true) {
        xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        set_weather_status(WEATHER_STATUS_UPDATING);
        // Give LVGL time to draw the status once, then stay idle while TLS uses
        // the shared internal DMA heap.
        vTaskDelay(pdMS_TO_TICKS(750));
        const esp_err_t ret = fetch_weather();
        if (ret != ESP_OK) {
            set_weather_status(WEATHER_STATUS_ERROR);
        }

        const TickType_t delay_ticks = pdMS_TO_TICKS(ret == ESP_OK ?
                                                     WEATHER_REFRESH_MS : WEATHER_RETRY_MS);
        for (TickType_t elapsed = 0; elapsed < delay_ticks;
             elapsed += pdMS_TO_TICKS(1000)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            if ((xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) == 0) {
                break;
            }
        }
    }
}

esp_err_t weather_service_start(void)
{
    if (!TAMADUPI_CONFIGURED) {
        ESP_LOGW(TAG, "Wi-Fi is not configured; copy secrets.example.h to secrets.h");
        return ESP_OK;
    }

    set_weather_status(WEATHER_STATUS_CONNECTING);
    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    if (esp_netif_create_default_wifi_sta() == NULL) {
        return ESP_FAIL;
    }

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    if ((ret = esp_wifi_init(&init_config)) != ESP_OK) {
        return ret;
    }
    if ((ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                          wifi_event_handler, NULL)) != ESP_OK ||
        (ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                          wifi_event_handler, NULL)) != ESP_OK) {
        return ret;
    }

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, TAMADUPI_WIFI_SSID,
            sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, TAMADUPI_WIFI_PASSWORD,
            sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    if ((ret = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK ||
        (ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config)) != ESP_OK ||
        (ret = esp_wifi_start()) != ESP_OK) {
        return ret;
    }
    if (xTaskCreate(weather_task, "weather", 6144, NULL, 3, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Weather service started");
    return ESP_OK;
}

weather_snapshot_t weather_service_snapshot(void)
{
    weather_snapshot_t snapshot;
    portENTER_CRITICAL(&s_weather_lock);
    snapshot = s_weather;
    portEXIT_CRITICAL(&s_weather_lock);
    return snapshot;
}
