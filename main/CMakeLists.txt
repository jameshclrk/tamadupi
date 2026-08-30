idf_component_register(
    SRCS "main.c" "pet_state.c" "social_scan.c" "weather_service.c"
    INCLUDE_DIRS "."
    REQUIRES esp32_s3_touch_amoled_1_8 qmi8658 nvs_flash esp_timer bt
             esp_wifi esp_event esp_netif esp_http_client json mbedtls
)
