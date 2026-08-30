idf_component_register(
    SRCS "main.c" "pet_state.c" "social_scan.c"
    INCLUDE_DIRS "."
    REQUIRES esp32_s3_touch_amoled_1_8 qmi8658 nvs_flash esp_timer bt
)
