#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include "pet_state.h"
#include "social_scan.h"
#include "step_tracker.h"
#include "weather_service.h"

// The QMI8658 header defines M_PI unconditionally, so avoid colliding with newlib's definition.
#ifdef M_PI
#undef M_PI
#endif
#include "qmi8658.h"

#define IMU_PROBE_TIMEOUT_MS 100
#define IMU_SAMPLE_PERIOD_MS 40
#define INITIAL_WEATHER_WAIT_MS 15000
#define QMI8658_RESET_REGISTER 0x60
#define QMI8658_RESET_COMMAND 0xB0
#define QMI8658_CTRL1_VALUE 0x60
#define QMI8658_RESET_DELAY_MS 20

#define GRAVITY_MPS2 9.80665f
#define SHAKE_ACCEL_THRESHOLD 3.5f
#define SHAKE_GYRO_THRESHOLD_DPS 170.0f
#define SHAKE_COOLDOWN_MS 650
#define CELEBRATION_DURATION_MS 900
#define UI_ACTIVE_PERIOD_MS 66
#define UI_IDLE_PERIOD_MS 200
#define LVGL_TASK_CORE 1
#define BUDDY_PI 3.14159265358979323846f

#define BUDDY_BODY_X 30
#define BUDDY_BODY_Y 50
#define BUDDY_BODY_WIDTH 170
#define BUDDY_BODY_HEIGHT 170

#define COLOR_BG 0x10142D
#define COLOR_PANEL 0x1B2142
#define COLOR_PANEL_EDGE 0x303A68
#define COLOR_TEXT 0xF8F5FF
#define COLOR_MUTED 0xAEB6D9
#define COLOR_MINT 0x67E4C2
#define COLOR_PEACH 0xFFB36B
#define COLOR_PEACH_LIGHT 0xFFD092
#define COLOR_PEACH_DARK 0xF28B62
#define COLOR_CHEEK 0xFF7894
#define COLOR_INK 0x302743
#define COLOR_STAR 0xFFE07A
#define COLOR_RAIN 0x78BCEC
#define COLOR_CLOUD 0x8290B8
#define COLOR_SNOW 0xE8F5FF
#define COLOR_CREAM 0xFFF3D6
#define COLOR_LILAC 0xC9B8FF
#define COLOR_DAY_SKY 0x65BCE9
#define COLOR_DAY_HORIZON 0xC7EEF0
#define COLOR_DAY_HILL 0x72B77D
#define COLOR_DAY_GRASS 0x4F9D57

static const char *TAG = "tamadupi";

typedef struct {
    bool sensor_online;
    float roll_deg;
    uint32_t shake_generation;
} motion_state_t;

typedef struct {
    lv_obj_t *character;
    lv_obj_t *body;
    lv_obj_t *left_eye;
    lv_obj_t *right_eye;
    lv_obj_t *left_pupil;
    lv_obj_t *right_pupil;
    lv_obj_t *left_brow;
    lv_obj_t *right_brow;
    lv_obj_t *mouth;
    lv_obj_t *belly_glow;
    lv_obj_t *left_cheek;
    lv_obj_t *right_cheek;
    lv_obj_t *left_ear;
    lv_obj_t *right_ear;
    lv_obj_t *left_arm;
    lv_obj_t *right_arm;
    lv_obj_t *left_foot;
    lv_obj_t *right_foot;
    lv_obj_t *status_dot;
    lv_obj_t *score_bar;
    lv_obj_t *score_icon;
    lv_obj_t *score_pips[4];
    lv_obj_t *score_label;
    lv_obj_t *multiplier_badge;
    lv_obj_t *multiplier_label;
    lv_obj_t *weather_clouds[3];
    lv_obj_t *weather_particles[7];
    lv_obj_t *scene_horizon;
    lv_obj_t *scene_hills[2];
    lv_obj_t *scene_grass;
    lv_obj_t *scene_sun_glow;
    lv_obj_t *scene_sun;
    lv_obj_t *scene_moon;
    lv_obj_t *scene_moon_cutout;
    lv_obj_t *scene_stars[9];
    lv_obj_t *scene_fireflies[5];
    lv_obj_t *sparkles[4];
    lv_timer_t *update_timer;
    uint32_t seen_shake_generation;
    int64_t surprised_until_ms;
    bool status_initialized;
    bool displayed_sensor_online;
    uint32_t displayed_weather_generation;
    int displayed_weather_kind;
    int displayed_ambience_kind;
    int displayed_expression;
    bool displayed_blink;
    int32_t displayed_gaze_x;
    int32_t displayed_character_x;
    int32_t displayed_character_y;
    int32_t displayed_health;
    int32_t displayed_social;
    int32_t displayed_score;
    int32_t displayed_multiplier;
    int32_t displayed_nearby_devices;
    int32_t displayed_activity_highlight;
    int32_t displayed_score_pip;
    int16_t displayed_firefly_opa[5];
    int64_t displayed_scene_minute;
    int displayed_scene_weather_kind;
    int64_t celebration_started_ms;
    int64_t celebration_until_ms;
} buddy_ui_t;

typedef enum {
    BUDDY_EXPRESSION_IDLE,
    BUDDY_EXPRESSION_TILT,
    BUDDY_EXPRESSION_SURPRISED,
    BUDDY_EXPRESSION_HAPPY,
    BUDDY_EXPRESSION_TIRED,
    BUDDY_EXPRESSION_LONELY,
} buddy_expression_t;

typedef enum {
    WEATHER_KIND_NONE,
    WEATHER_KIND_CLEAR,
    WEATHER_KIND_CLOUDY,
    WEATHER_KIND_RAIN,
    WEATHER_KIND_SNOW,
    WEATHER_KIND_STORM,
} weather_kind_t;

typedef struct {
    uint32_t sky;
    uint32_t horizon;
    uint32_t hill;
    uint32_t grass;
    float stars;
    float fireflies;
    float daylight;
} scene_palette_t;

static portMUX_TYPE s_motion_lock = portMUX_INITIALIZER_UNLOCKED;
static motion_state_t s_motion = {0};
static buddy_ui_t s_ui = {0};

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static void start_celebration(int64_t time_ms)
{
    s_ui.celebration_started_ms = time_ms;
    s_ui.celebration_until_ms = time_ms + CELEBRATION_DURATION_MS;
    if (s_ui.update_timer != NULL) {
        lv_timer_set_period(s_ui.update_timer, UI_ACTIVE_PERIOD_MS);
        lv_timer_ready(s_ui.update_timer);
    }
}

static void tap_event_cb(lv_event_t *event)
{
    (void)event;
    start_celebration(now_ms());
}

static lv_obj_t *active_screen(void)
{
#if LVGL_VERSION_MAJOR >= 9
    return lv_screen_active();
#else
    return lv_scr_act();
#endif
}

static lv_obj_t *make_shape(lv_obj_t *parent, int32_t x, int32_t y,
                            int32_t width, int32_t height,
                            uint32_t color, int32_t radius)
{
    lv_obj_t *shape = lv_obj_create(parent);
    lv_obj_set_pos(shape, x, y);
    lv_obj_set_size(shape, width, height);
    lv_obj_set_style_bg_color(shape, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(shape, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(shape, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(shape, radius, LV_PART_MAIN);
    lv_obj_set_style_pad_all(shape, 0, LV_PART_MAIN);
    lv_obj_clear_flag(shape, LV_OBJ_FLAG_SCROLLABLE);
    return shape;
}

static lv_obj_t *make_sparkle(lv_obj_t *parent, int32_t x, int32_t y,
                              uint32_t color)
{
    lv_obj_t *sparkle = make_shape(parent, x, y, 18, 18, color, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(sparkle, LV_OPA_TRANSP, LV_PART_MAIN);
    make_shape(sparkle, 7, 0, 4, 18, color, 2);
    make_shape(sparkle, 0, 7, 18, 4, color, 2);
    return sparkle;
}

static lv_obj_t *make_heart(lv_obj_t *parent, int32_t x, int32_t y,
                            uint32_t color)
{
    lv_obj_t *heart = make_shape(parent, x, y, 26, 25, color, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(heart, LV_OPA_TRANSP, LV_PART_MAIN);
    make_shape(heart, 1, 1, 13, 13, color, LV_RADIUS_CIRCLE);
    make_shape(heart, 12, 1, 13, 13, color, LV_RADIUS_CIRCLE);
    make_shape(heart, 6, 8, 14, 16, color, 7);
    return heart;
}

static uint32_t blend_color(uint32_t from, uint32_t to, float amount)
{
    amount = clampf(amount, 0.0f, 1.0f);
    const uint32_t from_r = (from >> 16) & 0xff;
    const uint32_t from_g = (from >> 8) & 0xff;
    const uint32_t from_b = from & 0xff;
    const uint32_t to_r = (to >> 16) & 0xff;
    const uint32_t to_g = (to >> 8) & 0xff;
    const uint32_t to_b = to & 0xff;
    const uint32_t r = (uint32_t)((float)from_r + ((float)to_r - (float)from_r) * amount);
    const uint32_t g = (uint32_t)((float)from_g + ((float)to_g - (float)from_g) * amount);
    const uint32_t b = (uint32_t)((float)from_b + ((float)to_b - (float)from_b) * amount);
    return (r << 16) | (g << 8) | b;
}

static scene_palette_t blend_palette(scene_palette_t from, scene_palette_t to,
                                     float amount)
{
    return (scene_palette_t) {
        .sky = blend_color(from.sky, to.sky, amount),
        .horizon = blend_color(from.horizon, to.horizon, amount),
        .hill = blend_color(from.hill, to.hill, amount),
        .grass = blend_color(from.grass, to.grass, amount),
        .stars = from.stars + (to.stars - from.stars) * amount,
        .fireflies = from.fireflies + (to.fireflies - from.fireflies) * amount,
        .daylight = from.daylight + (to.daylight - from.daylight) * amount,
    };
}

static scene_palette_t scene_palette_for(const weather_snapshot_t *weather,
                                         int64_t time_ms, int64_t *current_unix)
{
    static const scene_palette_t night = {
        0x111936, 0x26345C, 0x294A55, 0x21492F, 1.0f, 0.85f, 0.0f,
    };
    static const scene_palette_t dawn = {
        0x8C729F, 0xFFB184, 0x71827A, 0x3E7146, 0.25f, 0.25f, 0.45f,
    };
    static const scene_palette_t day = {
        COLOR_DAY_SKY, COLOR_DAY_HORIZON, COLOR_DAY_HILL, COLOR_DAY_GRASS,
        0.0f, 0.0f, 1.0f,
    };
    static const scene_palette_t sunset = {
        0x785D98, 0xFF8B63, 0x77735F, 0x456B3E, 0.15f, 0.45f, 0.42f,
    };

    const bool solar_time_ready = weather->status == WEATHER_STATUS_READY &&
                                  weather->current_time_unix > 0 &&
                                  weather->sunrise_unix > 0 &&
                                  weather->sunset_unix > weather->sunrise_unix;
    if (!solar_time_ready) {
        *current_unix = 0;
        return weather->is_day ? day : night;
    }

    const int64_t elapsed_seconds = (time_ms - weather->observed_at_ms) / 1000;
    const int64_t current = weather->current_time_unix + elapsed_seconds;
    const int64_t sunrise = weather->sunrise_unix;
    const int64_t sunset_time = weather->sunset_unix;
    *current_unix = current;

    if (current < sunrise - 3600 || current >= sunset_time + 2700) {
        return night;
    }
    if (current < sunrise) {
        return blend_palette(night, dawn,
                             (float)(current - (sunrise - 3600)) / 3600.0f);
    }
    if (current < sunrise + 2700) {
        return blend_palette(dawn, day, (float)(current - sunrise) / 2700.0f);
    }
    if (current < sunset_time - 3600) {
        return day;
    }
    if (current < sunset_time) {
        return blend_palette(day, sunset,
                             (float)(current - (sunset_time - 3600)) / 3600.0f);
    }
    return blend_palette(sunset, night,
                         (float)(current - sunset_time) / 2700.0f);
}

static void create_time_scene(lv_obj_t *screen)
{
    s_ui.scene_horizon = make_shape(screen, -35, 124, 438, 240,
                                    COLOR_DAY_HORIZON, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.scene_horizon, LV_OPA_50, LV_PART_MAIN);

    s_ui.scene_sun_glow = make_shape(screen, 270, 26, 78, 78,
                                     COLOR_STAR, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.scene_sun_glow, LV_OPA_30, LV_PART_MAIN);
    s_ui.scene_sun = make_shape(screen, 283, 39, 52, 52,
                                COLOR_STAR, LV_RADIUS_CIRCLE);

    const int16_t star_positions[9][2] = {
        {27, 55}, {83, 27}, {145, 62}, {211, 28}, {330, 104},
        {52, 154}, {184, 129}, {282, 153}, {344, 211},
    };
    for (size_t i = 0; i < 9; ++i) {
        const int32_t size = (i % 3) == 0 ? 6 : 4;
        s_ui.scene_stars[i] = make_shape(screen, star_positions[i][0],
                                         star_positions[i][1], size, size,
                                         COLOR_CREAM, LV_RADIUS_CIRCLE);
    }

    s_ui.scene_moon = make_shape(screen, 286, 35, 49, 49,
                                 COLOR_CREAM, LV_RADIUS_CIRCLE);
    s_ui.scene_moon_cutout = make_shape(s_ui.scene_moon, 14, -4, 42, 42,
                                        COLOR_BG, LV_RADIUS_CIRCLE);

    s_ui.scene_hills[0] = make_shape(screen, -68, 240, 270, 132,
                                     COLOR_DAY_HILL, LV_RADIUS_CIRCLE);
    s_ui.scene_hills[1] = make_shape(screen, 144, 236, 294, 144,
                                     COLOR_DAY_HILL, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.scene_hills[1], LV_OPA_80, LV_PART_MAIN);
    s_ui.scene_grass = make_shape(screen, 0, 286, 368, 94,
                                  COLOR_DAY_GRASS, 38);

    const int16_t tuft_x[14] = {8, 31, 57, 82, 112, 137, 166,
                                 196, 225, 251, 278, 302, 328, 351};
    for (size_t i = 0; i < 14; ++i) {
        lv_obj_t *blade = make_shape(s_ui.scene_grass, tuft_x[i],
                                     2 + (int32_t)(i % 3) * 4,
                                     5, 18 - (int32_t)(i % 3) * 3,
                                     0x8DD16F, 3);
        lv_obj_set_style_transform_rotation(blade, (i % 2) == 0 ? -120 : 120,
                                            LV_PART_MAIN);
    }

    const int16_t flower_positions[4][2] = {{24, 30}, {70, 49}, {294, 43}, {337, 23}};
    const uint32_t flower_colors[4] = {COLOR_CREAM, COLOR_CHEEK, COLOR_LILAC, COLOR_STAR};
    for (size_t i = 0; i < 4; ++i) {
        lv_obj_t *flower = make_shape(s_ui.scene_grass,
                                      flower_positions[i][0], flower_positions[i][1],
                                      18, 18, flower_colors[i], LV_RADIUS_CIRCLE);
        lv_obj_set_style_bg_opa(flower, LV_OPA_TRANSP, LV_PART_MAIN);
        make_shape(flower, 0, 5, 8, 8, flower_colors[i], LV_RADIUS_CIRCLE);
        make_shape(flower, 10, 5, 8, 8, flower_colors[i], LV_RADIUS_CIRCLE);
        make_shape(flower, 5, 0, 8, 8, flower_colors[i], LV_RADIUS_CIRCLE);
        make_shape(flower, 5, 10, 8, 8, flower_colors[i], LV_RADIUS_CIRCLE);
        make_shape(flower, 6, 6, 6, 6, COLOR_STAR, LV_RADIUS_CIRCLE);
    }

    const int16_t firefly_positions[5][2] = {
        {43, 250}, {102, 274}, {241, 260}, {310, 282}, {344, 238},
    };
    for (size_t i = 0; i < 5; ++i) {
        s_ui.scene_fireflies[i] = make_shape(screen, firefly_positions[i][0],
                                             firefly_positions[i][1], 7, 7,
                                             COLOR_STAR, LV_RADIUS_CIRCLE);
    }
}

static void make_decorations(lv_obj_t *screen)
{
    const int16_t positions[4][2] = {{42, 124}, {306, 148}, {43, 282}, {305, 291}};

    for (size_t i = 0; i < 4; ++i) {
        lv_obj_t *sparkle = make_sparkle(screen, positions[i][0], positions[i][1],
                                         COLOR_STAR);
        lv_obj_set_style_opa(sparkle, LV_OPA_30, LV_PART_MAIN);
        s_ui.sparkles[i] = sparkle;
    }
}

static void create_weather_ambience(lv_obj_t *screen)
{
    s_ui.weather_clouds[0] = make_shape(screen, 250, 44, 88, 29,
                                        COLOR_CLOUD, LV_RADIUS_CIRCLE);
    s_ui.weather_clouds[1] = make_shape(screen, 266, 31, 39, 37,
                                        COLOR_CLOUD, LV_RADIUS_CIRCLE);
    s_ui.weather_clouds[2] = make_shape(screen, 296, 35, 35, 34,
                                        COLOR_CLOUD, LV_RADIUS_CIRCLE);
    for (size_t i = 0; i < 3; ++i) {
        lv_obj_set_style_bg_opa(s_ui.weather_clouds[i], LV_OPA_70, LV_PART_MAIN);
        lv_obj_add_flag(s_ui.weather_clouds[i], LV_OBJ_FLAG_HIDDEN);
    }

    const int16_t particle_x[7] = {35, 83, 132, 184, 237, 285, 330};
    for (size_t i = 0; i < 7; ++i) {
        s_ui.weather_particles[i] = make_shape(screen, particle_x[i], 88,
                                                4, 16, COLOR_RAIN, 2);
        lv_obj_add_flag(s_ui.weather_particles[i], LV_OBJ_FLAG_HIDDEN);
    }
}

static weather_kind_t classify_weather(const weather_snapshot_t *weather)
{
    if (weather->status != WEATHER_STATUS_READY) {
        return WEATHER_KIND_NONE;
    }
    const int code = weather->weather_code;
    if (code == 0) {
        return WEATHER_KIND_CLEAR;
    }
    if (code <= 48) {
        return WEATHER_KIND_CLOUDY;
    }
    if ((code >= 71 && code <= 77) || (code >= 85 && code <= 86)) {
        return WEATHER_KIND_SNOW;
    }
    if (code >= 95) {
        return WEATHER_KIND_STORM;
    }
    return WEATHER_KIND_RAIN;
}

static void create_header(lv_obj_t *screen)
{
    lv_obj_t *halo = make_shape(screen, 22, 22, 28, 28, COLOR_PANEL, LV_RADIUS_CIRCLE);
    lv_obj_set_style_border_width(halo, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(halo, lv_color_hex(COLOR_PANEL_EDGE), LV_PART_MAIN);
    s_ui.status_dot = make_shape(halo, 8, 8, 12, 12, COLOR_MUTED, LV_RADIUS_CIRCLE);
}

static void create_character(lv_obj_t *screen)
{
    lv_obj_t *shadow = make_shape(screen, 96, 319, 176, 26, 0x090B1D, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(shadow, LV_OPA_50, LV_PART_MAIN);

    s_ui.character = lv_obj_create(screen);
    lv_obj_set_pos(s_ui.character, 69, 83);
    lv_obj_set_size(s_ui.character, 230, 230);
    lv_obj_set_style_bg_opa(s_ui.character, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_ui.character, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_ui.character, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_ui.character, LV_OBJ_FLAG_SCROLLABLE);
    make_shape(s_ui.character, 184, 132, 35, 35, COLOR_PEACH_LIGHT, LV_RADIUS_CIRCLE);
    s_ui.left_ear = make_shape(s_ui.character, 42, 15, 66, 82,
                               COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    s_ui.right_ear = make_shape(s_ui.character, 122, 15, 66, 82,
                                COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.left_ear, 15, 15, 36, 53, COLOR_CHEEK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.right_ear, 15, 15, 36, 53, COLOR_CHEEK, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(lv_obj_get_child(s_ui.left_ear, 0), LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_obj_get_child(s_ui.right_ear, 0), LV_OPA_50, LV_PART_MAIN);

    s_ui.left_arm = make_shape(s_ui.character, 13, 131, 61, 34,
                               COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    s_ui.right_arm = make_shape(s_ui.character, 156, 131, 61, 34,
                                COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    s_ui.left_foot = make_shape(s_ui.character, 43, 193, 59, 34,
                                COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    s_ui.right_foot = make_shape(s_ui.character, 128, 193, 59, 34,
                                 COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.left_foot, 13, 6, 11, 11, COLOR_PEACH_LIGHT, LV_RADIUS_CIRCLE);
    make_shape(s_ui.left_foot, 31, 5, 11, 11, COLOR_PEACH_LIGHT, LV_RADIUS_CIRCLE);
    make_shape(s_ui.right_foot, 13, 5, 11, 11, COLOR_PEACH_LIGHT, LV_RADIUS_CIRCLE);
    make_shape(s_ui.right_foot, 31, 6, 11, 11, COLOR_PEACH_LIGHT, LV_RADIUS_CIRCLE);

    s_ui.body = make_shape(s_ui.character, BUDDY_BODY_X, BUDDY_BODY_Y,
                           BUDDY_BODY_WIDTH, BUDDY_BODY_HEIGHT, COLOR_PEACH, 82);
    lv_obj_set_style_border_width(s_ui.body, 3, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_ui.body, lv_color_hex(COLOR_CREAM), LV_PART_MAIN);

    s_ui.belly_glow = make_shape(s_ui.body, 53, 137, 64, 28,
                                 COLOR_CREAM, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.belly_glow, LV_OPA_30, LV_PART_MAIN);
    lv_obj_t *belly_heart = make_heart(s_ui.belly_glow, 19, 2, COLOR_CHEEK);
    lv_obj_set_style_opa(belly_heart, LV_OPA_70, LV_PART_MAIN);

    s_ui.left_brow = make_shape(s_ui.body, 35, 51, 29, 5, COLOR_PEACH_DARK, 3);
    s_ui.right_brow = make_shape(s_ui.body, 106, 51, 29, 5, COLOR_PEACH_DARK, 3);
    s_ui.left_eye = make_shape(s_ui.body, 29, 60, 42, 45,
                               COLOR_CREAM, LV_RADIUS_CIRCLE);
    s_ui.right_eye = make_shape(s_ui.body, 99, 60, 42, 45,
                                COLOR_CREAM, LV_RADIUS_CIRCLE);
    s_ui.left_pupil = make_shape(s_ui.left_eye, 12, 11, 18, 23,
                                 COLOR_INK, LV_RADIUS_CIRCLE);
    s_ui.right_pupil = make_shape(s_ui.right_eye, 12, 11, 18, 23,
                                  COLOR_INK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.left_pupil, 4, 4, 6, 8, COLOR_TEXT, LV_RADIUS_CIRCLE);
    make_shape(s_ui.right_pupil, 4, 4, 6, 8, COLOR_TEXT, LV_RADIUS_CIRCLE);

    s_ui.left_cheek = make_shape(s_ui.body, 13, 111, 37, 17,
                                 COLOR_CHEEK, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.left_cheek, LV_OPA_70, LV_PART_MAIN);
    s_ui.right_cheek = make_shape(s_ui.body, 120, 111, 37, 17,
                                  COLOR_CHEEK, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.right_cheek, LV_OPA_70, LV_PART_MAIN);

    s_ui.mouth = make_shape(s_ui.body, 70, 111, 30, 18, COLOR_INK, 9);
    make_shape(s_ui.mouth, 7, 7, 16, 9, COLOR_CHEEK, LV_RADIUS_CIRCLE);
}

static void create_footer(lv_obj_t *screen)
{
    lv_obj_t *card = make_shape(screen, 20, 370, 328, 58, COLOR_PANEL, 29);
    lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, lv_color_hex(COLOR_PANEL_EDGE), LV_PART_MAIN);

    s_ui.score_icon = make_heart(card, 12, 16, COLOR_MINT);
    make_shape(s_ui.score_icon, 9, 11, 8, 11, COLOR_STAR, LV_RADIUS_CIRCLE);
    make_shape(s_ui.score_icon, 6, 8, 5, 5, COLOR_STAR, LV_RADIUS_CIRCLE);
    make_shape(s_ui.score_icon, 16, 7, 5, 5, COLOR_STAR, LV_RADIUS_CIRCLE);
    lv_obj_t *score_track = make_shape(card, 47, 21, 170, 16, 0x30385F, 8);
    s_ui.score_bar = make_shape(score_track, 0, 0, 1, 16, COLOR_MINT, 8);

    for (size_t i = 0; i < 4; ++i) {
        s_ui.score_pips[i] = make_shape(score_track, 31 + (int32_t)i * 33, 5,
                                       6, 6, COLOR_CREAM, LV_RADIUS_CIRCLE);
        lv_obj_set_style_opa(s_ui.score_pips[i], LV_OPA_30, LV_PART_MAIN);
    }

    lv_obj_t *score_badge = make_shape(card, 222, 14, 38, 30, 0x30385F, 15);
    s_ui.score_label = lv_label_create(score_badge);
    lv_label_set_text(s_ui.score_label, "0");
    lv_obj_set_style_text_color(s_ui.score_label, lv_color_hex(COLOR_CREAM),
                                LV_PART_MAIN);
    lv_obj_center(s_ui.score_label);

    s_ui.multiplier_badge = make_shape(card, 267, 12, 47, 34,
                                       COLOR_PANEL_EDGE, 17);
    s_ui.multiplier_label = lv_label_create(s_ui.multiplier_badge);
    lv_label_set_text(s_ui.multiplier_label, "1x");
    lv_obj_set_style_text_color(s_ui.multiplier_label, lv_color_hex(COLOR_CREAM),
                                LV_PART_MAIN);
    lv_obj_center(s_ui.multiplier_label);
}

static motion_state_t motion_snapshot(void)
{
    motion_state_t snapshot;
    portENTER_CRITICAL(&s_motion_lock);
    snapshot = s_motion;
    portEXIT_CRITICAL(&s_motion_lock);
    return snapshot;
}

static void set_eye_expression(bool blink, buddy_expression_t expression, int32_t gaze_x)
{
    const bool surprised = expression == BUDDY_EXPRESSION_SURPRISED;
    const bool happy = expression == BUDDY_EXPRESSION_HAPPY;
    const bool tired = expression == BUDDY_EXPRESSION_TIRED;
    const int32_t eye_height = surprised ? 49 : (blink ? 5 : (happy ? 18 : (tired ? 23 : 45)));
    const int32_t eye_y = surprised ? 56 : (blink ? 80 : (happy ? 72 : (tired ? 70 : 60)));

    lv_obj_set_y(s_ui.left_eye, eye_y);
    lv_obj_set_y(s_ui.right_eye, eye_y);
    lv_obj_set_height(s_ui.left_eye, eye_height);
    lv_obj_set_height(s_ui.right_eye, eye_height);
    lv_obj_set_y(s_ui.left_brow, surprised ? 43 : (happy ? 49 : (tired ? 62 : 51)));
    lv_obj_set_y(s_ui.right_brow, surprised ? 43 : (happy ? 49 : (tired ? 62 : 51)));

    const lv_opa_t pupil_opa = blink || happy ? LV_OPA_TRANSP : LV_OPA_COVER;
    lv_obj_set_style_opa(s_ui.left_pupil, pupil_opa, LV_PART_MAIN);
    lv_obj_set_style_opa(s_ui.right_pupil, pupil_opa, LV_PART_MAIN);
    lv_obj_set_x(s_ui.left_pupil, 12 + gaze_x);
    lv_obj_set_x(s_ui.right_pupil, 12 + gaze_x);
}

static void update_time_scene(const weather_snapshot_t *weather,
                              weather_kind_t weather_kind, int64_t time_ms)
{
    int64_t current_unix = 0;
    const scene_palette_t palette = scene_palette_for(weather, time_ms, &current_unix);
    const int64_t scene_minute = current_unix > 0 ? current_unix / 60 : time_ms / 60000;

    if (scene_minute != s_ui.displayed_scene_minute ||
        weather_kind != s_ui.displayed_scene_weather_kind) {
        lv_obj_set_style_bg_color(active_screen(), lv_color_hex(palette.sky), LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_ui.scene_horizon, lv_color_hex(palette.horizon),
                                  LV_PART_MAIN);
        for (size_t i = 0; i < 2; ++i) {
            lv_obj_set_style_bg_color(s_ui.scene_hills[i], lv_color_hex(palette.hill),
                                      LV_PART_MAIN);
        }
        lv_obj_set_style_bg_color(s_ui.scene_grass, lv_color_hex(palette.grass),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_ui.scene_moon_cutout, lv_color_hex(palette.sky),
                                  LV_PART_MAIN);

        const bool solar_time_ready = current_unix > 0 &&
                                      weather->sunset_unix > weather->sunrise_unix;
        bool sun_up = weather->is_day;
        float sun_progress = 0.55f;
        if (solar_time_ready) {
            sun_up = current_unix >= weather->sunrise_unix - 900 &&
                     current_unix <= weather->sunset_unix + 900;
            sun_progress = clampf((float)(current_unix - weather->sunrise_unix) /
                                  (float)(weather->sunset_unix - weather->sunrise_unix),
                                  0.0f, 1.0f);
        }
        const int32_t sun_x = 8 + (int32_t)(sun_progress * 300.0f);
        const int32_t sun_y = 72 - (int32_t)(sinf(sun_progress * BUDDY_PI) * 62.0f);
        lv_obj_set_pos(s_ui.scene_sun_glow, sun_x - 13, sun_y - 13);
        lv_obj_set_pos(s_ui.scene_sun, sun_x, sun_y);

        float weather_dimming = 1.0f;
        if (weather_kind == WEATHER_KIND_CLOUDY) {
            weather_dimming = 0.55f;
        } else if (weather_kind == WEATHER_KIND_RAIN || weather_kind == WEATHER_KIND_SNOW) {
            weather_dimming = 0.30f;
        } else if (weather_kind == WEATHER_KIND_STORM) {
            weather_dimming = 0.12f;
        }
        const lv_opa_t sun_opa = sun_up ?
            (lv_opa_t)(255.0f * palette.daylight * weather_dimming) : LV_OPA_TRANSP;
        lv_obj_set_style_opa(s_ui.scene_sun, sun_opa, LV_PART_MAIN);
        lv_obj_set_style_opa(s_ui.scene_sun_glow, sun_opa / 3, LV_PART_MAIN);

        const lv_opa_t moon_opa = sun_up ? LV_OPA_TRANSP :
                                  (lv_opa_t)(255.0f * (1.0f - palette.daylight));
        lv_obj_set_style_opa(s_ui.scene_moon, moon_opa, LV_PART_MAIN);
        for (size_t i = 0; i < 9; ++i) {
            const float varied = palette.stars * (0.58f + (float)(i % 3) * 0.18f);
            lv_obj_set_style_opa(s_ui.scene_stars[i], (lv_opa_t)(255.0f * varied),
                                 LV_PART_MAIN);
        }
        s_ui.displayed_scene_minute = scene_minute;
        s_ui.displayed_scene_weather_kind = weather_kind;
    }

    for (size_t i = 0; i < 5; ++i) {
        const float pulse = 0.48f + 0.52f *
                            sinf((float)time_ms * 0.003f + (float)i * 1.7f);
        const float opacity = clampf(palette.fireflies * pulse, 0.0f, 1.0f);
        const lv_opa_t firefly_opa = (lv_opa_t)(255.0f * opacity);
        if (firefly_opa != s_ui.displayed_firefly_opa[i]) {
            lv_obj_set_style_opa(s_ui.scene_fireflies[i], firefly_opa, LV_PART_MAIN);
            s_ui.displayed_firefly_opa[i] = firefly_opa;
        }
    }
}

static void update_weather_ambience(weather_kind_t kind, int64_t time_ms)
{
    const bool show_cloud = kind == WEATHER_KIND_CLOUDY || kind == WEATHER_KIND_RAIN ||
                            kind == WEATHER_KIND_SNOW || kind == WEATHER_KIND_STORM;
    const bool show_particles = kind == WEATHER_KIND_RAIN || kind == WEATHER_KIND_SNOW ||
                                kind == WEATHER_KIND_STORM;
    const bool ambience_changed = kind != s_ui.displayed_ambience_kind;

    if (ambience_changed) {
        for (size_t i = 0; i < 3; ++i) {
            if (show_cloud) {
                lv_obj_clear_flag(s_ui.weather_clouds[i], LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(s_ui.weather_clouds[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    for (size_t i = 0; i < 7; ++i) {
        lv_obj_t *particle = s_ui.weather_particles[i];
        if (!show_particles) {
            lv_obj_add_flag(particle, LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(particle, LV_OBJ_FLAG_HIDDEN);
        const int32_t y = 105 + (int32_t)((time_ms / (kind == WEATHER_KIND_SNOW ? 15 : 7) +
                                           (int64_t)i * 47) % 205);
        lv_obj_set_y(particle, y);
        if (ambience_changed) {
            if (kind == WEATHER_KIND_SNOW) {
                lv_obj_set_size(particle, 7, 7);
                lv_obj_set_style_radius(particle, LV_RADIUS_CIRCLE, LV_PART_MAIN);
                lv_obj_set_style_bg_color(particle, lv_color_hex(COLOR_SNOW), LV_PART_MAIN);
            } else {
                lv_obj_set_size(particle, kind == WEATHER_KIND_STORM ? 5 : 4,
                                kind == WEATHER_KIND_STORM ? 20 : 16);
                lv_obj_set_style_radius(particle, 2, LV_PART_MAIN);
                lv_obj_set_style_bg_color(particle, lv_color_hex(COLOR_RAIN), LV_PART_MAIN);
            }
        }
    }
    s_ui.displayed_ambience_kind = kind;
}

static uint32_t buddy_body_color(buddy_expression_t expression, weather_kind_t weather)
{
    if (expression == BUDDY_EXPRESSION_TIRED) {
        return 0xD89A78;
    }
    if (weather == WEATHER_KIND_RAIN || weather == WEATHER_KIND_STORM) {
        return 0xE9A27E;
    }
    if (weather == WEATHER_KIND_SNOW) {
        return COLOR_PEACH_LIGHT;
    }
    return expression == BUDDY_EXPRESSION_SURPRISED ? COLOR_PEACH_LIGHT : COLOR_PEACH;
}

static void buddy_update(lv_timer_t *timer)
{
    (void)timer;
    const motion_state_t motion = motion_snapshot();
    const pet_state_snapshot_t needs = pet_state_snapshot();
    const weather_snapshot_t weather = weather_service_snapshot();
    const weather_kind_t weather_kind = classify_weather(&weather);
    const int64_t time_ms = now_ms();
    const uint8_t score = needs.health;

    if (weather.status == WEATHER_STATUS_UPDATING) {
        if (!s_ui.status_initialized ||
            weather.generation != s_ui.displayed_weather_generation) {
            lv_obj_set_style_bg_color(s_ui.status_dot, lv_color_hex(COLOR_RAIN), LV_PART_MAIN);
            s_ui.displayed_sensor_online = motion.sensor_online;
            s_ui.displayed_weather_generation = weather.generation;
            s_ui.status_initialized = true;
        }
        lv_timer_set_period(timer, UI_IDLE_PERIOD_MS);
        return;
    }

    if (motion.shake_generation != s_ui.seen_shake_generation) {
        s_ui.seen_shake_generation = motion.shake_generation;
        s_ui.surprised_until_ms = time_ms + 850;
    }

    if (s_ui.displayed_score != INT32_MIN && score > s_ui.displayed_score) {
        start_celebration(time_ms);
    }

    const bool surprised = time_ms < s_ui.surprised_until_ms;
    const bool celebrating = time_ms < s_ui.celebration_until_ms;
    const bool blink = !surprised && ((time_ms % 4300) > 4130);
    const float roll = clampf(motion.roll_deg, -28.0f, 28.0f);
    const float phase = (float)(time_ms % 1800) / 1800.0f;
    const int32_t weather_bounce = weather_kind == WEATHER_KIND_CLEAR ?
                                   (int32_t)(sinf(phase * 4.0f * BUDDY_PI) * 1.5f) : 0;
    const int32_t breathe = (int32_t)(sinf(phase * 2.0f * BUDDY_PI) * 2.0f) +
                            weather_bounce;
    const float dance_wave = sinf((float)time_ms * 0.014f);
    const int32_t dance_x = needs.walking ? (int32_t)(dance_wave * 6.0f) : 0;
    const int32_t dance_jump = needs.walking ?
                               (int32_t)(fabsf(dance_wave) * 9.0f) : 0;
    float celebration_progress = 0.0f;
    if (celebrating) {
        celebration_progress = clampf((float)(time_ms - s_ui.celebration_started_ms) /
                                      (float)CELEBRATION_DURATION_MS, 0.0f, 1.0f);
    }
    const float celebration_wave = sinf(celebration_progress * 4.0f * BUDDY_PI);
    const int32_t celebration_x = celebrating ? (int32_t)(celebration_wave * 9.0f) : 0;
    const int32_t celebration_jump = celebrating ?
        (int32_t)(sinf(celebration_progress * BUDDY_PI) * 22.0f +
                  fabsf(celebration_wave) * 7.0f) : 0;
    const int32_t character_x = 69 + dance_x + celebration_x;
    const int32_t character_y = 83 - dance_jump - celebration_jump;
    const int32_t gaze_x = (int32_t)(roll / 7.0f);
    const buddy_expression_t expression = surprised ? BUDDY_EXPRESSION_SURPRISED :
        (celebrating || needs.walking || score >= 85 ? BUDDY_EXPRESSION_HAPPY :
         (fabsf(roll) > 11.0f ? BUDDY_EXPRESSION_TILT :
          (needs.health < 25 ? BUDDY_EXPRESSION_TIRED :
           (needs.social < 25 ? BUDDY_EXPRESSION_LONELY : BUDDY_EXPRESSION_IDLE))));
    const bool expression_changed = expression != s_ui.displayed_expression;
    const bool needs_changed = needs.health != s_ui.displayed_health ||
                               needs.social != s_ui.displayed_social ||
                               needs.nearby_devices != s_ui.displayed_nearby_devices;
    const bool weather_changed = weather_kind != s_ui.displayed_weather_kind;

    update_time_scene(&weather, weather_kind, time_ms);
    update_weather_ambience(weather_kind, time_ms);
    lv_obj_set_y(s_ui.belly_glow, 137 + breathe);

    if (character_x != s_ui.displayed_character_x || character_y != s_ui.displayed_character_y) {
        lv_obj_set_pos(s_ui.character, character_x, character_y);
        s_ui.displayed_character_x = character_x;
        s_ui.displayed_character_y = character_y;
    }

    const int32_t appendage_bob = celebrating ? (int32_t)(celebration_wave * 12.0f) :
                                  (needs.walking ? (int32_t)(dance_wave * 8.0f) : 0);
    lv_obj_set_y(s_ui.left_ear, 15 - appendage_bob / 2);
    lv_obj_set_y(s_ui.right_ear, 15 + appendage_bob / 2);
    lv_obj_set_y(s_ui.left_arm, 131 + appendage_bob);
    lv_obj_set_y(s_ui.right_arm, 131 - appendage_bob);
    lv_obj_set_y(s_ui.left_foot, 193 - appendage_bob);
    lv_obj_set_y(s_ui.right_foot, 193 + appendage_bob);
    const int32_t bright_pip = needs.walking ? (int32_t)((time_ms / 140) % 4) : -1;
    if (bright_pip != s_ui.displayed_score_pip) {
        for (size_t i = 0; i < 4; ++i) {
            lv_obj_set_style_opa(s_ui.score_pips[i],
                                 bright_pip == (int32_t)i ? LV_OPA_COVER : LV_OPA_30,
                                 LV_PART_MAIN);
        }
        s_ui.displayed_score_pip = bright_pip;
    }

    if (blink != s_ui.displayed_blink || gaze_x != s_ui.displayed_gaze_x ||
        expression != s_ui.displayed_expression) {
        set_eye_expression(blink, expression, gaze_x);
        s_ui.displayed_blink = blink;
        s_ui.displayed_gaze_x = gaze_x;
    }

    if (expression_changed) {
        if (expression == BUDDY_EXPRESSION_SURPRISED) {
            lv_obj_set_pos(s_ui.mouth, 73, 113);
            lv_obj_set_size(s_ui.mouth, 24, 29);
            lv_obj_set_style_radius(s_ui.mouth, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        } else if (expression == BUDDY_EXPRESSION_HAPPY) {
            lv_obj_set_pos(s_ui.mouth, 62, 110);
            lv_obj_set_size(s_ui.mouth, 46, 27);
            lv_obj_set_style_radius(s_ui.mouth, 14, LV_PART_MAIN);
        } else if (expression == BUDDY_EXPRESSION_TIRED) {
            lv_obj_set_pos(s_ui.mouth, 68, 126);
            lv_obj_set_size(s_ui.mouth, 34, 6);
            lv_obj_set_style_radius(s_ui.mouth, 3, LV_PART_MAIN);
        } else if (expression == BUDDY_EXPRESSION_LONELY) {
            lv_obj_set_pos(s_ui.mouth, 71, 124);
            lv_obj_set_size(s_ui.mouth, 28, 8);
            lv_obj_set_style_radius(s_ui.mouth, 4, LV_PART_MAIN);
        } else {
            lv_obj_set_pos(s_ui.mouth, 70, 113);
            lv_obj_set_size(s_ui.mouth, 30, 18);
            lv_obj_set_style_radius(s_ui.mouth, 9, LV_PART_MAIN);
        }

        s_ui.displayed_expression = expression;
    }

    if (expression_changed || needs_changed || weather_changed) {
        lv_obj_set_style_bg_color(s_ui.body,
                                  lv_color_hex(buddy_body_color(expression, weather_kind)),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_opa(s_ui.left_cheek,
                                expression == BUDDY_EXPRESSION_LONELY ? LV_OPA_30 : LV_OPA_70,
                                LV_PART_MAIN);
        lv_obj_set_style_bg_opa(s_ui.right_cheek,
                                expression == BUDDY_EXPRESSION_LONELY ? LV_OPA_30 : LV_OPA_70,
                                LV_PART_MAIN);
        const lv_opa_t sparkle_opa = surprised || expression == BUDDY_EXPRESSION_HAPPY ||
                                     needs.social >= 75 ?
                                     LV_OPA_COVER : LV_OPA_30;
        for (size_t i = 0; i < 4; ++i) {
            lv_obj_set_style_opa(s_ui.sparkles[i], sparkle_opa, LV_PART_MAIN);
        }
        s_ui.displayed_weather_kind = weather_kind;
        s_ui.displayed_nearby_devices = needs.nearby_devices;
    }

    if (score != s_ui.displayed_score) {
        char score_text[4];
        lv_obj_set_width(s_ui.score_bar, 1 + score * 169 / 100);
        lv_obj_set_style_opa(s_ui.score_icon,
                             score < 25 ? LV_OPA_40 : LV_OPA_COVER, LV_PART_MAIN);
        lv_snprintf(score_text, sizeof(score_text), "%u", score);
        lv_label_set_text(s_ui.score_label, score_text);
        s_ui.displayed_score = score;
    }
    const bool activity_highlight = needs.walking || celebrating;
    if ((int32_t)activity_highlight != s_ui.displayed_activity_highlight) {
        lv_obj_set_style_bg_color(s_ui.score_bar,
                                  lv_color_hex(activity_highlight ? COLOR_STAR : COLOR_MINT),
                                  LV_PART_MAIN);
        s_ui.displayed_activity_highlight = activity_highlight;
    }
    if (needs.social_multiplier != s_ui.displayed_multiplier) {
        const char *text = needs.social_multiplier >= 3 ? "3x" :
                           (needs.social_multiplier == 2 ? "2x" : "1x");
        const uint32_t color = needs.social_multiplier >= 3 ? COLOR_STAR :
                               (needs.social_multiplier == 2 ? COLOR_LILAC : COLOR_PANEL_EDGE);
        lv_label_set_text(s_ui.multiplier_label, text);
        lv_obj_set_style_bg_color(s_ui.multiplier_badge, lv_color_hex(color), LV_PART_MAIN);
        lv_obj_set_style_text_color(s_ui.multiplier_label,
                                    lv_color_hex(needs.social_multiplier >= 3 ?
                                                 COLOR_INK : COLOR_CREAM),
                                    LV_PART_MAIN);
        s_ui.displayed_multiplier = needs.social_multiplier;
    }
    s_ui.displayed_health = needs.health;
    s_ui.displayed_social = needs.social;

    if (!s_ui.status_initialized || motion.sensor_online != s_ui.displayed_sensor_online ||
        weather.generation != s_ui.displayed_weather_generation) {
        uint32_t color = COLOR_MUTED;
        if (!motion.sensor_online) {
            color = COLOR_CHEEK;
        } else if (weather.status == WEATHER_STATUS_NEEDS_CONFIG) {
            color = COLOR_PEACH;
        } else if (weather.status == WEATHER_STATUS_UPDATING) {
            color = COLOR_RAIN;
        } else if (weather.status == WEATHER_STATUS_ERROR) {
            color = COLOR_CHEEK;
        } else if (weather.status == WEATHER_STATUS_READY) {
            color = weather_kind == WEATHER_KIND_CLEAR ? COLOR_STAR :
                    (weather_kind == WEATHER_KIND_RAIN || weather_kind == WEATHER_KIND_STORM ?
                     COLOR_RAIN : COLOR_MINT);
        }
        lv_obj_set_style_bg_color(s_ui.status_dot, lv_color_hex(color), LV_PART_MAIN);
        s_ui.displayed_sensor_online = motion.sensor_online;
        s_ui.displayed_weather_generation = weather.generation;
        s_ui.status_initialized = true;
    }

    const bool precipitation = weather_kind == WEATHER_KIND_RAIN ||
                               weather_kind == WEATHER_KIND_SNOW ||
                               weather_kind == WEATHER_KIND_STORM;
    lv_timer_set_period(timer, needs.walking || celebrating || precipitation ?
                        UI_ACTIVE_PERIOD_MS : UI_IDLE_PERIOD_MS);
}

static void create_buddy_ui(void)
{
    lv_obj_t *screen = active_screen();
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    create_time_scene(screen);
    make_decorations(screen);
    create_weather_ambience(screen);
    create_header(screen);
    create_character(screen);
    create_footer(screen);
    s_ui.displayed_expression = -1;
    s_ui.displayed_gaze_x = INT32_MIN;
    s_ui.displayed_character_x = INT32_MIN;
    s_ui.displayed_character_y = INT32_MIN;
    s_ui.displayed_health = INT32_MIN;
    s_ui.displayed_social = INT32_MIN;
    s_ui.displayed_score = INT32_MIN;
    s_ui.displayed_multiplier = INT32_MIN;
    s_ui.displayed_nearby_devices = INT32_MIN;
    s_ui.displayed_weather_generation = UINT32_MAX;
    s_ui.displayed_weather_kind = -1;
    s_ui.displayed_ambience_kind = -1;
    s_ui.displayed_scene_minute = INT64_MIN;
    s_ui.displayed_scene_weather_kind = -1;
    s_ui.displayed_activity_highlight = -1;
    s_ui.displayed_score_pip = INT32_MIN;
    for (size_t i = 0; i < 5; ++i) {
        s_ui.displayed_firefly_opa[i] = -1;
    }

    for (lv_indev_t *indev = lv_indev_get_next(NULL); indev != NULL;
         indev = lv_indev_get_next(indev)) {
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER) {
            lv_indev_add_event_cb(indev, tap_event_cb, LV_EVENT_CLICKED, NULL);
        }
    }
    s_ui.update_timer = lv_timer_create(buddy_update, UI_IDLE_PERIOD_MS, NULL);
}

static esp_err_t detect_imu_address(i2c_master_bus_handle_t bus, uint8_t *address)
{
    if (bus == NULL || address == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t candidates[] = {QMI8658_ADDRESS_HIGH, QMI8658_ADDRESS_LOW};
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        if (i2c_master_probe(bus, candidates[i], IMU_PROBE_TIMEOUT_MS) == ESP_OK) {
            *address = candidates[i];
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t configure_imu(qmi8658_dev_t *imu)
{
    esp_err_t ret = qmi8658_write_register(imu, QMI8658_RESET_REGISTER, QMI8658_RESET_COMMAND);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(QMI8658_RESET_DELAY_MS));

    ret = qmi8658_write_register(imu, QMI8658_CTRL1, QMI8658_CTRL1_VALUE);
    if (ret != ESP_OK) {
        return ret;
    }
    if ((ret = qmi8658_set_accel_range(imu, QMI8658_ACCEL_RANGE_4G)) != ESP_OK ||
        (ret = qmi8658_set_accel_odr(imu, QMI8658_ACCEL_ODR_250HZ)) != ESP_OK ||
        (ret = qmi8658_set_gyro_range(imu, QMI8658_GYRO_RANGE_256DPS)) != ESP_OK ||
        (ret = qmi8658_set_gyro_odr(imu, QMI8658_GYRO_ODR_250HZ)) != ESP_OK) {
        return ret;
    }

    qmi8658_set_accel_unit_mps2(imu, true);
    qmi8658_set_gyro_unit_dps(imu, true);
    return qmi8658_enable_sensors(imu, QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
}

static esp_err_t start_imu(qmi8658_dev_t *imu)
{
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    uint8_t address = 0;
    esp_err_t ret = detect_imu_address(bus, &address);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = qmi8658_init(imu, bus, address);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t who_am_i = 0;
    ret = qmi8658_get_who_am_i(imu, &who_am_i);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "QMI8658 detected at 0x%02x (WHO_AM_I=0x%02x)", address, who_am_i);
    return configure_imu(imu);
}

static void publish_motion(const qmi8658_data_t *data)
{
    static step_tracker_t step_tracker;
    static float filtered_roll = 0.0f;
    static int64_t last_shake_ms = 0;

    const float horizontal = sqrtf(data->accelY * data->accelY + data->accelZ * data->accelZ);
    const float measured_roll = atan2f(data->accelX, horizontal) * 180.0f / BUDDY_PI;
    const float accel_magnitude = sqrtf(data->accelX * data->accelX +
                                        data->accelY * data->accelY +
                                        data->accelZ * data->accelZ);
    const float gyro_magnitude_squared = data->gyroX * data->gyroX +
                                         data->gyroY * data->gyroY +
                                         data->gyroZ * data->gyroZ;
    const bool shake = fabsf(accel_magnitude - GRAVITY_MPS2) > SHAKE_ACCEL_THRESHOLD ||
                       gyro_magnitude_squared >
                       SHAKE_GYRO_THRESHOLD_DPS * SHAKE_GYRO_THRESHOLD_DPS;
    const int64_t time_ms = now_ms();
    const step_tracker_result_t pedometer = step_tracker_update(&step_tracker,
                                                                 accel_magnitude,
                                                                 time_ms);

    filtered_roll = filtered_roll * 0.82f + measured_roll * 0.18f;

    portENTER_CRITICAL(&s_motion_lock);
    s_motion.sensor_online = true;
    s_motion.roll_deg = filtered_roll;
    if (shake && (time_ms - last_shake_ms) > SHAKE_COOLDOWN_MS) {
        ++s_motion.shake_generation;
        last_shake_ms = time_ms;
    }
    portEXIT_CRITICAL(&s_motion_lock);
    pet_state_record_pedometer(pedometer.confirmed_steps, pedometer.cadence_spm,
                               pedometer.walking);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Tamadupi");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(pet_state_init());
    ret = weather_service_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Weather service failed: %s", esp_err_to_name(ret));
    }

    // Let the first Wi-Fi handshake and TLS request use the internal heap
    // before the display and BLE stacks reserve their long-lived buffers.
    for (int elapsed_ms = 0; elapsed_ms < INITIAL_WEATHER_WAIT_MS;
         elapsed_ms += 100) {
        const weather_status_t status = weather_service_snapshot().status;
        if (status == WEATHER_STATUS_READY || status == WEATHER_STATUS_ERROR ||
            status == WEATHER_STATUS_NEEDS_CONFIG) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ret = social_scan_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE social sensing failed: %s", esp_err_to_name(ret));
    }

    bsp_display_cfg_t display_config = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
        },
    };
    display_config.lvgl_port_cfg.task_affinity = LVGL_TASK_CORE;
    lv_display_t *display = bsp_display_start_with_config(&display_config);
    if (display == NULL) {
        ESP_LOGE(TAG, "Display initialization failed");
        return;
    }
    ESP_ERROR_CHECK(bsp_display_brightness_set(80));

    if (!bsp_display_lock(1000)) {
        ESP_LOGE(TAG, "Could not lock LVGL to create the UI");
        return;
    }
    create_buddy_ui();
    bsp_display_unlock();

    qmi8658_dev_t imu = {0};
    ret = start_imu(&imu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed: %s", esp_err_to_name(ret));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    portENTER_CRITICAL(&s_motion_lock);
    s_motion.sensor_online = true;
    portEXIT_CRITICAL(&s_motion_lock);

    uint32_t consecutive_failures = 0;
    while (true) {
        qmi8658_data_t data = {0};
        ret = qmi8658_read_sensor_data(&imu, &data);
        if (ret == ESP_OK) {
            consecutive_failures = 0;
            publish_motion(&data);
        } else {
            ++consecutive_failures;
            if (consecutive_failures == 1 || (consecutive_failures % 25) == 0) {
                ESP_LOGW(TAG, "IMU read failed (%lu consecutive): %s",
                         (unsigned long)consecutive_failures, esp_err_to_name(ret));
            }
            if (consecutive_failures >= 5) {
                portENTER_CRITICAL(&s_motion_lock);
                s_motion.sensor_online = false;
                portEXIT_CRITICAL(&s_motion_lock);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
    }
}
