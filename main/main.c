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
#define BUDDY_PI 3.14159265358979323846f

#define BUDDY_TRAVEL_LIMIT_PX 99.0f
#define BUDDY_ACCEL_FORCE 850.0f
#define BUDDY_CENTERING_SPRING 2.0f
#define BUDDY_MOTION_DAMPING 2.2f
#define BUDDY_WALL_BOUNCE 0.24f
#define BUDDY_MAX_SQUISH_PX 28.0f
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

static const char *TAG = "tamadupi";

typedef struct {
    bool sensor_online;
    float roll_deg;
    float movement;
    float lateral_accel;
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
    lv_obj_t *health_bar;
    lv_obj_t *social_bar;
    lv_obj_t *health_icon;
    lv_obj_t *social_icon;
    lv_obj_t *footprints[4];
    lv_obj_t *weather_sun;
    lv_obj_t *weather_clouds[3];
    lv_obj_t *weather_particles[7];
    lv_obj_t *sparkles[4];
    uint32_t seen_shake_generation;
    int64_t surprised_until_ms;
    bool status_initialized;
    bool displayed_sensor_online;
    uint32_t displayed_weather_generation;
    int displayed_weather_kind;
    int displayed_expression;
    bool displayed_blink;
    int32_t displayed_gaze_x;
    int32_t displayed_character_x;
    int32_t displayed_character_y;
    int32_t displayed_health;
    int32_t displayed_social;
    int32_t displayed_nearby_devices;
    int32_t displayed_compression;
    int displayed_deform_expression;
    float physics_x;
    float physics_velocity;
    float squish;
    int wall_side;
    int64_t last_physics_ms;
} buddy_ui_t;

typedef enum {
    BUDDY_EXPRESSION_IDLE,
    BUDDY_EXPRESSION_TILT,
    BUDDY_EXPRESSION_SURPRISED,
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

static void make_decorations(lv_obj_t *screen)
{
    lv_obj_t *orb = make_shape(screen, -54, 78, 142, 142, 0x252956, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(orb, LV_OPA_40, LV_PART_MAIN);

    orb = make_shape(screen, 304, 222, 108, 108, 0x35294F, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(orb, LV_OPA_50, LV_PART_MAIN);

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
    s_ui.weather_sun = make_shape(screen, 286, 24, 48, 48, COLOR_STAR, LV_RADIUS_CIRCLE);
    lv_obj_add_flag(s_ui.weather_sun, LV_OBJ_FLAG_HIDDEN);

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

    s_ui.health_icon = make_heart(card, 14, 16, COLOR_MINT);
    lv_obj_t *health_track = make_shape(card, 49, 23, 91, 12, 0x30385F, 6);
    s_ui.health_bar = make_shape(health_track, 0, 0, 1, 12, COLOR_MINT, 6);

    s_ui.social_icon = make_shape(card, 181, 15, 28, 28, COLOR_CHEEK,
                                  LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.social_icon, LV_OPA_TRANSP, LV_PART_MAIN);
    make_shape(s_ui.social_icon, 1, 3, 12, 12, COLOR_CHEEK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.social_icon, 15, 3, 12, 12, COLOR_LILAC, LV_RADIUS_CIRCLE);
    make_shape(s_ui.social_icon, 7, 15, 14, 11, COLOR_CHEEK, LV_RADIUS_CIRCLE);
    lv_obj_t *social_track = make_shape(card, 218, 23, 91, 12, 0x30385F, 6);
    s_ui.social_bar = make_shape(social_track, 0, 0, 1, 12, COLOR_CHEEK, 6);

    const int16_t footprint_x[4] = {147, 155, 163, 171};
    for (size_t i = 0; i < 4; ++i) {
        s_ui.footprints[i] = make_shape(card, footprint_x[i],
                                        (i % 2) == 0 ? 16 : 31,
                                        7, 11, COLOR_STAR, LV_RADIUS_CIRCLE);
        lv_obj_set_style_opa(s_ui.footprints[i], LV_OPA_20, LV_PART_MAIN);
    }
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
    const bool tired = expression == BUDDY_EXPRESSION_TIRED;
    const int32_t eye_height = surprised ? 49 : (blink ? 5 : (tired ? 23 : 45));
    const int32_t eye_y = surprised ? 56 : (blink ? 80 : (tired ? 70 : 60));

    lv_obj_set_y(s_ui.left_eye, eye_y);
    lv_obj_set_y(s_ui.right_eye, eye_y);
    lv_obj_set_height(s_ui.left_eye, eye_height);
    lv_obj_set_height(s_ui.right_eye, eye_height);
    lv_obj_set_y(s_ui.left_brow, surprised ? 43 : (tired ? 62 : 51));
    lv_obj_set_y(s_ui.right_brow, surprised ? 43 : (tired ? 62 : 51));

    const lv_opa_t pupil_opa = blink ? LV_OPA_TRANSP : LV_OPA_COVER;
    lv_obj_set_style_opa(s_ui.left_pupil, pupil_opa, LV_PART_MAIN);
    lv_obj_set_style_opa(s_ui.right_pupil, pupil_opa, LV_PART_MAIN);
    lv_obj_set_x(s_ui.left_pupil, 12 + gaze_x);
    lv_obj_set_x(s_ui.right_pupil, 12 + gaze_x);
}

static void update_weather_ambience(weather_kind_t kind, int64_t time_ms)
{
    const bool show_sun = kind == WEATHER_KIND_CLEAR;
    const bool show_cloud = kind == WEATHER_KIND_CLOUDY || kind == WEATHER_KIND_RAIN ||
                            kind == WEATHER_KIND_SNOW || kind == WEATHER_KIND_STORM;
    const bool show_particles = kind == WEATHER_KIND_RAIN || kind == WEATHER_KIND_SNOW ||
                                kind == WEATHER_KIND_STORM;

    if (show_sun) {
        lv_obj_clear_flag(s_ui.weather_sun, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ui.weather_sun, LV_OBJ_FLAG_HIDDEN);
    }
    for (size_t i = 0; i < 3; ++i) {
        if (show_cloud) {
            lv_obj_clear_flag(s_ui.weather_clouds[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_ui.weather_clouds[i], LV_OBJ_FLAG_HIDDEN);
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

static float update_buddy_physics(const motion_state_t *motion, int64_t time_ms)
{
    if (s_ui.last_physics_ms == 0) {
        s_ui.last_physics_ms = time_ms;
        return 0.0f;
    }

    const float dt = clampf((float)(time_ms - s_ui.last_physics_ms) / 1000.0f, 0.01f, 0.12f);
    s_ui.last_physics_ms = time_ms;

    const float force = -motion->lateral_accel * BUDDY_ACCEL_FORCE;
    const float acceleration = force - s_ui.physics_x * BUDDY_CENTERING_SPRING -
                               s_ui.physics_velocity * BUDDY_MOTION_DAMPING;
    s_ui.physics_velocity += acceleration * dt;
    s_ui.physics_velocity = clampf(s_ui.physics_velocity, -620.0f, 620.0f);
    s_ui.physics_x += s_ui.physics_velocity * dt;

    float squish_target = 0.0f;
    if (s_ui.physics_x <= -BUDDY_TRAVEL_LIMIT_PX) {
        const float impact_speed = fabsf(s_ui.physics_velocity);
        s_ui.physics_x = -BUDDY_TRAVEL_LIMIT_PX;
        s_ui.wall_side = -1;
        if (s_ui.physics_velocity < 0.0f) {
            s_ui.physics_velocity *= -BUDDY_WALL_BOUNCE;
        }
        squish_target = clampf(impact_speed / 420.0f + fmaxf(-force, 0.0f) / 2600.0f, 0.0f, 1.0f);
    } else if (s_ui.physics_x >= BUDDY_TRAVEL_LIMIT_PX) {
        const float impact_speed = fabsf(s_ui.physics_velocity);
        s_ui.physics_x = BUDDY_TRAVEL_LIMIT_PX;
        s_ui.wall_side = 1;
        if (s_ui.physics_velocity > 0.0f) {
            s_ui.physics_velocity *= -BUDDY_WALL_BOUNCE;
        }
        squish_target = clampf(impact_speed / 420.0f + fmaxf(force, 0.0f) / 2600.0f, 0.0f, 1.0f);
    }

    if (squish_target > s_ui.squish) {
        s_ui.squish += (squish_target - s_ui.squish) * 0.72f;
    } else {
        s_ui.squish *= 0.74f;
    }
    if (s_ui.squish < 0.01f) {
        s_ui.squish = 0.0f;
        if (fabsf(s_ui.physics_x) < BUDDY_TRAVEL_LIMIT_PX - 2.0f) {
            s_ui.wall_side = 0;
        }
    }
    return s_ui.squish;
}

static void deform_buddy(float squish, buddy_expression_t expression)
{
    const int32_t compression = (int32_t)(squish * BUDDY_MAX_SQUISH_PX);
    if (compression == s_ui.displayed_compression &&
        expression == s_ui.displayed_deform_expression) {
        return;
    }

    const int32_t body_width = BUDDY_BODY_WIDTH - compression;
    const int32_t body_x = BUDDY_BODY_X + (s_ui.wall_side > 0 ? compression : 0);
    const int32_t stretch = compression / 2;
    const float scale_x = (float)body_width / (float)BUDDY_BODY_WIDTH;

    lv_obj_set_pos(s_ui.body, body_x, BUDDY_BODY_Y - stretch / 2);
    lv_obj_set_size(s_ui.body, body_width, BUDDY_BODY_HEIGHT + stretch);

    lv_obj_set_x(s_ui.left_eye, (int32_t)(50.0f * scale_x) - 21);
    lv_obj_set_x(s_ui.right_eye, (int32_t)(120.0f * scale_x) - 21);
    lv_obj_set_x(s_ui.left_brow, (int32_t)(49.5f * scale_x) - 14);
    lv_obj_set_x(s_ui.right_brow, (int32_t)(120.5f * scale_x) - 14);
    lv_obj_set_x(s_ui.left_cheek, (int32_t)(31.5f * scale_x) - 18);
    lv_obj_set_x(s_ui.right_cheek, (int32_t)(138.5f * scale_x) - 18);
    lv_obj_set_x(s_ui.belly_glow, (int32_t)(53.0f * scale_x));
    lv_obj_set_width(s_ui.belly_glow, (int32_t)(64.0f * scale_x));

    const int32_t mouth_width = expression == BUDDY_EXPRESSION_SURPRISED ? 24 :
                                (expression == BUDDY_EXPRESSION_TIRED ? 34 :
                                 (expression == BUDDY_EXPRESSION_LONELY ? 28 : 30));
    lv_obj_set_x(s_ui.mouth, body_width / 2 - mouth_width / 2);
    s_ui.displayed_compression = compression;
    s_ui.displayed_deform_expression = expression;
}

static void buddy_update(lv_timer_t *timer)
{
    (void)timer;
    const motion_state_t motion = motion_snapshot();
    const pet_state_snapshot_t needs = pet_state_snapshot();
    const weather_snapshot_t weather = weather_service_snapshot();
    const weather_kind_t weather_kind = classify_weather(&weather);
    const int64_t time_ms = now_ms();

    if (weather.status == WEATHER_STATUS_UPDATING) {
        if (!s_ui.status_initialized ||
            weather.generation != s_ui.displayed_weather_generation) {
            lv_obj_set_style_bg_color(s_ui.status_dot, lv_color_hex(COLOR_RAIN), LV_PART_MAIN);
            s_ui.displayed_sensor_online = motion.sensor_online;
            s_ui.displayed_weather_generation = weather.generation;
            s_ui.status_initialized = true;
        }
        return;
    }

    if (motion.shake_generation != s_ui.seen_shake_generation) {
        s_ui.seen_shake_generation = motion.shake_generation;
        s_ui.surprised_until_ms = time_ms + 850;
    }

    const bool surprised = time_ms < s_ui.surprised_until_ms;
    const bool blink = !surprised && ((time_ms % 4300) > 4130);
    const float roll = clampf(motion.roll_deg, -28.0f, 28.0f);
    const float squish = update_buddy_physics(&motion, time_ms);
    const float phase = (float)(time_ms % 1800) / 1800.0f;
    const int32_t weather_bounce = weather_kind == WEATHER_KIND_CLEAR ?
                                   (int32_t)(sinf(phase * 4.0f * BUDDY_PI) * 1.5f) : 0;
    const int32_t breathe = (int32_t)(sinf(phase * 2.0f * BUDDY_PI) * 2.0f) +
                            weather_bounce;
    const int32_t movement_bob = (int32_t)clampf(motion.movement * 2.2f, 0.0f, 11.0f);
    const int32_t character_x = 69 + (int32_t)s_ui.physics_x;
    const int32_t character_y = 83 + breathe - movement_bob;
    const int32_t gaze_x = (int32_t)(roll / 7.0f);
    const buddy_expression_t expression = surprised ? BUDDY_EXPRESSION_SURPRISED :
        (fabsf(roll) > 11.0f ? BUDDY_EXPRESSION_TILT :
         (needs.health < 25 ? BUDDY_EXPRESSION_TIRED :
          (needs.social < 25 ? BUDDY_EXPRESSION_LONELY : BUDDY_EXPRESSION_IDLE)));
    const bool expression_changed = expression != s_ui.displayed_expression;
    const bool needs_changed = needs.health != s_ui.displayed_health ||
                               needs.social != s_ui.displayed_social ||
                               needs.nearby_devices != s_ui.displayed_nearby_devices;
    const bool weather_changed = weather_kind != s_ui.displayed_weather_kind;

    update_weather_ambience(weather_kind, time_ms);

    if (character_x != s_ui.displayed_character_x || character_y != s_ui.displayed_character_y) {
        lv_obj_set_pos(s_ui.character, character_x, character_y);
        s_ui.displayed_character_x = character_x;
        s_ui.displayed_character_y = character_y;
    }

    const float walk_wave = sinf((float)time_ms *
                                  (needs.walking ? 0.012f : 0.0035f));
    const int32_t appendage_bob = needs.walking ? (int32_t)(walk_wave * 4.0f) : 0;
    lv_obj_set_y(s_ui.left_ear, 15 - appendage_bob / 2);
    lv_obj_set_y(s_ui.right_ear, 15 + appendage_bob / 2);
    lv_obj_set_y(s_ui.left_arm, 131 + appendage_bob);
    lv_obj_set_y(s_ui.right_arm, 131 - appendage_bob);
    lv_obj_set_y(s_ui.left_foot, 193 - appendage_bob);
    lv_obj_set_y(s_ui.right_foot, 193 + appendage_bob);
    const size_t bright_footprint = (size_t)((time_ms / 140) % 4);
    for (size_t i = 0; i < 4; ++i) {
        lv_obj_set_style_opa(s_ui.footprints[i],
                             needs.walking && i == bright_footprint ? LV_OPA_COVER : LV_OPA_20,
                             LV_PART_MAIN);
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
        const lv_opa_t sparkle_opa = surprised || needs.social >= 75 ?
                                     LV_OPA_COVER : LV_OPA_30;
        for (size_t i = 0; i < 4; ++i) {
            lv_obj_set_style_opa(s_ui.sparkles[i], sparkle_opa, LV_PART_MAIN);
        }
        s_ui.displayed_weather_kind = weather_kind;
        s_ui.displayed_nearby_devices = needs.nearby_devices;
    }

    deform_buddy(squish, expression);

    if (needs.health != s_ui.displayed_health) {
        lv_obj_set_width(s_ui.health_bar, 1 + needs.health * 90 / 100);
        lv_obj_set_style_opa(s_ui.health_icon,
                             needs.health < 25 ? LV_OPA_40 : LV_OPA_COVER,
                             LV_PART_MAIN);
        s_ui.displayed_health = needs.health;
    }

    if (needs.social != s_ui.displayed_social) {
        lv_obj_set_width(s_ui.social_bar, 1 + needs.social * 90 / 100);
        lv_obj_set_style_opa(s_ui.social_icon,
                             needs.social < 25 ? LV_OPA_40 : LV_OPA_COVER,
                             LV_PART_MAIN);
        s_ui.displayed_social = needs.social;
    }

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
}

static void create_buddy_ui(void)
{
    lv_obj_t *screen = active_screen();
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

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
    s_ui.displayed_nearby_devices = INT32_MIN;
    s_ui.displayed_weather_generation = UINT32_MAX;
    s_ui.displayed_weather_kind = -1;
    s_ui.displayed_compression = INT32_MIN;
    s_ui.displayed_deform_expression = -1;
    lv_timer_create(buddy_update, 66, NULL);
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
    static float gravity_x = 0.0f;
    static bool gravity_initialized = false;
    static int64_t last_shake_ms = 0;

    const float horizontal = sqrtf(data->accelY * data->accelY + data->accelZ * data->accelZ);
    const float measured_roll = atan2f(data->accelX, horizontal) * 180.0f / BUDDY_PI;
    const float accel_magnitude = sqrtf(data->accelX * data->accelX +
                                        data->accelY * data->accelY +
                                        data->accelZ * data->accelZ);
    const float gyro_magnitude = sqrtf(data->gyroX * data->gyroX +
                                       data->gyroY * data->gyroY +
                                       data->gyroZ * data->gyroZ);
    const float movement = fabsf(accel_magnitude - GRAVITY_MPS2) + gyro_magnitude * 0.012f;
    const bool shake = fabsf(accel_magnitude - GRAVITY_MPS2) > SHAKE_ACCEL_THRESHOLD ||
                       gyro_magnitude > SHAKE_GYRO_THRESHOLD_DPS;
    const int64_t time_ms = now_ms();
    const step_tracker_result_t pedometer = step_tracker_update(&step_tracker,
                                                                 accel_magnitude,
                                                                 time_ms);

    filtered_roll = filtered_roll * 0.82f + measured_roll * 0.18f;
    if (!gravity_initialized) {
        gravity_x = data->accelX;
        gravity_initialized = true;
    }
    gravity_x = gravity_x * 0.94f + data->accelX * 0.06f;
    const float lateral_accel = data->accelX - gravity_x;

    portENTER_CRITICAL(&s_motion_lock);
    s_motion.sensor_online = true;
    s_motion.roll_deg = filtered_roll;
    s_motion.movement = s_motion.movement * 0.72f + movement * 0.28f;
    s_motion.lateral_accel = s_motion.lateral_accel * 0.55f + lateral_accel * 0.45f;
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

    lv_display_t *display = bsp_display_start();
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
