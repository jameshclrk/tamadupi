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
    lv_obj_t *mouth;
    lv_obj_t *belly_glow;
    lv_obj_t *left_cheek;
    lv_obj_t *right_cheek;
    lv_obj_t *status_dot;
    lv_obj_t *status_label;
    lv_obj_t *hint_label;
    lv_obj_t *health_bar;
    lv_obj_t *social_bar;
    lv_obj_t *health_value_label;
    lv_obj_t *social_value_label;
    lv_obj_t *steps_label;
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
    uint32_t displayed_steps;
    uint16_t displayed_cadence_spm;
    bool displayed_walking;
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

static void make_decorations(lv_obj_t *screen)
{
    lv_obj_t *orb = make_shape(screen, -54, 78, 142, 142, 0x252956, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(orb, LV_OPA_40, LV_PART_MAIN);

    orb = make_shape(screen, 304, 222, 108, 108, 0x35294F, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(orb, LV_OPA_50, LV_PART_MAIN);

    const int16_t positions[4][2] = {{54, 147}, {292, 130}, {48, 277}, {304, 276}};
    const char *texts[4] = {"+", "x", "x", "+"};

    for (size_t i = 0; i < 4; ++i) {
        lv_obj_t *sparkle = lv_label_create(screen);
        lv_label_set_text(sparkle, texts[i]);
        lv_obj_set_pos(sparkle, positions[i][0], positions[i][1]);
        lv_obj_set_style_text_color(sparkle, lv_color_hex(COLOR_STAR), LV_PART_MAIN);
        lv_obj_set_style_text_opa(sparkle, LV_OPA_30, LV_PART_MAIN);
        s_ui.sparkles[i] = sparkle;
    }
}

static void create_weather_ambience(lv_obj_t *screen)
{
    s_ui.weather_sun = make_shape(screen, 280, 82, 52, 52, COLOR_STAR, LV_RADIUS_CIRCLE);
    lv_obj_add_flag(s_ui.weather_sun, LV_OBJ_FLAG_HIDDEN);

    s_ui.weather_clouds[0] = make_shape(screen, 256, 100, 82, 29,
                                        COLOR_CLOUD, LV_RADIUS_CIRCLE);
    s_ui.weather_clouds[1] = make_shape(screen, 268, 88, 38, 36,
                                        COLOR_CLOUD, LV_RADIUS_CIRCLE);
    s_ui.weather_clouds[2] = make_shape(screen, 294, 91, 34, 33,
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

static const char *weather_name(weather_kind_t kind)
{
    switch (kind) {
    case WEATHER_KIND_CLEAR:
        return "clear";
    case WEATHER_KIND_CLOUDY:
        return "cloudy";
    case WEATHER_KIND_RAIN:
        return "rain";
    case WEATHER_KIND_SNOW:
        return "snow";
    case WEATHER_KIND_STORM:
        return "storm";
    default:
        return "weather";
    }
}

static void create_header(lv_obj_t *screen)
{
    lv_obj_t *eyebrow = lv_label_create(screen);
    lv_label_set_text(eyebrow, "TAMADUPI");
    lv_obj_set_pos(eyebrow, 24, 22);
    lv_obj_set_style_text_color(eyebrow, lv_color_hex(COLOR_MINT), LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(eyebrow, 2, LV_PART_MAIN);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "Meet Mochi");
    lv_obj_set_pos(title, 24, 45);
    lv_obj_set_style_text_color(title, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);

    lv_obj_t *pill = make_shape(screen, 236, 25, 108, 34, COLOR_PANEL, 17);
    lv_obj_set_style_border_width(pill, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(pill, lv_color_hex(COLOR_PANEL_EDGE), LV_PART_MAIN);

    s_ui.status_dot = make_shape(pill, 12, 12, 10, 10, COLOR_MUTED, LV_RADIUS_CIRCLE);
    s_ui.status_label = lv_label_create(pill);
    lv_label_set_text(s_ui.status_label, "starting");
    lv_obj_set_pos(s_ui.status_label, 29, 9);
    lv_obj_set_style_text_color(s_ui.status_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
}

static void create_character(lv_obj_t *screen)
{
    lv_obj_t *shadow = make_shape(screen, 105, 325, 158, 24, 0x090B1D, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(shadow, LV_OPA_50, LV_PART_MAIN);

    s_ui.character = lv_obj_create(screen);
    lv_obj_set_pos(s_ui.character, 69, 105);
    lv_obj_set_size(s_ui.character, 230, 230);
    lv_obj_set_style_bg_opa(s_ui.character, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_ui.character, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_ui.character, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_ui.character, LV_OBJ_FLAG_SCROLLABLE);
    make_shape(s_ui.character, 55, 31, 60, 74, COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.character, 115, 31, 60, 74, COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.character, 66, 43, 38, 49, COLOR_PEACH_LIGHT, LV_RADIUS_CIRCLE);
    make_shape(s_ui.character, 126, 43, 38, 49, COLOR_PEACH_LIGHT, LV_RADIUS_CIRCLE);

    make_shape(s_ui.character, 20, 128, 58, 27, COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.character, 152, 128, 58, 27, COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.character, 58, 185, 49, 31, COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);
    make_shape(s_ui.character, 123, 185, 49, 31, COLOR_PEACH_DARK, LV_RADIUS_CIRCLE);

    s_ui.body = make_shape(s_ui.character, 40, 57, 150, 151, COLOR_PEACH, 70);
    lv_obj_set_style_border_width(s_ui.body, 3, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_ui.body, lv_color_hex(COLOR_PEACH_LIGHT), LV_PART_MAIN);

    s_ui.belly_glow = make_shape(s_ui.body, 23, 18, 87, 51, COLOR_PEACH_LIGHT, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.belly_glow, LV_OPA_30, LV_PART_MAIN);

    s_ui.left_eye = make_shape(s_ui.body, 34, 51, 30, 34, COLOR_TEXT, LV_RADIUS_CIRCLE);
    s_ui.right_eye = make_shape(s_ui.body, 86, 51, 30, 34, COLOR_TEXT, LV_RADIUS_CIRCLE);
    s_ui.left_pupil = make_shape(s_ui.left_eye, 9, 10, 12, 15, COLOR_INK, LV_RADIUS_CIRCLE);
    s_ui.right_pupil = make_shape(s_ui.right_eye, 9, 10, 12, 15, COLOR_INK, LV_RADIUS_CIRCLE);

    s_ui.left_cheek = make_shape(s_ui.body, 17, 91, 27, 13, COLOR_CHEEK, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.left_cheek, LV_OPA_70, LV_PART_MAIN);
    s_ui.right_cheek = make_shape(s_ui.body, 106, 91, 27, 13, COLOR_CHEEK, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(s_ui.right_cheek, LV_OPA_70, LV_PART_MAIN);

    s_ui.mouth = make_shape(s_ui.body, 61, 91, 28, 16, COLOR_INK, 8);
}

static void create_footer(lv_obj_t *screen)
{
    s_ui.hint_label = lv_label_create(screen);
    lv_label_set_text(s_ui.hint_label, "Walk with me!");
    lv_obj_set_pos(s_ui.hint_label, 20, 340);
    lv_obj_set_width(s_ui.hint_label, 328);
    lv_obj_set_style_text_align(s_ui.hint_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_ui.hint_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);

    lv_obj_t *card = make_shape(screen, 20, 363, 328, 72, COLOR_PANEL, 18);
    lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, lv_color_hex(COLOR_PANEL_EDGE), LV_PART_MAIN);

    lv_obj_t *health_label = lv_label_create(card);
    lv_label_set_text(health_label, "HEALTH");
    lv_obj_set_pos(health_label, 17, 10);
    lv_obj_set_style_text_color(health_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);

    s_ui.health_value_label = lv_label_create(card);
    lv_label_set_text(s_ui.health_value_label, "--");
    lv_obj_set_pos(s_ui.health_value_label, 119, 9);
    lv_obj_set_width(s_ui.health_value_label, 32);
    lv_obj_set_style_text_align(s_ui.health_value_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_ui.health_value_label, lv_color_hex(COLOR_MINT), LV_PART_MAIN);

    lv_obj_t *social_label = lv_label_create(card);
    lv_label_set_text(social_label, "SOCIAL");
    lv_obj_set_pos(social_label, 177, 10);
    lv_obj_set_style_text_color(social_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);

    s_ui.social_value_label = lv_label_create(card);
    lv_label_set_text(s_ui.social_value_label, "--");
    lv_obj_set_pos(s_ui.social_value_label, 279, 9);
    lv_obj_set_width(s_ui.social_value_label, 32);
    lv_obj_set_style_text_align(s_ui.social_value_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_ui.social_value_label, lv_color_hex(COLOR_CHEEK), LV_PART_MAIN);

    lv_obj_t *health_track = make_shape(card, 17, 40, 134, 10, 0x30385F, 5);
    s_ui.health_bar = make_shape(health_track, 0, 0, 1, 10, COLOR_MINT, 5);
    lv_obj_t *social_track = make_shape(card, 177, 40, 134, 10, 0x30385F, 5);
    s_ui.social_bar = make_shape(social_track, 0, 0, 1, 10, COLOR_CHEEK, 5);

    s_ui.steps_label = lv_label_create(card);
    lv_label_set_text(s_ui.steps_label, "0 steps");
    lv_obj_set_pos(s_ui.steps_label, 17, 53);
    lv_obj_set_width(s_ui.steps_label, 134);
    lv_obj_set_style_text_align(s_ui.steps_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_ui.steps_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
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
    const int32_t eye_height = surprised ? 39 : (blink ? 5 : (tired ? 20 : 34));
    const int32_t eye_y = surprised ? 48 : (blink ? 66 : (tired ? 58 : 51));

    lv_obj_set_y(s_ui.left_eye, eye_y);
    lv_obj_set_y(s_ui.right_eye, eye_y);
    lv_obj_set_height(s_ui.left_eye, eye_height);
    lv_obj_set_height(s_ui.right_eye, eye_height);

    const lv_opa_t pupil_opa = blink ? LV_OPA_TRANSP : LV_OPA_COVER;
    lv_obj_set_style_bg_opa(s_ui.left_pupil, pupil_opa, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_ui.right_pupil, pupil_opa, LV_PART_MAIN);
    lv_obj_set_x(s_ui.left_pupil, 9 + gaze_x);
    lv_obj_set_x(s_ui.right_pupil, 9 + gaze_x);
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

static void set_buddy_hint(buddy_expression_t expression, weather_kind_t weather,
                           const pet_state_snapshot_t *needs)
{
    if (expression == BUDDY_EXPRESSION_SURPRISED) {
        lv_label_set_text(s_ui.hint_label, "Whoa! That tickles!");
    } else if (expression == BUDDY_EXPRESSION_TILT) {
        lv_label_set_text(s_ui.hint_label, "Wheee! Keep tilting!");
    } else if (expression == BUDDY_EXPRESSION_TIRED) {
        lv_label_set_text(s_ui.hint_label, "Let's go for a little walk.");
    } else if (expression == BUDDY_EXPRESSION_LONELY) {
        lv_label_set_text(s_ui.hint_label, needs->nearby_devices == 0 ?
                                           "I could use some company." :
                                           "Oh! I can sense someone nearby!");
    } else if (weather == WEATHER_KIND_RAIN || weather == WEATHER_KIND_STORM) {
        lv_label_set_text(s_ui.hint_label, "Rainy-day cuddles!");
    } else if (weather == WEATHER_KIND_SNOW) {
        lv_label_set_text(s_ui.hint_label, "Brrr... snowflakes!");
    } else if (weather == WEATHER_KIND_CLEAR) {
        lv_label_set_text(s_ui.hint_label, "Sunshine makes me bouncy!");
    } else {
        lv_label_set_text(s_ui.hint_label, "Walk with me!");
    }
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

    const int32_t body_width = 150 - compression;
    const int32_t body_x = 40 + (s_ui.wall_side > 0 ? compression : 0);
    const int32_t stretch = compression / 2;
    const float scale_x = (float)body_width / 150.0f;

    lv_obj_set_pos(s_ui.body, body_x, 57 - stretch / 2);
    lv_obj_set_size(s_ui.body, body_width, 151 + stretch);

    lv_obj_set_x(s_ui.left_eye, (int32_t)(49.0f * scale_x) - 15);
    lv_obj_set_x(s_ui.right_eye, (int32_t)(101.0f * scale_x) - 15);
    lv_obj_set_x(s_ui.left_cheek, (int32_t)(30.5f * scale_x) - 13);
    lv_obj_set_x(s_ui.right_cheek, (int32_t)(119.5f * scale_x) - 13);
    lv_obj_set_x(s_ui.belly_glow, (int32_t)(23.0f * scale_x));
    lv_obj_set_width(s_ui.belly_glow, (int32_t)(87.0f * scale_x));

    const int32_t mouth_width = expression == BUDDY_EXPRESSION_SURPRISED ? 22 :
                                (expression == BUDDY_EXPRESSION_TIRED ? 30 :
                                 (expression == BUDDY_EXPRESSION_LONELY ? 24 : 28));
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
            lv_obj_set_style_text_color(s_ui.status_label, lv_color_hex(COLOR_RAIN), LV_PART_MAIN);
            lv_label_set_text(s_ui.status_label, "weather...");
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
    const int32_t character_y = 105 + breathe - movement_bob;
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

    if (blink != s_ui.displayed_blink || gaze_x != s_ui.displayed_gaze_x ||
        expression != s_ui.displayed_expression) {
        set_eye_expression(blink, expression, gaze_x);
        s_ui.displayed_blink = blink;
        s_ui.displayed_gaze_x = gaze_x;
    }

    if (expression_changed) {
        if (expression == BUDDY_EXPRESSION_SURPRISED) {
            lv_obj_set_pos(s_ui.mouth, 64, 94);
            lv_obj_set_size(s_ui.mouth, 22, 27);
            lv_obj_set_style_radius(s_ui.mouth, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        } else if (expression == BUDDY_EXPRESSION_TIRED) {
            lv_obj_set_pos(s_ui.mouth, 60, 100);
            lv_obj_set_size(s_ui.mouth, 30, 6);
            lv_obj_set_style_radius(s_ui.mouth, 3, LV_PART_MAIN);
        } else if (expression == BUDDY_EXPRESSION_LONELY) {
            lv_obj_set_pos(s_ui.mouth, 63, 99);
            lv_obj_set_size(s_ui.mouth, 24, 8);
            lv_obj_set_style_radius(s_ui.mouth, 4, LV_PART_MAIN);
        } else {
            lv_obj_set_pos(s_ui.mouth, 61, 93);
            lv_obj_set_size(s_ui.mouth, 28, 15);
            lv_obj_set_style_radius(s_ui.mouth, 8, LV_PART_MAIN);
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
            lv_obj_set_style_text_opa(s_ui.sparkles[i], sparkle_opa, LV_PART_MAIN);
        }
        set_buddy_hint(expression, weather_kind, &needs);
        s_ui.displayed_weather_kind = weather_kind;
        s_ui.displayed_nearby_devices = needs.nearby_devices;
    }

    deform_buddy(squish, expression);

    if (needs.health != s_ui.displayed_health) {
        char value[4];
        lv_snprintf(value, sizeof(value), "%u", needs.health);
        lv_label_set_text(s_ui.health_value_label, value);
        lv_obj_set_width(s_ui.health_bar, 1 + needs.health * 133 / 100);
        s_ui.displayed_health = needs.health;
    }

    if (needs.social != s_ui.displayed_social) {
        char value[4];
        lv_snprintf(value, sizeof(value), "%u", needs.social);
        lv_label_set_text(s_ui.social_value_label, value);
        lv_obj_set_width(s_ui.social_bar, 1 + needs.social * 133 / 100);
        s_ui.displayed_social = needs.social;
    }

    if (needs.steps != s_ui.displayed_steps ||
        needs.cadence_spm != s_ui.displayed_cadence_spm ||
        needs.walking != s_ui.displayed_walking) {
        char value[24];
        if (needs.walking && needs.cadence_spm > 0) {
            lv_snprintf(value, sizeof(value), "%lu | %u spm",
                        (unsigned long)needs.steps, needs.cadence_spm);
        } else {
            lv_snprintf(value, sizeof(value), "%lu steps",
                        (unsigned long)needs.steps);
        }
        lv_label_set_text(s_ui.steps_label, value);
        s_ui.displayed_steps = needs.steps;
        s_ui.displayed_cadence_spm = needs.cadence_spm;
        s_ui.displayed_walking = needs.walking;
    }

    if (!s_ui.status_initialized || motion.sensor_online != s_ui.displayed_sensor_online ||
        weather.generation != s_ui.displayed_weather_generation) {
        uint32_t color = COLOR_MUTED;
        char status[20];
        if (!motion.sensor_online) {
            color = COLOR_CHEEK;
            lv_snprintf(status, sizeof(status), "no IMU");
        } else if (weather.status == WEATHER_STATUS_NEEDS_CONFIG) {
            color = COLOR_PEACH;
            lv_snprintf(status, sizeof(status), "Wi-Fi setup");
        } else if (weather.status == WEATHER_STATUS_CONNECTING) {
            lv_snprintf(status, sizeof(status), "Wi-Fi...");
        } else if (weather.status == WEATHER_STATUS_UPDATING) {
            color = COLOR_RAIN;
            lv_snprintf(status, sizeof(status), "weather...");
        } else if (weather.status == WEATHER_STATUS_ERROR) {
            color = COLOR_CHEEK;
            lv_snprintf(status, sizeof(status), "weather err");
        } else {
            color = weather_kind == WEATHER_KIND_CLEAR ? COLOR_STAR :
                    (weather_kind == WEATHER_KIND_RAIN || weather_kind == WEATHER_KIND_STORM ?
                     COLOR_RAIN : COLOR_MINT);
            const int temperature_c = (int)(weather.temperature_c >= 0.0f ?
                                             weather.temperature_c + 0.5f :
                                             weather.temperature_c - 0.5f);
            lv_snprintf(status, sizeof(status), "%dC %s",
                        temperature_c, weather_name(weather_kind));
        }
        lv_obj_set_style_bg_color(s_ui.status_dot, lv_color_hex(color), LV_PART_MAIN);
        lv_obj_set_style_text_color(s_ui.status_label, lv_color_hex(color), LV_PART_MAIN);
        lv_label_set_text(s_ui.status_label, status);
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
    s_ui.displayed_steps = UINT32_MAX;
    s_ui.displayed_cadence_spm = UINT16_MAX;
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
