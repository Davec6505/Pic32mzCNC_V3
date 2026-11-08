#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

typedef enum { AXIS_X, AXIS_Y, AXIS_Z, AXIS_A, NUM_AXIS } E_AXIS;
typedef struct {
    uint32_t* homing_debounce;
    uint8_t* homing_dir_mask;
} HomingSettings;
static HomingSettings g_homing_settings[NUM_AXIS];
static struct { E_AXIS current_axis; } g_homing;
static bool min_limits[NUM_AXIS];
static bool max_limits[NUM_AXIS];
#define LIMIT_GetMin(axis) min_limits[axis]
#define LIMIT_GetMax(axis) max_limits[axis]
static uint32_t timer_count;
static uint32_t timer_frequency = 100000000U;
static uint32_t debounce_ms;
static unsigned checks;
static unsigned failures;

uint32_t CORETIMER_CounterGet(void) { return timer_count; }
uint32_t CORETIMER_FrequencyGet(void) { return timer_frequency; }

/* PRODUCTION_SOURCE */

static CNC_Settings current_settings;
CNC_Settings* SETTINGS_GetCurrent(void) { return &current_settings; }

#define CHECK(condition) do { \
    checks++; \
    if (!(condition)) { \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
        failures++; \
    } \
} while (0)

static void reset(uint32_t delay, E_AXIS axis, uint32_t start)
{
    debounce_ms = delay;
    timer_count = start;
    for (E_AXIS i = AXIS_X; i < NUM_AXIS; i++) {
        g_homing_settings[i].homing_debounce = &debounce_ms;
    }
    UTILS_HomingSetCurrentAxis(axis);
    UTILS_HomingLimitReset();
}

static void sample(bool active, uint32_t ticks)
{
    timer_count += ticks;
    UTILS_HomingLimitUpdate(active);
}

static void test_thresholds(void)
{
    static const uint32_t delays[] = {1U, 10U, 25U, 250U, 2000U};
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        for (unsigned i = 0; i < sizeof(delays) / sizeof(delays[0]); i++) {
            reset(delays[i], axis, 0U);
            sample(false, 0U);
            CHECK(!UTILS_HomingLimitRisingEdge());
            sample(true, 0U);
            CHECK(!UTILS_HomingLimitRisingEdge());
            sample(true, delays[i] * (timer_frequency / 1000U) - 1U);
            CHECK(!UTILS_HomingLimitRisingEdge());
            sample(true, 1U);
            CHECK(UTILS_HomingLimitRisingEdge());
            CHECK(!UTILS_HomingLimitRisingEdge());
            sample(true, 1U);
            CHECK(!UTILS_HomingLimitRisingEdge());
            sample(false, 0U);
            sample(false, delays[i] * (timer_frequency / 1000U) - 1U);
            CHECK(!UTILS_HomingLimitFallingEdge());
            sample(false, 1U);
            CHECK(UTILS_HomingLimitFallingEdge());
            CHECK(!UTILS_HomingLimitFallingEdge());
        }
    }
}

static void test_bounce_and_reset(void)
{
    reset(10U, AXIS_Z, 0U);
    sample(true, 0U);
    sample(true, 900000U);
    sample(false, 0U);
    sample(true, 0U);
    sample(true, 999999U);
    CHECK(!UTILS_HomingLimitRisingEdge());
    sample(true, 1U);
    CHECK(UTILS_HomingLimitRisingEdge());
    sample(false, 0U);
    sample(false, 900000U);
    sample(true, 0U);
    sample(false, 0U);
    sample(false, 999999U);
    CHECK(!UTILS_HomingLimitFallingEdge());
    sample(false, 1U);
    CHECK(UTILS_HomingLimitFallingEdge());

    sample(true, 0U);
    sample(true, 900000U);
    UTILS_HomingLimitResetAxis(AXIS_Z);
    sample(true, 100000U);
    CHECK(!UTILS_HomingLimitRisingEdge());
    sample(true, 1000000U);
    CHECK(UTILS_HomingLimitRisingEdge());
    sample(false, 0U);
    sample(false, 1000000U);
    UTILS_HomingLimitReset();
    CHECK(!UTILS_HomingLimitRisingEdge());
    CHECK(!UTILS_HomingLimitFallingEdge());
    sample(true, 0U);
    sample(true, 1000000U);
    CHECK(UTILS_HomingLimitRisingEdge());

    UTILS_HomingSetCurrentAxis(AXIS_Y);
    sample(true, 0U);
    sample(true, 1000000U);
    CHECK(UTILS_HomingLimitRisingEdge());
    UTILS_HomingSetCurrentAxis(AXIS_Z);
    sample(true, 1000000U);
    CHECK(!UTILS_HomingLimitRisingEdge());
}

static void test_zero_and_live_settings(void)
{
    reset(0U, AXIS_Z, 0U);
    sample(true, 0U);
    CHECK(UTILS_HomingLimitRisingEdge());
    sample(false, 0U);
    CHECK(UTILS_HomingLimitFallingEdge());
    CHECK(!UTILS_HomingLimitFallingEdge());

    reset(25U, AXIS_Y, 0U);
    sample(true, 0U);
    sample(true, 1000000U);
    CHECK(!UTILS_HomingLimitRisingEdge());
    debounce_ms = 10U;
    sample(true, 0U);
    CHECK(UTILS_HomingLimitRisingEdge());
    debounce_ms = 25U;
    sample(false, 0U);
    sample(false, 2499999U);
    CHECK(!UTILS_HomingLimitFallingEdge());
    sample(false, 1U);
    CHECK(UTILS_HomingLimitFallingEdge());
}

static void test_timer_wrap(void)
{
    reset(10U, AXIS_A, UINT32_MAX - 500000U);
    sample(true, 0U);
    sample(true, 999999U);
    CHECK(!UTILS_HomingLimitRisingEdge());
    sample(true, 1U);
    CHECK(UTILS_HomingLimitRisingEdge());

    reset(50000U, AXIS_Z, 123U);
    sample(true, 0U);
    for (unsigned i = 0; i < 49U; i++) {
        sample(true, 100000000U);
        CHECK(!UTILS_HomingLimitRisingEdge());
    }
    sample(true, 99999999U);
    CHECK(!UTILS_HomingLimitRisingEdge());
    sample(true, 1U);
    CHECK(UTILS_HomingLimitRisingEdge());

    reset(UINT32_MAX, AXIS_Y, 0U);
    sample(true, 0U);
    uint64_t remaining = (uint64_t)UINT32_MAX * (timer_frequency / 1000U);
    while (remaining > 1000000000U) {
        sample(true, 1000000000U);
        remaining -= 1000000000U;
    }
    sample(true, (uint32_t)remaining - 1U);
    CHECK(!UTILS_HomingLimitRisingEdge());
    sample(true, 1U);
    CHECK(UTILS_HomingLimitRisingEdge());
}

static float float_from_bits(uint32_t bits)
{
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void test_settings(void)
{
    CNC_Settings settings = {0};
    CHECK(SETTINGS_SetValue(&settings, 25U, 2000.0f));
    CHECK(SETTINGS_SetValue(&settings, 26U, 25.0f));
    CHECK(SETTINGS_GetValue(&settings, 25U) == 2000.0f);
    CHECK(SETTINGS_GetValue(&settings, 26U) == 25.0f);
    CHECK(SETTINGS_SetValue(&settings, 26U, 10.9f));
    CHECK(settings.homing_debounce == 10U);
    CHECK(SETTINGS_SetValue(&settings, 26U, 0.0f));
    CHECK(settings.homing_debounce == 0U);
    CHECK(SETTINGS_SetValue(&settings, 26U, 4294967040.0f));
    CHECK(settings.homing_debounce == UINT32_MAX - 255U);
    CHECK(SETTINGS_SetValue(&settings, 26U, 25.0f));

    static const uint32_t invalid_bits[] = {
        0xbf800000U, 0x4f800000U, 0x7f800000U, 0xff800000U, 0x7fc00000U
    };
    for (unsigned i = 0; i < sizeof(invalid_bits) / sizeof(invalid_bits[0]); i++) {
        CHECK(!SETTINGS_SetValue(&settings, 26U, float_from_bits(invalid_bits[i])));
        CHECK(settings.homing_debounce == 25U);
    }
    CHECK(!SETTINGS_SetValue(NULL, 26U, 10.0f));
}

static void test_parser(void)
{
    static const char* invalid[] = {
        "=", "= ", "=abc", "=10junk", "=10 20"
    };
    float value;
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
        char input[32];
        strcpy(input, invalid[i]);
        CHECK(!parse_value(26, input, &value));
    }
    char valid[] = "= 25.9 ";
    CHECK(parse_value(26, valid, &value));
    CHECK(value == 25.9f);
    char other_setting[] = "=2000";
    CHECK(parse_value(25, other_setting, &value));
    CHECK(value == 2000.0f);
}

static void test_limit_selection(void)
{
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        for (unsigned positive = 0; positive < 2U; positive++) {
            for (unsigned inverted = 0; inverted < 2U; inverted++) {
                reset(10U, axis, 0U);
                g_homing.current_axis = axis;
                current_settings.homing_dir_mask = (uint8_t)(positive << axis);
                current_settings.limit_pins_invert = (uint8_t)(inverted << axis);
                g_homing_settings[axis].homing_dir_mask = &current_settings.homing_dir_mask;
                min_limits[axis] = inverted != 0U;
                max_limits[axis] = inverted != 0U;
                CHECK(!HOMING_IsLimitActiveNow());
                CHECK(!HOMING_LimitTriggered());
                if (positive) {
                    min_limits[axis] = !min_limits[axis];
                } else {
                    max_limits[axis] = !max_limits[axis];
                }
                CHECK(!HOMING_IsLimitActiveNow());
                if (positive) {
                    max_limits[axis] = !max_limits[axis];
                } else {
                    min_limits[axis] = !min_limits[axis];
                }
                CHECK(HOMING_IsLimitActiveNow());
                CHECK(!HOMING_LimitTriggered());
                timer_count += 999999U;
                CHECK(!HOMING_LimitTriggered());
                timer_count++;
                CHECK(HOMING_LimitTriggered());
                CHECK(!HOMING_LimitTriggered());
            }
        }
    }
}

int main(void)
{
    test_thresholds();
    test_bounce_and_reset();
    test_zero_and_live_settings();
    test_timer_wrap();
    test_settings();
    test_parser();
    test_limit_selection();
    printf("Homing debounce: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
