#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>

#define GCODE_BUFFER_SIZE 80
#define GCODE_MAX_COMMANDS 64
#define UART_RX_CMD_MAX_BYTES 128
#define DEBUG_PRINT_GCODE(...) ((void)0)
#define DEBUG_EXEC_GCODE(...) ((void)0)
#define UART_SEND_BANNER() UART_Printf("Grbl test\r\n")
#define _T4CON_ON_MASK 1u
#define _CP0_GET_COUNT() 0u
typedef enum { AXIS_X, AXIS_Y, AXIS_Z, AXIS_A, NUM_AXIS } E_AXIS;
enum { APP_IDLE, APP_ALARM, ARC_GEN_IDLE, ARC_GEN_ACTIVE };
enum { HOMING_STATE_IDLE, HOMING_STATE_COMPLETE, HOMING_STATE_ALARM };
typedef struct { float coordinate[NUM_AXIS]; } CoordinatePoint;
typedef struct { CoordinatePoint offset; } WorkCoordinateSystem;
typedef struct { int32_t steps[NUM_AXIS]; float steps_per_mm[NUM_AXIS]; } StepperPosition;
typedef struct { char command[GCODE_BUFFER_SIZE]; } GCODE_Command;
typedef struct {
    GCODE_Command commands[GCODE_MAX_COMMANDS];
    uint32_t head, tail, count, commands_consumed;
} GCODE_CommandQueue;
typedef struct {
    unsigned state, alarmCode, arcGenState, modalPlane, activeWCS;
    uint32_t motionQueueCount, modalToolNumber, modalSpindleRPM;
    bool motionSegmentCompleted, absoluteMode, probeSuccess;
    float modalFeedrate;
    CoordinatePoint probePosition;
} APP_DATA;
enum { GCODE_EVENT_JOG };
typedef struct {
    unsigned type;
    struct {
        struct { float x, y, z, a, feedrate; bool isRapid, is_jog; } linearMove;
    } data;
} GCODE_Event;
typedef struct {
    uint8_t limit_pins_invert, status_report_mask, hard_limits_enable;
    float max_rate[NUM_AXIS];
    char startup_line[2][80];
    uint32_t checksum;
} CNC_Settings;
typedef struct {
    uint32_t ok_attempted, ok_enqueued, ok_dropped, last_ok_tx_free;
    uint32_t status_attempted, status_enqueued, status_dropped, last_status_tx_free, last_status_len;
} UART_TxStats;
/* PRODUCTION_TYPES */

static APP_DATA app;
static GCODE_CommandQueue queue;
static CNC_Settings settings;
static StepperPosition position;
static WorkCoordinateSystem fake_wcs;
static bool g_hard_limit_alarm, g_suppress_hard_limits, g_estop_pending;
static bool g_feed_hold_active, g_feed_hold_pending;
static bool limits_active, interpolator_active, homing_active, dispatch_blocked, tx_full;
static unsigned dispatches, checks, failures;
static uint32_t T4CON;
static char input[8192], output[32768], last_command[1024];
static size_t input_size, output_size;

static bool UART_Printf(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    int size = vsnprintf(output + output_size, sizeof(output) - output_size, format, args);
    va_end(args);
    if (size < 0 || (size_t)size >= sizeof(output) - output_size) abort();
    output_size += (size_t)size;
    return true;
}
static uint32_t UART3_Write(const uint8_t* bytes, uint32_t size)
{
    if (output_size + size >= sizeof(output)) abort();
    memcpy(output + output_size, bytes, size);
    output_size += size;
    output[output_size] = '\0';
    return size;
}
static bool UART_SendOK(void) { return !tx_full && UART_Printf("ok\r\n"); }
static bool UART_WriteStatusReport(const uint8_t* bytes, uint32_t size)
{
    UART3_Write(bytes, size);
    return true;
}
static uint32_t UART3_ReadCountGet(void) { return (uint32_t)input_size; }
static uint32_t UART3_ReadFreeBufferCountGet(void) { return 1023u; }
static uint32_t UART3_Read(uint8_t* bytes, uint32_t size)
{
    if (size > input_size) size = (uint32_t)input_size;
    memcpy(bytes, input, size);
    input_size -= size;
    memmove(input, input + size, input_size);
    return size;
}
static void UART_Initialize(void) {}
static void UART_GetTxStats(UART_TxStats* stats) { memset(stats, 0, sizeof(*stats)); }
static void UART_ResetTxStats(void) {}
static CNC_Settings* SETTINGS_GetCurrent(void) { return &settings; }
static bool SETTINGS_SaveToFlash(const CNC_Settings* s) { (void)s; return true; }
#define SETTINGS_CalculateCRC32(s) ((void)(s), 0u)
#define SETTINGS_SetValue(s, p, v) ((void)(s), (void)(p), (void)(v), true)
#define SETTINGS_GetValue(s, p) ((void)(s), (void)(p), 0.0f)
#define SETTINGS_RestoreDefaults(s) ((void)(s))
#define SETTINGS_PrintAll(s) ((void)(s), UART_SendOK())
#define SETTINGS_PrintBuildInfo() UART_Printf("[VER:test]\r\nok\r\n")
#define SETTINGS_GetG28Position(p) memset(p, 0, 4 * sizeof(float))
#define SETTINGS_GetG30Position(p) memset(p, 0, 4 * sizeof(float))
#define SETTINGS_GetWorkCoordinateSystem(i, x, y, z) \
    ((void)(i), *(x) = *(y) = *(z) = 0.0f, true)
#define SETTINGS_SetWorkCoordinateSystem(i, x, y, z) \
    ((void)(i), (void)(x), (void)(y), (void)(z))
#define KINEMATICS_GetG92Offset(x, y, z) (*(x) = *(y) = *(z) = 0.0f)
static float KINEMATICS_GetTLO(bool* active) { *active = false; return 0.0f; }
#define KINEMATICS_GetCurrentPosition() ((CoordinatePoint){{0}})
#define KINEMATICS_MachineToWork(p) (p)
#define KINEMATICS_GetWorkCoordinates() (&fake_wcs)
#define KINEMATICS_ClearG92Offset() ((void)0)
#define GET_COORDINATE_AXIS(p, axis) ((p)->coordinate[axis])
#define SET_COORDINATE_AXIS(p, axis, value) ((p)->coordinate[axis] = (value))
#define STEPPER_GetPosition() (&position)
#define STEPPER_GetCurrentFeedrateMmMin() 100.0f
#define STEPPER_Initialize(p) ((void)(p))
#define STEPPER_ReloadSettings() ((void)0)
#define STEPPER_StopMotion() (interpolator_active = false)
#define STEPPER_ResumeMotion() (g_feed_hold_active = false)
#define STEPPER_PauseMotion() (g_feed_hold_active = true)
#define MOTION_UTILS_CheckHardLimits(mask) ((void)(mask), limits_active)
#define LIMIT_CheckAxis(axis, mask) ((void)(axis), (void)(mask), limits_active)
#define MOTION_IsDwellActive() false
#define MOTION_IsJogging() false
#define MOTION_GetTrajectoryFree() 64u
#define MOTION_GetRapidOverridePct() 100u
#define MOTION_SetRapidOverridePct(p) ((void)(p))
#define MOTION_JogCancel() ((void)0)
#define MOTION_ProcessGcodeEvent(p, ev) ((void)(p), (void)(ev), ++dispatches, true)
#define INTERPOLATOR_IsActive() interpolator_active
#define INTERPOLATOR_GetFeedOverride() 1.0f
#define INTERPOLATOR_SetFeedOverride(p) ((void)(p))
#define SPINDLE_GetOverridePct() 100u
#define SPINDLE_SetOverridePct(p) ((void)(p))
#define HOMING_IsActive() homing_active
#define HOMING_GetState() HOMING_STATE_IDLE
#define HOMING_GetAlarmCode() 9u
#define HOMING_Abort() (homing_active = false)

static GCODE_CommandQueue* Extract_CommandLineFrom_Buffer(uint8_t* buffer, uint32_t size,
                                                         GCODE_CommandQueue* q)
{
    (void)buffer;
    (void)size;
    if (q->count < GCODE_MAX_COMMANDS) q->count++;
    return q;
}
static bool dispatch_command_line(APP_DATA* data, const uint8_t* buffer,
                                  uint32_t size, bool* is_probe)
{
    (void)data;
    *is_probe = false;
    if (dispatch_blocked) return false;
    memcpy(last_command, buffer, size);
    last_command[size] = '\0';
    dispatches++;
    return true;
}
/* PRODUCTION_SOURCE */

#define CHECK(condition) do { \
    checks++; \
    if (!(condition)) { \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
        failures++; \
    } \
} while (0)

static void pump(void)
{
    for (unsigned i = 0; i < 128; i++) GCODE_Tasks(&app, &queue);
}
static void feed(const char* bytes, size_t size)
{
    if (input_size + size > sizeof(input)) abort();
    memcpy(input + input_size, bytes, size);
    input_size += size;
    pump();
}
static void send_line(const char* bytes) { feed(bytes, strlen(bytes)); }
static void clear_output(void) { output_size = 0; output[0] = '\0'; }
static void reset(void)
{
    memset(&app, 0, sizeof(app));
    app.state = APP_IDLE;
    app.arcGenState = ARC_GEN_IDLE;
    app.absoluteMode = true;
    memset(&queue, 0, sizeof(queue));
    memset(&settings, 0, sizeof(settings));
    for (unsigned i = 0; i < NUM_AXIS; i++) {
        settings.max_rate[i] = 1000.0f;
        position.steps_per_mm[i] = 80.0f;
    }
    input_size = 0;
    dispatches = 0;
    limits_active = interpolator_active = homing_active = dispatch_blocked = tx_full = false;
    g_hard_limit_alarm = g_suppress_hard_limits = g_estop_pending = false;
    T4CON = 0;
    s_boot_inject_idx = -1;
    s_homing_pending = s_homing_started = false;
    GCODE_USART_Initialize(0);
    clear_output();
}
static unsigned count(const char* word)
{
    unsigned matches = 0;
    for (const char* p = output; (p = strstr(p, word)) != NULL; p += strlen(word)) matches++;
    return matches;
}
static void test_line_endings(void)
{
    static const char* lines[] = {"G21", "$I", "; comment", "   ", "", "X1"};
    static const char* endings[] = {"\n", "\r", "\r\n"};
    char stream[100];
    for (unsigned i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        for (unsigned j = 0; j < sizeof(endings) / sizeof(endings[0]); j++) {
            snprintf(stream, sizeof(stream), "%s%s%s%s", lines[i], endings[j], lines[i], endings[j]);
            size_t length = strlen(stream);
            for (size_t split = 0; split <= length; split++) {
                reset();
                feed(stream, split);
                feed(stream + split, length - split);
                CHECK(count("ok\r\n") == 2);
                CHECK(count("error:") == 0);
                CHECK(nBytesRead == 0 && input_size == 0);
            }
            reset();
            for (size_t k = 0; k < length; k++) feed(stream + k, 1);
            CHECK(count("ok\r\n") == 2);
        }
    }
    reset();
    send_line("G21\r\n\n$I\r\n\n; comment\r\n\n");
    CHECK(count("ok\r\n") == 6);
    reset();
    send_line("G21\r");
    send_line("?");
    send_line("\n");
    CHECK(count("ok\r\n") == 1);
    CHECK(count("<Idle|") == 1);

    reset();
    tx_full = true;
    send_line("G21\r\n");
    CHECK(count("ok\r\n") == 0);
    tx_full = false;
    pump();
    CHECK(count("ok\r\n") == 1);

    reset();
    dispatch_blocked = true;
    send_line("G1 X1\r");
    send_line("\nG21\r\n");
    CHECK(dispatches == 0 && count("ok\r\n") == 0);
    dispatch_blocked = false;
    pump();
    CHECK(dispatches == 2 && count("ok\r\n") == 2);
    CHECK(strcmp(last_command, "G21") == 0);
}
static void test_alarms(void)
{
    static const unsigned codes[] = {1, 2, 3, 5, 6, 8, 9, 10};
    for (unsigned i = 0; i < sizeof(codes) / sizeof(codes[0]); i++) {
        reset();
        app.state = APP_ALARM;
        app.alarmCode = codes[i];
        send_line("?");
        CHECK(count("<Alarm|") == 1);
        clear_output();
        send_line("G1 X1\r\nX2\r\n$J=G91 X1 F100\r\n");
        CHECK(count("error:9\r\n") == 3);
        CHECK(count("ok\r\n") == 0 && dispatches == 0 && queue.count == 0);
        clear_output();
        send_line("$I\n");
        CHECK(count("[VER:test]") == 1 && count("ok\r\n") == 1);
        clear_output();
        send_line("$X\n?");
        CHECK(count("ok\r\n") == 1 && count("<Idle|") == 1);
        CHECK(app.state == APP_IDLE && app.alarmCode == 0);
        send_line("G1 X1\n");
        CHECK(dispatches == 1);
    }
    reset();
    g_hard_limit_alarm = true;
    send_line("?");
    CHECK(count("<Alarm|") == 1);
    reset();
    limits_active = true;
    send_line("$X\r");
    send_line("\n");
    CHECK(count("Cannot clear alarm") == 1);
    CHECK(count("error:9\r\n") == 1 && count("ok\r\n") == 0);
    send_line("G1 X1\n$J=G91 X1 F100\n?");
    CHECK(dispatches == 0 && count("error:9\r\n") == 3);
    CHECK(count("<Alarm|") == 1);
    limits_active = false;
    send_line("$X\n");
    CHECK(!GCODE_HasAlarm(&app));

    reset();
    interpolator_active = true;
    send_line("\x18");
    CHECK(app.state == APP_ALARM && app.alarmCode == 3);
    send_line("?");
    CHECK(count("<Alarm|") == 1);
    clear_output();
    send_line("G1 X1\r\n");
    CHECK(count("error:9\r\n") == 1 && dispatches == 0);
    send_line("$X\nG1 X1\n");
    CHECK(app.state == APP_IDLE && dispatches == 1);

    reset();
    homing_active = true;
    send_line("\x18");
    CHECK(app.state == APP_ALARM && app.alarmCode == 6);
    send_line("?");
    CHECK(count("<Alarm|") == 1);

    reset();
    send_line("G21\r");
    send_line("\x18");
    clear_output();
    send_line("\n");
    CHECK(count("ok\r\n") == 1);

    reset();
    dispatch_blocked = true;
    send_line("G1 X1\n");
    app.state = APP_ALARM;
    pump();
    CHECK(count("error:9\r\n") == 1 && dispatches == 0);
}
int main(void)
{
    CHECK(s_canned_g98);
    test_line_endings();
    test_alarms();
    printf("G-code protocol: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
