/*
 * GRBL-like G-code system command and real-time handling for UGS compatibility.
 * Real-time chars: ?, ~, !, 0x18 (Ctrl+X soft reset)
 * System commands: $$, $#, $G, $I, $N / $Nn=, $C, $X, $RST=*, $RST=$, $RST=#, $F, single $
 * Generic settings read/write: $<number>, $<number>=<value>
 * G-code queue + event translation (subset)
 *
 * Soft reset handling (Ctrl+X and literal "0x18") uses UART_SoftReset() (uart_utils.c).
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>

#include "gcode_parser.h"
#include "common.h"
#include "stepper.h"
#include "motion.h"
#include "kinematics.h"
#include "motion/homing.h"
#include "utils.h"
#include "settings.h"
#include "data_structures.h"
#include "utils/uart_utils.h"
#include "../config/default/peripheral/uart/plib_uart3.h"
#include "definitions.h"                  // For T4CON/_T4CON_ON_MASK (hardware state)
#include "../config/default/peripheral/gpio/plib_gpio.h"  // For LED debug

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */
#define ENABLE_STARTUP_BANNER     // Enable/disable startup banner
                                  // G-code senders (UGS, bCNC) expect this banner
                                  // Comment out to disable for debugging or custom ID

// Banner message configured in common.h via STARTUP_BANNER_STRING macro

// Dual-threshold flow control for UGS compatibility
// HIGH_WATER: Start deferring "ok" when queue reaches this (almost full)
// LOW_WATER: Resume sending "ok" when queue drains to this (some space freed)
// With MAX=16: Defer at 14 (2 free), Resume at 12 (4 drained)
#define MOTION_BUFFER_HIGH_WATER 2    // Defer when (MAX - 2) = 14 segments used
#define MOTION_BUFFER_LOW_WATER  4    // Resume when (MAX - 4) = 12 segments used

/* -------------------------------------------------------------------------- */
/* Static Buffers / State                                                     */
/* -------------------------------------------------------------------------- */
static uint8_t txBuffer[1024];
static uint8_t rxBuffer[512];  // Increased to match UART3 RX buffer size
static volatile uint32_t nBytesRead = 0;

GCODE_Data gcodeData = {
    .state = GCODE_STATE_IDLE,
};

static uint32_t okPendingCount = 0;         // Flow control: count of deferred "ok" responses
static bool grblCheckMode = false;          /* $C toggle */
static bool grblAlarm = false;              /* $X clears alarm */
static bool feedHoldActive = false;         /* '!' feed hold, '~' resume */
static char startupLines[2][GCODE_BUFFER_SIZE] = {{0},{0}}; /* $N0 / $N1 */
static bool unitsInches = false;            /* false=mm (G21), true=inches (G20) */

/* ✅ REMOVED: Startup deferral variables (caused UGS stalling) */

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */
static inline bool is_control_char(uint8_t c){ return (c == '?' || c == '~' || c == '!' || c == 0x18); }
GCODE_CommandQueue* Extract_CommandLineFrom_Buffer(uint8_t* buffer, uint32_t length, GCODE_CommandQueue* commandQueue);


static inline char* find_char(char* s, char key){
    while (*s){
        if (*s == key) return s;
        s++;
    }
    return NULL;
}
static float parse_float_after(char* start){
    if (!start || !start[1]) return 0.0f;
    return (float)strtof(start + 1, NULL);
}

/* -------------------------------------------------------------------------- */
/* Centralized Soft Reset (PUBLIC FUNCTION)                                  */
/* -------------------------------------------------------------------------- */
void GCODE_SoftReset(APP_DATA* appData, GCODE_CommandQueue* cmdQueue)
{
    if (appData == NULL || cmdQueue == NULL) {
        return;
    }

    /* 1. Stop all motion immediately using centralized function */
    STEPPER_StopMotion();  // Disables steppers, stops TMR4, disables OC1
    
    // Abort homing if in progress
    HOMING_Abort();

    /* 2. Flush any pending RX bytes to avoid processing pre-reset junk */
    uint8_t scratch[64];
    uint32_t rc;
    while ((rc = UART3_ReadCountGet()) > 0U) {
        uint32_t toRead = (rc > sizeof(scratch)) ? (uint32_t)sizeof(scratch) : rc;
        (void)UART3_Read(scratch, toRead);
    }

    /* 3. Clear motion planner queue (planner and executor state) */
    appData->motionQueueHead = 0;
    appData->motionQueueTail = 0;
    appData->motionQueueCount = 0;
    appData->currentSegment   = NULL;

    /* 4. Modal state to GRBL defaults: G17, G21, G90, G94, M5, M9, T0, F0, S0 */
    appData->modalPlane       = 0;        /* 0->G17 (XY) as used by $G print */
    appData->absoluteMode     = true;     /* G90 */
    appData->modalFeedrate    = 0.0f;     /* F */
    appData->modalSpindleRPM  = 0;        /* S */
    appData->modalToolNumber  = 0;        /* T0 */

    /* 5. Clear alarm state (soft reset should clear alarms) */
    extern volatile bool g_hard_limit_alarm;
    extern volatile bool g_suppress_hard_limits;  // Suppress until limits physically clear
    g_hard_limit_alarm = false;
    g_suppress_hard_limits = true;  // Ignore hard limits until they physically clear
    appData->alarmCode = 0;
    grblAlarm = false;
    
    /* 6. Application state back to IDLE */
    appData->state = APP_IDLE;

    /* 7. Reset G-code command queue */
    cmdQueue->head  = 0;
    cmdQueue->tail  = 0;
    cmdQueue->count = 0;
    /* ✅ No sync needed - flow control reads appData->motionQueueCount directly */

    /* 8. Reset G-code parser state */
    okPendingCount = 0;  // Clear all deferred ok responses
    grblCheckMode = false;
    feedHoldActive = false;
    unitsInches = false;
    gcodeData.state = GCODE_STATE_IDLE;

    // ✅ REMOVED: Startup deferral reset (no longer used)

    // Motion fully idle after reset
    appData->motionActive = false;

    nBytesRead = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));

    /* 9. Mark steppers to be re-enabled automatically on first motion command */
    // stepperEnablePending = true;

    /* 10. Clear G92 temporary offset (DO NOT save to flash on soft reset) */
    // ✅ REMOVED: Flash save on soft reset causes power-up hangs
    // Problem: User power-cycles immediately after Ctrl+X banner prints,
    //          but flash hasn't fully stabilized from write operation
    // Solution: Only save settings when user explicitly changes them ($n=value)
    //          G92 offset cleared in RAM, not persisted to flash
    CNC_Settings* settings = SETTINGS_GetCurrent();
    if (settings != NULL) {
        SETTINGS_SetG92Offset(0.0f, 0.0f, 0.0f);  // Reset G92 to zero (in RAM only)
        DEBUG_PRINT_GCODE("[GCODE] G92 offset cleared during soft reset (not saved to flash)\r\n");
    }

    /* 11. Print GRBL startup banner (expected by senders after Ctrl+X) */
#ifdef ENABLE_STARTUP_BANNER
    UART_SEND_BANNER();  // Compile-time string and length from common.h
#endif
}

/* -------------------------------------------------------------------------- */
/* USART Initialization                                                       */
/* -------------------------------------------------------------------------- */
void GCODE_USART_Initialize(uint32_t RD_thresholds)
{
    (void)RD_thresholds;
    UART_Initialize();
    
    /* Print GRBL startup banner (disable for custom firmware identification) */
#ifdef ENABLE_STARTUP_BANNER
    UART_SEND_BANNER();  // Compile-time string and length from common.h
#endif

    nBytesRead = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));
    okPendingCount = 0;  // Clear all deferred ok responses
    gcodeData.state = GCODE_STATE_IDLE;
    unitsInches = false;
    grblCheckMode = false;
    grblAlarm = false;
    feedHoldActive = false;

    // ✅ REMOVED: Startup deferral initialization (no longer used)
}

/* -------------------------------------------------------------------------- */
/* Helper: queue command (assumes space available already checked)           */
/* -------------------------------------------------------------------------- */
static inline void queue_command(GCODE_CommandQueue* q, const char* src, size_t len){
    if (len == 0 || len >= GCODE_BUFFER_SIZE) return;
    memcpy(q->commands[q->head].command, src, len);
    q->commands[q->head].command[len] = '\0';
    q->head = (q->head + 1) % GCODE_MAX_COMMANDS;
    q->count++;
}

/* -------------------------------------------------------------------------- */
/* Helper: split combined modal tokens like "G21G90" or "G90G0Z5"             */
/* Rules:
 *  - Multiple G/M words may be concatenated.
 *  - Parameters belong to the final word only (e.g. G90G0X1 -> "G90", "G0X1")
 *  - Return number of pieces queued.
 * -------------------------------------------------------------------------- */
static uint32_t split_and_queue_multi_modal(GCODE_CommandQueue* q, char* token)
{
    uint32_t pieces = 0;
    size_t len = strlen(token);
    if (len < 3) { /* Too short to contain combined words */
        queue_command(q, token, len);
        return 1;
    }

    /* Collect positions of G/M designators */
    uint8_t pos[8];
    uint8_t posCount = 0;
    for (size_t i = 0; i < len && posCount < (sizeof(pos)/sizeof(pos[0])); i++){
        char c = token[i];
        if (c == 'G' || c == 'M'){
            pos[posCount++] = (uint8_t)i;
        }
    }

    if (posCount <= 1){
        queue_command(q, token, len);
        return 1;
    }

    /* For each modal except last, emit substring up to next modal start */
    for (uint8_t i = 0; i < posCount - 1; i++){
        uint8_t start = pos[i];
        uint8_t end   = pos[i+1];
        if (end > start){
            queue_command(q, &token[start], (size_t)(end - start));
            pieces++;
        }
    }
    /* Last piece: from last modal start to end of original token */
    queue_command(q, &token[pos[posCount-1]], len - pos[posCount-1]);
    pieces++;
    return pieces;
}

/* -------------------------------------------------------------------------- */
/* Extract line and tokenize                                                  */
/* -------------------------------------------------------------------------- */
GCODE_CommandQueue* Extract_CommandLineFrom_Buffer(uint8_t* buffer, uint32_t length, GCODE_CommandQueue* commandQueue)
{
    GCODE_CommandQueue* cmdQueue = commandQueue;
    char line_buffer[256];
    uint32_t safe_length = (length < sizeof(line_buffer) - 1U) ? length : (sizeof(line_buffer) - 1U);
    memcpy(line_buffer, buffer, safe_length);
    line_buffer[safe_length] = '\0';

    for (uint32_t i = 0; i < safe_length; i++) {
        char c = line_buffer[i];
        if (c >= 'a' && c <= 'z') line_buffer[i] = (char)(c - 'a' + 'A');
    }

    uint32_t write_pos = 0;
    for (uint32_t read_pos = 0; read_pos < safe_length; read_pos++) {
        char c = line_buffer[read_pos];
        if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '\t')
            line_buffer[write_pos++] = c;
    }
    line_buffer[write_pos] = '\0';
    safe_length = write_pos;

    TokenArray tokens;
    uint32_t token_count = UTILS_TokenizeGcodeLine(line_buffer, &tokens);

    for (uint32_t i = 0; i < token_count; i++) {
        if (UTILS_IsEmptyString(tokens.tokens[i]) || UTILS_IsComment(tokens.tokens[i])) continue;
        if (((cmdQueue->head + 1) % GCODE_MAX_COMMANDS) == cmdQueue->tail) break; /* queue full */

        /* Detect combined modal sequences (must start with G/M and contain another G/M) */
        bool combined = false;
        if (tokens.tokens[i][0] == 'G' || tokens.tokens[i][0] == 'M'){
            for (char* p = tokens.tokens[i] + 1; *p; ++p){
                if (*p == 'G' || *p == 'M'){
                    combined = true;
                    break;
                }
            }
        }

        if (combined){
            split_and_queue_multi_modal(cmdQueue, tokens.tokens[i]);
        } else {
            size_t tlen = UTILS_SafeStrlen(tokens.tokens[i], GCODE_BUFFER_SIZE - 1U);
            queue_command(cmdQueue, tokens.tokens[i], tlen);
        }
    }
    return cmdQueue;
}

/* -------------------------------------------------------------------------- */
/* Command -> Event Parser                                                    */
/* -------------------------------------------------------------------------- */
static bool parse_command_to_event(const char* cmd, GCODE_Event* ev)
{
    if (!cmd || !ev) return false;
    ev->type = GCODE_EVENT_NONE;

    // Axis letter array for scalable parsing (shared across all parsing blocks)
    static const char axis_letters[NUM_AXIS] = {'X', 'Y', 'Z', 'A'};

    DEBUG_PRINT_GCODE("[PARSE] cmd='%s'\r\n", cmd);

    if (cmd[0] == 'G') {
        char* pend = NULL;
        int gnum = (int)strtol(&cmd[1], &pend, 10);
        
        DEBUG_PRINT_GCODE("[PARSE] G-code detected: gnum=%d\r\n", gnum);

        // G20/G21 - Units (handled internally, no event needed)
        if (gnum == 20) { unitsInches = true; ev->type = GCODE_EVENT_NONE; return true; }
        if (gnum == 21) { unitsInches = false; ev->type = GCODE_EVENT_NONE; return true; }
        
        // G90/G91 - Positioning mode
        if (gnum == 90) { ev->type = GCODE_EVENT_SET_ABSOLUTE; return true; }
        if (gnum == 91) { ev->type = GCODE_EVENT_SET_RELATIVE; return true; }

        // Work coordinate system selection (G54-G59)
        if (gnum >= 54 && gnum <= 59) {
            ev->type = GCODE_EVENT_SET_WCS;
            ev->data.setWCS.wcs_number = gnum - 54;  // G54=0, G55=1, ..., G59=5
            return true;
        }

        // G92 - Set work coordinate offset (same as G10 L20 P0)
        if (gnum == 92) {
            StepperPosition* pos = STEPPER_GetPosition();
            WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();
            const float unit_scale = unitsInches ? 25.4f : 1.0f;
            
            // Array-based axis parameter parsing with loop
            float desired[NUM_AXIS];
            float mpos[NUM_AXIS];
            
            // Parse all axis parameters using loop
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                char* pAxis = find_char((char*)cmd, axis_letters[axis]);
                desired[axis] = pAxis ? (parse_float_after(pAxis) * unit_scale) : NAN;
                mpos[axis] = (float)pos->steps[axis] / pos->steps_per_mm[axis];
            }
            
            // Set offset = MachinePos - DesiredWorkPos (only X, Y, Z supported in WCS)
            for (E_AXIS axis = AXIS_X; axis < AXIS_Z + 1; axis++) {
                if (!isnan(desired[axis])) {
                    SET_COORDINATE_AXIS(&wcs->offset, axis, mpos[axis] - desired[axis]);
                }
            }
            
            return true;
        }

        if (gnum == 10) {
            char* pP = find_char((char*)cmd, 'P');
            char* pL = find_char((char*)cmd, 'L');
            int p_val = pP ? (int)strtol(pP + 1, NULL, 10) : 0;
            int l_val = pL ? (int)strtol(pL + 1, NULL, 10) : 0;
            if (l_val == 20) {
                if (p_val == 0 || p_val == 1) {
                    StepperPosition* pos = STEPPER_GetPosition();
                    WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();
                    const float unit_scale = unitsInches ? 25.4f : 1.0f;
                    
                    // Array-based axis parameter parsing with loop
                    float desired[NUM_AXIS];
                    float mpos[NUM_AXIS];
                    
                    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                        char* pAxis = find_char((char*)cmd, axis_letters[axis]);
                        desired[axis] = pAxis ? (parse_float_after(pAxis) * unit_scale) : NAN;
                        mpos[axis] = (float)pos->steps[axis] / pos->steps_per_mm[axis];
                    }
                    
                    // Set WCS offset (only X, Y, Z supported)
                    for (E_AXIS axis = AXIS_X; axis < AXIS_Z + 1; axis++) {
                        if (!isnan(desired[axis])) {
                            SET_COORDINATE_AXIS(&wcs->offset, axis, mpos[axis] - desired[axis]);
                        }
                    }
                    
                    return true;
                } else {
                    return true;
                }
            } else {
                return true;
            }
        }

        if (gnum == 0 || gnum == 1) {
            ev->type = GCODE_EVENT_LINEAR_MOVE;
            DEBUG_PRINT_GCODE("[PARSE] Linear move (G%d)\r\n", gnum);
        } else if (gnum == 2) {
            ev->type = GCODE_EVENT_ARC_MOVE;
            ev->data.arcMove.clockwise = true;
            DEBUG_PRINT_GCODE("[PARSE] Arc CW (G2)\r\n");
        } else if (gnum == 3) {
            ev->type = GCODE_EVENT_ARC_MOVE;
            ev->data.arcMove.clockwise = false;
            DEBUG_PRINT_GCODE("[PARSE] Arc CCW (G3)\r\n");
        } else if (gnum == 4) {
            ev->type = GCODE_EVENT_DWELL;
            char* pP = strstr((char*)cmd, "P");
            ev->data.dwell.seconds = pP ? parse_float_after(pP) : 0.0f;
            return true;
        } else {
            return false;
        }

        // Array-based axis parameter parsing with loop
        char* pAxis[NUM_AXIS];
        float axis_values[NUM_AXIS];
        
        for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
            pAxis[axis] = find_char((char*)cmd, axis_letters[axis]);
            axis_values[axis] = pAxis[axis] ? parse_float_after(pAxis[axis]) : NAN;
        }
        
        char* pF = find_char((char*)cmd, 'F');
        const float unit_scale = unitsInches ? 25.4f : 1.0f;

        if (ev->type == GCODE_EVENT_LINEAR_MOVE) {
            float f = pF ? parse_float_after(pF) : 0.0f;
            // Event structure still uses hardcoded members (future refactoring opportunity)
            ev->data.linearMove.x = !isnan(axis_values[AXIS_X]) ? axis_values[AXIS_X] * unit_scale : NAN;
            ev->data.linearMove.y = !isnan(axis_values[AXIS_Y]) ? axis_values[AXIS_Y] * unit_scale : NAN;
            ev->data.linearMove.z = !isnan(axis_values[AXIS_Z]) ? axis_values[AXIS_Z] * unit_scale : NAN;
            ev->data.linearMove.a = !isnan(axis_values[AXIS_A]) ? axis_values[AXIS_A] * unit_scale : NAN;
            ev->data.linearMove.feedrate = (f > 0.0f) ? (f * unit_scale) : 0.0f;
            return true;
        } else if (ev->type == GCODE_EVENT_ARC_MOVE) {
            char* pI = find_char((char*)cmd, 'I');
            char* pJ = find_char((char*)cmd, 'J');
            float i = pI ? parse_float_after(pI) : 0.0f;
            float j = pJ ? parse_float_after(pJ) : 0.0f;
            float f = pF ? parse_float_after(pF) : 0.0f;
            // Event structure still uses hardcoded members (future refactoring opportunity)
            ev->data.arcMove.x = !isnan(axis_values[AXIS_X]) ? axis_values[AXIS_X] * unit_scale : NAN;
            ev->data.arcMove.y = !isnan(axis_values[AXIS_Y]) ? axis_values[AXIS_Y] * unit_scale : NAN;
            ev->data.arcMove.z = !isnan(axis_values[AXIS_Z]) ? axis_values[AXIS_Z] * unit_scale : NAN;
            ev->data.arcMove.a = !isnan(axis_values[AXIS_A]) ? axis_values[AXIS_A] * unit_scale : NAN;
            ev->data.arcMove.centerX = i * unit_scale;
            ev->data.arcMove.centerY = j * unit_scale;
            ev->data.arcMove.feedrate = (f > 0.0f) ? (f * unit_scale) : 0.0f;
            DEBUG_PRINT_GCODE("[PARSE] Arc params: X=%.2f Y=%.2f I=%.2f J=%.2f F=%.1f\r\n",
                             ev->data.arcMove.x, ev->data.arcMove.y, 
                             ev->data.arcMove.centerX, ev->data.arcMove.centerY, 
                             ev->data.arcMove.feedrate);
            return true;
        }
    }

    if (cmd[0] == 'F') {
        ev->type = GCODE_EVENT_SET_FEEDRATE;
        float f = (float)strtof(&cmd[1], NULL);
        ev->data.setFeedrate.feedrate = unitsInches ? (f * 25.4f) : f;
        return true;
    }
    if (cmd[0] == 'S') {
        ev->type = GCODE_EVENT_SET_SPINDLE_SPEED;
        ev->data.setSpindleSpeed.rpm = (uint32_t)strtoul(&cmd[1], NULL, 10);
        return true;
    }
    if (cmd[0] == 'T') {
        ev->type = GCODE_EVENT_SET_TOOL;
        ev->data.setTool.toolNumber = (uint32_t)strtoul(&cmd[1], NULL, 10);
        return true;
    }

    if (cmd[0] == 'M') {
        char* pend = NULL;
        int mnum = (int)strtol(&cmd[1], &pend, 10);
        if (mnum == 3) {
            ev->type = GCODE_EVENT_SPINDLE_ON;
            char* pS = find_char((char*)cmd, 'S');
            ev->data.spindle.rpm = pS ? (uint32_t)strtoul(pS + 1, NULL, 10) : 1000;
            return true;
        }
        if (mnum == 5) { ev->type = GCODE_EVENT_SPINDLE_OFF; return true; }
        if (mnum == 7) { ev->type = GCODE_EVENT_COOLANT_ON; return true; }
        if (mnum == 9) { ev->type = GCODE_EVENT_COOLANT_OFF; return true; }
    }
    
    // System commands ($H, $X, etc.)
    if (cmd[0] == '$') {
        if (cmd[1] == 'H' && (cmd[2] == '\0' || cmd[2] == '\n' || cmd[2] == '\r')) {
            // $H - Home all axes command
            ev->type = GCODE_EVENT_HOMING;
            ev->data.homing.axes_mask = 0x0F; // Home all axes (XYZA = bits 0-3)
            return true;
        }
    }
    
    return false;
}

/* -------------------------------------------------------------------------- */
/* Public Event Retrieval                                                     */
/* -------------------------------------------------------------------------- */
bool GCODE_GetNextEvent(GCODE_CommandQueue* cmdQueue, GCODE_Event* event)
{
    if (!cmdQueue || !event) return false;
    if (cmdQueue->count == 0) return false;

    GCODE_Command* gc = &cmdQueue->commands[cmdQueue->tail];
    if (gc->command[0] == '\0') {
        // Empty command - consume it and return false
        cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
        cmdQueue->count--;
        return false;
    }

    if (!parse_command_to_event(gc->command, event)) {
        // Parse failed - consume command and return false
        DEBUG_PRINT_GCODE("[GetEvent] Parse failed for: '%s'\r\n", gc->command);
        cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
        cmdQueue->count--;
        return false;
    }

    // ✅ SUCCESS: Event parsed, but DON'T consume yet!
    // Event will be consumed after successful processing via GCODE_ConsumeEvent()
    DEBUG_PRINT_GCODE("[GetEvent] Event ready: type=%d from cmd='%s'\r\n", event->type, gc->command);
    return true;
}

/**
 * @brief Consume the current event from the queue after successful processing
 * 
 * CRITICAL: Only call this AFTER successfully processing the event from GCODE_GetNextEvent!
 */
void GCODE_ConsumeEvent(GCODE_CommandQueue* cmdQueue)
{
    if (!cmdQueue) return;
    if (cmdQueue->count == 0) return;
    
    // Remove the event that was just processed
    cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
    cmdQueue->count--;
}

/* -------------------------------------------------------------------------- */
/* Flow Control Helpers - Centralized OK Management                           */
/* -------------------------------------------------------------------------- */

/* Removed IsOkPending/SetOkPending/ClearOkPending helpers to avoid unused warnings;
   logic now accesses okPending directly inside the flow-control helpers below. */

/**
 * @brief Compute high-water mark for flow control (when to START deferring ok)
 * 
 * Returns maxSegments - HIGH_WATER threshold
 * Example: With MAX=16, HIGH_WATER=2 → defer when queue >= 14
 */
static inline uint32_t Flow_HighWater(const GCODE_CommandQueue* q)
{
    if (q == NULL) return 0U;
    return (q->maxMotionSegments > MOTION_BUFFER_HIGH_WATER)
         ? (uint32_t)(q->maxMotionSegments - MOTION_BUFFER_HIGH_WATER)
         : 0U;
}

/**
 * @brief Compute low-water mark for flow control (when to RESUME sending ok)
 * 
 * Returns maxSegments - LOW_WATER threshold
 * Example: With MAX=16, LOW_WATER=4 → resume when queue <= 12
 */
static inline uint32_t Flow_LowWater(const GCODE_CommandQueue* q)
{
    if (q == NULL) return 0U;
    return (q->maxMotionSegments > MOTION_BUFFER_LOW_WATER)
         ? (uint32_t)(q->maxMotionSegments - MOTION_BUFFER_LOW_WATER)
         : 0U;
}

/**
 * @brief Check if we should send a deferred "ok" response
 * 
 * With aggressive flow control, only send deferred "ok" when motion queue
 * is COMPLETELY empty. This ensures UGS doesn't see "Finished" until all
 * motion has executed.
 * 
 * @param appData Application data with motion queue count
 * @param q Command queue (unused with current logic)
 */
void GCODE_CheckDeferredOk(APP_DATA* appData, GCODE_CommandQueue* q) {
    // Send deferred "ok" responses when buffer drops below low-water mark
    // This prevents deadlock on long motion files while still providing backpressure
    uint32_t lowWater = appData->gcodeCommandQueue.maxMotionSegments - MOTION_BUFFER_LOW_WATER;
    
    while (okPendingCount > 0 && appData->motionQueueCount < lowWater) {
        DEBUG_PRINT_GCODE("[DEFERRED] Sending deferred ok (queue=%lu < lowWater=%lu, pending=%lu)\r\n",
                          (unsigned long)appData->motionQueueCount, (unsigned long)lowWater,
                          (unsigned long)okPendingCount);
        if (UART_SendOK()) {
            okPendingCount--;
        } else {
            break;  // TX buffer full, try again next iteration
        }
    }
}

/**
 * @brief Decide whether to send "ok" immediately or defer it
 * 
 * Called when a command is received.
 * Uses HIGH_WATER threshold - defers ok when buffer is almost full.
 * 
 * @param currentQueueCount Current motion queue count
 * @param maxSegments Maximum motion queue size
 */
static void SendOrDeferOk(APP_DATA* appData, GCODE_CommandQueue* q)
{
    // ✅ Aggressive flow control to prevent UGS "Finished" before motion completes
    // Only send "ok" immediately when queue is completely empty
    // This ensures the last "ok" is sent only after all motion completes
    
    if (appData->motionQueueCount == 0) {
        DEBUG_PRINT_GCODE("[FLOW] Sending immediate ok (queue empty, motion complete)\r\n");
        (void)UART_SendOK();
    } else {
        DEBUG_PRINT_GCODE("[FLOW] Deferring ok (queue=%lu > 0, total pending=%lu)\r\n", 
                          (unsigned long)appData->motionQueueCount,
                          (unsigned long)(okPendingCount + 1));
        okPendingCount++;
    }
}

/* -------------------------------------------------------------------------- */
/* Main G-code / Protocol Tasks                                               */
/* -------------------------------------------------------------------------- */
void GCODE_Tasks(APP_DATA* appData, GCODE_CommandQueue* commandQueue)
{
    static uint32_t call_counter = 0;
    call_counter++;
    
    // if ((call_counter % 10000) == 0) {
    //     DEBUG_PRINT_GCODE("[GCODE_Tasks] Called %lu times, state=%d\r\n", 
    //                      call_counter, gcodeData.state);
    // }
    
    GCODE_CommandQueue* cmdQueue = commandQueue;
    uint32_t nBytesAvailable = 0;

    // ✅ CRITICAL: Check deferred "ok" ONCE at entry using FRESH count
    // If buffer drained and okPending, send "ok" immediately then continue processing
    GCODE_CheckDeferredOk(appData, cmdQueue);

    switch (gcodeData.state)
    {
    case GCODE_STATE_IDLE:
        nBytesAvailable = UART3_ReadCountGet();
        if (nBytesAvailable > 0) {
            uint32_t space = (uint32_t)(sizeof(rxBuffer) - 1U - nBytesRead);
            if (space > 0U) {
                uint32_t toRead = (nBytesAvailable < space) ? nBytesAvailable : space;
                uint32_t got = UART3_Read(&rxBuffer[nBytesRead], toRead);
                nBytesRead += got;
                rxBuffer[nBytesRead] = '\0';
            }
        } else {
            // ✅ No new bytes arriving - check for deferred ok
            // This runs every iteration when idle, ensuring final "ok" is sent
            GCODE_CheckDeferredOk(appData, cmdQueue);
        }

        if (nBytesRead == 0) {
            break;  // No data in buffer, exit IDLE state
        }

        /* Literal "0x18" typed by user */
        if (nBytesRead >= 4 &&
            rxBuffer[0] == '0' && rxBuffer[1] == 'x' &&
            rxBuffer[2] == '1' && rxBuffer[3] == '8') {
            GCODE_SoftReset(appData, cmdQueue);
            break;
        }

        if (is_control_char(rxBuffer[0])) {
            gcodeData.state = GCODE_STATE_CONTROL_CHAR;
        } else {
            bool has_terminator = false;
            uint32_t terminator_pos = 0;
            for (uint32_t i = 0; i < nBytesRead; i++) {
                if (rxBuffer[i] == '\n' || rxBuffer[i] == '\r') {
                    has_terminator = true;
                    terminator_pos = i;
                    break;
                }
            }
            if (!has_terminator) break;
            rxBuffer[terminator_pos] = '\0';

            // ✅ Check for control chars AFTER termination (handles '?' with newline)
            if (is_control_char(rxBuffer[0])) {
                gcodeData.state = GCODE_STATE_CONTROL_CHAR;
            } else if (rxBuffer[0] == '$') {
                gcodeData.state = GCODE_STATE_QUERY_CHARS;
            } else if (rxBuffer[0] == 'G' || rxBuffer[0] == 'g' ||
                       rxBuffer[0] == 'M' || rxBuffer[0] == 'm' ||
                       rxBuffer[0] == 'F' || rxBuffer[0] == 'f' ||
                       rxBuffer[0] == 'T' || rxBuffer[0] == 't' ||
                       rxBuffer[0] == 'S' || rxBuffer[0] == 's') {
                gcodeData.state = GCODE_STATE_GCODE_COMMAND;
            } else {
                // Determine if line contains G-code content (ignore pure comments / whitespace)
                bool has_content = false;
                // Find first non-whitespace character
                uint32_t first_idx = 0;
                while (first_idx < terminator_pos) {
                    char c = rxBuffer[first_idx];
                    if (c == ' ' || c == '\t') { first_idx++; continue; }
                    break;
                }
                if (first_idx < terminator_pos) {
                    char first = rxBuffer[first_idx];
                    if (first == ';' || first == '(') {
                        // Comment-only line → treat as blank, do not queue
                        has_content = false;
                    } else {
                        // Any other non-whitespace, non-comment first char counts as content
                        has_content = true;
                    }
                }
                if (has_content) {
                    DEBUG_PRINT_GCODE("[IDLE] Processing line: '%s'\r\n", rxBuffer);
                    cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, terminator_pos, cmdQueue);
                    
                    // ✅ Send "ok" for all lines with content
                    DEBUG_PRINT_GCODE("[IDLE] Sending ok for content line\r\n");
                    SendOrDeferOk(appData, cmdQueue);
                } else {
                    // ✅ GRBL v1.1 behavior: Blank lines get "ok" response
                    // UGS counts ALL lines including blanks, so we must respond
                    // Flow control applies - will defer if motion queue has content
                    DEBUG_PRINT_GCODE("[IDLE] Blank/comment line - sending ok with flow control\r\n");
                    SendOrDeferOk(appData, cmdQueue);
                }
                uint32_t skip_pos = terminator_pos + 1;
                while (skip_pos < nBytesRead && (rxBuffer[skip_pos] == '\r' || rxBuffer[skip_pos] == '\n'))
                    skip_pos++;
                uint32_t remaining_bytes = nBytesRead - skip_pos;
                if (remaining_bytes > 0) {
                    memmove(rxBuffer, &rxBuffer[skip_pos], remaining_bytes);
                    nBytesRead = remaining_bytes;
                    rxBuffer[nBytesRead] = '\0';
                } else {
                    nBytesRead = 0;
                    memset(rxBuffer, 0, sizeof(rxBuffer));
                }
                // After consuming a line, attempt to release any deferred ok now that we may have headroom
                GCODE_CheckDeferredOk(appData, cmdQueue);
                break;
            }
        }
        /* fall-through */

    case GCODE_STATE_CONTROL_CHAR:
    {
        if (gcodeData.state != GCODE_STATE_CONTROL_CHAR) break;

        uint8_t c = rxBuffer[0];
        switch (c) {
            case '?': {
                StepperPosition* pos = STEPPER_GetPosition();
                WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();

                // Array-based position calculation with loop for scalability
                float mpos[NUM_AXIS];
                float wpos[NUM_AXIS];
                
                for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                    mpos[axis] = (float)pos->steps[axis] / pos->steps_per_mm[axis];
                    wpos[axis] = mpos[axis] - GET_COORDINATE_AXIS(&wcs->offset, axis);
                }

                const char* state = "Idle";
                if (grblAlarm) {
                    state = "Alarm";
                } else if (feedHoldActive) {
                    state = "Hold";
                } else if (appData->arcGenState == ARC_GEN_ACTIVE) {
                    // Arc generator active → still processing arc segments
                    state = "Run";
                } else if ((T4CON & _T4CON_ON_MASK) != 0) {
                    // Hardware timer running → motion in progress (or about to start)
                    state = "Run";
                } else if (appData->currentSegment != NULL &&
                           appData->currentSegment->steps_completed < appData->currentSegment->steps_remaining) {
                    state = "Run";
                } else if (appData->motionQueueCount > 0) {
                    state = "Run";
                }

                float feedrate_mm_min = (state[0] == 'R') ? appData->modalFeedrate : 0.0f;
                if (state[0] == 'R' && feedrate_mm_min <= 0.0f) {
                    feedrate_mm_min = (appData->modalFeedrate > 0.0f) ? appData->modalFeedrate : 600.0f;
                    appData->modalFeedrate = feedrate_mm_min;
                }
                uint32_t spindle_rpm = appData->modalSpindleRPM;

                uint32_t response_len = (uint32_t)snprintf((char*)txBuffer, sizeof(txBuffer),
                    "<%s|MPos:%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f|FS:%.0f,%u%s%s>\r\n",
                    state, 
                    mpos[AXIS_X], mpos[AXIS_Y], mpos[AXIS_Z], 
                    wpos[AXIS_X], wpos[AXIS_Y], wpos[AXIS_Z],
                    feedrate_mm_min, (unsigned)spindle_rpm,
                    grblCheckMode ? "|Cm:1" : "",
                    feedHoldActive ? "|FH:1" : "");
                UART3_Write(txBuffer, response_len);
                
                // ⚠️ Real-time commands NEVER trigger deferred ok checks
                // The deferred ok will be sent by IDLE loop (line 668) or next command
                break;
            }
            case '~': /* Cycle start / resume */
                if (feedHoldActive) {
                    feedHoldActive = false;
                    /* Future: Re-enable motion pipeline if paused */
                }
                break;
            case '!': /* Feed hold */
                feedHoldActive = true;
                STEPPER_DisableAll();
                break;
            case 0x18: /* Soft reset */
            {
                GCODE_SoftReset(appData, cmdQueue);
                break;
            }
            default:
                break;
        }

        // ✅ GRBL PROTOCOL: Real-time commands NEVER generate "ok" responses
        // Consume control character at position 0, plus any trailing CR/LF
        // This prevents "?\r\n" from leaving "\r\n" which would be processed as blank line
        uint32_t skip_pos = 1;  // Skip the control character itself
        while (skip_pos < nBytesRead && (rxBuffer[skip_pos] == '\r' || rxBuffer[skip_pos] == '\n')) {
            skip_pos++;  // Skip all trailing line terminators
        }
        
        if (skip_pos < nBytesRead) {
            // Remaining bytes after control char + terminators
            memmove(rxBuffer, &rxBuffer[skip_pos], nBytesRead - skip_pos);
            nBytesRead -= skip_pos;
            rxBuffer[nBytesRead] = '\0';
            if (nBytesRead > 0 && is_control_char(rxBuffer[0])) {
                gcodeData.state = GCODE_STATE_CONTROL_CHAR;
            } else {
                gcodeData.state = GCODE_STATE_IDLE;
            }
        } else {
            // Nothing left after consuming control char and terminators - clear everything
            nBytesRead = 0;
            rxBuffer[0] = '\0';  // ✅ Ensure first byte is null
            memset(rxBuffer, 0, sizeof(rxBuffer));
            gcodeData.state = GCODE_STATE_IDLE;
        }
        break;
    }

    case GCODE_STATE_QUERY_CHARS:
    {
        const char* cmd = (const char*)rxBuffer;
        size_t len = strlen(cmd);

        bool send_ok = true;
        bool handled = false;

        if (len >= 4 && cmd[0] == '$' && cmd[1] == 'N' && isdigit((unsigned char)cmd[2]) && cmd[3] == '=') {
            int idx = cmd[2] - '0';
            if (idx == 0 || idx == 1) {
                const char* line = &cmd[4];
                size_t l = strlen(line);
                if (l >= GCODE_BUFFER_SIZE) l = GCODE_BUFFER_SIZE - 1;
                size_t w = 0;
                for (size_t i = 0; i < l && w < (GCODE_BUFFER_SIZE - 1); i++) {
                    unsigned char ch = (unsigned char)line[i];
                    if (ch == '\r' || ch == '\n') break;
                    if (ch >= 32 && ch <= 126) startupLines[idx][w++] = (char)ch;
                }
                startupLines[idx][w] = '\0';
                handled = true;
            } else {
                UART3_Write((uint8_t*)"error:4\r\n", 10);
                send_ok = false;
                handled = true;
            }
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == '$') {
            SETTINGS_PrintAll(SETTINGS_GetCurrent());
            handled = true;
            send_ok = false;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == '#') {
            // GRBL $# command: Report work coordinate systems and offsets
            char buf[400];  // Increased buffer for all WCS data
            int p = 0;
            
            // Report all work coordinate systems (G54-G59)
            for (uint8_t wcs = 0; wcs < 6; wcs++) {
                float x, y, z;
                if (SETTINGS_GetWorkCoordinateSystem(wcs, &x, &y, &z)) {
                    p += snprintf(&buf[p], sizeof(buf)-p, "[G%d:%.3f,%.3f,%.3f]\r\n", 
                                 54 + wcs, x, y, z);
                }
            }
            
            // Report G92 coordinate offset
            float g92_x, g92_y, g92_z;
            SETTINGS_GetG92Offset(&g92_x, &g92_y, &g92_z);
            p += snprintf(&buf[p], sizeof(buf)-p, "[G92:%.3f,%.3f,%.3f]\r\n", g92_x, g92_y, g92_z);
            
            // Report tool length offset
            float tlo = SETTINGS_GetToolLengthOffset();
            p += snprintf(&buf[p], sizeof(buf)-p, "[TLO:%.3f]\r\n", tlo);
            
            // Report probe position (currently not implemented - show zeros)
            p += snprintf(&buf[p], sizeof(buf)-p, "[PRB:0.000,0.000,0.000:0]\r\n");
            
            UART3_Write((uint8_t*)buf, (uint32_t)p);
            handled = true;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == 'G') {
            char state_buffer[160];
            int l = 0;
            l += sprintf(&state_buffer[l], "[GC:");
            l += sprintf(&state_buffer[l], "G0 ");
            l += sprintf(&state_buffer[l], "G%d ", 54 + appData->activeWCS);  // Dynamic WCS (G54-G59)
            l += sprintf(&state_buffer[l], "G%d ", appData->modalPlane == 0 ? 17 : (appData->modalPlane == 1 ? 18 : 19));
            l += sprintf(&state_buffer[l], "%s ", unitsInches ? "G20" : "G21");
            l += sprintf(&state_buffer[l], "G%d ", appData->absoluteMode ? 90 : 91);
            l += sprintf(&state_buffer[l], "G94 ");
            l += sprintf(&state_buffer[l], "M5 ");
            l += sprintf(&state_buffer[l], "M9 ");
            l += sprintf(&state_buffer[l], "T%u ", appData->modalToolNumber);
            l += sprintf(&state_buffer[l], "F%.1f ", appData->modalFeedrate);
            l += sprintf(&state_buffer[l], "S%u", appData->modalSpindleRPM);
            l += sprintf(&state_buffer[l], "]\r\n");
            UART3_Write((uint8_t*)state_buffer, (uint32_t)l);
            handled = true;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == 'H') {
            // $H - Homing cycle command
            // Create homing event and add to command queue
            if (cmdQueue->count < GCODE_MAX_COMMANDS) {
                strcpy(cmdQueue->commands[cmdQueue->head].command, "$H");
                cmdQueue->head = (cmdQueue->head + 1) % GCODE_MAX_COMMANDS;
                cmdQueue->count++;
            }
            handled = true;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == 'L') {
            // $L - Debug command: Show limit switch states
            CNC_Settings* settings = SETTINGS_GetCurrent();
            
            DEBUG_PRINT_GCODE("[LIMIT] Pins: X_Min=%d X_Max=%d Y_Min=%d Y_Max=%d Z_Min=%d Z_Max=%d A_Min=%d A_Max=%d\r\n",
                             LIMIT_GetMin(AXIS_X), LIMIT_GetMax(AXIS_X),
                             LIMIT_GetMin(AXIS_Y), LIMIT_GetMax(AXIS_Y),
                             LIMIT_GetMin(AXIS_Z), LIMIT_GetMax(AXIS_Z),
                             LIMIT_GetMin(AXIS_A), LIMIT_GetMax(AXIS_A));
            
            DEBUG_PRINT_GCODE("[LIMIT] Settings: $5=%u $21=%u (%s)\r\n", 
                             settings->limit_pins_invert,
                             settings->hard_limits_enable,
                             settings->hard_limits_enable ? "ENABLED" : "DISABLED");
            
            // Show which axes would trigger alarm
            bool any_triggered = false;
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                bool triggered = LIMIT_CheckAxis(axis, settings->limit_pins_invert);
                if (triggered) {
                    DEBUG_EXEC_GCODE({
                        char axis_name = 'X' + axis;
                        DEBUG_PRINT_GCODE("[LIMIT] *** AXIS %c TRIGGERED ***\r\n", axis_name);
                    });
                    any_triggered = true;
                }
            }
            if (!any_triggered) {
                DEBUG_PRINT_GCODE("[LIMIT] All clear - no limits triggered\r\n");
            }
            
            handled = true;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == 'I') {
            SETTINGS_PrintBuildInfo();
            handled = true;
            send_ok = false;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == 'N') {
            char buf[160];
            int p = 0;
            p += snprintf(&buf[p], sizeof(buf)-p, "[N0:%s]\r\n", startupLines[0]);
            p += snprintf(&buf[p], sizeof(buf)-p, "[N1:%s]\r\n", startupLines[1]);
            UART3_Write((uint8_t*)buf, (uint32_t)p);
            handled = true;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == 'C') {
            grblCheckMode = !grblCheckMode;
            handled = true;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == 'X') {
            grblAlarm = false;
            
            // ✅ Clear hard limit alarm flag (set by stepper ISR)
            extern volatile bool g_hard_limit_alarm;
            g_hard_limit_alarm = false;
            
            DEBUG_PRINT_GCODE("[GCODE] Alarm cleared via $X command\r\n");
            handled = true;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == 'F') {
            UART3_Write((uint8_t*)"[F:100|S:100]\r\n", 15);
            handled = true;
        }
        else if (len >= 5 && cmd[0] == '$' && cmd[1] == 'R' && cmd[2] == 'S' && cmd[3] == 'T' && cmd[4] == '=') {
            char target = cmd[5];
            if (target == '*') {
                SETTINGS_RestoreDefaults(SETTINGS_GetCurrent());
                grblAlarm = false;
            } else if (target == '$') {
                SETTINGS_RestoreDefaults(SETTINGS_GetCurrent());
            } else if (target == '#') {
                WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();
                for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                    SET_COORDINATE_AXIS(&wcs->offset, axis, 0.0f);
                }
            } else {
                UART3_Write((uint8_t*)"error:4\r\n", 10);
                send_ok = false;
            }
            handled = true;
        }
        else if (len >= 4 && cmd[0] == '$' && cmd[1] == 'S' && cmd[2] == 'L' && cmd[3] == 'P') {
            UART3_Write((uint8_t*)"error:2\r\n", 10);
            send_ok = false;
            handled = true;
        }
        else if (cmd[0] == '$' && isdigit((unsigned char)cmd[1])) {
            char* pend;
            long param = strtol(&cmd[1], &pend, 10);
            if (param < 0) {
                UART3_Write((uint8_t*)"error:3\r\n", 10);
                send_ok = false;
                handled = true;
            } else if (*pend == '=') {
                float val = strtof(pend + 1, NULL);
                CNC_Settings* s = SETTINGS_GetCurrent();
                if (SETTINGS_SetValue(s, (uint32_t)param, val)) {
                    s->checksum = SETTINGS_CalculateCRC32(s);
                    
                    // ✅ Save to flash immediately so setting persists across resets
                    SETTINGS_SaveToFlash(s);
                    
                    // Reload stepper cached settings if step/dir/enable invert changed ($0-$5)
                    if (param <= 5) {
                        STEPPER_ReloadSettings();
                    }
                    
                    handled = true;
                } else {
                    UART3_Write((uint8_t*)"error:3\r\n", 10);
                    send_ok = false;
                    handled = true;
                }
            } else if (*pend == '\0') {
                float val = SETTINGS_GetValue(SETTINGS_GetCurrent(), (uint32_t)param);
                int out = snprintf((char*)txBuffer, sizeof(txBuffer), "$%ld=%g\r\n", param, val);
                UART3_Write(txBuffer, (uint32_t)out);
                handled = true;
            } else {
                UART3_Write((uint8_t*)"error:4\r\n", 10);
                send_ok = false;
                handled = true;
            }
        }
        else if (len >= 5 && cmd[0] == '$' && cmd[1] == 'S' && cmd[2] == 'A' && cmd[3] == 'V' && cmd[4] == 'E') {
            // $SAVE - Explicitly save current settings to flash
            CNC_Settings* settings = SETTINGS_GetCurrent();
            if (settings != NULL && SETTINGS_SaveToFlash(settings)) {
                UART3_Write((uint8_t*)"[MSG:Settings saved to flash]\r\n", 33);
            } else {
                UART3_Write((uint8_t*)"error:9\r\n", 10);  // Error 9: Flash write failed
                send_ok = false;
            }
            handled = true;
        }
        else if (len == 1 && cmd[0] == '$') {
            UART3_Write((uint8_t*)"[HLP:$$ $# $G $I $N $C $X $F $RST= $SAVE $SLP $L]\r\n", 53);
            UART3_Write((uint8_t*)"[MSG:$21 Hard Limits Enable - $21=0 (disabled), $21=1 (enabled)]\r\n", 68);
            UART3_Write((uint8_t*)"[MSG:$5 Limit Pin Invert - NO switch:$5=0 (pin HIGH triggers), NC switch:$5=255 (pin LOW triggers)]\r\n", 103);
            UART3_Write((uint8_t*)"[MSG:$L - Show limit switch states (debug)]\r\n", 46);
            handled = true;
        }

        if (handled && send_ok) {
            UART_SendOK();
        } else if (!handled) {
            UART3_Write((uint8_t*)"error:4\r\n", 10);
        }

        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        gcodeData.state = GCODE_STATE_IDLE;
        break;
    }

    case GCODE_STATE_GCODE_COMMAND:
    {
        uint32_t cmd_end = 0;
        for (uint32_t i = 0; i < nBytesRead; i++) {
            if (rxBuffer[i] == '\0') { cmd_end = i; break; }
        }
        DEBUG_PRINT_GCODE("[GCODE_CMD] Processing command: '%s'\r\n", rxBuffer);
        cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, cmd_end, cmdQueue);
        
        // ✅ DISABLED: Startup deferral caused stalling with single arc commands
        // Normal flow control for all commands (immediate or deferred based on buffer)
        DEBUG_PRINT_GCODE("[GCODE_CMD] Normal flow control\r\n");
        SendOrDeferOk(appData, cmdQueue);
        
        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        gcodeData.state = GCODE_STATE_IDLE;
        break;
    }

    case GCODE_STATE_ERROR:
    {
        uint32_t error_len = (uint32_t)sprintf((char*)txBuffer, "error:1\r\n");
        UART3_Write(txBuffer, error_len);
        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        gcodeData.state = GCODE_STATE_IDLE;
        break;
    }

    default:
        break;
    }
}

/* Optional future advanced parsing disabled */
#if 0
#endif


