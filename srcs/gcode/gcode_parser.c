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
#include "app.h"          // For APP_Initialize()
#include "stepper.h"
#include "motion.h"
#include "motion_utils.h" // For MOTION_UTILS_EnableAllAxes()
#include "kinematics.h"
#include "motion/homing.h"
#include "utils.h"
#include "settings.h"
#include "data_structures.h"
#include "utils/uart_utils.h"
#ifdef HAS_TMC5160_AXIS
#include "motion/tmc5160.h"   // For TMC5160_ApplySettings()
#endif
#include "../config/default/peripheral/uart/plib_uart3.h"
#include <xc.h>  // For RSWRST, SYSKEY, and processor registers
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
// GCODE QUEUE FLOW CONTROL
// -------------------------
// The 64-slot gcode command queue is the UGS throttle.  The trajectory
// planner queue (also 64 slots) is managed independently by MOTION_Arc
// which backs off at 62/64 — we do NOT use trajectory depth to gate UGS.
//
// HIGH_WATER: stop sending "ok" (defer) when gcode queue reaches this many entries.
// LOW_WATER:  resume sending "ok" when gcode queue drains back to this level.
//
// With GCODE_MAX_COMMANDS=64:
//   HIGH_WATER=48 → defer once 3/4 full  (16 free slots)
//   LOW_WATER=16  → release once 1/4 full (48 free slots)
//
// This creates a 32-command hysteresis band.  UGS pushes commands until it
// hits high-water, pauses, then gets a burst of deferred oks when the MCU
// drains the queue below low-water — keeping both ends busy.
#define GCODE_QUEUE_HIGH_WATER 48
#define GCODE_QUEUE_LOW_WATER  16

/* -------------------------------------------------------------------------- */
/* Static Buffers / State                                                     */
/* -------------------------------------------------------------------------- */
static uint8_t txBuffer[1024];
static uint8_t rxBuffer[1024];  // Increased to match UART3 RX buffer size
static volatile uint32_t nBytesRead = 0;

GCODE_Data gcodeData = {
    .state = GCODE_STATE_IDLE,
};

static uint32_t okPendingCount = 0;         // Flow control: count of deferred "ok" responses

// Startup pre-fill gate.
// On power-on or soft-reset the trajectory queue is empty.  We send "ok"
// freely (immediately) while the queue is below high-water so UGS keeps
// pushing commands and the queue fills up.  Once the queue first reaches
// high-water we flip this flag and enter normal 1-for-1 deferred mode:
// one "ok" released per freed slot, keeping the queue pinned at 63/64.
// Pre-fill phase removed: it was designed for UGS-style pipelined senders.
// With a sequential streamer (wait-for-ok) and the interpolator now counted
// in motionQueueCount, normal 1-for-1 flow control is correct immediately.
/* startupPrefillDone removed — flow control is purely queue-depth-based */

static bool grblCheckMode = false;          /* $C toggle */
static bool grblAlarm = false;              /* $X clears alarm */
// g_feed_hold_active is in stepper.h (defined in stepper.c) — global, gated in app.c APP_IDLE
static char startupLines[2][GCODE_BUFFER_SIZE] = {{0},{0}}; /* $N0 / $N1 */
static bool unitsInches  = false;           /* false=mm (G21), true=inches (G20) */
static bool s_canned_g98 = true;            /* true=G98 (return initial Z), false=G99 (return R) */

/* ✅ REMOVED: Startup deferral variables (caused UGS stalling) */

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */
// Control characters that should be handled specially (not treated as G-code)
// 0x18 = Ctrl+X (soft reset)
// 0x95 = XOFF (flow control - ignored)
// 0x90 = DLE (data link escape - ignored)  
// 0x11 = XON (flow control - ignored)
// 0x13 = DC3/XOFF (flow control - ignored)
static inline bool is_control_char(uint8_t c){ 
    return (c == '?' || c == '~' || c == '!' || c == 0x18 || c == 0x85 ||
            c == 0x95 || c == 0x90 || c == 0x99 || c == 0x11 || c == 0x13); 
}
GCODE_CommandQueue* Extract_CommandLineFrom_Buffer(uint8_t* buffer, uint32_t length, GCODE_CommandQueue* commandQueue);


static inline char* find_char(char* s, char key){
    while (*s){
        if (*s == key) return s;
        s++;
    }
    return NULL;
}
static float parse_float_after(char* start){
    if (!start || !start[1]) return NAN;
    char* endptr = NULL;
    float value = (float)strtof(start + 1, &endptr);
    // If no conversion occurred, endptr == start+1
    if (endptr == start + 1) return NAN;
    return value;
}

/* -------------------------------------------------------------------------- */
/* Centralized Soft Reset (PUBLIC FUNCTION)                                  */
/* -------------------------------------------------------------------------- */
void GCODE_SoftReset(APP_DATA* appData, GCODE_CommandQueue* cmdQueue)
{
    if (appData == NULL || cmdQueue == NULL) {
        return;
    }

    DEBUG_PRINT_GCODE("[SOFT RESET] Software reset - clearing buffers and state\r\n");
    
    /* 1. Stop all motion immediately */
    STEPPER_StopMotion();  // Disables steppers, stops TMR4, disables OC1
    HOMING_Abort();        // Abort homing if in progress
    
    /* 2. Re-initialize stepper module (clears Bresenham state, direction bits, etc.) */
    // This fixes the X-axis selective disable issue after soft reset
    STEPPER_Initialize(appData);
    
    /* 3. Flush UART RX buffer to avoid processing pre-reset commands */
    uint8_t scratch[64];
    uint32_t rc;
    while ((rc = UART3_ReadCountGet()) > 0U) {
        uint32_t toRead = (rc > sizeof(scratch)) ? (uint32_t)sizeof(scratch) : rc;
        (void)UART3_Read(scratch, toRead);
    }
    

        /* 10. Clear RX buffer */
    nBytesRead = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));

    /* 3. Clear motion queue */
    memset(appData->motionQueue, 0, sizeof(appData->motionQueue));
    appData->motionQueueHead = 0;
    appData->motionQueueTail = 0;
    appData->motionQueueCount = 0;
    appData->currentSegment = NULL;
    appData->motionSegmentCompleted = false;
    
    /* 4. Clear arc generator state */
    appData->arcGenState = ARC_GEN_IDLE;
    
    /* 5. Reset modal state to GRBL defaults */
    appData->modalPlane = 0;        // G17 (XY plane)
    appData->absoluteMode = true;   // G90 (absolute mode)
    appData->modalFeedrate = 0.0f;  // F (no feedrate set)
    appData->modalSpindleRPM = 0;   // S (spindle off)
    appData->modalToolNumber = 0;   // T0
    
    /* 6. Clear alarm state */
    // Clear alarm but suppress hard limits until user moves off limit
    // This prevents immediate re-alarm after soft reset while on limit
    g_hard_limit_alarm = false;
    g_estop_pending = false;        // Just in case ESTOP was pending when soft limit was hit.
    g_suppress_hard_limits = true;  // Will auto-clear in IDLE when limits released
    appData->alarmCode = 0;
    grblAlarm = false;
    
    /* 7. Reset application state to IDLE */
    appData->state = APP_IDLE;
    
    /* 8. Clear G-code command queue */
    cmdQueue->head = 0;
    cmdQueue->tail = 0;
    cmdQueue->count = 0;
    cmdQueue->commands_consumed = 0;
    
    /* 9. Reset G-code parser state */
    okPendingCount = 0;       // Clear all deferred ok responses
    grblCheckMode = false;
    g_feed_hold_active  = false;  // Clear hold (defined in stepper.c / stepper.h)
    g_feed_hold_pending = false;  // Cancel any in-flight pending hold
    unitsInches = false;
    

    gcodeData.state = GCODE_STATE_IDLE;
    
    /* 11. Send GRBL startup banner (UGS expects this after soft reset) */
    UART_SEND_BANNER();
    
    DEBUG_PRINT_GCODE("[SOFT RESET] Complete - ready for new commands\r\n");
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
    okPendingCount = 0;       // Clear all deferred ok responses
    gcodeData.state = GCODE_STATE_IDLE;
    unitsInches = false;
    grblCheckMode = false;
    grblAlarm = false;
    g_feed_hold_active  = false;
    g_feed_hold_pending = false;

    // ✅ REMOVED: Startup deferral initialization (no longer used)
}

/* -------------------------------------------------------------------------- */
/* Helper: queue command (assumes space available already checked)           */
/* -------------------------------------------------------------------------- */
static inline void queue_command(GCODE_CommandQueue* q, const char* src, size_t len){
    if (len == 0 || len >= GCODE_BUFFER_SIZE) return;
    if (q->count >= GCODE_MAX_COMMANDS) {
        // Flow control should prevent this — if it fires, the sender got an ok
        // before the command queue had a free slot, which means a command is
        // silently dropped.  Emit a UART error so it shows up in trace output.
        UART_Printf("error:20\r\n");  // error:20 = gcode queue overflow
        return;
    }
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
        if (gnum == 20) { unitsInches = true;  ev->type = GCODE_EVENT_NONE; return true; }
        if (gnum == 21) { unitsInches = false; ev->type = GCODE_EVENT_NONE; return true; }

        // G98/G99 - Canned cycle return mode (modal, no event)
        if (gnum == 98) { s_canned_g98 = true;  ev->type = GCODE_EVENT_NONE; return true; }
        if (gnum == 99) { s_canned_g98 = false; ev->type = GCODE_EVENT_NONE; return true; }

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
            // G10 L2  Pn X# Y# Z# — set WCS offset directly (offset IS the value)
            // G10 L20 Pn X# Y# Z# — set WCS so current machine pos = given work pos
            // P0 = active WCS; P1–P6 = G54–G59 respectively
            char* pP = find_char((char*)cmd, 'P');
            char* pL = find_char((char*)cmd, 'L');
            int p_val = pP ? (int)strtol(pP + 1, NULL, 10) : 0;
            int l_val = pL ? (int)strtol(pL + 1, NULL, 10) : 0;
            if (l_val != 2 && l_val != 20) return true;  // Unsupported L — silently consume

            const float unit_scale = unitsInches ? 25.4f : 1.0f;
            ev->type = GCODE_EVENT_SET_WORK_OFFSET;
            ev->data.workOffset.l_value    = (uint32_t)l_val;
            // P0 → sentinel 255 = "resolve to active WCS at event-handle time"
            // P1–P6 → G54–G59 (wcs_number = p_val - 1)
            ev->data.workOffset.wcs_number = (p_val == 0) ? 255u
                : ((p_val >= 1 && p_val <= 6) ? (uint8_t)(p_val - 1) : 255u);

            ev->data.workOffset.x = NAN;
            ev->data.workOffset.y = NAN;
            ev->data.workOffset.z = NAN;
            ev->data.workOffset.a = NAN;
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                char* pAxis = find_char((char*)cmd, axis_letters[axis]);
                float v = pAxis ? (parse_float_after(pAxis) * unit_scale) : NAN;
                switch (axis) {
                    case AXIS_X: ev->data.workOffset.x = v; break;
                    case AXIS_Y: ev->data.workOffset.y = v; break;
                    case AXIS_Z: ev->data.workOffset.z = v; break;
                    case AXIS_A: ev->data.workOffset.a = v; break;
                    default: break;
                }
            }
            DEBUG_PRINT_GCODE("[PARSE] G10 L%d P%d wcs=%u x=%.3f y=%.3f z=%.3f\r\n",
                l_val, p_val, (unsigned)ev->data.workOffset.wcs_number,
                (double)ev->data.workOffset.x, (double)ev->data.workOffset.y,
                (double)ev->data.workOffset.z);
            return true;
        }

        // G38.2, G38.3, G38.4, G38.5 - Probe commands
        if (gnum == 38) {
            // Parse subcode after decimal point (e.g., G38.2 -> subcode = 2)
            char* pDot = find_char((char*)cmd, '.');
            if (pDot) {
                int subcode = (int)strtol(pDot + 1, NULL, 10);
                if (subcode >= 2 && subcode <= 5) {
                    // Determine probe direction and alarm behavior
                    if (subcode == 2 || subcode == 3) {
                        ev->type = GCODE_EVENT_PROBE_TOWARD;  // Probe toward workpiece
                        ev->data.probe.probe_toward = true;
                    } else {  // subcode == 4 || subcode == 5
                        ev->type = GCODE_EVENT_PROBE_AWAY;    // Probe away from workpiece
                        ev->data.probe.probe_toward = false;
                    }
                    
                    // Set alarm behavior: G38.2 and G38.4 alarm on failure
                    ev->data.probe.alarm_on_fail = (subcode == 2 || subcode == 4);
                    
                    // Parse axis parameters and feedrate
                    const float unit_scale = unitsInches ? 25.4f : 1.0f;
                    
                    // Array-based axis parameter parsing
                    float axis_values[NUM_AXIS];
                    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                        char* pAxis = find_char((char*)cmd, axis_letters[axis]);
                        axis_values[axis] = pAxis ? parse_float_after(pAxis) : NAN;
                    }
                    
                    // Store target coordinates (NAN = no change for that axis)
                    ev->data.probe.x = !isnan(axis_values[AXIS_X]) ? axis_values[AXIS_X] * unit_scale : NAN;
                    ev->data.probe.y = !isnan(axis_values[AXIS_Y]) ? axis_values[AXIS_Y] * unit_scale : NAN;
                    ev->data.probe.z = !isnan(axis_values[AXIS_Z]) ? axis_values[AXIS_Z] * unit_scale : NAN;
                    ev->data.probe.a = !isnan(axis_values[AXIS_A]) ? axis_values[AXIS_A] * unit_scale : NAN;
                    
                    // Parse feedrate (required for probe moves)
                    char* pF = find_char((char*)cmd, 'F');
                    ev->data.probe.feedrate = pF ? (parse_float_after(pF) * unit_scale) : 0.0f;
                    
                    DEBUG_PRINT_GCODE("[PARSE] Probe G38.%d: toward=%d alarm=%d Z=%.2f F=%.1f\r\n",
                                     subcode, ev->data.probe.probe_toward, 
                                     ev->data.probe.alarm_on_fail,
                                     ev->data.probe.z, ev->data.probe.feedrate);
                    
                    return true;
                } else {
                    DEBUG_PRINT_GCODE("[PARSE] Invalid G38 subcode: %d\r\n", subcode);
                    return false;
                }
            }
            DEBUG_PRINT_GCODE("[PARSE] G38 missing subcode\r\n");
            return false;
        }

        // G43 / G43.1 - Activate Tool Length Offset
        // G43      : activate stored TLO (from settings; optional H-word ignored — single tool supported)
        // G43.1 Z# : dynamic inline TLO from Z parameter
        if (gnum == 43) {
            char* pDot = find_char((char*)cmd, '.');
            int subcode = pDot ? (int)strtol(pDot + 1, NULL, 10) : 0;
            ev->type = GCODE_EVENT_TLO_SET;
            if (subcode == 1) {
                // G43.1 — inline dynamic value from Z parameter
                char* pZ = find_char((char*)cmd, 'Z');
                const float unit_scale = unitsInches ? 25.4f : 1.0f;
                ev->data.tlo.value   = pZ ? (parse_float_after(pZ) * unit_scale) : 0.0f;
                ev->data.tlo.dynamic = true;
            } else {
                // G43 — use persisted tool length offset from settings
                ev->data.tlo.value   = SETTINGS_GetToolLengthOffset();
                ev->data.tlo.dynamic = false;
            }
            DEBUG_PRINT_GCODE("[PARSE] G43.%d TLO=%.3f dynamic=%d\r\n",
                              subcode, (double)ev->data.tlo.value, (int)ev->data.tlo.dynamic);
            return true;
        }

        // G49 - Cancel Tool Length Offset
        if (gnum == 49) {
            ev->type = GCODE_EVENT_TLO_CANCEL;
            DEBUG_PRINT_GCODE("[PARSE] G49 TLO cancel\r\n");
            return true;
        }

        // G80 - Cancel active canned cycle
        if (gnum == 80) {
            ev->type = GCODE_EVENT_CANNED_CANCEL;
            DEBUG_PRINT_GCODE("[PARSE] G80 canned cancel\r\n");
            return true;
        }

        // G81 - Simple drill cycle: rapid to X/Y, rapid to R, feed to Z, rapid out
        // G83 - Peck drill cycle: same approach but depth in Q increments with retract between
        if (gnum == 81 || gnum == 83) {
            const float unit_scale = unitsInches ? 25.4f : 1.0f;

            char* pX = find_char((char*)cmd, 'X');
            char* pY = find_char((char*)cmd, 'Y');
            char* pZ = find_char((char*)cmd, 'Z');
            char* pR = find_char((char*)cmd, 'R');
            char* pQ = find_char((char*)cmd, 'Q');
            char* pF = find_char((char*)cmd, 'F');
            char* pL = find_char((char*)cmd, 'L');

            // Z and R are required; reject silently if missing
            if (!pZ || !pR) {
                DEBUG_PRINT_GCODE("[PARSE] G8%d missing Z or R — ignored\r\n", gnum == 81 ? 1 : 3);
                return false;
            }

            ev->type = (gnum == 81) ? GCODE_EVENT_CANNED_DRILL : GCODE_EVENT_CANNED_PECK;

            ev->data.cannedDrill.x        = pX ? (parse_float_after(pX) * unit_scale) : NAN;
            ev->data.cannedDrill.y        = pY ? (parse_float_after(pY) * unit_scale) : NAN;
            ev->data.cannedDrill.z        = parse_float_after(pZ) * unit_scale;
            ev->data.cannedDrill.r        = parse_float_after(pR) * unit_scale;
            ev->data.cannedDrill.q        = pQ ? (parse_float_after(pQ) * unit_scale) : 0.0f;
            ev->data.cannedDrill.feedrate = pF ? (parse_float_after(pF) * unit_scale) : 0.0f;
            // L0 or L absent → treat as L1 (one hole), GRBL convention
            uint32_t l_raw = pL ? (uint32_t)strtol(pL + 1, NULL, 10) : 1u;
            ev->data.cannedDrill.l        = (l_raw == 0u) ? 1u : l_raw;
            ev->data.cannedDrill.g98      = s_canned_g98;

            DEBUG_PRINT_GCODE("[PARSE] G%d X=%.2f Y=%.2f Z=%.2f R=%.2f Q=%.2f F=%.1f L=%lu G%s\r\n",
                gnum,
                (double)ev->data.cannedDrill.x, (double)ev->data.cannedDrill.y,
                (double)ev->data.cannedDrill.z, (double)ev->data.cannedDrill.r,
                (double)ev->data.cannedDrill.q, (double)ev->data.cannedDrill.feedrate,
                (unsigned long)ev->data.cannedDrill.l,
                s_canned_g98 ? "98" : "99");
            return true;
        }

        // G33 - Spindle-synchronised motion (rigid tapping precursor)
        // Blocked until spindle encoder feedback is available (Phase 5).
        if (gnum == 33) {
            UART_Printf("error:2\r\n");  // unsupported command
            DEBUG_PRINT_GCODE("[PARSE] G33 unsupported (no spindle encoder)\r\n");
            ev->type = GCODE_EVENT_NONE;
            return true;  // consume to avoid infinite replay
        }

        if (gnum == 0 || gnum == 1) {
            ev->type = GCODE_EVENT_LINEAR_MOVE;
            ev->data.linearMove.isRapid = (gnum == 0);  // G0 = rapid, G1 = feed
            DEBUG_PRINT_GCODE("[PARSE] Linear move (G%d)%s\r\n", gnum, (gnum == 0) ? " RAPID" : "");
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
            // For G0, isRapid was already set above; feedrate=0 means "use max_rate in motion.c"
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
        if (mnum == 0) { ev->type = GCODE_EVENT_PROGRAM_END; return true; }  // M0 - Program stop
        if (mnum == 2) { ev->type = GCODE_EVENT_PROGRAM_END; return true; }  // M2 - Program end
        if (mnum == 3) {
            ev->type = GCODE_EVENT_SPINDLE_ON;
            char* pS = find_char((char*)cmd, 'S');
            ev->data.spindle.rpm = pS ? (uint32_t)strtoul(pS + 1, NULL, 10) : 1000;
            return true;
        }
        if (mnum == 5) { ev->type = GCODE_EVENT_SPINDLE_OFF; return true; }
        if (mnum == 7) { ev->type = GCODE_EVENT_COOLANT_ON; return true; }
        if (mnum == 9) { ev->type = GCODE_EVENT_COOLANT_OFF; return true; }
        if (mnum == 30) { ev->type = GCODE_EVENT_PROGRAM_END; return true; } // M30 - Program end and reset
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
        cmdQueue->commands_consumed++;
        return false;
    }

    if (!parse_command_to_event(gc->command, event)) {
        // Parse failed - consume command and return false
        DEBUG_PRINT_GCODE("[GetEvent] Parse failed for: '%s'\r\n", gc->command);
        cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
        cmdQueue->count--;
        cmdQueue->commands_consumed++;
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
    cmdQueue->commands_consumed++;
}

/* -------------------------------------------------------------------------- */
/* Flow Control Helpers - Centralized OK Management                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check if we should send a deferred "ok" response
 *
 * Two-phase release strategy:
 *
 * Phase A — gcode queue still has commands (q->count > 0):
 *   Release ONE deferred ok each time the gcode queue count DECREASES (a
 *   command was consumed and dispatched to the trajectory planner).
 *   prev_count ensures exactly one ok per consumed slot regardless of how
 *   many times this function is called per iteration.
 *
 * Phase B — gcode queue fully drained (q->count == 0):
 *   Hold ALL remaining deferred oks until physical motion is completely done:
 *     - trajectory queue empty  (motionQueueCount == 0)
 *     - arc generation finished (arcGenState == ARC_GEN_IDLE)
 *   Then flush every remaining pending ok in one burst.
 *
 * This guarantees UGS cannot receive its final ok(s) while stepper motion
 * is still executing, regardless of how many trajectory segments an arc or
 * a dense section of linear moves generates.
 */
void GCODE_CheckDeferredOk(APP_DATA* appData, GCODE_CommandQueue* q) {
    // Use the monotonically-increasing commands_consumed counter to determine
    // how many OKs to release.  This is correct even when new commands arrive
    // simultaneously (which makes the raw q->count delta unreliable — the
    // depth can stay flat while commands flow through, starving UGS).
    static uint32_t prev_consumed = 0;
    uint32_t curr_consumed = q->commands_consumed;

    // Do NOT release deferred oks while a G4 dwell is in progress.
    if (MOTION_IsDwellActive()) {
        DEBUG_PRINT_GCODE("[FLOW] Dwell active — suppressing deferred ok release\r\n");
        return;
    }

    if (okPendingCount == 0) {
        // Nothing to release — keep prev_consumed in sync so we don't get a
        // spurious burst (uint32 wraparound) when commands_consumed is reset
        // on soft-reset but prev_consumed still holds the old value.
        prev_consumed = curr_consumed;
        return;
    }

    if (curr_consumed != prev_consumed) {
        // Commands were consumed since last check — release one ok per command.
        uint32_t consumed = curr_consumed - prev_consumed; // wraps safely (uint32)
        prev_consumed = curr_consumed;
        while (consumed > 0 && okPendingCount > 0) {
            if (!UART_SendOK()) break;
            DEBUG_PRINT_GCODE("[DEFERRED] Command consumed, sent ok (pending=%lu)\r\n",
                              (unsigned long)(okPendingCount - 1));
            okPendingCount--;
            consumed--;
        }
    } else if (q->count == 0 && okPendingCount > 0) {
        // Queue drained to zero with no new consumption detected this call.
        // Flush remaining so UGS is never permanently blocked on the last batch.
        DEBUG_PRINT_GCODE("[DEFERRED] Queue empty — flushing %lu remaining oks\r\n",
                          (unsigned long)okPendingCount);
        while (okPendingCount > 0) {
            if (!UART_SendOK()) break;
            okPendingCount--;
        }
    }
}

/**
 * @brief Decide whether to send "ok" immediately or defer it
 *
 * Gates purely on gcode command queue depth:
 *   >= HIGH_WATER → defer (stop filling UGS send window)
 *   <  HIGH_WATER → send immediately (keep UGS pushing commands)
 *
 * The trajectory planner queue is managed independently (MOTION_Arc backs
 * off at 62/64) — it does NOT gate UGS flow here.
 */
static void SendOrDeferOk(APP_DATA* appData, GCODE_CommandQueue* q)
{
    // While a G4 dwell is active, always defer.
    if (MOTION_IsDwellActive()) {
        DEBUG_PRINT_GCODE("[FLOW] Dwell active — deferring ok\r\n");
        okPendingCount++;
        return;
    }

    // Gate purely on gcode queue depth: one ok per one accepted command.
    if (q->count >= GCODE_QUEUE_HIGH_WATER) {
        DEBUG_PRINT_GCODE("[FLOW] Deferring ok (gcodeQ=%lu >= HIGH=%d, pending=%lu)\r\n",
                          (unsigned long)q->count, GCODE_QUEUE_HIGH_WATER,
                          (unsigned long)(okPendingCount + 1));
        okPendingCount++;
    } else {
        DEBUG_PRINT_GCODE("[FLOW] Sending immediate ok (gcodeQ=%lu < HIGH=%d)\r\n",
                          (unsigned long)q->count, GCODE_QUEUE_HIGH_WATER);
        (void)UART_SendOK();
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
                } else if (g_feed_hold_pending) {
                    state = "Hold:1";  // GRBL v1.1: Hold:1 = draining / decelerating
                } else if (g_feed_hold_active) {
                    state = "Hold:0";  // GRBL v1.1: Hold:0 = fully stopped
                } else if (MOTION_IsJogging()) {
                    state = "Jog";
                } else if (appData->arcGenState == ARC_GEN_ACTIVE) {
                    // Arc generator active → still processing arc segments
                    state = "Run";
                } else if ((T4CON & _T4CON_ON_MASK) != 0) {
                    // Hardware timer running → motion in progress (or about to start)
                    state = "Run";
                } else if (appData->motionQueueCount > 0) {
                    // Motion queue has segments waiting
                    state = "Run";
                }

                // FS feedrate: read actual step_interval from the executing ISR segment.
                // prep_current_speed tracks the look-ahead preparation cursor, NOT
                // the segment currently clocking through the ISR, so it lags/leads
                // the real executing speed.  STEPPER_GetCurrentFeedrateMmMin() reads
                // the hardware timer ticks directly – zero ambiguity.
                float feedrate_mm_min = 0.0f;
                if (state[0] == 'R' || state[0] == 'J') {
                    feedrate_mm_min = STEPPER_GetCurrentFeedrateMmMin();
                    if (feedrate_mm_min < 1.0f) {
                        // Fallback: use modal feedrate if step_interval not yet loaded
                        feedrate_mm_min = (appData->modalFeedrate > 0.0f) ? appData->modalFeedrate : 0.0f;
                    }
                }
                // ✅ FIX: XC32 -msingle-float ABI mismatch — printf lib expects 8-byte double
                // for %f/%g but caller passes 4-byte float.  Convert both to uint32_t and
                // use %lu so there is zero float/double argument-size ambiguity.
                uint32_t feed_display   = (uint32_t)(feedrate_mm_min + 0.5f);
                uint32_t spindle_display = appData->modalSpindleRPM;

                uint32_t response_len = (uint32_t)snprintf((char*)txBuffer, sizeof(txBuffer),
                    "<%s|MPos:%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f|FS:%lu,%lu%s%s>\r\n",
                    state, 
                    mpos[AXIS_X], mpos[AXIS_Y], mpos[AXIS_Z], 
                    wpos[AXIS_X], wpos[AXIS_Y], wpos[AXIS_Z],
                    (unsigned long)feed_display, (unsigned long)spindle_display,
                    grblCheckMode ? "|Cm:1" : "",
                    "");  // Hold state reported via state string (Hold:0), not separate flag
                UART3_Write(txBuffer, response_len);
                
                // ⚠️ Real-time commands NEVER trigger deferred ok checks
                // The deferred ok will be sent by IDLE loop (line 668) or next command
                break;
            }
            case '~': /* Cycle start / resume */
                if (g_feed_hold_active || g_feed_hold_pending) {
                    STEPPER_ResumeMotion();  // Cancels pending OR restarts from full stop
                    DEBUG_PRINT_GCODE("[GCODE] Feed hold released — motion resuming\r\n");
                }
                break;
            case '!': /* Feed hold */
                if (!g_feed_hold_active && !g_feed_hold_pending) {
                    STEPPER_PauseMotion();   // Sets pending (drain) or immediate if queue empty
                    DEBUG_PRINT_GCODE("[GCODE] Feed hold active\r\n");
                }
                break;
            case 0x18: /* Soft reset (Ctrl+X) */
            {
                GCODE_SoftReset(appData, cmdQueue);
                break;
            }
            case 0x85: /* Jog cancel — flush queued jog moves, no alarm */
                MOTION_JogCancel();
                // No response per GRBL v1.1 spec
                break;
            case 0x95: /* XOFF - flow control, ignore */
            case 0x90: /* DLE - data link escape, ignore */
            case 0x99: /* Unknown control char - ignore */
            case 0x11: /* XON - flow control, ignore */
            case 0x13: /* DC3/XOFF - flow control, ignore */
                // Flow control characters - silently consume without "ok" response
                // These are not G-code commands and don't require acknowledgment
                DEBUG_PRINT_GCODE("[GCODE] Ignoring flow control byte: 0x%02X\r\n", rxBuffer[0]);
                break;
            default:
                // Unknown control character - ignore
                DEBUG_PRINT_GCODE("[GCODE] Unknown control char: 0x%02X\r\n", rxBuffer[0]);
                break;
        }

        // ✅ GRBL PROTOCOL: Real-time commands (? ! ~) NEVER generate "ok" responses
        // ✅ Flow control characters (0x95, 0x90, etc.) also don't generate "ok"
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
            
            // Report live tool length offset (reflects G43, G43.1 and G49)
            bool tlo_active = false;
            float tlo = KINEMATICS_GetTLO(&tlo_active);
            (void)tlo_active;  // active flag available for future use
            p += snprintf(&buf[p], sizeof(buf)-p, "[TLO:%.3f]\r\n", tlo);
            
            // Report last probe result (contact position + triggered flag)
            p += snprintf(&buf[p], sizeof(buf)-p, "[PRB:%.3f,%.3f,%.3f:%d]\r\n",
                (double)appData->probePosition.coordinate[AXIS_X],
                (double)appData->probePosition.coordinate[AXIS_Y],
                (double)appData->probePosition.coordinate[AXIS_Z],
                appData->probeSuccess ? 1 : 0);
            
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
            // Tool length offset modal state (G43 if active, G49 if cancelled)
            { bool tlo_on = false; KINEMATICS_GetTLO(&tlo_on);
              l += sprintf(&state_buffer[l], "%s ", tlo_on ? "G43" : "G49"); }
            l += sprintf(&state_buffer[l], "M5 ");
            l += sprintf(&state_buffer[l], "M9 ");
            l += sprintf(&state_buffer[l], "T%lu ", (unsigned long)appData->modalToolNumber);
            // ✅ FIX: Use integer cast to avoid float/double ABI mismatch in sprintf
            l += sprintf(&state_buffer[l], "F%lu ", (unsigned long)(uint32_t)(appData->modalFeedrate + 0.5f));
            l += sprintf(&state_buffer[l], "S%lu", (unsigned long)appData->modalSpindleRPM);
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
            // g_hard_limit_alarm declared in stepper.h (already included at top)
            g_hard_limit_alarm = false;
            
            // ✅ CRITICAL FIX: Clear hard limit suppression flag
            // This allows recovery after moving off limit switch
            // User must physically move off limit before $X will work
            CNC_Settings* settings = SETTINGS_GetCurrent();
            if (!MOTION_UTILS_CheckHardLimits(settings->limit_pins_invert)) {
                // Limits are clear - allow recovery
                g_suppress_hard_limits = false;
                DEBUG_PRINT_GCODE("[GCODE] Alarm cleared via $X - limits clear, recovery allowed\r\n");
            } else {
                // Still on limit - cannot clear alarm
                UART_Printf("[MSG:Cannot clear alarm - move off limit switch first]\r\n");
                g_hard_limit_alarm = true;  // Keep alarm active
                grblAlarm = true;
                send_ok = false;  // Don't send ok for failed $X
            }
            
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

#ifdef HAS_TMC5160_AXIS
                    // Re-apply TMC5160 configuration when motor driver params change ($200-$253)
                    // Parameter layout: $2X0 = axis X, $2X1 = axis Y, $2X2 = Z, $2X3 = A
                    if (param >= 200 && param <= 253) {
                        uint32_t axis_idx = (uint32_t)param % 10;
                        if (axis_idx < (uint32_t)NUM_AXIS) {
                            TMC5160_ApplySettings((E_AXIS)axis_idx, s);
                        }
                    }
#endif
                    
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
        else if (len >= 3 && cmd[0] == '$' && cmd[1] == 'J' && cmd[2] == '=') {
            // $J= jog command — parsed and submitted directly (bypasses command queue).
            // GRBL v1.1: sends "ok" on acceptance; error on alarm, bad syntax, or queue full.
            if (grblAlarm) {
                UART3_Write((uint8_t*)"error:2\r\n", 9);
                send_ok = false;
            } else {
                const char *p = &cmd[3];
                bool jog_relative = !appData->absoluteMode;
                bool jog_inches   = unitsInches;
                float jx = NAN, jy = NAN, jz = NAN, ja = NAN, jf = NAN;
                bool parse_err = false;

                while (*p && !parse_err) {
                    char ch = (char)toupper((unsigned char)*p);
                    if (ch == ' ' || ch == '\t') { p++; continue; }
                    if (ch == 'G') {
                        p++;
                        char *end = NULL;
                        long gnum = strtol(p, &end, 10);
                        if (end == p) { parse_err = true; break; }
                        p = end;
                        if      (gnum == 90) jog_relative = false;
                        else if (gnum == 91) jog_relative = true;
                        else if (gnum == 20) jog_inches   = true;
                        else if (gnum == 21) jog_inches   = false;
                        else { parse_err = true; }
                    } else if (ch == 'X' || ch == 'Y' || ch == 'Z' || ch == 'A' || ch == 'F') {
                        p++;
                        char *end = NULL;
                        float val = strtof(p, &end);
                        if (end == p) { parse_err = true; break; }
                        p = end;
                        if      (ch == 'X') jx = val;
                        else if (ch == 'Y') jy = val;
                        else if (ch == 'Z') jz = val;
                        else if (ch == 'A') ja = val;
                        else if (ch == 'F') jf = val;
                    } else {
                        parse_err = true;
                    }
                }

                // F word is mandatory; at least one axis must be specified
                if (parse_err || isnan(jf) || jf <= 0.0f ||
                    (isnan(jx) && isnan(jy) && isnan(jz) && isnan(ja))) {
                    UART3_Write((uint8_t*)"error:4\r\n", 9);
                    send_ok = false;
                } else {
                    float scale = jog_inches ? 25.4f : 1.0f;

                    // Resolve target in work coords (default unspecified axes to current work pos)
                    CoordinatePoint cur_m = KINEMATICS_GetCurrentPosition();
                    CoordinatePoint cur_w = KINEMATICS_MachineToWork(cur_m);
                    CoordinatePoint tgt_w = cur_w;

                    if (!isnan(jx)) {
                        if (jog_relative) tgt_w.coordinate[AXIS_X] += jx * scale;
                        else              tgt_w.coordinate[AXIS_X]  = jx * scale;
                    }
                    if (!isnan(jy)) {
                        if (jog_relative) tgt_w.coordinate[AXIS_Y] += jy * scale;
                        else              tgt_w.coordinate[AXIS_Y]  = jy * scale;
                    }
                    if (!isnan(jz)) {
                        if (jog_relative) tgt_w.coordinate[AXIS_Z] += jz * scale;
                        else              tgt_w.coordinate[AXIS_Z]  = jz * scale;
                    }
                    if (!isnan(ja)) {
                        if (jog_relative) tgt_w.coordinate[AXIS_A] += ja * scale;
                        else              tgt_w.coordinate[AXIS_A]  = ja * scale;
                    }

                    jf *= scale;  // scale feedrate if in inches mode

                    // Cap feedrate to the most restrictive axis max_rate
                    const CNC_Settings *js = SETTINGS_GetCurrent();
                    float fr_cap = js->max_rate[AXIS_X];
                    for (E_AXIS ax = AXIS_Y; ax < NUM_AXIS; ax++) {
                        if (js->max_rate[ax] < fr_cap) fr_cap = js->max_rate[ax];
                    }
                    if (jf > fr_cap) jf = fr_cap;
                    if (jf < 1.0f)   jf = 1.0f;

                    // Build jog event and submit directly to motion bridge
                    GCODE_Event ev;
                    memset(&ev, 0, sizeof(ev));
                    ev.type                     = GCODE_EVENT_JOG;
                    ev.data.linearMove.x        = tgt_w.coordinate[AXIS_X];
                    ev.data.linearMove.y        = tgt_w.coordinate[AXIS_Y];
                    ev.data.linearMove.z        = tgt_w.coordinate[AXIS_Z];
                    ev.data.linearMove.a        = tgt_w.coordinate[AXIS_A];
                    ev.data.linearMove.feedrate = jf;
                    ev.data.linearMove.isRapid  = false;
                    ev.data.linearMove.is_jog   = true;

                    if (!MOTION_ProcessGcodeEvent(appData, &ev)) {
                        // Trajectory queue full
                        UART3_Write((uint8_t*)"error:8\r\n", 9);
                        send_ok = false;
                    }
                    // send_ok remains true on successful submission
                }
            }
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

        // Probe commands (G38.x) own their own ok: it is sent by app.c ONLY after
        // probe completion ([PRB:...] result line precedes the ok).  Suppress the
        // normal SendOrDeferOk here so UGS does not receive a premature ok.
        bool is_probe_cmd = false;
        for (uint32_t pi = 0; pi + 2 < nBytesRead && rxBuffer[pi] != '\0'; pi++) {
            if ((rxBuffer[pi] == 'G' || rxBuffer[pi] == 'g') &&
                 rxBuffer[pi+1] == '3' && rxBuffer[pi+2] == '8') {
                is_probe_cmd = true;
                break;
            }
        }
        if (!is_probe_cmd) {
            DEBUG_PRINT_GCODE("[GCODE_CMD] Normal flow control\r\n");
            SendOrDeferOk(appData, cmdQueue);
        } else {
            DEBUG_PRINT_GCODE("[GCODE_CMD] Probe command — ok withheld until completion\r\n");
        }

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


