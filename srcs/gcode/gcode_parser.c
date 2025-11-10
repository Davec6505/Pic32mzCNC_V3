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

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */
#define ENABLE_STARTUP_BANNER     // Enable/disable startup banner
                                  // G-code senders (UGS, bCNC) expect this banner
                                  // Comment out to disable for debugging or custom ID

// Banner message configured in common.h via STARTUP_BANNER_STRING macro

#define MOTION_BUFFER_THRESHOLD 2

/* -------------------------------------------------------------------------- */
/* Static Buffers / State                                                     */
/* -------------------------------------------------------------------------- */
static uint8_t txBuffer[250];
static uint8_t rxBuffer[250];
static volatile uint32_t nBytesRead = 0;

GCODE_Data gcodeData = {
    .state = GCODE_STATE_IDLE,
};

static bool okPending = false;
static bool grblCheckMode = false;          /* $C toggle */
static bool grblAlarm = false;              /* $X clears alarm */
static bool feedHoldActive = false;         /* '!' feed hold, '~' resume */
static char startupLines[2][GCODE_BUFFER_SIZE] = {{0},{0}}; /* $N0 / $N1 */
static bool unitsInches = false;            /* false=mm (G21), true=inches (G20) */

/* ADDED: After soft reset steppers remain disabled; enable on first motion */
/* static bool stepperEnablePending = false;*/

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

    /* 1. Stop all motion immediately */
    STEPPER_DisableAll();

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

    /* 5. Application state back to IDLE */
    appData->state            = APP_IDLE;

    /* 6. Reset G-code command queue */
    cmdQueue->head  = 0;
    cmdQueue->tail  = 0;
    cmdQueue->count = 0;
    cmdQueue->motionQueueCount = appData->motionQueueCount;

    /* 7. Reset G-code parser state */
    okPending = false;
    grblAlarm = false;
    grblCheckMode = false;
    feedHoldActive = false;
    unitsInches = false;
    gcodeData.state = GCODE_STATE_IDLE;

    nBytesRead = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));

    /* 8. Mark steppers to be re-enabled automatically on first motion command */
    // stepperEnablePending = true;

    /* 9. Print GRBL startup banner (expected by senders after Ctrl+X) */
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
    okPending = false;
    gcodeData.state = GCODE_STATE_IDLE;
    unitsInches = false;
    grblCheckMode = false;
    grblAlarm = false;
    feedHoldActive = false;
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

    if (cmd[0] == 'G') {
        char* pend = NULL;
        int gnum = (int)strtol(&cmd[1], &pend, 10);

        if (gnum == 20) { unitsInches = true; return true; }
        if (gnum == 21) { unitsInches = false; return true; }
        if (gnum == 90) { ev->type = GCODE_EVENT_SET_ABSOLUTE; return true; }
        if (gnum == 91) { ev->type = GCODE_EVENT_SET_RELATIVE; return true; }

        if (gnum == 10) {
            char* pP = find_char((char*)cmd, 'P');
            char* pL = find_char((char*)cmd, 'L');
            int p_val = pP ? (int)strtol(pP + 1, NULL, 10) : 0;
            int l_val = pL ? (int)strtol(pL + 1, NULL, 10) : 0;
            if (l_val == 20) {
                char* pX = find_char((char*)cmd, 'X');
                char* pY = find_char((char*)cmd, 'Y');
                char* pZ = find_char((char*)cmd, 'Z');
                char* pA = find_char((char*)cmd, 'A');
                const float unit_scale = unitsInches ? 25.4f : 1.0f;
                float desiredX = pX ? (parse_float_after(pX) * unit_scale) : NAN;
                float desiredY = pY ? (parse_float_after(pY) * unit_scale) : NAN;
                float desiredZ = pZ ? (parse_float_after(pZ) * unit_scale) : NAN;
                float desiredA = pA ? (parse_float_after(pA) * unit_scale) : NAN;
                if (p_val == 0 || p_val == 1) {
                    StepperPosition* pos = STEPPER_GetPosition();
                    WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();
                    float mpos_x = (float)pos->x_steps / pos->steps_per_mm_x;
                    float mpos_y = (float)pos->y_steps / pos->steps_per_mm_y;
                    float mpos_z = (float)pos->z_steps / pos->steps_per_mm_z;
                    float mpos_a = 0.0f;
                    if (!isnan(desiredX)) wcs->offset.x = mpos_x - desiredX;
                    if (!isnan(desiredY)) wcs->offset.y = mpos_y - desiredY;
                    if (!isnan(desiredZ)) wcs->offset.z = mpos_z - desiredZ;
                    (void)mpos_a;
                    (void)desiredA;
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
        } else if (gnum == 2) {
            ev->type = GCODE_EVENT_ARC_MOVE;
            ev->data.arcMove.clockwise = true;
        } else if (gnum == 3) {
            ev->type = GCODE_EVENT_ARC_MOVE;
            ev->data.arcMove.clockwise = false;
        } else if (gnum == 4) {
            ev->type = GCODE_EVENT_DWELL;
            char* pP = strstr((char*)cmd, "P");
            ev->data.dwell.seconds = pP ? parse_float_after(pP) : 0.0f;
            return true;
        } else {
            return false;
        }

        // If this is the first motion after a soft reset, re-enable steppers
      /*  if (stepperEnablePending &&
            (ev->type == GCODE_EVENT_LINEAR_MOVE || ev->type == GCODE_EVENT_ARC_MOVE)) {
            STEPPER_EnableAll();
            stepperEnablePending = false;
            DEBUG_PRINT_GCODE("[GCODE] Steppers re-enabled after soft reset\r\n");
        }
      */
        char* pX = find_char((char*)cmd, 'X');
        char* pY = find_char((char*)cmd, 'Y');
        char* pZ = find_char((char*)cmd, 'Z');
        char* pA = find_char((char*)cmd, 'A');
        char* pF = find_char((char*)cmd, 'F');

        const float unit_scale = unitsInches ? 25.4f : 1.0f;

        if (ev->type == GCODE_EVENT_LINEAR_MOVE) {
            float x = pX ? parse_float_after(pX) : NAN;
            float y = pY ? parse_float_after(pY) : NAN;
            float z = pZ ? parse_float_after(pZ) : NAN;
            float a = pA ? parse_float_after(pA) : NAN;
            float f = pF ? parse_float_after(pF) : 0.0f;
            ev->data.linearMove.x = !isnan(x) ? x * unit_scale : NAN;
            ev->data.linearMove.y = !isnan(y) ? y * unit_scale : NAN;
            ev->data.linearMove.z = !isnan(z) ? z * unit_scale : NAN;
            ev->data.linearMove.a = !isnan(a) ? a * unit_scale : NAN;
            ev->data.linearMove.feedrate = (f > 0.0f) ? (f * unit_scale) : 0.0f;
            return true;
        } else if (ev->type == GCODE_EVENT_ARC_MOVE) {
            float x = pX ? parse_float_after(pX) : NAN;
            float y = pY ? parse_float_after(pY) : NAN;
            float z = pZ ? parse_float_after(pZ) : NAN;
            float a = pA ? parse_float_after(pA) : NAN;
            char* pI = find_char((char*)cmd, 'I');
            char* pJ = find_char((char*)cmd, 'J');
            float i = pI ? parse_float_after(pI) : 0.0f;
            float j = pJ ? parse_float_after(pJ) : 0.0f;
            float f = pF ? parse_float_after(pF) : 0.0f;
            ev->data.arcMove.x = !isnan(x) ? x * unit_scale : NAN;
            ev->data.arcMove.y = !isnan(y) ? y * unit_scale : NAN;
            ev->data.arcMove.z = !isnan(z) ? z * unit_scale : NAN;
            ev->data.arcMove.a = !isnan(a) ? a * unit_scale : NAN;
            ev->data.arcMove.centerX = i * unit_scale;
            ev->data.arcMove.centerY = j * unit_scale;
            ev->data.arcMove.feedrate = (f > 0.0f) ? (f * unit_scale) : 0.0f;
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
        cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
        cmdQueue->count--;
        return false;
    }

    if (!parse_command_to_event(gc->command, event)) {
        cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
        cmdQueue->count--;
        return false;
    }

    cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
    cmdQueue->count--;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Main G-code / Protocol Tasks                                               */
/* -------------------------------------------------------------------------- */
void GCODE_Tasks(APP_DATA* appData, GCODE_CommandQueue* commandQueue)
{
    GCODE_CommandQueue* cmdQueue = commandQueue;
    uint32_t nBytesAvailable = 0;

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

        if (nBytesRead == 0) break;

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

            if (rxBuffer[0] == '$') {
                gcodeData.state = GCODE_STATE_QUERY_CHARS;
            } else if (rxBuffer[0] == 'G' || rxBuffer[0] == 'g' ||
                       rxBuffer[0] == 'M' || rxBuffer[0] == 'm' ||
                       rxBuffer[0] == 'F' || rxBuffer[0] == 'f' ||
                       rxBuffer[0] == 'T' || rxBuffer[0] == 't' ||
                       rxBuffer[0] == 'S' || rxBuffer[0] == 's') {
                gcodeData.state = GCODE_STATE_GCODE_COMMAND;
            } else {
                bool has_content = false;
                for (uint32_t i = 0; i < terminator_pos; i++) {
                    char c = rxBuffer[i];
                    if (c != ' ' && c != '\t' && c != 0) { has_content = true; break; }
                }
                if (has_content) {
                    cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, terminator_pos, cmdQueue);
                    if (!okPending && cmdQueue->motionQueueCount < (cmdQueue->maxMotionSegments - MOTION_BUFFER_THRESHOLD)) {
                        UART_SendOK();
                    } else okPending = true;
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
                if (okPending && cmdQueue->motionQueueCount < (cmdQueue->maxMotionSegments - MOTION_BUFFER_THRESHOLD)) {
                    UART_SendOK(); okPending = false;
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

                float mpos_x = (float)pos->x_steps / pos->steps_per_mm_x;
                float mpos_y = (float)pos->y_steps / pos->steps_per_mm_y;
                float mpos_z = (float)pos->z_steps / pos->steps_per_mm_z;

                float wpos_x = mpos_x - wcs->offset.x;
                float wpos_y = mpos_y - wcs->offset.y;
                float wpos_z = mpos_z - wcs->offset.z;

                const char* state = "Idle";
                if (grblAlarm) state = "Alarm";
                else if (feedHoldActive) state = "Hold";
                else if (appData->currentSegment != NULL &&
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
                    state, mpos_x, mpos_y, mpos_z, wpos_x, wpos_y, wpos_z,
                    feedrate_mm_min, (unsigned)spindle_rpm,
                    grblCheckMode ? "|Cm:1" : "",
                    feedHoldActive ? "|FH:1" : "");
                UART3_Write(txBuffer, response_len);
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

        if (nBytesRead > 1) {
            memmove(rxBuffer, &rxBuffer[1], nBytesRead - 1);
            nBytesRead -= 1;
            rxBuffer[nBytesRead] = '\0';
            if (nBytesRead > 0 && is_control_char(rxBuffer[0])) {
                gcodeData.state = GCODE_STATE_CONTROL_CHAR;
            } else gcodeData.state = GCODE_STATE_IDLE;
        } else {
            nBytesRead = 0;
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
            WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();
            char buf[160];
            int p = 0;
            p += snprintf(&buf[p], sizeof(buf)-p, "[G54:%.3f,%.3f,%.3f]\r\n", wcs->offset.x, wcs->offset.y, wcs->offset.z);
            p += snprintf(&buf[p], sizeof(buf)-p, "[G55:0.000,0.000,0.000]\r\n");
            p += snprintf(&buf[p], sizeof(buf)-p, "[G56:0.000,0.000,0.000]\r\n");
            p += snprintf(&buf[p], sizeof(buf)-p, "[G57:0.000,0.000,0.000]\r\n");
            p += snprintf(&buf[p], sizeof(buf)-p, "[G58:0.000,0.000,0.000]\r\n");
            p += snprintf(&buf[p], sizeof(buf)-p, "[G59:0.000,0.000,0.000]\r\n");
            p += snprintf(&buf[p], sizeof(buf)-p, "[G92:0.000,0.000,0.000]\r\n");
            p += snprintf(&buf[p], sizeof(buf)-p, "[TLO:0.000]\r\n");
            p += snprintf(&buf[p], sizeof(buf)-p, "[PRB:0.000,0.000,0.000:0]\r\n");
            UART3_Write((uint8_t*)buf, (uint32_t)p);
            handled = true;
        }
        else if (len >= 2 && cmd[0] == '$' && cmd[1] == 'G') {
            char state_buffer[160];
            int l = 0;
            l += sprintf(&state_buffer[l], "[GC:");
            l += sprintf(&state_buffer[l], "G0 ");
            l += sprintf(&state_buffer[l], "G54 ");
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
                wcs->offset.x = 0.0f;
                wcs->offset.y = 0.0f;
                wcs->offset.z = 0.0f;
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
        else if (len == 1 && cmd[0] == '$') {
            UART3_Write((uint8_t*)"[HLP:$$ $# $G $I $N $C $X $F $RST= $SLP]\r\n", 44);
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
        cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, cmd_end, cmdQueue);
        if (!okPending && cmdQueue->motionQueueCount < (cmdQueue->maxMotionSegments - MOTION_BUFFER_THRESHOLD)) {
            UART_SendOK();
        } else okPending = true;
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


