#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H


#include "definitions.h"
#include "plib_uart3.h"
#include "plib_uart_common.h"
#include "../data_structures.h"  // ✅ Use unified data structures (parent directory)

/* string literals for GRBL firmware commands */
#ifndef WindowsOS
#define NEWLINE "\r\n"
#else
#define NEWLINE "\n"
#endif

// ✅ GCODE_Command, GCODE_CommandQueue now defined in data_structures.h

/* GCODE State Machine */
typedef enum {
    GCODE_STATE_IDLE,
    GCODE_STATE_CONTROL_CHAR,      // Real-time control chars: ?, ~, !, 0x18
    GCODE_STATE_QUERY_CHARS,       // Query commands: $, $$, $I, $G, etc.
    GCODE_STATE_GCODE_COMMAND,
    GCODE_STATE_ERROR
} GCODE_State;

typedef struct{
    GCODE_State state;
} GCODE_Data;

/* G-code Event System - Clean interface without APP_DATA exposure */
typedef enum {
    GCODE_EVENT_NONE,
    GCODE_EVENT_LINEAR_MOVE,
    GCODE_EVENT_ARC_MOVE,
    GCODE_EVENT_DWELL,
    GCODE_EVENT_SPINDLE_ON,
    GCODE_EVENT_SPINDLE_OFF,
    GCODE_EVENT_COOLANT_ON,
    GCODE_EVENT_COOLANT_OFF,
    GCODE_EVENT_SET_ABSOLUTE,
    GCODE_EVENT_SET_RELATIVE,
    GCODE_EVENT_SET_FEEDRATE,       // Standalone F command (modal)
    GCODE_EVENT_SET_SPINDLE_SPEED,  // Standalone S command (modal)
    GCODE_EVENT_SET_TOOL,           // Standalone T command (modal)
    GCODE_EVENT_SET_WORK_OFFSET,    // G92 - Set work coordinate system
    GCODE_EVENT_SET_WCS,            // G54-G59 - Select work coordinate system
    GCODE_EVENT_HOMING,             // $H - Homing cycle
    GCODE_EVENT_PROGRAM_END         // M0, M2, M30 - Program end/stop
} GCODE_EventType;

typedef struct {
    GCODE_EventType type;
    union {
        struct {
            float x, y, z, a;    // Target coordinates
            float feedrate;      // Feed rate in units/min
        } linearMove;
        
        struct {
            float centerX, centerY;  // Arc center relative to start
            float x, y, z, a;        // Target coordinates  
            bool clockwise;          // Direction
            float feedrate;          // Feed rate
        } arcMove;
        
        struct {
            float seconds;           // Dwell time
        } dwell;
        
        struct {
            uint32_t rpm;           // Spindle speed
        } spindle;
        
        struct {
            float feedrate;         // Modal feedrate setting
        } setFeedrate;
        
        struct {
            uint32_t rpm;           // Modal spindle speed setting
        } setSpindleSpeed;
        
        struct {
            uint32_t toolNumber;    // Tool number
        } setTool;
        
        struct {
            float x, y, z, a;       // Work offset coordinates (G92 or G10)
            uint32_t l_value;       // L parameter (2 or 20 for G10)
        } workOffset;
        
        struct {
            uint8_t wcs_number;     // Work coordinate system (0=G54, 1=G55, ..., 5=G59)
        } setWCS;
        
        struct {
            uint32_t axes_mask;     // Bitmap of axes to home (bit 0=X, 1=Y, 2=Z, 3=A)
        } homing;
    } data;
} GCODE_Event;

/* GCODE USART function prototypes */
void GCODE_USART_Initialize(uint32_t RD_thresholds);
void GCODE_Tasks(APP_DATA* appData, GCODE_CommandQueue* commandQueue);

// Soft reset function (handles Ctrl+X/0x18) - consolidates all reset logic
void GCODE_SoftReset(APP_DATA* appData, GCODE_CommandQueue* cmdQueue);
bool GCODE_GetNextEvent(GCODE_CommandQueue* cmdQueue, GCODE_Event* event);
void GCODE_ConsumeEvent(GCODE_CommandQueue* cmdQueue);  // Consume event after successful processing

// Check and send deferred "ok" responses (for flow control and startup deferral)
void GCODE_CheckDeferredOk(APP_DATA* appData, GCODE_CommandQueue* cmdQueue);

#endif // GCODE_PARSER_H