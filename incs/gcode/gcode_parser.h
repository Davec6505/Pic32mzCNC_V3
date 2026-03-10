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

/* Load startup lines ($N0/$N1) from settings into the parser.
   If non-empty, they will be automatically injected at next GCODE_Tasks call. */
void GCODE_LoadStartupLines(const char* l0, const char* l1);

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
    GCODE_EVENT_PROGRAM_END,        // M0, M2, M30 - Program end/stop
    GCODE_EVENT_PROBE_TOWARD,       // G38.2, G38.3 - Probe toward workpiece
    GCODE_EVENT_PROBE_AWAY,         // G38.4, G38.5 - Probe away from workpiece
    GCODE_EVENT_TLO_SET,            // G43, G43.1   - Activate tool length offset
    GCODE_EVENT_TLO_CANCEL,         // G49           - Cancel tool length offset
    GCODE_EVENT_CANNED_DRILL,       // G81           - Simple drill cycle
    GCODE_EVENT_CANNED_PECK,        // G83           - Peck drilling cycle
    GCODE_EVENT_CANNED_CANCEL,      // G80           - Cancel canned cycle
    GCODE_EVENT_JOG                 // $J=           - Jog move (bypasses command queue)
} GCODE_EventType;

typedef struct {
    GCODE_EventType type;
    union {
        struct {
            float x, y, z, a;    // Target coordinates
            float feedrate;      // Feed rate in units/min (0 = use modal/rapid)
            bool  isRapid;       // true = G0 rapid (use max_rate), false = G1 feed
            bool  is_jog;        // true = jog move (0x85 cancellable, no junction blend)
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
            uint32_t l_value;        // L parameter (2 or 20 for G10)
            uint8_t  wcs_number;     // Target WCS: 0-5 = G54-G59; 255 = use current active (P0)
        } workOffset;
        
        struct {
            uint8_t wcs_number;     // Work coordinate system (0=G54, 1=G55, ..., 5=G59)
        } setWCS;
        
        struct {
            uint32_t axes_mask;     // Bitmap of axes to home (bit 0=X, 1=Y, 2=Z, 3=A)
        } homing;

        struct {
            float value;    // TLO offset in mm (+ = tool longer than reference)
            bool  dynamic;  // true = G43.1 (inline value), false = G43 (from settings/tool table)
        } tlo;

        struct {
            float x, y, z, a;       // Target coordinates for probe move
            float feedrate;         // Probe feedrate
            bool alarm_on_fail;     // true for G38.2/G38.4, false for G38.3/G38.5
            bool probe_toward;      // true = toward (G38.2/G38.3), false = away (G38.4/G38.5)
        } probe;

        struct {
            float x, y;         // Hole position (work coords)
            float z;            // Drill depth (work coord, typically negative)
            float r;            // R-plane: rapid-to clearance height (work coord)
            float q;            // G83 only: peck increment (positive, mm)
            float feedrate;     // Drill feedrate (mm/min)
            uint32_t l;         // Repeat count (0 or 1 = one hole, >1 = multiple)
            bool g98;           // true = retract to initial Z (G98), false = retract to R (G99)
        } cannedDrill;
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