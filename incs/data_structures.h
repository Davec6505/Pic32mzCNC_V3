


#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

// ================= Axis Enum (must be first for struct usage) =====================
typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_A,
    NUM_AXIS
} E_AXIS;

// Standard includes
#include <stdint.h>
#include <stdbool.h>
#include "common.h"




// ================= Homing State Machine Types (moved from homing.h) =====================
typedef enum {
    HOMING_STATE_IDLE = 0,      // No homing in progress
    HOMING_STATE_SEEK,          // Fast approach to limit
    HOMING_STATE_LOCATE,        // Slow precision homing
    HOMING_STATE_PULLOFF,       // Retract from limit
    HOMING_STATE_COMPLETE,      // Homing successful
    HOMING_STATE_ALARM          // Homing failed
} HomingState;

typedef struct {
    HomingState state;
    E_AXIS current_axis;        // Axis currently being homed
    uint32_t axes_to_home;      // Bitmask of axes to home (bit 0=X, 1=Y, 2=Z, 3=A)
    uint32_t axes_homed;        // Bitmask of completed axes
    // Timing
    uint32_t debounce_start;    // CoreTimer value when limit triggered
    bool debouncing;            // True during debounce delay
    // Motion tracking
    bool motion_active;         // True when homing move in progress
    uint32_t alarm_code;        // Non-zero if homing failed
} HomingControl;
typedef struct {
    uint32_t current_rpm;
    bool is_running;
    uint16_t current_pwm_duty;
} SpindleState;

// ================= G-code Parser State (Centralized) ======================

#define GCODE_RX_BUFFER_SIZE 512
#define GCODE_TX_BUFFER_SIZE 1024

typedef struct {
    uint32_t okPendingCount;         // Flow control: count of deferred "ok" responses
    bool grblCheckMode;              // $C toggle
    bool grblAlarm;                  // $X clears alarm
    bool feedHoldActive;             // '!' feed hold, '~' resume
    char startupLines[2][GCODE_RX_BUFFER_SIZE]; // $N0 / $N1
    bool unitsInches;                // false=mm (G21), true=inches (G20)
    // Only protocol/flow-control state here. Buffers are file-local in gcode_parser.c
} GcodeParserState;
// Forward declaration for motion state
typedef enum {
    MOTION_STATE_IDLE,
    MOTION_STATE_LOAD_SEGMENT,
    MOTION_STATE_EXECUTING,
    MOTION_STATE_SEGMENT_COMPLETE
} MotionState;

#include <stdint.h>
#include <stdbool.h>


// ============================================================================
// Motion Queue Structures
// ============================================================================

#define MAX_MOTION_SEGMENTS 16  // Lookahead buffer size (64 causes crash even with 128KB stack - needs investigation)

// ✅ Number of axes (must match E_AXIS enum in data_structures.h)
#define AXIS_COUNT 4



// ============================================================================
// Motion Phase System (Priority-Based Task Scheduling)
// ============================================================================

typedef enum {
    MOTION_PHASE_IDLE = 255,        // No motion active - safe for G-code processing
    MOTION_PHASE_VELOCITY = 0,      // Highest priority - velocity conditioning
    MOTION_PHASE_BRESENHAM = 1,     // Bresenham error accumulation
    MOTION_PHASE_SCHEDULE = 2,      // OCx register scheduling
    MOTION_PHASE_COMPLETE = 3       // Segment completion check
} MotionPhase;

// ============================================================================
// Arc Generation State (Non-Blocking Incremental)
// ============================================================================

typedef enum {
    ARC_GEN_IDLE = 0,
    ARC_GEN_ACTIVE
} ArcGenState;

// ============================================================================
// Motion Segment Type
// ============================================================================

typedef enum {
    SEGMENT_TYPE_LINEAR = 0,    // G0/G1 linear motion
    SEGMENT_TYPE_ARC,           // G2/G3 arc motion
    SEGMENT_TYPE_DWELL          // G4 dwell (no motion, just timer)
} SegmentType;

// ============================================================================
// Coordinate System Structures
// ============================================================================

typedef struct {
    float coordinate[NUM_AXIS];
} CoordinatePoint;


// ============================================================================
// Motion Segment Structure
// ============================================================================

typedef struct {
    // Segment type
    SegmentType type;            // LINEAR, ARC, or DWELL
    
    // Dwell-specific parameters (only used when type == SEGMENT_TYPE_DWELL)
    uint32_t dwell_duration;     // Dwell duration in core timer ticks (100MHz)
    
    // ✅ ARRAY-BASED: Bresenham parameters (only used for LINEAR/ARC segments)
    int32_t delta[NUM_AXIS];     // Step deltas per axis [X, Y, Z, A]
    int32_t error[NUM_AXIS];     // Bresenham error accumulators [X, Y, Z, A]
    
    // Dominant axis (pre-calculated during motion planning)
    E_AXIS dominant_axis;        // Axis with largest delta (drives step timing)
    int32_t dominant_delta;      // Largest absolute delta value
    
    // Motion parameters
    uint32_t steps_remaining;
    uint32_t steps_completed;
    uint32_t step_interval;      // Current step interval (for velocity profiling)
    uint32_t pulse_width;        // Pulse width in timer ticks
    
    // ✅ Trapezoidal velocity profile parameters (GRBL-style)
    // These are pre-computed by KINEMATICS in main loop (floating point OK)
    // ISR uses these for integer-only segment conditioning
    uint32_t initial_rate;       // Starting step interval (slowest, timer ticks)
    uint32_t nominal_rate;       // Cruise step interval (fastest, timer ticks)
    uint32_t final_rate;         // Ending step interval (slowest, timer ticks)
    
    uint32_t accelerate_until;   // Step count to end acceleration phase
    uint32_t decelerate_after;   // Step count to start deceleration phase
    
    int32_t rate_delta;          // Interval change per step (signed, timer ticks)
                                 // Negative = accelerating (interval decreasing)
                                 // Positive = decelerating (interval increasing)
    
    // Physics parameters (calculated by kinematics for reference/debugging)
    float start_velocity;        // Starting velocity for this segment (mm/sec)
    float max_velocity;          // Maximum velocity for this segment (mm/sec)
    float end_velocity;          // Ending velocity for this segment (mm/sec)
    float acceleration;          // Acceleration/deceleration rate (mm/sec²)
} MotionSegment;

// ============================================================================
// G-code Queue Structures
// ============================================================================

#define GCODE_MAX_COMMANDS 16
#define GCODE_BUFFER_SIZE 80

typedef struct {
    char command[GCODE_BUFFER_SIZE];
} GCODE_Command;

typedef struct {
    GCODE_Command commands[GCODE_MAX_COMMANDS];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    
    // ✅ Motion queue info for flow control (no circular dependency!)
    // Flow control reads appData->motionQueueCount directly (single authoritative source)
    uint32_t maxMotionSegments;     // Maximum motion buffer size
} GCODE_CommandQueue;

// ============================================================================
// Application States
// ============================================================================

typedef enum {
    APP_CONFIG = 0,
    APP_LOAD_SETTINGS,
    APP_GCODE_INIT,
    APP_IDLE,
    APP_ALARM,                      // Emergency stop / hard limit triggered
    APP_WAIT_FOR_CONFIGURATION,
    APP_DEVICE_ATTACHED,
    APP_WAIT_FOR_DEVICE_ATTACH,
    APP_DEVICE_DETACHED,
    APP_ERROR
} APP_STATES;

// ============================================================================
// Application Data (Master Structure)
// ============================================================================

typedef struct {
    APP_STATES state;
    // G-code parser runtime state (centralized)
    GcodeParserState gcodeParser;
    
    // G-code command queue (with nested motion info for flow control)
    GCODE_CommandQueue gcodeCommandQueue;
    
    // Motion queue
    MotionSegment motionQueue[MAX_MOTION_SEGMENTS];
    uint32_t motionQueueHead;
    uint32_t motionQueueTail;
    uint32_t motionQueueCount;
    
    // ✅ ARRAY-BASED: Current position tracking (work coordinates) [X, Y, Z, A]
    float current[4];
    
    // Modal state (GRBL-style)
    float modalFeedrate;      // Last F value (mm/min)
    uint32_t modalSpindleRPM; // Last S value
    uint32_t modalToolNumber; // Last T value
    bool absoluteMode;        // G90/G91 state
    uint8_t activeWCS;        // Active work coordinate system (0=G54, 1=G55, ..., 5=G59)
    uint8_t modalPlane;       // G17=0 (XY), G18=1 (XZ), G19=2 (YZ)

    // Centralized spindle runtime state
    SpindleState spindleState;
    
    // Alarm state (GRBL safety)
    uint8_t alarmCode;        // 0=no alarm, 1=hard limit, 2=soft limit, 3=abort, etc.
    
    // ✅ Motion phase system (priority-based scheduling)
    volatile MotionPhase motionPhase;  // Current phase (set by ISR, read by main)
    E_AXIS dominantAxis;               // Which axis is master for current segment
    uint32_t currentStepInterval;      // Current interval (changes with velocity profile)
    MotionSegment* currentSegment;     // Pointer to active segment being executed

    // ===== MOTION STATE MACHINE (CENTRALIZED) =====
    MotionState motionState;           // State for MOTION_Tasks state machine
    uint32_t stepsCompleted;           // Steps executed in current segment (if needed)
    
    // ✅ ARRAY-BASED: Bresenham state (for phase processing) [X, Y, Z, A]
    // Note: All axes can have errors since ANY axis can be dominant based on plane selection
    int32_t bresenham_error[NUM_AXIS];
    
    // Rate limiting counters
    uint32_t uartPollCounter;          // Counter for UART polling (every ~10ms)
    
    // ✅ Arc generation state (non-blocking incremental streaming)
    ArcGenState arcGenState;
    float arcTheta;                    // Current angle (radians)
    float arcThetaStart;               // Initial start angle (radians, fixed)
    float arcThetaEnd;                 // Target angle (radians)
    float arcThetaIncrement;           // Angle step per segment (radians)
    uint32_t arcSegmentCurrent;        // Current segment number (0-based)
    uint32_t arcSegmentTotal;          // Total number of segments
    CoordinatePoint arcCenter;         // Arc center point (absolute)
    CoordinatePoint arcCurrent;        // Current position on arc
    CoordinatePoint arcStartPoint;     // Initial arc start point (for Z/A interpolation)
    float arcRadius;                   // Arc radius (mm)
    bool arcClockwise;                 // G2=true, G3=false
    uint8_t arcPlane;                  // G17=0 (XY), G18=1 (XZ), G19=2 (YZ)
    float arcFeedrate;                 // Arc feedrate (mm/min)
    CoordinatePoint arcEndPoint;       // Final arc destination
    
    // Runtime motion state flags
    // true when OC/TMR are running and motion is active; false when fully idle
    bool motionActive;
    // true when a segment completes (signals to check deferred ok)
    bool motionSegmentCompleted;
    
    // ===== STEPPER/BRESENHAM STATE (CENTRALIZED) =====
    // Stepper position tracking (for all axes)
    struct {
        int32_t steps[NUM_AXIS];
        float steps_per_mm[NUM_AXIS];
    } stepper_pos;

    // Direction bits (bitmask: 1=forward, 0=reverse)
    volatile uint8_t direction_bits;

    // Bresenham error and delta arrays
    volatile int32_t bresenham_error_isr[NUM_AXIS];
    volatile int32_t bresenham_delta_isr[NUM_AXIS];
    volatile int32_t bresenham_dominant_delta;
    volatile E_AXIS bresenham_dominant_axis;

    // Stepper enable state
    bool steppers_enabled;

    // Dwell timer state
    volatile bool dwell_active;
    volatile uint32_t dwell_start_ticks;
    volatile uint32_t dwell_duration;

    // Cached settings for ISR performance
    uint8_t step_pulse_invert_mask;
    uint8_t direction_invert_mask;
    uint8_t enable_invert;
    // ===== HOMING STATE (CENTRALIZED) =====
    HomingControl homing;
} APP_DATA;

#endif // DATA_STRUCTURES_H
