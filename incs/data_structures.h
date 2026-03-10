#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <stdint.h>
#include <stdbool.h>


// ============================================================================
// Motion Queue Structures
// ============================================================================

#define MAX_MOTION_SEGMENTS 16  // Lookahead buffer size (64 causes crash even with 128KB stack - needs investigation)

// ✅ Number of axes (must match E_AXIS enum in data_structures.h)
#define AXIS_COUNT 4

// Forward declaration for E_AXIS (defined in common.h)
typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_A,
    NUM_AXIS
} E_AXIS;

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
// ============================================================================
// GRBL-Style Segment Buffer Architecture (March 2, 2026)
// ============================================================================
// GRBL breaks each planner block into many small constant-velocity segments.
// The ISR executes pure Bresenham with a fixed step rate per segment.
// No acceleration profiling in ISR - all velocity changes are pre-computed.

#define SEGMENT_BUFFER_SIZE 16  // Ring buffer for small segments

// Stepper Block - Bresenham data for one planner block
// One st_block per planner block, contains step counts for Bresenham
typedef struct {
    uint32_t steps[NUM_AXIS];        // Total steps per axis for this planner block
    uint32_t step_event_count;       // Total dominant axis steps
    uint8_t direction_bits;          // Direction bits (1 bit per axis)
} StepperBlock;

// Step Segment - Small constant-velocity chunk
// Many segments per planner block, each with fixed PR4 value
typedef struct {
    uint16_t n_step;                 // Number of dominant-axis steps in this segment
    uint32_t step_interval;          // PR4 value (constant for this segment)
    uint8_t st_block_index;          // Index into stepper block buffer
} StepSegment;

// Motion Segment Structure (Planner Block)
// ============================================================================
// This is what the planner works with - full move with trapezoid profile

typedef struct {
    // Segment type
    SegmentType type;            // LINEAR, ARC, or DWELL
    
    // Dwell-specific parameters (only used when type == SEGMENT_TYPE_DWELL)
    uint32_t dwell_duration;     // Dwell duration in core timer ticks (100MHz)
    
    // ✅ ARRAY-BASED: Bresenham parameters (only used for LINEAR/ARC segments)
    int32_t delta[NUM_AXIS];     // Step deltas per axis [X, Y, Z, A]
    
    // Dominant axis (pre-calculated during motion planning)
    E_AXIS dominant_axis;        // Axis with largest delta (drives step timing)
    int32_t dominant_delta;      // Largest absolute delta value
    
    // Motion parameters
    uint32_t steps_remaining;    // Total steps for this planner block
    uint32_t pulse_width;        // Pulse width in timer ticks
    
    // ✅ Trapezoidal velocity profile parameters (GRBL-style)
    // Computed by KINEMATICS, used by segment generator (not ISR)
    float initial_speed;         // Starting speed (mm/s)
    float nominal_speed;         // Cruise speed (mm/s)
    float final_speed;           // Ending speed (mm/s)
    
    float accelerate_until;      // Distance to end accel (mm)
    float decelerate_after;      // Distance to start decel (mm)

    // Physics parameters
    float acceleration;          // Acceleration/deceleration rate (mm/sec²)
    float millimeters;           // Total segment distance (mm)

    // =========================================================================
    // GRBL-exact planner fields (March 2026)
    // =========================================================================
    float unit_vec[NUM_AXIS];           // Normalised direction unit vector
    float entry_speed_sqr;              // Planned entry speed² (mm/s)²
    float max_entry_speed_sqr;          // Max allowable entry speed²
    float max_junction_speed_sqr;       // Junction speed limit
    bool  speed_locked;                 // true = planner must not modify
} MotionSegment;

// ============================================================================
// G-code Queue Structures
// ============================================================================

#define GCODE_MAX_COMMANDS 64
#define GCODE_BUFFER_SIZE 80

typedef struct {
    char command[GCODE_BUFFER_SIZE];
} GCODE_Command;

typedef struct {
    GCODE_Command commands[GCODE_MAX_COMMANDS];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    
    // Monotonically increasing counter: incremented every time a command is
    // removed from the queue (consumed/discarded).  CheckDeferredOk uses the
    // delta of this value to release exactly one deferred "ok" per consumed
    // command — correct even when new commands arrive simultaneously
    // (which makes the raw q->count delta unreliable).
    uint32_t commands_consumed;

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
// Probe States (G38.x commands)
// ============================================================================

typedef enum {
    PROBE_STATE_IDLE,
    PROBE_STATE_MOVING,
    PROBE_STATE_TRIGGERED,
    PROBE_STATE_FAILED
} ProbeState;

// ============================================================================
// Application Data (Master Structure)
// ============================================================================
// stack size note: Keep this structure under 128KB, although MIPS can handle more,
// large arrays (motion queue, command queue) can bloat stack usage quickly.
typedef struct {
    APP_STATES state;
    
    // G-code command queue (with nested motion info for flow control)
    GCODE_CommandQueue gcodeCommandQueue;
    
    // Motion queue (Planner blocks)
    MotionSegment motionQueue[MAX_MOTION_SEGMENTS];
    uint32_t motionQueueHead;
    uint32_t motionQueueTail;
    uint32_t motionQueueCount;
    
    // =========================================================================
    // GRBL Segment Buffer - Small constant-velocity segments
    // =========================================================================
    StepperBlock stepperBlocks[MAX_MOTION_SEGMENTS];  // One per planner block
    StepSegment segmentBuffer[SEGMENT_BUFFER_SIZE];   // Ring buffer of small segments
    volatile uint8_t segmentBufferTail;               // ISR reads from tail
    uint8_t segmentBufferHead;                        // Main loop writes to head
    uint8_t segmentNextHead;                          // Next head position
    
    // Segment preparation state (exact port of GRBL's st_prep_t)
    uint8_t prep_st_block_index;      // Current stepper block being prepped
    float prep_mm_remaining;          // mm remaining in current block (counts DOWN from millimeters to 0); MUST be initialised to millimeters at block load
    float prep_current_speed;         // Current speed at end of last segment (mm/s)
    uint8_t prep_ramp_type;           // 0=accel, 1=cruise, 2=decel
    MotionSegment* prep_pl_block;     // Planner block being segmented
    // GRBL st_prep_t fields (added to match GRBL's st_prep_buffer exactly)
    float prep_steps_remaining;       // Float step count remaining (high precision, decrements)
    float prep_step_per_mm;           // steps/mm for current block
    float prep_req_mm_increment;      // Min mm to guarantee >= 1 step (1.25/step_per_mm)
    float prep_dt_remainder;          // Fractional step time carried from previous segment
    float prep_accelerate_until;      // mm_remaining at end of accel ramp (GRBL notation: counts down)
    float prep_decelerate_after;      // mm_remaining at start of decel ramp (GRBL notation: counts down)
    float prep_maximum_speed;         // Peak speed in block (nominal or triangle peak) (mm/s)
    float prep_mm_complete;           // Target mm_remaining (0.0 = run to end of block)
    float prep_exit_speed;            // Block exit speed (mm/s)
    
    // =========================================================================
    
    // ✅ ARRAY-BASED: Current position tracking (work coordinates) [X, Y, Z, A]
    float current[4];
    
    // Modal state (GRBL-style)
    float modalFeedrate;      // Last F value (mm/min)
    uint32_t modalSpindleRPM; // Last S value
    uint32_t modalToolNumber; // Last T value
    bool absoluteMode;        // G90/G91 state
    uint8_t activeWCS;        // Active work coordinate system (0=G54, 1=G55, ..., 5=G59)
    uint8_t modalPlane;       // G17=0 (XY), G18=1 (XZ), G19=2 (YZ)
    
    // Alarm state (GRBL safety)
    uint8_t alarmCode;        // 0=no alarm, 1=hard limit, 2=soft limit, 3=abort, 10=E-Stop
    volatile bool eStopTriggered;  // Set by E-Stop ISR (RF4/CN-F IPL7)
    
    E_AXIS dominantAxis;               // Which axis drives the step clock for current segment
    MotionSegment* currentSegment;     // Pointer to active segment being executed
    
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
    
    // ✅ Probe state (G38.x commands)
    ProbeState probeState;           // Current probe operation state
    bool probeSuccess;               // true if probe triggered, false if missed
    bool probeAlarmOnFail;           // true for G38.2/G38.4, false for G38.3/G38.5
    CoordinatePoint probePosition;   // Position where probe triggered

    // Homed state — set true when $H completes successfully, false on reset
    // Used to gate soft limits (soft limits only enforced when machine is homed)
    bool machine_homed;
} APP_DATA;

#endif // DATA_STRUCTURES_H
