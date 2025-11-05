#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Coordinate System Structures
// ============================================================================

typedef struct {
    float x, y, z, a;
} CoordinatePoint;

// ============================================================================
// Motion Queue Structures
// ============================================================================

#define MAX_MOTION_SEGMENTS 16  // Lookahead buffer size

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
// Motion Segment Structure
// ============================================================================

typedef struct {
    // Bresenham parameters
    int32_t delta_x, delta_y, delta_z, delta_a;
    int32_t error_y, error_z, error_a;
    
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
    
    // Axis management
    E_AXIS dominant_axis;        // Which axis drives the timing
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
    
    // ✅ Nested motion queue info for flow control (no circular dependency!)
    // Updated by app.c before calling GCODE_Tasks()
    uint32_t motionQueueCount;      // Current motion buffer occupancy
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
    
    // G-code command queue (with nested motion info for flow control)
    GCODE_CommandQueue gcodeCommandQueue;
    
    // Motion queue
    MotionSegment motionQueue[MAX_MOTION_SEGMENTS];
    uint32_t motionQueueHead;
    uint32_t motionQueueTail;
    uint32_t motionQueueCount;
    
    // Current position tracking (work coordinates)
    float currentX;
    float currentY;
    float currentZ;
    float currentA;
    
    // Modal state (GRBL-style)
    float modalFeedrate;      // Last F value (mm/min)
    uint32_t modalSpindleRPM; // Last S value
    uint32_t modalToolNumber; // Last T value
    bool absoluteMode;        // G90/G91 state
    uint8_t modalPlane;       // G17=0 (XY), G18=1 (XZ), G19=2 (YZ)
    
    // Alarm state (GRBL safety)
    uint8_t alarmCode;        // 0=no alarm, 1=hard limit, 2=soft limit, 3=abort, etc.
    
    // ✅ Motion phase system (priority-based scheduling)
    volatile MotionPhase motionPhase;  // Current phase (set by ISR, read by main)
    E_AXIS dominantAxis;               // Which axis is master for current segment
    uint32_t currentStepInterval;      // Current interval (changes with velocity profile)
    MotionSegment* currentSegment;     // Pointer to active segment being executed
    
    // Bresenham state (for phase processing)
    int32_t bresenham_error_y;
    int32_t bresenham_error_z;
    int32_t bresenham_error_a;
    
    // Rate limiting counters
    uint32_t uartPollCounter;          // Counter for UART polling (every ~10ms)
    
    // ✅ Arc generation state (non-blocking incremental streaming)
    ArcGenState arcGenState;
    float arcTheta;                    // Current angle (radians)
    float arcThetaEnd;                 // Target angle (radians)
    float arcThetaIncrement;           // Angle step per segment (radians)
    CoordinatePoint arcCenter;         // Arc center point (absolute)
    CoordinatePoint arcCurrent;        // Current position on arc
    float arcRadius;                   // Arc radius (mm)
    bool arcClockwise;                 // G2=true, G3=false
    uint8_t arcPlane;                  // G17=0 (XY), G18=1 (XZ), G19=2 (YZ)
    float arcFeedrate;                 // Arc feedrate (mm/min)
    CoordinatePoint arcEndPoint;       // Final arc destination
} APP_DATA;

#endif // DATA_STRUCTURES_H
