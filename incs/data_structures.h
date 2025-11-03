#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Motion Queue Structures
// ============================================================================

#define MAX_MOTION_SEGMENTS 16  // Lookahead buffer size

// Forward declaration for E_AXIS (defined in common.h)
typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_A,
    NUM_AXIS
} E_AXIS;

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
} APP_DATA;

#endif // DATA_STRUCTURES_H
