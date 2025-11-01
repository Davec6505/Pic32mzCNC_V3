#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"


typedef struct {
    // Bresenham parameters
    int32_t delta_x, delta_y, delta_z, delta_a;
    int32_t error_y, error_z, error_a;
    
    // Motion parameters
    uint32_t steps_remaining;
    uint32_t steps_completed;
    uint32_t step_interval;      // Current step interval (for velocity profiling)
    uint32_t pulse_width;        // Pulse width in timer ticks
    
    // Physics parameters (calculated by kinematics)
    float start_velocity;        // Starting velocity for this segment
    float max_velocity;          // Maximum velocity for this segment
    float end_velocity;          // Ending velocity for this segment
    float acceleration;          // Acceleration/deceleration rate
    
    // Axis management
    E_AXIS dominant_axis;        // Which axis drives the timing
} MotionSegment;

void MOTION_Initialize(void);
void MOTION_Tasks(MotionSegment motionQueue[], uint32_t* head, uint32_t* tail, uint32_t* count);



#endif /* MOTION_H */