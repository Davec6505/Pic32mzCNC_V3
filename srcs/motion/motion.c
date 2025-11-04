#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "common.h"
#include "data_structures.h"
#include "stepper.h"      // ✅ Include stepper.h BEFORE motion.h to ensure STEPPER_SetDirection is declared
#include "motion.h"
#include "kinematics.h"
#include "../config/default/peripheral/tmr/plib_tmr2.h"

// ============================================================================
// Motion State Machine (Non-Blocking Architecture)
// ============================================================================
// This state machine runs every APP_Tasks() iteration during motion.
// Static variables persist across scans (like ISR variables would).
// All motion logic runs here - ISRs only count positions.
// ============================================================================

typedef enum {
    MOTION_STATE_IDLE,              // Waiting for segment from queue
    MOTION_STATE_LOAD_SEGMENT,      // Load new segment and initialize
    MOTION_STATE_EXECUTING,         // Active motion with Bresenham + velocity profiling
    MOTION_STATE_SEGMENT_COMPLETE   // Segment done, move to next
} MotionState;

// ============================================================================
// Static Variables (Persist Across Scans - Like ISR Variables)
// ============================================================================

static MotionState motion_state = MOTION_STATE_IDLE;
static MotionSegment* current_segment = NULL;

// Bresenham error accumulators
static int32_t error_y = 0;
static int32_t error_z = 0;
static int32_t error_a = 0;

// Velocity profiling state
static uint32_t current_step_interval = 0;   // Timer ticks between steps
static uint32_t steps_completed = 0;         // Steps executed in current segment
static uint32_t next_step_time = 0;          // Absolute TMR2 value for next step

// ============================================================================
// Initialization
// ============================================================================

void MOTION_Initialize(void) {
    motion_state = MOTION_STATE_IDLE;
    current_segment = NULL;
    
    // Initialize stepper system
    STEPPER_Initialize();
}

// ============================================================================
// Motion Tasks - Non-Blocking State Machine (Runs Every APP_Tasks Iteration)
// ============================================================================

void MOTION_Tasks(MotionSegment motionQueue[], uint32_t* head, uint32_t* tail, uint32_t* count) {
    
    switch(motion_state) {
        // ====================================================================
        // IDLE: Wait for segment from queue
        // ====================================================================
        case MOTION_STATE_IDLE:
            if (*count > 0) {
                // Segment available - transition to load state
                motion_state = MOTION_STATE_LOAD_SEGMENT;
            }
            break;
            
        // ====================================================================
        // LOAD_SEGMENT: Initialize new segment for execution
        // ====================================================================
        case MOTION_STATE_LOAD_SEGMENT:
        {
            // Get segment from tail of circular buffer
            current_segment = &motionQueue[*tail];
            
            // Initialize Bresenham error accumulators
            // Errors start at delta/2 for symmetric rounding
            error_y = current_segment->delta_x / 2;
            error_z = current_segment->delta_x / 2;
            error_a = current_segment->delta_x / 2;
            
            // Initialize velocity profiling
            current_step_interval = current_segment->initial_rate;
            steps_completed = 0;
            
            // Set directions for all axes based on delta signs
            // Positive delta = forward (true), negative = reverse (false)
            STEPPER_SetDirection(AXIS_X, (bool)(current_segment->delta_x >= 0));
            STEPPER_SetDirection(AXIS_Y, (bool)(current_segment->delta_y >= 0));
            STEPPER_SetDirection(AXIS_Z, (bool)(current_segment->delta_z >= 0));
            STEPPER_SetDirection(AXIS_A, (bool)(current_segment->delta_a >= 0));
            
            // Schedule first dominant axis step
            uint32_t now = TMR2;
            next_step_time = now + current_step_interval;
            STEPPER_ScheduleStep(current_segment->dominant_axis, next_step_time);
            
            // Transition to executing state
            motion_state = MOTION_STATE_EXECUTING;
            break;
        }
            
        // ====================================================================
        // EXECUTING: Bresenham + Velocity Profiling (NON-BLOCKING)
        // ====================================================================
        case MOTION_STATE_EXECUTING:
        {
            // Poll timer - check if time for next step
            uint32_t now = TMR2;
            
            // CRITICAL: Use signed comparison for timer wraparound handling
            if ((int32_t)(now - next_step_time) >= 0) {
                
                // ============================================================
                // VELOCITY PROFILING (Trapezoidal)
                // ============================================================
                // Update step interval based on acceleration phase
                
                if (steps_completed < current_segment->accelerate_until) {
                    // ACCELERATION PHASE
                    // Increase speed (decrease interval)
                    // TODO: Use precomputed rate table from segment
                    current_step_interval = current_segment->initial_rate - 
                        (steps_completed * current_segment->rate_delta);
                    
                } else if (steps_completed >= current_segment->decelerate_after) {
                    // DECELERATION PHASE
                    // Decrease speed (increase interval)
                    uint32_t decel_steps = steps_completed - current_segment->decelerate_after;
                    current_step_interval = current_segment->nominal_rate + 
                        (decel_steps * current_segment->rate_delta);
                    
                } else {
                    // CRUISE PHASE
                    // Constant speed
                    current_step_interval = current_segment->nominal_rate;
                }
                
                // ============================================================
                // BRANCHLESS BRESENHAM (MIPS Optimized)
                // ============================================================
                // Accumulate errors and schedule subordinate axes
                // Branchless version eliminates branch misprediction penalties
                
                // Y-axis Bresenham
                error_y += abs(current_segment->delta_y);
                bool y_step = (error_y >= abs(current_segment->delta_x));
                if (y_step) {
                    STEPPER_ScheduleStep(AXIS_Y, next_step_time + 10);  // Slight offset to spread pulses
                    error_y -= abs(current_segment->delta_x);
                } else {
                    STEPPER_DisableAxis(AXIS_Y);  // OCxR = OCxRS (no pulse)
                }
                
                // Z-axis Bresenham
                error_z += abs(current_segment->delta_z);
                bool z_step = (error_z >= abs(current_segment->delta_x));
                if (z_step) {
                    STEPPER_ScheduleStep(AXIS_Z, next_step_time + 15);  // Slight offset
                    error_z -= abs(current_segment->delta_x);
                } else {
                    STEPPER_DisableAxis(AXIS_Z);
                }
                
                // A-axis Bresenham
                error_a += abs(current_segment->delta_a);
                bool a_step = (error_a >= abs(current_segment->delta_x));
                if (a_step) {
                    STEPPER_ScheduleStep(AXIS_A, next_step_time + 20);  // Slight offset
                    error_a -= abs(current_segment->delta_x);
                } else {
                    STEPPER_DisableAxis(AXIS_A);
                }
                
                // ============================================================
                // SCHEDULE NEXT DOMINANT AXIS STEP
                // ============================================================
                
                steps_completed++;
                current_segment->steps_remaining--;
                
                if (current_segment->steps_remaining > 0) {
                    // More steps remaining - schedule next
                    next_step_time += current_step_interval;
                    STEPPER_ScheduleStep(current_segment->dominant_axis, next_step_time);
                    
                } else {
                    // Segment complete
                    motion_state = MOTION_STATE_SEGMENT_COMPLETE;
                }
            }
            // If not time for next step yet, exit and check again next iteration
            break;
        }
            
        // ====================================================================
        // SEGMENT_COMPLETE: Finish current segment, move to next
        // ====================================================================
        case MOTION_STATE_SEGMENT_COMPLETE:
        {
            // Disable all axes (stop pulse generation)
            STEPPER_DisableAxis(AXIS_X);
            STEPPER_DisableAxis(AXIS_Y);
            STEPPER_DisableAxis(AXIS_Z);
            STEPPER_DisableAxis(AXIS_A);
            
            // Remove segment from queue
            *tail = (*tail + 1) % MAX_MOTION_SEGMENTS;
            (*count)--;
            
            // Check if more segments available
            if (*count > 0) {
                // Load next segment immediately (no gap in motion)
                motion_state = MOTION_STATE_LOAD_SEGMENT;
            } else {
                // No more segments - return to idle
                current_segment = NULL;
                motion_state = MOTION_STATE_IDLE;
            }
            break;
        }
    }
}