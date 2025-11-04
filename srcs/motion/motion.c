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
#include "settings.h"
#include "../config/default/peripheral/tmr/plib_tmr2.h"  // ✅ TMR2 PLIB access
#include <math.h>

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
}

// ============================================================================
// Motion Tasks - Non-Blocking State Machine (Runs Every APP_Tasks Iteration)
// ============================================================================

void MOTION_SegmentTasks(MotionSegment motionQueue[], uint32_t* head, uint32_t* tail, uint32_t* count) {
    
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
            uint32_t now = TMR2_CounterGet();  // ✅ Use PLIB function
            next_step_time = now + current_step_interval;
            STEPPER_ScheduleStep(current_segment->dominant_axis, next_step_time);
            
            // Transition to executing state
            motion_state = MOTION_STATE_EXECUTING;
            break;
        }
            
        // ====================================================================
        // EXECUTING: Bresenham + Velocity Profil
        case MOTION_STATE_EXECUTING:
        {
            // Poll timer - check if time for next step
            uint32_t now = TMR2_CounterGet();  // ✅ Use PLIB function
            
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

void MOTION_Tasks(APP_DATA* appData) {
    // ===== SEGMENT LOADING (HIGHEST PRIORITY - BEFORE PHASE PROCESSING) =====
    // Load next segment from queue if current segment is NULL
    if(appData->currentSegment == NULL && appData->motionQueueCount > 0) {
        // Get segment from tail of circular buffer
        appData->currentSegment = &appData->motionQueue[appData->motionQueueTail];
        
        // Initialize Bresenham error accumulators to delta/2 for symmetric rounding
        int32_t dominant_delta = appData->currentSegment->delta_x;  // Assume X is dominant
        switch(appData->currentSegment->dominant_axis) {
            case AXIS_X: dominant_delta = appData->currentSegment->delta_x; break;
            case AXIS_Y: dominant_delta = appData->currentSegment->delta_y; break;
            case AXIS_Z: dominant_delta = appData->currentSegment->delta_z; break;
            case AXIS_A: dominant_delta = appData->currentSegment->delta_a; break;
            default: break;
        }
        
        appData->bresenham_error_y = dominant_delta / 2;
        appData->bresenham_error_z = dominant_delta / 2;
        appData->bresenham_error_a = dominant_delta / 2;
        
        // Initialize velocity profiling
        appData->currentStepInterval = appData->currentSegment->initial_rate;
        appData->currentSegment->steps_completed = 0;
        
        // Set directions for all axes
        STEPPER_SetDirection(AXIS_X, appData->currentSegment->delta_x >= 0);
        STEPPER_SetDirection(AXIS_Y, appData->currentSegment->delta_y >= 0);
        STEPPER_SetDirection(AXIS_Z, appData->currentSegment->delta_z >= 0);
        STEPPER_SetDirection(AXIS_A, appData->currentSegment->delta_a >= 0);
        
        // Update dominant axis tracker for ISR
        appData->dominantAxis = appData->currentSegment->dominant_axis;
        
        // ✅ Don't schedule here - let phase system handle it
        // Signal phase processing to begin (will schedule in MOTION_PHASE_SCHEDULE)
        appData->motionPhase = MOTION_PHASE_VELOCITY;
    }
    
    // ===== PHASE PROCESSING (RUNS WHEN SEGMENT IS LOADED) =====
    switch(appData->motionPhase) {
                case MOTION_PHASE_VELOCITY:
                {
                    // Phase 0: Velocity conditioning (highest priority)
                    // Update step interval based on acceleration/cruise/decel state
                    if(appData->currentSegment != NULL) {
                        MotionSegment* seg = appData->currentSegment;
                        
                        if(seg->steps_completed < seg->accelerate_until) {
                            // Acceleration phase - interval decreases (speed increases)
                            appData->currentStepInterval += seg->rate_delta;  // rate_delta is negative
                            if(appData->currentStepInterval < seg->nominal_rate) {
                                appData->currentStepInterval = seg->nominal_rate;  // Clamp to max speed
                            }
                        } else if(seg->steps_completed > seg->decelerate_after) {
                            // Deceleration phase - interval increases (speed decreases)
                            appData->currentStepInterval += seg->rate_delta;  // rate_delta is positive
                            if(appData->currentStepInterval > seg->final_rate) {
                                appData->currentStepInterval = seg->final_rate;  // Clamp to min speed
                            }
                        } else {
                            // Cruise phase - maintain nominal rate
                            appData->currentStepInterval = seg->nominal_rate;
                        }
                    }

                    appData->motionPhase = MOTION_PHASE_BRESENHAM;
                    // Fall through to next phase
                }
                    
                case MOTION_PHASE_BRESENHAM:
                {
                    // Phase 1: Bresenham error accumulation
                    // Determine which subordinate axes need steps
                    if(appData->currentSegment != NULL) {
                        MotionSegment* seg = appData->currentSegment;
                        
                        // Accumulate errors for subordinate axes (not dominant)
                        if(seg->dominant_axis != AXIS_Y && seg->delta_y != 0) {
                            appData->bresenham_error_y += seg->delta_y;
                        }
                        if(seg->dominant_axis != AXIS_Z && seg->delta_z != 0) {
                            appData->bresenham_error_z += seg->delta_z;
                        }
                        if(seg->dominant_axis != AXIS_A && seg->delta_a != 0) {
                            appData->bresenham_error_a += seg->delta_a;
                        }
                    }

                    appData->motionPhase = MOTION_PHASE_SCHEDULE;
                    // Fall through to next phase
                }
                    
                case MOTION_PHASE_SCHEDULE:
                {
                    // Phase 2: OCx register scheduling
                    // Write OCxR/OCxRS with absolute TMR2 values
                    if(appData->currentSegment != NULL) {
                        MotionSegment* seg = appData->currentSegment;
                        
                        // Get dominant axis delta for comparison
                        int32_t dominant_delta = 0;
                        switch(seg->dominant_axis) {
                            case AXIS_X: dominant_delta = seg->delta_x; break;
                            case AXIS_Y: dominant_delta = seg->delta_y; break;
                            case AXIS_Z: dominant_delta = seg->delta_z; break;
                            case AXIS_A: dominant_delta = seg->delta_a; break;
                            default: break;
                        }
                        
                        // Schedule dominant axis (always steps)
                        STEPPER_ScheduleStep(seg->dominant_axis, appData->currentStepInterval);
                        
                        // Schedule subordinate axes if Bresenham says they need a step
                        if(seg->dominant_axis != AXIS_Y && seg->delta_y != 0) {
                            if(appData->bresenham_error_y >= dominant_delta) {
                                STEPPER_ScheduleStep(AXIS_Y, appData->currentStepInterval);
                                appData->bresenham_error_y -= dominant_delta;
                            } else {
                                STEPPER_DisableAxis(AXIS_Y);  // No step this cycle
                            }
                        }
                        
                        if(seg->dominant_axis != AXIS_Z && seg->delta_z != 0) {
                            if(appData->bresenham_error_z >= dominant_delta) {
                                STEPPER_ScheduleStep(AXIS_Z, appData->currentStepInterval);
                                appData->bresenham_error_z -= dominant_delta;
                            } else {
                                STEPPER_DisableAxis(AXIS_Z);
                            }
                        }
                        
                        if(seg->dominant_axis != AXIS_A && seg->delta_a != 0) {
                            if(appData->bresenham_error_a >= dominant_delta) {
                                STEPPER_ScheduleStep(AXIS_A, appData->currentStepInterval);
                                appData->bresenham_error_a -= dominant_delta;
                            } else {
                                STEPPER_DisableAxis(AXIS_A);
                            }
                        }
                        
                        // ✅ NOTE: steps_completed is incremented by ISR, not here!
                        // This prevents double-incrementing and premature completion
                    }
                    
                    appData->motionPhase = MOTION_PHASE_COMPLETE;
                    // Fall through to next phase
                }
                    
                case MOTION_PHASE_COMPLETE:
                {
                    // Phase 3: Segment completion check
                    // Check if current segment done
                    if(appData->currentSegment != NULL) {
                        MotionSegment* seg = appData->currentSegment;
                        
                        if(seg->steps_completed >= seg->steps_remaining) {
                            // ✅ Segment complete - remove from queue
                            appData->motionQueueTail = (appData->motionQueueTail + 1) % MAX_MOTION_SEGMENTS;
                            appData->motionQueueCount--;
                            
                            // Reset Bresenham errors for next segment
                            appData->bresenham_error_y = 0;
                            appData->bresenham_error_z = 0;
                            appData->bresenham_error_a = 0;
                            
                            // Mark current segment as NULL so next one loads
                            appData->currentSegment = NULL;
                        }
                    }
                    
                    // Return to IDLE (will be re-triggered by ISR on next dominant axis step)
                    appData->motionPhase = MOTION_PHASE_IDLE;
                    break;  // Exit phase processing
                }
                    
                case MOTION_PHASE_IDLE:
                    // No motion processing needed - fall through to other tasks
                    break;
            }
 
}

void MOTION_Arc(APP_DATA* appData) {
    // Only generate if arc is active and motion queue has space
    if(appData->arcGenState != ARC_GEN_ACTIVE || appData->motionQueueCount >= MAX_MOTION_SEGMENTS) {
        return;
    }
    
    // Increment angle for next segment
    appData->arcTheta += appData->arcThetaIncrement;
    
    CoordinatePoint next;
    bool is_last_segment = false;
    
    // Determine if this is the final segment
    if(appData->arcClockwise) {
        is_last_segment = (appData->arcTheta <= appData->arcThetaEnd);
    } else {
        is_last_segment = (appData->arcTheta >= appData->arcThetaEnd);
    }
    
    if(is_last_segment) {
        // Use exact end point to prevent accumulated error
        next = appData->arcEndPoint;
        appData->arcGenState = ARC_GEN_IDLE;
        
    } else {
        // Calculate intermediate point using sin/cos (FPU accelerated)
        next.x = appData->arcCenter.x + appData->arcRadius * cosf(appData->arcTheta);
        next.y = appData->arcCenter.y + appData->arcRadius * sinf(appData->arcTheta);
        
        // Linear interpolation for Z and A axes (helical motion)
        float start_angle = atan2f(appData->arcCurrent.y - appData->arcCenter.y,
                                   appData->arcCurrent.x - appData->arcCenter.x);
        float end_angle = atan2f(appData->arcEndPoint.y - appData->arcCenter.y,
                                 appData->arcEndPoint.x - appData->arcCenter.x);
        
        float total_angle;
        if(appData->arcClockwise) {
            total_angle = (start_angle >= end_angle) ? 
                         (start_angle - end_angle) : 
                         (start_angle - end_angle + 2.0f * M_PI);
        } else {
            total_angle = (end_angle >= start_angle) ? 
                         (end_angle - start_angle) : 
                         (end_angle - start_angle + 2.0f * M_PI);
        }
        
        float progress = fabsf(appData->arcTheta - start_angle) / total_angle;
        
        next.z = appData->arcCurrent.z + (appData->arcEndPoint.z - appData->arcCurrent.z) * progress;
        next.a = appData->arcCurrent.a + (appData->arcEndPoint.a - appData->arcCurrent.a) * progress;
    }
    
    // Generate motion segment for this arc increment
    MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
    KINEMATICS_LinearMove(appData->arcCurrent, next, appData->arcFeedrate, segment);
    
    // Add to motion queue
    appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
    appData->motionQueueCount++;
    
    // Update current position
    appData->arcCurrent = next;
    
    // Update work coordinates when arc completes
    if(appData->arcGenState == ARC_GEN_IDLE) {
        appData->currentX = appData->arcEndPoint.x;
        appData->currentY = appData->arcEndPoint.y;
        appData->currentZ = appData->arcEndPoint.z;
        appData->currentA = appData->arcEndPoint.a;
    }
}