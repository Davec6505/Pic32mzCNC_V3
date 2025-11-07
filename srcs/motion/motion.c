#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "common.h"
#include "data_structures.h"
#include "stepper.h"      // Include stepper.h BEFORE motion.h to ensure STEPPER_SetDirection is declared
#include "motion.h"
#include "kinematics.h"
#include "settings.h"
#include "utils/uart_utils.h"  // Required for DEBUG_PRINT_XXX macros
#include "../config/default/peripheral/tmr/plib_tmr4.h"  // TMR4 PLIB access (16-bit timer for OC1)
#include "../config/default/peripheral/gpio/plib_gpio.h"  // Required for DEBUG_EXEC_XXX LED toggles

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

// Velocity profiling state (Bresenham now in ISR)
static uint32_t current_step_interval = 0;   // Timer ticks between steps
static uint32_t steps_completed = 0;         // Steps executed in current segment

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
            
            // ✅ NEW ARCHITECTURE: Load entire segment into stepper module
            // This sets directions, Bresenham errors, deltas, and initial PR2
            STEPPER_LoadSegment(current_segment);
            
            // Initialize local velocity state
            current_step_interval = current_segment->initial_rate;
            steps_completed = 0;
            
            // Transition to executing state
            motion_state = MOTION_STATE_EXECUTING;
            break;
        }
            
        // ====================================================================
        // EXECUTING: Monitor segment completion and update velocity (PR2)
        // ====================================================================
        case MOTION_STATE_EXECUTING:
        {
            // Check if segment is complete (ISR updates steps_completed)
            if (current_segment->steps_completed >= current_segment->steps_remaining) {
                motion_state = MOTION_STATE_SEGMENT_COMPLETE;
                break;
            }
            
            // ============================================================
            // VELOCITY PROFILING - Update PR2 based on acceleration state
            // ============================================================
            uint32_t new_rate;
            
            if (current_segment->steps_completed < current_segment->accelerate_until) {
                // ACCELERATION PHASE - decrease rate (increase speed)
                new_rate = current_segment->initial_rate - 
                    (current_segment->steps_completed * current_segment->rate_delta);
                if (new_rate < current_segment->nominal_rate) {
                    new_rate = current_segment->nominal_rate;
                }
                
            } else if (current_segment->steps_completed >= current_segment->decelerate_after) {
                // DECELERATION PHASE - increase rate (decrease speed)
                uint32_t decel_steps = current_segment->steps_completed - current_segment->decelerate_after;
                new_rate = current_segment->nominal_rate + 
                    (decel_steps * current_segment->rate_delta);
                if (new_rate > current_segment->final_rate) {
                    new_rate = current_segment->final_rate;
                }
                
            } else {
                // CRUISE PHASE - constant speed
                new_rate = current_segment->nominal_rate;
            }
            
            // Update step rate if changed
            if (new_rate != current_step_interval) {
                current_step_interval = new_rate;
                STEPPER_SetStepRate(new_rate);  // Updates PR2
            }
            
            break;
        }
            
        // ====================================================================
        // SEGMENT_COMPLETE: Dequeue and move to next segment or idle
        // ====================================================================
        case MOTION_STATE_SEGMENT_COMPLETE:
        {
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
    // ===== SIMPLIFIED MOTION CONTROL (NEW ARCHITECTURE) =====
    // This function is called continuously from APP_Tasks (main loop).
    // It manages segment loading and monitoring - all real-time work happens in OCP1_ISR!
    
    // ===== 1. SEGMENT LOADING =====
    // Load new segment when currentSegment is NULL and queue has data
    if(appData->currentSegment == NULL && appData->motionQueueCount > 0) {
        // Get segment from tail of circular buffer
        appData->currentSegment = &appData->motionQueue[appData->motionQueueTail];

        // If this is a zero-length segment (no steps), skip it gracefully
        if (appData->currentSegment->steps_remaining == 0) {
            // Dequeue and move to next segment
            appData->motionQueueTail = (appData->motionQueueTail + 1) % MAX_MOTION_SEGMENTS;
            appData->motionQueueCount--;
            appData->currentSegment = NULL;
            return;  // Try again next iteration
        }
        
        // ✅ Load segment into ISR state (this is where the magic happens!)
        STEPPER_LoadSegment(appData->currentSegment);
        
        // Update dominant axis tracker for ISR
        appData->dominantAxis = appData->currentSegment->dominant_axis;
        
        // Set initial step rate (PR4 controls OC1 period)
        STEPPER_SetStepRate(appData->currentSegment->initial_rate);
        
        // ===== DEBUG: Segment Loading =====
        // This code ONLY appears in debug builds when DEBUG_SEGMENT is enabled
        // In release builds, the compiler completely removes this code (zero overhead)
        DEBUG_PRINT_SEGMENT("[SEGMENT] Loaded: steps=%lu, rate=%lu\r\n", 
                            appData->currentSegment->steps_remaining,
                            appData->currentSegment->initial_rate);
        DEBUG_EXEC_SEGMENT(LED1_Set());  // Visual indicator: segment loaded
        
        return;  // Exit and let ISR start stepping
    }
    
    // ===== 2. VELOCITY PROFILING (DURING MOTION) =====
    // Update step rate based on acceleration/cruise/decel
    if(appData->currentSegment != NULL) {
        MotionSegment* seg = appData->currentSegment;
        
        // Simple velocity profiling (can be enhanced with real trapezoid later)
        uint32_t new_rate = seg->nominal_rate;  // For now, use constant cruise rate
        
        // TODO: Implement acceleration/deceleration based on steps_completed
        // if (seg->steps_completed < seg->accelerate_until) {
        //     new_rate = calculate_accel_rate(seg);
        // } else if (seg->steps_completed > seg->decelerate_after) {
        //     new_rate = calculate_decel_rate(seg);
        // }
        
        // Update PR2 if rate changed (non-blocking, single register write)
        if(new_rate != appData->currentStepInterval) {
            STEPPER_SetStepRate(new_rate);
            appData->currentStepInterval = new_rate;
        }
    }
    
    // ===== 3. SEGMENT COMPLETION CHECK =====
    // Check if current segment is done (ISR updates steps_completed)
    if(appData->currentSegment != NULL) {
        MotionSegment* seg = appData->currentSegment;
        
        if(seg->steps_completed >= seg->steps_remaining) {
            // ===== DEBUG: Segment Completion =====
            DEBUG_PRINT_SEGMENT("[SEGMENT] Complete: %lu steps\r\n", seg->steps_completed);
            DEBUG_EXEC_SEGMENT(LED1_Clear());  // Visual indicator: segment done
            
            // Segment complete - remove from queue
            appData->motionQueueTail = (appData->motionQueueTail + 1) % MAX_MOTION_SEGMENTS;
            appData->motionQueueCount--;
            
            // Mark current segment as NULL so next one loads
            appData->currentSegment = NULL;
            
            // Stop TMR4 when ALL motion is complete (motion queue empty)
            // TMR4 runs continuously across segments for smooth multi-segment motion
            // Only stop when the complete distance has been reached (no more segments)
            if(appData->motionQueueCount == 0) {
                TMR4_Stop();
            }
        }
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



// Add this new function after the existing functions

/**
 * @brief Process a single G-code event and convert to motion segments
 * @param appData Pointer to application data structure
 * @param event Pointer to G-code event to process
 * @return true if event was processed successfully, false if motion queue full or error
 */
bool MOTION_ProcessGcodeEvent(APP_DATA* appData, GCODE_Event* event) {
    
    DEBUG_PRINT_MOTION("[MOTION] ProcessEvent: type=%d\r\n", event->type);
    
    switch (event->type) {
        case GCODE_EVENT_LINEAR_MOVE:
        {
            DEBUG_PRINT_MOTION("[MOTION] Linear move event\r\n");
            
            // Check if motion queue has space before processing
            if(appData->motionQueueCount >= MAX_MOTION_SEGMENTS) {
                DEBUG_PRINT_MOTION("[MOTION] Queue full!\r\n");
                return false;  // Queue full - try again later
            }
            
            // Build start and end coordinates
            CoordinatePoint start = {appData->currentX, appData->currentY, appData->currentZ, appData->currentA};
            CoordinatePoint end;
            
            if(appData->absoluteMode) {
                // G90 absolute mode - use coordinates directly (or current if NAN)
                end.x = isnan(event->data.linearMove.x) ? appData->currentX : event->data.linearMove.x;
                end.y = isnan(event->data.linearMove.y) ? appData->currentY : event->data.linearMove.y;
                end.z = isnan(event->data.linearMove.z) ? appData->currentZ : event->data.linearMove.z;
                end.a = isnan(event->data.linearMove.a) ? appData->currentA : event->data.linearMove.a;
            } else {
                // G91 relative mode - add to current position (or keep current if NAN)
                end.x = isnan(event->data.linearMove.x) ? appData->currentX : (appData->currentX + event->data.linearMove.x);
                end.y = isnan(event->data.linearMove.y) ? appData->currentY : (appData->currentY + event->data.linearMove.y);
                end.z = isnan(event->data.linearMove.z) ? appData->currentZ : (appData->currentZ + event->data.linearMove.z);
                end.a = isnan(event->data.linearMove.a) ? appData->currentA : (appData->currentA + event->data.linearMove.a);
            }
            
            // ✅ SOFT LIMIT CHECK - TEMPORARILY DISABLED FOR DEBUG
            /*
            GRBL_Settings* settings = SETTINGS_GetCurrent();
            bool limit_violation = false;
            
            if(!isnan(settings->max_travel_x) && !isnan(settings->max_travel_y) && !isnan(settings->max_travel_z)) {
                if(fabsf(end.x) > settings->max_travel_x) limit_violation = true;
                if(fabsf(end.y) > settings->max_travel_y) limit_violation = true;
                if(fabsf(end.z) > settings->max_travel_z) limit_violation = true;
            }
            
            if(limit_violation) {
                appData->alarmCode = 2;  // Soft limit alarm
                appData->state = APP_ALARM;
                return false;
            }
            */
            
            // Use feedrate from event, or modal feedrate if not specified
            float feedrate = event->data.linearMove.feedrate;
            if (feedrate == 0.0f) {
                // If modal is also zero (e.g., after reset), apply a safe default (600 mm/min)
                if (appData->modalFeedrate <= 0.0f) {
                    appData->modalFeedrate = 600.0f;
                }
                feedrate = appData->modalFeedrate;
            } else {
                appData->modalFeedrate = feedrate;
            }
            
            // Get next queue slot
            MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
            
            DEBUG_PRINT_MOTION("[MOTION] Calling KINEMATICS_LinearMove: start=(%.2f,%.2f), end=(%.2f,%.2f), F=%.1f\r\n",
                              start.x, start.y, end.x, end.y, feedrate);
            
            // Convert to motion segment
            KINEMATICS_LinearMove(start, end, feedrate, segment);

            DEBUG_PRINT_MOTION("[MOTION] Segment: steps=%lu, dx=%ld, dy=%ld, dz=%ld\r\n",
                              segment->steps_remaining, segment->delta_x, segment->delta_y, segment->delta_z);

            // Guard: skip zero-length segments (no steps)
            if (segment->steps_remaining == 0) {
                DEBUG_PRINT_MOTION("[MOTION] Zero-length segment - skipping\r\n");
                // Update current position and treat as no-op
                appData->currentX = end.x;
                appData->currentY = end.y;
                appData->currentZ = end.z;
                appData->currentA = end.a;
                return true;
            }

            DEBUG_PRINT_MOTION("[MOTION] Adding segment to queue (count will be %d)\r\n", appData->motionQueueCount + 1);
            
            // Add to motion queue
            appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
            appData->motionQueueCount++;
            
            // Update current position
            appData->currentX = end.x;
            appData->currentY = end.y;
            appData->currentZ = end.z;
            appData->currentA = end.a;
            
            return true;
        }
        
        case GCODE_EVENT_ARC_MOVE:
        {
            // ✅ ARC MOTION - Initialize arc generation (KEEP ALL EXISTING ARC CODE!)
            CoordinatePoint start = {appData->currentX, appData->currentY, appData->currentZ, appData->currentA};
            CoordinatePoint end;
            
            if(appData->absoluteMode) {
                end.x = event->data.arcMove.x;
                end.y = event->data.arcMove.y;
                end.z = event->data.arcMove.z;
                end.a = event->data.arcMove.a;
            } else {
                end.x = appData->currentX + event->data.arcMove.x;
                end.y = appData->currentY + event->data.arcMove.y;
                end.z = appData->currentZ + event->data.arcMove.z;
                end.a = appData->currentA + event->data.arcMove.a;
            }
            
            CoordinatePoint center;
            center.x = start.x + event->data.arcMove.centerX;
            center.y = start.y + event->data.arcMove.centerY;
            center.z = start.z;
            center.a = 0.0f;
            
            // Radius validation
            float r_start = sqrtf(event->data.arcMove.centerX * event->data.arcMove.centerX + 
                                 event->data.arcMove.centerY * event->data.arcMove.centerY);
            float r_end = sqrtf((end.x - center.x) * (end.x - center.x) + 
                               (end.y - center.y) * (end.y - center.y));
            
            if(fabsf(r_start - r_end) > 0.005f) {
                appData->alarmCode = 33;  // Arc radius error
                appData->state = APP_ALARM;
                return false;
            }
            
            // Calculate angles
            float start_angle = atan2f(start.y - center.y, start.x - center.x);
            float end_angle = atan2f(end.y - center.y, end.x - center.x);
            
            float total_angle;
            if(event->data.arcMove.clockwise) {
                total_angle = start_angle - end_angle;
                if(total_angle <= 0.0f) total_angle += 2.0f * M_PI;
            } else {
                total_angle = end_angle - start_angle;
                if(total_angle <= 0.0f) total_angle += 2.0f * M_PI;
            }
            
            // Initialize arc state
            appData->arcGenState = ARC_GEN_ACTIVE;
            appData->arcTheta = start_angle;
            appData->arcThetaEnd = end_angle;
            appData->arcCenter.x = center.x;
            appData->arcCenter.y = center.y;
            appData->arcCurrent = start;
            appData->arcEndPoint = end;
            appData->arcRadius = r_start;
            appData->arcClockwise = event->data.arcMove.clockwise;
            appData->arcPlane = appData->modalPlane;
            appData->arcFeedrate = event->data.arcMove.feedrate;
            
            GRBL_Settings* settings = SETTINGS_GetCurrent();
            float arc_length = r_start * total_angle;
            uint32_t num_segments = (uint32_t)ceilf(arc_length / settings->mm_per_arc_segment);
            if(num_segments < 1) num_segments = 1;
            appData->arcThetaIncrement = total_angle / (float)num_segments;
            if(!event->data.arcMove.clockwise) {
                appData->arcThetaIncrement = fabsf(appData->arcThetaIncrement);
            } else {
                appData->arcThetaIncrement = -fabsf(appData->arcThetaIncrement);
            }
            
            return true;
        }
        
        case GCODE_EVENT_SPINDLE_ON:
            appData->modalSpindleRPM = event->data.spindle.rpm;
            // TODO: Spindle hardware control
            return true;
            
        case GCODE_EVENT_SPINDLE_OFF:
            appData->modalSpindleRPM = 0;
            // TODO: Spindle hardware control
            return true;
            
        case GCODE_EVENT_SET_ABSOLUTE:
            appData->absoluteMode = true;
            return true;
            
        case GCODE_EVENT_SET_RELATIVE:
            appData->absoluteMode = false;
            return true;
            
        case GCODE_EVENT_SET_WORK_OFFSET:
        {
            // G92 - Set work coordinate offset so current machine position = specified work position
            // Formula: offset = MPos - desired_WPos
            // Example: If MPos=(29.975, 7.500) and we want WPos=(0,0), then offset=(29.975, 7.500)
            // Note: Only updates axes that are specified (non-NaN values)
            
            StepperPosition* pos = STEPPER_GetPosition();
            WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();
            
            // Get current machine position
            float mpos_x = (float)pos->x_steps / pos->steps_per_mm_x;
            float mpos_y = (float)pos->y_steps / pos->steps_per_mm_y;
            float mpos_z = (float)pos->z_steps / pos->steps_per_mm_z;
            
            // Calculate new offsets (only for specified axes)
            float offset_x = wcs->offset.x;  // Keep existing
            float offset_y = wcs->offset.y;  // Keep existing
            float offset_z = wcs->offset.z;  // Keep existing
            
            // Update work position and offset for each specified axis
            if (!isnan(event->data.setWorkOffset.x)) {
                offset_x = mpos_x - event->data.setWorkOffset.x;
                appData->currentX = event->data.setWorkOffset.x;
            }
            if (!isnan(event->data.setWorkOffset.y)) {
                offset_y = mpos_y - event->data.setWorkOffset.y;
                appData->currentY = event->data.setWorkOffset.y;
            }
            if (!isnan(event->data.setWorkOffset.z)) {
                offset_z = mpos_z - event->data.setWorkOffset.z;
                appData->currentZ = event->data.setWorkOffset.z;
            }
            if (!isnan(event->data.setWorkOffset.a)) {
                appData->currentA = event->data.setWorkOffset.a;
            }
            
            // Set the work coordinate offset
            KINEMATICS_SetWorkCoordinates(offset_x, offset_y, offset_z);
            
            return true;
        }
            
        case GCODE_EVENT_SET_FEEDRATE:
            appData->modalFeedrate = event->data.setFeedrate.feedrate;
            return true;
            
        case GCODE_EVENT_SET_SPINDLE_SPEED:
            appData->modalSpindleRPM = event->data.setSpindleSpeed.rpm;
            return true;
            
        case GCODE_EVENT_SET_TOOL:
            appData->modalToolNumber = event->data.setTool.toolNumber;
            // TODO: Tool change logic
            return true;
            
        case GCODE_EVENT_DWELL:
            // TODO: Implement dwell
            return true;
            
        case GCODE_EVENT_COOLANT_ON:
            // TODO: Coolant control
            return true;
            
        case GCODE_EVENT_COOLANT_OFF:
            // TODO: Coolant control
            return true;
            
        case GCODE_EVENT_NONE:
        default:
            return true;  // Ignore unknown events
    }
}
