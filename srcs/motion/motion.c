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
#include "spindle.h"      // Spindle PWM control
#include "homing.h"       // Homing system control
#include "settings.h"
#include "utils/utils.h"       // For AxisConfig and UTILS_GetAxisConfig
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
            // ✅ DWELL SEGMENT: Check timer instead of steps
            if (current_segment->type == SEGMENT_TYPE_DWELL) {
                if (STEPPER_IsDwellComplete()) {
                    DEBUG_PRINT_MOTION("[COMPLETE] Dwell segment done\r\n");
                    motion_state = MOTION_STATE_SEGMENT_COMPLETE;
                }
                break;  // Don't check steps or velocity for dwell
            }
            
            // ✅ MOTION SEGMENT (LINEAR/ARC): Check steps and update velocity
            // ✅ DEBUG: Check completion status periodically
            static uint32_t debug_check_counter = 0;
            if (++debug_check_counter >= 1000) {
                debug_check_counter = 0;
                DEBUG_PRINT_MOTION("[EXECUTING] steps_completed=%lu, steps_remaining=%lu\r\n",
                    current_segment->steps_completed, current_segment->steps_remaining);
            }
            
            // Check if segment is complete (ISR updates steps_completed)
            if (current_segment->steps_completed >= current_segment->steps_remaining) {
                DEBUG_PRINT_MOTION("[COMPLETE] Segment done: %lu >= %lu\r\n",
                    current_segment->steps_completed, current_segment->steps_remaining);
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
    
    // ⚠️ DEBUG REMOVED - was flooding UART, causing system freeze
    // Only print when actually loading a segment (see below)
    
    // ===== 1. SEGMENT LOADING =====
    // Load new segment when currentSegment is NULL and queue has data
    if(appData->currentSegment == NULL && appData->motionQueueCount > 0) {
        
        // DEBUG_PRINT_MOTION("[MOTION_Tasks] Loading segment: queue=%lu\r\n", appData->motionQueueCount);
        // Get segment from tail of circular buffer
        appData->currentSegment = &appData->motionQueue[appData->motionQueueTail];

        // If this is a zero-length segment (no steps), skip it gracefully
        if (appData->currentSegment->steps_remaining == 0) {
            // Dequeue and move to next segment
            appData->motionQueueTail = (appData->motionQueueTail + 1) % MAX_MOTION_SEGMENTS;
            appData->motionQueueCount--;
            appData->currentSegment = NULL;
            
            // ✅ Signal that queue space became available
            appData->motionSegmentCompleted = true;
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
        
        // ✅ TRAPEZOIDAL VELOCITY PROFILING
        uint32_t new_rate;
        
        if(seg->steps_completed < seg->accelerate_until) {
            // ✅ ACCELERATION PHASE
            // Rate decreases (interval gets shorter, speed increases)
            // Guard against underflow from multiplication overflow
            int32_t rate_change = (int32_t)seg->steps_completed * abs(seg->rate_delta);
            if (rate_change > (int32_t)seg->initial_rate) {
                new_rate = seg->nominal_rate;  // Clamp to nominal if underflow would occur
            } else {
                new_rate = seg->initial_rate - rate_change;
                
                // Clamp to nominal rate (don't overshoot)
                if(new_rate < seg->nominal_rate) {
                    new_rate = seg->nominal_rate;
                }
            }
            
        } else if(seg->steps_completed > seg->decelerate_after) {
            // ✅ DECELERATION PHASE
            // Rate increases (interval gets longer, speed decreases)
            uint32_t decel_steps = seg->steps_completed - seg->decelerate_after;
            int32_t rate_change = (int32_t)decel_steps * abs(seg->rate_delta);
            
            new_rate = seg->nominal_rate + rate_change;
            
            // Clamp to final rate (don't slow below minimum)
            if(new_rate > seg->final_rate) {
                new_rate = seg->final_rate;
            }
            
        } else {
            // ✅ CRUISE PHASE (constant velocity)
            new_rate = seg->nominal_rate;
        }
        
        // Update PR2 if rate changed (non-blocking, single register write)
        if(new_rate != appData->currentStepInterval) {
            STEPPER_SetStepRate(new_rate);
            appData->currentStepInterval = new_rate;
            
            // ⚠️ DISABLED: Too verbose - floods UART and causes crashes
            // DEBUG_PRINT_MOTION("[MOTION] Rate update: %lu (steps=%lu, phase=%s)\r\n", 
            //                  new_rate, seg->steps_completed,
            //                  (seg->steps_completed < seg->accelerate_until) ? "ACCEL" :
            //                  (seg->steps_completed > seg->decelerate_after) ? "DECEL" : "CRUISE");
        }
    }
    
    // ===== 3. SEGMENT COMPLETION CHECK =====
    // Check if current segment is done (ISR updates steps_completed)
    if(appData->currentSegment != NULL) {
        MotionSegment* seg = appData->currentSegment;
        
        // ✅ DEBUG: Periodic completion check
        static uint32_t completion_check_counter = 0;
        if (++completion_check_counter >= 5000) {
            completion_check_counter = 0;
            DEBUG_PRINT_MOTION("[CHECK] steps_completed=%lu, steps_remaining=%lu\r\n",
                seg->steps_completed, seg->steps_remaining);
        }
        
        if(seg->steps_completed >= seg->steps_remaining) {
            // ===== DEBUG: Segment Completion =====
            DEBUG_PRINT_MOTION("[SEGMENT] Complete: %lu >= %lu steps\r\n", 
                seg->steps_completed, seg->steps_remaining);
            DEBUG_EXEC_SEGMENT(LED1_Clear());  // Visual indicator: segment done
            
            // Segment complete - remove from queue
            appData->motionQueueTail = (appData->motionQueueTail + 1) % MAX_MOTION_SEGMENTS;
            appData->motionQueueCount--;
            
            // ✅ Signal that queue space became available
            appData->motionSegmentCompleted = true;
            
            // Mark current segment as NULL so next one loads
            appData->currentSegment = NULL;
            
            // Stop TMR4 when ALL motion is complete (motion queue empty)
            // TMR4 runs continuously across segments for smooth multi-segment motion
            // Only stop when the complete distance has been reached (no more segments)
            if(appData->motionQueueCount == 0) {
                TMR4_Stop();
                appData->motionActive = false;  // Mark motion fully stopped
            }
        }
    }
}
void MOTION_Arc(APP_DATA* appData) {
    // DEBUG_PRINT_MOTION("[ARC] Entry: state=%d, queueCount=%d\r\n", 
    //                   appData->arcGenState, appData->motionQueueCount);
    
    // Only generate if arc is active and motion queue has space
    if(appData->arcGenState != ARC_GEN_ACTIVE || appData->motionQueueCount >= MAX_MOTION_SEGMENTS) {
        // DEBUG_PRINT_MOTION("[ARC] Blocked: state=%d, queueCount=%d\r\n", 
        //                   appData->arcGenState, appData->motionQueueCount);
        return;
    }
    
    // DEBUG_PRINT_MOTION("[ARC] Generating segment: seg=%lu/%lu, theta=%.3f\r\n",
    //                   appData->arcSegmentCurrent, appData->arcSegmentTotal, appData->arcTheta);
    
    CoordinatePoint next;
    bool is_last_segment = false;
    
    // Check if this is the last segment based on segment counter
    is_last_segment = (appData->arcSegmentCurrent >= (appData->arcSegmentTotal - 1));
    
    if(is_last_segment) {
        // Use exact end point to prevent accumulated error
        next = appData->arcEndPoint;
        appData->arcGenState = ARC_GEN_IDLE;
        // DEBUG_PRINT_MOTION("[ARC] ✅ COMPLETE: Arc finished, state now IDLE\r\n");
        // DEBUG_PRINT_MOTION("[ARC] COMPLETE: Final segment, exact end=(%.2f,%.2f)\r\n",
        //                   GET_COORDINATE_AXIS(&next, AXIS_X), GET_COORDINATE_AXIS(&next, AXIS_Y));
        
    } else {
        // ✅ CRITICAL FIX: Increment angle BEFORE calculating next position
        // Otherwise first segment calculates at START theta → zero steps!
        appData->arcTheta += appData->arcThetaIncrement;
        
        // Calculate intermediate point using NEXT theta
        SET_COORDINATE_AXIS(&next, AXIS_X, 
            GET_COORDINATE_AXIS(&appData->arcCenter, AXIS_X) + appData->arcRadius * cosf(appData->arcTheta));
        SET_COORDINATE_AXIS(&next, AXIS_Y, 
            GET_COORDINATE_AXIS(&appData->arcCenter, AXIS_Y) + appData->arcRadius * sinf(appData->arcTheta));
        
        // Linear interpolation for Z and A axes (helical motion)
        // Calculate progress based on segment count (more reliable than angles)
        float progress = 0.0f;
        if (appData->arcSegmentTotal > 1) {
            progress = (float)appData->arcSegmentCurrent / (float)(appData->arcSegmentTotal - 1);
        }
        
        // Interpolate Z and A from start to end
        SET_COORDINATE_AXIS(&next, AXIS_Z,
            GET_COORDINATE_AXIS(&appData->arcStartPoint, AXIS_Z) + 
            (GET_COORDINATE_AXIS(&appData->arcEndPoint, AXIS_Z) - GET_COORDINATE_AXIS(&appData->arcStartPoint, AXIS_Z)) * progress);
        SET_COORDINATE_AXIS(&next, AXIS_A,
            GET_COORDINATE_AXIS(&appData->arcStartPoint, AXIS_A) + 
            (GET_COORDINATE_AXIS(&appData->arcEndPoint, AXIS_A) - GET_COORDINATE_AXIS(&appData->arcStartPoint, AXIS_A)) * progress);
    }
    
    // Generate motion segment for this arc increment
    MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
    
    // DEBUG_PRINT_MOTION("[ARC] Before LinearMoveSimple: current=(%.2f,%.2f), next=(%.2f,%.2f)\r\n",
    //                   appData->arcCurrent.x, appData->arcCurrent.y, next.x, next.y);
    
    // Use simple linear move for arc segments (no junction planning needed for smooth arcs)
    KINEMATICS_LinearMoveSimple(appData->arcCurrent, next, appData->arcFeedrate, segment);
    
    // DEBUG_PRINT_MOTION("[ARC] After LinearMoveSimple\r\n");
    
    // Add to motion queue
    appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
    appData->motionQueueCount++;
    
    // Update current position (arc internal state)
    appData->arcCurrent = next;
    
    // ✅ CRITICAL: Update work coordinates INCREMENTALLY for each segment
    // This ensures current[] reflects actual position even during long arcs
    // Required for multi-segment arcs where next command may arrive before completion
    // ✅ ARRAY-BASED: Copy all axes using loop
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        appData->current[axis] = GET_COORDINATE_AXIS(&next, axis);
    }
    
    // Increment segment counter
    appData->arcSegmentCurrent++;
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
    
    // Handle non-motion events (G20/G21 units, etc.) - no motion generated
    if (event->type == GCODE_EVENT_NONE) {
        DEBUG_PRINT_MOTION("[MOTION] NONE event - no motion\r\n");
        return true;  // Acknowledged, no motion needed
    }
    
    switch (event->type) {
        case GCODE_EVENT_LINEAR_MOVE:
        {
            DEBUG_PRINT_MOTION("[MOTION] Linear move event\r\n");
            
            // Check if motion queue has space before processing
            if (appData->motionQueueCount >= MAX_MOTION_SEGMENTS) {
                // DEBUG_PRINT_MOTION("[MOTION] Queue full! Skipping segment (will be lost)\r\n");
                return false;  // Queue full - event already consumed, segment lost
            }
            
            // ✅ ARRAY-BASED: Build start coordinate from current position
            CoordinatePoint start = {{
                appData->current[AXIS_X], appData->current[AXIS_Y],
                appData->current[AXIS_Z], appData->current[AXIS_A]
            }};
            CoordinatePoint end;
            
            // ✅ ARRAY-BASED: Build end coordinate with absolute/relative mode handling
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                float event_value;
                // Extract event value based on axis (still from old event structure for now)
                if (axis == AXIS_X) event_value = event->data.linearMove.x;
                else if (axis == AXIS_Y) event_value = event->data.linearMove.y;
                else if (axis == AXIS_Z) event_value = event->data.linearMove.z;
                else event_value = event->data.linearMove.a;
                
                // ✅ DEBUG: Show Z-axis event values
                if (axis == AXIS_Z) {
                    DEBUG_PRINT_MOTION("[MOTION] Z: event_value=%.3f current=%.3f mode=%s\r\n",
                        event_value, appData->current[AXIS_Z], 
                        appData->absoluteMode ? "ABS" : "REL");
                }
                
                if (appData->absoluteMode) {
                    // G90 absolute mode - use coordinates directly (or current if NAN)
                    SET_COORDINATE_AXIS(&end, axis, 
                        isnan(event_value) ? appData->current[axis] : event_value);
                } else {
                    // G91 relative mode - add to current position (or keep current if NAN)
                    SET_COORDINATE_AXIS(&end, axis,
                        isnan(event_value) ? appData->current[axis] : (appData->current[axis] + event_value));
                }
                
                // ✅ DEBUG: Show final Z coordinate
                if (axis == AXIS_Z) {
                    DEBUG_PRINT_MOTION("[MOTION] Z: target=%.3f\r\n", GET_COORDINATE_AXIS(&end, axis));
                }
            }
            
            // ✅ SOFT LIMIT CHECK - TEMPORARILY DISABLED FOR DEBUG
            /*
            CNC_Settings* settings = SETTINGS_GetCurrent();
            bool limit_violation = false;
            
            // ✅ ARRAY-BASED: Check soft limits (loop for scalability - only X/Y/Z, not A)
            float end_coords[3] = {end.x, end.y, end.z};  // Map to array for loop
            for (E_AXIS axis = AXIS_X; axis <= AXIS_Z; axis++) {
                if (!isnan(settings->max_travel[axis])) {
                    if (fabsf(end_coords[axis]) > settings->max_travel[axis]) {
                        limit_violation = true;
                        break;
                    }
                }
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
            
            // ✅ CRITICAL: Check if motion queue has space
            if (appData->motionQueueCount >= MAX_MOTION_SEGMENTS) {
                DEBUG_PRINT_MOTION("[LINEAR] Motion queue full (%lu/%d), deferring command\r\n",
                                  (unsigned long)appData->motionQueueCount, MAX_MOTION_SEGMENTS);
                return false;  // Queue full, leave event in queue for retry
            }
            
            // Get next queue slot
            MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
            
            // DEBUG_PRINT_MOTION("[MOTION] Calling KINEMATICS_LinearMove: start=(%.2f,%.2f), end=(%.2f,%.2f), F=%.1f\r\n",
            //                   start.x, start.y, end.x, end.y, feedrate);
            
            // ===== JUNCTION PLANNING: Calculate entry and exit velocities =====
            float entry_velocity = 8.33f;  // Default: start from minimum speed (500 mm/min)
            float exit_velocity = 8.33f;   // Default: decelerate to stop
            
            // Check if we can apply junction deviation with previous segment
            if (appData->motionQueueCount > 0) {
                // Get previous segment for junction analysis
                uint32_t prev_index = (appData->motionQueueHead - 1 + MAX_MOTION_SEGMENTS) % MAX_MOTION_SEGMENTS;
                MotionSegment* prev_segment = &appData->motionQueue[prev_index];
                
                // Calculate normalized direction vectors
                if (prev_segment->dominant_delta > 0) {
                    // ✅ ARRAY-BASED: Previous direction vector from segment deltas
                    CoordinatePoint prev_dir;
                    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                        SET_COORDINATE_AXIS(&prev_dir, axis, 
                            (float)prev_segment->delta[axis] / (float)prev_segment->dominant_delta);
                    }
                    
                    // We need current segment's direction, so do a preview calculation
                    float delta_mm[NUM_AXIS];
                    float distance = 0.0f;
                    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                        delta_mm[axis] = GET_COORDINATE_AXIS(&end, axis) - GET_COORDINATE_AXIS(&start, axis);
                        distance += delta_mm[axis] * delta_mm[axis];
                    }
                    distance = sqrtf(distance);
                    
                    if (distance > 0.001f) {  // Avoid division by zero
                        // ✅ ARRAY-BASED: Current direction vector
                        CoordinatePoint curr_dir;
                        for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                            SET_COORDINATE_AXIS(&curr_dir, axis, delta_mm[axis] / distance);
                        }
                        
                        // Get settings for junction calculation
                        CNC_Settings* settings = SETTINGS_GetCurrent();
                        float junction_speed = KINEMATICS_CalculateJunctionSpeed(prev_dir, curr_dir, 
                                                                                settings->junction_deviation,
                                                                                *(g_axis_settings[AXIS_X].acceleration));  // Use X axis for junction accel
                        
                        // Limit junction speed to reasonable maximum (feedrate / 60)
                        float max_junction = feedrate / 60.0f;  // Convert mm/min to mm/sec
                        if (junction_speed > max_junction) {
                            junction_speed = max_junction;
                        }
                        
                        // Update previous segment's exit velocity for smoother transition
                        entry_velocity = junction_speed;
                        
                        // DEBUG_PRINT_MOTION("[JUNCTION] Junction speed: %.1f mm/sec (entry to current segment)\r\n", 
                        //                   junction_speed);
                    }
                }
            }
            
            // Convert to motion segment with junction velocities
            KINEMATICS_LinearMove(start, end, feedrate, segment, entry_velocity, exit_velocity);

            // DEBUG_PRINT_MOTION("[MOTION] Segment: steps=%lu, dx=%ld, dy=%ld, dz=%ld\r\n",
            //                   segment->steps_remaining, segment->delta_x, segment->delta_y, segment->delta_z);

            // Guard: skip zero-length segments (no steps)
            if (segment->steps_remaining == 0) {
                // DEBUG_PRINT_MOTION("[MOTION] Zero-length segment - skipping\r\n");
                // ✅ ARRAY-BASED: Update current position and treat as no-op
                for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                    appData->current[axis] = GET_COORDINATE_AXIS(&end, axis);
                }
                return true;
            }

            // DEBUG_PRINT_MOTION("[MOTION] Adding segment to queue (count will be %d)\r\n", appData->motionQueueCount + 1);
            
            // Add to motion queue
            appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
            appData->motionQueueCount++;
            
            // ✅ ARRAY-BASED: Update current position using coordinate utilities
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                appData->current[axis] = GET_COORDINATE_AXIS(&end, axis);
            }
            
            return true;
        }
        
        case GCODE_EVENT_ARC_MOVE:
        {
            // ✅ ARC MOTION - Initialize arc generation (KEEP ALL EXISTING ARC CODE!)
            CoordinatePoint start = {{appData->current[AXIS_X], appData->current[AXIS_Y], appData->current[AXIS_Z], appData->current[AXIS_A]}};
            CoordinatePoint end;
            
            // ✅ ARRAY-BASED: Build end coordinate (still using old event structure temporarily)
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                float event_value;
                if (axis == AXIS_X) event_value = event->data.arcMove.x;
                else if (axis == AXIS_Y) event_value = event->data.arcMove.y;
                else if (axis == AXIS_Z) event_value = event->data.arcMove.z;
                else event_value = event->data.arcMove.a;
                
                if (appData->absoluteMode) {
                    // G90 absolute mode - use event value or current if NAN
                    SET_COORDINATE_AXIS(&end, axis, 
                        isnan(event_value) ? appData->current[axis] : event_value);
                } else {
                    // G91 relative mode - add to current or keep current if NAN
                    SET_COORDINATE_AXIS(&end, axis,
                        isnan(event_value) ? appData->current[axis] : (appData->current[axis] + event_value));
                }
            }
            
            // Calculate arc center using loop for scalability
            CoordinatePoint center;
            float center_offsets[NUM_AXIS] = {
                event->data.arcMove.centerX,  // X offset
                event->data.arcMove.centerY,  // Y offset
                0.0f,                          // Z (no offset)
                0.0f                           // A (no offset)
            };
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                SET_COORDINATE_AXIS(&center, axis, 
                    GET_COORDINATE_AXIS(&start, axis) + center_offsets[axis]);
            }
            
            // GRBL v1.1 radius validation with automatic compensation
            // Calculate radius from both start and end points
            float r_start = sqrtf(event->data.arcMove.centerX * event->data.arcMove.centerX + 
                                 event->data.arcMove.centerY * event->data.arcMove.centerY);
            float r_end = sqrtf(
                (GET_COORDINATE_AXIS(&end, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X)) * 
                (GET_COORDINATE_AXIS(&end, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X)) + 
                (GET_COORDINATE_AXIS(&end, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y)) * 
                (GET_COORDINATE_AXIS(&end, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y)));
            
            float radius_error = fabsf(r_start - r_end);
            
            // Get arc tolerance from settings (GRBL $13)
            CNC_Settings* settings = SETTINGS_GetCurrent();
            float arc_tolerance = settings->arc_tolerance;
            
            // Check if radius error exceeds tolerance
            if(radius_error > arc_tolerance) {
                DEBUG_PRINT_MOTION("[MOTION] ⚠️ ARC RADIUS ERROR: r_start=%.3f, r_end=%.3f, diff=%.3f (tolerance=%.3f)\r\n",
                                  r_start, r_end, radius_error, arc_tolerance);
                UART_Printf("error:33\r\n");  // GRBL error code for arc radius error
                appData->alarmCode = 33;  // Arc radius error
                appData->state = APP_ALARM;
                return false;
            }
            
            // GRBL radius compensation: Use average radius to compensate for CAM rounding errors
            float radius = (r_start + r_end) / 2.0f;
            
            // DEBUG_PRINT_MOTION("[ARC INIT] Radius compensation: r_start=%.4f, r_end=%.4f, avg=%.4f, error=%.4f\r\n",
            //                   r_start, r_end, radius, radius_error);
            
            // Calculate angles
            float start_angle = atan2f(
                GET_COORDINATE_AXIS(&start, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y), 
                GET_COORDINATE_AXIS(&start, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X));
            float end_angle = atan2f(
                GET_COORDINATE_AXIS(&end, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y), 
                GET_COORDINATE_AXIS(&end, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X));
            
            // DEBUG_PRINT_MOTION("[ARC INIT] Start=(%.2f,%.2f) End=(%.2f,%.2f) Center=(%.2f,%.2f)\r\n",
            //                   GET_COORDINATE_AXIS(&start, AXIS_X), GET_COORDINATE_AXIS(&start, AXIS_Y),
            //                   GET_COORDINATE_AXIS(&end, AXIS_X), GET_COORDINATE_AXIS(&end, AXIS_Y),
            //                   GET_COORDINATE_AXIS(&center, AXIS_X), GET_COORDINATE_AXIS(&center, AXIS_Y));
            // DEBUG_PRINT_MOTION("[ARC INIT] Direction=%s Radius=%.4f (compensated)\r\n",
            //                   event->data.arcMove.clockwise ? "CW(G2)" : "CCW(G3)", radius);
            // DEBUG_PRINT_MOTION("[ARC INIT] Start_angle=%.3f End_angle=%.3f\r\n",
            //                   start_angle, end_angle);
            
            float total_angle;
            if(event->data.arcMove.clockwise) {
                total_angle = start_angle - end_angle;
                if(total_angle <= 0.0f) total_angle += 2.0f * M_PI;
            } else {
                total_angle = end_angle - start_angle;
                if(total_angle <= 0.0f) total_angle += 2.0f * M_PI;
            }
            
            // DEBUG_PRINT_MOTION("[ARC INIT] Total_angle=%.3f rad (%.1f deg)\r\n",
            //                   total_angle, total_angle * 180.0f / M_PI);
            
            // Initialize arc state
            appData->arcGenState = ARC_GEN_ACTIVE;
            // DEBUG_PRINT_MOTION("[ARC] ✅ NEW ARC: Initialized, state now ACTIVE\r\n");
            appData->arcTheta = start_angle;
            appData->arcThetaStart = start_angle;  // Store initial angle for progress calculation
            appData->arcThetaEnd = end_angle;
            appData->arcCenter = center;  // Direct copy - both are CoordinatePoint
            appData->arcCurrent = start;
            appData->arcStartPoint = start;  // Store initial position for Z/A interpolation
            appData->arcEndPoint = end;
            appData->arcRadius = radius;  // Use compensated average radius
            appData->arcClockwise = event->data.arcMove.clockwise;
            appData->arcPlane = appData->modalPlane;
            appData->arcFeedrate = event->data.arcMove.feedrate;
            
            // ✅ REMOVED: Don't reset accumulators - arc segments MUST accumulate fractional steps!
            // Each 0.1mm segment may be <1 step, but fractional steps accumulate across all segments
            // KINEMATICS_ResetAccumulators();  // ❌ REMOVED - caused zero-step arc segments
            
            // Calculate arc length and number of segments (reuse settings from radius check above)
            float arc_length = radius * total_angle;  // Use compensated radius
            uint32_t num_segments = (uint32_t)ceilf(arc_length / settings->mm_per_arc_segment);
            if(num_segments < 2) num_segments = 2;  // Minimum 2 segments to prevent division by zero
            
            // DEBUG_PRINT_MOTION("[ARC INIT] Arc_length=%.2f mm, Segments=%lu, mm_per_segment=%.3f\r\n",
            //                   arc_length, (unsigned long)num_segments, settings->mm_per_arc_segment);
            
            appData->arcSegmentCurrent = 0;      // Start at segment 0
            appData->arcSegmentTotal = num_segments;  // Store total for termination check
            
            appData->arcThetaIncrement = total_angle / (float)num_segments;
            if(!event->data.arcMove.clockwise) {
                appData->arcThetaIncrement = fabsf(appData->arcThetaIncrement);
            } else {
                appData->arcThetaIncrement = -fabsf(appData->arcThetaIncrement);
            }
            
            // DEBUG_PRINT_MOTION("[ARC INIT] Theta_increment=%.4f rad, Total segments=%lu\r\n",
            //                   appData->arcThetaIncrement, (unsigned long)num_segments);
            
            return true;
        }
        
        case GCODE_EVENT_SPINDLE_ON:
            appData->modalSpindleRPM = event->data.spindle.rpm;
            SPINDLE_SetSpeed(event->data.spindle.rpm);
            SPINDLE_Start();
            return true;
            
        case GCODE_EVENT_SPINDLE_OFF:
            appData->modalSpindleRPM = 0;
            SPINDLE_Stop();
            return true;
            
        case GCODE_EVENT_SET_ABSOLUTE:
            appData->absoluteMode = true;
            return true;
            
        case GCODE_EVENT_SET_RELATIVE:
            appData->absoluteMode = false;
            return true;
            
        case GCODE_EVENT_SET_WCS:
        {
            // G54-G59 - Select active work coordinate system
            uint8_t wcs_number = event->data.setWCS.wcs_number;
            if (wcs_number <= 5) {  // G54=0 through G59=5
                appData->activeWCS = wcs_number;
                // No need to save to flash - modal state persists during session only
            }
            return true;
        }
            
        case GCODE_EVENT_SET_WORK_OFFSET:
        {
            // G10 L2/L20 or G92 - Set work coordinate system offset
            StepperPosition* pos = STEPPER_GetPosition();
            CNC_Settings* settings = SETTINGS_GetCurrent();
            WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();
            
            // ✅ ARRAY-BASED: Calculate machine position (loop for scalability - only X/Y/Z)
            float mpos[3];  // X, Y, Z machine positions
            for (E_AXIS axis = AXIS_X; axis <= AXIS_Z; axis++) {
                mpos[axis] = (float)pos->steps[axis] / settings->steps_per_mm[axis];
            }
            
            // ✅ ARRAY-BASED: Set work offset for all axes (only X/Y/Z, not A)
            float offset_values[3] = {
                event->data.workOffset.x,
                event->data.workOffset.y,
                event->data.workOffset.z
            };
            
            for (E_AXIS axis = AXIS_X; axis <= AXIS_Z; axis++) {
                if (!isnan(offset_values[axis])) {
                    SET_COORDINATE_AXIS(&wcs->offset, axis, mpos[axis] - offset_values[axis]);
                    appData->current[axis] = offset_values[axis];
                }
            }
            
            return true;
        }
        
        case GCODE_EVENT_SET_FEEDRATE:
            // Feedrate is handled in modal state
            return true;
            
        case GCODE_EVENT_SET_SPINDLE_SPEED:
            // Update modal spindle speed and apply if running
            appData->modalSpindleRPM = event->data.setSpindleSpeed.rpm;
            SPINDLE_SetSpeed(event->data.setSpindleSpeed.rpm);
            return true;
            
        case GCODE_EVENT_HOMING:
        {
            // Start homing cycle
            if (HOMING_Start(appData, event->data.homing.axes_mask)) {
                DEBUG_PRINT_MOTION("[MOTION] Homing cycle started for axes mask: 0x%02X\r\n", 
                                  event->data.homing.axes_mask);
                return true;
            } else {
                DEBUG_PRINT_MOTION("[MOTION] Homing cycle failed to start\r\n");
                return false;
            }
        }
        
        case GCODE_EVENT_DWELL:
        {
            // G4 P<seconds> - Create a DWELL segment in the motion queue
            // Convert seconds to core timer ticks (core timer runs at CPU_FREQ/2 = 100MHz)
            float seconds = event->data.dwell.seconds;
            uint32_t ticks = (uint32_t)(seconds * 100000000.0f);  // 100MHz core timer
            
            // Check if motion queue has space
            if (appData->motionQueueCount >= MAX_MOTION_SEGMENTS) {
                DEBUG_PRINT_MOTION("[DWELL] Motion queue full, cannot enqueue dwell\r\n");
                return false;
            }
            
            // Create DWELL segment
            MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
            memset(segment, 0, sizeof(MotionSegment));  // Zero all fields
            
            segment->type = SEGMENT_TYPE_DWELL;
            segment->dwell_duration = ticks;
            
            // Enqueue segment
            appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
            appData->motionQueueCount++;
            
            DEBUG_PRINT_MOTION("[DWELL] Enqueued %.3f second dwell segment (%lu ticks)\r\n", seconds, (unsigned long)ticks);
            return true;
        }
        case GCODE_EVENT_COOLANT_ON:
        case GCODE_EVENT_COOLANT_OFF:
        case GCODE_EVENT_SET_TOOL:
        case GCODE_EVENT_NONE:
        default:
            // Not yet implemented or no action needed
            return true;
    }
    
    return true;
}
