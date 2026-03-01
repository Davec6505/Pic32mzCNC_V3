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
// ============================================================================
// Initialization
// ============================================================================

void MOTION_Initialize(void) {
    // Velocity profiling and Bresenham execute inside OCP1_ISR (shadow register set).
    // No main-loop state to initialise.
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
        
        return;  // Exit and let ISR start stepping
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
                // ✅ FEED HOLD: finalize hold if '!' was pending a segment drain
                if (g_feed_hold_pending) {
                    STEPPER_FinalizeHold();  // Hold:1 → Hold:0, stops TMR4/OC1
                } else {
                    TMR4_Stop();
                    appData->motionActive = false;
                }
            }
        }
    }
}

// ============================================================================
// Look-Ahead Helper: Recompute Exit Profile
// ============================================================================
// Called when the NEXT segment's entry speed (junction speed) is known.
// Patches the exit side of a queued-but-not-executing segment so it decelerates
// to the correct speed instead of always grinding down to safe-start (8.33 mm/s).
// Updates: exit_speed_mms, final_rate, decelerate_after, decel_rate_delta.
// Safe to call only on segments that are NOT currentSegment (motionQueueTail).
// ============================================================================
static void MOTION_RecomputeExit(MotionSegment* seg, float new_exit_mms) {
    const float TIMER_FREQ = (float)TMR4_FrequencyGet();
    float steps_per_mm = *g_axis_settings[seg->dominant_axis].steps_per_mm;
    float accel        = seg->acceleration;

    // Cruise speed in mm/s (nominal_rate is ticks — lower ticks = faster speed)
    float nominal_mms = (TIMER_FREQ / (float)seg->nominal_rate) / steps_per_mm;

    // Clamp: exit can't exceed cruise or go negative
    if (new_exit_mms > nominal_mms) new_exit_mms = nominal_mms;
    if (new_exit_mms < 0.0f)        new_exit_mms = 0.0f;
    seg->exit_speed_mms = new_exit_mms;

    // Recompute final_rate from new exit speed
    float exit_steps_per_sec = new_exit_mms * steps_per_mm;
    if (exit_steps_per_sec < 1.0f) exit_steps_per_sec = 1.0f;
    uint32_t new_final = (uint32_t)(TIMER_FREQ / exit_steps_per_sec);
    if (new_final < seg->nominal_rate) new_final = seg->nominal_rate;
    seg->final_rate = new_final;

    // Recompute decelerate_after:
    // decel distance = (v_nominal² - v_exit²) / (2 * a)
    float decel_dist_mm = (nominal_mms * nominal_mms - new_exit_mms * new_exit_mms) / (2.0f * accel);
    if (decel_dist_mm < 0.0f) decel_dist_mm = 0.0f;
    uint32_t decel_steps = (uint32_t)(decel_dist_mm * steps_per_mm);
    if (decel_steps > seg->steps_remaining) decel_steps = seg->steps_remaining;

    uint32_t new_decel_after = seg->steps_remaining - decel_steps;
    // Decel start must be at or after the acceleration end
    if (new_decel_after < seg->accelerate_until) new_decel_after = seg->accelerate_until;
    seg->decelerate_after = new_decel_after;

    // Recompute decel_rate_delta for the updated decel phase
    uint32_t actual_decel = seg->steps_remaining - seg->decelerate_after;
    if (actual_decel > 0) {
        seg->decel_rate_delta = (seg->final_rate - seg->nominal_rate) / actual_decel;
    } else {
        seg->decel_rate_delta = 0;
    }
}

void MOTION_Arc(APP_DATA* appData) {

    // Only generate if arc is active
    if(appData->arcGenState != ARC_GEN_ACTIVE) {
        return;
    }

    // ✅ FILL QUEUE: loop until queue is full or arc is complete.
    // Generating one segment at a time caused stop/go jitter because the
    // main loop could not refill faster than the ISR consumed segments.
    while(appData->arcGenState == ARC_GEN_ACTIVE &&
          appData->motionQueueCount < MAX_MOTION_SEGMENTS) {

    CoordinatePoint next;
    bool is_last_segment = false;
    
    // Check if this is the last segment based on segment counter
    is_last_segment = (appData->arcSegmentCurrent >= (appData->arcSegmentTotal - 1));
    
    if(is_last_segment) {
        // Use exact end point to prevent accumulated error
        next = appData->arcEndPoint;
        appData->arcGenState = ARC_GEN_IDLE;
  
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
    segment->speed_locked = true;   // arc cruise is fixed — look-ahead must not modify this segment

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

    } // end while(arcGenState == ARC_GEN_ACTIVE && queue not full)
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
                return false;  // Queue full, leave event in queue for retry
            }
            
            // Get next queue slot
            MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
              
            // ===== LOOK-AHEAD JUNCTION PLANNING =====
            // entry_velocity : speed this segment starts at  (from N-1→N junction angle)
            // exit_velocity  : placeholder safe-start        (patched when segment N+1 arrives)
            // junction_speed : saved so we can retroactively patch N-1's exit after queuing N
            float entry_velocity     = 8.33f;
            float exit_velocity      = 8.33f;
            float junction_speed     = 8.33f;
            uint32_t look_prev_index = 0;
            bool     has_prev_patch  = false;

            if (appData->motionQueueCount > 0) {
                look_prev_index = (appData->motionQueueHead - 1 + MAX_MOTION_SEGMENTS) % MAX_MOTION_SEGMENTS;
                MotionSegment* prev_seg = &appData->motionQueue[look_prev_index];

                if (prev_seg->dominant_delta > 0) {
                    // Previous direction vector (normalised from Bresenham deltas)
                    CoordinatePoint prev_dir;
                    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                        SET_COORDINATE_AXIS(&prev_dir, axis,
                            (float)prev_seg->delta[axis] / (float)prev_seg->dominant_delta);
                    }

                    // Current direction vector preview
                    float delta_mm[NUM_AXIS];
                    float distance = 0.0f;
                    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                        delta_mm[axis] = GET_COORDINATE_AXIS(&end, axis) - GET_COORDINATE_AXIS(&start, axis);
                        distance += delta_mm[axis] * delta_mm[axis];
                    }
                    distance = sqrtf(distance);

                    if (distance > 0.001f) {
                        CoordinatePoint curr_dir;
                        for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                            SET_COORDINATE_AXIS(&curr_dir, axis, delta_mm[axis] / distance);
                        }

                        CNC_Settings* jct_settings = SETTINGS_GetCurrent();
                        junction_speed = KINEMATICS_CalculateJunctionSpeed(prev_dir, curr_dir,
                                                                           jct_settings->junction_deviation,
                                                                           *(g_axis_settings[AXIS_X].acceleration));

                        float max_junction = feedrate / 60.0f;
                        if (junction_speed > max_junction) junction_speed = max_junction;

                        entry_velocity = junction_speed;

                        // Flag N-1 for retroactive exit patch (skip if arc-locked)
                        has_prev_patch = !prev_seg->speed_locked;

                        DEBUG_PRINT_MOTION("[JUNCTION] junction=%.1f entry=%.1f mm/s\r\n",
                                          junction_speed, entry_velocity);
                    }
                }
            }

            // Build segment N with settled entry; exit is placeholder and will be patched
            KINEMATICS_LinearMove(start, end, feedrate, segment, entry_velocity, exit_velocity);

            // Guard: skip zero-length segments (no steps)
            if (segment->steps_remaining == 0) {
                for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                    appData->current[axis] = GET_COORDINATE_AXIS(&end, axis);
                }
                return true;
            }

            // Add to motion queue
            appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
            appData->motionQueueCount++;

            // ===== FULL BACKWARD PASS (Look-Ahead Planner) =====
            // Now that segment N is queued, walk backward from N-1 toward the executing
            // segment (motionQueueTail) and propagate speed constraints.
            //
            // Logic per step:
            //   seg[i] can exit at v_exit only if it can decelerate from its entry_speed
            //   to v_exit within its available steps:
            //       v_achievable = sqrt(entry² + 2*a*dist)
            //   If v_achievable <= required exit → this segment is already fine, stop.
            //   If v_achievable >  required exit → clamp its exit and continue backward.
            //
            // The "required exit" for seg[i] is seg[i+1]->entry_speed_mms, because that
            // is the speed the NEXT segment was built to start at.
            //
            // Guards:
            //   - Never patch motionQueueTail (currently executing, ISR owns it)
            //   - Never patch speed_locked segments (arc cruise is fixed)
            //   - Stop as soon as no change is needed (speeds are already consistent)
            if (has_prev_patch) {
                // Required exit for N-1 is the entry we just computed for N
                float required_exit = entry_velocity;   // == junction_speed for N-1→N
                uint32_t i    = look_prev_index;
                uint32_t tail = appData->motionQueueTail;

                while (i != tail) {
                    MotionSegment* seg = &appData->motionQueue[i];

                    if (seg->speed_locked) {
                        // Arc segment — its cruise speed is what the next segment must
                        // match, but we cannot modify it. The required_exit for the
                        // segment before this arc is the arc's own entry speed.
                        required_exit = seg->entry_speed_mms;
                        i = (i - 1 + MAX_MOTION_SEGMENTS) % MAX_MOTION_SEGMENTS;
                        continue;
                    }

                    // Maximum exit speed this segment can achieve given its entry speed
                    // and the distance it has to decelerate.
                    float dist_mm = (float)seg->steps_remaining
                                    / *g_axis_settings[seg->dominant_axis].steps_per_mm;
                    float v_achievable = sqrtf(seg->entry_speed_mms * seg->entry_speed_mms
                                               + 2.0f * seg->acceleration * dist_mm);

                    if (v_achievable <= required_exit) {
                        // This segment is already decelerating enough — all earlier
                        // segments must also be fine (speeds only decrease going back).
                        break;
                    }

                    // Clamp exit to what the forward segment actually needs
                    float new_exit = (v_achievable < seg->exit_speed_mms)
                                     ? v_achievable : seg->exit_speed_mms;
                    if (new_exit > required_exit) new_exit = required_exit;

                    MOTION_RecomputeExit(seg, new_exit);
                    DEBUG_PRINT_MOTION("[LOOKAHEAD] i=%lu exit patched to %.1f mm/s\r\n",
                                      (unsigned long)i, new_exit);

                    // The required exit for the segment before this one is this
                    // segment's entry speed (unchanged — we never modify entry here).
                    required_exit = seg->entry_speed_mms;
                    i = (i - 1 + MAX_MOTION_SEGMENTS) % MAX_MOTION_SEGMENTS;
                }
            }

            // ✅ ARRAY-BASED: Update current position
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
                    
            // Calculate angles
            float start_angle = atan2f(
                GET_COORDINATE_AXIS(&start, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y), 
                GET_COORDINATE_AXIS(&start, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X));
            float end_angle = atan2f(
                GET_COORDINATE_AXIS(&end, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y), 
                GET_COORDINATE_AXIS(&end, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X));
                      
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
                    
            appData->arcSegmentCurrent = 0;      // Start at segment 0
            appData->arcSegmentTotal = num_segments;  // Store total for termination check
            
            appData->arcThetaIncrement = total_angle / (float)num_segments;
            if(!event->data.arcMove.clockwise) {
                appData->arcThetaIncrement = fabsf(appData->arcThetaIncrement);
            } else {
                appData->arcThetaIncrement = -fabsf(appData->arcThetaIncrement);
            }

            // LOOK-AHEAD: patch the preceding G1 segment to decelerate to arc_cruise
            // rather than safe-start, giving a smooth G1→arc entry.
            // arc_cruise is the same triangle-peak formula used by KINEMATICS_LinearMoveSimple.
            {
                float arc_feedrate_mms = event->data.arcMove.feedrate / 60.0f;
                float arc_accel = fminf(settings->acceleration[AXIS_X], settings->acceleration[AXIS_Y]);
                if (arc_accel < 1.0f) arc_accel = 1.0f;
                float arc_cruise = fminf(arc_feedrate_mms, sqrtf(arc_accel * settings->mm_per_arc_segment));

                if (appData->motionQueueCount > 0) {
                    uint32_t arc_prev = (appData->motionQueueHead - 1 + MAX_MOTION_SEGMENTS) % MAX_MOTION_SEGMENTS;
                    if (arc_prev != appData->motionQueueTail
                        && !appData->motionQueue[arc_prev].speed_locked) {
                        MOTION_RecomputeExit(&appData->motionQueue[arc_prev], arc_cruise);
                        DEBUG_PRINT_MOTION("[LOOKAHEAD] Arc entry: patched preceding G1 exit to %.1f mm/s\r\n", arc_cruise);
                    }
                }
            }

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
        
        case GCODE_EVENT_PROBE_TOWARD:
        case GCODE_EVENT_PROBE_AWAY:
        {
            // G38.2/G38.3 (probe toward) or G38.4/G38.5 (probe away)
            
            // Check if motion queue has space before processing
            if (appData->motionQueueCount >= MAX_MOTION_SEGMENTS) {
                return false;  // Queue full - retry later
            }
            
            // ✅ ARRAY-BASED: Build start coordinate from current position
            CoordinatePoint start = {{
                appData->current[AXIS_X], appData->current[AXIS_Y],
                appData->current[AXIS_Z], appData->current[AXIS_A]
            }};
            CoordinatePoint end;
            
            // ✅ ARRAY-BASED: Build end coordinate (absolute mode only for probing)
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                float event_value;
                // Extract event value based on axis
                if (axis == AXIS_X) event_value = event->data.probe.x;
                else if (axis == AXIS_Y) event_value = event->data.probe.y;
                else if (axis == AXIS_Z) event_value = event->data.probe.z;
                else event_value = event->data.probe.a;
                
                // Probing uses absolute coordinates only (NAN = no change)
                SET_COORDINATE_AXIS(&end, axis, 
                    isnan(event_value) ? appData->current[axis] : event_value);
            }
            
            // Get feedrate (required for probe moves)
            float feedrate = event->data.probe.feedrate;
            if (feedrate <= 0.0f) {
                // Probe requires explicit feedrate
                DEBUG_PRINT_MOTION("[PROBE] Error: No feedrate specified for probe move\r\n");
                return false;
            }
            
            // Get next queue slot
            MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
            
            // Convert to motion segment (probe moves always start/end at minimum speed)
            float probe_speed = 8.33f;  // ~500 mm/min minimum speed
            KINEMATICS_LinearMove(start, end, feedrate, segment, probe_speed, probe_speed);
            
            // Guard: skip zero-length segments
            if (segment->steps_remaining == 0) {
                DEBUG_PRINT_MOTION("[PROBE] Zero-length probe move - skipping\r\n");
                return true;
            }
            
            // Initialize probe state
            appData->probeState = PROBE_STATE_MOVING;
            appData->probeSuccess = false;
            appData->probeAlarmOnFail = event->data.probe.alarm_on_fail;
            
            // Add to motion queue
            appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
            appData->motionQueueCount++;
            
            DEBUG_PRINT_MOTION("[PROBE] Started G38.%d: alarm=%d feedrate=%.1f\r\n",
                             event->data.probe.alarm_on_fail ? 2 : 3,
                             event->data.probe.alarm_on_fail,
                             feedrate);
            
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
