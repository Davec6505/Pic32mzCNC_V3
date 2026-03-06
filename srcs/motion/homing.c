/**
 * @file homing.c
 * @brief CNC controller homing cycle implementation
 */

#include "motion/homing.h"
#include "motion/kinematics.h"
#include "motion/motion.h"
#include "motion/stepper.h"
#include "motion/motion_utils.h"
#include "utils/utils.h"
#include "utils/uart_utils.h"
#include "settings/settings.h"
#include "../config/default/peripheral/coretimer/plib_coretimer.h"
#include "../config/default/peripheral/gpio/plib_gpio.h"
#include "common.h"

#include <string.h>
#include <math.h>

// Global homing control
HomingControl g_homing;

// ===== INITIALIZATION =====

void HOMING_Initialize(void) {
    memset(&g_homing, 0, sizeof(HomingControl));
    g_homing.state = HOMING_STATE_IDLE;
}

// ===== PUBLIC API =====

bool HOMING_Start(APP_DATA* appData, uint32_t axes_mask) {
    // Check if homing already active
    if (g_homing.state != HOMING_STATE_IDLE) {
        return false;
    }
    
    // Validate axes_mask (only bits 0-3 valid for XYZA)
    if (axes_mask == 0 || (axes_mask & ~0x0F)) {
        return false;  // Invalid mask
    }
    
    // $22 is a global boolean (0=disabled, 1=enabled), NOT a per-axis bitmask.
    // A value of 1 means homing is enabled for all axes — don't AND it with axes_mask.
    uint8_t homing_globally_enabled = *g_homing_settings[0].homing_enable;
    if (!homing_globally_enabled) {
        return false;  // Homing globally disabled ($22=0)
    }
    uint8_t enabled_axes = (uint8_t)axes_mask;  // All requested axes are eligible
    
    // Initialize homing control
    g_homing.axes_to_home = enabled_axes;  // Use filtered mask
    g_homing.axes_homed = 0;
    g_homing.debouncing = false;
    g_homing.motion_active = false;
    g_homing.alarm_code = 0;
    
    // Reset persistent limit state tracker (for clean edge detection)
    UTILS_HomingLimitReset();
    
    // Find first axis to home - SAFETY: Z first (axis 2), then Y (axis 1), then X (axis 0)
    // Homing order: Z → Y → X (lift tool to safe height before moving XY)
    // Use H_AXIS typedef to clarify this is homing order, not motor axis type
    typedef E_AXIS H_AXIS;
    static const H_AXIS homing_order[] = {AXIS_Z, AXIS_Y, AXIS_X, AXIS_A};
    for (uint8_t i = 0; i < NUM_AXIS; i++) {
        H_AXIS axis = homing_order[i];
        if (enabled_axes & (1 << axis)) {
            g_homing.current_axis = axis;
            UTILS_HomingSetCurrentAxis(axis);  // Set tracker to first axis
            break;
        }
    }
    
    // Start seek phase
    g_homing.state = HOMING_STATE_SEEK;
    HOMING_StartSeek(appData);
    
    return true;
}

HomingState HOMING_Tasks(APP_DATA* appData) {
    if (g_homing.state == HOMING_STATE_IDLE) {
        return HOMING_STATE_IDLE;
    }
    
    DEBUG_EXEC_MOTION({
        static uint32_t task_call_counter = 0;
        static uint32_t last_print = 0;
        task_call_counter++;
        if (CORETIMER_CounterGet() - last_print > 100000000) { // Every second
            DEBUG_PRINT_MOTION("[HOMING_Tasks] Called %lu times, state=%d\r\n", 
                              (unsigned long)task_call_counter, g_homing.state);
            task_call_counter = 0;
            last_print = CORETIMER_CounterGet();
        }
    });
    
    switch (g_homing.state) {
        
        case HOMING_STATE_SEEK:
            // APP.C handles limit detection and transitions to LOCATE
            // Here we just check if motion completed without hitting limit (alarm)
            if (!g_homing.motion_active && appData->motionQueueCount == 0) {
                DEBUG_PRINT_GCODE("[HOMING] SEEK motion complete without limit - ALARM\r\n");
                g_homing.state = HOMING_STATE_ALARM;
                g_homing.alarm_code = 9;  // GRBL Alarm 9: Homing fail (travel exceeded)
                STEPPER_DisableAll();
            }
            break;
            
        case HOMING_STATE_LOCATE:
            // Queue-empty drives all transitions:
            //  • locate_backing_off + queue empty  → auto-start re-approach
            //  • locate_reapproaching + queue empty → alarm (missed switch)
            // NOTE: do NOT gate on g_homing.motion_active — that flag is set true by
            // HOMING_StartLocate() and is never cleared during backoff, so using it
            // would prevent the re-approach from ever starting.
            if (appData->motionQueueCount == 0) {
                if (g_homing.locate_reapproaching) {
                    // Re-approach ran to end without hitting the switch → alarm
                    DEBUG_PRINT_MOTION("[HOMING] LOCATE re-approach ended without switch hit - ALARM\r\n");
                    g_homing.state = HOMING_STATE_ALARM;
                    g_homing.alarm_code = 9;
                    STEPPER_DisableAll();
                }
                // Backoff move completed (queue drained) — automatically start re-approach.
                // The limit is not polled during backoff so we use queue-empty as the
                // trigger rather than the falling edge.
                else if (g_homing.locate_backing_off) {
                    DEBUG_PRINT_MOTION("[HOMING] LOCATE backoff complete → starting re-approach\r\n");
                    g_homing.locate_backing_off = false;
                    HOMING_StartLocateReapproach(appData);
                }
                // Otherwise we're between the falling-edge stop and HOMING_StartLocateReapproach
                // queuing a new move — do nothing, re-approach will start momentarily.
            }
            break;
            
        case HOMING_STATE_PULLOFF:
            // Pulloff complete when motion queue drains — advance to COMPLETE.
            // motion_active flag is NOT reliable here (set true by StartPulloff, never cleared
            // by the ISR path), so check motionQueueCount alone.
            if (appData->motionQueueCount == 0 && !appData->motionActive) {
                DEBUG_PRINT_GCODE("[HOMING] PULLOFF complete → COMPLETE\r\n");
                g_homing.motion_active = false;
                g_homing.state = HOMING_STATE_COMPLETE;
            }
            break;
            
        case HOMING_STATE_COMPLETE:
            // Homing complete - step counter was already zeroed at the LOCATE trigger
            // (machine position 0 = home switch). After pulloff, step counter reflects
            // +pull_off_distance (MIN) or max_travel-pull_off_distance (MAX).
            // Sync appData.current[] from step counters so the motion planner agrees.
            g_homing.axes_homed |= (1 << g_homing.current_axis);
            {
                float machine_pos = (float)AXIS_GetSteps(g_homing.current_axis) /
                                    (*g_axis_settings[g_homing.current_axis].steps_per_mm);
                appData->current[g_homing.current_axis] = machine_pos;
                DEBUG_PRINT_MOTION("[HOMING_COMPLETE] Axis %d: step_count=%ld, MPos=%.3f\r\n",
                                  g_homing.current_axis,
                                  (long)AXIS_GetSteps(g_homing.current_axis), machine_pos);
            }
            
            if (HOMING_NextAxis()) {
                g_homing.state = HOMING_STATE_SEEK;
                HOMING_StartSeek(appData);
            } else {
                g_homing.state = HOMING_STATE_IDLE;
                // Reset GRBL planner state so the first G-code move after homing
                // doesn't inherit the homing segment's unit vector as its junction.
                KINEMATICS_ResetPlannerState();
            }
            break;
            
        case HOMING_STATE_ALARM:
            // Stay in alarm until cleared by user
            STEPPER_DisableAll();
            break;
            
        case HOMING_STATE_IDLE:
        default:
            // Idle - do nothing
            break;
    }
    
    return g_homing.state;
}

void HOMING_Abort(void) {
    g_homing.state = HOMING_STATE_IDLE;  // ✅ Reset to IDLE, not ALARM
    g_homing.alarm_code = 0;             // Clear alarm code
    UTILS_HomingLimitReset();            // ✅ Clear per-axis limit state tracker to prevent stale limit data
    // NOTE: Don't call STEPPER_DisableAll() here - soft reset will handle stepper state via APP_Initialize()
    DEBUG_PRINT_MOTION("[HOMING_Abort] Homing aborted, limit state cleared, returned to IDLE\r\n");
}

bool HOMING_IsActive(void) {
    return (g_homing.state != HOMING_STATE_IDLE && 
            g_homing.state != HOMING_STATE_COMPLETE &&
            g_homing.state != HOMING_STATE_ALARM);
}

HomingState HOMING_GetState(void) {
    return g_homing.state;
}

void HOMING_ClearAlarm(void) {
    if (g_homing.state == HOMING_STATE_ALARM) {
        g_homing.state = HOMING_STATE_IDLE;
        g_homing.alarm_code = 0;
    }
}

// ===== INTERNAL HELPERS =====

void HOMING_StartSeek(APP_DATA* appData) {
    // Determine homing direction from $23 mask
    uint8_t dir_mask = *g_homing_settings[g_homing.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> g_homing.current_axis) & 0x01;
    
    DEBUG_PRINT_MOTION("[HOMING_SEEK] RAW: dir_mask=0x%02X\r\n", dir_mask);
    
    // Calculate search distance based on $23 setting and $130-$133 max_travel.
    // $23=0 → home to MIN switch (negative direction) → search -max_travel
    // $23=1 → home to MAX switch (positive direction) → search +max_travel
    CNC_Settings* settings = SETTINGS_GetCurrent();
    float travel = settings->max_travel[g_homing.current_axis];
    float search_distance = home_positive ? travel : -travel;
    
    DEBUG_PRINT_MOTION("[HOMING_SEEK] axis=%d, $23=%d, distance=%.1f\r\n",
                      g_homing.current_axis, dir_mask, 
                      search_distance);
    
    // Build target position (current + search distance on current axis)
    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint target = current;
    
    // Array-based axis targeting (replaces switch statement)
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, search_distance);
    
    // Submit seek move to the trajectory queue via the motion bridge.
    // MOTION_HomingMove handles the queue-full check, Recalculate, and
    // interpolator kick-start — homing.c no longer touches motionQueue[] directly.
    g_homing.motion_active =
        MOTION_HomingMove(appData, current, target,
                          *g_homing_settings[g_homing.current_axis].homing_seek_rate);
}

void HOMING_StartLocate(APP_DATA* appData) {
    // Phase 1 of LOCATE: drive AWAY from the switch with a long move at feed rate.
    // We do NOT pre-queue the re-approach here — we wait until the switch actually
    // opens (falling edge in app.c) before starting the re-approach.  This removes
    // the race condition where a fixed backoff distance might not clear the switch.
    //
    // $23=0 (MIN) → switch is in the - direction  → back off in + direction
    // $23=1 (MAX) → switch is in the + direction  → back off in - direction
    uint8_t dir_mask = *g_homing_settings[g_homing.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> g_homing.current_axis) & 0x01;

    // Use a generous backoff distance (10× pull-off) so we definitely clear the
    // switch regardless of overshoot.  App.c will stop the move on the falling edge.
    float pull_off = *g_homing_settings[g_homing.current_axis].homing_pull_off;
    float backoff_distance = home_positive ? -(pull_off * 10.0f) : (pull_off * 10.0f);

    DEBUG_PRINT_MOTION("[HOMING_LOCATE] axis=%d, backoff=%.1f mm (waiting for switch open)\r\n",
                       g_homing.current_axis, backoff_distance);

    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint target  = current;
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, backoff_distance);

    g_homing.locate_backing_off   = true;
    g_homing.locate_reapproaching = false;
    g_homing.motion_active =
        MOTION_HomingMove(appData, current, target,
                          *g_homing_settings[g_homing.current_axis].homing_feed_rate);
}

void HOMING_StartLocateReapproach(APP_DATA* appData) {
    // Phase 2 of LOCATE: backoff complete.  Reset the limit state tracker so we get
    // a clean rising edge when the slow re-approach hits the switch.
    // (limit_previous was last set to 'active=true' during seek; without this reset
    //  the re-approach will never generate a rising edge.)
    UTILS_HomingLimitReset();

    // Phase 2 of LOCATE: Add a fixed $27 extra
    // clearance (already stopped at the edge), then re-approach slowly toward the switch.
    // $23=0 (MIN) → re-approach in - direction
    // $23=1 (MAX) → re-approach in + direction
    uint8_t dir_mask = *g_homing_settings[g_homing.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> g_homing.current_axis) & 0x01;

    float pull_off = *g_homing_settings[g_homing.current_axis].homing_pull_off;
    // Extra clearance away from switch before re-approach
    float clearance  = home_positive ? -(pull_off) : (pull_off);
    // Re-approach distance: clearance + enough to reach switch (pull_off * 2 headroom)
    float reapproach = home_positive ? (pull_off * 3.0f) : -(pull_off * 3.0f);

    DEBUG_PRINT_MOTION("[HOMING_LOCATE_REAPPROACH] axis=%d, clearance=%.1f, reapproach=%.1f\r\n",
                       g_homing.current_axis, clearance, reapproach);

    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint mid     = current;
    ADD_COORDINATE_AXIS(&mid, g_homing.current_axis, clearance);

    // Queue extra clearance move first
    MOTION_HomingMove(appData, current, mid,
                      *g_homing_settings[g_homing.current_axis].homing_feed_rate);

    // Then queue the slow re-approach
    CoordinatePoint target = mid;
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, reapproach);

    g_homing.locate_backing_off   = false;
    g_homing.locate_reapproaching = true;
    g_homing.motion_active =
        MOTION_HomingMove(appData, mid, target,
                          *g_homing_settings[g_homing.current_axis].homing_feed_rate);
}

void HOMING_StartPulloff(APP_DATA* appData) {
    // Determine homing direction from $23 mask
    uint8_t dir_mask = *g_homing_settings[g_homing.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> g_homing.current_axis) & 0x01;
    
    // Pull off away from switch
    // $23=0 (MIN) → pull off +distance (away from MIN switch)
    // $23=1 (MAX) → pull off -distance (away from MAX switch)
    // NOTE: $3 (step_direction_invert) only inverts GPIO pin, G-code already handles this
    float pulloff_distance = home_positive ? -*g_homing_settings[g_homing.current_axis].homing_pull_off : *g_homing_settings[g_homing.current_axis].homing_pull_off;
    
    DEBUG_PRINT_MOTION("[HOMING_PULLOFF] axis=%d, $23=%d, pulloff=%.1f\r\n",
                      g_homing.current_axis, (dir_mask >> g_homing.current_axis) & 0x01, 
                      pulloff_distance);
    
    // Build target position
    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint target = current;
    
    // Array-based axis targeting (replaces switch statement)
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, pulloff_distance);
    
    // Submit pulloff move to trajectory queue via the motion bridge.
    g_homing.motion_active =
        MOTION_HomingMove(appData, current, target,
                          *g_homing_settings[g_homing.current_axis].homing_feed_rate);
}

bool HOMING_LimitTriggered(void) {
    // Get settings
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Determine which limit to check based on homing direction from $23 ONLY
    // $23=0 → check MIN switch
    // $23=1 → check MAX switch
    // NOTE: $3 (step_direction_invert) only inverts GPIO pin, doesn't change which switch to check
    bool home_positive = (*g_homing_settings[g_homing.current_axis].homing_dir_mask >> g_homing.current_axis) & 0x01;
    
    // Get raw limit pin state
    bool limit_min = LIMIT_GetMin(g_homing.current_axis);
    bool limit_max = LIMIT_GetMax(g_homing.current_axis);
    bool limit_state = home_positive ? limit_max : limit_min;
    
    DEBUG_EXEC_MOTION({
        static uint32_t debug_counter = 0;
        if (debug_counter++ % 50000 == 0) {  // Print occasionally to reduce spam
            DEBUG_PRINT_MOTION("[LIMIT_CHECK] axis=%d, checking %s, MIN=%d, MAX=%d, selected=%d\r\n",
                              g_homing.current_axis, home_positive ? "MAX" : "MIN", 
                              limit_min, limit_max, limit_state);
        }
    });
    
    // Apply invert mask from settings ($5)
    // Current simplified model: One invert bit per axis applies to BOTH Min and Max
    // GRBL standard: Separate bits for Min (axis*2) and Max (axis*2+1)
    bool inverted = (settings->limit_pins_invert >> g_homing.current_axis) & 0x01;
    
    // XOR inverts the trigger logic:
    // NO switch ($5 bit=0): Pin HIGH (1) triggers → 1^0=1 (alarm)
    // NC switch ($5 bit=1): Pin LOW (0) triggers → 0^1=1 (alarm)
    DEBUG_EXEC_MOTION({
        bool raw_state = limit_state;
        limit_state ^= inverted;
        
        static uint32_t debug_xor = 0;
        if (debug_xor++ % 50000 == 0) {
            DEBUG_PRINT_MOTION("[LIMIT_XOR] raw=%d, inverted=%d, final=%d\r\n",
                              raw_state, inverted, limit_state);
        }
    });
    
    // Apply inversion (must be outside DEBUG block for Release builds)
    limit_state ^= inverted;
    
    // Check for limit trigger
    if (limit_state && !g_homing.debouncing) {
        // Limit just triggered - start debounce
        g_homing.debounce_start = CORETIMER_CounterGet();
        g_homing.debouncing = true;
        return false;  // Not confirmed yet
        
    } else if (limit_state && g_homing.debouncing) {
        // Check if debounce delay elapsed
        uint32_t now = CORETIMER_CounterGet();
        uint32_t elapsed_ticks = now - g_homing.debounce_start;
        
        // Convert debounce microseconds to core timer ticks (200MHz = 5ns per tick)
        uint32_t debounce_ticks = (*g_homing_settings[g_homing.current_axis].homing_debounce * 200) / 1000;  // μs to ticks
        
        if (elapsed_ticks >= debounce_ticks) {
            // Debounce complete - limit confirmed
            g_homing.debouncing = false;
            return true;
        }
        return false;  // Still debouncing
        
    } else {
        // Limit not triggered - reset debounce
        g_homing.debouncing = false;
        return false;
    }
}

// Read instantaneous (non-debounced) limit state for the current homing axis.
// Called every iteration by app.c to feed UTILS_HomingLimitUpdate() for edge detection.
// Applies $23 direction (MIN vs MAX switch) and $5 invert mask.
bool HOMING_IsLimitActiveNow(void) {
    CNC_Settings* settings = SETTINGS_GetCurrent();

    bool home_positive = (*g_homing_settings[g_homing.current_axis].homing_dir_mask
                          >> g_homing.current_axis) & 0x01;

    bool limit_min = LIMIT_GetMin(g_homing.current_axis);
    bool limit_max = LIMIT_GetMax(g_homing.current_axis);
    bool limit_state = home_positive ? limit_max : limit_min;

    // Apply invert mask from settings ($5)
    bool inverted = (settings->limit_pins_invert >> g_homing.current_axis) & 0x01;
    limit_state ^= inverted;

    return limit_state;
}

bool HOMING_NextAxis(void) {
    // Homing order: Z → Y → X → A (same as start order)
    static const E_AXIS homing_order[] = {AXIS_Z, AXIS_Y, AXIS_X, AXIS_A};
    
    // Find current axis in order array
    uint8_t current_index = NUM_AXIS;  // Invalid default
    for (uint8_t i = 0; i < NUM_AXIS; i++) {
        if (homing_order[i] == g_homing.current_axis) {
            current_index = i;
            break;
        }
    }
    
    // Find next axis to home (after current in priority order)
    for (uint8_t i = current_index + 1; i < NUM_AXIS; i++) {
        E_AXIS axis = homing_order[i];
        if (g_homing.axes_to_home & (1 << axis)) {
            g_homing.current_axis = axis;
            
            // Update limit state tracker for new axis
            UTILS_HomingSetCurrentAxis(axis);
            
            return true;  // More axes to home
        }
    }
    
    // No more axes - homing complete
    return false;
}
