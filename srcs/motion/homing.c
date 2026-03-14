/**
 * @file homing.c
 * @brief CNC controller homing cycle implementation
 */

#include "motion/homing.h"
#include "motion/kinematics.h"
#include "motion/motion.h"
#include "motion/motion_bridge.h"
#include "motion/motion_utils.h"
#include "motion/interpolator.h"
#include "motion/trajectory.h"
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
            // APP.C reads limit and edges — here we only alarm if travel exceeded
            if (!INTERPOLATOR_IsActive() && TRAJECTORY_QueueCount() == 0u) {
                DEBUG_PRINT_GCODE("[HOMING] SEEK motion complete without limit - ALARM\r\n");
                g_homing.state = HOMING_STATE_ALARM;
                g_homing.alarm_code = 9;
                STEPPER_DisableAll();
            }
            break;
            
        case HOMING_STATE_LOCATE_BACKOFF:
            // Limit NOT sampled here (app.c skips sampling in this state).
            // When the backoff move queue drains, auto-start re-approach.
            if (appData->motionQueueCount == 0) {
                DEBUG_PRINT_MOTION("[HOMING] LOCATE_BACKOFF complete → starting re-approach\r\n");
                HOMING_StartLocateReapproach(appData);  // sets state = LOCATE_REAPPROACH
            }
            break;

        case HOMING_STATE_LOCATE_REAPPROACH:
            // APP.C reads limit and fires rising edge → app.c transitions to PULLOFF.
            // If the re-approach queue drains without hitting the switch → alarm.
            if (appData->motionQueueCount == 0) {
                DEBUG_PRINT_MOTION("[HOMING] LOCATE_REAPPROACH ended without switch hit - ALARM\r\n");
                g_homing.state = HOMING_STATE_ALARM;
                g_homing.alarm_code = 9;
                STEPPER_DisableAll();
            }
            break;
            
        case HOMING_STATE_PULLOFF:
            // Pulloff complete when the s-curve interpolator and trajectory queue are both
            // empty.  We check INTERPOLATOR_IsActive() and TRAJECTORY_QueueCount() directly
            // rather than appData->motionQueueCount because motionQueueCount is a snapshot
            // written once at the top of MOTION_Tasks().  When app.c sets HOMING_STATE_PULLOFF
            // and calls HOMING_StartPulloff() AFTER MOTION_Tasks() has already run in the
            // same main-loop iteration, motionQueueCount still reads 0 (the pre-pulloff
            // value) even though INTERPOLATOR_IsActive() is already true.  Using the live
            // volatile flags avoids the stale-snapshot false-positive that would cause an
            // immediate PULLOFF→IDLE transition before the pulloff move runs.
            if (!INTERPOLATOR_IsActive() && TRAJECTORY_QueueCount() == 0u) {
                g_homing.motion_active = false;
                // ALARM:8 — limit switch still active after pull-off completed.
                // The pull-off distance in $27 was not enough to release the switch.
                if (HOMING_IsLimitActiveNow()) {
                    DEBUG_PRINT_GCODE("[HOMING] PULLOFF complete but limit still active — ALARM:8\r\n");
                    g_homing.state = HOMING_STATE_ALARM;
                    g_homing.alarm_code = 8;
                    STEPPER_DisableAll();
                    // UART_Printf is sent by GCODE_CheckDeferredOk using HOMING_GetAlarmCode()
                    break;
                }
                DEBUG_PRINT_GCODE("[HOMING] PULLOFF complete → COMPLETE\r\n");
                g_homing.state = HOMING_STATE_COMPLETE;
                // Fall through to HOMING_STATE_COMPLETE immediately.
                // Without this, g_homing.state stays COMPLETE for one full main-loop
                // iteration: HOMING_IsActive() returns false (COMPLETE is treated as
                // inactive), so GCODE_CheckDeferredOk releases the $H ok and clears
                // s_homing_pending — but HOMING_Start() on a second $H still sees
                // state==COMPLETE (not IDLE) and returns false → error:8.
                // By falling through we complete the COMPLETE→IDLE transition in the
                // same call, so state is IDLE before any ok is released.
            } else {
                break;
            }
            /* fall through */

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
                // All axes homed — mark machine as homed so soft limits are enforced
                appData->machine_homed = true;
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

uint32_t HOMING_GetAlarmCode(void) {
    return g_homing.alarm_code;
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

    // If already on the limit switch, skip SEEK entirely and go straight to backoff.
    // A rising edge will never occur if the switch is already closed on entry.
    UTILS_HomingLimitReset();
    if (HOMING_IsLimitActiveNow()) {
        DEBUG_PRINT_MOTION("[HOMING_SEEK] limit already active on entry → skip to LOCATE backoff\r\n");
        HOMING_StartLocate(appData);
        return;
    }

    // Calculate search distance based on $23 setting and $130-$133 max_travel.
    // $23=0 → home to MIN switch (negative direction) → search -max_travel
    // $23=1 → home to MAX switch (positive direction) → search +max_travel
    CNC_Settings* settings = SETTINGS_GetCurrent();
    float travel = settings->max_travel[g_homing.current_axis] + 10.0f; // Add extra 10mm to ensure we hit the switch even if it's slightly out of position 
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
    // LOCATE phase 1: back away from the switch at feed rate.
    // Limit is NOT sampled while in LOCATE_BACKOFF — the switch is still physically
    // closed. Queue-empty in HOMING_Tasks() will auto-trigger HOMING_StartLocateReapproach().
    uint8_t dir_mask = *g_homing_settings[g_homing.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> g_homing.current_axis) & 0x01;

    float backoff_distance = home_positive ? -HOMING_LOCATE_BACKOFF_MM : HOMING_LOCATE_BACKOFF_MM;

    DEBUG_PRINT_MOTION("[HOMING_LOCATE_BACKOFF] axis=%d, backoff=%.1f mm\r\n",
                       g_homing.current_axis, backoff_distance);

    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint target  = current;
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, backoff_distance);

    g_homing.state = HOMING_STATE_LOCATE_BACKOFF;
    g_homing.motion_active =
        MOTION_HomingMove(appData, current, target,
                          *g_homing_settings[g_homing.current_axis].homing_feed_rate);
}

void HOMING_StartLocateReapproach(APP_DATA* appData) {
    // LOCATE phase 2: backoff complete. Reset limit tracker for a clean rising edge,
    // then move slowly back toward the switch.
    UTILS_HomingLimitReset();

    uint8_t dir_mask = *g_homing_settings[g_homing.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> g_homing.current_axis) & 0x01;

    // Travel 2x the fixed backoff — guaranteed to retrigger the switch.
    // Limit detection in app.c LOCATE_REAPPROACH stops the move; distance is just a ceiling.
    float reapproach = home_positive ? (HOMING_LOCATE_BACKOFF_MM * 2.0f) : -(HOMING_LOCATE_BACKOFF_MM * 2.0f);

    DEBUG_PRINT_MOTION("[HOMING_LOCATE_REAPPROACH] axis=%d, distance=%.1f mm\r\n",
                       g_homing.current_axis, reapproach);

    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint target  = current;
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, reapproach);

    g_homing.state = HOMING_STATE_LOCATE_REAPPROACH;
    g_homing.motion_active =
        MOTION_HomingMove(appData, current, target,
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
