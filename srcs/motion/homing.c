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
    
    // Filter axes_mask to only include axes with homing enabled via $22 bit mask
    // $22 bit mask: bit 0=X, bit 1=Y, bit 2=Z, bit 3=A
    // This allows $H to work even if some axes don't have limit switches
    uint8_t homing_enable_mask = *g_homing_settings[0].homing_enable;  // All axes point to same setting
    uint8_t enabled_axes = axes_mask & homing_enable_mask;  // AND with enable mask
    
    // Check if any axes are enabled for homing
    if (enabled_axes == 0) {
        return false;  // No axes enabled for homing
    }
    
    // Initialize homing control
    g_homing.axes_to_home = enabled_axes;  // Use filtered mask
    g_homing.axes_homed = 0;
    g_homing.debouncing = false;
    g_homing.motion_active = false;
    g_homing.alarm_code = 0;
    
    // Reset persistent limit state tracker (for clean edge detection)
    UTILS_HomingLimitReset();
    
    // Find first axis to home (start with X=0)
    for (uint8_t i = 0; i < NUM_AXIS; i++) {
        if (enabled_axes & (1 << i)) {
            g_homing.current_axis = (E_AXIS)i;
            UTILS_HomingSetCurrentAxis((E_AXIS)i);  // Set tracker to first axis
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
            // APP.C handles limit detection and transitions to PULLOFF
            // Here we just check if motion completed without hitting limit (alarm)
            if (!g_homing.motion_active && appData->motionQueueCount == 0) {
                DEBUG_PRINT_GCODE("[HOMING] LOCATE motion complete without limit - ALARM\r\n");
                g_homing.state = HOMING_STATE_ALARM;
                g_homing.alarm_code = 9;  // GRBL Alarm 9: Homing fail
                STEPPER_DisableAll();
            }
            break;
            
        case HOMING_STATE_PULLOFF:
            // APP.C handles limit clearing and transitions to COMPLETE
            // Here we just wait for motion to complete
            if (!g_homing.motion_active && appData->motionQueueCount == 0) {
                DEBUG_PRINT_GCODE("[HOMING] PULLOFF motion complete\r\n");
                // APP.C should have already transitioned to COMPLETE if limit cleared
                // If we're still here, keep waiting
            }
            break;
            
        case HOMING_STATE_COMPLETE:
            // Homing complete - set position and move to next axis
            g_homing.axes_homed |= (1 << g_homing.current_axis);
            KINEMATICS_SetAxisMachinePosition(g_homing.current_axis, 0.0f);
            
            if (HOMING_NextAxis()) {
                g_homing.state = HOMING_STATE_SEEK;
                HOMING_StartSeek(appData);
            } else {
                g_homing.state = HOMING_STATE_IDLE;
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
    STEPPER_DisableAll();
    DEBUG_PRINT_MOTION("[HOMING_Abort] Homing aborted, returned to IDLE\r\n");
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
    // Get settings
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Determine homing direction from $23 mask
    uint8_t dir_mask = *g_homing_settings[g_homing.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> g_homing.current_axis) & 0x01;
    
    DEBUG_PRINT_MOTION("[HOMING_SEEK] RAW: dir_mask=0x%02X, settings->homing_dir_mask=0x%02X\r\n",
                      dir_mask, settings->homing_dir_mask);
    
    // Check if direction is inverted by $3 setting
    bool dir_inverted = (settings->step_direction_invert >> g_homing.current_axis) & 0x01;
    
    // Calculate search distance based on $23 setting
    // $23=0 → home to MIN switch (negative direction in machine coordinates)
    // $23=1 → home to MAX switch (positive direction in machine coordinates)
    float search_distance = home_positive ? 300.0f : -300.0f;
    
    // If $3 direction is inverted, flip the search distance to compensate
    // Example: $23=0 wants -300mm, but $3=1 inverts motion, so use +300mm to actually go negative
    if (dir_inverted) {
        search_distance = -search_distance;
    }
    
    DEBUG_PRINT_MOTION("[HOMING_SEEK] axis=%d, $23=%d, $3_bit=%d, distance=%.1f\r\n",
                      g_homing.current_axis, (dir_mask >> g_homing.current_axis) & 0x01, 
                      dir_inverted, search_distance);
    
    DEBUG_PRINT_MOTION("[HOMING_SEEK] search_distance=%.1f\r\n", search_distance);
    
    // Build target position (current + search distance on current axis)
    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint target = current;
    
    // Array-based axis targeting (replaces switch statement)
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, search_distance);
    
    // Generate motion segment at seek rate
    if (appData->motionQueueCount < MAX_MOTION_SEGMENTS) {
        MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
        KINEMATICS_LinearMoveSimple(current, target, *g_homing_settings[g_homing.current_axis].homing_seek_rate, segment);
        
        appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
        appData->motionQueueCount++;
        
        g_homing.motion_active = true;
    } else {
        // Queue full - retry next iteration
        g_homing.motion_active = false;
    }
}

void HOMING_StartLocate(APP_DATA* appData) {
    // Get settings
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Determine homing direction from $23 mask
    uint8_t dir_mask = *g_homing_settings[g_homing.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> g_homing.current_axis) & 0x01;
    
    // Check if direction is inverted by $3 setting
    bool dir_inverted = (settings->step_direction_invert >> g_homing.current_axis) & 0x01;
    
    // Back off 5mm to clear switch, then approach slowly
    // $23=0 (MIN) → back off +5mm, then approach -10mm
    // $23=1 (MAX) → back off -5mm, then approach +10mm
    float backoff_distance = home_positive ? -5.0f : 5.0f;
    float locate_distance = home_positive ? 10.0f : -10.0f;  // Re-approach slowly
    
    // If $3 direction is inverted, flip distances to compensate
    if (dir_inverted) {
        backoff_distance = -backoff_distance;
        locate_distance = -locate_distance;
    }
    
    DEBUG_PRINT_MOTION("[HOMING_LOCATE] axis=%d, $23=%d, $3_bit=%d, backoff=%.1f, locate=%.1f\r\n",
                      g_homing.current_axis, (dir_mask >> g_homing.current_axis) & 0x01, 
                      dir_inverted, backoff_distance, locate_distance);
    
    // Build target position (back off first)
    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint target = current;
    
    // Array-based axis targeting (replaces switch statement)
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, backoff_distance);
    
    // Generate backoff motion segment at slow rate
    if (appData->motionQueueCount < MAX_MOTION_SEGMENTS) {
        MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
        KINEMATICS_LinearMoveSimple(current, target, *g_homing_settings[g_homing.current_axis].homing_feed_rate, segment);
        
        appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
        appData->motionQueueCount++;
    }
    
    // Generate slow re-approach segment
    current = target;
    
    // Array-based axis targeting (replaces switch statement)
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, locate_distance);
    
    if (appData->motionQueueCount < MAX_MOTION_SEGMENTS) {
        MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
        KINEMATICS_LinearMoveSimple(current, target, *g_homing_settings[g_homing.current_axis].homing_feed_rate, segment);
        
        appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
        appData->motionQueueCount++;
        
        g_homing.motion_active = true;
    }
}

void HOMING_StartPulloff(APP_DATA* appData) {
    // Get settings
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Determine homing direction from $23 mask
    uint8_t dir_mask = *g_homing_settings[g_homing.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> g_homing.current_axis) & 0x01;
    
    // Pull off away from switch
    // $23=0 (MIN) → pull off +distance (away from MIN switch)
    // $23=1 (MAX) → pull off -distance (away from MAX switch)
    float pulloff_distance = home_positive ? -*g_homing_settings[g_homing.current_axis].homing_pull_off : *g_homing_settings[g_homing.current_axis].homing_pull_off;
    
    // Check if direction is inverted by $3 setting
    bool dir_inverted = (settings->step_direction_invert >> g_homing.current_axis) & 0x01;
    
    // If $3 direction is inverted, flip the pulloff distance to compensate
    // Example: $23=0 wants +2mm pulloff, but $3=1 inverts motion, so use -2mm to actually go positive
    if (dir_inverted) {
        pulloff_distance = -pulloff_distance;
    }
    
    DEBUG_PRINT_MOTION("[HOMING_PULLOFF] axis=%d, $23=%d, $3_bit=%d, pulloff=%.1f\r\n",
                      g_homing.current_axis, (dir_mask >> g_homing.current_axis) & 0x01, 
                      dir_inverted, pulloff_distance);
    
    // Build target position
    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint target = current;
    
    // Array-based axis targeting (replaces switch statement)
    ADD_COORDINATE_AXIS(&target, g_homing.current_axis, pulloff_distance);
    
    // Generate pulloff motion segment
    if (appData->motionQueueCount < MAX_MOTION_SEGMENTS) {
        MotionSegment* segment = &appData->motionQueue[appData->motionQueueHead];
        KINEMATICS_LinearMoveSimple(current, target, *g_homing_settings[g_homing.current_axis].homing_feed_rate, segment);
        
        appData->motionQueueHead = (appData->motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
        appData->motionQueueCount++;
        
        g_homing.motion_active = true;
    }
}

bool HOMING_LimitTriggered(void) {
    // Get settings
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Determine which limit to check based on homing direction
    bool home_positive = (*g_homing_settings[g_homing.current_axis].homing_dir_mask >> g_homing.current_axis) & 0x01;
    
    // Check if direction is inverted by $3 setting
    bool dir_inverted = (settings->step_direction_invert >> g_homing.current_axis) & 0x01;
    
    // If $3 inverts this axis, flip the homing direction to compensate
    // This ensures we check the SAME switch that motion is actually approaching
    if (dir_inverted) {
        home_positive = !home_positive;
    }
    
    // Get raw limit pin state (now checks the CORRECT switch based on actual motion direction)
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

bool HOMING_NextAxis(void) {
    // Find next axis to home
    for (uint8_t i = g_homing.current_axis + 1; i < NUM_AXIS; i++) {
        if (g_homing.axes_to_home & (1 << i)) {
            g_homing.current_axis = (E_AXIS)i;
            
            // Update limit state tracker for new axis
            UTILS_HomingSetCurrentAxis((E_AXIS)i);
            
            return true;  // More axes to home
        }
    }
    
    // No more axes - homing complete
    return false;
}
