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
    
    // Check if homing enabled via settings
    if (!(*g_homing_settings[AXIS_X].homing_enable)) {
        return false;  // Homing disabled
    }
    
    // Validate axes_mask (only bits 0-3 valid for XYZA)
    if (axes_mask == 0 || (axes_mask & ~0x0F)) {
        return false;  // Invalid mask
    }
    
    // Initialize homing control
    g_homing.axes_to_home = axes_mask;
    g_homing.axes_homed = 0;
    g_homing.debouncing = false;
    g_homing.motion_active = false;
    g_homing.alarm_code = 0;
    
    // Find first axis to home (start with X=0)
    for (uint8_t i = 0; i < NUM_AXIS; i++) {
        if (axes_mask & (1 << i)) {
            g_homing.current_axis = (E_AXIS)i;
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
    
    switch (g_homing.state) {
        
        case HOMING_STATE_SEEK:
            // Check if limit triggered
            if (HOMING_LimitTriggered()) {
                // Limit hit - transition to locate phase
                g_homing.motion_active = false;
                g_homing.state = HOMING_STATE_LOCATE;
                HOMING_StartLocate(appData);
            } else if (!g_homing.motion_active && appData->motionQueueCount == 0) {
                // Motion completed without hitting limit - ALARM
                g_homing.state = HOMING_STATE_ALARM;
                g_homing.alarm_code = 9;  // GRBL Alarm 9: Homing fail (travel exceeded)
                STEPPER_DisableAll();
            }
            break;
            
        case HOMING_STATE_LOCATE:
            // Check if limit triggered again (precision positioning)
            if (HOMING_LimitTriggered()) {
                // Limit hit at slow speed - transition to pulloff
                g_homing.motion_active = false;
                g_homing.state = HOMING_STATE_PULLOFF;
                HOMING_StartPulloff(appData);
            } else if (!g_homing.motion_active && appData->motionQueueCount == 0) {
                // Motion completed without hitting limit - ALARM
                g_homing.state = HOMING_STATE_ALARM;
                g_homing.alarm_code = 9;  // GRBL Alarm 9: Homing fail
                STEPPER_DisableAll();
            }
            break;
            
        case HOMING_STATE_PULLOFF:
            // Wait for pulloff motion to complete
            if (!g_homing.motion_active && appData->motionQueueCount == 0) {
                // Check if limit cleared
                bool limit_cleared = !HOMING_LimitTriggered();
                
                if (limit_cleared) {
                    // Pulloff successful - mark axis as homed
                    g_homing.axes_homed |= (1 << g_homing.current_axis);
                    
                    // Set machine position to zero for this axis
                    KINEMATICS_SetAxisMachinePosition(g_homing.current_axis, 0.0f);
                    
                    // Move to next axis or complete
                    if (HOMING_NextAxis()) {
                        g_homing.state = HOMING_STATE_SEEK;
                        HOMING_StartSeek(appData);
                    } else {
                        // All axes homed successfully
                        g_homing.state = HOMING_STATE_COMPLETE;
                    }
                } else {
                    // Pulloff failed - limit still triggered
                    g_homing.state = HOMING_STATE_ALARM;
                    g_homing.alarm_code = 9;  // GRBL Alarm 9: Homing fail
                    STEPPER_DisableAll();
                }
            }
            break;
            
        case HOMING_STATE_COMPLETE:
            // Homing complete - return to idle
            g_homing.state = HOMING_STATE_IDLE;
            break;
            
        case HOMING_STATE_ALARM:
            // Stay in alarm until cleared by user
            STEPPER_DisableAll();
            break;
            
        default:
            g_homing.state = HOMING_STATE_IDLE;
            break;
    }
    
    return g_homing.state;
}

void HOMING_Abort(void) {
    g_homing.state = HOMING_STATE_ALARM;
    g_homing.alarm_code = 11;  // GRBL Alarm 11: Homing cycle aborted
    STEPPER_DisableAll();
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
    bool home_positive = (*g_homing_settings[g_homing.current_axis].homing_dir_mask >> g_homing.current_axis) & 0x01;
    
    // Calculate large search distance (assume 300mm max travel)
    float search_distance = home_positive ? 300.0f : -300.0f;
    
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
    // Determine homing direction from $23 mask
    bool home_positive = (*g_homing_settings[g_homing.current_axis].homing_dir_mask >> g_homing.current_axis) & 0x01;
    
    // Back off 5mm to clear switch, then approach slowly
    float backoff_distance = home_positive ? -5.0f : 5.0f;
    float locate_distance = home_positive ? 10.0f : -10.0f;  // Re-approach slowly
    
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
    // Determine homing direction from $23 mask
    bool home_positive = (*g_homing_settings[g_homing.current_axis].homing_dir_mask >> g_homing.current_axis) & 0x01;
    float pulloff_distance = home_positive ? -*g_homing_settings[g_homing.current_axis].homing_pull_off : *g_homing_settings[g_homing.current_axis].homing_pull_off;
    
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
    // Determine which limit to check based on homing direction
    bool home_positive = (*g_homing_settings[g_homing.current_axis].homing_dir_mask >> g_homing.current_axis) & 0x01;
    bool limit_state = home_positive ? LIMIT_GetMax(g_homing.current_axis) : 
                                       LIMIT_GetMin(g_homing.current_axis);
    
    // Apply invert mask from settings
    CNC_Settings* settings = SETTINGS_GetCurrent();
    bool inverted = (settings->limit_pins_invert >> g_homing.current_axis) & 0x01;
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
            return true;  // More axes to home
        }
    }
    
    // No more axes - homing complete
    return false;
}
