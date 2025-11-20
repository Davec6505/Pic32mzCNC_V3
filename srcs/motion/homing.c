

#include "../../incs/data_structures.h"
#include "../../incs/motion/homing.h"
#include "../../incs/motion/kinematics.h"
#include "../../incs/settings/settings.h"
#include "../../incs/utils/utils.h"
#include "../../incs/common.h"

// Static internal state for homing
#include <string.h>
static HomingControl homing_state;

// === STUBS FOR LINKING ===

void HOMING_Initialize(void) {
    memset(&homing_state, 0, sizeof(homing_state));
    homing_state.state = HOMING_STATE_IDLE;
    homing_state.current_axis = AXIS_X;
    homing_state.axes_to_home = 0;
    homing_state.axes_homed = 0;
    homing_state.debounce_start = 0;
    homing_state.debouncing = false;
    homing_state.motion_active = false;
    homing_state.alarm_code = 0;
}

void HOMING_Reset(void) {
    HOMING_Initialize();
}

HomingState HOMING_GetState(void) {
    return homing_state.state;
}


HomingState HOMING_Tasks(void) {
    // Implement state machine logic here as needed
    return homing_state.state;
}


void HOMING_StartLocate(void) {
    // Implement locate phase logic as needed
}


bool HOMING_Start(uint32_t axes_mask) {
    if (homing_state.state != HOMING_STATE_IDLE && homing_state.state != HOMING_STATE_COMPLETE && homing_state.state != HOMING_STATE_ALARM)
        return false;
    homing_state.state = HOMING_STATE_SEEK;
    homing_state.axes_to_home = axes_mask;
    homing_state.current_axis = AXIS_X;
    homing_state.axes_homed = 0;
    homing_state.motion_active = true;
    return true;
}





void HOMING_Abort(void) {
    homing_state.state = HOMING_STATE_IDLE;
    homing_state.alarm_code = 0;
    UTILS_HomingLimitReset();
    DEBUG_PRINT_MOTION("[HOMING_Abort] Homing aborted, limit state cleared, returned to IDLE\r\n");
}


bool HOMING_IsActive(void) {
    return (homing_state.state != HOMING_STATE_IDLE &&
            homing_state.state != HOMING_STATE_COMPLETE &&
            homing_state.state != HOMING_STATE_ALARM);
}


void HOMING_ClearAlarm(void) {
    if (homing_state.state == HOMING_STATE_ALARM) {
        homing_state.state = HOMING_STATE_IDLE;
        homing_state.alarm_code = 0;
        UTILS_HomingLimitReset();
        DEBUG_PRINT_MOTION("[HOMING_ClearAlarm] Alarm cleared, returned to IDLE\r\n");
    }
}


void HOMING_StartPulloff(void) {
    uint8_t dir_mask = *g_homing_settings[homing_state.current_axis].homing_dir_mask;
    bool home_positive = (dir_mask >> homing_state.current_axis) & 0x01;
    float pulloff_distance = home_positive ? -*g_homing_settings[homing_state.current_axis].homing_pull_off : *g_homing_settings[homing_state.current_axis].homing_pull_off;
    DEBUG_PRINT_MOTION("[HOMING_PULLOFF] axis=%d, $23=%d, pulloff=%.1f\r\n",
                      homing_state.current_axis, (dir_mask >> homing_state.current_axis) & 0x01, 
                      pulloff_distance);
    CoordinatePoint current = KINEMATICS_GetCurrentPosition();
    CoordinatePoint target = current;
    ADD_COORDINATE_AXIS(&target, homing_state.current_axis, pulloff_distance);
    // NOTE: You must update the motion queue externally as this function no longer has access to appData
}



bool HOMING_LimitTriggered(void) {
    CNC_Settings* settings = SETTINGS_GetCurrent();
    bool home_positive = (*g_homing_settings[homing_state.current_axis].homing_dir_mask >> homing_state.current_axis) & 0x01;
    bool limit_min = LIMIT_GetMin(homing_state.current_axis);
    bool limit_max = LIMIT_GetMax(homing_state.current_axis);
    bool limit_state = home_positive ? limit_max : limit_min;
    DEBUG_EXEC_MOTION({
        static uint32_t debug_counter = 0;
        if (debug_counter++ % 50000 == 0) {
            DEBUG_PRINT_MOTION("[LIMIT_CHECK] axis=%d, checking %s, MIN=%d, MAX=%d, selected=%d\r\n",
                              homing_state.current_axis, home_positive ? "MAX" : "MIN",
                              limit_min, limit_max, limit_state);
        }
    });
    bool inverted = (settings->limit_pins_invert >> homing_state.current_axis) & 0x01;
    DEBUG_EXEC_MOTION({
        bool raw_state = limit_state;
        limit_state ^= inverted;
        static uint32_t debug_xor = 0;
        if (debug_xor++ % 50000 == 0) {
            DEBUG_PRINT_MOTION("[LIMIT_XOR] raw=%d, inverted=%d, final=%d\r\n",
                              raw_state, inverted, limit_state);
        }
    });
    limit_state ^= inverted;
    if (limit_state && !homing_state.debouncing) {
        homing_state.debounce_start = CORETIMER_CounterGet();
        homing_state.debouncing = true;
        return false;
    } else if (limit_state && homing_state.debouncing) {
        uint32_t now = CORETIMER_CounterGet();
        uint32_t elapsed_ticks = now - homing_state.debounce_start;
        uint32_t debounce_ticks = (*g_homing_settings[homing_state.current_axis].homing_debounce * 200) / 1000;
        if (elapsed_ticks >= debounce_ticks) {
            homing_state.debouncing = false;
            return true;
        }
        return false;
    } else {
        homing_state.debouncing = false;
        return false;
    }
}


bool HOMING_NextAxis(void) {
    for (uint8_t i = homing_state.current_axis + 1; i < NUM_AXIS; i++) {
        if (homing_state.axes_to_home & (1 << i)) {
            homing_state.current_axis = (E_AXIS)i;
            UTILS_HomingSetCurrentAxis((E_AXIS)i);
            return true;
        }
    }
    return false;
}
