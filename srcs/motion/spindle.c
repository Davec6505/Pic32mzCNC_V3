/*
 * SPINDLE PWM CONTROL SYSTEM
 * 
 * Hardware Configuration (November 10, 2025):
 * - OC8 in PWM mode (OCTSEL = 0 → TMR6)
 * - TMR6: 1:64 prescaler, 781.25kHz, PR6 = 233
 * - PWM Frequency: 781.25kHz / (233 + 1) = 3.338kHz
 * - Duty cycle: OC8RS / PR6 (0-233 range for 0-100%)
 * 
 * Usage:
 * - M3 S1000  → SPINDLE_SetSpeed(1000), SPINDLE_Start()
 * - M5        → SPINDLE_Stop() 
 * - S2000     → SPINDLE_SetSpeed(2000) (modal)
 */

#include "../../incs/data_structures.h"
#include "spindle.h"
#include "../config/default/peripheral/ocmp/plib_ocmp8.h"
#include "../config/default/peripheral/tmr/plib_tmr6.h"
#include "settings/settings.h"

// =============================================================================
// STATE IS NOW CENTRALIZED IN APP_DATA (spindleState)
// =============================================================================

// =============================================================================
// PWM FREQUENCY CALCULATION (Based on Your Hardware Setup)
// =============================================================================

// TMR6 Configuration (from plib_tmr6.c):
// - PBCLK3 = 50MHz  
// - Prescaler = 1:64 (TCKPS = 6)
// - Timer Frequency = 781.25kHz
// - PR6 = 233
// - PWM Frequency = 781.25kHz / (233 + 1) = 3.338kHz

#define PWM_TIMER_FREQ_HZ    781250UL    // TMR6 frequency (Hz)
#define PWM_PERIOD_TICKS     233U        // PR6 value from plib_tmr6.c
#define PWM_FREQUENCY_HZ     (PWM_TIMER_FREQ_HZ / (PWM_PERIOD_TICKS + 1))  // 3.338kHz

// =============================================================================
// INITIALIZATION
// =============================================================================

void SPINDLE_Initialize(APP_DATA* appData) {
    // Hardware already initialized by MCC/PLIB
    // TMR6_Initialize() and OCMP8_Initialize() called from SYS_Initialize()

    // Set initial state
    appData->spindleState.current_rpm = 0;
    appData->spindleState.is_running = false;
    appData->spindleState.current_pwm_duty = 0;

    // Start TMR6 (required for OC8 PWM operation)
    TMR6_Start();

    // Enable OC8 PWM output
    OCMP8_Enable();

    // Set initial duty cycle to 0% (spindle off)
    OCMP8_CompareSecondaryValueSet(0);
}

// =============================================================================
// PUBLIC API FUNCTIONS
// =============================================================================

void SPINDLE_SetSpeed(APP_DATA* appData, uint32_t rpm) {
    appData->spindleState.current_rpm = rpm;

    // Convert RPM to PWM duty cycle
    uint16_t duty_cycle = SPINDLE_RPMToPWMDuty(rpm);
    appData->spindleState.current_pwm_duty = duty_cycle;

    // Update hardware PWM duty cycle
    OCMP8_CompareSecondaryValueSet(duty_cycle);
}

void SPINDLE_Start(APP_DATA* appData) {
    if (!appData->spindleState.is_running) {
        appData->spindleState.is_running = true;

        // Apply current RPM setting
        SPINDLE_SetSpeed(appData, appData->spindleState.current_rpm);
    }
}

void SPINDLE_Stop(APP_DATA* appData) {
    if (appData->spindleState.is_running) {
        appData->spindleState.is_running = false;

        // Set PWM duty cycle to 0% (but preserve RPM setting)
        OCMP8_CompareSecondaryValueSet(0);
    }
}

uint32_t SPINDLE_GetCurrentRPM(APP_DATA* appData) {
    return appData->spindleState.current_rpm;
}

bool SPINDLE_IsRunning(APP_DATA* appData) {
    return appData->spindleState.is_running;
}

// =============================================================================
// PWM CONVERSION FUNCTIONS
// =============================================================================

uint16_t SPINDLE_RPMToPWMDuty(uint32_t rpm) {
    // Get GRBL spindle settings
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Clamp RPM to configured range
    if (rpm > (uint32_t)settings->spindle_max_rpm) {
        rpm = (uint32_t)settings->spindle_max_rpm;
    }
    if (rpm < (uint32_t)settings->spindle_min_rpm && rpm > 0) {
        rpm = (uint32_t)settings->spindle_min_rpm;
    }
    
    // Handle off condition
    if (rpm == 0) {
        return 0;
    }
    
    // Linear mapping: RPM → PWM duty cycle
    // duty = (rpm - min_rpm) / (max_rpm - min_rpm) * PWM_PERIOD_TICKS
    uint32_t rpm_range = (uint32_t)(settings->spindle_max_rpm - settings->spindle_min_rpm);
    uint32_t rpm_offset = rpm - (uint32_t)settings->spindle_min_rpm;
    
    if (rpm_range == 0) {
        return PWM_PERIOD_TICKS;  // 100% if no range configured
    }
    
    uint32_t duty_cycle = (rpm_offset * PWM_PERIOD_TICKS) / rpm_range;
    
    // Clamp to valid PWM range
    if (duty_cycle > PWM_PERIOD_TICKS) {
        duty_cycle = PWM_PERIOD_TICKS;
    }
    
    return (uint16_t)duty_cycle;
}

uint32_t SPINDLE_PWMDutyToRPM(uint16_t duty) {
    // Get GRBL spindle settings
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Handle off condition
    if (duty == 0) {
        return 0;
    }
    
    // Reverse linear mapping: PWM duty cycle → RPM
    uint32_t rpm_range = (uint32_t)(settings->spindle_max_rpm - settings->spindle_min_rpm);
    uint32_t rpm = (uint32_t)settings->spindle_min_rpm + ((uint32_t)duty * rpm_range) / PWM_PERIOD_TICKS;
    
    return rpm;
}

// =============================================================================
// DIAGNOSTIC/DEBUG FUNCTIONS
// =============================================================================

void SPINDLE_GetDiagnostics(APP_DATA* appData) {
    // For debugging - can be called from debug builds
    // PWM frequency: 3.338kHz
    // Current duty: appData->spindleState.current_pwm_duty / PWM_PERIOD_TICKS * 100%
    // Current RPM: appData->spindleState.current_rpm
    // Running: appData->spindleState.is_running
    (void)appData;
    (void)0; // Placeholder for debug implementation
}