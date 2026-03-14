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

#include "spindle.h"
#include "../config/default/peripheral/ocmp/plib_ocmp8.h"
#include "../config/default/peripheral/tmr/plib_tmr6.h"
#include "settings/settings.h"

// =============================================================================
// PRIVATE STATE VARIABLES
// =============================================================================

static struct {
    uint32_t current_rpm;        // Current commanded RPM
    bool is_running;             // Spindle on/off state
    uint16_t current_pwm_duty;   // Current PWM duty cycle (0-PR6)
} spindle_state = {0};

// Override state — kept separate from spindle_state so commanded RPM is preserved
static uint32_t s_commanded_rpm       = 0;    // Last RPM set by G-code (M3 Sxxx)
static uint8_t  s_spindle_override_pct = 100; // 100 = nominal (10-200 valid range)

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

void SPINDLE_Initialize(void) {
    // Hardware already initialized by MCC/PLIB
    // TMR6_Initialize() and OCMP8_Initialize() called from SYS_Initialize()
    
    // Set initial state
    spindle_state.current_rpm = 0;
    spindle_state.is_running = false;
    spindle_state.current_pwm_duty = 0;
    
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

void SPINDLE_SetSpeed(uint32_t rpm) {
    s_commanded_rpm = rpm;  // preserve commanded RPM for override re-application

    // Apply current override: effective = commanded * override_pct / 100
    uint32_t effective_rpm = (uint32_t)((float)rpm * (float)s_spindle_override_pct / 100.0f);
    spindle_state.current_rpm = effective_rpm;
    
    // Convert RPM to PWM duty cycle
    uint16_t duty_cycle = SPINDLE_RPMToPWMDuty(effective_rpm);
    spindle_state.current_pwm_duty = duty_cycle;
    
    // Update hardware PWM duty cycle
    OCMP8_CompareSecondaryValueSet(duty_cycle);
}

void SPINDLE_Start(void) {
    if (!spindle_state.is_running) {
        spindle_state.is_running = true;
        
        // Apply current RPM setting
        SPINDLE_SetSpeed(spindle_state.current_rpm);
    }
}

void SPINDLE_Stop(void) {
    if (spindle_state.is_running) {
        spindle_state.is_running = false;
        
        // Set PWM duty cycle to 0% (but preserve RPM setting)
        OCMP8_CompareSecondaryValueSet(0);
    }
}

uint32_t SPINDLE_GetCurrentRPM(void) {
    return spindle_state.current_rpm;
}

bool SPINDLE_IsRunning(void) {
    return spindle_state.is_running;
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
// SPINDLE OVERRIDE API (GRBL real-time bytes 0x99-0x9D)
// =============================================================================

void SPINDLE_SetOverridePct(uint8_t pct) {
    // Clamp to [10, 200] percent
    if (pct < 10u)  pct = 10u;
    if (pct > 200u) pct = 200u;
    s_spindle_override_pct = pct;
    // Re-apply to hardware immediately if spindle is running
    if (spindle_state.is_running && s_commanded_rpm > 0u) {
        uint32_t effective_rpm = (uint32_t)((float)s_commanded_rpm *
                                             (float)s_spindle_override_pct / 100.0f);
        spindle_state.current_rpm = effective_rpm;
        uint16_t duty = SPINDLE_RPMToPWMDuty(effective_rpm);
        spindle_state.current_pwm_duty = duty;
        OCMP8_CompareSecondaryValueSet(duty);
    }
}

uint8_t SPINDLE_GetOverridePct(void) {
    return s_spindle_override_pct;
}

// =============================================================================
// DIAGNOSTIC/DEBUG FUNCTIONS
// =============================================================================

// ─── Laser mode helpers ─────────────────────────────────────────────────────

/*
 * SPINDLE_LaserScale — ISR-safe laser power scaling.
 *
 * Called every 10 µs from the DDS interpolator when $32=1 (laser mode).
 * Scales the OC8 PWM duty proportionally to the instantaneous feedrate so
 * the laser dims automatically during acceleration / deceleration and goes
 * dark completely when the machine stops at a corner.
 *
 * Safety properties:
 *   – Only writes OC8RS (one SFR write — atomic on PIC32MZ).
 *   – Reads spindle_state.current_pwm_duty, written only by main-loop
 *     context; worst-case is one tick at a stale value which is negligible.
 *   – No heap allocation, no locking, no branching on hardware state.
 */
void SPINDLE_LaserScale(float scale)
{
    if (scale < 0.0f) scale = 0.0f;
    if (scale > 1.0f) scale = 1.0f;
    uint16_t duty = (uint16_t)((float)spindle_state.current_pwm_duty * scale + 0.5f);
    OCMP8_CompareSecondaryValueSet(duty);
}

/*
 * SPINDLE_GetCommandedDuty — return the PWM duty for the current S value.
 * Cached by INTERPOLATOR_LoadMove so the ISR can use it without calling
 * into the spindle module every tick.
 */
uint16_t SPINDLE_GetCommandedDuty(void)
{
    return spindle_state.current_pwm_duty;
}

void SPINDLE_GetDiagnostics(void) {
    // For debugging - can be called from debug builds
    // PWM frequency: 3.338kHz
    // Current duty: spindle_state.current_pwm_duty / PWM_PERIOD_TICKS * 100%
    // Current RPM: spindle_state.current_rpm
    // Running: spindle_state.is_running
    (void)0; // Placeholder for debug implementation
}