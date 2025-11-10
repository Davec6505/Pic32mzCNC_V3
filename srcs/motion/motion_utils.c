#include "motion_utils.h"
#include "definitions.h"
#include "utils/utils.h"  // For g_axis_config with function pointers

// Legacy GPIO_PIN arrays - kept for compatibility but function pointers preferred
// Uses Harmony GPIO_PIN constants (e.g., DirX_PIN, EnX_PIN, StepX_PIN)
// These are used with GPIO_PinSet(), GPIO_PinClear(), GPIO_PinRead() functions

// Direction GPIO pin numbers (indexed by axis)
const GPIO_PIN DIR_PINS[AXIS_COUNT] = {
    DirX_PIN,    // AXIS_X = 0
    DirY_PIN,    // AXIS_Y = 1
    DirZ_PIN,    // AXIS_Z = 2
    DirA_PIN     // AXIS_A = 3
};

// Enable GPIO pin numbers (indexed by axis)
const GPIO_PIN EN_PINS[AXIS_COUNT] = {
    EnX_PIN,     // AXIS_X = 0
    EnY_PIN,     // AXIS_Y = 1
    EnZ_PIN,     // AXIS_Z = 2
    EnA_PIN      // AXIS_A = 3
};

// Step GPIO pin numbers (indexed by axis)
// NOTE: Step pins are configured as INPUTS (connected to OC modules)
const GPIO_PIN STEP_PINS[AXIS_COUNT] = {
    StepX_PIN,   // AXIS_X = 0
    StepY_PIN,   // AXIS_Y = 1
    StepZ_PIN,   // AXIS_Z = 2
    StepA_PIN    // AXIS_A = 3
};

// ✅ Utility Functions

/* Set direction for an axis (respects inversion mask from settings)
 * @param axis: E_AXIS enum (AXIS_X, AXIS_Y, AXIS_Z, AXIS_A)
 * @param forward: true = forward direction, false = reverse
 * @param invert_mask: Direction inversion mask from settings ($3)
 */
void MOTION_UTILS_SetDirection(E_AXIS axis, bool forward, uint8_t invert_mask)
{
    if(axis >= AXIS_COUNT) return;
    
    // Check if this axis direction should be inverted (from settings $3 mask)
    bool inverted = (invert_mask >> axis) & 0x01;
    
    // Apply inversion logic:
    // Normal: forward=true → pin HIGH, forward=false → pin LOW
    // Inverted: forward=true → pin LOW, forward=false → pin HIGH  
    bool pin_state = inverted ? !forward : forward;
    
    // Set GPIO pin to computed state using atomic functions (zero overhead)
    if(pin_state) {
        AXIS_DirSet(axis);      // Pin HIGH
    } else {
        AXIS_DirClear(axis);    // Pin LOW
    }
}

/* Enable/disable axis motor driver
 * @param axis: E_AXIS enum (AXIS_X, AXIS_Y, AXIS_Z, AXIS_A)
 * @param enable: true = enable motor, false = disable (idle)
 * @param invert_mask: Enable inversion mask from settings ($4)
 */
void MOTION_UTILS_EnableAxis(E_AXIS axis, bool enable, uint8_t invert_mask)
{
    if(axis >= AXIS_COUNT) return;
    
    // Check if enable signal should be inverted (from settings $4 mask)
    bool inverted = (invert_mask >> axis) & 0x01;
    
    // Apply inversion logic:
    // Normal: enable=true → pin HIGH, enable=false → pin LOW
    // Inverted: enable=true → pin LOW, enable=false → pin HIGH
    bool pin_state = inverted ? !enable : enable;
    
    // Set GPIO pin to computed state using atomic functions (zero overhead)
    if(pin_state) {
        AXIS_EnableSet(axis);    // Pin HIGH
    } else {
        AXIS_EnableClear(axis);  // Pin LOW
    }
}

/* Enable/disable all axes
 * @param enable: true = enable all motors, false = disable all
 * @param invert_mask: Enable inversion mask from settings ($4)
 */
void MOTION_UTILS_EnableAllAxes(bool enable, uint8_t invert_mask)
{
    for(uint8_t axis = 0; axis < AXIS_COUNT; axis++){
        MOTION_UTILS_EnableAxis((E_AXIS)axis, enable, invert_mask);
    }
}

/* Disable all axes (safety function - no inversion check)
 * @param invert_mask: Enable inversion mask from settings ($4)
 */
void MOTION_UTILS_DisableAllAxes(uint8_t invert_mask)
{
    MOTION_UTILS_EnableAllAxes(false, invert_mask);
}

/* Read step pin state for diagnostics
 * @param axis: E_AXIS enum (AXIS_X, AXIS_Y, AXIS_Z, AXIS_A)
 * @return: true = pin high, false = pin low (raw GPIO state, no inversion)
 * NOTE: Step pins are typically inputs connected to OC modules
 */
bool MOTION_UTILS_ReadStepPin(E_AXIS axis)
{
    if(axis >= AXIS_COUNT) return false;
    return GPIO_PinRead(STEP_PINS[axis]);
}

/* Check all hard limit switches (GRBL v1.1 implementation)
 * @param invert_mask: Limit pins inversion mask from settings ($5)
 * @return: true if ANY limit switch is triggered
 * 
 * NOTE: Limit switch pin names (to be added to plib_gpio.h):
 * X_Min, X_Max, Y_Min, Y_Max, Z_Min, Z_Max, A_Min, A_Max
 */
bool MOTION_UTILS_CheckHardLimits(uint8_t invert_mask)
{
    // Read all limit switches (will be defined in plib_gpio.h)
    // For now, using placeholder names - will compile once pins are defined
    
    #ifdef X_Min_Get  // Only compile if limit pins are defined
    bool x_min = X_Min_Get();
    bool x_max = X_Max_Get();
    bool y_min = Y_Min_Get();
    bool y_max = Y_Max_Get();
    bool z_min = Z_Min_Get();
    bool z_max = Z_Max_Get();
    bool a_min = A_Min_Get();
    bool a_max = A_Max_Get();
    
    // Apply inversion mask (bit-mapped per axis)
    // GRBL uses active-low switches by default (triggered = low)
    // Extract individual axis invert bits from mask
    bool x_inv = (invert_mask >> AXIS_X) & 0x01;
    bool y_inv = (invert_mask >> AXIS_Y) & 0x01;
    bool z_inv = (invert_mask >> AXIS_Z) & 0x01;
    bool a_inv = (invert_mask >> AXIS_A) & 0x01;
    
    // Check if any limit is triggered (XOR with invert to get actual state)
    // Normal: triggered = LOW (0), Inverted: triggered = HIGH (1)
    if((x_min ^ x_inv) || (x_max ^ x_inv)) return true;
    if((y_min ^ y_inv) || (y_max ^ y_inv)) return true;
    if((z_min ^ z_inv) || (z_max ^ z_inv)) return true;
    if((a_min ^ a_inv) || (a_max ^ a_inv)) return true;
    #endif
    
    return false;  // No limits triggered (or pins not defined yet)
}

/* Check specific axis limit switches
 * @param axis: E_AXIS enum (AXIS_X, AXIS_Y, AXIS_Z, AXIS_A)
 * @param invert_mask: Limit pins inversion mask from settings ($5)
 * @return: true if this axis limit switch is triggered
 */
bool MOTION_UTILS_CheckAxisLimits(E_AXIS axis, uint8_t invert_mask)
{
    if(axis >= AXIS_COUNT) return false;
    
    #ifdef X_Min_Get  // Only compile if limit pins are defined
    bool inverted = (invert_mask >> axis) & 0x01;
    
    // Array-based limit checking (replaces switch statement)
    return (g_limit_config[axis].limit.GetMin() ^ inverted) || 
           (g_limit_config[axis].limit.GetMax() ^ inverted);
    
    #else
    return false;  // Pins not defined yet
    #endif
}
