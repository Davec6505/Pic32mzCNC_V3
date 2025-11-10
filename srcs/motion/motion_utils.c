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
 * Uses array-based limit configuration for better performance
 */
bool MOTION_UTILS_CheckHardLimits(uint8_t invert_mask)
{
    // Array-based limit checking (replaces switch statements and manual pins)
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        if (LIMIT_CheckAxis((E_AXIS)axis, invert_mask)) {
            return true;  // Any limit triggered
        }
    }
    
    return false;  // No limits triggered
}

/* Check specific axis limit switches
 * @param axis: E_AXIS enum (AXIS_X, AXIS_Y, AXIS_Z, AXIS_A)
 * @param invert_mask: Limit pins inversion mask from settings ($5)
 * @return: true if this axis limit switch is triggered
 */
bool MOTION_UTILS_CheckAxisLimits(E_AXIS axis, uint8_t invert_mask)
{
    if(axis >= AXIS_COUNT) return false;
    
    // Use array-based limit checking from utils module
    return LIMIT_CheckAxis(axis, invert_mask);
}
