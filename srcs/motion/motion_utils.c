#include "motion_utils.h"
#include "definitions.h"

// ✅ GPIO Pin Arrays indexed by E_AXIS enum
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
    
    // Check if this axis direction should be inverted
    bool inverted = (invert_mask >> axis) & 0x01;
    
    // Apply inversion if configured
    bool actual_state = inverted ? !forward : forward;
    
    // Set GPIO pin using Harmony inline function
    if(actual_state){
        GPIO_PinSet(DIR_PINS[axis]);
    } else {
        GPIO_PinClear(DIR_PINS[axis]);
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
    
    // Check if enable signal should be inverted
    bool inverted = (invert_mask >> axis) & 0x01;
    
    // Apply inversion if configured
    bool actual_state = inverted ? !enable : enable;
    
    // Set GPIO pin using Harmony inline function
    if(actual_state){
        GPIO_PinSet(EN_PINS[axis]);
    } else {
        GPIO_PinClear(EN_PINS[axis]);
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
 * @return: true = pin high, false = pin low
 */
bool MOTION_UTILS_ReadStepPin(E_AXIS axis)
{
    if(axis >= AXIS_COUNT) return false;
    return GPIO_PinRead(STEP_PINS[axis]);
}
