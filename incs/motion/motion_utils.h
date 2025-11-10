#ifndef MOTION_UTILS_H
#define MOTION_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include "data_structures.h"  // ✅ E_AXIS defined here (parent directory)
#include "plib_gpio.h"

// ✅ GRBL Invert Mask Format (8-bit bitmask)
// Bit 0 (0x01) = X axis invert
// Bit 1 (0x02) = Y axis invert  
// Bit 2 (0x04) = Z axis invert
// Bit 3 (0x08) = A axis invert
// Bit 4-7      = Reserved (future expansion)
//
// GRBL Settings that use invert masks:
// $3 = Direction invert mask (for MOTION_UTILS_SetDirection)
// $4 = Step enable invert mask (for MOTION_UTILS_EnableAxis)  
// $5 = Limit pins invert mask (for MOTION_UTILS_CheckHardLimits)


// ✅ GPIO Pin Arrays - Indexed by E_AXIS enum
// Uses Harmony GPIO_PIN constants with GPIO_PinSet/GPIO_PinClear functions
// Example: GPIO_PinSet(DIR_PINS[AXIS_X]) sets X direction pin high

// Direction GPIO pin numbers (indexed by E_AXIS)
extern const GPIO_PIN DIR_PINS[AXIS_COUNT];

// Enable GPIO pin numbers (indexed by E_AXIS)
extern const GPIO_PIN EN_PINS[AXIS_COUNT];

// Step GPIO pin numbers (indexed by E_AXIS) - read only
extern const GPIO_PIN STEP_PINS[AXIS_COUNT];

// ✅ Utility Functions - All respect GRBL invert masks from settings

// Set direction for an axis (respects inversion mask from settings $3)
void MOTION_UTILS_SetDirection(E_AXIS axis, bool forward, uint8_t invert_mask);

// Enable/disable axis motor driver (respects inversion mask from settings $4)
void MOTION_UTILS_EnableAxis(E_AXIS axis, bool enable, uint8_t invert_mask);

// Enable/disable all axes (respects inversion mask from settings $4)
void MOTION_UTILS_EnableAllAxes(bool enable, uint8_t invert_mask);

// Disable all axes (safety function - respects inversion mask from settings $4)
void MOTION_UTILS_DisableAllAxes(uint8_t invert_mask);

// Read step pin state for diagnostics (raw GPIO state, no inversion applied)
bool MOTION_UTILS_ReadStepPin(E_AXIS axis);

// ✅ Hard limit checking (reads limit switch GPIO pins)
// Returns true if ANY limit switch is triggered (respects inversion mask from settings $5)
bool MOTION_UTILS_CheckHardLimits(uint8_t invert_mask);

// Check specific axis limit switches (respects inversion mask from settings $5)
bool MOTION_UTILS_CheckAxisLimits(E_AXIS axis, uint8_t invert_mask);

#endif // MOTION_UTILS_H
