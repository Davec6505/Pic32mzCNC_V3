#ifndef MOTION_UTILS_H
#define MOTION_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include "../data_structures.h"  // ✅ E_AXIS defined here (parent directory)
#include "peripheral/gpio/plib_gpio.h"

// ✅ Number of axes (must match E_AXIS enum in data_structures.h)
#define AXIS_COUNT 4

// ✅ GPIO Pin Arrays - Indexed by E_AXIS enum
// Uses Harmony GPIO_PIN constants with GPIO_PinSet/GPIO_PinClear functions
// Example: GPIO_PinSet(DIR_PINS[AXIS_X]) sets X direction pin high

// Direction GPIO pin numbers (indexed by E_AXIS)
extern const GPIO_PIN DIR_PINS[AXIS_COUNT];

// Enable GPIO pin numbers (indexed by E_AXIS)
extern const GPIO_PIN EN_PINS[AXIS_COUNT];

// Step GPIO pin numbers (indexed by E_AXIS) - read only
extern const GPIO_PIN STEP_PINS[AXIS_COUNT];

// ✅ Utility Functions

// Set direction for an axis (respects inversion mask from settings)
void MOTION_UTILS_SetDirection(E_AXIS axis, bool forward, uint8_t invert_mask);

// Enable/disable axis motor driver
void MOTION_UTILS_EnableAxis(E_AXIS axis, bool enable, uint8_t invert_mask);

// Enable/disable all axes
void MOTION_UTILS_EnableAllAxes(bool enable, uint8_t invert_mask);

// Disable all axes (safety function)
void MOTION_UTILS_DisableAllAxes(uint8_t invert_mask);

// Read step pin state for diagnostics
bool MOTION_UTILS_ReadStepPin(E_AXIS axis);

#endif // MOTION_UTILS_H
