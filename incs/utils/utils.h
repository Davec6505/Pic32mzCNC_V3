#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include "data_structures.h"  // For E_AXIS enum and NUM_AXIS

// ===== AXIS HARDWARE ABSTRACTION =====

// Function pointer types for GPIO operations (MCC abstraction layer)
typedef void (*GPIO_SetFunc)(void);
typedef void (*GPIO_ClearFunc)(void);
typedef void (*GPIO_ToggleFunc)(void);
typedef bool (*GPIO_GetFunc)(void);

// GPIO control structure with function pointers
typedef struct {
    GPIO_SetFunc Set;
    GPIO_ClearFunc Clear;
    GPIO_ToggleFunc Toggle;
    GPIO_GetFunc Get;
} GPIO_Control;

// Array-based axis configuration (respects MCC pin assignments)
typedef struct {
    GPIO_Control step;      // Step pin control (wraps MCC macros)
    GPIO_Control dir;       // Direction pin control
    GPIO_Control enable;    // Enable pin control
    
    // Settings pointers (from GRBL settings structure)
    float* max_rate;        // Pointer to max_rate_x/y/z/a
    float* acceleration;    // Pointer to acceleration_x/y/z/a
    float* steps_per_mm;    // Pointer to steps_per_mm_x/y/z/a
    
    // Stepper position pointer
    int32_t* step_count;    // Pointer to stepper_pos.x_steps/y_steps/z_steps/a_steps
} AxisConfig;

// Global axis configuration arrays (indexed by axis number 0-3)
extern AxisConfig g_axis_config[NUM_AXIS];
extern int32_t g_axis_deltas[NUM_AXIS];      // For Bresenham: delta_x/y/z/a
extern int32_t* g_axis_errors[NUM_AXIS];     // For Bresenham: &error_x/y/z/a

// Axis utility functions
void UTILS_InitAxisConfig(void);  // Must be called during initialization
const AxisConfig* UTILS_GetAxisConfig(E_AXIS axis);  // Get axis configuration

// ===== INLINE GPIO HELPERS (ZERO OVERHEAD WITH MCC ABSTRACTION!) =====
// These compile to direct function calls that inline the MCC macros

static inline void __attribute__((always_inline)) AXIS_StepSet(E_AXIS axis) {
    g_axis_config[axis].step.Set();
}

static inline void __attribute__((always_inline)) AXIS_StepClear(E_AXIS axis) {
    g_axis_config[axis].step.Clear();
}

static inline void __attribute__((always_inline)) AXIS_DirSet(E_AXIS axis) {
    g_axis_config[axis].dir.Set();
}

static inline void __attribute__((always_inline)) AXIS_DirClear(E_AXIS axis) {
    g_axis_config[axis].dir.Clear();
}

static inline void __attribute__((always_inline)) AXIS_EnableSet(E_AXIS axis) {
    g_axis_config[axis].enable.Set();
}

static inline void __attribute__((always_inline)) AXIS_EnableClear(E_AXIS axis) {
    g_axis_config[axis].enable.Clear();
}

// Inline step increment/decrement (direct memory access - no function call)
static inline void __attribute__((always_inline)) AXIS_IncrementSteps(E_AXIS axis) {
    (*g_axis_config[axis].step_count)++;
}

static inline void __attribute__((always_inline)) AXIS_DecrementSteps(E_AXIS axis) {
    (*g_axis_config[axis].step_count)--;
}

// Inline step getter/setter (direct memory access for position tracking)
static inline int32_t __attribute__((always_inline)) AXIS_GetSteps(E_AXIS axis) {
    return *g_axis_config[axis].step_count;
}

static inline void __attribute__((always_inline)) AXIS_SetSteps(E_AXIS axis, int32_t steps) {
    *g_axis_config[axis].step_count = steps;
}

// ===== LIMIT SWITCH HARDWARE ABSTRACTION =====

// Limit switch control structure (same pattern as GPIO_Control)
typedef struct {
    GPIO_GetFunc GetMin;    // Read minimum limit switch state
    GPIO_GetFunc GetMax;    // Read maximum limit switch state
} GPIO_LimitControl;

// Limit configuration per axis (respects MCC pin assignments)
typedef struct {
    GPIO_LimitControl limit;
    
    // Homing settings pointers (from CNC settings structure)
    uint8_t* homing_enable;      // Pointer to settings->homing_enable
    uint8_t* homing_dir_mask;    // Pointer to settings->homing_dir_mask
    float* homing_feed_rate;     // Pointer to settings->homing_feed_rate
    float* homing_seek_rate;     // Pointer to settings->homing_seek_rate
    uint32_t* homing_debounce;   // Pointer to settings->homing_debounce
    float* homing_pull_off;      // Pointer to settings->homing_pull_off
} LimitConfig;

// Global limit configuration array (indexed by E_AXIS)
extern LimitConfig g_limit_config[NUM_AXIS];

// Limit switch utility functions
void UTILS_InitLimitConfig(void);  // Must be called during initialization
const LimitConfig* UTILS_GetLimitConfig(E_AXIS axis);  // Get limit configuration

// ===== INLINE LIMIT SWITCH HELPERS (ZERO OVERHEAD!) =====

static inline bool __attribute__((always_inline)) LIMIT_GetMin(E_AXIS axis) {
    return g_limit_config[axis].limit.GetMin();
}

static inline bool __attribute__((always_inline)) LIMIT_GetMax(E_AXIS axis) {
    return g_limit_config[axis].limit.GetMax();
}

// Check if any limit triggered for an axis (respects invert mask)
static inline bool __attribute__((always_inline)) LIMIT_CheckAxis(E_AXIS axis, uint8_t invert_mask) {
    bool inverted = (invert_mask >> axis) & 0x01;
    return (LIMIT_GetMin(axis) ^ inverted) || (LIMIT_GetMax(axis) ^ inverted);
}

// String tokenization utilities for G-code parsing
#define MAX_TOKENS 16           // Maximum tokens per line (G90G1X10Y10F100S200M3 = ~8 tokens)
#define MAX_TOKEN_LENGTH 32     // Maximum length per token

// Token array structure - caller provides storage
typedef struct {
    char tokens[MAX_TOKENS][MAX_TOKEN_LENGTH];
    uint32_t count;             // Number of tokens found
} TokenArray;

// G-code specific tokenization functions
uint32_t UTILS_TokenizeGcodeLine(const char* str, TokenArray* token_array);
uint32_t UTILS_SplitString(const char* str, const char* delimiters, TokenArray* token_array);

// Helper functions for G-code parsing
bool UTILS_IsGcodeCommand(const char* token);
bool UTILS_IsParameterCommand(const char* token); 
bool UTILS_IsCoordinateAxis(const char* token);
bool UTILS_IsComment(const char* token);

// String utility functions
void UTILS_TrimWhitespace(char* str);
bool UTILS_IsEmptyString(const char* str);
uint32_t UTILS_SafeStrlen(const char* str, uint32_t max_len);

#endif /* UTILS_H */