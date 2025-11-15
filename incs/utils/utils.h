#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include "data_structures.h"  // For E_AXIS enum and NUM_AXIS


// ===== AXIS HARDWARE ABSTRACTION =====


// ===== SIMPLE GPIO HARDWARE ABSTRACTION (LED pattern) =====
typedef void (*GPIO_SetFunc)(void);
typedef void (*GPIO_ClearFunc)(void);
typedef void (*GPIO_ToggleFunc)(void);
typedef bool (*GPIO_GetFunc)(void);

// LED function pointer array (simple clean pattern)
extern GPIO_ToggleFunc led_toggle[];

// Axis function pointer arrays (following LED pattern - simple and direct!)
extern GPIO_SetFunc axis_step_set[];
extern GPIO_ClearFunc axis_step_clear[];
extern GPIO_SetFunc axis_dir_set[];
extern GPIO_ClearFunc axis_dir_clear[];
extern GPIO_SetFunc axis_enable_set[];
extern GPIO_ClearFunc axis_enable_clear[];

// Limit switch function pointer arrays
extern GPIO_GetFunc axis_limit_min_get[];
extern GPIO_GetFunc axis_limit_max_get[];

// Settings and position pointers (indexed by axis)
typedef struct {
    float* max_rate;
    float* acceleration;
    float* steps_per_mm;
    int32_t* step_count;
} AxisSettings;

extern AxisSettings g_axis_settings[NUM_AXIS];

// Utility functions
void UTILS_InitAxisConfig(void);

// ===== INLINE GPIO HELPERS (direct array access - zero overhead!) =====

static inline void __attribute__((always_inline)) AXIS_StepSet(E_AXIS axis) {
    axis_step_set[axis]();
}

static inline void __attribute__((always_inline)) AXIS_StepClear(E_AXIS axis) {
    axis_step_clear[axis]();
}

static inline void __attribute__((always_inline)) AXIS_DirSet(E_AXIS axis) {
    axis_dir_set[axis]();
}

static inline void __attribute__((always_inline)) AXIS_DirClear(E_AXIS axis) {
    axis_dir_clear[axis]();
}

static inline void __attribute__((always_inline)) AXIS_EnableSet(E_AXIS axis) {
    axis_enable_set[axis]();
}

static inline void __attribute__((always_inline)) AXIS_EnableClear(E_AXIS axis) {
    axis_enable_clear[axis]();
}

static inline bool __attribute__((always_inline)) AXIS_LimitMinGet(E_AXIS axis) {
    return axis_limit_min_get[axis]();
}

static inline bool __attribute__((always_inline)) AXIS_LimitMaxGet(E_AXIS axis) {
    return axis_limit_max_get[axis]();
}

// Inline step increment/decrement (direct memory access)
static inline void __attribute__((always_inline)) AXIS_IncrementSteps(E_AXIS axis) {
    (*g_axis_settings[axis].step_count)++;
}

static inline void __attribute__((always_inline)) AXIS_DecrementSteps(E_AXIS axis) {
    (*g_axis_settings[axis].step_count)--;
}

static inline int32_t __attribute__((always_inline)) AXIS_GetSteps(E_AXIS axis) {
    return *g_axis_settings[axis].step_count;
}

static inline void __attribute__((always_inline)) AXIS_SetSteps(E_AXIS axis, int32_t steps) {
    *g_axis_settings[axis].step_count = steps;
}

// ===== LIMIT SWITCH HARDWARE ABSTRACTION =====

// Homing settings per axis
typedef struct {
    uint8_t* homing_enable;
    uint8_t* homing_dir_mask;
    float* homing_feed_rate;
    float* homing_seek_rate;
    uint32_t* homing_debounce;
    float* homing_pull_off;
} HomingSettings;

extern HomingSettings g_homing_settings[NUM_AXIS];

void UTILS_InitLimitConfig(void);

// ===== INLINE LIMIT SWITCH HELPERS (direct array access!) =====

static inline bool __attribute__((always_inline)) LIMIT_GetMin(E_AXIS axis) {
    return axis_limit_min_get[axis]();
}

static inline bool __attribute__((always_inline)) LIMIT_GetMax(E_AXIS axis) {
    return axis_limit_max_get[axis]();
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

// ===== COORDINATE ARRAY UTILITIES (ELIMINATES AXIS SWITCH STATEMENTS) =====

// CoordinatePoint array utilities
// CoordinatePoint { float coordinate[NUM_AXIS]; } -> [0]=X, [1]=Y, [2]=Z, [3]=A

static inline void SET_COORDINATE_AXIS(CoordinatePoint* coord, E_AXIS axis, float value) {
    coord->coordinate[axis] = value;
}

static inline float GET_COORDINATE_AXIS(const CoordinatePoint* coord, E_AXIS axis) {
    return coord->coordinate[axis];
}

static inline void ADD_COORDINATE_AXIS(CoordinatePoint* coord, E_AXIS axis, float delta) {
    coord->coordinate[axis] += delta;
}

// Usage examples (replaces all switch statements for coordinate manipulation):
//   SET_COORDINATE_AXIS(&target, current_axis, base_value + distance);
//   float current_value = GET_COORDINATE_AXIS(&position, axis);
//   ADD_COORDINATE_AXIS(&target, axis, move_distance);

#endif /* UTILS_H */