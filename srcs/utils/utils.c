#include "utils.h"
#include "common.h"
#include "utils/uart_utils.h"
#include "settings/settings.h"
#include "motion/stepper.h"
#include "../config/default/peripheral/gpio/plib_gpio.h"  // GPIO macros
#include "../config/default/peripheral/coretimer/plib_coretimer.h"  // CoreTimer for sampling
#include <sys/kmem.h>  // For register address macros
#include <string.h>
#include <ctype.h>
#include "../config/default/peripheral/gpio/plib_gpio.h"

// ===== AXIS HARDWARE CONFIGURATION =====


// ===== SIMPLE GPIO HARDWARE ABSTRACTION (LED pattern - clean and scalable!) =====
static inline void __attribute__((always_inline)) led1_toggle(void) { LED1_Toggle(); }
static inline void __attribute__((always_inline)) led2_toggle(void) { LED2_Toggle(); }
GPIO_ToggleFunc led_toggle[] = {led1_toggle, led2_toggle};

// ===== AXIS GPIO WRAPPERS (following clean LED pattern) =====
// Step pin wrappers (one per axis)
static inline void __attribute__((always_inline)) step_x_set(void)   { StepX_Set(); }
static inline void __attribute__((always_inline)) step_x_clear(void) { StepX_Clear(); }
static inline void __attribute__((always_inline)) step_y_set(void)   { StepY_Set(); }
static inline void __attribute__((always_inline)) step_y_clear(void) { StepY_Clear(); }
static inline void __attribute__((always_inline)) step_z_set(void)   { StepZ_Set(); }
static inline void __attribute__((always_inline)) step_z_clear(void) { StepZ_Clear(); }
static inline void __attribute__((always_inline)) step_a_set(void)   { StepA_Set(); }
static inline void __attribute__((always_inline)) step_a_clear(void) { StepA_Clear(); }

// Direction pin wrappers (one per axis)
static inline void __attribute__((always_inline)) dir_x_set(void)   { DirX_Set(); }
static inline void __attribute__((always_inline)) dir_x_clear(void) { DirX_Clear(); }
static inline void __attribute__((always_inline)) dir_y_set(void)   { DirY_Set(); }
static inline void __attribute__((always_inline)) dir_y_clear(void) { DirY_Clear(); }
static inline void __attribute__((always_inline)) dir_z_set(void)   { DirZ_Set(); }
static inline void __attribute__((always_inline)) dir_z_clear(void) { DirZ_Clear(); }
static inline void __attribute__((always_inline)) dir_a_set(void)   { DirA_Set(); }
static inline void __attribute__((always_inline)) dir_a_clear(void) { DirA_Clear(); }

// Enable pin wrappers (one per axis)
static inline void __attribute__((always_inline)) enable_x_set(void)   { EnX_Set(); }
static inline void __attribute__((always_inline)) enable_x_clear(void) { EnX_Clear(); }
static inline void __attribute__((always_inline)) enable_y_set(void)   { EnY_Set(); }
static inline void __attribute__((always_inline)) enable_y_clear(void) { EnY_Clear(); }
static inline void __attribute__((always_inline)) enable_z_set(void)   { EnZ_Set(); }
static inline void __attribute__((always_inline)) enable_z_clear(void) { EnZ_Clear(); }
static inline void __attribute__((always_inline)) enable_a_set(void)   { EnA_Set(); }
static inline void __attribute__((always_inline)) enable_a_clear(void) { EnA_Clear(); }

// Limit switch wrappers (min/max per axis)
static inline bool __attribute__((always_inline)) x_min_get(void) { return X_Min_Get(); }
static inline bool __attribute__((always_inline)) x_max_get(void) { return X_Max_Get(); }
static inline bool __attribute__((always_inline)) y_min_get(void) { return Y_Min_Get(); }
static inline bool __attribute__((always_inline)) y_max_get(void) { return Y_Max_Get(); }
static inline bool __attribute__((always_inline)) z_min_get(void) { return Z_Min_Get(); }
static inline bool __attribute__((always_inline)) z_max_get(void) { return Z_Max_Get(); }
static inline bool __attribute__((always_inline)) a_min_get(void) { return A_Min_Get(); }
static inline bool __attribute__((always_inline)) a_max_get(void) { return A_Max_Get(); }

// ✅ CLEAN FUNCTION POINTER ARRAYS (following LED pattern - much simpler!)
GPIO_SetFunc axis_step_set[NUM_AXIS] = {
    step_x_set, step_y_set, step_z_set, step_a_set
};
GPIO_ClearFunc axis_step_clear[NUM_AXIS] = {
    step_x_clear, step_y_clear, step_z_clear, step_a_clear
};

GPIO_SetFunc axis_dir_set[NUM_AXIS] = {
    dir_x_set, dir_y_set, dir_z_set, dir_a_set
};
GPIO_ClearFunc axis_dir_clear[NUM_AXIS] = {
    dir_x_clear, dir_y_clear, dir_z_clear, dir_a_clear
};

GPIO_SetFunc axis_enable_set[NUM_AXIS] = {
    enable_x_set, enable_y_set, enable_z_set, enable_a_set
};
GPIO_ClearFunc axis_enable_clear[NUM_AXIS] = {
    enable_x_clear, enable_y_clear, enable_z_clear, enable_a_clear
};

GPIO_GetFunc axis_limit_min_get[NUM_AXIS] = {
    x_min_get, y_min_get, z_min_get, a_min_get
};
GPIO_GetFunc axis_limit_max_get[NUM_AXIS] = {
    x_max_get, y_max_get, z_max_get, a_max_get
};

// Global arrays (simple and direct!)
AxisSettings g_axis_settings[NUM_AXIS];
HomingSettings g_homing_settings[NUM_AXIS];

// Initialize axis configuration - MUST be called during app initialization
void UTILS_InitAxisConfig(void) {
    CNC_Settings* settings = SETTINGS_GetCurrent();
    StepperPosition* stepper_pos = STEPPER_GetPositionPointer();
    
    // Simple loop - assign settings pointers for all axes
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        g_axis_settings[axis].max_rate = &settings->max_rate[axis];
        g_axis_settings[axis].acceleration = &settings->acceleration[axis];
        g_axis_settings[axis].steps_per_mm = &settings->steps_per_mm[axis];
        g_axis_settings[axis].step_count = &stepper_pos->steps[axis];
    }
}

// ===== LIMIT SWITCH HARDWARE CONFIGURATION =====

// Initialize limit switch configuration - MUST be called during app initialization
void UTILS_InitLimitConfig(void) {
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // Simple loop - all axes share same homing settings
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        g_homing_settings[axis].homing_enable = &settings->homing_enable;
        g_homing_settings[axis].homing_dir_mask = &settings->homing_dir_mask;
        g_homing_settings[axis].homing_feed_rate = &settings->homing_feed_rate;
        g_homing_settings[axis].homing_seek_rate = &settings->homing_seek_rate;
        g_homing_settings[axis].homing_debounce = &settings->homing_debounce;
        g_homing_settings[axis].homing_pull_off = &settings->homing_pull_off;
    }
}

// Forward declarations
static uint32_t find_token_end(const char* str, uint32_t start, uint32_t max_len);

/* @brief Tokenize G-code line into individual commands
 * @param str Pointer to the input string
 * @param max_len Maximum length to check
 * Input: "G90G1X10Y10F100S200M3" 
 * Output: ["G90", "G1X10Y10F100", "S200", "M3"]
 * @return Number of tokens found
 */
uint32_t UTILS_TokenizeGcodeLine(const char* str, TokenArray* token_array)
{
    if (!str || !token_array) return 0;
    
    // Initialize token array
    token_array->count = 0;
    memset(token_array->tokens, 0, sizeof(token_array->tokens));
    
    uint32_t str_len = UTILS_SafeStrlen(str, 256);
    uint32_t i = 0;
    
    // Skip leading whitespace only
    while (i < str_len && (str[i] == ' ' || str[i] == '\t')) {
        i++;
    }
    
    // DEBUG_PRINT_GCODE("[TOKENIZER] Input: '%s' (len=%lu)\r\n", str, str_len);
    
    while (i < str_len && token_array->count < MAX_TOKENS) {
        // Check for comment - ignore rest of line
        if (str[i] == '(' || str[i] == '#' || str[i] == ';') {
            break;
        }
        
        uint32_t token_start = i;
        uint32_t token_end = find_token_end(str, token_start, str_len);
        
        // DEBUG_PRINT_GCODE("[TOKENIZER] Token %lu: start=%lu, end=%lu\r\n", 
        //                  token_array->count, token_start, token_end);
        
        // Extract token if valid length
        uint32_t token_len = token_end - token_start;
        if (token_len > 0 && token_len < MAX_TOKEN_LENGTH) {
            memcpy(token_array->tokens[token_array->count], &str[token_start], token_len);
            token_array->tokens[token_array->count][token_len] = '\0';
            // DEBUG_PRINT_GCODE("[TOKENIZER] Extracted: '%s'\r\n", 
            //                  token_array->tokens[token_array->count]);
            token_array->count++;
        }
        
        i = token_end;
        
        // Skip trailing spaces after this token (for next G/M command)
        while (i < str_len && (str[i] == ' ' || str[i] == '\t')) {
            i++;
        }
    }
    
    // DEBUG_PRINT_GCODE("[TOKENIZER] Total tokens: %lu\r\n", token_array->count);
    return token_array->count;
}

/* @brief Find the end index of a token starting at given position
 * @param str Pointer to the input string
 * @param start Starting index of the token
 * @param max_len Maximum length of the string
 * @return Index of the end of the token
 */
static uint32_t find_token_end(const char* str, uint32_t start, uint32_t max_len)
{
    char start_char = str[start];
    uint32_t end = start + 1;
    
    // DEBUG_PRINT_GCODE("[find_token_end] start=%lu, start_char='%c' (0x%02X), max_len=%lu\r\n", 
    //                  start, start_char, (unsigned char)start_char, max_len);
    
    if (start_char == 'G' || start_char == 'M') {
        // ✅ G/M command: consume ALL parameters until next G/M command or line end
        // Example: "G1X10Y10F1000" → stays together as one token
        // Example: "G90G1X10" → splits into "G90" and "G1X10"
        
        // First, consume the G/M code number (digits after G/M)
        while (end < max_len && str[end] >= '0' && str[end] <= '9') {
            // DEBUG_PRINT_GCODE("[find_token_end] Digit loop: str[%lu]='%c'\r\n", end, str[end]);
            end++;
        }
        
        // DEBUG_PRINT_GCODE("[find_token_end] After digits: end=%lu\r\n", end);
        
        // Now consume ALL parameters (X, Y, Z, F, S, etc.) until next G/M or line end
        while (end < max_len) {
            char c = str[end];
            
            // DEBUG_PRINT_GCODE("[find_token_end] Param loop: str[%lu]='%c' (0x%02X)\r\n", 
            //                  end, (c >= 32 && c < 127) ? c : '?', (unsigned char)c);
            
            // Stop at next G/M command (start of new command)
            if (c == 'G' || c == 'M') {
                // DEBUG_PRINT_GCODE("[find_token_end] Stop: next G/M\r\n");
                break;
            }
            
            // Stop at line terminators or comments
            if (c == '\n' || c == '\r' || c == '\0' || c == '(' || c == '#' || c == ';') {
                // DEBUG_PRINT_GCODE("[find_token_end] Stop: line terminator\r\n");
                break;
            }
            
            // ✅ Keep going - consume spaces, parameters, everything
            end++;
        }
        
        // DEBUG_PRINT_GCODE("[find_token_end] Final end=%lu\r\n", end);
    }
    else if (start_char == 'F' || start_char == 'S' || start_char == 'T' || start_char == 'P') {
        // Standalone parameter command: read numeric value only
        // This handles cases like "G1X10" followed by "S1000M3" on next line
        while (end < max_len) {
            char c = str[end];
            if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
                end++;
            } else {
                break;  // End of numeric value
            }
        }
    }
    else if (start_char == 'X' || start_char == 'Y' || start_char == 'Z' || start_char == 'A') {
        // Standalone coordinate (shouldn't happen with proper G-code, but handle it)
        // Read numeric value
        while (end < max_len) {
            char c = str[end];
            if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
                end++;
            } else {
                break;
            }
        }
    }
    else {
        // Unknown command - read until next space or command
        while (end < max_len && str[end] != ' ' && str[end] != '\t' && 
               str[end] != '\n' && str[end] != '\r' && str[end] != '\0') {
            end++;
        }
    }
    
    return end;
}

/** @brief Split a string into tokens based on delimiters
 *  @param str Pointer to the input string
 *  @param delimiters String containing delimiter characters
 *  @param token_array Pointer to TokenArray to store the tokens
 *  @return Number of tokens found
 */

uint32_t UTILS_SplitString(const char* str, const char* delimiters, TokenArray* token_array)
{
    if (!str || !delimiters || !token_array) return 0;
    
    // Initialize token array
    token_array->count = 0;
    memset(token_array->tokens, 0, sizeof(token_array->tokens));
    
    // Create working copy of input string
    char work_str[256];
    uint32_t str_len = UTILS_SafeStrlen(str, sizeof(work_str) - 1);
    memcpy(work_str, str, str_len);
    work_str[str_len] = '\0';
    
    // Use strtok for generic splitting
    char* token = strtok(work_str, delimiters);
    while (token && token_array->count < MAX_TOKENS) {
        uint32_t token_len = UTILS_SafeStrlen(token, MAX_TOKEN_LENGTH - 1);
        if (token_len > 0) {
            memcpy(token_array->tokens[token_array->count], token, token_len);
            token_array->tokens[token_array->count][token_len] = '\0';
            token_array->count++;
        }
        token = strtok(NULL, delimiters);
    }
    
    return token_array->count;
}



/* @brief Check if token is a G-code command
 * @param token Pointer to the token string
 * @return true if token is a G-code command, false otherwise
 */
bool UTILS_IsGcodeCommand(const char* token)
{
    if (!token || token[0] == '\0') return false;
    return (token[0] == 'G' || token[0] == 'M');
}

bool UTILS_IsParameterCommand(const char* token)
{
    if (!token || token[0] == '\0') return false;
    return (token[0] == 'F' || token[0] == 'S' || token[0] == 'T' || token[0] == 'P');
}

bool UTILS_IsCoordinateAxis(const char* token)
{
    if (!token || token[0] == '\0') return false;
    return (token[0] == 'X' || token[0] == 'Y' || token[0] == 'Z' || token[0] == 'A');
}

bool UTILS_IsComment(const char* token)
{
    if (!token || token[0] == '\0') return false;
    return (token[0] == '(' || token[0] == '#');
}

// String utility functions
void UTILS_TrimWhitespace(char* str)
{
    if (!str) return;
    
    // Trim leading whitespace
    char* start = str;
    while (*start && (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r')) {
        start++;
    }
    
    // Trim trailing whitespace
    char* end = start + strlen(start) - 1;
    while (end > start && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) {
        end--;
    }
    
    // Move trimmed string to start of buffer
    uint32_t trimmed_len = end - start + 1;
    memmove(str, start, trimmed_len);
    str[trimmed_len] = '\0';
}

bool UTILS_IsEmptyString(const char* str)
{
    if (!str) return true;
    
    while (*str) {
        if (*str != ' ' && *str != '\t' && *str != '\n' && *str != '\r') {
            return false;
        }
        str++;
    }
    return true;
}

uint32_t UTILS_SafeStrlen(const char* str, uint32_t max_len)
{
    if (!str) return 0;
    
    uint32_t len = 0;
    while (len < max_len && str[len] != '\0') {
        len++;
    }
    return len;
}

// ===== HOMING LIMIT STATE CHANGE TRACKER =====
// Per-axis persistent state tracker for homing limit detection
// Prevents multiple state transitions on same edge
typedef struct {
    bool limit_previous;      // Last limit state
    bool rising_edge_flag;    // Set once on rising edge, cleared when limit goes LOW
    bool falling_edge_flag;   // Set once on falling edge, cleared when limit goes HIGH
    uint32_t debounce_count;  // Stable state counter (prevent false edges)
} HomingLimitState;

static HomingLimitState g_homing_limit_state[NUM_AXIS] = {0};
static E_AXIS g_homing_current_axis = AXIS_X;  // Track which axis is currently homing
static uint32_t g_last_sample_time = 0;        // CoreTimer value of last sample

#define HOMING_LIMIT_DEBOUNCE_COUNT 10         // Require 10 stable reads before edge
#define HOMING_SAMPLE_INTERVAL_US 1000         // Sample every 1ms (1000 microseconds)
#define CORE_TIMER_HZ 100000000                // 100MHz core timer (200MHz sys / 2)

// Set the current axis being homed (call when switching axes)
void UTILS_HomingSetCurrentAxis(E_AXIS axis)
{
    if (axis < NUM_AXIS) {
        g_homing_current_axis = axis;
    }
}

// Update limit state for current axis - call every iteration during homing
// Uses CoreTimer-based sampling to prevent excessive polling
void UTILS_HomingLimitUpdate(bool limit_active)
{
    // Timer-based sampling: only update at fixed intervals (like V1!)
    uint32_t current_time = CORETIMER_CounterGet();
    uint32_t ticks_per_sample = (CORE_TIMER_HZ / 1000000) * HOMING_SAMPLE_INTERVAL_US;
    
    // Check if sample interval has elapsed (with wraparound handling)
    uint32_t elapsed = current_time - g_last_sample_time;
    if (elapsed < ticks_per_sample) {
        return;  // Not time to sample yet
    }
    
    // Update sample time for next interval
    g_last_sample_time = current_time;
    
    HomingLimitState* state = &g_homing_limit_state[g_homing_current_axis];
    
    // Debounce: require stable state before accepting change
    if (limit_active != state->limit_previous) {
        state->debounce_count++;
        if (state->debounce_count < HOMING_LIMIT_DEBOUNCE_COUNT) {
            return;  // Not stable yet, don't change state
        }
        // State is stable for 10 samples - accept the transition
        state->debounce_count = 0;
        
        // Detect BOTH edges - app.c decides which to use
        // RISING EDGE: limit goes inactive→active
        if (limit_active && !state->limit_previous) {
            state->rising_edge_flag = true;
        }
        // FALLING EDGE: limit goes active→inactive
        else if (!limit_active && state->limit_previous) {
            state->falling_edge_flag = true;
        }
        
        // Update previous state AFTER edge detection
        state->limit_previous = limit_active;
    } else {
        state->debounce_count = 0;  // Same state, reset counter
    }
}

// Check for rising edge on current axis - consumes flag (one-shot pattern)
bool UTILS_HomingLimitRisingEdge(void)
{
    HomingLimitState* state = &g_homing_limit_state[g_homing_current_axis];
    bool edge = state->rising_edge_flag;
    state->rising_edge_flag = false;  // Clear flag after reading (one-shot)
    return edge;
}

// Check for falling edge on current axis - consumes flag (one-shot pattern)
bool UTILS_HomingLimitFallingEdge(void)
{
    HomingLimitState* state = &g_homing_limit_state[g_homing_current_axis];
    bool edge = state->falling_edge_flag;
    state->falling_edge_flag = false;  // Clear flag after reading (one-shot)
    return edge;
}

// Reset state tracker for specific axis
void UTILS_HomingLimitResetAxis(E_AXIS axis)
{
    if (axis < NUM_AXIS) {
        g_homing_limit_state[axis].limit_previous = false;
        g_homing_limit_state[axis].rising_edge_flag = false;
        g_homing_limit_state[axis].falling_edge_flag = false;
        g_homing_limit_state[axis].debounce_count = 0;
    }
}

// Reset state tracker for all axes (call at start of $H)
void UTILS_HomingLimitReset(void)
{
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        UTILS_HomingLimitResetAxis(axis);
    }
    g_homing_current_axis = AXIS_X;  // Reset to first axis
}
