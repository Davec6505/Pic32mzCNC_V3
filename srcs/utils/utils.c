#include "utils.h"
#include "common.h"
#include "utils/uart_utils.h"
#include "settings/settings.h"
#include "motion/stepper.h"
#include "../config/default/peripheral/gpio/plib_gpio.h"  // GPIO macros
#include <sys/kmem.h>  // For register address macros
#include <string.h>
#include <ctype.h>

// ===== AXIS HARDWARE CONFIGURATION =====

// Global axis configuration array (indexed by E_AXIS enum)
AxisConfig g_axis_config[NUM_AXIS];

// ===== MCC WRAPPER FUNCTIONS (preserves pin assignment abstraction) =====
// X Axis Wrappers
static inline void __attribute__((always_inline)) Wrapper_StepX_Set(void)    { StepX_Set(); }
static inline void __attribute__((always_inline)) Wrapper_StepX_Clear(void)  { StepX_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_StepX_Toggle(void) { StepX_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_StepX_Get(void)    { return StepX_Get(); }

static inline void __attribute__((always_inline)) Wrapper_DirX_Set(void)     { DirX_Set(); }
static inline void __attribute__((always_inline)) Wrapper_DirX_Clear(void)   { DirX_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_DirX_Toggle(void)  { DirX_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_DirX_Get(void)     { return DirX_Get(); }

static inline void __attribute__((always_inline)) Wrapper_EnX_Set(void)      { EnX_Set(); }
static inline void __attribute__((always_inline)) Wrapper_EnX_Clear(void)    { EnX_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_EnX_Toggle(void)   { EnX_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_EnX_Get(void)      { return EnX_Get(); }

// Y Axis Wrappers
static inline void __attribute__((always_inline)) Wrapper_StepY_Set(void)    { StepY_Set(); }
static inline void __attribute__((always_inline)) Wrapper_StepY_Clear(void)  { StepY_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_StepY_Toggle(void) { StepY_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_StepY_Get(void)    { return StepY_Get(); }

static inline void __attribute__((always_inline)) Wrapper_DirY_Set(void)     { DirY_Set(); }
static inline void __attribute__((always_inline)) Wrapper_DirY_Clear(void)   { DirY_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_DirY_Toggle(void)  { DirY_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_DirY_Get(void)     { return DirY_Get(); }

static inline void __attribute__((always_inline)) Wrapper_EnY_Set(void)      { EnY_Set(); }
static inline void __attribute__((always_inline)) Wrapper_EnY_Clear(void)    { EnY_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_EnY_Toggle(void)   { EnY_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_EnY_Get(void)      { return EnY_Get(); }

// Z Axis Wrappers
static inline void __attribute__((always_inline)) Wrapper_StepZ_Set(void)    { StepZ_Set(); }
static inline void __attribute__((always_inline)) Wrapper_StepZ_Clear(void)  { StepZ_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_StepZ_Toggle(void) { StepZ_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_StepZ_Get(void)    { return StepZ_Get(); }

static inline void __attribute__((always_inline)) Wrapper_DirZ_Set(void)     { DirZ_Set(); }
static inline void __attribute__((always_inline)) Wrapper_DirZ_Clear(void)   { DirZ_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_DirZ_Toggle(void)  { DirZ_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_DirZ_Get(void)     { return DirZ_Get(); }

static inline void __attribute__((always_inline)) Wrapper_EnZ_Set(void)      { EnZ_Set(); }
static inline void __attribute__((always_inline)) Wrapper_EnZ_Clear(void)    { EnZ_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_EnZ_Toggle(void)   { EnZ_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_EnZ_Get(void)      { return EnZ_Get(); }

// A Axis Wrappers
static inline void __attribute__((always_inline)) Wrapper_StepA_Set(void)    { StepA_Set(); }
static inline void __attribute__((always_inline)) Wrapper_StepA_Clear(void)  { StepA_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_StepA_Toggle(void) { StepA_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_StepA_Get(void)    { return StepA_Get(); }

static inline void __attribute__((always_inline)) Wrapper_DirA_Set(void)     { DirA_Set(); }
static inline void __attribute__((always_inline)) Wrapper_DirA_Clear(void)   { DirA_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_DirA_Toggle(void)  { DirA_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_DirA_Get(void)     { return DirA_Get(); }

static inline void __attribute__((always_inline)) Wrapper_EnA_Set(void)      { EnA_Set(); }
static inline void __attribute__((always_inline)) Wrapper_EnA_Clear(void)    { EnA_Clear(); }
static inline void __attribute__((always_inline)) Wrapper_EnA_Toggle(void)   { EnA_Toggle(); }
static inline bool __attribute__((always_inline)) Wrapper_EnA_Get(void)      { return EnA_Get(); }

// ===== LIMIT SWITCH MCC WRAPPERS =====
// X Axis Limit Wrappers
static inline bool __attribute__((always_inline)) Wrapper_X_Min_Get(void)    { return X_Min_Get(); }
static inline bool __attribute__((always_inline)) Wrapper_X_Max_Get(void)    { return X_Max_Get(); }

// Y Axis Limit Wrappers
static inline bool __attribute__((always_inline)) Wrapper_Y_Min_Get(void)    { return Y_Min_Get(); }
static inline bool __attribute__((always_inline)) Wrapper_Y_Max_Get(void)    { return Y_Max_Get(); }

// Z Axis Limit Wrappers
static inline bool __attribute__((always_inline)) Wrapper_Z_Min_Get(void)    { return Z_Min_Get(); }
static inline bool __attribute__((always_inline)) Wrapper_Z_Max_Get(void)    { return Z_Max_Get(); }

// A Axis Limit Wrappers
static inline bool __attribute__((always_inline)) Wrapper_A_Min_Get(void)    { return A_Min_Get(); }
static inline bool __attribute__((always_inline)) Wrapper_A_Max_Get(void)    { return A_Max_Get(); }

// ✅ ARRAY-BASED FUNCTION POINTER TABLES (indexed by E_AXIS enum)
// This eliminates 72 lines of repetitive assignments!
static GPIO_SetFunc axis_step_set[NUM_AXIS] = {
    Wrapper_StepX_Set, Wrapper_StepY_Set, Wrapper_StepZ_Set, Wrapper_StepA_Set
};
static GPIO_ClearFunc axis_step_clear[NUM_AXIS] = {
    Wrapper_StepX_Clear, Wrapper_StepY_Clear, Wrapper_StepZ_Clear, Wrapper_StepA_Clear
};
static GPIO_ToggleFunc axis_step_toggle[NUM_AXIS] = {
    Wrapper_StepX_Toggle, Wrapper_StepY_Toggle, Wrapper_StepZ_Toggle, Wrapper_StepA_Toggle
};
static GPIO_GetFunc axis_step_get[NUM_AXIS] = {
    Wrapper_StepX_Get, Wrapper_StepY_Get, Wrapper_StepZ_Get, Wrapper_StepA_Get
};

static GPIO_SetFunc axis_dir_set[NUM_AXIS] = {
    Wrapper_DirX_Set, Wrapper_DirY_Set, Wrapper_DirZ_Set, Wrapper_DirA_Set
};
static GPIO_ClearFunc axis_dir_clear[NUM_AXIS] = {
    Wrapper_DirX_Clear, Wrapper_DirY_Clear, Wrapper_DirZ_Clear, Wrapper_DirA_Clear
};
static GPIO_ToggleFunc axis_dir_toggle[NUM_AXIS] = {
    Wrapper_DirX_Toggle, Wrapper_DirY_Toggle, Wrapper_DirZ_Toggle, Wrapper_DirA_Toggle
};
static GPIO_GetFunc axis_dir_get[NUM_AXIS] = {
    Wrapper_DirX_Get, Wrapper_DirY_Get, Wrapper_DirZ_Get, Wrapper_DirA_Get
};

static GPIO_SetFunc axis_enable_set[NUM_AXIS] = {
    Wrapper_EnX_Set, Wrapper_EnY_Set, Wrapper_EnZ_Set, Wrapper_EnA_Set
};
static GPIO_ClearFunc axis_enable_clear[NUM_AXIS] = {
    Wrapper_EnX_Clear, Wrapper_EnY_Clear, Wrapper_EnZ_Clear, Wrapper_EnA_Clear
};
static GPIO_ToggleFunc axis_enable_toggle[NUM_AXIS] = {
    Wrapper_EnX_Toggle, Wrapper_EnY_Toggle, Wrapper_EnZ_Toggle, Wrapper_EnA_Toggle
};
static GPIO_GetFunc axis_enable_get[NUM_AXIS] = {
    Wrapper_EnX_Get, Wrapper_EnY_Get, Wrapper_EnZ_Get, Wrapper_EnA_Get
};

// Initialize axis configuration - MUST be called during app initialization
void UTILS_InitAxisConfig(void) {
    CNC_Settings* settings = SETTINGS_GetCurrent();
    StepperPosition* stepper_pos = STEPPER_GetPositionPointer();  // Get LIVE counters, not snapshot!
    
    // ✅ ARRAY-BASED: Loop through all axes and assign function pointers
    // Adding a 5th axis now requires ONLY adding entries to the function pointer arrays above!
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        // Step pin function pointers
        g_axis_config[axis].step.Set = axis_step_set[axis];
        g_axis_config[axis].step.Clear = axis_step_clear[axis];
        g_axis_config[axis].step.Toggle = axis_step_toggle[axis];
        g_axis_config[axis].step.Get = axis_step_get[axis];
        
        // Direction pin function pointers
        g_axis_config[axis].dir.Set = axis_dir_set[axis];
        g_axis_config[axis].dir.Clear = axis_dir_clear[axis];
        g_axis_config[axis].dir.Toggle = axis_dir_toggle[axis];
        g_axis_config[axis].dir.Get = axis_dir_get[axis];
        
        // Enable pin function pointers
        g_axis_config[axis].enable.Set = axis_enable_set[axis];
        g_axis_config[axis].enable.Clear = axis_enable_clear[axis];
        g_axis_config[axis].enable.Toggle = axis_enable_toggle[axis];
        g_axis_config[axis].enable.Get = axis_enable_get[axis];
    }
    
    // ✅ ARRAY-BASED: Assign settings pointers for all axes (loop for scalability)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        g_axis_config[axis].max_rate = &settings->max_rate[axis];
        g_axis_config[axis].acceleration = &settings->acceleration[axis];
        g_axis_config[axis].steps_per_mm = &settings->steps_per_mm[axis];
        g_axis_config[axis].step_count = &stepper_pos->steps[axis];
    }
}

// Get axis configuration (safe accessor with bounds checking)
const AxisConfig* UTILS_GetAxisConfig(E_AXIS axis) {
    if (axis >= NUM_AXIS) {
        return &g_axis_config[AXIS_X];  // Safe fallback
    }
    return &g_axis_config[axis];
}

// ===== LIMIT SWITCH HARDWARE CONFIGURATION =====

// Global limit configuration array (indexed by E_AXIS enum)
LimitConfig g_limit_config[NUM_AXIS];

// ✅ ARRAY-BASED LIMIT SWITCH FUNCTION POINTER TABLES
static GPIO_GetFunc axis_limit_min_get[NUM_AXIS] = {
    Wrapper_X_Min_Get, Wrapper_Y_Min_Get, Wrapper_Z_Min_Get, Wrapper_A_Min_Get
};
static GPIO_GetFunc axis_limit_max_get[NUM_AXIS] = {
    Wrapper_X_Max_Get, Wrapper_Y_Max_Get, Wrapper_Z_Max_Get, Wrapper_A_Max_Get
};

// Initialize limit switch configuration - MUST be called during app initialization
void UTILS_InitLimitConfig(void) {
    CNC_Settings* settings = SETTINGS_GetCurrent();
    
    // ✅ ARRAY-BASED: Loop through all axes and assign limit switch function pointers
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        g_limit_config[axis].limit.GetMin = axis_limit_min_get[axis];
        g_limit_config[axis].limit.GetMax = axis_limit_max_get[axis];
    }
    
    // ✅ ARRAY-BASED: Assign homing settings pointers for all axes (loop for scalability)
    // All axes share the same homing settings (system-wide configuration)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        g_limit_config[axis].homing_enable = &settings->homing_enable;
        g_limit_config[axis].homing_dir_mask = &settings->homing_dir_mask;
        g_limit_config[axis].homing_feed_rate = &settings->homing_feed_rate;
        g_limit_config[axis].homing_seek_rate = &settings->homing_seek_rate;
        g_limit_config[axis].homing_debounce = &settings->homing_debounce;
        g_limit_config[axis].homing_pull_off = &settings->homing_pull_off;
    }
}

// Get limit configuration (safe accessor with bounds checking)
const LimitConfig* UTILS_GetLimitConfig(E_AXIS axis) {
    if (axis >= NUM_AXIS) {
        return &g_limit_config[AXIS_X];  // Safe fallback
    }
    return &g_limit_config[axis];
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
