#include "utils.h"
#include "common.h"
#include "utils/uart_utils.h"
#include <string.h>
#include <ctype.h>

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
    
    DEBUG_PRINT_GCODE("[TOKENIZER] Input: '%s' (len=%lu)\r\n", str, str_len);
    
    while (i < str_len && token_array->count < MAX_TOKENS) {
        // Check for comment - ignore rest of line
        if (str[i] == '(' || str[i] == '#' || str[i] == ';') {
            break;
        }
        
        uint32_t token_start = i;
        uint32_t token_end = find_token_end(str, token_start, str_len);
        
        DEBUG_PRINT_GCODE("[TOKENIZER] Token %lu: start=%lu, end=%lu\r\n", 
                         token_array->count, token_start, token_end);
        
        // Extract token if valid length
        uint32_t token_len = token_end - token_start;
        if (token_len > 0 && token_len < MAX_TOKEN_LENGTH) {
            memcpy(token_array->tokens[token_array->count], &str[token_start], token_len);
            token_array->tokens[token_array->count][token_len] = '\0';
            DEBUG_PRINT_GCODE("[TOKENIZER] Extracted: '%s'\r\n", 
                             token_array->tokens[token_array->count]);
            token_array->count++;
        }
        
        i = token_end;
        
        // Skip trailing spaces after this token (for next G/M command)
        while (i < str_len && (str[i] == ' ' || str[i] == '\t')) {
            i++;
        }
    }
    
    DEBUG_PRINT_GCODE("[TOKENIZER] Total tokens: %lu\r\n", token_array->count);
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
    
    DEBUG_PRINT_GCODE("[find_token_end] start=%lu, start_char='%c' (0x%02X), max_len=%lu\r\n", 
                     start, start_char, (unsigned char)start_char, max_len);
    
    if (start_char == 'G' || start_char == 'M') {
        // ✅ G/M command: consume ALL parameters until next G/M command or line end
        // Example: "G1X10Y10F1000" → stays together as one token
        // Example: "G90G1X10" → splits into "G90" and "G1X10"
        
        // First, consume the G/M code number (digits after G/M)
        while (end < max_len && str[end] >= '0' && str[end] <= '9') {
            DEBUG_PRINT_GCODE("[find_token_end] Digit loop: str[%lu]='%c'\r\n", end, str[end]);
            end++;
        }
        
        DEBUG_PRINT_GCODE("[find_token_end] After digits: end=%lu\r\n", end);
        
        // Now consume ALL parameters (X, Y, Z, F, S, etc.) until next G/M or line end
        while (end < max_len) {
            char c = str[end];
            
            DEBUG_PRINT_GCODE("[find_token_end] Param loop: str[%lu]='%c' (0x%02X)\r\n", 
                             end, (c >= 32 && c < 127) ? c : '?', (unsigned char)c);
            
            // Stop at next G/M command (start of new command)
            if (c == 'G' || c == 'M') {
                DEBUG_PRINT_GCODE("[find_token_end] Stop: next G/M\r\n");
                break;
            }
            
            // Stop at line terminators or comments
            if (c == '\n' || c == '\r' || c == '\0' || c == '(' || c == '#' || c == ';') {
                DEBUG_PRINT_GCODE("[find_token_end] Stop: line terminator\r\n");
                break;
            }
            
            // ✅ Keep going - consume spaces, parameters, everything
            end++;
        }
        
        DEBUG_PRINT_GCODE("[find_token_end] Final end=%lu\r\n", end);
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