#include "utils.h"
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
    
    while (i < str_len && token_array->count < MAX_TOKENS) {
        // Skip whitespace and separators
        while (i < str_len && (str[i] == ' ' || str[i] == '\t' || str[i] == ';')) {
            i++;
        }
        
        if (i >= str_len) break;
        
        // Check for comment - ignore rest of line
        if (str[i] == '(' || str[i] == '#') {
            break;
        }
        
        uint32_t token_start = i;
        uint32_t token_end = find_token_end(str, token_start, str_len);
        
        // Extract token if valid length
        uint32_t token_len = token_end - token_start;
        if (token_len > 0 && token_len < MAX_TOKEN_LENGTH) {
            memcpy(token_array->tokens[token_array->count], &str[token_start], token_len);
            token_array->tokens[token_array->count][token_len] = '\0';
            token_array->count++;
        }
        
        i = token_end;
    }
    
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
    
    if (start_char == 'G' || start_char == 'M') {
        // G/M command: read until next command letter or line end
        while (end < max_len) {
            char c = str[end];
            // Stop at next command letter
            if (c == 'G' || c == 'M' || c == 'F' || c == 'S' || c == 'T' || c == 'P') {
                break;
            }
            // Stop at line terminators or comments
            if (c == '\n' || c == '\r' || c == '\0' || c == '(' || c == '#' || c == ';') {
                break;
            }
            end++;
        }
    }
    else if (start_char == 'F' || start_char == 'S' || start_char == 'T' || start_char == 'P') {
        // Parameter command: read numeric value
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
        // Coordinate group: read all coordinate axes together (X10Y20Z5)
        while (end < max_len) {
            char c = str[end];
            if (c == 'X' || c == 'Y' || c == 'Z' || c == 'A') {
                end++;  // Skip axis letter
                // Read numeric value
                while (end < max_len) {
                    char val = str[end];
                    if ((val >= '0' && val <= '9') || val == '.' || val == '-') {
                        end++;
                    } else {
                        break;
                    }
                }
            } else {
                break;  // End of coordinate group
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