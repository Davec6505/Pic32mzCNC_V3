#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdbool.h>

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