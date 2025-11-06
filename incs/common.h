#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NUM_OF_AXIS 4  // X, Y, Z, A

// E_AXIS enum is defined in data_structures.h to avoid circular dependencies
// Include data_structures.h if you need the enum

// Simple validation - no function overhead needed
#define IS_VALID_AXIS(axis) ((axis) < NUM_OF_AXIS)

// GRBL Protocol Constants
#define GRBL_FIRMWARE_VERSION "Grbl v1.1 [PIC32MZ CNC v3.0]\r\n"
#define GRBL_BUILD_DATE "[Build Date: " __DATE__ "]\r\n"
#define GRBL_BUILD_TIME "[Build Time: " __TIME__ "]\r\n"

// ===== DEBUG CONFIGURATION =====
// Debug levels (always defined)
#define DBG_LEVEL_NONE        0
#define DBG_LEVEL_UART        1
#define DBG_LEVEL_GCODE       2
#define DBG_LEVEL_SEGMENT     3
#define DBG_LEVEL_MOTION      4
#define DBG_LEVEL_STEPPER     5
#define DBG_LEVEL_INFO        6

// Set debug level (can be overridden by Makefile via -DDEBUG_=X)
#ifndef DEBUG_
#define DEBUG_ DBG_LEVEL_NONE  // Default: debugging OFF
#endif

// Helper macros for conditional compilation (exclusive debug levels)
#if DEBUG_ == DBG_LEVEL_UART
    #define DEBUG_UART 1
#else
    #define DEBUG_UART 0
#endif

#if DEBUG_ == DBG_LEVEL_GCODE
    #define DEBUG_GCODE 1
#else
    #define DEBUG_GCODE 0
#endif

#if DEBUG_ == DBG_LEVEL_SEGMENT
    #define DEBUG_SEGMENT 1
#else
    #define DEBUG_SEGMENT 0
#endif

#if DEBUG_ == DBG_LEVEL_MOTION
    #define DEBUG_MOTION 1
#else
    #define DEBUG_MOTION 0
#endif

#if DEBUG_ == DBG_LEVEL_STEPPER
    #define DEBUG_STEPPER 1
#else
    #define DEBUG_STEPPER 0
#endif

#if DEBUG_ == DBG_LEVEL_INFO
    #define DEBUG_INFO 1
#else
    #define DEBUG_INFO 0
#endif

#endif /* COMMON_H */