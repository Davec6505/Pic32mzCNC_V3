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
#define GRBL_FIRMWARE_VERSION "Pic32mzCNC v1.1 [\"$\" for help]\r\n"
#define GRBL_BUILD_DATE "[Build Date: " __DATE__ "]\r\n"
#define GRBL_BUILD_TIME "[Build Time: " __TIME__ "]\r\n"

// ╔════════════════════════════════════════════════════════════════════════════╗
// ║                          DEBUG INFRASTRUCTURE                              ║
// ║                                                                            ║
// ║  Professional compile-time debug system with ZERO runtime overhead        ║
// ║  Debug code is completely removed from release builds via preprocessor    ║
// ╚════════════════════════════════════════════════════════════════════════════╝

// ===== HOW TO USE =====
// 
// 1. BUILD WITH DEBUG:
//    make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"
//
// 2. IN YOUR CODE:
//    DEBUG_PRINT_MOTION("Loading segment: steps=%ld\r\n", steps);
//    
//    The macro expands to:
//    - Debug build:   UART_Printf("Loading segment: steps=%ld\r\n", steps);
//    - Release build: /* nothing - code removed by compiler */
//
// 3. MULTIPLE FLAGS:
//    You can enable multiple subsystems simultaneously:
//    DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE DEBUG_STEPPER"
//
// ===== AVAILABLE DEBUG FLAGS =====
// DEBUG_MOTION   - Motion planning and execution
// DEBUG_GCODE    - G-code parsing and events
// DEBUG_STEPPER  - Low-level stepper control
// DEBUG_SEGMENT  - Segment loading and completion
// DEBUG_UART     - UART communication
// DEBUG_APP      - Application state machine

// ===== DEBUG MACRO DEFINITIONS =====
// Each subsystem has two macros:
// - DEBUG_PRINT_XXX(fmt, ...) - Printf-style debug output
// - DEBUG_EXEC_XXX(code)       - Execute arbitrary code (e.g., LED toggles)

// --- Motion Debug ---
#ifdef DEBUG_MOTION
    #define DEBUG_PRINT_MOTION(fmt, ...) UART_Printf(fmt, ##__VA_ARGS__)
    #define DEBUG_EXEC_MOTION(code) do { code; } while(0)
#else
    #define DEBUG_PRINT_MOTION(fmt, ...) ((void)0)  // Compiles to nothing
    #define DEBUG_EXEC_MOTION(code) ((void)0)
#endif

// --- G-code Debug ---
#ifdef DEBUG_GCODE
    #define DEBUG_PRINT_GCODE(fmt, ...) UART_Printf(fmt, ##__VA_ARGS__)
    #define DEBUG_EXEC_GCODE(code) do { code; } while(0)
#else
    #define DEBUG_PRINT_GCODE(fmt, ...) ((void)0)
    #define DEBUG_EXEC_GCODE(code) ((void)0)
#endif

// --- Stepper Debug ---
#ifdef DEBUG_STEPPER
    #define DEBUG_PRINT_STEPPER(fmt, ...) UART_Printf(fmt, ##__VA_ARGS__)
    #define DEBUG_EXEC_STEPPER(code) do { code; } while(0)
#else
    #define DEBUG_PRINT_STEPPER(fmt, ...) ((void)0)
    #define DEBUG_EXEC_STEPPER(code) ((void)0)
#endif

// --- Segment Debug ---
#ifdef DEBUG_SEGMENT
    #define DEBUG_PRINT_SEGMENT(fmt, ...) UART_Printf(fmt, ##__VA_ARGS__)
    #define DEBUG_EXEC_SEGMENT(code) do { code; } while(0)
#else
    #define DEBUG_PRINT_SEGMENT(fmt, ...) ((void)0)
    #define DEBUG_EXEC_SEGMENT(code) ((void)0)
#endif

// --- UART Debug ---
#ifdef DEBUG_UART
    #define DEBUG_PRINT_UART(fmt, ...) UART_Printf(fmt, ##__VA_ARGS__)
    #define DEBUG_EXEC_UART(code) do { code; } while(0)
#else
    #define DEBUG_PRINT_UART(fmt, ...) ((void)0)
    #define DEBUG_EXEC_UART(code) ((void)0)
#endif

// --- Application Debug ---
#ifdef DEBUG_APP
    #define DEBUG_PRINT_APP(fmt, ...) UART_Printf(fmt, ##__VA_ARGS__)
    #define DEBUG_EXEC_APP(code) do { code; } while(0)
#else
    #define DEBUG_PRINT_APP(fmt, ...) ((void)0)
    #define DEBUG_EXEC_APP(code) ((void)0)
#endif

// ===== EXAMPLE USAGE IN CODE =====
// 
// // In motion.c:
// DEBUG_PRINT_MOTION("[MOTION] Loading segment %d\r\n", segment_id);
// DEBUG_EXEC_MOTION(LED1_Set());  // Visual indicator
//
// // In stepper.c ISR (use sparingly - keeps ISR fast):
// DEBUG_EXEC_STEPPER(LED2_Toggle());
//
// // In gcode_parser.c:
// DEBUG_PRINT_GCODE("[GCODE] Parsed: %s\r\n", token);
//
// ===== BENEFITS =====
// ✅ Zero runtime overhead in release builds
// ✅ No runtime checks (if/else eliminated by preprocessor)
// ✅ Clean, readable code that documents itself
// ✅ Easy to enable/disable entire subsystems
// ✅ Multiple subsystems can be debugged simultaneously
// ✅ Compiler removes all debug code in release (-O1 optimization)

// ===== TECHNICAL NOTES =====
// - ((void)0) is a no-op expression that produces no code
// - ##__VA_ARGS__ handles empty argument lists (GNU extension)
// - do { code; } while(0) ensures proper semicolon handling
// - Macros are evaluated at compile-time (preprocessor pass)
// - Release builds: -O1 optimization removes empty statements

#endif /* COMMON_H */