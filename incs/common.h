#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ╔════════════════════════════════════════════════════════════════════════════╗
// ║                       HARDWARE CONFIGURATION                               ║
// ║                                                                            ║
// ║  Per-axis stepper driver assignment.  Edit the four AXIS_x_DRIVER lines   ║
// ║  below to match your physical wiring.  Everything else is auto-derived.   ║
// ║                                                                            ║
// ║  Mixing is fully supported — e.g. X+Y = TMC5160, Z+A = DRV8825.          ║
// ╚════════════════════════════════════════════════════════════════════════════╝

// ── Driver type tokens (do NOT change these values) ─────────────────────────
#define DRIVER_TMC5160  1   // TMC5160 via SPI2  — shared ENN pin (active LOW)
#define DRIVER_DRV8825  2   // DRV8825 / A4988 / TMC2208 step-dir — uses shared EnXYZA pin

// ── Per-axis driver assignment: set each to DRIVER_TMC5160 or DRIVER_DRV8825 ─
#define AXIS_X_DRIVER   DRIVER_DRV8825  // DRIVER_TMC5160 when TMC drivers arrive
#define AXIS_Y_DRIVER   DRIVER_DRV8825  // DRIVER_TMC5160 when TMC drivers arrive
#define AXIS_Z_DRIVER   DRIVER_DRV8825
#define AXIS_A_DRIVER   DRIVER_DRV8825

// ── Auto-derived aggregate flags (do NOT edit) ───────────────────────────────
// HAS_TMC5160_AXIS — defined when ≥1 axis uses TMC5160; gates all SPI / TMC code.
#if (AXIS_X_DRIVER == DRIVER_TMC5160) || (AXIS_Y_DRIVER == DRIVER_TMC5160) || \
    (AXIS_Z_DRIVER == DRIVER_TMC5160) || (AXIS_A_DRIVER == DRIVER_TMC5160)
    #define HAS_TMC5160_AXIS
#endif

// HAS_DRV8825_AXIS — defined when ≥1 axis uses a discrete driver with per-axis EN.
#if (AXIS_X_DRIVER == DRIVER_DRV8825) || (AXIS_Y_DRIVER == DRIVER_DRV8825) || \
    (AXIS_Z_DRIVER == DRIVER_DRV8825) || (AXIS_A_DRIVER == DRIVER_DRV8825)
    #define HAS_DRV8825_AXIS
#endif

#if !defined(HAS_TMC5160_AXIS) && !defined(HAS_DRV8825_AXIS)
    #error "Hardware config: at least one AXIS_x_DRIVER must be set to DRIVER_TMC5160 or DRIVER_DRV8825"
#endif

// ── TMC5160_AXIS_MASK ─────────────────────────────────────────────────────────
// Compile-time bitmask: bit N set means axis N is a TMC5160.  Used by the
// mixed-driver enable inlines in utils.h.  Bit positions match E_AXIS (X=0 …).
#define TMC5160_AXIS_MASK \
    (((AXIS_X_DRIVER == DRIVER_TMC5160) ? (1u << 0) : 0u) | \
     ((AXIS_Y_DRIVER == DRIVER_TMC5160) ? (1u << 1) : 0u) | \
     ((AXIS_Z_DRIVER == DRIVER_TMC5160) ? (1u << 2) : 0u) | \
     ((AXIS_A_DRIVER == DRIVER_TMC5160) ? (1u << 3) : 0u))

// ── TMC5160 chopper mode tokens (only consulted for DRIVER_TMC5160 axes) ──────
//
//  TMC5160_MODE_STEALTHCHOP  Voltage-mode chopper.  Completely silent at all
//                            speeds.  Best for plotting, engraving, slow feeds.
//                            (GCONF.en_pwm_mode=1, TPWMTHRS=0)
//
//  TMC5160_MODE_SPREADCYCLE  Advanced current-mode chopper.  Highest torque
//                            and dynamics, slight audible switching noise.
//                            Best for high-speed cutting feeds.
//                            (GCONF.en_pwm_mode=0)
//
//  TMC5160_MODE_MIXED        StealthChop at low speed, automatic switch to
//                            SpreadCycle above TPWMTHRS velocity threshold.
//                            Best of both worlds for mixed-duty machines.
//                            (GCONF.en_pwm_mode=1, TPWMTHRS=cfg->tpwm_thrs)
//
//  TMC5160_MODE_COOLSTEP     MIXED + CoolStep load-adaptive current reduction.
//                            Saves up to 75% motor current at light loads.
//                            Requires TCOOLTHRS and COOLCONF tuning per motor.
//                            (as MIXED, plus TCOOLTHRS + COOLCONF written)
#define TMC5160_MODE_STEALTHCHOP  1
#define TMC5160_MODE_SPREADCYCLE  2
#define TMC5160_MODE_MIXED        3
#define TMC5160_MODE_COOLSTEP     4

// ── Per-axis TMC5160 chopper mode: set each to a TMC5160_MODE_xxx token ───────
// (Ignored for axes where AXIS_x_DRIVER == DRIVER_DRV8825)
#define AXIS_X_TMC_MODE   TMC5160_MODE_STEALTHCHOP
#define AXIS_Y_TMC_MODE   TMC5160_MODE_STEALTHCHOP
#define AXIS_Z_TMC_MODE   TMC5160_MODE_STEALTHCHOP
#define AXIS_A_TMC_MODE   TMC5160_MODE_STEALTHCHOP

#define NUM_OF_AXIS 4  // X, Y, Z, A

// E_AXIS enum is defined in data_structures.h to avoid circular dependencies
// Include data_structures.h if you need the enum

// Simple validation - no function overhead needed
#define IS_VALID_AXIS(axis) ((axis) < NUM_OF_AXIS)

// GRBL Protocol Constants - Configurable firmware identification
//
// TO CUSTOMIZE THE STARTUP BANNER:
// 1. Change GRBL_FIRMWARE_VERSION to your desired banner string
// 2. Must include \r\n at end for proper GRBL protocol compliance
// 3. G-code senders expect specific format: "Firmware_Name version ['$' for help]"
//
// Examples:
// #define GRBL_FIRMWARE_VERSION "Grbl 1.1h ['$' for help]\r\n"           // Standard GRBL
// #define GRBL_FIRMWARE_VERSION "Pic32mzCNC v1.2 ['$' for help]\r\n"     // Custom version
// #define GRBL_FIRMWARE_VERSION "MyCompany CNC v2.0 ['$' for help]\r\n"  // OEM version

#define GRBL_FIRMWARE_VERSION "Pic32mzCNC v1.1h ['$' for help]\r\n"
#define GRBL_BUILD_DATE "[Build Date: " __DATE__ "]\r\n"
#define GRBL_BUILD_TIME "[Build Time: " __TIME__ "]\r\n"

// Banner configuration macros - compile-time string and length calculation
#define STARTUP_BANNER_STRING GRBL_FIRMWARE_VERSION
#define STARTUP_BANNER_LENGTH (sizeof(GRBL_FIRMWARE_VERSION) - 1)  // -1 for null terminator

// Convenient macro for UART banner transmission (zero runtime overhead)
#define UART_SEND_BANNER() UART3_Write((uint8_t*)STARTUP_BANNER_STRING, STARTUP_BANNER_LENGTH)

// ╔════════════════════════════════════════════════════════════════════════════╗
// ║                          DEBUG INFRASTRUCTURE                              ║
// ║                                                                            ║
// ║  Professional compile-time debug system with ZERO runtime overhead         ║
// ║  in release builds.  Enable with DEBUG_FLAGS="DEBUG_MOTION DEBUG_G         
// ║  Debug code is completely removed from release builds via preprocessor     
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