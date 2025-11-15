# Debug System Learning Guide - Pic32mzCNC_V3

## Overview
This guide teaches you how to use the compile-time debug system for effective firmware debugging without performance impact in production builds.

## Quick Start Example

### Problem: Motion doesn't start after UGS soft reset

1. **Add debug output** to stepper module:
   ```c
   // In srcs/motion/stepper.c - STEPPER_LoadSegment()
   DEBUG_PRINT_STEPPER("[STEPPER] Loading segment: OC1CON=0x%08X, T4CON=0x%08X\r\n", 
                       (unsigned)OC1CON, (unsigned)T4CON);
   DEBUG_PRINT_STEPPER("[STEPPER] Timer freq: %lu Hz\r\n", TMR4_FrequencyGet());
   ```

2. **Build with debug flags**:
   ```powershell
   # From repository root (CRITICAL: always run make from root!)
   make clean
   make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_STEPPER DEBUG_MOTION"
   ```

3. **Flash and test** - Connect serial terminal, trigger soft reset (Ctrl+X), send motion command

4. **Observe debug output**:
   ```
   [STEPPER] Loading segment: OC1CON=0x00000000, T4CON=0x00000000
   [STEPPER] Timer freq: 781250 Hz
   [STEPPER] Re-enabling OC1 and TMR4 after soft reset
   ```

5. **Production build** - Remove debug with zero overhead:
   ```powershell
   make clean
   make BUILD_CONFIG=Release  # or just: make
   ```

## Debug System Architecture

### Compile-Time Conditional Compilation
Debug macros use preprocessor `#ifdef` to completely remove debug code in release builds:

```c
// incs/common.h
#ifdef DEBUG_STEPPER
    #define DEBUG_PRINT_STEPPER(fmt, ...) UART_Printf(fmt, ##__VA_ARGS__)
    #define DEBUG_EXEC_STEPPER(code) do { code } while(0)
#else
    #define DEBUG_PRINT_STEPPER(fmt, ...) ((void)0)
    #define DEBUG_EXEC_STEPPER(code) ((void)0)
#endif
```

**Result**: In release builds, `DEBUG_PRINT_STEPPER(...)` literally becomes `((void)0)` - a no-op that the compiler optimizes away completely.

### Available Debug Subsystems

| Flag | Purpose | Common Use Cases |
|------|---------|------------------|
| `DEBUG_MOTION` | Motion planning and segment execution | Motion not starting, wrong speeds, acceleration issues |
| `DEBUG_GCODE` | G-code parsing and event processing | Commands not recognized, protocol issues, UGS connection |
| `DEBUG_STEPPER` | Low-level stepper ISR and pulse generation | Hardware timing, pulse width, timer configuration |
| `DEBUG_SEGMENT` | Segment loading and queue management | Motion queue starvation, segment transitions |
| `DEBUG_UART` | UART communication | Communication timeouts, buffer overflows |
| `DEBUG_APP` | Application state machine | State transitions, initialization issues |

### Build System Integration

The debug system is integrated with the Makefile:

```makefile
# srcs/Makefile
ifdef DEBUG_FLAGS
    CPPFLAGS += $(addprefix -D,$(DEBUG_FLAGS))
endif
```

**Usage Pattern**:
```powershell
# Enable multiple subsystems
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE DEBUG_SEGMENT"

# Single subsystem
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_STEPPER"

# No debug (production)
make BUILD_CONFIG=Release
```

## Practical Debugging Workflows

### Debugging UGS Connection Issues

**Problem**: UGS connects but disconnects after sending `$$`

1. **Add G-code debug**:
   ```c
   // In gcode_parser.c - settings query handler
   DEBUG_PRINT_GCODE("[GCODE $$] Sending settings response\r\n");
   DEBUG_PRINT_GCODE("[GCODE $$] Response length: %u bytes\r\n", response_length);
   ```

2. **Enable debug and test**:
   ```powershell
   make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_GCODE"
   ```

3. **Observe output** - Look for buffer overflows, incomplete responses

4. **Fix identified issue** (e.g., increase TX buffer from 256 to 1024 bytes)

5. **Verify fix** in release build:
   ```powershell
   make clean && make
   ```

### Debugging Motion Timing Issues

**Problem**: Stepper motors make noise, irregular motion

1. **Add timing debug**:
   ```c
   // In stepper.c - OC1 ISR or step scheduling
   DEBUG_PRINT_STEPPER("[STEPPER] Step interval: %lu ticks (%.2f µs)\r\n", 
                       step_interval, (float)step_interval * 1.28f);
   DEBUG_PRINT_STEPPER("[STEPPER] Pulse width: %u ticks (%.2f µs)\r\n", 
                       pulse_width, (float)pulse_width * 1.28f);
   ```

2. **Build and observe**:
   ```powershell
   make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_STEPPER"
   ```

3. **Analyze timing** - Look for:
   - Pulse width < 1µs (too short for drivers)
   - Step intervals < 10µs (too fast)
   - Irregular timing patterns

### Debugging Arc Interpolation

**Problem**: G2/G3 commands produce incorrect arcs

1. **Add arc debug**:
   ```c
   // In app.c - arc generation
   DEBUG_PRINT_MOTION("[ARC] Theta: %.3f rad, Radius: %.3f mm\r\n", 
                      appData.arcTheta, appData.arcRadius);
   DEBUG_PRINT_MOTION("[ARC] Next point: X=%.3f, Y=%.3f\r\n", 
                      next.x, next.y);
   ```

2. **Build and test**:
   ```powershell
   make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION"
   ```

## Best Practices

### DO: Use Appropriate Debug Levels
```c
// Low-frequency events - always OK
DEBUG_PRINT_GCODE("[GCODE] Parsed command: %s\r\n", command);

// High-frequency events - use sparingly
DEBUG_PRINT_STEPPER("[STEPPER] Step %lu\r\n", step_count);

// ISR-safe alternatives
DEBUG_EXEC_STEPPER(LED1_Toggle());  // Visual indicator instead of UART
```

### DO: Self-Documenting Debug
```c
// Good - subsystem and context clear
DEBUG_PRINT_MOTION("[MOTION] Segment loaded: %lu steps, %.2f mm/min\r\n", steps, feedrate);

// Bad - unclear context
DEBUG_PRINT_MOTION("Value: %d\r\n", x);
```

### DO: Use Debug Execution Blocks
```c
// Complex debug logic
DEBUG_EXEC_GCODE({
    UART_Printf("[GCODE] Buffer hex dump: ");
    for(uint32_t i = 0; i < length; i++) {
        UART_Printf("%02X ", buffer[i]);
        if((i + 1) % 16 == 0) UART_Printf("\r\n");
    }
    UART_Printf("\r\n");
});
```

### DON'T: Mix Debug and Production Code
```c
// BAD - Manual debug that stays in code
char debug_buf[64];
snprintf(debug_buf, sizeof(debug_buf), "Debug: %d\r\n", value);
UART3_Write((uint8_t*)debug_buf, strlen(debug_buf));

// GOOD - Macro that disappears in release
DEBUG_PRINT_MOTION("Debug: %d\r\n", value);
```

### DON'T: Use Debug in Performance-Critical Paths
```c
// BAD - High-frequency ISR debug (will flood output)
void OC1_ISR() {
    DEBUG_PRINT_STEPPER("ISR fired\r\n");  // Called 100,000+ times/sec
    // ...
}

// GOOD - Use visual indicators instead
void OC1_ISR() {
    DEBUG_EXEC_STEPPER(LED2_Toggle());  // No UART, just LED
    // ...
}
```

## Troubleshooting

### Debug Output Not Appearing
1. **Check debug flag spelling**:
   ```powershell
   # Wrong
   make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"
   
   # Correct (check for typos)
   make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"
   ```

2. **Verify build configuration**:
   ```powershell
   # Make sure you're building Debug, not Release
   make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION"
   ```

3. **Check serial terminal connection** - 115200 baud, 8N1

### Too Much Debug Output
1. **Reduce debug scope**:
   ```powershell
   # Instead of all subsystems
   make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE DEBUG_STEPPER"
   
   # Focus on one issue
   make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION"
   ```

2. **Use conditional debug**:
   ```c
   // Only debug first few iterations
   static uint32_t debug_counter = 0;
   if(debug_counter < 10) {
       DEBUG_PRINT_MOTION("[MOTION] Iteration %lu\r\n", debug_counter);
       debug_counter++;
   }
   ```

### Build Errors with Debug
1. **Include required headers**:
   ```c
   #include "common.h"           // For DEBUG_ macros
   #include "utils/uart_utils.h" // For UART_Printf
   ```

2. **Check macro syntax**:
   ```c
   // Wrong - missing comma
   DEBUG_PRINT_MOTION("[MOTION] Value %d" value);
   
   // Correct
   DEBUG_PRINT_MOTION("[MOTION] Value %d\r\n", value);
   ```

## Advanced Techniques

### Conditional Debug by Feature
```c
// Enable debug only for specific features
#ifdef DEBUG_ARC_INTERPOLATION
    #define DEBUG_ARC_PRINT(fmt, ...) UART_Printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_ARC_PRINT(fmt, ...) ((void)0)
#endif

// Usage
DEBUG_ARC_PRINT("[ARC] Theta increment: %.6f rad\r\n", theta_inc);
```

### Performance Measurement
```c
// Time critical sections
DEBUG_EXEC_MOTION({
    uint32_t start_time = _CP0_GET_COUNT();
    // ... critical code ...
    uint32_t end_time = _CP0_GET_COUNT();
    UART_Printf("[PERF] Function took %lu cycles\r\n", end_time - start_time);
});
```

### Visual Debug Indicators
```c
// Use LEDs for high-frequency events where UART would be too slow
DEBUG_EXEC_STEPPER(LED1_Set());      // Motion started
DEBUG_EXEC_STEPPER(LED2_Toggle());   // Each step pulse
DEBUG_EXEC_MOTION(LED3_Clear());     // Motion completed
```

## Summary

The debug system provides:
- **Zero runtime overhead** in production builds
- **Selective debugging** by subsystem
- **Clean code** without manual debug clutter
- **ISR-safe** debug options
- **Professional** debugging workflow

Key commands to remember:
```powershell
# Debug build with specific flags
make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"

# Production build (removes all debug)
make clean && make

# Clean between builds
make clean
```

This system enables rapid debugging during development while ensuring production firmware has zero debug overhead.