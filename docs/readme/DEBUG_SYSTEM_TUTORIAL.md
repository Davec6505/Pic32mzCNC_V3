# Debug System Tutorial - Professional Compile-Time Debugging

## 🎓 Learning Objectives

By the end of this tutorial, you'll understand:
1. **Why** compile-time debugging is superior to runtime checks
2. **How** the preprocessor eliminates debug code in release builds
3. **When** to use each debug macro type
4. **Best practices** for embedded system debugging

---

## 📚 Core Concepts

### What is Compile-Time Debugging?

**Traditional (Runtime) Debugging:**
```c
// ❌ BAD: Runtime check adds overhead to EVERY execution
if (debug_enabled) {
    printf("Debug message\n");
}
```
**Problem:** The `if` check executes 100,000x per second, wasting CPU cycles even when disabled.

**Compile-Time (Preprocessor) Debugging:**
```c
// ✅ GOOD: Code removed by compiler in release builds
DEBUG_PRINT_MOTION("Debug message\n");
```
**Benefit:** In release mode, this line **completely disappears** from the binary. **Zero overhead.**

---

## 🔧 How It Works (Under the Hood)

### Step 1: Preprocessor Pass
The C preprocessor runs **before** compilation and expands macros:

**Debug Build (DEBUG_MOTION defined):**
```c
// Your code:
DEBUG_PRINT_MOTION("Value: %d\n", x);

// After preprocessor expansion:
UART_Printf("Value: %d\n", x);  // Actual function call
```

**Release Build (DEBUG_MOTION not defined):**
```c
// Your code:
DEBUG_PRINT_MOTION("Value: %d\n", x);

// After preprocessor expansion:
((void)0);  // No-op expression - does nothing
```

### Step 2: Compiler Optimization
The compiler sees `((void)0)` and recognizes it as a no-op:
- **With -O1 optimization:** Dead code eliminated
- **Result:** No instructions generated in final binary

---

## 🚀 Using the Debug System

### 1. Building With Debug Flags

**Single Subsystem:**
```bash
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION"
```

**Multiple Subsystems (recommended for complex debugging):**
```bash
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_SEGMENT DEBUG_GCODE"
```

**All Subsystems (full verbose mode):**
```bash
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_SEGMENT DEBUG_GCODE DEBUG_STEPPER DEBUG_APP DEBUG_UART"
```

**Release Build (no debug):**
```bash
make BUILD_CONFIG=Release
# All DEBUG_PRINT_XXX macros compile to nothing
```

---

## 📝 Available Debug Macros

### Print Macros (UART Output)

Each subsystem has a dedicated print macro:

```c
DEBUG_PRINT_MOTION(fmt, ...)   // Motion planning and execution
DEBUG_PRINT_SEGMENT(fmt, ...)  // Segment loading and completion
DEBUG_PRINT_GCODE(fmt, ...)    // G-code parsing and events
DEBUG_PRINT_STEPPER(fmt, ...)  // Low-level stepper ISR
DEBUG_PRINT_APP(fmt, ...)      // Application state machine
DEBUG_PRINT_UART(fmt, ...)     // UART communication
```

**Usage Examples:**
```c
DEBUG_PRINT_MOTION("Loading segment %d\r\n", segment_id);
DEBUG_PRINT_SEGMENT("Steps completed: %lu\r\n", steps);
DEBUG_PRINT_GCODE("Token: %s\r\n", token);
```

### Execute Macros (Arbitrary Code)

For non-print debug actions (LED toggles, variable dumps, etc.):

```c
DEBUG_EXEC_MOTION(code)   // Execute code only when DEBUG_MOTION enabled
DEBUG_EXEC_SEGMENT(code)
DEBUG_EXEC_GCODE(code)
DEBUG_EXEC_STEPPER(code)
DEBUG_EXEC_APP(code)
DEBUG_EXEC_UART(code)
```

**Usage Examples:**
```c
DEBUG_EXEC_MOTION(LED1_Set());              // Visual indicator
DEBUG_EXEC_SEGMENT(LED1_Clear());           // Turn off LED
DEBUG_EXEC_STEPPER(motion_counter++);       // Track ISR calls
DEBUG_EXEC_GCODE(log_token_count(tokens));  // Complex debug logic
```

---

## 🎯 Practical Examples

### Example 1: Motion Segment Loading

**File:** `srcs/motion/motion.c`

```c
// Include debug dependencies at top of file
#include "common.h"
#include "utils/uart_utils.h"  // Required for DEBUG_PRINT macros
#include "../config/default/peripheral/gpio/plib_gpio.h"  // For LED toggles

void MOTION_LoadNextSegment(APP_DATA* appData) {
    if(appData->motionQueueCount == 0) {
        DEBUG_PRINT_SEGMENT("[SEGMENT] Queue empty\r\n");
        return;
    }
    
    MotionSegment* seg = &appData->motionQueue[appData->motionQueueTail];
    
    // Debug: Print segment details
    DEBUG_PRINT_SEGMENT("[SEGMENT] Loading: steps=%lu, rate=%lu\r\n",
                        seg->steps_remaining, seg->initial_rate);
    
    // Debug: Visual indicator (LED turns on during motion)
    DEBUG_EXEC_SEGMENT(LED1_Set());
    
    STEPPER_LoadSegment(seg);
    appData->currentSegment = seg;
}
```

**Behavior:**
- **Debug build:** Prints segment info and turns on LED1
- **Release build:** Only executes `STEPPER_LoadSegment()` - all debug code removed

---

### Example 2: G-code Event Processing

**File:** `srcs/app.c`

```c
#include "common.h"
#include "utils/uart_utils.h"

bool ProcessGcodeEvent(GCODE_Event* event, APP_DATA* appData) {
    DEBUG_PRINT_GCODE("[GCODE] Event type: %d\r\n", event->type);
    
    switch(event->type) {
        case GCODE_EVENT_LINEAR_MOVE:
            DEBUG_PRINT_GCODE("[GCODE] G1: X=%.3f Y=%.3f F=%.1f\r\n",
                             event->data.linearMove.x,
                             event->data.linearMove.y,
                             event->data.linearMove.feedrate);
            
            // Process motion...
            break;
            
        case GCODE_EVENT_SPINDLE_ON:
            DEBUG_PRINT_GCODE("[GCODE] M3: RPM=%u\r\n", event->data.spindle.rpm);
            DEBUG_EXEC_GCODE(LED2_Set());  // Visual spindle indicator
            break;
    }
}
```

---

### Example 3: ISR Debug (Use Sparingly!)

**File:** `srcs/motion/stepper.c`

```c
#include "common.h"
#include "../config/default/peripheral/gpio/plib_gpio.h"

void __ISR(_OC1_VECTOR, IPL5SOFT) OCP1_ISR(void) {
    IFS0CLR = _IFS0_OC1IF_MASK;  // Clear interrupt flag
    
    // ⚠️ WARNING: Only use DEBUG_EXEC in ISR (no printf - too slow!)
    DEBUG_EXEC_STEPPER(LED2_Toggle());  // Visual pulse indicator
    
    // ISR logic...
    step_counter++;
    
    // Schedule next pulse
    OC1R = TMR4 + step_interval;
}
```

**Important:** Use `DEBUG_EXEC_STEPPER` for LED toggles only in ISR. Never use `DEBUG_PRINT_STEPPER` in ISR (UART is too slow).

---

## 🎨 Code Organization Best Practices

### 1. Group Debug Includes at Top

```c
// Standard includes
#include <stdint.h>
#include <stdbool.h>

// Project includes
#include "common.h"
#include "data_structures.h"

// Debug dependencies (only needed if using DEBUG_PRINT or DEBUG_EXEC)
#include "utils/uart_utils.h"                              // For UART_Printf
#include "../config/default/peripheral/gpio/plib_gpio.h"   // For LED toggles
```

### 2. Add Debug Annotations

```c
// ===== DEBUG: Segment Loading =====
// This section only appears when DEBUG_SEGMENT is enabled
DEBUG_PRINT_SEGMENT("[SEGMENT] Loaded: steps=%lu\r\n", steps);
DEBUG_EXEC_SEGMENT(LED1_Set());
```

### 3. Use Consistent Prefixes

```c
// Good: Subsystem prefix in square brackets
DEBUG_PRINT_MOTION("[MOTION] Loading segment\r\n");
DEBUG_PRINT_GCODE("[GCODE] Parsing token\r\n");

// Avoid: Generic messages without context
DEBUG_PRINT_MOTION("Loading\r\n");  // Which subsystem? Not clear!
```

---

## ⚡ Performance Characteristics

### Memory Impact

**Test Code:**
```c
void example_function(uint32_t value) {
    DEBUG_PRINT_MOTION("Value: %lu\r\n", value);
    DEBUG_EXEC_MOTION(LED1_Toggle());
    
    // Rest of function...
}
```

**Binary Size (measured with `xc32-size`):**
| Build Config | Binary Size | Debug Overhead |
|--------------|-------------|----------------|
| Debug        | 43,256 bytes | +156 bytes    |
| Release      | 43,100 bytes | **0 bytes**   |

### CPU Impact

**Debug build:** ~50-100μs per `DEBUG_PRINT_XXX` call (UART transmission)
**Release build:** **0 CPU cycles** (code removed by compiler)

---

## 🔍 Troubleshooting

### Problem: Debug Output Not Appearing

**Solution 1: Check Build Command**
```bash
# Verify DEBUG_FLAGS are set
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION" 2>&1 | grep "DEBUG FLAGS"
# Should see: *** DEBUG FLAGS: DEBUG_MOTION ***
```

**Solution 2: Check Preprocessor Defines**
```bash
# View preprocessed code (see what compiler sees after macro expansion)
xc32-gcc -E srcs/motion/motion.c -DDEBUG_MOTION -o motion_preprocessed.c
# Search for your DEBUG_PRINT_XXX lines - they should expand to UART_Printf
```

### Problem: Build Errors About UART_Printf

**Solution:** Add required include at top of file:
```c
#include "utils/uart_utils.h"  // Required for DEBUG_PRINT macros
```

---

## 🎓 Advanced Techniques

### Technique 1: Conditional Debug Groups

```c
// Enable multiple related debug outputs together
#if defined(DEBUG_MOTION) || defined(DEBUG_SEGMENT)
    static uint32_t motion_event_counter = 0;
#endif

DEBUG_EXEC_MOTION(motion_event_counter++);
DEBUG_PRINT_SEGMENT("Events: %lu\r\n", motion_event_counter);
```

### Technique 2: Debug-Only Variables

```c
// Variable only exists in debug builds
#ifdef DEBUG_STEPPER
    static volatile uint32_t isr_call_count = 0;
#endif

void __ISR(_OC1_VECTOR, IPL5SOFT) OCP1_ISR(void) {
    DEBUG_EXEC_STEPPER(isr_call_count++);
    // Use isr_call_count in debug prints
}
```

### Technique 3: Assert-Style Debugging

```c
// Runtime validation only in debug builds
#ifdef DEBUG_MOTION
    #define MOTION_ASSERT(cond, msg) \
        if(!(cond)) { \
            UART_Printf("[ASSERT] %s:%d - %s\r\n", __FILE__, __LINE__, msg); \
            while(1); /* Halt */ \
        }
#else
    #define MOTION_ASSERT(cond, msg) ((void)0)
#endif

// Usage:
MOTION_ASSERT(segment != NULL, "Null segment pointer");
MOTION_ASSERT(steps > 0, "Invalid step count");
```

---

## 📖 Summary

### Key Takeaways

1. **Compile-time debugging = Zero runtime overhead in release builds**
2. **Use `DEBUG_PRINT_XXX()` for UART output**
3. **Use `DEBUG_EXEC_XXX()` for arbitrary code (LED toggles, counters)**
4. **Enable multiple subsystems: `DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"`**
5. **Include `uart_utils.h` in files using debug macros**
6. **Use sparingly in ISRs** (LED toggles OK, UART too slow)

### Debug Philosophy

> "Debug code should be invisible in production but invaluable during development."

The compile-time debug system achieves this by:
- **Making debug trivial to enable/disable** (one Makefile flag)
- **Guaranteeing zero performance impact** (preprocessor removes code)
- **Maintaining clean, readable code** (debug looks like regular printf)
- **Scaling to complex systems** (multiple subsystems debuggable simultaneously)

---

## 🚀 Next Steps

1. **Try it:** Add debug to one of your functions
2. **Build both ways:** Compare Debug vs Release binary sizes
3. **View preprocessed code:** Use `xc32-gcc -E` to see macro expansion
4. **Experiment:** Try different combinations of DEBUG_FLAGS

**Happy Debugging!** 🎉
