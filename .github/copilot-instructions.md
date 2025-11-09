# GitHub Copilot Instructions for Pic32mzCNC_V3

## Project Overview
This is a CNC motion control system for PIC32MZ microcontrollers using hardware timers and Bresenham interpolation for precise multi-axis stepper motor control.
##    Always run make from root directory VI
To ensure proper build configuration and output paths, always execute `make` commands from the root directory of the Pic32mzCNC_V3 project. This guarantees that all relative paths and build settings are correctly applied. makefile incs target is dynamic, it knows the paths no need to add absolute file references, all paths are relative to the root directory.
## 🚀 Current Implementation Status (November 7, 2025)
### ✅ COMPLETED FEATURES
- **Professional event-driven G-code system** with clean architecture
- **Event queue implementation** respecting APP_DATA abstraction layer
- **Comprehensive G-code support**: G1, G2/G3, G4, M3/M5, M7/M9, G90/G91, F, S, T
- **Core architecture implemented** with period-based timer (TMR4/PR4)
- **Single instance pattern in appData** for clean separation  
- **Proper tokenization** - G/M commands keep all parameters (LinuxCNC/GRBL compatible)
- **Multi-command line support** - "G90G1X10Y10F1000" → ["G90", "G1X10Y10F1000"]
- **Modal parameter support** - Standalone F, S, T commands (GRBL v1.1 compliant)
- **16-command circular buffer** with flow control and overflow protection
- **Harmony state machine pattern** - proper APP_Tasks architecture
- **Non-blocking event processing** - one event per iteration
- **Kinematics module complete** with physics calculations and velocity profiling
- **Stepper module complete** with hardware abstraction and emergency stop
- **Persistent GRBL settings** with NVM flash storage (29 parameters including arc)
- **Delayed flash initialization** - read after peripherals ready (APP_LOAD_SETTINGS state)
- **Unified data structures** - no circular dependencies, clean module separation
- **Hardware FPU enabled** - Single-precision floating point for motion planning
- **Trapezoidal velocity profiling IMPLEMENTED** - KINEMATICS_LinearMove with full physics
- **Emergency stop system complete** - APP_ALARM state with hard/soft limit checking
- **Position tracking and modal state** - Work coordinates with G90/G91 support
- **Safety system complete** - STEPPER_DisableAll(), MOTION_UTILS_CheckHardLimits()
- **256 microstepping validated** - ISR budget analysis shows 42% headroom at 512kHz
- **Single ISR architecture designed** - GRBL pattern, no multi-ISR complexity
- **PRIORITY PHASE SYSTEM IMPLEMENTED** - Hybrid ISR/main loop architecture (best of both worlds!)
- **INCREMENTAL ARC INTERPOLATION COMPLETE** - Non-blocking G2/G3 with FPU acceleration
- **Motion phase system operational** - VELOCITY/BRESENHAM/SCHEDULE/COMPLETE phases
- **Project compiles successfully** with XC32 compiler
- **UART3 fully functional** - TX and RX working perfectly, status queries respond
- **Non-blocking UART utilities module** - uart_utils.c/h with callback-based output
- **Professional compile-time debug system** - Zero runtime overhead, multiple subsystems (November 7, 2025)
- **Clean build system** - make/make all defaults to Release, make build for incremental (November 7, 2025)
- **ATOMIC INLINE GPIO FUNCTIONS** - Zero-overhead ISR with `__attribute__((always_inline))` (November 8, 2025)
- **FUNCTION POINTER ARCHITECTURE** - Array-based axis control with GPIO_Control structs (November 8, 2025)
- **ISR STACK OPTIMIZATION** - Eliminated 32 bytes/call local arrays with static pointers (November 8, 2025)
- **PRE-CALCULATED DOMINANT AXIS** - Stored in MotionSegment, zero ISR overhead (November 8, 2025)

### 🔧 ATOMIC INLINE GPIO ARCHITECTURE (November 8, 2025)
**Module:** `incs/utils/utils.h`, `srcs/utils/utils.c`, `srcs/motion/stepper.c`

**Purpose:** Maximum ISR performance using PIC32 atomic SET/CLR/INV registers with always-inline functions.

**Key Architecture:**
```c
// AxisConfig structure (one per axis)
typedef struct {
    GPIO_Control step;           // Function pointers (compatibility)
    GPIO_Control dir;
    GPIO_Control enable;
    
    GPIO_Atomic step_atomic;     // Direct register pointers (ISR performance)
    GPIO_Atomic dir_atomic;
    GPIO_Atomic enable_atomic;
    
    float* max_rate;             // Settings pointers
    float* acceleration;
    float* steps_per_mm;
    int32_t* step_count;         // Position tracking
} AxisConfig;

// Atomic register structure
typedef struct {
    volatile uint32_t* set_reg;  // &LATxSET
    volatile uint32_t* clr_reg;  // &LATxCLR
    volatile uint32_t* inv_reg;  // &LATxINV
    volatile uint32_t* port_reg; // &PORTx
    uint32_t mask;               // Bit mask
} GPIO_Atomic;
```

**Always-Inline Helpers (Zero Overhead!):**
```c
static inline void __attribute__((always_inline)) AXIS_StepSet(E_AXIS axis) {
    *g_axis_config[axis].step_atomic.set_reg = g_axis_config[axis].step_atomic.mask;
}
// Compiles to single instruction: sw $t1, 0($t0)
```

**ISR Usage:**
```c
void OCP1_ISR(uintptr_t context) {
    AXIS_StepSet(dominant_axis);  // Single instruction atomic GPIO
    
    if (direction_bits & (1 << dominant_axis)) {
        AXIS_IncrementSteps(dominant_axis);  // Inlined (*ptr)++
    } else {
        AXIS_DecrementSteps(dominant_axis);  // Inlined (*ptr)--
    }
}
```

**Performance Benefits:**
- ✅ Single instruction GPIO writes (LATxSET atomic hardware)
- ✅ Zero function call overhead (always_inline attribute)
- ✅ No stack usage for GPIO operations
- ✅ Compiler-optimized to raw register access
- ✅ Maintains clean, readable code structure

**Available Inline Functions:**
- `AXIS_StepSet(axis)` / `AXIS_StepClear(axis)`
- `AXIS_DirSet(axis)` / `AXIS_DirClear(axis)`
- `AXIS_EnableSet(axis)` / `AXIS_EnableClear(axis)`
- `AXIS_IncrementSteps(axis)` / `AXIS_DecrementSteps(axis)`

### 🔧 COMPILE-TIME DEBUG SYSTEM (November 7, 2025)
**Module:** `incs/common.h`, `srcs/Makefile`, `docs/DEBUG_SYSTEM_TUTORIAL.md`

**Purpose:** Professional debug infrastructure with ZERO runtime overhead. Debug code is completely removed by compiler in release builds via preprocessor macros.

**Key Benefits:**
- ✅ **Zero runtime overhead** - Debug code eliminated in release builds
- ✅ **Multiple subsystems** - Enable/disable by subsystem (motion, gcode, stepper, etc.)
- ✅ **Clean syntax** - Looks like regular printf, works everywhere
- ✅ **No runtime checks** - Pure compile-time conditional compilation
- ✅ **ISR-safe** - Can use DEBUG_EXEC_XXX for LED toggles in interrupts

**Available Debug Flags:**
- `DEBUG_MOTION` - Motion planning and segment execution
- `DEBUG_GCODE` - G-code parsing and event processing
- `DEBUG_STEPPER` - Low-level stepper ISR and pulse generation
- `DEBUG_SEGMENT` - Segment loading and queue management
- `DEBUG_UART` - UART communication
- `DEBUG_APP` - Application state machine

**Build Usage:**
```powershell
# ⚠️ CRITICAL: ALWAYS run make from repository root directory
# NEVER cd into srcs before running make!

# Single subsystem (builds to Debug folder with debug symbols)
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION"

# Multiple subsystems
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE DEBUG_SEGMENT"

# Release build (no debug, default - builds to Release folder)
make
make BUILD_CONFIG=Release

# Clean build artifacts
make clean  # Cleans current BUILD_CONFIG (Debug or Release)
```

**Code Usage:**
```c
// Include required headers
#include "common.h"
#include "utils/uart_utils.h"  // For DEBUG_PRINT_XXX macros
#include "../config/default/peripheral/gpio/plib_gpio.h"  // For LED toggles

// Debug print (compiles to UART_Printf in debug, nothing in release)
DEBUG_PRINT_MOTION("[MOTION] Loading segment: steps=%lu\r\n", steps);

// Debug execute (compiles to code in debug, nothing in release)
DEBUG_EXEC_SEGMENT(LED1_Set());  // Visual indicator

// In release builds, both lines above compile to ((void)0) - zero overhead
```

**Documentation:** See `docs/DEBUG_SYSTEM_TUTORIAL.md` for complete guide with examples, best practices, and troubleshooting.

### ⚠️ CRITICAL DEBUG WORKFLOW (November 7, 2025)
**ALWAYS use the compile-time debug system instead of manual UART writes!**

❌ **WRONG - Manual Debug (DO NOT DO THIS):**
```c
// BAD: Manual UART writes that clutter code
char debug_buf[64];
snprintf(debug_buf, sizeof(debug_buf), "[DEBUG] Value: %d\r\n", value);
UART3_Write((uint8_t*)debug_buf, strlen(debug_buf));
```

✅ **CORRECT - Use Debug Macros:**
```c
// GOOD: Clean debug macros that compile to nothing in release
DEBUG_PRINT_GCODE("[GCODE] Value: %d\r\n", value);
```

**Debugging Protocol Issues (e.g., UGS connection):**
1. **Add debug macros** to the relevant code section:
   ```c
   DEBUG_PRINT_GCODE("[GCODE $] Buffer: nBytesRead=%u\r\n", (unsigned)nBytesRead);
   DEBUG_EXEC_GCODE({
       UART_Printf("[GCODE $] Hex dump: ");
       for(uint32_t i = 0; i < nBytesRead; i++) {
           UART_Printf("%02X ", rxBuffer[i]);
       }
       UART_Printf("\r\n");
   });
   ```

2. **Build with debug flag:**
   ```bash
   make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_GCODE"
   ```

3. **Flash and test** - debug output appears in terminal

4. **Release build** - debug code removed automatically:
   ```bash
   make clean && make BUILD_CONFIG=Release
   # or simply:
   make clean && make
   ```

**Why This Matters:**
- Manual debug code gets forgotten and left in production
- Debug macros are self-documenting (flag name shows what's being debugged)
- Zero performance impact in release builds
- Easy to enable/disable without code changes

### 🔧 NON-BLOCKING UART UTILITIES (November 6, 2025)
**Module:** `srcs/utils/uart_utils.c`, `incs/utils/uart_utils.h`

**Purpose:** Centralized non-blocking UART communication for debug output and GRBL protocol responses, preventing real-time motion interference.

**Implementation:**
- **Callback-based architecture**: UART3_WriteCallbackRegister() with persistent notifications
- **Global flag**: `volatile bool uart3TxReady` tracks TX buffer state
- **Event handler**: `UART_EVENT_WRITE_THRESHOLD_REACHED` sets flag when buffer ready
- **Non-blocking printf**: `UART_Printf()` checks flag before sending, drops messages if busy
- **Protocol helpers**: `UART_SendOK()`, `UART_IsReady()`

**Key Functions:**
```c
void UART_Initialize(void);           // Setup callback and notifications
bool UART_Printf(const char* fmt, ...); // Non-blocking formatted output
void UART_SendOK(void);               // Send "OK\r\n" response
bool UART_IsReady(void);              // Check TX ready state
```

**Initialization Pattern:**
```c
// In APP_Initialize() - called once at startup
UART_Initialize();  // Registers callback and enables notifications
```

**Usage Pattern:**
```c
// Non-blocking debug output (safe for ISR and main loop)
if (UART_Printf("[DEBUG] Value: %d\r\n", value)) {
    // Message sent successfully
} else {
    // TX busy, message dropped (no blocking)
}

// Protocol response
UART_SendOK();  // Sends "OK\r\n" when ready
```

**Benefits:**
- ✅ **No blocking** - Real-time motion never waits for UART
- ✅ **ISR-safe** - Can be called from interrupts (messages drop if busy)
- ✅ **Centralized** - Single module for all UART communication
- ✅ **Persistent callbacks** - No need to re-register after each write
- ✅ **Rate-limited** - Natural flow control via uart3TxReady flag

**Files Updated:**
- `srcs/app.c` - Added `#include "utils/uart_utils.h"`, calls UART_Initialize()
- `srcs/gcode/gcode_parser.c` - Uses UART_SendOK() for protocol responses
- `srcs/motion/motion.c` - Uses UART_Printf() for debug output
- `srcs/motion/stepper.c` - Uses UART_Printf() for debug output, LED2_Toggle() for visual confirmation

### 🔧 VISUAL MOTION DEBUG (November 6, 2025)
**Added:** `LED2_Toggle()` in `STEPPER_ScheduleStep()` (srcs/motion/stepper.c line ~125)

**Purpose:** Visual confirmation that motion scheduling is executing

**Behavior:**
- **Rapid blink (many Hz)** → `STEPPER_ScheduleStep()` IS being called → Motion system working
- **Slow blink (~1Hz heartbeat)** → Function NOT being called → Phase system or segment loading issue

**Usage:**
```gcode
G92 X0 Y0 Z0    # Set work origin
G1 X1 F100      # Move 1mm in X axis
```

**Observe LED2:**
- If rapid blink: Motion hardware OK, check if motors actually moving
- If slow blink: Motion segments not loading or phase system stuck

### 🔧 ACTIVE DEBUGGING SESSION (November 5-6, 2025)
**Problem:** G-code commands are accepted (OK response) but position never updates. Motion does not execute.

**Root Cause Found:** Commands are being queued but `GCODE_GetNextEvent()` is not successfully converting them to events that reach the motion system.

**Debug Progress Chain:**
1. ✅ **UART3 communication working** - Banner prints, status queries respond correctly
2. ✅ **G-code parser receiving commands** - "OK" responses confirm reception
3. ✅ **Tokenization working** - `[GCODE] Extract called, length=X`, `[GCODE] Tokens=X`
4. ✅ **Commands being queued** - `[GCODE] Queued: G1X10F100` confirms queue population
5. ✅ **GCODE_GetNextEvent() being called** - Function executes in APP_IDLE event loop
6. ✅ **Non-blocking UART implemented** - uart_utils module prevents motion blocking
7. ✅ **LED2 visual debug added** - Will show if STEPPER_ScheduleStep() executes
8. ❌ **CRITICAL ISSUE IDENTIFIED:** `parse_command_to_event()` is being called but events are NOT reaching APP_Tasks event processing
9. ❌ **No motion segments generated** - `[MOTION] Loading segment:` never prints
10. ❌ **Position stays 0.000** - Motion system never executes

**Debug Output Added (Currently in Code):**
- `srcs/gcode/gcode_parser.c`:
  - Line ~141: `[GCODE] Extract called, length=X` - Confirms buffer extraction
  - Line ~151: `[GCODE] Tokens=X` - Shows tokenization count
  - Line ~167: `[GCODE] Queued: <cmd>` - Shows what entered queue
  - Line ~217-228: `[GCODE] GetNextEvent:` and `parse_command_to_event returned:` (DISABLED - too verbose, floods UART)
- `srcs/app.c`:
  - Line ~269: `[APP] Event received: type=X` - Would show if event retrieved (NEVER PRINTS!)
  - Line ~322: `[APP] G1 Event: X Y Z F` - Would show linear move details (NEVER PRINTS!)
  - Line ~329: `[APP] Segment queued: count=X` - Would show segment added (NEVER PRINTS!)
- `srcs/motion/motion.c`:
  - Line ~237: `[MOTION] Loading segment: queueCount=X` - Would show segment load attempt (NEVER PRINTS!)
  - Line ~260: `[MOTION] Segment loaded: initial_rate=X` - Would show timing parameters (NEVER PRINTS!)
- `srcs/motion/stepper.c`:
  - Line ~125: `LED2_Toggle()` - Visual confirmation if STEPPER_ScheduleStep() called
  - Line ~132: `[STEPPER] Axis X: now=X, offset=X, pulse_start=X` - Would show step scheduling (NEVER PRINTS!)

**Observed Behavior:**
```
G92X0Y0Z0
[GCODE] Extract called, length=10
[GCODE] Tokens=2
[GCODE] Queued: G92X0Y0Z0
OK

G1X10F100
[GCODE] Extract called, length=10
[GCODE] Tokens=2
[GCODE] Queued: G1X10F100
OK

?
<Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|FS:0,0>
```

**Key Finding:** NO `[APP] Event received:` messages appear, meaning `GCODE_GetNextEvent()` returns false even though commands are in queue.

**Next Steps for Testing:**
1. **Flash firmware with LED2_Toggle()** - Visual confirmation of motion execution
2. **Test simple motion**: `G92 X0`, then `G1 X1 F100`
3. **Observe LED2 behavior**:
   - Rapid blink = `STEPPER_ScheduleStep()` executing → Check motor drivers
   - Slow blink = Function not called → Phase system or event parsing issue
4. **Enable selective debug** - Only G1/G92 events, not control characters
5. **Check parse_command_to_event() return value** - Is it returning false for valid G-code?

**Critical Hypothesis:** `parse_command_to_event()` is likely returning **false** for valid G-code commands (G92, G1), preventing events from being created. The `?` status query floods output when debug enabled, suggesting it's being parsed repeatedly without being consumed.

### 🔧 ACTIVE DEBUGGING SESSION - UGS CONNECTION (November 7, 2025)
**Problem:** UGS connects, sends commands (`?`, `$I`, `$$`), but immediately disconnects. Putty works correctly, confirming firmware responds.

**UGS Connection Sequence Observed:**
```
*** Connecting to jserialcomm://COM4:115200
*** Fetching device status
>>> ?
*** Fetching device version
>>> $I
<Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|FS:0,0>
*** Fetching device settings
>>> $$
*** Connection closed
```

**Key Findings:**
1. ✅ UGS sends `?` (status query) - firmware responds correctly
2. ❌ UGS sends `$I` (build info) - **NO RESPONSE SEEN**
3. ❌ UGS sends `$$` (settings) - **CONNECTION CLOSES**

**Root Cause Analysis:**
- **Putty works** → Firmware CAN respond to `$I` command
- **UGS fails** → Firmware NOT responding when UGS sends it
- **Hypothesis:** Byte-by-byte reception or timing difference between UGS and Putty

**GRBL Protocol Requirements (from UGS source code):**
- `$I` response format: `[VER: 1.1h.20251102:]\r\n[OPT: VHM,35,1024,4]\r\nok\r\n`
- **CRITICAL:** Space after colons required! `[VER: ]` not `[VER:]`
- **CRITICAL:** Space after colons required! `[OPT: ]` not `[OPT:]`

**Fixes Applied:**
1. ✅ Added spaces after colons in `SETTINGS_PrintBuildInfo()` (settings.c line ~427)
2. ✅ Added DEBUG_GCODE macros to `case '$':` handler in gcode_parser.c

**Debug Macros Added (November 7, 2025):**
```c
// In gcode_parser.c case '$': handler
DEBUG_PRINT_GCODE("[GCODE $] Entered $ handler: nBytesRead=%u\r\n", (unsigned)nBytesRead);
DEBUG_EXEC_GCODE({
    UART_Printf("[GCODE $] Buffer hex: ");
    for(uint32_t i = 0; i < nBytesRead && i < 10; i++) {
        UART_Printf("%02X ", rxBuffer[i]);
    }
    UART_Printf("\r\n");
});
DEBUG_PRINT_GCODE("[GCODE $] has_terminator=%d\r\n", has_terminator);
DEBUG_PRINT_GCODE("[GCODE $] bytes_available=%u\r\n", (unsigned)bytes_available);
DEBUG_PRINT_GCODE("[GCODE $] Complete command received: '%c%c' (0x%02X 0x%02X)\r\n", ...);
DEBUG_PRINT_GCODE("[GCODE $] Matched $I command, calling SETTINGS_PrintBuildInfo\r\n");
```

**Next Steps:**
1. Build with `make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_GCODE"`
2. Flash firmware
3. Connect with UGS and observe debug output
4. Check if `$I` command is being received and parsed correctly
5. Verify spaces in response format match GRBL spec

**Files Modified:**
- `srcs/settings/settings.c` - Fixed `$I` response format with spaces
- `srcs/gcode/gcode_parser.c` - Added DEBUG_GCODE macros for $ handler
- `.github/copilot-instructions.md` - Documented debug workflow (this file)

**Critical Hypothesis:** `parse_command_to_event()` is likely returning **false** for valid G-code commands (G92, G1), preventing events from being created. The `?` status query floods output when debug enabled, suggesting it's being parsed repeatedly without being consumed.

### 🔧 OLD DEBUGGING SESSION - MOTION NOT EXECUTING (November 5-6, 2025)

**Modified Files (November 5-6 debug session):**
- `srcs/utils/uart_utils.c` - Created non-blocking UART utilities module
- `incs/utils/uart_utils.h` - UART utilities header with function prototypes
- `srcs/motion/motion.c` - Added debug output (lines 237, 260), updated to use uart_utils
- `srcs/motion/stepper.c` - Added LED2_Toggle() (line ~125), debug output (line 132), uses uart_utils
- `srcs/app.c` - Added debug output (lines 269, 322, 329), calls UART_Initialize(), uses uart_utils
- `srcs/gcode/gcode_parser.c` - Uses UART_SendOK() for protocol responses
- `srcs/Makefile` - Added directory creation before linking, clean target removes all build artifacts
- `srcs/gcode/gcode_parser.c` - Added debug output (lines 141, 151, 167, 217-228), added UART3 include

**See README.md TODO section for remaining implementation tasks (homing, spindle/coolant, advanced features)**

## Core Architecture Principles

### Priority-Based Phase System (NEW - CRITICAL!)
**The "Best of Both Worlds" Hybrid Architecture**

**Problem Solved:**
- G-code processing could block motion timing (arc generation takes time)
- UART polled 512,000x/sec (wasteful CPU usage, only need 100x/sec)
- Need ISR precision but main loop flexibility

**Solution: Priority Phase System**
- **ISR sets flag** when dominant axis fires → wakes main loop
- **Main loop processes phases** in priority order (0 = highest)
- **G-code only runs when IDLE** → prevents blocking
- **Rate-limited UART** → polled every 10ms (not every μs)

**Phase Priorities:**
```c
typedef enum {
    MOTION_PHASE_IDLE = 255,      // Lowest - safe for G-code processing
    MOTION_PHASE_VELOCITY = 0,    // Highest - velocity conditioning
    MOTION_PHASE_BRESENHAM = 1,   // Bresenham error accumulation
    MOTION_PHASE_SCHEDULE = 2,    // OCx register scheduling
    MOTION_PHASE_COMPLETE = 3     // Segment completion
} MotionPhase;
```

**ISR Behavior (stepper.c):**
```c
void OCP5_ISR(uintptr_t context) {
    // X Axis - count steps
    if (direction_bits & (1 << AXIS_X)) {
        stepper_pos.x_steps++;
    } else {
        stepper_pos.x_steps--;
    }
    
    // ✅ CRITICAL: Signal main loop if X is dominant axis
    if (app_data_ref != NULL && app_data_ref->dominantAxis == AXIS_X) {
        app_data_ref->motionPhase = MOTION_PHASE_VELOCITY;  // Wake main loop
    }
}
```

**Main Loop Processing (app.c):**
```c
case APP_IDLE:
    switch(appData.motionPhase) {
        case MOTION_PHASE_VELOCITY:
            // Update currentStepInterval (accel/cruise/decel)
            appData.motionPhase = MOTION_PHASE_BRESENHAM;
            // Fall through to next phase (no break)
            
        case MOTION_PHASE_BRESENHAM:
            // Accumulate error terms, determine subordinate steps
            appData.motionPhase = MOTION_PHASE_SCHEDULE;
            // Fall through to next phase
            
        case MOTION_PHASE_SCHEDULE:
            // Write OCxR/OCxRS with absolute values
            appData.motionPhase = MOTION_PHASE_COMPLETE;
            // Fall through to next phase
            
        case MOTION_PHASE_COMPLETE:
            // Check segment done, load next from queue
            appData.motionPhase = MOTION_PHASE_IDLE;
            break;  // Exit phase processing
            
        case MOTION_PHASE_IDLE:
            // Safe for G-code processing
            break;
    }
    
    // ✅ Rate-limited UART (only when IDLE)
    if(appData.motionPhase == MOTION_PHASE_IDLE) {
        if(uartPollCounter >= 1250) {  // ~10ms
            GCODE_Tasks(&appData.gcodeCommandQueue);
            uartPollCounter = 0;
        }
    }
```

**Benefits:**
- ✅ **ISR precision** - dominant axis timing rock-solid
- ✅ **Main loop flexibility** - complex calculations without ISR bloat
- ✅ **Guaranteed execution order** - phases process in sequence
- ✅ **Non-blocking G-code** - motion always gets priority
- ✅ **CPU efficiency** - UART polled 100x/sec vs 512,000x/sec
- ✅ **Dynamic axis swapping** - ISR knows which axis is master

### Timer Architecture (Period-Based, TMR4/PR4)
- **TMR4 rolls over at PR4 value** - not free-running
- OC1 uses **relative compare values** against the rolling timer
- `PR4` sets the period (step interval + pulse width + margin)
- `OC1R` sets when pulse starts (step_interval)
- `OC1RS` sets when pulse ends (step_interval + pulse_width)
- Example: For 1ms steps with 3µs pulse: `OC1R = 12500`, `OC1RS = 12537`, `PR4 = 12539`
- **Hardware Configuration:**
  - PBCLK3 = 50MHz (peripheral bus clock)
  - Prescaler = 1:4 (TCKPS = 2)
  - Timer Frequency = 12.5MHz (50MHz / 4)
  - Timer Resolution = **80ns per tick** (1 / 12.5MHz)
- **No timer rollover issues** - TMR4 automatically resets to 0 at PR4, OCx values remain valid
- **Step timing** controlled entirely by OC1 ISR scheduling next pulse

### Dynamic Dominant Axis Tracking
- **Dominant axis** (highest step count) drives the step timing
- Dominant axis determines step_interval for OC1/PR4
- **Subordinate axes** step on-demand when Bresenham requires a step
- **Dominant axis can swap** mid-motion by recalculating Bresenham state

### Bresenham Integration  
- Bresenham algorithm runs in **OC1 ISR** for precise timing
- ISR: Generate step pulse, run Bresenham, schedule next step
- Error term updates happen in ISR each step
- Subordinate axis pulse generation based on error accumulation

### Single Instance Pattern in appData ✅
- **All major data structures** centralized in APP_DATA struct
- **No static module data** - clean separation of concerns  
- **Pass by reference** through function calls for explicit ownership
- **Work coordinates protected** by private static in kinematics module

### Professional G-Code Event System ✅
- **Event-driven architecture**: Clean `GCODE_GetNextEvent()` interface
- **Comprehensive G-code support**: G1, G2/G3, G4, M3/M5, M7/M9, G90/G91, F, S, T commands
- **Proper tokenization**: G/M commands consume ALL parameters until next G/M
  - Example: `G90G1X10Y10F1000` → Tokens: `"G90"`, `"G1X10Y10F1000"`
  - Example: `G1X10F1000` → Token: `"G1X10F1000"` (stays together)
- **Modal parameter support**: Standalone F, S, T commands (LinuxCNC/GRBL compatible)
  - `F1500` → Changes feedrate without motion
  - `S2000` → Changes spindle speed
  - `T1` → Tool change
- **Abstraction layer respect**: No APP_DATA exposure, maintains clean boundaries
- **Zero memory allocation**: Deterministic processing for real-time systems
- **Utils module**: Professional string parsing with robust tokenization
- **Flow control**: 16-command circular buffer with "OK" withholding
- **Real-time characters**: Bypass tokenization for immediate '?!~^X' processing
- **GRBL compliance**: Full v1.1 protocol support with proper status reporting
- **Non-blocking design**: One event processed per APP_Tasks() iteration

## Code Style Guidelines

### ISR Implementation
```c
void __ISR(_OC1_VECTOR, IPL5SOFT) OC1Handler(void) {
    // Clear interrupt flag FIRST
    IFS0CLR = _IFS0_OC1IF_MASK;
    
    // Schedule next pulse using period-based timing
    uint32_t step_interval = current_segment->step_interval;
    uint32_t pulse_width = current_segment->pulse_width;
    
    OC1R = step_interval;                      // Pulse start
    OC1RS = step_interval + pulse_width;       // Pulse end
    PR4 = step_interval + pulse_width + 2;     // Timer period
    
    // Update step counter
    steps_completed++;
    
    // Run Bresenham for subordinate axes
    // ... (Bresenham code in ISR)
}
```

### Compare Register Updates
```c
// CORRECT - Period-based timing
uint32_t step_interval = 12500;  // Timer ticks for this step
uint32_t pulse_width = 37;        // 3µs pulse width
OC1R = step_interval;             // Pulse starts at interval
OC1RS = step_interval + pulse_width;  // Pulse ends
PR4 = step_interval + pulse_width + 2;  // Timer rolls over

// INCORRECT - Don't use absolute timer reads
uint32_t now = TMR4;  // WRONG - timer is period-based!
OC1R = now + 500;     // WRONG - ignores period rollover
```

### Subordinate Axis Scheduling
```c
// Schedule subordinate axis only when needed (in ISR)
if (error_y >= delta_x) {
    // Y axis needs a step this cycle
    LATCSET = (1 << 2);  // Set Y step pin HIGH
    error_y -= delta_x;
}
// TMR5 will clear all step pins after pulse width
```

### Disabling Pulse Generation
```c
// To stop pulse generation (end of segment):
TMR4_Stop();  // Stops all motion
```
// To stop pulse generation on any axis (for safety/sanity):
// Set OCxR = OCxRS (equal values prevent compare match pulse)
OC2R = OC2RS;  // Y axis stops generating pulses
OC3R = OC3RS;  // Z axis stops generating pulses

// This is critical during:
// - Arc interpolation when axis doesn't need steps
// - Bresenham cycles where subordinate axis is inactive
// - Motion completion or emergency stop scenarios
```

### G-Code Line Buffering Pattern
```c
case GCODE_STATE_IDLE:
    nBytesAvailable = UART2_ReadCountGet();

    if(nBytesAvailable > 0){
        // ✅ Accumulate bytes until complete line
        uint32_t space_available = sizeof(rxBuffer) - nBytesRead - 1;
        uint32_t bytes_to_read = (nBytesAvailable < space_available) ? nBytesAvailable : space_available;
        
        if (bytes_to_read > 0) {
            uint32_t new_bytes = UART2_Read((uint8_t*)&rxBuffer[nBytesRead], bytes_to_read);
            nBytesRead += new_bytes;
        }
        
        // ✅ CRITICAL: Static variable persists across GCODE_Tasks() calls
        // Allows line terminator state to accumulate during byte reception
        static bool has_line_terminator = false;
        for(uint32_t i = 0; i < nBytesRead; i++) {
            if(rxBuffer[i] == '\n' || rxBuffer[i] == '\r') {
                has_line_terminator = true;
                break;
            }
        }

        // Check first byte for control characters (bypass line buffering)
        bool control_char_found = false;
        for(uint8_t i = 0; i < sizeof(GRBL_CONTROL_CHARS); i++){
            if(rxBuffer[0] == GRBL_CONTROL_CHARS[i]){
                control_char_found = true;
                gcodeData.state = GCODE_STATE_CONTROL_CHAR;   
                break;          
            }
        }

        // ✅ CRITICAL: Early exit if no line terminator found yet
        // Prevents premature processing during byte accumulation
        if(!has_line_terminator){
            break;
        }

        // ✅ Check for actual G-code content (not just whitespace)
        bool has_gcode = false;
        for(uint32_t i = 0; i < nBytesRead; i++) {
            if(rxBuffer[i] != '\r' && rxBuffer[i] != '\n' && 
                rxBuffer[i] != ' ' && rxBuffer[i] != '\t' && rxBuffer[i] != 0) {
                has_gcode = true;
                break;
            }
        }

        // Process complete line based on content
        if(control_char_found){              
            // Fall through to GCODE_STATE_CONTROL_CHAR immediately
        
        } else if(has_line_terminator && has_gcode){
            // ✅ Complete G-code line with actual content - process it
            cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, nBytesRead, cmdQueue);
        
            // ✅ Send ONE "OK" per complete G-code line (GRBL v1.1 protocol)
            UART2_Write((uint8_t*)"OK\r\n", 4);
            
            // Clear buffer and reset static flag for next line
            nBytesRead = 0; 
            memset(rxBuffer, 0, sizeof(rxBuffer));
            gcodeData.state = GCODE_STATE_IDLE;
            has_line_terminator = false;  // ✅ CRITICAL: Reset static variable
            break;
            
        } else if(has_line_terminator && !has_gcode){
            // ✅ Line terminator but no G-code (empty line or whitespace only)
            // Clear buffer, don't send "OK", stay in IDLE
            nBytesRead = 0; 
            memset(rxBuffer, 0, sizeof(rxBuffer));
            gcodeData.state = GCODE_STATE_IDLE;
            has_line_terminator = false;  // ✅ CRITICAL: Reset static variable
            break;
        }
    } else {
        gcodeData.state = GCODE_STATE_IDLE;
        break;
    }
    // ✅ Fall through to GCODE_STATE_CONTROL_CHAR only when control_char_found = true

case GCODE_STATE_CONTROL_CHAR:
    switch(rxBuffer[0]) {
        case '?':  // Status query - NO "OK" response
            // Send status report only
            break;
        case '~':  // Cycle start/resume - NO response (real-time)
            break;
        case '!':  // Feed hold - NO response (real-time)
            break;
        case 0x18: // Soft reset (Ctrl+X) - startup banner ONLY
            UART2_Write((uint8_t*)GRBL_FIRMWARE_VERSION, sizeof(GRBL_FIRMWARE_VERSION));
            break;
        default:
            // Unknown control char - silently ignore (GRBL behavior)
            break;
    }
    
    // Clear buffer and return to idle
    nBytesRead = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));
    gcodeData.state = GCODE_STATE_IDLE;
    break;
```

### G-Code Event Processing Pattern
```c
// Professional event-driven processing in APP_Tasks
GCODE_Event event;
while (GCODE_GetNextEvent(&appData.gcodeCommandQueue, &event)) {
    switch (event.type) {
        case GCODE_EVENT_LINEAR_MOVE:
            // Convert to motion segment using kinematics
            MotionSegment segment;
            KINEMATICS_LinearMove(currentPos, targetPos, 
                                event.data.linearMove.feedrate, &segment);
            
            // Add to motion queue through YOUR abstraction layer
            if (appData.motionQueueCount < MAX_MOTION_SEGMENTS) {
                memcpy(&appData.motionQueue[appData.motionQueueHead], 
                       &segment, sizeof(MotionSegment));
                appData.motionQueueHead = (appData.motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
                appData.motionQueueCount++;
            }
            break;
            
        case GCODE_EVENT_SPINDLE_ON:
            // Handle spindle control through your interfaces
            SPINDLE_SetSpeed(event.data.spindle.rpm);
            break;
            
        case GCODE_EVENT_ARC_MOVE:
            // Handle arc interpolation
            // Use event.data.arcMove for center, target, direction
            break;
    }
}
```

## Important Constraints

### Never Do
- ❌ Use absolute timer reads for scheduling (use period-based intervals)
- ❌ Set PR4 smaller than OC1RS (pulse won't complete)
- ❌ Use blocking delays in main loop (let APP_Tasks run freely)
- ❌ Modify OC1R/OC1RS outside of ISR during active motion

### Always Do
- ✅ Use period-based timing: `OC1R = step_interval; PR4 = step_interval + pulse_width + margin`
- ✅ Clear interrupt flags immediately at ISR entry
- ✅ Keep ISRs minimal and fast
- ✅ Run Bresenham logic in OC1 ISR for precise timing
- ✅ Schedule subordinate axes only when required
- ✅ **Set OCxR = OCxRS to disable pulse generation** (prevents spurious pulses)

## Data Structures

### Segment Structure (Recommended)
```c
typedef struct {
    // Delta values for Bresenham
    int32_t delta_x, delta_y, delta_z, delta_a;
    
    // Error accumulators
    int32_t error_y, error_z, error_a;
    
    // Step counts
    uint32_t steps_remaining;
    uint32_t steps_completed;
    
    // Timing
    uint32_t step_interval;      // Ticks between dominant axis steps
    uint32_t pulse_width;        // Pulse width in ticks
    
    // Dominant axis designation
    enum { AXIS_X, AXIS_Y, AXIS_Z, AXIS_A } dominant_axis;
} MotionSegment;
```

## Future Enhancements

### Velocity Profiling (Under Consideration)
When implementing velocity profiling in the future:
- Adjust `step_interval` dynamically based on acceleration state
- Maintain trapezoidal or S-curve velocity profiles
- Consider look-ahead planning for smooth transitions between segments
- Pre-compute interval values or use real-time calculation
- Update dominant axis OCx scheduling with variable intervals

**Note:** Velocity profiling is not yet implemented but should be designed with the period-based timer architecture in mind.

## Hardware Details
- **Microcontroller:** PIC32MZ2048EFH100
- **System Clock:** 200MHz
- **Peripheral Bus Clock (PBCLK3):** 50MHz
- **Compiler:** XC32
- **Build System:** Make
- **Bootloader:** MikroE USB HID Bootloader (39KB, starts at 0x9D1F4000)
- **Hardware FPU:** Single-precision floating point unit enabled
  - Compiler flags: `-mhard-float -msingle-float -mfp64`
  - Optimizations: `-ffast-math -fno-math-errno`
  - Use FPU for planning (KINEMATICS), integer math for ISR
- **Timer Configuration:**
  - TMR4 16-bit timer (period-based, rolls over at PR4)
  - Prescaler: 1:4 (TCKPS = 2)
  - Timer Frequency: 12.5MHz
  - **Timer Resolution: 80ns per tick**
  - TMR5 16-bit timer for step pulse width (one-shot mode)
- **Output Compare Modules:** OC1 (X), OC2 (Y), OC3 (Z), OC4 (A)
- **Microstepping Support:** Designed for up to 256 microstepping
  - Worst case: 512kHz step rate (256 microsteps × high speed)
  - ISR budget: ~390 CPU cycles @ 512kHz (225 cycles used)
  - Per-step timing: ~24 timer ticks minimum

### Memory Layout (PIC32MZ2048EFH100 - 2MB Flash)
```
Physical Address    Virtual (KSEG1)     Size        Purpose
----------------    ---------------     ----        -------
0x1D000000          0x9D000000          1.87MB      Application Code
0x1D1F0000          0xBD1F0000          16KB        GRBL Settings Storage (NVM)
0x1D1F4000          0x9D1F4000          48KB        MikroE USB HID Bootloader
0x1FC00000          0xBFC00000          12KB        Boot Flash (Config Words)
```

**Critical Memory Rules:**
- ✅ **Settings NVM:** `0xBD1F0000` (KSEG1 virtual for both reads and writes)
- ✅ **Page-aligned:** 16KB boundaries (0x4000)
- ✅ **Row-aligned:** 2048-byte boundaries (0x800)
- ✅ **Safe margin:** 64KB (0x10000) before bootloader at 0xBD1F4000
- ❌ **Never write to:** 0x9D1F4000 / 0xBD1F4000 (bootloader region)
- ❌ **Never write to:** 0xBFC00000 (boot flash config)

**Address Space (MIPS Architecture):**
- **Physical (0x1D...):** Internal flash controller addressing
- **Virtual KSEG1 (0xBD...):** Uncached - REQUIRED for NVM operations (reads and writes)
- **Virtual KSEG0 (0x9D...):** Cached - used for code execution

**NVM Operations (Harmony Pattern):**
- Use **KSEG1 (0xBD...)** addresses for all NVM operations
- Harmony NVM drivers handle address conversion internally
- Flash page size: 16KB (must erase entire page before writing)
- Flash row size: 2048 bytes (512 words) - unit of RowWrite operations
- **Cache-aligned buffers REQUIRED:** Use `CACHE_ALIGN` attribute
- **Callback pattern:** Register handler, wait on `xferDone` flag
- **RowWrite preferred:** One operation vs 41 WordWrite operations for settings

## File Organization
- `srcs/main.c` - Entry point, main loop calls APP_Tasks()
- `srcs/app.c` - Application state machine (single instance pattern)
- `srcs/gcode/gcode_parser.c` - G-code parsing & GRBL protocol ✅
- `srcs/gcode/utils.c` - Professional string tokenization utilities ✅
- `srcs/motion/stepper.c` - Hardware abstraction layer ✅  
- `srcs/motion/motion.c` - Master motion controller ✅
- `srcs/motion/kinematics.c` - Physics calculations ✅
- `srcs/settings/settings.c` - Persistent GRBL settings with NVM flash ✅
- `incs/data_structures.h` - Unified data structures (no circular dependencies) ✅
- `incs/common.h` - Shared constants and enums
- `docs/plantuml/` - Architecture diagrams:
  - `01_system_overview.puml` - High-level system architecture
  - `02_segment_clock.puml` - Timer and motion segment flow
  - `03_arc_linear_interpolation.puml` - Arc/linear interpolation system
- `README.md` - Complete project documentation with TODO list

## Unified Data Structures (Completed ✅)

### Architecture Pattern
All major data structures consolidated in `incs/data_structures.h` to eliminate circular dependencies and provide clean module separation.

**Structure Hierarchy:**
```c
// incs/data_structures.h
- E_AXIS enum (AXIS_X, AXIS_Y, AXIS_Z, AXIS_A, NUM_AXIS)
- MotionSegment (Bresenham, physics, timing)
- GCODE_Command
- GCODE_CommandQueue
  ├── commands[16]
  ├── head, tail, count
  └── motionQueueCount, maxMotionSegments (nested for flow control)
- APP_STATES enum
- APP_DATA
  ├── state
  ├── gcodeCommandQueue (with nested motion info)
  └── motionQueue[16], head, tail, count
```

### Flow Control Infrastructure (Ready to Implement)
The nested motion queue info in `GCODE_CommandQueue` enables flow control without circular dependencies:

```c
// app.c syncs motion queue status before GCODE processing
case APP_IDLE:
    // ✅ Sync motion queue count for flow control
    appData.gcodeCommandQueue.motionQueueCount = appData.motionQueueCount;
    
    // Now GCODE_Tasks can check motion buffer occupancy
    GCODE_Tasks(&appData.gcodeCommandQueue);
    break;
```

### Benefits of Unified Structures
- ✅ **No circular dependencies** - all structures in one header
- ✅ **Clean module separation** - data vs logic separation
- ✅ **Single source of truth** - structure definitions in one place
- ✅ **Easy to test** - mock entire APP_DATA structure
- ✅ **Flow control ready** - nested motion queue info accessible to G-code parser

## Settings Implementation (Completed ✅)

### Critical Timing Requirement
- **NEVER read flash during SETTINGS_Initialize()** - will hang on boot
- **Must delay NVM_Read() until after all peripherals initialized**
- **Solution:** APP_LOAD_SETTINGS state executes after APP_CONFIG

### Implementation Pattern
```c
// SETTINGS_Initialize() - called from main.c after SYS_Initialize()
void SETTINGS_Initialize(void) {
    NVM_CallbackRegister(eventHandler, (uintptr_t)NULL);  // Register once
    SETTINGS_RestoreDefaults(&current_settings);          // Load defaults only
    // DO NOT read flash here - peripherals not ready!
}

// APP_LOAD_SETTINGS state - executes after APP_CONFIG in APP_Tasks()
case APP_LOAD_SETTINGS:
    if (SETTINGS_LoadFromFlash(SETTINGS_GetCurrent())) {
        // Flash settings loaded successfully
    }
    appData.state = APP_IDLE;
    break;
```

### NVM Write Pattern (Harmony)
```c
// Cache-aligned buffer (CRITICAL for PIC32MZ)
static uint32_t writeData[BUFFER_SIZE] CACHE_ALIGN;

// PageErase + RowWrite with callback
NVM_PageErase(address);
while(xferDone == false);
xferDone = false;

NVM_RowWrite((uint32_t *)writePtr, address);
while(xferDone == false);
xferDone = false;
```

### Why RowWrite vs WordWrite
- Settings = 164 bytes (fits in ONE row of 2048 bytes)
- Must erase entire 16KB page anyway
- RowWrite = 1 operation vs WordWrite = 41 operations
- More efficient, more reliable, matches Harmony pattern

## Arc Interpolation Implementation (COMPLETED ✅ November 4, 2025)

### Overview
Arc interpolation provides smooth circular motion for G2/G3 commands. The **incremental streaming architecture** generates one segment per iteration, preventing motion queue starvation and enabling non-blocking operation.

### Implementation Architecture

**Incremental Streaming Pattern:**
- **Non-blocking**: ONE segment generated per APP_Tasks() iteration
- **Self-regulating**: Only generates when motion queue has space
- **FPU-accelerated**: Hardware sin/cos for smooth arcs (50-100μs per segment)
- **Exact end point**: Final segment uses target coordinates (no accumulated error)
- **Queue never empties**: Continuous flow during arc execution

**Arc Generation State Machine:**
```c
typedef enum {
    ARC_GEN_IDLE = 0,      // No arc in progress
    ARC_GEN_ACTIVE         // Arc generation active
} ArcGenState;
```

**Arc Parameters in APP_DATA:**
```c
ArcGenState arcGenState;
float arcTheta;                    // Current angle (radians)
float arcThetaEnd;                 // Target angle (radians)
float arcThetaIncrement;           // Angle step per segment (radians)
CoordinatePoint arcCenter;         // Arc center point (absolute)
CoordinatePoint arcCurrent;        // Current position on arc
CoordinatePoint arcEndPoint;       // Final arc destination
float arcRadius;                   // Arc radius (mm)
bool arcClockwise;                 // G2=true, G3=false
uint8_t arcPlane;                  // G17/G18/G19
float arcFeedrate;                 // Arc feedrate (mm/min)
uint8_t modalPlane;                // Modal plane state (G17=0, G18=1, G19=2)
```

### Arc Math (GRBL v1.1 Compatible)

**G-code Format:**
- **G2**: Clockwise arc in current plane
- **G3**: Counter-clockwise arc in current plane
- **Parameters**:
  - `X Y Z`: End point coordinates (absolute or relative based on G90/G91)
  - `I J K`: Center offset from **start point** (always incremental)
  - Example: `G2 X10 Y0 I5 J0` → Arc from current pos to (10,0), center at (5,0), radius=5mm

**Arc Calculations (implemented in app.c lines 487-574):**
1. **Center point**: `center.x = start.x + centerX`, `center.y = start.y + centerY`
2. **Radius verification**: 
   - `r_start = sqrt(centerX² + centerY²)` (radius from start to center)
   - `r_end = sqrt((end.x - center.x)² + (end.y - center.y)²)` (radius from end to center)
   - If `|r_start - r_end| > 0.005mm`, trigger ALARM:33 (arc radius error)
3. **Angles**:
   - `start_angle = atan2f(start.y - center.y, start.x - center.x)`
   - `end_angle = atan2f(end.y - center.y, end.x - center.x)`
   - `total_angle` with wrap-around handling for CW/CCW direction
4. **Arc length**: `arc_length = radius × total_angle` (in radians)
5. **Segment count**: `segments = ceil(arc_length / mm_per_arc_segment)` (GRBL setting $12)

### Incremental Arc Generator (app.c lines 590-641)

**Operation Pattern:**
```c
// Runs in APP_IDLE state when arcGenState == ARC_GEN_ACTIVE
if(appData.arcGenState == ARC_GEN_ACTIVE && appData.motionQueueCount < MAX_MOTION_SEGMENTS) {
    
    appData.arcTheta += appData.arcThetaIncrement;  // Increment angle
    
    CoordinatePoint next;
    
    // Check if this is the last segment
    bool is_last_segment = (appData.arcClockwise && appData.arcTheta <= appData.arcThetaEnd) ||
                          (!appData.arcClockwise && appData.arcTheta >= appData.arcThetaEnd);
    
    if(is_last_segment) {
        // Use exact end point to prevent accumulated error
        next = appData.arcEndPoint;
        appData.arcGenState = ARC_GEN_IDLE;
        
    } else {
        // Calculate intermediate point using sin/cos (FPU accelerated)
        next.x = appData.arcCenter.x + appData.arcRadius * cosf(appData.arcTheta);
        next.y = appData.arcCenter.y + appData.arcRadius * sinf(appData.arcTheta);
        
        // Linear interpolation for Z and A axes (helical motion)
        float progress = fabsf(appData.arcTheta - atan2f(...)) / total_angle;
        next.z = arcCurrent.z + (arcEndPoint.z - arcCurrent.z) * progress;
        next.a = arcCurrent.a + (arcEndPoint.a - arcCurrent.a) * progress;
    }
    
    // Generate motion segment for this arc increment
    MotionSegment* segment = &appData.motionQueue[appData.motionQueueHead];
    KINEMATICS_LinearMove(appData.arcCurrent, next, appData.arcFeedrate, segment);
    
    // Add to motion queue
    appData.motionQueueHead = (appData.motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
    appData.motionQueueCount++;
    
    // Update current position
    appData.arcCurrent = next;
    
    // Update work coordinates when arc completes
    if(appData.arcGenState == ARC_GEN_IDLE) {
        appData.currentX = appData.arcEndPoint.x;
        appData.currentY = appData.arcEndPoint.y;
        appData.currentZ = appData.arcEndPoint.z;
        appData.currentA = appData.arcEndPoint.a;
    }
}
```

### Key Features

✅ **GRBL v1.1 Compatible Arc Math**
- Radius validation prevents malformed arcs
- Angle calculation with proper wrap-around
- CW (G2) and CCW (G3) direction support

✅ **Helical Motion Support**
- Z-axis linear interpolation during arc
- A-axis (rotary) linear interpolation
- Full 4-axis helical toolpaths

✅ **Plane Selection (Modal State)**
- G17: XY plane (default)
- G18: XZ plane
- G19: YZ plane
- Modal state tracked in `appData.modalPlane`

✅ **Performance Characteristics**
- Non-blocking: 50-100μs per segment generation
- FPU-accelerated: sin/cos in hardware
- Memory efficient: No buffer overflow possible
- Self-regulating: Checks queue space before adding

✅ **GRBL Setting Integration**
- `$12` - mm_per_arc_segment (default 0.1mm)
- Accessible via SETTINGS_GetCurrent()
- Persistent in NVM flash storage

### Testing Arc Interpolation

**Test G-code:**
```gcode
G17           ; Select XY plane
G90           ; Absolute positioning
G0 X0 Y0      ; Rapid to origin
G1 F500       ; Set feedrate to 500 mm/min
G2 X10 Y0 I5 J0   ; CW arc from (0,0) to (10,0), center at (5,0), radius=5mm
```

**Expected Behavior:**
- Arc broken into ~157 segments (π×10mm / 0.1mm per segment ≈ 31.4 for semicircle)
- Smooth circular motion in XY plane
- Final position: (10.0, 0.0)
- Motion queue never runs empty during arc
- Status query shows continuous motion: `<Run|MPos:...>`
    float r_start = sqrtf(I*I + J*J);
    float r_end = sqrtf((end.x-center.x)*(end.x-center.x) + (end.y-center.y)*(end.y-center.y));
    if(fabsf(r_start - r_end) > 0.005f) {
        return 0;  // Radius error - abort arc
    }
    
    // 2. Calculate angles and arc length
    float start_angle = atan2f(start.y - center.y, start.x - center.x);
    float end_angle = atan2f(end.y - center.y, end.x - center.x);
    float total_angle = clockwise ? (start_angle - end_angle) : (end_angle - start_angle);
    if(total_angle < 0) total_angle += 2.0f * M_PI;  // Normalize to [0, 2π]
    
    float arc_length = r_start * total_angle;  // mm
    
    // 3. Calculate segment count
    uint32_t segment_count = (uint32_t)ceilf(arc_length / settings->mm_per_arc_segment);
    if(segment_count > max_segments) segment_count = max_segments;
    if(segment_count == 0) segment_count = 1;  // At least one segment
    
    // 4. Generate intermediate points using sin/cos (FPU accelerated)
    float angle_per_segment = total_angle / segment_count;
    CoordinatePoint current = start;
    
    for(uint32_t i = 1; i <= segment_count; i++) {
        // Calculate angle for this segment
        float theta = start_angle + (clockwise ? -1.0f : 1.0f) * angle_per_segment * i;
        
        // Calculate next point on arc (using FPU sin/cos)
        CoordinatePoint next;
        next.x = center.x + r_start * cosf(theta);
        next.y = center.y + r_start * sinf(theta);
        next.z = start.z + (end.z - start.z) * ((float)i / segment_count);  // Linear Z
        next.a = start.a + (end.a - start.a) * ((float)i / segment_count);  // Linear A
        
        // Call KINEMATICS_LinearMove() for this arc segment (reuse existing function!)
        KINEMATICS_LinearMove(current, next, feedrate, &segment_buffer[i-1]);
        
        current = next;  // Move to next segment
    }
    
    return segment_count;  // Number of segments generated
}
```

**Key Implementation Points:**
- **Use hardware FPU** for sin/cos/atan2/sqrt (fast on PIC32MZ with `-mhard-float`)
- **Reuse KINEMATICS_LinearMove()** - each arc segment is a tiny linear move
- **Generate ALL segments atomically** before returning (no streaming during arc)
- **Pre-allocate buffer** in caller (APP_DATA or stack)
- **Error checking**: Verify radius matches at start and end

## 🔧 BUILD SYSTEM STATUS (November 6, 2025)

### Current State
- ✅ **Build works** - `make all` defaults to Release, project compiles successfully
- ✅ **Clean works** - `make clean` removes all build artifacts
- ✅ **Directory structure finalized** - Separate Debug/Release folders for bins, objs, libs, other

### Directory Structure (Final)
```
bins/Debug/     - Debug executables (.hex files)
bins/Release/   - Release executables (.hex files)
objs/Debug/     - Debug object files (.o)
objs/Release/   - Release object files (.o)
libs/Debug/     - Debug libraries (.a)
libs/Release/   - Release libraries (.a)
other/Debug/    - Debug map files and XML
other/Release/  - Release map files and XML
```

### Build Commands
```bash
make all                    # Build Release (default)
make BUILD_CONFIG=Debug     # Build Debug
make clean                  # Clean current BUILD_CONFIG artifacts
```

**No further Makefile changes planned** - current structure works well and follows standard practices.
