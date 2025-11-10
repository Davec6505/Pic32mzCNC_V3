# GitHub Copilot Instructions for Pic32mzCNC_V3

## Project Overview
This is a CNC motion control system for PIC32MZ microcontrollers using hardware timers and Bresenham interpolation for precise multi-axis stepper motor control.

##    Always run make from root directory VI
To ensure proper build configuration and output paths, always execute `make` commands from the root directory of the Pic32mzCNC_V3 project. This guarantees that all relative paths and build settings are correctly applied. makefile incs target is dynamic, it knows the paths no need to add absolute file references, all paths are relative to the root directory.

## 🚀 Current Implementation Status (November 10, 2025)
### ✅ COMPLETED FEATURES
- **COMPLETE HOMING & LIMIT SWITCH SYSTEM**: Professional GRBL v1.1 compatible homing with $H command, array-based limit configuration, hardware debouncing, and proper inversion logic (November 10, 2025)
- **COMPLETE SPINDLE PWM CONTROL**: OC8/TMR6 PWM system with 3.338kHz frequency, RPM conversion, M3/M5/S command integration (November 10, 2025)
- **ARRAY-BASED AXIS CONTROL**: Eliminated all switch statements for axis operations using coordinate array utilities (November 10, 2025)
- **PRODUCTION-READY G-CODE PARSER**: Professional event-driven system with clean architecture (⚠️ CRITICAL: Do not modify gcode_parser.c - working perfectly!)
- **ROBUST SOFT RESET RECOVERY**: UGS compatible soft reset with proper OC1/TMR4 restart logic (November 10, 2025)
- **OPTIMAL TIMER CONFIGURATION**: TMR4 1:64 prescaler (781.25kHz) with 2.5µs stepper pulses (November 10, 2025)
- **HARDWARE VALIDATION GUARDS**: OC1/TMR4 startup checks prevent motion restart failures (November 10, 2025)
- Event queue implementation respecting APP_DATA abstraction layer
- Comprehensive G-code support: G0/G1, G2/G3, G4, M3/M5, M7/M9, G90/G91, F, S, T, G10 L20
- Core architecture implemented with period-based timer (TMR4/PR4)
- Single instance pattern in appData for clean separation
- Proper tokenization with combined modal splitting
  - Examples: "G21G90" → ["G21", "G90"], "G90G0Z5" → ["G90", "G0Z5"]
- Multi-command line support - "G90G1X10Y10F1000" → ["G90", "G1X10Y10F1000"]
- Modal parameter support - Standalone F, S, T commands (GRBL v1.1 compliant)
- 16-command circular buffer with flow control and overflow protection ("ok" withholding)
- Harmony state machine pattern - proper APP_Tasks architecture
- Non-blocking event processing - one event per iteration
- Kinematics module complete with physics calculations and velocity profiling
- Stepper module complete with hardware abstraction and emergency stop
- Persistent GRBL settings with NVM flash storage (29 parameters including arc)
- Delayed flash initialization - read after peripherals ready (APP_LOAD_SETTINGS state)
- Unified data structures - no circular dependencies, clean module separation
- Hardware FPU enabled - Single-precision floating point for motion planning
- Trapezoidal velocity profiling IMPLEMENTED - KINEMATICS_LinearMove with full physics
- Emergency stop system complete - APP_ALARM state with hard/soft limit checking
- Position tracking and modal state - Work coordinates with G90/G91 support
- Safety system complete - STEPPER_DisableAll(), MOTION_UTILS_CheckHardLimits()
- 256 microstepping validated - ISR budget analysis shows 42% headroom at 512kHz
- Single ISR architecture designed - GRBL pattern, no multi-ISR complexity
- PRIORITY PHASE SYSTEM IMPLEMENTED - Hybrid ISR/main loop architecture (best of both worlds!)
- INCREMENTAL ARC INTERPOLATION COMPLETE - Non-blocking G2/G3 with FPU acceleration
- Motion phase system operational - VELOCITY/BRESENHAM/SCHEDULE/COMPLETE phases
- Project compiles successfully with XC32 compiler
- UART3 fully functional - TX and RX working, status queries respond
- Non-blocking UART utilities module - central helpers (UART_SendOK, UART_Printf)
- Professional compile-time debug system - Zero runtime overhead, multiple subsystems (November 7, 2025)
- Clean build system - make/make all defaults to Release, make build for incremental (November 7, 2025)
- ATOMIC INLINE GPIO FUNCTIONS - Zero-overhead ISR with `__attribute__((always_inline))` (November 8, 2025)
- FUNCTION POINTER ARCHITECTURE - Array-based axis control with GPIO_Control structs (November 8, 2025)
- ISR STACK OPTIMIZATION - Eliminated 32 bytes/call local arrays with static pointers (November 8, 2025)
- PRE-CALCULATED DOMINANT AXIS - Stored in MotionSegment, zero ISR overhead (November 8, 2025)

### 🔧 SOFT RESET RECOVERY SYSTEM (November 10, 2025) ⭐ NEW
Module: `srcs/motion/stepper.c` → `STEPPER_LoadSegment()`

**Problem Solved**: UGS soft reset (Ctrl+X) would stop motion, but subsequent G-code commands wouldn't restart motion system.

**Root Cause**: After soft reset, OC1 (Output Compare) module wasn't being re-enabled when motion resumed.

**Solution**: Hardware validation guards in segment loading:
```c
// Re-enable OC1 if disabled
if(!(OC1CON & _OC1CON_ON_MASK)) {
    OCMP1_Enable();
}

// Re-start TMR4 if stopped  
if(!(T4CON & _T4CON_ON_MASK)) {
    TMR4_Start();
}
```

**Timer Configuration Fixed**:
- TMR4 Prescaler: 1:64 (781.25kHz, 1.28µs per tick)
- Pulse Width: 2.5µs (2 ticks) - safe for stepper drivers
- Period Clamping: Minimum 7 ticks to accommodate pulse timing
- Uses PLIB functions: `TMR4_FrequencyGet()`, `TMR4_PeriodSet()`

**Benefits**:
- Motion always restarts after UGS soft reset
- Optimal stepper driver compatibility (2.5µs pulse width)
- Hardware state validation prevents startup failures
- Debug output shows timer frequency and timing calculations

### 🔧 ATOMIC INLINE GPIO ARCHITECTURE (November 8, 2025)
Module: `incs/utils/utils.h`, `srcs/utils/utils.c`, `srcs/motion/stepper.c`

Purpose: Maximum ISR performance using PIC32 atomic SET/CLR/INV registers with always-inline functions.

Key Architecture:
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

Always-Inline Helpers (Zero Overhead!):
```c
static inline void __attribute__((always_inline)) AXIS_StepSet(E_AXIS axis) {
    *g_axis_config[axis].step_atomic.set_reg = g_axis_config[axis].step_atomic.mask;
}
// Compiles to single instruction: sw $t1, 0($t0)
```

ISR Usage:
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

Performance Benefits:
- Single instruction GPIO writes (LATxSET atomic hardware)
- Zero function call overhead (always_inline attribute)
- No stack usage for GPIO operations
- Compiler-optimized to raw register access
- Maintains clean, readable code structure

Available Inline Functions:
- `AXIS_StepSet(axis)` / `AXIS_StepClear(axis)`
- `AXIS_DirSet(axis)` / `AXIS_DirClear(axis)`
- `AXIS_EnableSet(axis)` / `AXIS_EnableClear(axis)`
- `AXIS_IncrementSteps(axis)` / `AXIS_DecrementSteps(axis)`

### 🔧 ARRAY-BASED AXIS CONTROL (November 10, 2025)
Module: `incs/utils/utils.h`, `srcs/motion/homing.c`, `srcs/motion/motion_utils.c`

Purpose: Eliminate all axis switch statements using array-based iteration for better performance and maintainability.

Architecture Transformation:
```c
// OLD: Switch statement approach (5+ instances across codebase)
switch (g_homing.current_axis) {
    case AXIS_X: target.x = current.x + distance; break;
    case AXIS_Y: target.y = current.y + distance; break;
    case AXIS_Z: target.z = current.z + distance; break;
    case AXIS_A: target.a = current.a + distance; break;
}

// NEW: Array-based approach (single line, zero branches)
ADD_COORDINATE_AXIS(&target, g_homing.current_axis, distance);
```

Coordinate Array Utilities (utils.h):
```c
// Treat CoordinatePoint as float[4] array using guaranteed memory layout
// CoordinatePoint { float x, y, z, a; } -> [0]=x, [1]=y, [2]=z, [3]=a

static inline void SET_COORDINATE_AXIS(CoordinatePoint* coord, E_AXIS axis, float value) {
    ((float*)coord)[axis] = value;
}

static inline float GET_COORDINATE_AXIS(const CoordinatePoint* coord, E_AXIS axis) {
    return ((float*)coord)[axis];
}

static inline void ADD_COORDINATE_AXIS(CoordinatePoint* coord, E_AXIS axis, float delta) {
    ((float*)coord)[axis] += delta;
}
```

Refactored Modules:
- **homing.c**: 4 switch statements → `ADD_COORDINATE_AXIS()` calls
  - Search/backoff/locate/pulloff phases now use single-line array operations
- **motion_utils.c**: Limit checking switch → `g_limit_config[axis].limit.GetMin/Max()`
- **All axis operations**: Now use consistent array-based pattern

Performance Benefits:
- **Branch Elimination**: 20+ conditional branches removed from hot paths
- **Code Reduction**: ~40 lines of switch logic → 5 one-liner calls
- **CPU Efficiency**: Direct array indexing (1 instruction) vs branch tables
- **Cache Friendly**: Sequential memory access pattern

Architectural Benefits:
- **DRY Principle**: Single implementation for all axis operations
- **Type Safety**: Leverages C struct memory layout guarantees
- **Extensibility**: Adding 5th axis requires zero code changes
- **Consistency**: Aligns with existing `g_axis_config[]`/`g_limit_config[]` pattern

Memory Safety:
- Uses guaranteed sequential layout of CoordinatePoint struct members
- Bounds checking via E_AXIS enum (0-3) prevents buffer overruns
- Inline functions provide zero-overhead abstraction

Usage Examples:
```c
// Set specific axis coordinate
SET_COORDINATE_AXIS(&target, AXIS_X, 10.0f);

// Add movement to current axis
ADD_COORDINATE_AXIS(&target, g_homing.current_axis, search_distance);

// Read axis value
float current_pos = GET_COORDINATE_AXIS(&position, axis);
```

### 🔧 COMPLETE HOMING & LIMIT SWITCH SYSTEM (November 10, 2025) ⭐ NEW
Module: `srcs/motion/homing.c`, `srcs/motion/motion_utils.c`, `srcs/utils/utils.c`

**Purpose**: Professional GRBL v1.1 compatible homing system with array-based limit switch configuration and proper inversion logic.

**Key Features**:
- **$H Command Integration**: G-code parser handles `$H` system command with proper event generation
- **Array-Based Limit Configuration**: Function pointers for Min/Max limit switches per axis
- **4-Phase Homing Cycle**: GRBL-compatible seek → locate → pulloff → complete sequence  
- **Hardware Debouncing**: Core timer-based debouncing for reliable limit switch detection
- **Settings Integration**: Uses GRBL settings for direction, speeds, debounce, pulloff distance
- **Inversion Logic**: Proper handling of active-high/low limit switches via settings mask

**Hardware Pin Assignments** (from MCC configuration):
```c
X_Min: RA4    X_Max: RA7     // X-axis limit switches
Y_Min: RD0    Y_Max: RE0     // Y-axis limit switches  
Z_Min: RD13   Z_Max: RE1     // Z-axis limit switches
A_Min: RA6    A_Max: RB1     // A-axis limit switches
```

**GRBL Settings Integration**:
```c
$22 = 1         // Enable homing cycle
$23 = 0         // Homing direction mask (0=negative direction)
$24 = 500       // Homing feed rate (slow precision approach) mm/min
$25 = 2000      // Homing seek rate (fast initial search) mm/min
$26 = 250       // Homing debounce delay (microseconds)
$27 = 2.0       // Homing pull-off distance (mm)
$5 = 0          // Limit pins invert mask (0=active-low switches)
```

**Implementation Architecture**:
```c
// Array-based limit configuration (utils.c)
typedef struct {
    GPIO_LimitControl limit;         // GetMin/GetMax function pointers
    uint8_t* homing_enable;          // Pointer to settings->homing_enable
    uint8_t* homing_dir_mask;        // Pointer to settings->homing_dir_mask
    float* homing_feed_rate;         // Pointer to settings->homing_feed_rate
    float* homing_seek_rate;         // Pointer to settings->homing_seek_rate
    uint32_t* homing_debounce;       // Pointer to settings->homing_debounce
    float* homing_pull_off;          // Pointer to settings->homing_pull_off
} LimitConfig;

extern LimitConfig g_limit_config[NUM_AXIS];

// Zero-overhead inline limit checking
static inline bool LIMIT_CheckAxis(E_AXIS axis, uint8_t invert_mask) {
    bool inverted = (invert_mask >> axis) & 0x01;
    return (LIMIT_GetMin(axis) ^ inverted) || (LIMIT_GetMax(axis) ^ inverted);
}
```

**Homing Cycle Phases**:
1. **SEEK**: Fast movement toward limit switch at homing_seek_rate until triggered
2. **LOCATE**: Back off and slow approach at homing_feed_rate for precision
3. **PULLOFF**: Move away from limit by homing_pull_off distance  
4. **COMPLETE**: Set machine position to zero, move to next axis

**Testing Commands**:
```gcode
$H              // Home all axes (XYZA sequence)
$22=1           // Enable homing if disabled
$?              // Check status during homing cycle
$X              // Clear alarm if homing fails
$$              // View all homing settings
```

**Benefits**:
- **GRBL Compatibility**: Standard protocol support for existing CAM/sender software
- **Array Performance**: Eliminated switch statements for better CPU efficiency
- **Hardware Abstraction**: Clean separation between limit logic and GPIO implementation
- **Configurable**: All timing, direction, and behavior controlled by persistent settings
- **Reliable**: Hardware debouncing and proper alarm handling for production use

### 🔧 COMPILE-TIME DEBUG SYSTEM (November 7, 2025)
Module: `incs/common.h`, `srcs/Makefile`, `docs/DEBUG_SYSTEM_TUTORIAL.md`

Purpose: Professional debug infrastructure with ZERO runtime overhead. Debug code is completely removed by compiler in release builds via preprocessor macros.

Key Benefits:
- Zero runtime overhead - Debug code eliminated in release builds
- Multiple subsystems - Enable/disable by subsystem (motion, gcode, stepper, etc.)
- Clean syntax - Looks like regular printf, works everywhere
- No runtime checks - Pure compile-time conditional compilation
- ISR-safe - Can use DEBUG_EXEC_XXX for LED toggles in interrupts

Available Debug Flags:
- `DEBUG_MOTION` - Motion planning and segment execution
- `DEBUG_GCODE` - G-code parsing and event processing
- `DEBUG_STEPPER` - Low-level stepper ISR and pulse generation
- `DEBUG_SEGMENT` - Segment loading and queue management
- `DEBUG_UART` - UART communication
- `DEBUG_APP` - Application state machine

Build Usage:
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

Code Usage:
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

Documentation: See `docs/DEBUG_SYSTEM_TUTORIAL.md` for complete guide with examples, best practices, and troubleshooting.

### ⚠️ CRITICAL DEBUG WORKFLOW (November 7-10, 2025)
ALWAYS use the compile-time debug system instead of manual UART writes!

Wrong - Manual Debug (DO NOT DO THIS):
```c
// BAD: Manual UART writes that clutter code
char debug_buf[64];
snprintf(debug_buf, sizeof(debug_buf), "[DEBUG] Value: %d\r\n", value);
UART3_Write((uint8_t*)debug_buf, strlen(debug_buf));
```

Correct - Use Debug Macros:
```c
// GOOD: Clean debug macros that compile to nothing in release
DEBUG_PRINT_GCODE("[GCODE] Value: %d\r\n", value);
```

Debugging Protocol Issues (e.g., UGS connection):
1. Add debug macros to the relevant code section:
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
2. Build with debug flag:
   ```bash
   make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_GCODE"
   ```
3. Flash and test - debug output appears in terminal
4. Release build - debug code removed automatically:
   ```bash
   make clean && make BUILD_CONFIG=Release
   # or simply:
   make clean && make
   ```

Debugging Motion Restart Issues (e.g., UGS soft reset):
1. Add stepper debug to relevant areas:
   ```c
   DEBUG_PRINT_STEPPER("[STEPPER] OC1 state: 0x%08X, TMR4 state: 0x%08X\r\n", 
                       (unsigned)OC1CON, (unsigned)T4CON);
   DEBUG_PRINT_STEPPER("[STEPPER] Timer freq: %lu Hz\r\n", TMR4_FrequencyGet());
   ```
2. Enable stepper debug and test soft reset sequence:
   ```bash
   make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_STEPPER DEBUG_MOTION"
   ```
3. Observe hardware state during restart - guards should trigger
4. Remove debug for production build

Why This Matters:
- Manual debug code gets forgotten and left in production
- Debug macros are self-documenting (flag name shows what's being debugged)
- Zero performance impact in release builds
- Easy to enable/disable without code changes
- Essential for diagnosing hardware state issues like timer/OC module restart failures

### 🔧 NON-BLOCKING UART UTILITIES (November 6-9, 2025)
Module: `srcs/utils/uart_utils.c`, `incs/utils/uart_utils.h`

Purpose: Centralized non-blocking UART communication for debug output and GRBL protocol responses, preventing real-time motion interference.

Implementation:
- Uses Harmony PLIB UART3 ring buffers
- Fire-and-forget helpers: `UART_Write()`, `UART_Printf()`, `UART_SendOK()`

Key Functions:
```c
void UART_Initialize(void);           // Setup
bool UART_Printf(const char* fmt, ...); // Non-blocking formatted output
bool UART_Write(const uint8_t* msg, size_t len);
bool UART_SendOK(void);               // Send "ok\r\n"
bool UART_IsReady(void);
```

Initialization Pattern:
```c
// In APP_Initialize() - called once at startup
UART_Initialize();
```

Benefits:
- No blocking - Real-time motion never waits for UART
- ISR-safe - Can be called from interrupts (drops if buffers full)
- Centralized - Single module for all UART communication
- Correct TX buffer sizing prevents disconnects on $$

### 🔧 VISUAL MOTION DEBUG (November 6, 2025)
Added: `LED2_Toggle()` in `STEPPER_ScheduleStep()` (srcs/motion/stepper.c line ~125)

Purpose: Visual confirmation that motion scheduling is executing

Behavior:
- Rapid blink (many Hz) → `STEPPER_ScheduleStep()` IS being called → Motion system working
- Slow blink (~1Hz heartbeat) → Function NOT being called → Phase system or segment loading issue

Usage:
```gcode
G1 X1 F100      ; Move 1mm in X axis
```

Observe LED2:
- If rapid blink: Motion hardware OK, check if motors actually moving
- If slow blink: Motion segments not loading or phase system stuck

### 🔧 ACTIVE DEBUGGING SESSION (November 5-6, 2025)
Problem: G-code commands are accepted (ok response) but position never updates. Motion does not execute.

Root Cause Found: Commands were being queued but not parsed into events that reached the motion system.

Debug Progress Chain:
1. UART3 communication working - Banner prints, status queries respond correctly
2. G-code parser receiving commands - "ok" responses confirm reception
3. Tokenization working - Extract/Tokenize traces present
4. Commands being queued - "Queued: ..." traces confirm queue population
5. GCODE_GetNextEvent() being called - Function executes in APP_IDLE event loop
6. Non-blocking UART implemented - uart_utils module prevents motion blocking
7. LED2 visual debug added - Shows if STEPPER_ScheduleStep() executes
8. CRITICAL ISSUE IDENTIFIED: parse_command_to_event() returned false for some commands
9. No motion segments generated in that scenario
10. Position stayed 0.000 - Motion system never executed

Next Steps for Testing:
- Flash firmware with LED2_Toggle() - Visual confirmation of motion execution
- Test simple motion: `G1 X1 F100`
- Observe LED2 behavior:
  - Rapid blink = `STEPPER_ScheduleStep()` executing → Check motor drivers
  - Slow blink = Function not called → Phase system or event parsing issue
- Enable selective debug - Only G0/G1 events, not control characters

### 🔧 ACTIVE DEBUGGING SESSION - UGS CONNECTION (November 7-9, 2025)
Problem: UGS connects, sends commands (`?`, `$I`, `$$`), but may disconnect if TX buffer is too small. Putty works correctly, confirming firmware responds.

CRITICAL FIX:
- Root Cause: MCC regeneration can revert UART3 TX buffer size from 1024 to 256 bytes
- Symptom: `$$` command response (~400-500 bytes) overflows 256-byte TX buffer
- Result: UGS times out waiting for complete settings response, disconnects

⚠️ After ANY MCC regeneration, ALWAYS verify UART3 buffer sizes!

File: `srcs/config/default/peripheral/uart/plib_uart3.c`

Required Buffer Sizes:
```c
#define UART3_READ_BUFFER_SIZE      (512U)   // Must be 512
#define UART3_WRITE_BUFFER_SIZE     (1024U)  // Must be 1024 (NOT 256!)
```

Symptoms of Wrong Buffer Size:
- `?` (status query) works (~100 bytes)
- `$I` (build info) works (~80 bytes)
- `$$` (settings) fails - response truncated
- Sender disconnects immediately after `$$` command

GRBL Protocol Requirements:
- `$I` response format requires spaces after colons:
  - `[VER: 1.1h.20251102]\r\n`
  - `[OPT: VHM,35,1024,4]\r\n`
- Banner on soft reset must be GRBL-compatible:
  - `Grbl 1.1h ['$' for help]\r\n`

Fixes Applied:
- Added spaces after colons in `SETTINGS_PrintBuildInfo()`
- Ensured soft reset prints GRBL-like banner text for sender compatibility

## GRBL Real-Time Controls and Protocol Integration

### Real-Time Characters
- `?` Status report only. No "ok" response. Includes:
  - Machine position (MPos), Work position (WPos), Feed/Spindle (FS)
  - Optional flags:
    - `|Cm:1` when in check mode (`$C`)
    - `|FH:1` when feed hold is active (`!`)
- `!` Feed hold:
  - Immediately halts motion pipeline, disables steppers (`STEPPER_DisableAll()`)
  - Sets internal `feedHoldActive = true`
- `~` Cycle start / Resume:
  - Clears `feedHoldActive`, prepares motion pipeline to resume
  - No textual response (real-time)
- `Ctrl+X (0x18)` Soft reset:
  - Centralized reset via `UART_SoftReset(APP_DATA*, GCODE_CommandQueue*)`
  - Clears motion queue, G-code queue, modal states
  - Flushes UART RX buffer
  - Prints GRBL-compatible banner: `Grbl 1.1h ['$' for help]\r\n`
  - Literal string "0x18" is also accepted for manual testing

### System Commands ($)
Handled in GCODE_STATE_QUERY_CHARS:
- `$` help: `[HLP:$$ $# $G $I $N $C $X $F $RST= $SLP]` + ok
- `$$` prints all settings (ensure TX buffer is 1024 bytes), no ok appended by handler
- `$I` build info with spaces after colons, no ok appended by handler
- `$#` work coordinate offsets (G54..G59), probe, etc.
- `$G` modal state report (G0, G54, plane, units, distance mode, etc.)
- `$C` toggles check mode (no motion, `|Cm:1` in status)
- `$X` clears alarm
- `$Nn=` sets startup lines (N0/N1 persisted strings)
- `$RST=*|$|#` reset defaults, settings, or WCS offsets
- `$<number>` read setting, `$<number>=<value>` write setting
- `$SLP` not supported → `error:2`

### G10 L20 Work Offset Setting
- Supports `G10 L20` with `P0` (current WCS) or `P1` (G54)
- Parameters X/Y/Z set desired WCS work position at current machine position:
  - WCS offset computed as `offset = MPos - desired_WPos`
- A-axis parameter accepted but not applied (no A in current WCS struct)

### Combined Modal Token Splitting (Parser)
- Tokens like `G21G90` or `G90G0Z5` are split and queued as separate commands in order
- Parameters belong to the final modal in the combined token
  - Example: `G90G1X10` → ["G90", "G1X10"]
  - Example: `G21G90 G0Z5` → Tokens: "G21", "G90", "G0Z5"

## Core Architecture Principles

### Priority-Based Phase System (CRITICAL)
The "Best of Both Worlds" Hybrid Architecture

Problem Solved:
- G-code processing could block motion timing (arc generation takes time)
- UART should be polled at a reasonable rate, not every ISR tick
- Need ISR precision but main loop flexibility

Solution: Priority Phase System
- ISR sets flag when dominant axis fires → wakes main loop
- Main loop processes phases in priority order (0 = highest)
- G-code only runs when IDLE → prevents blocking
- Rate-limited UART → polled periodically in IDLE

Phase Priorities:
```c
typedef enum {
    MOTION_PHASE_IDLE = 255,      // Lowest - safe for G-code processing
    MOTION_PHASE_VELOCITY = 0,    // Highest - velocity conditioning
    MOTION_PHASE_BRESENHAM = 1,   // Bresenham error accumulation
    MOTION_PHASE_SCHEDULE = 2,    // OCx register scheduling
    MOTION_PHASE_COMPLETE = 3     // Segment completion
} MotionPhase;
```

ISR Behavior (stepper.c):
```c
void OCP5_ISR(uintptr_t context) {
    // X Axis - count steps
    if (direction_bits & (1 << AXIS_X)) {
        stepper_pos.x_steps++;
    } else {
        stepper_pos.x_steps--;
    }
    
    // Signal main loop if X is dominant axis
    if (app_data_ref != NULL && app_data_ref->dominantAxis == AXIS_X) {
        app_data_ref->motionPhase = MOTION_PHASE_VELOCITY;  // Wake main loop
    }
}
```

Main Loop Processing (app.c):
```c
case APP_IDLE:
    switch(appData.motionPhase) {
        case MOTION_PHASE_VELOCITY:
            // Update currentStepInterval (accel/cruise/decel)
            appData.motionPhase = MOTION_PHASE_BRESENHAM;
            // Fall through
            
        case MOTION_PHASE_BRESENHAM:
            // Accumulate error terms, determine subordinate steps
            appData.motionPhase = MOTION_PHASE_SCHEDULE;
            // Fall through
            
        case MOTION_PHASE_SCHEDULE:
            // Write OCxR/OCxRS with absolute values
            appData.motionPhase = MOTION_PHASE_COMPLETE;
            // Fall through
            
        case MOTION_PHASE_COMPLETE:
            // Check segment done, load next from queue
            appData.motionPhase = MOTION_PHASE_IDLE;
            break;
            
        case MOTION_PHASE_IDLE:
            // Safe for G-code processing
            break;
    }

    // Rate-limited UART/G-code (only when IDLE)
    if(appData.motionPhase == MOTION_PHASE_IDLE) {
        GCODE_Tasks(&appData.gcodeCommandQueue);
    }
```

Benefits:
- ISR precision - dominant axis timing rock-solid
- Main loop flexibility - complex calculations without ISR bloat
- Guaranteed execution order - phases process in sequence
- Non-blocking G-code - motion always gets priority
- CPU efficiency - UART handled in IDLE, not per-step

### Timer Architecture (Period-Based, TMR4/PR4)
- TMR4 rolls over at PR4 value - not free-running
- OC1 uses relative compare values against the rolling timer
- `PR4` sets the period (step interval + pulse width + margin)
- `OC1R` sets when pulse starts (step_interval)
- `OC1RS` sets when pulse ends (step_interval + pulse_width)
- Example: For 1ms steps with 2.5µs pulse: `OC1R = 781`, `OC1RS = 783`, `PR4 = 789`
- Hardware Configuration (November 10, 2025 - VERIFIED):
  - PBCLK3 = 50MHz (peripheral bus clock)
  - Prescaler = 1:64 (TCKPS = 6, verified in plib_tmr4.c)
  - Timer Frequency = 781.25kHz (50MHz / 64)
  - Timer Resolution = 1.28µs per tick
  - Pulse Width = 2.5µs (2 ticks) - safe for stepper drivers
- No timer rollover issues - TMR4 automatically resets to 0 at PR4, OCx values remain valid
- Step timing controlled entirely by OC1 ISR scheduling next pulse
- Hardware validation guards ensure OC1/TMR4 restart after soft reset

### Dynamic Dominant Axis Tracking
- Dominant axis (highest step count) drives the step timing
- Dominant axis determines step_interval for OC1/PR4
- Subordinate axes step on-demand when Bresenham requires a step
- Dominant axis can swap mid-motion by recalculating Bresenham state

### Bresenham Integration
- Bresenham algorithm runs in OC1 ISR for precise timing
- ISR: Generate step pulse, run Bresenham, schedule next step
- Error term updates happen in ISR each step
- Subordinate axis pulse generation based on error accumulation

### Single Instance Pattern in appData
- All major data structures centralized in APP_DATA struct
- No static module data - clean separation of concerns
- Pass by reference through function calls for explicit ownership
- Work coordinates protected by private static in kinematics module

### Professional G-Code Event System
- Event-driven architecture: Clean `GCODE_GetNextEvent()` interface
- Comprehensive G-code support: G0/G1, G2/G3, G4, M3/M5, M7/M9, G90/G91, F, S, T, G10 L20
- Proper tokenization with combined-modal splitting:
  - Example: `G90G1X10Y10F1000` → Tokens: "G90", "G1X10Y10F1000"
  - Example: `G21G90 G0Z5` → Tokens: "G21", "G90", "G0Z5"
- Modal parameter support: Standalone F, S, T commands (LinuxCNC/GRBL compatible)
- Abstraction layer respect: No APP_DATA exposure, maintains clean boundaries
- Zero memory allocation: Deterministic processing for real-time systems
- Utils module: Professional string parsing with robust tokenization
- Flow control: 16-command circular buffer with "ok" withholding
- Real-time characters: Immediate handling for '?!~^X' (and literal "0x18")
- GRBL compliance: v1.1 protocol support with proper status reporting
- Non-blocking design: One event processed per APP_Tasks() iteration

## Code Style Guidelines

### ISR Implementation
```c
void __ISR(_OC1_VECTOR, IPL5SOFT) OC1Handler(void) {
    // Clear interrupt flag FIRST
    IFS0CLR = _IFS0_OC1IF_MASK;
    
    // Schedule next pulse using period-based timing
    uint32_t step_interval = current_segment->step_interval;
    uint32_t pulse_width = 2;  // 2 ticks = 2.5µs at 781.25kHz
    
    OC1R = step_interval;                      // Pulse start
    OC1RS = step_interval + pulse_width;       // Pulse end
    
    // Ensure minimum period for pulse completion
    uint32_t minimum_period = step_interval + pulse_width + 2;
    if (minimum_period < 7) minimum_period = 7;  // 7 tick minimum
    TMR4_PeriodSet(minimum_period);            // Timer period
    
    // Update step counter
    steps_completed++;
    
    // Run Bresenham for subordinate axes
    // ...
}
```

### Compare Register Updates
```c
// CORRECT - Period-based timing (November 10, 2025)
uint32_t step_interval = 781;         // ~1ms at 781.25kHz
uint32_t pulse_width = 2;             // 2.5µs pulse width
OC1R = step_interval;                 // Pulse starts at interval
OC1RS = step_interval + pulse_width;  // Pulse ends
uint32_t period = step_interval + pulse_width + 2;
if (period < 7) period = 7;          // Minimum period guard
TMR4_PeriodSet(period);              // Timer rolls over

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
```c
// To stop pulse generation on any axis (for safety/sanity):
// Set OCxR = OCxRS (equal values prevent compare match pulse)
OC2R = OC2RS;  // Y axis stops generating pulses
OC3R = OC3RS;  // Z axis stops generating pulses

// This is critical during:
// - Arc interpolation when axis doesn't need steps
// - Bresenham cycles where subordinate axis is inactive
// - Motion completion or emergency stop scenarios
```

### G-Code Line Buffering Pattern (UART3)
```c
case GCODE_STATE_IDLE:
    nBytesAvailable = UART3_ReadCountGet();

    if(nBytesAvailable > 0){
        // Accumulate bytes until complete line
        uint32_t space_available = sizeof(rxBuffer) - nBytesRead - 1;
        uint32_t bytes_to_read = (nBytesAvailable < space_available) ? nBytesAvailable : space_available;
        
        if (bytes_to_read > 0) {
            uint32_t new_bytes = UART3_Read((uint8_t*)&rxBuffer[nBytesRead], bytes_to_read);
            nBytesRead += new_bytes;
            rxBuffer[nBytesRead] = '\0';
        }
    }

    if (nBytesRead == 0) break;

    // Literal "0x18" typed by user → soft reset
    if (nBytesRead >= 4 && rxBuffer[0]=='0' && rxBuffer[1]=='x' && rxBuffer[2]=='1' && rxBuffer[3]=='8') {
        GCODE_HandleSoftReset(cmdQueue);
        break;
    }

    // Real-time control characters bypass line buffering
    if (is_control_char(rxBuffer[0])) {
        gcodeData.state = GCODE_STATE_CONTROL_CHAR;
        break;
    }

    // Require a line terminator before processing
    bool has_terminator = false;
    uint32_t terminator_pos = 0;
    for (uint32_t i=0; i<nBytesRead; i++){
        if (rxBuffer[i] == '\n' || rxBuffer[i] == '\r') { has_terminator = true; terminator_pos = i; break; }
    }
    if (!has_terminator) break;

    // NUL-terminate at first terminator
    rxBuffer[terminator_pos] = '\0';

    // Route by first char
    if (rxBuffer[0] == '$') {
        gcodeData.state = GCODE_STATE_QUERY_CHARS;
        break;
    } else if (rxBuffer[0]=='G'||rxBuffer[0]=='g'||
               rxBuffer[0]=='M'||rxBuffer[0]=='m'||
               rxBuffer[0]=='F'||rxBuffer[0]=='f'||
               rxBuffer[0]=='T'||rxBuffer[0]=='t'||
               rxBuffer[0]=='S'||rxBuffer[0]=='s') {
        gcodeData.state = GCODE_STATE_GCODE_COMMAND;
        break;
    }

    // Generic content: extract, queue, and flow-control ok
    if (terminator_pos > 0) {
        cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, terminator_pos, cmdQueue);
        if (!okPending && cmdQueue->motionQueueCount < (cmdQueue->maxMotionSegments - MOTION_BUFFER_THRESHOLD))
            UART_SendOK();
        else
            okPending = true;
    }

    // Compact remaining bytes (consume all trailing CR/LF)
    uint32_t skip_pos = terminator_pos + 1;
    while (skip_pos < nBytesRead && (rxBuffer[skip_pos] == '\r' || rxBuffer[skip_pos] == '\n'))
        skip_pos++;
    uint32_t remaining_bytes = nBytesRead - skip_pos;
    if (remaining_bytes > 0) {
        memmove(rxBuffer, &rxBuffer[skip_pos], remaining_bytes);
        nBytesRead = remaining_bytes;
        rxBuffer[nBytesRead] = '\0';
    } else {
        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
    }

    if (okPending && cmdQueue->motionQueueCount < (cmdQueue->maxMotionSegments - MOTION_BUFFER_THRESHOLD)) {
        UART_SendOK();
        okPending = false;
    }
    break;

case GCODE_STATE_CONTROL_CHAR:
    switch(rxBuffer[0]) {
        case '?':  // Status report
            // Send "<...>" status only, no "ok"
            break;
        case '~':  // Cycle start/resume
            // Clear feed hold, resume motion, no "ok"
            break;
        case '!':  // Feed hold
            // Disable steppers immediately, no "ok"
            break;
        case 0x18: // Soft reset (Ctrl+X)
            GCODE_HandleSoftReset(cmdQueue);  // Banner only
            break;
        default:
            break;
    }
    // Consume one byte and return to idle
    // ...
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
            // Add to motion queue through abstraction layer
            if (appData.motionQueueCount < MAX_MOTION_SEGMENTS) {
                appData.motionQueue[appData.motionQueueHead] = segment;
                appData.motionQueueHead = (appData.motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
                appData.motionQueueCount++;
            }
            break;

        case GCODE_EVENT_ARC_MOVE:
            // Handle arc interpolation using incremental generator
            break;

        case GCODE_EVENT_SET_FEEDRATE:
        case GCODE_EVENT_SET_SPINDLE_SPEED:
        case GCODE_EVENT_SET_ABSOLUTE:
        case GCODE_EVENT_SET_RELATIVE:
        case GCODE_EVENT_SPINDLE_ON:
        case GCODE_EVENT_SPINDLE_OFF:
        case GCODE_EVENT_COOLANT_ON:
        case GCODE_EVENT_COOLANT_OFF:
        case GCODE_EVENT_DWELL:
            // Update modal state or peripherals
            break;

        default:
            break;
    }
}
```

## Important Constraints

### Never Do
- Use absolute timer reads for scheduling (use period-based intervals)
- Set PR4 smaller than OC1RS (pulse won't complete)
- Use blocking delays in main loop (let APP_Tasks run freely)
- Modify OC1R/OC1RS outside of ISR during active motion
- Set period < 7 ticks (insufficient time for 2.5µs pulse completion)
- Assume OC1/TMR4 remain enabled after soft reset

### Always Do
- Use period-based timing with proper guards: `TMR4_PeriodSet(max(7, step_interval + pulse_width + margin))`
- Clear interrupt flags immediately at ISR entry
- Keep ISRs minimal and fast
- Run Bresenham logic in OC1 ISR for precise timing
- Schedule subordinate axes only when required
- Set OCxR = OCxRS to disable pulse generation (prevents spurious pulses)
- Validate OC1/TMR4 hardware state before loading motion segments

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

Note: Velocity profiling is not yet implemented but should be designed with the period-based timer architecture in mind.

## Hardware Details
- Microcontroller: PIC32MZ2048EFH100
- System Clock: 200MHz
- Peripheral Bus Clock (PBCLK3): 50MHz
- Compiler: XC32
- Build System: Make
- Bootloader: MikroE USB HID Bootloader (39KB, starts at 0x9D1F4000)
- Hardware FPU: Single-precision floating point unit enabled
  - Compiler flags: `-mhard-float -msingle-float -mfp64`
  - Optimizations: `-ffast-math -fno-math-errno`
  - Use FPU for planning (KINEMATICS), integer math for ISR
- Timer Configuration (November 10, 2025 - VERIFIED WORKING):
  - TMR4 16-bit timer (period-based, rolls over at PR4)
  - Prescaler: 1:64 (TCKPS = 6, verified in plib_tmr4.c)
  - Timer Frequency: 781.25kHz (50MHz ÷ 64)
  - Timer Resolution: 1.28µs per tick
  - Pulse Width: 2.5µs (2 ticks) - safe for stepper drivers
  - Period Clamping: Minimum 7 ticks for pulse timing accommodation
  - TMR5 16-bit timer for step pulse width (one-shot mode)
- Output Compare Modules: OC1 (X), OC2 (Y), OC3 (Z), OC4 (A)
  - OC1 continuous pulse mode verified with TMR4 time base
  - Hardware validation guards prevent startup failures
- Microstepping Support: Designed for up to 256 microstepping
  - Worst case: 512kHz step rate (256 microsteps × high speed)
  - ISR budget: ~390 CPU cycles @ 512kHz (225 cycles used)
  - Per-step timing: ~6 timer ticks minimum (7.68µs)

### Memory Layout (PIC32MZ2048EFH100 - 2MB Flash)
```
Physical Address    Virtual (KSEG1)     Size        Purpose
----------------    ---------------     ----        -------
0x1D000000          0x9D000000          1.87MB      Application Code
0x1D1F0000          0xBD1F0000          16KB        GRBL Settings Storage (NVM)
0x1D1F4000          0x9D1F4000          48KB        MikroE USB HID Bootloader
0x1FC00000          0xBFC00000          12KB        Boot Flash (Config Words)
```

Critical Memory Rules:
- Settings NVM: `0xBD1F0000` (KSEG1 virtual for both reads and writes)
- Page-aligned: 16KB boundaries (0x4000)
- Row-aligned: 2048-byte boundaries (0x800)
- Safe margin: 64KB (0x10000) before bootloader at 0xBD1F4000
- Never write to: 0x9D1F4000 / 0xBD1F4000 (bootloader region)
- Never write to: 0xBFC00000 (boot flash config)

Address Space (MIPS Architecture):
- Physical (0x1D...): Internal flash controller addressing
- Virtual KSEG1 (0xBD...): Uncached - REQUIRED for NVM operations (reads and writes)
- Virtual KSEG0 (0x9D...): Cached - used for code execution

NVM Operations (Harmony Pattern):
- Use KSEG1 (0xBD...) addresses for all NVM operations
- Harmony NVM drivers handle address conversion internally
- Flash page size: 16KB (must erase entire page before writing)
- Flash row size: 2048 bytes (512 words) - unit of RowWrite operations
- Cache-aligned buffers REQUIRED: Use `CACHE_ALIGN` attribute
- Callback pattern: Register handler, wait on `xferDone` flag
- RowWrite preferred: One operation vs many WordWrite operations

## File Organization
- `srcs/main.c` - Entry point, main loop calls APP_Tasks()
- `srcs/app.c` - Application state machine (single instance pattern)
- `srcs/gcode/gcode_parser.c` - G-code parsing & GRBL protocol
- `srcs/gcode/utils.c` - Professional string tokenization utilities
- `srcs/motion/stepper.c` - Hardware abstraction layer
- `srcs/motion/motion.c` - Master motion controller
- `srcs/motion/kinematics.c` - Physics calculations
- `srcs/settings/settings.c` - Persistent GRBL settings with NVM flash
- `incs/data_structures.h` - Unified data structures (no circular dependencies)
- `incs/common.h` - Shared constants and enums
- `docs/plantuml/` - Architecture diagrams:
  - `01_system_overview.puml` - High-level system architecture
  - `02_segment_clock.puml` - Timer and motion segment flow
  - `03_arc_linear_interpolation.puml` - Arc/linear interpolation system
- `README.md` - Complete project documentation with TODO list

## Unified Data Structures (Completed)
Architecture Pattern:
All major data structures consolidated in `incs/data_structures.h` to eliminate circular dependencies and provide clean module separation.

Structure Hierarchy:
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

Flow Control Infrastructure (Ready to Use):
The nested motion queue info in `GCODE_CommandQueue` enables flow control without circular dependencies:
```c
// app.c syncs motion queue status before GCODE processing
case APP_IDLE:
    // Sync motion queue count for flow control
    appData.gcodeCommandQueue.motionQueueCount = appData.motionQueueCount;
    
    // Now GCODE_Tasks can check motion buffer occupancy
    GCODE_Tasks(&appData.gcodeCommandQueue);
    break;
```

Benefits of Unified Structures:
- No circular dependencies - all structures in one header
- Clean module separation - data vs logic separation
- Single source of truth - structure definitions in one place
- Easy to test - mock entire APP_DATA structure
- Flow control ready - nested motion queue info accessible to G-code parser

## Settings Implementation (Completed)

### Critical Timing Requirement
- NEVER read flash during `SETTINGS_Initialize()` - will hang on boot
- Must delay NVM_Read() until after all peripherals initialized
- Solution: APP_LOAD_SETTINGS state executes after APP_CONFIG

Implementation Pattern:
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

NVM Write Pattern (Harmony):
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

Why RowWrite vs WordWrite:
- Settings typically fit in one row (2048 bytes)
- Must erase full page anyway
- RowWrite fewer ops → more reliable

## Arc Interpolation Implementation (COMPLETED ✅ November 4, 2025)

### Overview
Arc interpolation provides smooth circular motion for G2/G3 commands. The incremental streaming architecture generates one segment per iteration, preventing motion queue starvation and enabling non-blocking operation.

### Implementation Architecture

Incremental Streaming Pattern:
- Non-blocking: ONE segment generated per APP_Tasks() iteration
- Self-regulating: Only generates when motion queue has space
- FPU-accelerated: Hardware sin/cos for smooth arcs (50-100μs per segment)
- Exact end point: Final segment uses target coordinates (no accumulated error)
- Queue never empties: Continuous flow during arc execution

Arc Generation State Machine:
```c
typedef enum {
    ARC_GEN_IDLE = 0,      // No arc in progress
    ARC_GEN_ACTIVE         // Arc generation active
} ArcGenState;
```

Arc Parameters in APP_DATA:
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

Arc Math (GRBL v1.1 Compatible)

G-code Format:
- G2: Clockwise arc in current plane
- G3: Counter-clockwise arc in current plane
- Parameters:
  - `X Y Z`: End point coordinates (absolute or relative based on G90/G91)
  - `I J K`: Center offset from start point (always incremental)

Arc Calculations (implemented in app.c):
1. Center point: `center.x = start.x + I`, `center.y = start.y + J`
2. Radius verification:
   - `r_start = sqrt(I² + J²)` (radius from start to center)
   - `r_end = sqrt((end.x - center.x)² + (end.y - center.y)²)` (radius from end to center)
   - If `|r_start - r_end| > 0.005mm`, trigger ALARM (arc radius error)
3. Angles:
   - `start_angle = atan2f(start.y - center.y, start.x - center.x)`
   - `end_angle = atan2f(end.y - center.y, end.x - center.x)`
   - `total_angle` with wrap-around handling for CW/CCW direction
4. Arc length: `arc_length = radius × total_angle` (in radians)
5. Segment count: `segments = ceil(arc_length / mm_per_arc_segment)` (GRBL setting $12)

Incremental Arc Generator (app.c)

Operation Pattern:
```c
// Runs in APP_IDLE state when arcGenState == ARC_GEN_ACTIVE
if(appData.arcGenState == ARC_GEN_ACTIVE && appData.motionQueueCount < MAX_MOTION_SEGMENTS) {
    
    appData.arcTheta += appData.arcThetaIncrement;  // Increment angle
    
    CoordinatePoint next;
    
    // Determine if last segment
    bool last = /* ... compare theta to thetaEnd based on direction ... */;
    
    if(last) {
        next = appData.arcEndPoint;     // Exact end point
        appData.arcGenState = ARC_GEN_IDLE;
    } else {
        // Calculate intermediate point using sin/cos (FPU accelerated)
        next.x = appData.arcCenter.x + appData.arcRadius * cosf(appData.arcTheta);
        next.y = appData.arcCenter.y + appData.arcRadius * sinf(appData.arcTheta);
        // Linear interpolation for Z and A axes (helical motion)
        float progress = /* ... */;
        next.z = appData.arcCurrent.z + (appData.arcEndPoint.z - appData.arcCurrent.z) * progress;
        next.a = appData.arcCurrent.a + (appData.arcEndPoint.a - appData.arcCurrent.a) * progress;
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

Key Features
- GRBL v1.1 Compatible Arc Math
- Helical Motion Support (Z/A linear interpolation during arcs)
- Plane Selection (G17/G18/G19) via modal state
- Performance: non-blocking, FPU-accelerated, memory-efficient
- GRBL Setting Integration: `$12` - mm_per_arc_segment (persistent in NVM)

Testing Arc Interpolation
```gcode
G17           ; Select XY plane
G90           ; Absolute positioning
G0 X0 Y0      ; Rapid to origin
G1 F500       ; Set feedrate to 500 mm/min
G2 X10 Y0 I5 J0   ; CW arc from (0,0) to (10,0), center at (5,0), radius=5mm
```

Expected Behavior:
- Arc broken into short segments based on `$12`
- Smooth circular motion in XY plane
- Final position: (10.0, 0.0)
- Status query shows continuous motion: `<Run|MPos:...>`

## 🔧 BUILD SYSTEM STATUS (November 6, 2025)

### Current State
- Build works - `make all` defaults to Release, project compiles successfully
- Clean works - `make clean` removes all build artifacts
- Directory structure finalized - Separate Debug/Release folders for bins, objs, libs, other

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

No further Makefile changes planned - current structure works well and follows standard practices.
