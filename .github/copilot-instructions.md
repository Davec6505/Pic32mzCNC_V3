# GitHub Copilot Instructions for Pic32mzCNC_V3

## 🔴 CRITICAL: Git — Use Raw Git Commands Only

**ALWAYS use raw `git` commands in the terminal. NEVER use GitKraken, GitHub Desktop, or any other GUI git tool.**

```powershell
# CORRECT — always run from repo root
git status
git add <files>
git commit -m "message"
git push origin <branch>
git checkout <branch>
git merge <branch> --no-ff -m "message"
git log --oneline -10
```

This applies to ALL git operations: commits, merges, pushes, branch creation, rebases, log inspection — everything.

---

## 🔴 CRITICAL: PCB Hardware Constraints — READ BEFORE EDITING GPIO CODE

### Stepper Enable Pin
- **Single shared enable pin only**: `EnXYZA` (RE6) — `EnXYZA_Set()` / `EnXYZA_Clear()` / `EnXYZA_Toggle()`
- **There are NO per-axis enable GPIO macros** (`EnX_Set`, `EnY_Set`, etc. do NOT exist in `plib_gpio.h`)
- All per-axis enable wrappers in `srcs/utils/utils.c` delegate to `enable_all_set()` / `enable_all_clear()`
- Do NOT introduce `EnX_Set/Clear`, `EnY_Set/Clear`, `EnZ_Set/Clear`, `EnA_Set/Clear` — they will cause compiler errors
- This applies to BOTH `DRIVER_TMC5160` and `DRIVER_DRV8825` axes on this board

### Driver Selection
- Per-axis driver type is set in `incs/common.h` via `AXIS_X_DRIVER` … `AXIS_A_DRIVER`
- Valid values: `DRIVER_TMC5160` or `DRIVER_DRV8825`
- Do NOT use the old `STEPPER_DRIVER_TMC5160` / `STEPPER_DRIVER_LEGACY` defines — they are removed
- Use `HAS_TMC5160_AXIS` / `HAS_DRV8825_AXIS` aggregate flags for conditional compilation

---

## 🔴 CRITICAL: Change Tracking Requirements

**ALL code changes MUST be documented in STATUS.md**

For every code addition, modification, or correction:
- ✅ **DO**: Update `STATUS.md` with: File path, Line number, Function name, Description of change
- ✅ **DO**: Create planning documents (e.g., LITEPLACER_GRBL_IMPLEMENTATION.md)
- ❌ **DO NOT**: Create individual .md files for each code change/session
- ❌ **DO NOT**: Create change log files for corrections or additions

**Example STATUS.md Entry**:
```
- `incs/gcode/gcode_parser.h:35` - Added `GCODE_EVENT_PROBE_TOWARD` enum value
- `srcs/gcode/gcode_parser.c:456` - `parse_command_to_event()` - Added G38.2/G38.3 parsing
```

All sub README's are under docs/readme

## 🔴 CRITICAL: README.md Must Stay Current

**`README.md` is the authoritative public description of the system. It must be updated whenever code changes affect:**
- Feature status table (any feature added, completed, or changed)
- ISR architecture or step sequence
- Hardware pin assignments or timer configuration
- New GRBL settings ($-parameters)
- New safety flags or global state variables
- Motion pipeline or call hierarchy
- Build commands or file structure

For every code change session:
- ✅ **DO**: Update the relevant README.md section(s) to match the new code
- ✅ **DO**: Update STATUS.md AND README.md when both are affected
- ❌ **DO NOT**: Leave README.md describing old/removed behaviour (e.g. a phase system that no longer exists)
- ❌ **DO NOT**: Commit code changes without checking whether README.md needs a corresponding update

## Project Overview

CNC motion control system for PIC32MZ microcontrollers with GRBL v1.1 protocol compliance, 4-axis coordinated motion, and hardware-optimized stepper control.## Project Overview

This is a CNC motion control system for PIC32MZ microcontrollers using hardware timers and Bresenham interpolation for precise multi-axis stepper motor control.

**Status**: Under Active Development (November 21, 2025)  

**Firmware**: `bins/Release/CS23.hex` (264KB)

### 🔧 ARC Z-AXIS FIX (November 21, 2025) ✅ DEPLOYED

**Critical Bug Fixed**: Z-axis motion after arc moves (G2/G3)
- Arc moves now preserve unspecified axis positions (GRBL v1.1 compliant)
- Parser returns NAN for missing values (distinguishes "not specified" from "zero")
- Added isnan() checks in arc coordinate building
- **See**: `docs/readme/ARC_Z_AXIS_FIX_2025-11-21.md` for complete documentation##    Always run make from root directory VI

To ensure proper build configuration and output paths, always execute `make` commands from the root directory of the Pic32mzCNC_V3 project. This guarantees that all relative paths and build settings are correctly applied. makefile incs target is dynamic, it knows the paths no need to add absolute file references, all paths are relative to the root directory.

## ­ƒÜÇ Current Implementation Status (November 13, 2025)

### Always Run Make from Root Directory

```powershell### ­ƒÄë TESTING IN PROGRESS - FEATURES DEPLOYED TO GITHUB Ô£à

# CORRECT - from repository root

make**Successfully Merged to Master and Pushed (November 13, 2025)**:

make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION"- Ô£à **Aggressive Flow Control**: Single-threshold system defers "ok" until motion queue empty

- Ô£à **GRBL v1.1 Protocol Compliance**: Full blank line/comment handling with proper "ok" responses  

# WRONG - never cd into srcs/- Ô£à **Motion Completion Synchronization**: Deferred "ok" only sent when `motionQueueCount == 0`

cd srcs && make  # ÔØî This breaks paths!- Ô£à **File Streaming Success**: Rectangle, circle, and complex toolpaths complete flawlessly

```- Ô£à **UGS Integration**: Automatic file completion without manual reset

- Ô£à **Counter-Based Deferred OK**: Burst sending of all deferred responses

All Makefile paths are relative to project root. The build system automatically handles subdirectories.- Ô£à **Arc Radius Compensation**: GRBL v1.1 $13 setting with 0.002mm tolerance

- Ô£à **Control Character Cleanup**: Proper CR/LF consumption prevents spurious responses

## Core Architecture

### ÔÜá´©Å ACTIVE ISSUES

### Hardware Abstraction Layer (utils.c/utils.h)**None - All major issues resolved and deployed!**

**LED Pattern Design** - All GPIO operations use clean function pointer arrays:

**Recently RESOLVED Issues (November 13, 2025)**:

```c- Ô£à **UGS Premature "Finished" State** - Fixed with aggressive flow control (defer until queue empty)

// Direct function pointer arrays (no nested structures!)- Ô£à **File Streaming Completion** - Rectangle test completes both iterations successfully

GPIO_SetFunc axis_step_set[NUM_AXIS] = {step_x_set, step_y_set, step_z_set, step_a_set};- Ô£à **Blank Line Handling** - GRBL v1.1 compliant "ok" responses for all lines

GPIO_ClearFunc axis_step_clear[NUM_AXIS] = {...};- Ô£à **Motion Queue Synchronization** - Deferred "ok" only sent when motion truly completes

GPIO_SetFunc axis_dir_set[NUM_AXIS] = {...};- Ô£à **UGS Visualization Delay** - Fixed with startup OK deferral (November 11)

GPIO_GetFunc axis_limit_min_get[NUM_AXIS] = {x_min_get, y_min_get, z_min_get, a_min_get};- Ô£à **UGS File Streaming Stalls** - Fixed with deferred "ok" check in IDLE loop (November 11)

- Ô£à Flash save hanging - Fixed with NVM_IsBusy() polling (November 10)

// Flat settings structures (no nesting!)- Ô£à Motion speed slow (98 mm/min) - Fixed, now ~8000 mm/min (November 10)

typedef struct {- Ô£à Settings persistence - Working correctly (November 10)

    float* max_rate;

    float* acceleration;### Ô£à COMPLETED FEATURES

    float* steps_per_mm;- **AGGRESSIVE FLOW CONTROL**: Single-threshold system defers "ok" until motion queue empty, prevents UGS premature "Finished" (November 13, 2025)

    int32_t* step_count;- **BLANK LINE GRBL COMPLIANCE**: All lines (blank, comments, G-code) get "ok" responses with flow control applied (November 13, 2025)

} AxisSettings;- **MOTION COMPLETION SYNCHRONIZATION**: Deferred "ok" only sent when `motionQueueCount == 0` ensures UGS accuracy (November 13, 2025)

- **SEGMENT COMPLETION TRIGGERS**: `motionSegmentCompleted` flag set when queue drains, triggers deferred ok check (November 13, 2025)

extern AxisSettings g_axis_settings[NUM_AXIS];- **STARTUP OK DEFERRAL FOR UGS VISUALIZATION**: Withholds first 2 motion command OKs until 2nd motion received, preventing UGS premature "Finished" state (November 11, 2025)

extern HomingSettings g_homing_settings[NUM_AXIS];- **SINGLE AUTHORITATIVE COUNT ARCHITECTURE**: Eliminated stale count copies, all flow control reads `appData->motionQueueCount` directly (November 11, 2025)

- **IMPROVED FLOW CONTROL**: Deferred "ok" check in IDLE loop prevents UGS streaming stalls (November 11, 2025)

// Inline helpers for zero-overhead access- **MOTION PIPELINE VALIDATED**: Segments execute correctly, position tracking accurate, ISR operates properly (November 11, 2025)

static inline void AXIS_StepSet(E_AXIS axis) {- **COMPLETE HOMING & LIMIT SWITCH SYSTEM**: Professional GRBL v1.1 compatible homing with $H command, array-based limit configuration, hardware debouncing, and proper inversion logic (November 10, 2025)

    axis_step_set[axis]();  // Inlines to: LATDSET = 0x10- **COMPLETE SPINDLE PWM CONTROL**: OC8/TMR6 PWM system with 3.338kHz frequency, RPM conversion, M3/M5/S command integration (November 10, 2025)

}- **ARRAY-BASED AXIS CONTROL**: Eliminated all switch statements for axis operations using coordinate array utilities (November 10, 2025)

```- **PRODUCTION-READY G-CODE PARSER**: Professional event-driven system with clean architecture (ÔÜá´©Å CRITICAL: Do not modify gcode_parser.c - working perfectly!)

- **ROBUST SOFT RESET RECOVERY**: UGS compatible soft reset with proper TMR4 restart logic (November 10, 2025)

**Architecture Principles**:- **OPTIMAL TIMER CONFIGURATION**: TMR4 1:64 prescaler (781.25kHz) with 2.5┬Ás stepper pulses (November 10, 2025)

- Ô£à **Direct array access** - No nested structures, no accessor functions- **HARDWARE VALIDATION GUARDS**: TMR4 startup checks prevent motion restart failures (November 10, 2025)

- Ô£à **LED pattern everywhere** - Simple, clean, consistent- Event queue implementation respecting APP_DATA abstraction layer

- Ô£à **Always inline** - Compiler optimizes to single instructions- Comprehensive G-code support: G0/G1, G2/G3, G4, M3/M5, M7/M9, G90/G91, F, S, T, G10 L20

- ÔØî **NO nested structs** - Removed GPIO_Control, AxisConfig, LimitConfig- Core architecture implemented with period-based timer (TMR4/PR4)

- ÔØî **NO accessor functions** - Access `g_axis_settings[axis].field` directly- Single instance pattern in appData for clean separation

- Proper tokenization with combined modal splitting

### Flow Control System (gcode_parser.c)  - Examples: "G21G90" ÔåÆ ["G21", "G90"], "G90G0Z5" ÔåÆ ["G90", "G0Z5"]

**Single Authoritative Motion Count** - No copies, no sync:- Multi-command line support - "G90G1X10Y10F1000" ÔåÆ ["G90", "G1X10Y10F1000"]

- Modal parameter support - Standalone F, S, T commands (GRBL v1.1 compliant)

```c- 16-command circular buffer with flow control and overflow protection ("ok" withholding)

// Ô£à CORRECT - Read fresh count directly- Harmony state machine pattern - proper APP_Tasks architecture

void GCODE_Tasks(APP_DATA* appData, GCODE_CommandQueue* cmdQueue) {- Non-blocking event processing - one event per iteration

    CheckAndSendDeferredOk(appData->motionQueueCount, cmdQueue->maxMotionSegments);- Kinematics module complete with physics calculations and velocity profiling

}- Stepper module complete with hardware abstraction and emergency stop

- Persistent GRBL settings with NVM flash storage (29 parameters including arc)

// ÔØî WRONG - Don't create copies- Delayed flash initialization - read after peripherals ready (APP_LOAD_SETTINGS state)

// cmdQueue->motionQueueCount = appData->motionQueueCount;  // NO!- Unified data structures - no circular dependencies, clean module separation

```- Hardware FPU enabled - Single-precision floating point for motion planning

- Trapezoidal velocity profiling IMPLEMENTED - KINEMATICS_LinearMove with full physics

**Aggressive Flow Control**:- Emergency stop system complete - APP_ALARM state with hard/soft limit checking

- Defer "ok" when `motionQueueCount > 0`- Position tracking and modal state - Work coordinates with G90/G91 support

- Send "ok" only when `motionQueueCount == 0`- Safety system complete - STEPPER_DisableAll(), MOTION_UTILS_CheckHardLimits()

- Check deferred ok immediately after `motionSegmentCompleted` flag- 256 microstepping validated - ISR budget analysis shows 42% headroom at 512kHz

- Single ISR architecture designed - GRBL pattern, no multi-ISR complexity

### Timer Configuration (stepper.c)- PRIORITY PHASE SYSTEM IMPLEMENTED - Hybrid ISR/main loop architecture (best of both worlds!)

**Hardware Validation Guards** prevent soft reset failures:- INCREMENTAL ARC INTERPOLATION COMPLETE - Non-blocking G2/G3 with FPU acceleration

- Motion phase system operational - VELOCITY/BRESENHAM/SCHEDULE/COMPLETE phases

```c- Project compiles successfully with XC32 compiler

void STEPPER_LoadSegment(MotionSegment* segment) {- UART3 fully functional - TX and RX working, status queries respond

    // Ô£à Always validate hardware state- Non-blocking UART utilities module - central helpers (UART_SendOK, UART_Printf)

    if(!(T4CON & _T4CON_ON_MASK)) {- Professional compile-time debug system - Zero runtime overhead, multiple subsystems (November 7, 2025)

        TMR4_Start();- Clean build system - make/make all defaults to Release, make build for incremental (November 7, 2025)

    }- ATOMIC INLINE GPIO FUNCTIONS - Zero-overhead ISR with `__attribute__((always_inline))` (November 8, 2025)

    if(!(T4CON & _T4CON_ON_MASK)) {- FUNCTION POINTER ARCHITECTURE - Array-based axis control with GPIO_Control structs (November 8, 2025)

        TMR4_Start();- ISR STACK OPTIMIZATION - Eliminated 32 bytes/call local arrays with static pointers (November 8, 2025)

    }- PRE-CALCULATED DOMINANT AXIS - Stored in MotionSegment, zero ISR overhead (November 8, 2025)

    

    // Configure timing### ­ƒöº SOFT RESET RECOVERY SYSTEM (November 10, 2025) Ô¡É NEW

    uint32_t step_interval = /* calculated */;Module: `srcs/motion/stepper.c` ÔåÆ `STEPPER_LoadSegment()`

    uint32_t pulse_width = 2;  // 2.5┬Ás at 781.25kHz

    TMR4_PeriodSet(max(7, step_interval + pulse_width + 2));**Problem Solved**: UGS soft reset (Ctrl+X) would stop motion, but subsequent G-code commands wouldn't restart motion system.

}

```**Root Cause**: After soft reset, TMR4 step timer wasn't being re-started when motion resumed.



**Timer Facts**:**Solution**: Hardware validation guards in segment loading:

- TMR4: 1:64 prescaler ÔåÆ 781.25kHz (1.28┬Ás/tick)```c

- Pulse width: 2.5┬Ás (2 ticks) - safe for all stepper drivers// Re-enable OC1 if disabled

- Minimum period: 7 ticks (ensures pulse completion)// Re-start TMR4 if stopped

    TMR4_Start(); // Re-start if stopped

### Debug System (common.h, uart_utils.h)}

**Zero Runtime Overhead** - Debug code completely removed in Release builds:

// Re-start TMR4 if stopped  

```cif(!(T4CON & _T4CON_ON_MASK)) {

// Enable in Makefile    TMR4_Start();

make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"}

```

// Use in code

DEBUG_PRINT_MOTION("[MOTION] Loading segment: steps=%lu\r\n", steps);**Timer Configuration Fixed**:

DEBUG_EXEC_SEGMENT(LED1_Toggle());- TMR4 Prescaler: 1:64 (781.25kHz, 1.28┬Ás per tick)

- Pulse Width: 2.5┬Ás (2 ticks) - safe for stepper drivers

// Release build compiles to:- Period Clamping: Minimum 7 ticks to accommodate pulse timing

((void)0)  // Nothing - zero bytes!- Uses PLIB functions: `TMR4_FrequencyGet()`, `TMR4_PeriodSet()`

```

**Benefits**:

**Available Flags**:- Motion always restarts after UGS soft reset

- `DEBUG_MOTION` - Motion planning and segment execution- Optimal stepper driver compatibility (2.5┬Ás pulse width)

- `DEBUG_GCODE` - G-code parsing and event processing- Hardware state validation prevents startup failures

- `DEBUG_STEPPER` - Low-level stepper ISR- Debug output shows timer frequency and timing calculations

- `DEBUG_SEGMENT` - Segment loading and queue management

- `DEBUG_UART` - UART communication### ­ƒöº FLOW CONTROL ARCHITECTURE (November 11, 2025) Ô¡É UPDATED

- `DEBUG_APP` - Application state machine**Single Authoritative Count Source**



## Code Style RulesProblem Solved: Stale count copies causing deferred "ok" to never send when buffer drained.



### Ô£à DO Use Direct Array AccessRoot Cause Analysis:

```c- `appData->motionQueueCount` was being synced to `gcodeCommandQueue.motionQueueCount` once per iteration

// Ô£à GOOD - Direct access like LED pattern- Flow control checked the STALE copy instead of FRESH authoritative count

float rate = *(g_axis_settings[axis].max_rate);- When buffer drained, copied value hadn't updated yet, so deferred "ok" was never sent

AXIS_StepSet(AXIS_X);- UGS waited indefinitely for "ok", stopped sending remaining commands

bool triggered = LIMIT_GetMin(axis);

```Solution: Single Authoritative Source

```c

### ÔØî DON'T Create Nested Structures// REMOVED from GCODE_CommandQueue structure:

```c// uint32_t motionQueueCount;  // DELETE - was stale copy

// ÔØî BAD - Removed from codebase

typedef struct {// REMOVED from app.c APP_IDLE:

    GPIO_SetFunc Set;// appData.gcodeCommandQueue.motionQueueCount = appData.motionQueueCount;  // DELETE sync

    GPIO_ClearFunc Clear;

} GPIO_Control;  // Don't recreate this!// Flow control now reads fresh count directly:

CheckAndSendDeferredOk(appData->motionQueueCount, cmdQueue->maxMotionSegments);

typedef struct {SendOrDeferOk(appData->motionQueueCount, cmdQueue->maxMotionSegments);

    GPIO_Control step;  // 3 levels deep!```

    GPIO_Control dir;

    float* max_rate;Files Modified:

} AxisConfig;  // Don't recreate this!- `incs/data_structures.h` - Removed `motionQueueCount` field from `GCODE_CommandQueue`

```- `srcs/app.c` - Removed sync at line 241

- `srcs/gcode/gcode_parser.c` - Flow control reads `appData->motionQueueCount` directly

### Ô£à DO Use Coordinate Array Utilities

```cIdle Loop Enhancement:

// Ô£à GOOD - Loop instead of individual assignments```c

for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {case GCODE_STATE_IDLE:

    SET_COORDINATE_AXIS(&target, axis, value[axis]);    // ... UART processing ...

}    

    if (nBytesRead == 0) {

// ÔØî BAD - Individual assignments        // Ô£à Check for deferred "ok" even when no new data

target.x = value_x;        CheckAndSendDeferredOk(appData->motionQueueCount, cmdQueue->maxMotionSegments);

target.y = value_y;        break;

target.z = value_z;    }

target.a = value_a;```

```

Benefits:

### Ô£à DO Use Debug Macros- Ô£à No stale copies - always reads current buffer occupancy

```c- Ô£à Deferred "ok" sent during idle loop when buffer drains

// Ô£à GOOD - Clean debug that compiles to nothing in Release- Ô£à UGS file streaming completes full programs

DEBUG_PRINT_MOTION("[MOTION] Message\r\n");- Ô£à Cleaner architecture - single source of truth



// ÔØî BAD - Manual UART writes that clutter code### ­ƒöº STARTUP OK DEFERRAL FOR UGS VISUALIZATION (November 11, 2025) Ô¡É NEW

char buf[64];Module: `srcs/gcode/gcode_parser.c` ÔåÆ Startup deferral state machine

snprintf(buf, sizeof(buf), "[DEBUG] Message\r\n");

UART3_Write((uint8_t*)buf, strlen(buf));**Problem Solved**: UGS (Universal G-Code Sender) would receive all "ok" responses before motion started, mark file as "Finished", and stop automatic status polling. Motion wouldn't visualize until manual `?` query sent.

```

**Root Cause**: Setup commands (G21, G90, G17, G92, G1F500) received "ok" immediately. By the time first motion command executed, UGS had all expected "ok" responses and considered file complete.

## G-Code Parser Rules

**Solution**: Startup OK Deferral with Motion Detection

### ÔÜá´©Å CRITICAL: DO NOT MODIFY gcode_parser.c!- Withholds first 2 **motion command** "ok" responses

The G-code parser is **production-ready and fully validated**. It handles:- Flushes both OKs together after 2nd motion command received

- GRBL v1.1 protocol compliance- UGS sees motion start before receiving all OKs ÔåÆ continues automatic polling

- Blank line/comment handling

- Flow control synchronization**Implementation Architecture**:

- Real-time character processing```c

- Combined modal splitting (G90G1X10 ÔåÆ ["G90", "G1X10"])// Static state variables (gcode_parser.c lines 62-64)

static bool startupDeferralActive = false;

**If you think the parser needs changes**, you're probably wrong. Check:static uint8_t startupDeferralRemaining = 0;  // Counts down from 2

1. Is the issue in event handling (app.c)?static uint8_t startupDeferredOkCredits = 0;  // Accumulates withheld OKs

2. Is the issue in motion generation (kinematics.c)?

3. Is the issue in flow control timing?// Motion detection function (lines 549-585)

static bool LineHasMotionWord(const char* line, uint32_t len) {

The parser itself is **working perfectly** - all UGS/Candle/bCNC tests pass.    // Two-stage check:

    // 1. Find G0/G1/G2/G3 (excluding G10, G17, G21, etc.)

## Data Structure Patterns    // 2. Verify at least one axis parameter (X/Y/Z/A) present

    

### Single Instance Pattern    // Stage 1: Scan for G followed by 0-3

```c    bool found_g_code = false;

// All major structures in APP_DATA (app.h, data_structures.h)    for (uint32_t i = 0; i < len - 1; i++) {

typedef struct {        char c = line[i];

    APP_STATES state;        if (c == 'G' || c == 'g') {

    GCODE_CommandQueue gcodeCommandQueue;            char next = line[i+1];

    MotionSegment motionQueue[MAX_MOTION_SEGMENTS];            if (next >= '0' && next <= '3') {

    uint32_t motionQueueHead, motionQueueTail, motionQueueCount;                // Exclude G10, G17, G21, G28, G30, etc.

    bool motionSegmentCompleted;  // Triggers deferred ok check                if (i+2 < len && line[i+2] >= '0' && line[i+2] <= '9') {

    // ...                    continue;  // Skip G10+

} APP_DATA;                }

```                found_g_code = true;

                break;

**Pass by reference** through function calls - no static module data.            }

        }

### Coordinate Structures    }

```c    if (!found_g_code) return false;

// CoordinatePoint as float array (guaranteed memory layout)    

typedef struct {    // Stage 2: Verify axis parameters exist

    float x, y, z, a;  // Sequential in memory: [0]=x, [1]=y, [2]=z, [3]=a    // Prevents false positives like "G1F500" (feedrate only, no movement)

} CoordinatePoint;    for (uint32_t i = 0; i < len; i++) {

        char c = line[i];

// Array-based helpers        if (c == 'X' || c == 'x' || c == 'Y' || c == 'y' || 

static inline void SET_COORDINATE_AXIS(CoordinatePoint* coord, E_AXIS axis, float value) {            c == 'Z' || c == 'z' || c == 'A' || c == 'a') {

    ((float*)coord)[axis] = value;            return true;  // Motion command with axis movement

}        }

```    }

    return false;  // G-code present but no axis parameters

## File Organization}

```

```

srcs/**Deferral Logic** (lines 715-743 IDLE, 1045-1070 GCODE_COMMAND):

Ôö£ÔöÇÔöÇ app.c                   # Main state machine, event loop```c

Ôö£ÔöÇÔöÇ main.c                  # Entry pointbool isMotion = LineHasMotionWord(rxBuffer, terminator_pos);

Ôö£ÔöÇÔöÇ gcode/

Ôöé   ÔööÔöÇÔöÇ gcode_parser.c      # ÔÜá´©Å DO NOT MODIFY - working perfectly!// Arm deferral on first motion command

Ôö£ÔöÇÔöÇ motion/if (!startupDeferralActive && isMotion) {

Ôöé   Ôö£ÔöÇÔöÇ stepper.c           # Hardware abstraction, ISR    startupDeferralActive = true;

Ôöé   Ôö£ÔöÇÔöÇ kinematics.c        # Physics, velocity profiling    startupDeferralRemaining = 2;

Ôöé   Ôö£ÔöÇÔöÇ motion.c            # Motion queue management    startupDeferredOkCredits = 0;

Ôöé   Ôö£ÔöÇÔöÇ homing.c            # $H command, limit switches}

Ôöé   ÔööÔöÇÔöÇ spindle.c           # M3/M5, PWM control

Ôö£ÔöÇÔöÇ settings/// Withhold OKs for first 2 motion commands

Ôöé   ÔööÔöÇÔöÇ settings.c          # Persistent NVM storageif (startupDeferralActive && isMotion && startupDeferralRemaining > 0) {

ÔööÔöÇÔöÇ utils/    startupDeferralRemaining--;

    Ôö£ÔöÇÔöÇ utils.c             # Ô£à LED pattern GPIO abstraction    startupDeferredOkCredits++;

    ÔööÔöÇÔöÇ uart_utils.c        # Non-blocking UART helpers    

    // Flush both OKs when 2nd motion command received

incs/    if (startupDeferralRemaining == 0) {

Ôö£ÔöÇÔöÇ common.h                # Debug flags, shared constants        while (startupDeferredOkCredits > 0) {

Ôö£ÔöÇÔöÇ data_structures.h       # Unified structures (no circular deps)            UART_SendOK();

ÔööÔöÇÔöÇ utils/utils.h           # GPIO function pointer arrays            startupDeferredOkCredits--;

```        }

        startupDeferralActive = false;

## Hardware Details    }

} else {

**Microcontroller**: PIC32MZ2048EFH100      SendOrDeferOk(appData, cmdQueue);  // Normal flow control

**Clock**: 200MHz system, 50MHz peripheral (PBCLK3)  }

**FPU**: Single-precision enabled (`-mhard-float -msingle-float`)  ```

**Bootloader**: MikroE USB HID @ 0x9D1F4000 (39KB)

**Critical Design Details**:

**Memory Layout**:- **Two-Stage Motion Detection**: Prevents false positives from:

```  - G17/G18/G19 (plane selection)

0x9D000000 - 0x9D1EFFFF : Application (1.87MB)  - G21/G20 (units)

0xBD1F0000 - 0xBD1F3FFF : Settings NVM (16KB)  - G90/G91 (distance mode)

0x9D1F4000 - 0x9D1FBFFF : Bootloader (48KB)  - **G1F500** (feedrate setting with no axis movement)

```- **Axis Parameter Requirement**: Only commands with X/Y/Z/A trigger deferral

- **Inline OK Flushing**: Both OKs sent together when `remaining==0`

**Timer Modules**:- **No Hardware Dependencies**: Pure protocol-level solution, no motion system coupling

- TMR4: Step timing ISR source (TIMER_4_InterruptHandler, IPL6)

- TMR6: Spindle PWM (OC8 time base)**Typical Execution Sequence**:

- Core Timer: Homing debounce, delays```gcode

G21          ÔåÆ OK (setup, no deferral)

## GRBL Protocol ImplementationG90          ÔåÆ OK (setup, no deferral)

G17          ÔåÆ OK (setup, no deferral)

### Supported G-CodesG92X0Y0Z0    ÔåÆ OK (setup, G92 not motion)

- **Motion**: G0, G1, G2, G3G1F500       ÔåÆ OK (feedrate only, no X/Y/Z/A parameters!)

- **Plane**: G17, G18, G19             ÔåÆ OK (blank line)

- **Units**: G20 (inches), G21 (mm)G1X20Y0      ÔåÆ Deferral armed, OK withheld (1st motion, remaining=1)

- **Distance**: G90 (absolute), G91 (incremental)G1X20Y10     ÔåÆ OK, OK flushed together (2nd motion, remaining=0)

- **Offsets**: G92 (set position), G10 L20 (WCS)             ÔåÆ UGS sees Run state, continues automatic polling

- **Dwell**: G4 P(seconds)G1X0Y10      ÔåÆ OK (normal flow control)

G1X0Y0       ÔåÆ OK (normal flow control)

### Supported M-Codes```

- **Spindle**: M3 (CW), M4 (CCW), M5 (off)

- **Coolant**: M7 (mist), M8 (flood), M9 (off)**Benefits**:

- Ô£à UGS automatically polls during motion (no manual `?` needed)

### System Commands- Ô£à Real-time visualization without protocol hacks

- `$` - Help- Ô£à GRBL-compatible behavior pattern

- `$$` - View settings- Ô£à Robust motion detection (no false positives)

- `$I` - Build info- Ô£à Works with any G-code sender expecting GRBL protocol

- `$G` - Modal state

- `$#` - Work offsets**Testing**:

- `$H` - Home all axesRectangle test shows complete success:

- `$X` - Clear alarm- Setup commands receive immediate OKs

- `$C` - Toggle check mode- First 2 motion commands withheld

- `$<n>=<val>` - Set parameter- Motion starts, UGS sees `<Run|MPos:...>` updates

- Complete path execution with continuous visualization

### Real-Time Commands- Clean `<Idle|...>` return when finished

- `?` - Status report

- `!` - Feed hold### ­ƒöº AGGRESSIVE FLOW CONTROL (November 13, 2025) Ô¡É NEW

- `~` - ResumeModule: `srcs/gcode/gcode_parser.c` ÔåÆ Flow control system redesign

- Ctrl+X - Soft reset

**Problem Solved**: UGS marked files as "Finished" before motion completed because it received all "ok" responses while the motion queue still had segments executing. This caused UGS to stop automatic status polling, leaving motion running without visualization updates.

## Testing

**Root Cause**: Previous flow control used dual thresholds (high-water/low-water) which sent deferred "ok" responses before the motion queue was truly empty. UGS would receive all expected "ok" responses, mark the file complete, and stop polling - even though motion was still executing.

### PowerShell Scripts

```powershell**Solution**: Aggressive Single-Threshold Flow Control

# Simple test- **Defer ALL "ok" responses** when motion queue has any content (`motionQueueCount > 0`)

.\ps_commands\test_gcode.ps1 -FilePath .\gcode_tests\01_simple_square.gcode- **Send "ok" ONLY when** motion queue is completely empty (`motionQueueCount == 0`)

- Applied to ALL commands: G-code, blank lines, and comments

# Rectangle (validates dual iteration)- Ensures the final "ok" is sent only after motion truly completes

.\ps_commands\test_double_rectangle.ps1

**Implementation Architecture**:

# Circle (20 segments, 0.025mm error)```c

.\ps_commands\test_gcode.ps1 -FilePath .\tests\03_circle_20segments.gcode// Flow control function (gcode_parser.c)

```static void SendOrDeferOk(APP_DATA* appData, GCODE_CommandQueue* q)

{

### Manual Testing    // Only send "ok" immediately when motion queue is completely empty

```gcode    if (appData->motionQueueCount == 0) {

# Via PuTTY (115200,8,N,1)        DEBUG_PRINT_GCODE("[FLOW] Sending immediate ok (queue empty, motion complete)\r\n");

?                    # Status        (void)UART_SendOK();

$$                   # Settings    } else {

G90 G1 X10 F500     # Move 10mm        DEBUG_PRINT_GCODE("[FLOW] Deferring ok (queue=%lu > 0)\r\n", 

$H                   # Home                          (unsigned long)appData->motionQueueCount);

```        okPending = true;

    }

## Common Pitfalls}

### ❌ CRITICAL LESSON: Never Hardcode Homing Axes Mask — Read From $22 Setting

**Mistake made (March 2026)**: `parse_command_to_event()` in `gcode_parser.c` had:
```c
ev->data.homing.axes_mask = 0x0F; // ❌ WRONG — includes A-axis
```

**Why it caused hours of debugging**:
- `0x0F` = bits 0-3 = XYZA. The A-axis has **no limit switch** on this PCB.
- Homing XYZ succeeded, then firmware tried to home A, sought max_travel distance, found nothing → `ALARM:9`.
- Because state got stuck in `HOMING_STATE_ALARM`, a second `$H` returned `error:8`.
- The symptom looked like a state-machine bug, not a settings bug. Wasted time tracing ok-timing and SEEK alarm logic when the real problem was the wrong constant.

**Correct code — always read the setting**:
```c
ev->data.homing.axes_mask = SETTINGS_GetCurrent()->homing_enable; // ✅ from $22
```

**The setting `$22` (homing_enable) is already the per-axis bitmask**:
- `$22=7`  → 0x07 = bits 0-2 = XYZ only (default, correct for this board)
- `$22=15` → 0x0F = XYZA (only valid if A has a limit switch fitted)
- `$22=1`  → UGS sends this meaning "enable", maps to 0x07 internally

**Rule**: Any time you write code that iterates over homing axes or builds an axes mask for `$H`, read `settings->homing_enable` — **never** write a literal bitmask.


// Deferred ok check (gcode_parser.c)

```cvoid GCODE_CheckDeferredOk(APP_DATA* appData, GCODE_CommandQueue* q) {

// ÔØî BAD - Breaks validated timing    // Only send deferred "ok" when motion queue is completely empty

TMR4_PeriodSet(step_interval);  // Missing pulse width + margin!    if (okPending && appData->motionQueueCount == 0) {

        DEBUG_PRINT_GCODE("[DEFERRED] Sending deferred ok (queue empty)\r\n");

// Ô£à GOOD - Validated formula        if (UART_SendOK()) okPending = false;

TMR4_PeriodSet(max(7, step_interval + pulse_width + 2));    }

```}

```

### ÔØî Don't Create Copies of Motion Count

```c**Motion Segment Completion Triggers** (`srcs/motion/motion.c`):

// ÔØî BAD - Stale copy causes flow control failures```c

uint32_t local_count = appData->motionQueueCount;// Zero-length segment skip (line 211)

if (local_count == 0) { /* too late! */ }if (appData->currentSegment->steps_remaining == 0) {

    appData->motionQueueTail = (appData->motionQueueTail + 1) % MAX_MOTION_SEGMENTS;

// Ô£à GOOD - Always read fresh    appData->motionQueueCount--;

if (appData->motionQueueCount == 0) { /* current! */ }    appData->currentSegment = NULL;

```    

    // Ô£à Signal that queue space became available

### ÔØî Don't Block in Main Loop    appData->motionSegmentCompleted = true;

```c    return;

// ÔØî BAD - Blocking delay}

for(int i=0; i<1000000; i++);

// Normal segment completion (line 315)

// Ô£à GOOD - Non-blocking state machineappData->motionQueueTail = (appData->motionQueueTail + 1) % MAX_MOTION_SEGMENTS;

if (timer_elapsed()) {appData->motionQueueCount--;

    state = NEXT_STATE;

}// Ô£à Signal that queue space became available

```appData->motionSegmentCompleted = true;

```

## Production Validation Status

**Deferred OK Check in Main Loop** (`srcs/app.c` line 247):

**Validated (November 13-15, 2025)**:```c

- Ô£à Rectangle path (dual iteration, 4 corners)case APP_IDLE:

- Ô£à Circle (20 segments, 0.025mm final error)    MOTION_Tasks(&appData);

- Ô£à Arc compensation (0.001mm radius tolerance)    

- Ô£à Back-to-back file execution    // Ô£à Check deferred OKs immediately when segment completes

- Ô£à UGS automatic completion    if (appData.motionSegmentCompleted) {

- Ô£à Soft reset recovery        appData.motionSegmentCompleted = false;

- Ô£à LED pattern GPIO refactoring (264KB firmware)        GCODE_CheckDeferredOk(&appData, &appData.gcodeCommandQueue);

- Ô£à Array-based architecture refactoring    }

```

**Known Limitations**:

- No S-curve acceleration (trapezoidal only)

- Homing not CNC-tested

- Spindle PWM not CNC-validated

- StallGuard2 / sensorless homing deferred (Phase 2)

- G38.x probe implemented but pending hardware test

**Implemented (March 2026)**:

- ✅ GRBL-exact three-pass look-ahead planner (reverse + forward + trapezoid pass)

- ✅ Trapezoidal velocity profiling (KINEMATICS_RecalculateTrapezoid)

- ✅ Axis-limited acceleration and rate (limit_acceleration_by_axis / limit_rate_by_axis)

- ✅ Junction speed via GRBL centripetal approximation (no trig — dot product only)

### Build Commands```

```powershell

make                    # Release build**Files Modified**:

make clean && make      # Clean build1. `srcs/gcode/gcode_parser.c` - Aggressive flow control logic

make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION"2. `srcs/motion/motion.c` - Set `motionSegmentCompleted` flag when segments complete

```3. `srcs/app.c` - Check deferred ok immediately after motion tasks

4. `incs/data_structures.h` - Added `bool motionSegmentCompleted` flag to APP_DATA

### GPIO Access

```c**Execution Flow**:

AXIS_StepSet(axis);           // Set step pin1. First command arrives, queue = 0 ÔåÆ Send "ok" immediately

AXIS_StepClear(axis);         // Clear step pin2. Command queued, queue = 1

AXIS_DirSet(axis);            // Set direction3. Second command arrives, queue > 0 ÔåÆ Defer "ok" (set okPending = true)

bool limit = LIMIT_GetMin(axis);  // Read limit switch4. Motion executes, segment completes, queue = 0

```5. `motionSegmentCompleted` flag triggers check

6. Deferred ok check: `okPending && queue == 0` ÔåÆ Send "ok"

### Settings Access7. UGS receives "ok", sends next command

```c8. Process repeats until file completes

float rate = *(g_axis_settings[axis].max_rate);9. Final command (M0, blank lines, etc.) deferred until queue = 0

float accel = *(g_axis_settings[axis].acceleration);10. UGS only sees "Finished" AFTER all motion truly completes

bool homing_enabled = *(g_homing_settings[axis].homing_enable);

```**Benefits**:

- Ô£à UGS stays in "Run" state until motion completes

### Debug Output- Ô£à Continuous status polling and visualization

```c- Ô£à Accurate "Finished" timing - only when motion done

DEBUG_PRINT_MOTION("[MOTION] Value: %d\r\n", value);- Ô£à Works with blank lines and comments (GRBL v1.1 compliant)

DEBUG_EXEC_SEGMENT(LED1_Toggle());- Ô£à No premature "Finished" state

```- Ô£à Clean file streaming from start to finish



### Coordinate Utilities**Test Results** (Rectangle Test - November 13, 2025):

```c- Both rectangles completed successfully

SET_COORDINATE_AXIS(&point, axis, value);- Diagonal move executed

float val = GET_COORDINATE_AXIS(&point, axis);- Return to origin (0,0,0) completed

ADD_COORDINATE_AXIS(&point, axis, delta);- All "ok" responses properly timed

```- UGS showed `<Run>` throughout motion

- UGS showed `<Idle>` only after motion stopped

## Documentation References- No premature "Finished" state

- Complete file execution with continuous visualization

- **[README.md](../README.md)** - Project overview

- **[Settings Reference](../docs/readme/SETTINGS_REFERENCE.md)** - GRBL parameters### ­ƒöº ATOMIC INLINE GPIO ARCHITECTURE (November 8, 2025)

- **[Debug Tutorial](../docs/readme/DEBUG_SYSTEM_TUTORIAL.md)** - Debug system guideModule: `incs/utils/utils.h`, `srcs/utils/utils.c`, `srcs/motion/stepper.c`

- **[Memory Map](../docs/readme/MEMORY_MAP.md)** - Flash layout

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
void TIMER_4_InterruptHandler(uintptr_t context) {
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

### ­ƒöº ARRAY-BASED AXIS CONTROL (November 10, 2025)
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
- **homing.c**: 4 switch statements ÔåÆ `ADD_COORDINATE_AXIS()` calls
  - Search/backoff/locate/pulloff phases now use single-line array operations
- **motion_utils.c**: Limit checking switch ÔåÆ `g_limit_config[axis].limit.GetMin/Max()`
- **All axis operations**: Now use consistent array-based pattern

Performance Benefits:
- **Branch Elimination**: 20+ conditional branches removed from hot paths
- **Code Reduction**: ~40 lines of switch logic ÔåÆ 5 one-liner calls
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

### ­ƒöº COMPLETE HOMING & LIMIT SWITCH SYSTEM (November 10, 2025) Ô¡É NEW
Module: `srcs/motion/homing.c`, `srcs/motion/motion_utils.c`, `srcs/utils/utils.c`

**Purpose**: Professional GRBL v1.1 compatible homing system with array-based limit switch configuration and proper inversion logic.

**Key Features**:
- **$H Command Integration**: G-code parser handles `$H` system command with proper event generation
- **Array-Based Limit Configuration**: Function pointers for Min/Max limit switches per axis
- **4-Phase Homing Cycle**: GRBL-compatible seek ÔåÆ locate ÔåÆ pulloff ÔåÆ complete sequence  
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

### ­ƒöº COMPILE-TIME DEBUG SYSTEM (November 7, 2025)
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
# ÔÜá´©Å CRITICAL: ALWAYS run make from repository root directory
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

### ÔÜá´©Å CRITICAL DEBUG WORKFLOW (November 7-10, 2025)
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
   DEBUG_PRINT_STEPPER("[STEPPER] TMR4 state: 0x%08X\r\n",
                       (unsigned)T4CON);
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

### ­ƒöº NON-BLOCKING UART UTILITIES (November 6-9, 2025)
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

### ­ƒöº VISUAL MOTION DEBUG (November 6, 2025)
Added: `LED2_Toggle()` in `STEPPER_ScheduleStep()` (srcs/motion/stepper.c line ~125)

Purpose: Visual confirmation that motion scheduling is executing

Behavior:
- Rapid blink (many Hz) ÔåÆ `STEPPER_ScheduleStep()` IS being called ÔåÆ Motion system working
- Slow blink (~1Hz heartbeat) ÔåÆ Function NOT being called ÔåÆ Phase system or segment loading issue

Usage:
```gcode
G1 X1 F100      ; Move 1mm in X axis
```

Observe LED2:
- If rapid blink: Motion hardware OK, check if motors actually moving
- If slow blink: Motion segments not loading or phase system stuck

### ­ƒöº ACTIVE DEBUGGING SESSION (November 5-6, 2025)
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
  - Rapid blink = `STEPPER_ScheduleStep()` executing ÔåÆ Check motor drivers
  - Slow blink = Function not called ÔåÆ Phase system or event parsing issue
- Enable selective debug - Only G0/G1 events, not control characters

### ­ƒöº ACTIVE DEBUGGING SESSION - UGS CONNECTION (November 7-9, 2025)
Problem: UGS connects, sends commands (`?`, `$I`, `$$`), but may disconnect if TX buffer is too small. Putty works correctly, confirming firmware responds.

CRITICAL FIX:
- Root Cause: MCC regeneration can revert UART3 TX buffer size from 1024 to 256 bytes
- Symptom: `$$` command response (~400-500 bytes) overflows 256-byte TX buffer
- Result: UGS times out waiting for complete settings response, disconnects

ÔÜá´©Å After ANY MCC regeneration, ALWAYS verify UART3 buffer sizes!

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
- `$SLP` not supported ÔåÆ `error:2`

### G10 L20 Work Offset Setting
- Supports `G10 L20` with `P0` (current WCS) or `P1` (G54)
- Parameters X/Y/Z set desired WCS work position at current machine position:
  - WCS offset computed as `offset = MPos - desired_WPos`
- A-axis parameter accepted but not applied (no A in current WCS struct)

### Combined Modal Token Splitting (Parser)
- Tokens like `G21G90` or `G90G0Z5` are split and queued as separate commands in order
- Parameters belong to the final modal in the combined token
  - Example: `G90G1X10` ÔåÆ ["G90", "G1X10"]
  - Example: `G21G90 G0Z5` ÔåÆ Tokens: "G21", "G90", "G0Z5"

## Core Architecture Principles

### Priority-Based Phase System (CRITICAL)
The "Best of Both Worlds" Hybrid Architecture

Problem Solved:
- G-code processing could block motion timing (arc generation takes time)
- UART should be polled at a reasonable rate, not every ISR tick
- Need ISR precision but main loop flexibility

Solution: Priority Phase System
- ISR sets flag when dominant axis fires ÔåÆ wakes main loop
- Main loop processes phases in priority order (0 = highest)
- G-code only runs when IDLE ÔåÆ prevents blocking
- Rate-limited UART ÔåÆ polled periodically in IDLE

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
- OC1 dual-compare auto-generates step GPIO pulses in hardware (no OC1 interrupt)
- `PR4` sets the period (step interval + pulse width + margin)
- Example: For 1ms steps: `PR4 = 789` (OC1 dual-compare configures hardware pulse shape via SFR)
- Hardware Configuration (November 10, 2025 - VERIFIED):
  - PBCLK3 = 50MHz (peripheral bus clock)
  - Prescaler = 1:64 (TCKPS = 6, verified in plib_tmr4.c)
  - Timer Frequency = 781.25kHz (50MHz / 64)
  - Timer Resolution = 1.28┬Ás per tick
  - Pulse Width = 2.5┬Ás (2 ticks) - safe for stepper drivers
- No timer rollover issues - TMR4 automatically resets to 0 at PR4, OCx values remain valid
- Step timing controlled entirely by TIMER_4_InterruptHandler scheduling the next PR4 period
- Hardware validation guards ensure OC1/TMR4 restart after soft reset

### Dynamic Dominant Axis Tracking
- Dominant axis (highest step count) drives the step timing
- Dominant axis determines step_interval for PR4
- Subordinate axes step on-demand when Bresenham requires a step
- Dominant axis can swap mid-motion by recalculating Bresenham state

### Bresenham Integration
- Bresenham algorithm runs in TIMER_4_InterruptHandler for precise timing
- ISR: Update velocity profile, run Bresenham, schedule next PR4 period
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
  - Example: `G90G1X10Y10F1000` ÔåÆ Tokens: "G90", "G1X10Y10F1000"
  - Example: `G21G90 G0Z5` ÔåÆ Tokens: "G21", "G90", "G0Z5"
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
void TIMER_4_InterruptHandler(void) {
    // Clear interrupt flag FIRST
    IFS0CLR = _IFS0_T4IF_MASK;
    
    // Schedule next pulse using period-based timing
    uint32_t step_interval = current_segment->step_interval;
    uint32_t pulse_width = 2;  // 2 ticks = 2.5┬Ás at 781.25kHz
    
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
uint32_t pulse_width = 2;             // 2.5┬Ás pulse width
uint32_t period = step_interval + pulse_width + 2;
if (period < 7) period = 7;          // Minimum period guard
TMR4_PeriodSet(period);              // Timer rolls over

// INCORRECT - Don't use absolute timer reads
uint32_t now = TMR4;  // WRONG - timer is period-based!
// now + 500 would ignore period rollover -- wrong approach
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

    // Literal "0x18" typed by user ÔåÆ soft reset
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
- Set PR4 smaller than pulse_width + step_interval (pulse won't complete)
- Use blocking delays in main loop (let APP_Tasks run freely)
- Modify PR4 outside of TIMER_4_ISR during active motion
- Set period < 7 ticks (insufficient time for 2.5┬Ás pulse completion)
- Assume OC1/TMR4 remain enabled after soft reset

### Always Do
- Use period-based timing with proper guards: `TMR4_PeriodSet(max(7, step_interval + pulse_width + margin))`
- Clear interrupt flags immediately at ISR entry
- Keep ISRs minimal and fast
- Run Bresenham logic in TIMER_4_InterruptHandler for precise timing
- Schedule subordinate axes only when required
- Set OCxR = OCxRS to disable pulse generation (prevents spurious pulses)
- Validate TMR4 hardware state before loading motion segments

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

## Planner Architecture (March 2026 — GRBL-Exact, Implemented)

### MOTION_PlannerRecalculate (motion.c)
Called after every segment is enqueued. Exact port of GRBL's `planner_recalculate()`.

**Three passes:**
1. **Reverse** (newest → oldest): `entry_speed_sqr = min(max_entry_speed_sqr, next->entry_speed_sqr + 2*accel*mm)`
2. **Forward** (oldest → newest): cap `entry_speed_sqr` where preceding segment cannot accelerate to it
3. **Trapezoid**: `KINEMATICS_RecalculateTrapezoid(seg, entry_mms, exit_mms)` on every buffered segment

`speed_locked` segments are skipped in the reverse and forward passes (arc segments with fixed cruise speed).

### KINEMATICS_LinearMove (kinematics.c)
Exact port of GRBL's `plan_buffer_line()`:
- `convert_to_unit_vector(delta_mm)` — normalises move direction, returns `seg->millimeters`
- `limit_acceleration_by_axis(unit_vec)` — axis-limited combined acceleration
- `limit_rate_by_axis(unit_vec)` — axis-limited combined feed rate
- Junction cosθ = -dot(prev_unit_vec, unit_vec); centripetal approximation for `max_junction_speed_sqr`
- Updates `pl_previous_unit_vec[]` and `pl_previous_nominal_speed` for next call

### KINEMATICS_RecalculateTrapezoid (kinematics.c)
Exact port of GRBL's `calculate_trapezoid_for_block()`:
- Takes settled `entry_mms` and `exit_mms` from the planner passes
- Solves accel/decel intersection (handles truncated trapezoid when no plateau fits)
- Writes: `initial_rate`, `nominal_rate`, `final_rate`, `accelerate_until`, `decelerate_after`, `step_interval`
- ISR reads these as pure integers — no float, no division in the ISR path

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
  - Timer Frequency: 781.25kHz (50MHz ├À 64)
  - Timer Resolution: 1.28┬Ás per tick
  - Pulse Width: 2.5┬Ás (2 ticks) - safe for stepper drivers
  - Period Clamping: Minimum 7 ticks for pulse timing accommodation
  - TMR5 16-bit timer for step pulse width (one-shot mode)
- Output Compare Modules: OC1–OC4 (dual-compare hardware auto-generates step GPIO pulses; no OC interrupt used)
  - OC1 configured with TMR4 time base for X-axis step pulse
  - Hardware pulse generation only — TIMER_4_InterruptHandler is the step ISR
  - Hardware validation guards prevent startup failures
- Microstepping Support: Designed for up to 256 microstepping
  - Worst case: 512kHz step rate (256 microsteps ├ù high speed)
  - ISR budget: ~390 CPU cycles @ 512kHz (225 cycles used)
  - Per-step timing: ~6 timer ticks minimum (7.68┬Ás)

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

## Unified Data Structures (November 11, 2025 - UPDATED)
Architecture Pattern:
All major data structures consolidated in `incs/data_structures.h` to eliminate circular dependencies and provide clean module separation.

Structure Hierarchy:
```c
// incs/data_structures.h
- E_AXIS enum (AXIS_X, AXIS_Y, AXIS_Z, AXIS_A, NUM_AXIS)
- MotionSegment (Bresenham, physics, timing)
- GCODE_Command
- GCODE_CommandQueue
  Ôö£ÔöÇÔöÇ commands[16]
  Ôö£ÔöÇÔöÇ head, tail, count
  ÔööÔöÇÔöÇ maxMotionSegments (max buffer size ONLY, no current count)
- APP_STATES enum
- APP_DATA
  Ôö£ÔöÇÔöÇ state
  Ôö£ÔöÇÔöÇ gcodeCommandQueue
  ÔööÔöÇÔöÇ motionQueue[16], head, tail, count (AUTHORITATIVE)
```

Flow Control Infrastructure (Single Source):
Flow control reads `appData->motionQueueCount` directly - no copies, no sync:
```c
// app.c - NO sync needed (removed November 11)
case APP_IDLE:
    // Ô£à No sync - flow control reads appData.motionQueueCount directly
    GCODE_Tasks(&appData, &appData.gcodeCommandQueue);
    break;

// gcode_parser.c - reads fresh count directly
void GCODE_Tasks(APP_DATA* appData, GCODE_CommandQueue* cmdQueue) {
    // Check deferred "ok" using FRESH authoritative count
    CheckAndSendDeferredOk(appData->motionQueueCount, cmdQueue->maxMotionSegments);
    // ...
}
```

Benefits of Single Source Architecture:
- No circular dependencies - all structures in one header
- Clean module separation - data vs logic separation
- Single source of truth - `appData->motionQueueCount` is ONLY modified in `motion.c`
- No stale copies - flow control always sees current buffer occupancy
- Easy to test - mock entire APP_DATA structure
- Flow control works - deferred "ok" sent when buffer actually drains

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
- RowWrite fewer ops ÔåÆ more reliable

## Arc Interpolation Implementation (COMPLETED Ô£à November 4, 2025)

### Overview
Arc interpolation provides smooth circular motion for G2/G3 commands. The incremental streaming architecture generates one segment per iteration, preventing motion queue starvation and enabling non-blocking operation.

### Implementation Architecture

Incremental Streaming Pattern:
- Non-blocking: ONE segment generated per APP_Tasks() iteration
- Self-regulating: Only generates when motion queue has space
- FPU-accelerated: Hardware sin/cos for smooth arcs (50-100╬╝s per segment)
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
   - `r_start = sqrt(I┬▓ + J┬▓)` (radius from start to center)
   - `r_end = sqrt((end.x - center.x)┬▓ + (end.y - center.y)┬▓)` (radius from end to center)
   - If `|r_start - r_end| > 0.005mm`, trigger ALARM (arc radius error)
3. Angles:
   - `start_angle = atan2f(start.y - center.y, start.x - center.x)`
   - `end_angle = atan2f(end.y - center.y, end.x - center.x)`
   - `total_angle` with wrap-around handling for CW/CCW direction
4. Arc length: `arc_length = radius ├ù total_angle` (in radians)
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

## ­ƒöº BUILD SYSTEM STATUS (November 6, 2025)

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

## 🏁 PRODUCTION DEPLOYMENT STATUS (March 9, 2026)

### **GitHub Repository Status**
- **Branch**: `scurve_motion` → **merge to master pending**
- **HEAD commit**: `95e4c4e` — "docs: full test matrix — all tests pass both modes"
- **Status**: ✅ **PRODUCTION READY — pending final merge to master**
- **Last validated**: March 9, 2026

### **Validated Build**
- **Commit**: `49e1bf6` — arc-to-arc streaming stall fix + pipelined mode validation
- **Firmware**: `bins/CNC_V3.hex`
- **Architecture**: S-curve velocity profiling, inline G-code dispatch, DDS interpolator

### **Hardware Validated Test Matrix**
All tests run in both UGS **single-step** and **pipelined streaming** modes:

| Test file | Mode | Result | Notes |
|-----------|------|--------|-------|
| `08_arc_cw_ccw_stress.gcode` | Single-step | ✅ PASS | 36 arcs, 2:18 min, return to origin |
| `08_arc_cw_ccw_stress.gcode` | Pipeline    | ✅ PASS | Smooth continuous, zero stop-starts |
| `09_concentric_semicircles.gcode` | Pipeline | ✅ PASS ×2 | 56s/55s, X error <0.031mm |
| `05_three_arcs_simple.gcode` | Both | ✅ PASS | CW/CCW arc handoff stable |
| `02_rectangle_path.gcode` | Both | ✅ PASS | Corners sharp, return to origin |
| `03_circle_20segments.gcode` | Both | ✅ PASS | 20-segment circle, <0.025mm error |
| `07_complex_long_run_fast.gcode` | Both | ✅ PASS | Full pipeline: arcs, linears, G4, return |

### **Key Arc Streaming Fixes (March 2026)**
1. **Inline trajectory dispatch** — G-code bypasses 64-slot queue; backpressure at trajectory level prevents arc-to-arc stall
2. **`service_realtime_byte()`** — `?` status answered during arc retry; UGS never times out
3. **Push-back slot + drain gate** — first G-code byte preserved across retry; pipelined bytes safe in ring

### **Architecture (Current)**
```
UART RX → Parser (inline dispatch) → Trajectory (64-slot S-curve) → Interpolator → TMR4_ISR
```
- `srcs/gcode/gcode_parser.c` — inline dispatch, `service_realtime_byte`, push-back slot
- `srcs/motion/motion_bridge.c` — G-code event → trajectory move bridge
- `srcs/motion/trajectory.c` — S-curve planner, 3-pass lookahead (GRBL-exact)
- `srcs/motion/interpolator.c` — fixed-rate DDS step generator (100 kHz TMR4 ISR)
- `srcs/motion/kinematics.c` — coordinate transforms (Work ↔ Machine) only
- `srcs/motion/homing.c` — event-driven 4-phase homing with INTERPOLATOR_Stop/TRAJECTORY_Reset

### **System Status**: 🚀 **READY FOR PRODUCTION CNC OPERATIONS**

---

## TMC5160 SPI Driver - Implementation Plan (February 23, 2026)

**Branch**: `liteplacer`
**Status**: Planning / Infrastructure Complete - Implementation Pending

### Overview

Add TMC5160 stepper driver support via SPI2. The architecture preserves the existing Step/Dir/Enable GPIO motion system completely - SPI is used exclusively for:
1. **Startup configuration** - motor current, microstepping, StealthChop, ChopConf registers
2. **Runtime diagnostics** - DRVSTATUS polling (stall, overtemp, open load, short circuit)
3. **Phase 2 (future)** - StallGuard2 for sensorless homing

The existing stepper ISR, Bresenham interpolation, OC/TMR timing, and all motion code are **untouched**.

### Infrastructure Completed (February 23, 2026)

#### MCC / Harmony Changes
- SPI2 peripheral added and configured in MCC (SPI Master mode)
- MCC regenerated: `srcs/config/config/default/peripheral/spi/spi_master/plib_spi2_master.c/.h`
- Headers moved to `incs/config/config/default/peripheral/spi/spi_master/` via `make build_dir DRY_RUN=0`

#### Build System Fixes
- Root `Makefile` - `build_dir` target now forwards `DRY_RUN=$(DRY_RUN)` to inner make
- `srcs/Makefile` - `SRC_DIRS` wildcard extended from 4 to 6 levels deep to handle `spi/spi_master/` nesting

### Files to Create

```
srcs/motion/tmc5160.c        - TMC5160 SPI driver implementation
incs/motion/tmc5160.h        - Register map, config structs, public API
```

### Architecture - Step/Dir + SPI Config

```
PIC32MZ                          TMC5160 (per axis)
SPI2 SCK  ─────────────────────► SCK
SPI2 MOSI ─────────────────────► SDI
SPI2 MISO ◄─────────────────────SDO
CS_X (GPIO) ────────────────────► CSN  (axis X)
CS_Y (GPIO) ────────────────────► CSN  (axis Y)
CS_Z (GPIO) ────────────────────► CSN  (axis Z)
CS_A (GPIO) ────────────────────► CSN  (axis A)
StepX/DirX ─────────────────────► STEP/DIR (unchanged)
EnX ────────────────────────────► EN   (unchanged, active low)
```

CS Strategy: Separate GPIO CS per axis (4 pins) - simpler than daisy-chain, easier to debug.

### TMC5160 Key Registers to Configure at Startup

| Register   | Address | Purpose                                     |
|------------|---------|---------------------------------------------|
| GCONF      | 0x00    | Global config (en_pwm_mode for StealthChop) |
| IHOLD_IRUN | 0x10    | Motor current (hold / run current)          |
| CHOPCONF   | 0x6C    | Chopper config (microstep resolution)       |
| PWMCONF    | 0x70    | StealthChop PWM config                      |
| GSTAT      | 0x01    | Read global status / clear flags            |
| DRVSTATUS  | 0x6F    | Read driver status (stall, temp, shorts)    |

### SPI Frame Format (TMC5160)

40-bit frame (5 bytes):
Byte 0:  bit7=Write(1)/Read(0)  bits6:0=Register address
Bytes 1-4: 32-bit data MSB first
Response: Previous register read data returned during next transaction

### Planned API (tmc5160.h)

```c
// Initialization
void TMC5160_Initialize(void);
void TMC5160_ConfigAxis(E_AXIS axis);

// Register access
bool TMC5160_WriteRegister(E_AXIS axis, uint8_t reg, uint32_t data);
uint32_t TMC5160_ReadRegister(E_AXIS axis, uint8_t reg);

// Diagnostics (non-blocking, called from APP_IDLE rate-limited)
bool TMC5160_CheckStatus(E_AXIS axis, uint32_t* status_out);
bool TMC5160_IsOverTemp(E_AXIS axis);
bool TMC5160_IsOpenLoad(E_AXIS axis);

// Current control
void TMC5160_SetRunCurrent(E_AXIS axis, uint8_t irun);   // 0-31 scale
void TMC5160_SetHoldCurrent(E_AXIS axis, uint8_t ihold); // 0-31 scale
```

### Integration Points in Existing Code

- `srcs/app.c` APP_CONFIG state - Call `TMC5160_Initialize()` at startup
- `srcs/app.c` APP_IDLE state - Call `TMC5160_CheckStatus()` rate-limited (not every loop)
- `srcs/motion/stepper.c` - No changes (Step/Dir/Enable fully unchanged)
- `incs/utils/utils.h` - Add CS GPIO inline functions following existing LED/axis pattern

### CS Pin Assignment (TBD - confirm after PCB/MCC pin manager review)

```c
// Placeholders - update once physical pins confirmed
static inline void TMC_CS_X_Set(void)   { /* GPIO */ }
static inline void TMC_CS_X_Clear(void) { /* GPIO */ }
// Y, Z, A follow same pattern
```

### GRBL Settings for TMC5160 — IMPLEMENTED ✅ ($200–$253)

Added to `CNC_Settings` struct (guarded `#ifdef HAS_TMC5160_AXIS`). Persist to NVM flash
(SETTINGS_VERSION = 3). Changes take effect immediately via `TMC5160_ApplySettings()` — no reboot needed.

| Param      | Axis | Description                                               | Default |
|------------|------|-----------------------------------------------------------|---------|
| $200–$203  | X–A  | Chopper mode: 1=StealthChop 2=SpreadCycle 3=Mixed 4=CoolStep | 1   |
| $210–$213  | X–A  | Run current 0–31 (linear % of Vref)                      | 20      |
| $220–$223  | X–A  | Hold current 0–31                                         | 10      |
| $230–$233  | X–A  | Microstep resolution (0=256 … 8=full-step)                | 4 (16µ) |
| $240–$243  | X–A  | TPWMTHRS — StealthChop→SpreadCycle crossover (Mixed mode) | 500     |
| $250–$253  | X–A  | TCOOLTHRS — CoolStep lower velocity threshold             | 0       |

Parameter format: `$2XY=value` where X=tens digit (0=mode,1=irun,2=ihold,3=mres,4=pwmthrs,5=coolthrs)
and Y=axis (0=X, 1=Y, 2=Z, 3=A). Only active when `HAS_TMC5160_AXIS` is defined.
Handler in `srcs/gcode/gcode_parser.c` calls `TMC5160_ApplySettings((E_AXIS)(param % 10), s)`
immediately after flash save.

### StallGuard2 / Sensorless Homing (Phase 2 - deferred)

- Read SG_RESULT from DRVSTATUS during homing seek
- Replace physical limit switch trigger with stall detection threshold
- Requires careful tuning of SGTHRS per axis and speed
- Do NOT implement until basic SPI config is validated on hardware

### Rules for TMC5160 Implementation

- DO NOT modify srcs/motion/stepper.c ISR - timing-critical, Step/Dir unchanged
- DO NOT modify srcs/gcode/gcode_parser.c - working perfectly
- DO NOT use blocking SPI waits in any ISR context
- All SPI transactions must be in main loop context only (APP_CONFIG or rate-limited APP_IDLE)
- Follow LED pattern for CS GPIO - add to axis_cs_set[]/axis_cs_clear[] arrays in utils.c

---

## 🗺️ DEVELOPMENT ROADMAP — Ordered Implementation Phases

Phases must be implemented **in sequence**. Do not begin a phase until the previous one is
hardware-validated and committed to master.

---

### Phase 1 — Probing (G38.x) ✅ CODE COMPLETE — pending hardware test

**Status**: Fully implemented in code. Pending validation on real hardware (probe pin/switch).

**What is implemented**:
- G38.2/3/4/5 fully parsed in `gcode_parser.c` (subcode, toward/away, alarm_on_fail flags, all axis params)
- `GCODE_EVENT_PROBE_TOWARD` / `GCODE_EVENT_PROBE_AWAY` dispatched inline via `motion_bridge.c`
- `motion_bridge.c` arms `PROBE_STATE_MOVING`, queues trajectory move to probe target at probe feedrate
- `app.c` APP_IDLE monitors `PROBE_Get()` every iteration; on trigger: stops interpolator, latches `probePosition`
- Sends `[PRB:x,y,z,a:1]` on success; `[PRB:x,y,z,a:0]` + `ALARM:5` on G38.2/G38.4 failure
- `$#` handler reports last `[PRB:x,y,z:n]` result
- Probe pin: Z-Max (RE1) with `$6` inversion support

**Key files**: `srcs/gcode/gcode_parser.c:501`, `srcs/motion/motion_bridge.c:883`, `srcs/app.c:360`

---

### Phase 2 — Tool Length Offsets (G43 / G49)

**Goal**: Apply tool length compensation so Z=0 is always the workpiece surface regardless of tool.

**Scope**:
- G43 Hn — activate tool length offset for tool n
- G49 — cancel tool length offset
- Tool table: up to 16 tools, length stored in NVM (`settings.tool_length_offset[16]`)
- G43.1 Zn — set dynamic offset (current tool, inline value)
- `TLO:` field in `$#` report

**Architecture**:
- Offset applied in `KINEMATICS_GetCurrentPosition()` / coordinate transforms — not in motion planner
- Tool number tracked in modal state (existing `T` word handling)
- `$T` command to view current tool offset table

**Key files**: `srcs/motion/kinematics.c`, `srcs/settings/settings.c`, `srcs/gcode/gcode_parser.c`

---

### Phase 3 — Multiple Work Offsets (G54–G59)

**Goal**: Six independent work coordinate systems, switchable mid-program.

**Scope**:
- G54–G59 modal WCS select (currently only G54/G92 implemented)
- `G10 L2 Pn` — set WCS offset from machine position
- `G10 L20 Pn` — set WCS offset from current work position
- `$#` response prints all six WCS offsets

**Architecture**:
- `wcs_offset[6][NUM_AXIS]` in `CNC_Settings` (replace current single `wcs_offset[NUM_AXIS]`)
- Active WCS index in modal state
- All coordinate transforms route through `KINEMATICS_WorkToMachine()` using active index
- NVM layout change → bump `SETTINGS_VERSION`

**Key files**: `srcs/motion/kinematics.c`, `incs/data_structures.h`, `srcs/settings/settings.c`

---

### Phase 4 — Rigid Tapping & Fixed Cycle Probing (G33, G81–G89)

**Goal**: Canned drilling/tapping cycles and spindle-synchronised tapping.

**Scope**:
- G81 — simple drill cycle
- G83 — peck drilling
- G33 — spindle-synchronised motion (rigid tapping precursor; requires spindle encoder)
- G98/G99 — canned cycle return plane

**Architecture**:
- Canned cycles expand to sequences of G0/G1 moves inside `gcode_parser.c` event handler — no new motion primitives needed
- G33 blocked until spindle encoder feedback is available (Phase 5)
- Implemented as a state machine inside `GCODE_Tasks` to avoid blocking

**Key files**: `srcs/gcode/gcode_parser.c`, `srcs/motion/motion_bridge.c`

---

### Phase 5 — CAN Bus & EtherCAT Drive Communication

**Goal**: Replace Step/Dir GPIO with fieldbus commands to smart drives.

**Scope — CAN (first)**:
- PIC32MZ has on-chip CAN1/CAN2 — use Harmony CAN PLIB
- CANopen DS402 profile (standard motion drive profile)
- One CAN node per axis drive
- Cyclic Synchronous Position (CSP) mode: master sends target position every servo period
- Status word polling: fault, ready, in-position

**Scope — EtherCAT (second)**:
- Requires external EtherCAT slave controller IC (e.g. LAN9252 or AX58100) on SPI2 or SPI3
- EtherCAT master stack (e.g. SOEM or proprietary) running on PIC32MZ
- Process Data Objects (PDO) map target position + control word each cycle
- Deterministic 1ms cycle time — driven from TMR4 100kHz ISR decimated to 1kHz

**Architecture**:
- Abstract drive interface: `DRIVE_SetTargetPosition(axis, steps)`, `DRIVE_GetActualPosition(axis)`
- Step/Dir path and fieldbus path selected at compile time via `DRIVE_BACKEND` in `common.h`
- `INTERPOLATOR_Tick()` writes to drive interface instead of GPIO when fieldbus backend active
- motion_bridge.c unchanged — only `interpolator.c` output path changes

**Key files**: `srcs/motion/interpolator.c`, new `srcs/drives/can_drive.c`, new `srcs/drives/ethercat_drive.c`

**Do NOT** attempt EtherCAT before CAN is validated on hardware.

---

### Phase 6 — Closed-Loop Encoder Feedback via EtherCAT / CAN

**Goal**: Close the position loop — drive reports actual encoder position; controller corrects error.

**Scope**:
- Read actual position from drive PDO each fieldbus cycle
- Position error = target (DDS accumulator) − actual (encoder)
- Proportional correction term added to next cycle's target position command
- Following error alarm: if |error| > threshold ($-parameter), trigger ALARM state
- Full servo loop runs at fieldbus cycle rate (1kHz CAN, up to 4kHz EtherCAT)

**Architecture**:
- `INTERPOLATOR_Tick()` reads `DRIVE_GetActualPosition()` each cycle
- Following error checked against `settings.following_error_limit[axis]` ($-parameter, NVM)
- Alarm raised via existing `g_hard_limit_alarm` path → APP_ALARM state
- Position display (`?` status) reports encoder position, not DDS accumulator

**This phase transforms the system from open-loop stepper to closed-loop servo.**

**Key files**: `srcs/motion/interpolator.c`, `srcs/drives/ethercat_drive.c` / `can_drive.c`, `srcs/app.c`

---

### Roadmap Summary Table

| Phase | Feature                        | Status      | Branch (planned)    |
|-------|--------------------------------|-------------|---------------------|
| 0     | S-curve engine + arc + homing  | ✅ Complete  | `scurve_motion`     |
| 1     | Probing (G38.x)                | ✅ Code done — hw test pending | `scurve_motion` |
| 2     | Tool length offsets (G43/G49)  | 🔲 Pending  | `tool_offsets`      |
| 3     | Multi WCS (G54–G59)            | 🔲 Pending  | `multi_wcs`         |
| 4     | Rigid tapping / canned cycles  | 🔲 Pending  | `canned_cycles`     |
| 5     | CAN + EtherCAT drives          | 🔲 Pending  | `fieldbus`          |
| 6     | Closed-loop encoder feedback   | 🔲 Pending  | `closed_loop`       |
