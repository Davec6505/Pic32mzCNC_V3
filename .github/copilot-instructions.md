# GitHub Copilot Instructions for Pic32mzCNC_V3

## Project Overview
This is a CNC motion control system for PIC32MZ microcontrollers using hardware timers and Bresenham interpolation for precise multi-axis stepper motor control.

## 🚀 Current Implementation Status (November 2025)
- ✅ **Professional event-driven G-code system** with clean architecture
- ✅ **Event queue implementation** respecting APP_DATA abstraction layer
- ✅ **Comprehensive G-code support**: G1, G2/G3, G4, M3/M5, M7/M9, G90/G91, F, S, T
- ✅ **Core architecture implemented** with absolute compare mode
- ✅ **Single instance pattern in appData** for clean separation  
- ✅ **Proper tokenization** - G/M commands keep all parameters (LinuxCNC/GRBL compatible)
- ✅ **Multi-command line support** - "G90G1X10Y10F1000" → ["G90", "G1X10Y10F1000"]
- ✅ **Modal parameter support** - Standalone F, S, T commands (GRBL v1.1 compliant)
- ✅ **16-command circular buffer** with flow control and overflow protection
- ✅ **Harmony state machine pattern** - proper APP_Tasks architecture
- ✅ **Non-blocking event processing** - one event per iteration
- ✅ **Kinematics module complete** with physics calculations and velocity profiling
- ✅ **Stepper module complete** with hardware abstraction and emergency stop
- ✅ **Persistent GRBL settings** with NVM flash storage (27 parameters)
- ✅ **Delayed flash initialization** - read after peripherals ready (APP_LOAD_SETTINGS state)
- ✅ **Unified data structures** - no circular dependencies, clean module separation
- ✅ **LED blink rate stable** - no slowdown with complex commands
- ✅ **Hardware FPU enabled** - Single-precision floating point for motion planning
- ✅ **Trapezoidal velocity profiling IMPLEMENTED** - KINEMATICS_LinearMove with full physics
- ✅ **Emergency stop system complete** - APP_ALARM state with hard/soft limit checking
- ✅ **Position tracking and modal state** - Work coordinates with G90/G91 support
- ✅ **Safety system complete** - STEPPER_DisableAll(), MOTION_UTILS_CheckHardLimits()
- ✅ **256 microstepping validated** - ISR budget analysis shows 42% headroom at 512kHz
- ✅ **Single ISR architecture designed** - GRBL pattern, no multi-ISR complexity
- � **ARC INTERPOLATION NEXT** - Critical for G2/G3 circular motion (see implementation guide below)
- 🚧 **Motion controller in progress** - Ready to implement ISR + state machine
- ✅ **Project compiles successfully** with XC32 compiler

## Core Architecture Principles

### Timer Architecture
- **TMR2 runs continuously** - Managed rollover strategy (see below)
- **32-bit timer** (TMR2:TMR3 pair) for extended range
- **Hardware Configuration:**
  - PBCLK3 = 50MHz (peripheral bus clock)
  - Prescaler = 1:4 (TCKPS = 2)
  - Timer Frequency = 12.5MHz (50MHz / 4)
  - Timer Resolution = **80ns per tick** (1 / 12.5MHz)
  - Period Register (PR2) = 0xFFFFFFFF (32-bit free-running)
  - **Rollover time: 343.6 seconds (~5.7 minutes)** at 12.5MHz
- **No timer interrupts used** - timer provides stable time base only
- **Step timing** controlled entirely by OC module compare interrupts

### Timer Rollover Management (CRITICAL)
**Problem:** 32-bit TMR2 overflows after 343.6 seconds, causing OCx scheduling issues

**Solution: Controlled Reset Before Rollover**
- Monitor TMR2 value in main loop or low-priority task
- When TMR2 > **0xF0000000** (~96% full, ~328 seconds):
  1. **Stop accepting new motion segments** (set motion queue full flag)
  2. **Wait for all pending motion to complete** (motionQueueCount == 0)
  3. **Disable all OCx compare interrupts** (IEC0/1/2 clear OCx bits)
  4. **Reset TMR2 = 0** (atomic write: TMR2 = 0)
  5. **Clear all OCx compare values** (OCxR = 0, OCxRS = 0)
  6. **Re-enable OCx interrupts**
  7. **Resume motion processing**

**Implementation Pattern:**
```c
// In MOTION_Tasks or APP_Tasks (main loop)
#define TMR2_RESET_THRESHOLD  0xF0000000  // ~328 seconds, leaves 15.6s margin

static bool tmr2_reset_pending = false;

void MOTION_CheckTimerRollover(void) {
    uint32_t tmr_now = TMR2;
    
    if (tmr_now > TMR2_RESET_THRESHOLD && !tmr2_reset_pending) {
        tmr2_reset_pending = true;
        // Stop accepting new motion (signal to APP_Tasks)
    }
    
    if (tmr2_reset_pending && appData.motionQueueCount == 0) {
        // Safe to reset - no pending motion
        IEC0CLR = _IEC0_OC1IE_MASK | _IEC0_OC2IE_MASK;  // Disable OC interrupts
        IEC0CLR = _IEC0_OC3IE_MASK | _IEC0_OC4IE_MASK;
        
        TMR2 = 0;  // Reset timer
        
        // Clear all compare registers
        OC1R = 0; OC1RS = 0;
        OC2R = 0; OC2RS = 0;
        OC3R = 0; OC3RS = 0;
        OC4R = 0; OC4RS = 0;
        
        IFS0CLR = _IFS0_OC1IF_MASK | _IFS0_OC2IF_MASK;  // Clear flags
        IFS0CLR = _IFS0_OC3IF_MASK | _IFS0_OC4IF_MASK;
        
        IEC0SET = _IEC0_OC1IE_MASK | _IEC0_OC2IE_MASK;  // Re-enable
        IEC0SET = _IEC0_OC3IE_MASK | _IEC0_OC4IE_MASK;
        
        tmr2_reset_pending = false;  // Resume motion
    }
}
```

**Why This Works:**
- ✅ **Predictable reset point** - controlled by software, not hardware overflow
- ✅ **No lost steps** - only resets when motion queue is empty
- ✅ **Clean OCx state** - all compare registers cleared before resuming
- ✅ **Large safety margin** - 15.6 seconds to drain motion queue
- ✅ **Transparent to motion** - resumes normally after reset

**Alternative: Period Match Reset (Simpler)**
Set PR2 to a large value (e.g., 0xF0000000) and use timer period interrupt:
```c
// Timer initialization
PR2 = 0xF0000000;  // Period match at ~328 seconds
IPC2bits.T2IP = 1;  // Low priority
IEC0SET = _IEC0_T2IE_MASK;  // Enable period interrupt

// In TMR2 ISR
void __ISR(_TIMER_2_VECTOR, IPL1SOFT) TMR2Handler(void) {
    IFS0CLR = _IFS0_T2IF_MASK;
    // TMR2 automatically resets to 0 on period match
    // OCx values remain valid (relative to 0)
    // Motion continues seamlessly
}
```

**Recommendation:** Use **controlled reset approach** for better control over when reset occurs.

### Absolute Compare Mode
- All OCx compare registers use **ABSOLUTE timer values**, not relative offsets
- `OCxR` and `OCxRS` are set to specific TMR2 count values
- Compare values must always be scheduled **ahead of current TMR2 count**
- Example: If TMR2 = 1000, set OC1R = 1500 (not OC1R = TMR2 + 500)

### Dynamic Dominant Axis Tracking
- **Dominant axis** (highest step count) continuously tracks ahead of TMR2
- Dominant axis OCx pair is always scheduled for next pulse
- **Subordinate axes** scheduled on-demand only when Bresenham requires a step
- **Dominant axis can swap** mid-motion by changing which OCx tracks ahead

### Bresenham Integration  
- Bresenham algorithm runs in **state machine**, NOT in ISR
- ISRs are minimal: clear flag, schedule next pulse, exit
- Error term updates happen in main loop or state machine
- Subordinate axis scheduling based on error accumulation

### Single Instance Pattern in appData ✅
- **All major data structures** centralized in APP_DATA struct
- **No static module data** - clean separation of concerns  
- **Pass by reference** through function calls for explicit ownership
- **Work coordinates protected** by private static in kinematics module

### Professional G-Code Event System ✅
- **Event-driven architecture**: Clean `GCODE_GetNextEvent()` interface
- **Comprehensive G-code support**: G1, G2/G3, G4, M3/M5, M7/M9, G90/G91 commands
- **Abstraction layer respect**: No APP_DATA exposure, maintains clean boundaries
- **Multi-command tokenization**: "G90G1X10Y10F1000S200M3" → individual events
- **Zero memory allocation**: Deterministic processing for real-time systems
- **Utils module**: Professional string parsing with robust tokenization
- **Flow control**: 16-command circular buffer with "OK" withholding
- **Real-time characters**: Bypass tokenization for immediate '?!~^X' processing
- **GRBL compliance**: Full v1.1 protocol support with proper status reporting

### GRBL v1.1 Protocol Compliance
- **One "OK" per G-code line**: Send acknowledgment only after complete line is received and queued
- **Line-oriented buffering**: Accumulate bytes until `\n` or `\r` terminator before processing
- **Static variable pattern**: Use `static bool has_line_terminator` to persist state across polled function calls
  - Allows state to accumulate during byte reception
  - **CRITICAL**: Reset to `false` after processing to prevent stale state
- **Early exit pattern**: Check `!has_line_terminator` and break immediately to prevent premature processing
- **Control characters bypass buffering**: `?!~^X` process immediately without waiting for terminator
- **Flow control**: Withhold "OK" when command queue is full (GRBL behavior)
- **Real-time commands never get "OK"**: 
  - `?` → Status report only
  - `!` → Feed hold, silent
  - `~` → Resume, silent
  - `^X` → Reset + banner only
- **Empty lines ignored**: Whitespace-only input gets no response
- **Echo disabled**: GRBL doesn't echo commands by default

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
    
    // Minimal work: schedule next pulse using ABSOLUTE value
    uint32_t now = TMR2;
    OC1R = now + step_interval;  // ABSOLUTE timer count
    OC1RS = OC1R + pulse_width;   // ABSOLUTE timer count
    
    // Update step counter
    steps_completed++;
    
    // Exit quickly - Bresenham runs in state machine
}
```

### Compare Register Updates
```c
// CORRECT - Absolute values
uint32_t current_time = TMR2;
OC1R = current_time + 500;    // Pulse at TMR2 = current + 500
OC1RS = OC1R + 20;             // Pulse width = 20 ticks

// INCORRECT - Don't use relative offsets
OC1R = 500;  // WRONG - not absolute!
```

### Subordinate Axis Scheduling
```c
// Schedule subordinate axis only when needed
if (error_y >= delta_x) {
    uint32_t now = TMR2;
    OC2R = now + offset;       // ABSOLUTE value
    OC2RS = OC2R + pulse_width; // ABSOLUTE value
    error_y -= delta_x;
} else {
    // CRITICAL: Set equal values to STOP pulse generation
    // Prevents spurious pulses during arc and Bresenham execution
    OC2R = OC2RS;  // Equal values = no pulse
}
```

### Disabling Pulse Generation
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
- ❌ Use relative compare values (always use absolute TMR2 counts)
- ❌ Perform Bresenham calculations in ISR
- ❌ Schedule compare values behind current TMR2 count
- ❌ Use blocking delays in main loop (let APP_Tasks run freely)
- ❌ Allow TMR2 to overflow uncontrolled (implement rollover management)

### Always Do
- ✅ Schedule OCx registers with absolute timer values ahead of TMR2
- ✅ Clear interrupt flags immediately at ISR entry
- ✅ Keep ISRs minimal and fast
- ✅ Run Bresenham logic in state machine
- ✅ Track dominant axis continuously ahead of TMR2
- ✅ Monitor TMR2 for rollover and reset before overflow (see Timer Rollover Management)
- ✅ Ensure motion queue is empty before TMR2 reset
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

**Note:** Velocity profiling is not yet implemented but should be designed with the absolute compare mode architecture in mind.

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
  - TMR2/TMR3 in 32-bit mode (T32 = 1)
  - Prescaler: 1:4 (TCKPS = 2)
  - Timer Frequency: 12.5MHz
  - **Timer Resolution: 80ns per tick**
  - Free-running, no period match interrupts
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
- `srcs/motion/motion.c` - Master motion controller 🚧
- `srcs/motion/kinematics.c` - Physics calculations ✅
- `srcs/settings/settings.c` - Persistent GRBL settings with NVM flash ✅
- `incs/data_structures.h` - Unified data structures (no circular dependencies) ✅
- `incs/common.h` - Shared constants and enums
- `docs/plantuml/` - Architecture diagrams (includes tokenization flow)
- `README.md` - Complete architecture documentation

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

## Arc Interpolation Implementation Guide (PRIORITY: HIGH)

### Overview
Arc interpolation (G2/G3 commands) is **CRITICAL** for circular motion. Without this, the CNC cannot run most real programs. This is the **next priority** implementation task.

### Step 1: Understand Arc Math (5 minutes)
**G-code Format:**
- **G2**: Clockwise arc in current plane
- **G3**: Counter-clockwise arc in current plane
- **Parameters**:
  - `X Y Z`: End point coordinates (absolute or relative based on G90/G91)
  - `I J K`: Center offset from **start point** (always incremental)
  - Example: `G2 X10 Y0 I5 J0` → Arc from current pos to (10,0), center at (current.x+5, current.y+0)

**Arc Calculations:**
1. **Center point**: `center.x = start.x + I`, `center.y = start.y + J`, `center.z = start.z + K`
2. **Radius verification**: 
   - `r_start = sqrt(I² + J²)` (radius from start to center)
   - `r_end = sqrt((end.x - center.x)² + (end.y - center.y)²)` (radius from end to center)
   - If `|r_start - r_end| > tolerance`, send error: "ALARM:33 Arc radius error"
3. **Angles**:
   - `start_angle = atan2(start.y - center.y, start.x - center.x)`
   - `end_angle = atan2(end.y - center.y, end.x - center.x)`
   - `total_angle = end_angle - start_angle` (adjust for clockwise/counter-clockwise)
4. **Arc length**: `arc_length = radius × |total_angle|` (in radians)
5. **Segment count**: `segments = ceil(arc_length / mm_per_arc_segment)` (GRBL setting $27)

### Step 2: Add KINEMATICS_ArcMove() Function (30 minutes)
**File:** `srcs/motion/kinematics.c` and `incs/motion/kinematics.h`

**Function Signature:**
```c
// Add to kinematics.h
uint32_t KINEMATICS_ArcMove(
    CoordinatePoint start,           // Start position (mm)
    CoordinatePoint end,             // End position (mm)
    CoordinatePoint center,          // Arc center (absolute, mm)
    bool clockwise,                  // G2=true, G3=false
    uint8_t plane,                   // G17=XY, G18=XZ, G19=YZ
    float feedrate,                  // Feedrate (mm/min)
    MotionSegment* segment_buffer,   // Pre-allocated buffer
    uint32_t max_segments            // Buffer size limit
);
```

**Implementation Pattern:**
```c
uint32_t KINEMATICS_ArcMove(...) {
    GRBL_Settings* settings = SETTINGS_GetCurrent();
    
    // 1. Verify radius (error check)
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

### Step 3: Integrate Arc Event Processing in APP_Tasks (15 minutes)
**File:** `srcs/app.c` in APP_IDLE state, event processing switch

**Add Case Handler:**
```c
case GCODE_EVENT_ARC_MOVE:
{
    // Estimate segment count for buffer check
    float i = event.data.arcMove.i;
    float j = event.data.arcMove.j;
    float radius = sqrtf(i*i + j*j);
    float estimated_arc_length = 2.0f * M_PI * radius;  // Worst case: full circle
    GRBL_Settings* settings = SETTINGS_GetCurrent();
    uint32_t max_arc_segments = (uint32_t)ceilf(estimated_arc_length / settings->mm_per_arc_segment) + 1;
    
    // Check if motion queue has space for ALL arc segments
    if(appData.motionQueueCount + max_arc_segments < MAX_MOTION_SEGMENTS) {
        
        // Build start, end, center points
        CoordinatePoint start = {appData.currentX, appData.currentY, appData.currentZ, appData.currentA};
        CoordinatePoint end;
        
        if(appData.absoluteMode) {
            end.x = event.data.arcMove.x;
            end.y = event.data.arcMove.y;
            end.z = event.data.arcMove.z;
            end.a = event.data.arcMove.a;
        } else {
            end.x = appData.currentX + event.data.arcMove.x;
            end.y = appData.currentY + event.data.arcMove.y;
            end.z = appData.currentZ + event.data.arcMove.z;
            end.a = appData.currentA + event.data.arcMove.a;
        }
        
        // Center is ALWAYS incremental (I J K offsets from start point)
        CoordinatePoint center;
        center.x = start.x + event.data.arcMove.i;
        center.y = start.y + event.data.arcMove.j;
        center.z = start.z + event.data.arcMove.k;
        center.a = 0;  // A-axis doesn't participate in arc
        
        // Allocate temporary buffer for arc segments (on stack or in APP_DATA)
        MotionSegment arc_segments[64];  // Max 64 segments per arc (tune based on max arc size)
        
        // Generate arc segments
        uint32_t segment_count = KINEMATICS_ArcMove(
            start, end, center, 
            event.data.arcMove.clockwise,
            event.data.arcMove.plane,  // G17/G18/G19
            event.data.arcMove.feedrate,
            arc_segments, 
            64  // Max segments
        );
        
        if(segment_count == 0) {
            // Arc generation failed (radius error)
            appData.alarmCode = 33;  // GRBL alarm code for arc error
            appData.state = APP_ALARM;
            break;
        }
        
        // Add ALL arc segments to motion queue atomically
        for(uint32_t i = 0; i < segment_count; i++) {
            memcpy(&appData.motionQueue[appData.motionQueueHead], 
                   &arc_segments[i], sizeof(MotionSegment));
            appData.motionQueueHead = (appData.motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
            appData.motionQueueCount++;
        }
        
        // Update current position to arc end point
        appData.currentX = end.x;
        appData.currentY = end.y;
        appData.currentZ = end.z;
        appData.currentA = end.a;
        
    } else {
        // Motion queue full - event stays in G-code queue (flow control)
    }
    break;
}
```

### Step 4: Add Arc Segment Setting to GRBL Settings (5 minutes)
**File:** `incs/settings/settings.h` and `srcs/settings/settings.c`

**Add to GRBL_Settings struct:**
```c
typedef struct {
    // ... existing settings ...
    
    float mm_per_arc_segment;    // $27 - Arc segment length (default 0.1mm)
} GRBL_Settings;
```

**Initialize in SETTINGS_RestoreDefaults():**
```c
settings->mm_per_arc_segment = 0.1f;  // GRBL default: 0.1mm per segment
```

### Step 5: Testing Arc Interpolation (10 minutes)
**Send Test G-code via UART:**
```gcode
G17           ; Select XY plane
G90           ; Absolute positioning
G0 X0 Y0      ; Rapid to origin
G1 F500       ; Set feedrate to 500 mm/min
G2 X10 Y0 I5 J0   ; Clockwise arc from (0,0) to (10,0), center at (5,0), radius=5mm
```

**Expected Behavior:**
- Arc broken into ~157 segments (π×10mm / 0.1mm per segment = 31.4 segments for semicircle)
- Smooth circular motion in XY plane
- Final position: (10.0, 0.0)

**Verify with Status Query:**
```
?   ; Send status query
<Idle|MPos:10.000,0.000,0.000|WPos:10.000,0.000,0.000|FS:500,0>
```

### Common Pitfalls to Avoid
1. **DON'T forget**: I J K are **offsets from start point**, not absolute coordinates
2. **DON'T mix**: Center calculation must use start point, not current position
3. **DON'T skip**: Radius verification prevents malformed arcs
4. **DON'T stream**: Generate ALL segments before sending "OK" to host
5. **DON'T use degrees**: All angles in radians for sin/cos/atan2
6. **DO use FPU**: Hardware accelerated on PIC32MZ (-mhard-float flag already set)

### Performance Considerations
- **FPU operations**: sin/cos/sqrt are ~10-20 cycles with hardware FPU
- **Arc generation overhead**: ~50μs per segment @ 200MHz CPU (acceptable during planning)
- **Motion queue**: Pre-generate all segments, then execute as normal linear moves
- **Memory**: Stack-allocate arc_segments[64] = 64 × sizeof(MotionSegment) = ~10KB max

**Total Implementation Time: ~1 hour**

## Development Workflow
1. Build: `make` or `make all` (clean + build)
2. Flash: MikroE bootloader
3. Clean: `make clean`

## When Suggesting Code
- Always use absolute timer values for OCx registers
- Never suggest stopping TMR2 during motion
- Keep ISR implementations minimal
- Use state machines for complex logic, not interrupts
- Respect the dominant/subordinate axis architecture
- Consider dynamic axis swapping capability
- **For arcs**: Use FPU, verify radius, generate all segments atomically

## Questions to Ask
When uncertain about implementation details:
- "Is this the dominant or subordinate axis?"
- "Should this logic run in ISR or state machine?"
- "Are we using absolute or relative timer values?"
- "Does this arc need radius verification?"
- "How many segments will this arc generate?"
- "Does this require scheduling ahead of TMR2?"

## Questions to Ask
When uncertain about implementation details:
- "Is this the dominant or subordinate axis?"
- "Should this logic run in ISR or state machine?"
- "Are we using absolute or relative timer values?"
- "Does this require scheduling ahead of TMR2?"
