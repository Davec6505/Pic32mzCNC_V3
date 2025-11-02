# GitHub Copilot Instructions for Pic32mzCNC_V3

## Project Overview
This is a CNC motion control system for PIC32MZ microcontrollers using hardware timers and Bresenham interpolation for precise multi-axis stepper motor control.

## 🚀 Current Implementation Status (November 2025)
- ✅ **Professional event-driven G-code system** with clean architecture
- ✅ **Event queue implementation** respecting APP_DATA abstraction layer
- ✅ **Comprehensive G-code support**: G1, G2/G3, G4, M3/M5, M7/M9, G90/G91
- ✅ **Core architecture implemented** with absolute compare mode
- ✅ **Single instance pattern in appData** for clean separation  
- ✅ **Utils module complete** with professional string tokenization
- ✅ **Multi-command line support** for complex G-code like "G90G1X10Y10F1000"
- ✅ **16-command circular buffer** with flow control and overflow protection
- ✅ **Kinematics module complete** with physics calculations
- ✅ **Stepper module complete** with hardware abstraction
- 🚧 **Motion controller in progress** - Bresenham state machine
- ✅ **Project compiles successfully** with XC32 compiler

## Core Architecture Principles

### Timer Architecture
- **TMR2 runs continuously** - NEVER stop or reset the timer during operation
- **32-bit timer** (TMR2:TMR3 pair) for extended range without overflow
- **Hardware Configuration:**
  - PBCLK3 = 50MHz (peripheral bus clock)
  - Prescaler = 1:4 (TCKPS = 2)
  - Timer Frequency = 12.5MHz (50MHz / 4)
  - Timer Resolution = **80ns per tick** (1 / 12.5MHz)
  - Period Register (PR2) = 124 (unused for free-running mode)
- **No timer interrupts used** - timer provides stable time base only
- **Step timing** controlled entirely by OC module compare interrupts

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
- **Line-oriented buffering**: Accumulate bytes until `\n` terminator before processing
- **Control characters bypass buffering**: `?!~^X` process immediately without waiting for `\n`
- **Flow control**: Withhold "OK" when command queue is full (GRBL behavior)
- **Real-time commands never get "OK"**: 
  - `?` → Status report only
  - `!` → Feed hold, silent
  - `~` → Resume, silent
  - `^X` → Reset + banner only
- **Empty lines ignored**: Whitespace-only input gets no response
- **Echo disabled**: GRBL doesn't echo commands by default### Single Instance Pattern in appData ✅
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
        
        // ✅ CRITICAL: Check for line terminator OR control character
        bool has_line_terminator = false;
        for(uint32_t i = 0; i < nBytesRead; i++) {
            if(rxBuffer[i] == '\n') {
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

        // ✅ CRITICAL: Check for actual G-code content BEFORE processing
        // Must be in same scope as has_line_terminator to persist across re-entry
        bool has_gcode = false;
        for(uint32_t i = 0; i < nBytesRead; i++) {
            if(rxBuffer[i] != '\r' && rxBuffer[i] != '\n' && 
                rxBuffer[i] != ' ' && rxBuffer[i] != '\t' && rxBuffer[i] != 0) {
                has_gcode = true;
                break;
            }
        }

        if(control_char_found){              
            // Fall through to GCODE_STATE_CONTROL_CHAR immediately
        } else if(has_line_terminator && has_gcode){
            // ✅ Complete G-code line with actual content - process it
            cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, nBytesRead, cmdQueue);
        
            // ✅ Send ONE "OK" per complete G-code line (GRBL v1.1 protocol)
            UART2_Write((uint8_t*)"OK\r\n", 4);
            
            // Clear buffer for next line
            nBytesRead = 0; 
            memset(rxBuffer, 0, sizeof(rxBuffer));
            gcodeData.state = GCODE_STATE_IDLE;
            break;
        } else if(has_line_terminator && !has_gcode){
            // ✅ Line terminator but no G-code (empty line or whitespace only)
            // Clear buffer, don't send "OK", stay in IDLE
            nBytesRead = 0; 
            memset(rxBuffer, 0, sizeof(rxBuffer));
            gcodeData.state = GCODE_STATE_IDLE;
            break;
        } else {
            // ✅ CRITICAL: Incomplete line - wait for more data
            // Don't clear buffer, don't send "OK", just exit and accumulate more bytes
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
- ❌ Stop or reset TMR2 during operation
- ❌ Use relative compare values (always use absolute TMR2 counts)
- ❌ Perform Bresenham calculations in ISR
- ❌ Schedule compare values behind current TMR2 count
- ❌ Use blocking delays in main loop (let APP_Tasks run freely)

### Always Do
- ✅ Schedule OCx registers with absolute timer values ahead of TMR2
- ✅ Clear interrupt flags immediately at ISR entry
- ✅ Keep ISRs minimal and fast
- ✅ Run Bresenham logic in state machine
- ✅ Track dominant axis continuously ahead of TMR2
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
- **Timer Configuration:**
  - TMR2/TMR3 in 32-bit mode (T32 = 1)
  - Prescaler: 1:4 (TCKPS = 2)
  - Timer Frequency: 12.5MHz
  - **Timer Resolution: 80ns per tick**
  - Free-running, no period match interrupts
- **Output Compare Modules:** OC1 (X), OC2 (Y), OC3 (Z), OC4 (A)

## File Organization
- `srcs/main.c` - Entry point, main loop calls APP_Tasks()
- `srcs/app.c` - Application state machine (single instance pattern)
- `srcs/gcode/gcode_parser.c` - G-code parsing & GRBL protocol ✅
- `srcs/gcode/utils.c` - Professional string tokenization utilities ✅
- `srcs/motion/stepper.c` - Hardware abstraction layer ✅  
- `srcs/motion/motion.c` - Master motion controller 🚧
- `srcs/motion/kinematics.c` - Physics calculations ✅
- `incs/common.h` - Shared constants and enums
- `docs/plantuml/` - Architecture diagrams (includes tokenization flow)
- `README.md` - Complete architecture documentation

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

## Questions to Ask
When uncertain about implementation details:
- "Is this the dominant or subordinate axis?"
- "Should this logic run in ISR or state machine?"
- "Are we using absolute or relative timer values?"
- "Does this require scheduling ahead of TMR2?"

## Questions to Ask
When uncertain about implementation details:
- "Is this the dominant or subordinate axis?"
- "Should this logic run in ISR or state machine?"
- "Are we using absolute or relative timer values?"
- "Does this require scheduling ahead of TMR2?"
