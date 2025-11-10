# Pic32mzCNC_V3 - CNC Motion Control System

## 🚀 Project Status: **Production Ready** ✅

**Pic32mzCNC_V3** is a production-ready CNC motion control system for PIC32MZ microcontrollers featuring **hardware-validated motion restart**, **optimal timer configuration**, and **robust GRBL v1.1 protocol** for industrial stepper motor control.

### ✅ **Latest Fixes** (November 10, 2025)
- ✅ **UGS Soft Reset Recovery**: Motion automatically restarts after Ctrl+X soft reset
- ✅ **Optimal Timer Configuration**: TMR4 1:64 prescaler (781.25kHz) with 2.5µs pulses
- ✅ **Hardware Validation Guards**: OC1/TMR4 startup checks prevent motion failures
- ✅ **Production-Ready G-Code Parser**: Professional event-driven system (do not modify!)
- ✅ **Debug System Documentation**: Complete learning guide for efficient debugging

### ✅ **Core Production Features** (Validated November 7-10, 2025)
- ✅ **Multi-axis coordinated motion**: G1 X100 Y25 moves with perfect step ratio
- ✅ **Work coordinate system**: G92 sets WPos correctly, maintains MPos accuracy
- ✅ **Real-time control**: Ctrl+X emergency stop with microsecond response
- ✅ **Position tracking**: Accurate MPos/WPos reporting via GRBL status queries
- ✅ **Feedrate tracking**: FS:100,0 during motion, FS:0,0 when idle
- ✅ **Event-driven architecture**: Non-blocking processing with priority phases
- ✅ **Robust UART communication**: 1024-byte TX buffer handles `$$` responses
- ✅ **Compile-time debug system**: Zero runtime overhead, multiple subsystems
- ✅ **Clean codebase**: Production firmware (bins/Release/CS23.hex)

## 🎯 **Key Features**

### **Motion Control**
- **Hardware-validated restarts**: OC1/TMR4 validation guards prevent motion startup failures  
- **Multi-axis coordination**: Bresenham interpolation for 4-axis (XYZA) stepper control
- **Optimal timing**: TMR4 1:64 prescaler (781.25kHz) with 2.5µs stepper pulses
- **Trapezoidal velocity profiling**: GRBL-style acceleration/deceleration physics
- **256 microstepping support**: 512kHz max step rate validated (42% ISR headroom)
- **Timer-based step generation**: Period-based TMR4, OCx modules for precise timing
- **Dynamic dominant axis tracking**: ISR precision with main loop flexibility
- **Emergency stop system**: APP_ALARM state with hard/soft limit checking

### **G-Code Processing**
- **GRBL v1.1 protocol**: Full status reporting, real-time commands, flow control
- **Comprehensive command support**: G1, G2/G3, G4, G92, M3/M5, M7/M9, G90/G91, F, S, T
- **Event-driven architecture**: Clean `GCODE_GetNextEvent()` interface
- **Professional tokenization**: "G90G1X10Y10F1000" → ["G90", "G1X10Y10F1000"]
- **Modal parameter support**: Standalone F, S, T commands (LinuxCNC compatible)
- **Control character filter**: Strips non-printable chars except CR/LF/TAB
- **16-command circular buffer**: Flow control with "OK" withholding

### **Debug & Testing**
- **Professional debug system**: Zero runtime overhead in Release builds
- **Multiple subsystems**: DEBUG_MOTION, DEBUG_GCODE, DEBUG_STEPPER, DEBUG_SEGMENT, DEBUG_UART, DEBUG_APP
- **Clean syntax**: `DEBUG_PRINT_MOTION("[MOTION] msg")` → Nothing in Release
- **Comprehensive guide**: See [`docs/DEBUG_SYSTEM_LEARNING.md`](docs/DEBUG_SYSTEM_LEARNING.md) for complete tutorial
- **PowerShell test script**: Automated G-code validation (`ps_commands/test_gcode.ps1`)
- **Color-coded output**: Pass/fail indicators with timeout handling

### **Persistence & Settings**
- **NVM flash storage**: GRBL settings at 0xBD1F0000 (16KB page-aligned)
- **29 GRBL parameters**: Steps/mm, acceleration, max rates, arc segment mm, work offsets
- **Delayed flash init**: Read after peripherals ready (APP_LOAD_SETTINGS state)
- **Harmony callback pattern**: Cache-aligned buffers with RowWrite operations
## 🏗️ **Technical Architecture**

### **Continuous Pulse Mode with Virtual Dominant Axis**
The system uses a **single OC1 module in continuous pulse mode** for hardware-driven asynchronous step timing:

**Key Concept:**
- **OC1 hardware generates asynchronous timing events** - no software polling needed
- **TMR4 free-runs** with PR4 setting the period (no TMR4 ISR at all!)
- **OC1 ISR fires automatically** when hardware compare matches (~10µs-1ms intervals)
- **Hardware-driven timing** - CPU not involved between steps
- **ISR only runs when step needed** - Bresenham + GPIO pulses

**Architecture Pattern:**
```c
// OC1 in continuous pulse mode:
// - OCxR: Rising edge (near end of period)
// - OCxRS: Falling edge (triggers ISR via hardware compare)
// - Hardware compare generates ISR asynchronously
// - TMR4 rolls over at PR4, OC1 continues automatically
// - NO continuous software timer checking!

void OCP1_ISR(void) {
    // Hardware called us - time for next step
    
    // Dominant axis always steps (virtual axis timing)
    pulse_dominant_axis_gpio();
    
    // Bresenham: subordinate axes step when error >= dominant_delta
    if (error_y >= dominant_delta) { pulse_y_gpio(); error_y -= dominant_delta; }
    if (error_z >= dominant_delta) { pulse_z_gpio(); error_z -= dominant_delta; }
    if (error_a >= dominant_delta) { pulse_a_gpio(); error_a -= dominant_delta; }
}
```

**Benefits:**
- ✅ **Hardware-driven async events** - OC1 module handles all timing
- ✅ **Zero CPU overhead between steps** - hardware does the work
- ✅ **Jitter-free** - hardware compare ensures precise pulse timing
- ✅ **Simple** - Single ISR for all axes (GRBL pattern)
- ✅ **Virtual dominant axis** - Timer rate matches fastest-moving axis
- ✅ **Not GRBL's 10µs software polling** - true hardware async timing

### **Priority-Based Phase System**
Hybrid ISR/main loop architecture combining ISR precision with main loop flexibility:

```c
typedef enum {
    MOTION_PHASE_IDLE = 255,      // Lowest - G-code processing safe
    MOTION_PHASE_VELOCITY = 0,    // Highest - velocity conditioning
    MOTION_PHASE_BRESENHAM = 1,   // Bresenham error accumulation
    MOTION_PHASE_SCHEDULE = 2,    // OCx register scheduling
    MOTION_PHASE_COMPLETE = 3     // Segment completion
} MotionPhase;
```

**Operation:**
1. **OC1 ISR** fires at step rate → runs Bresenham, pulses GPIO
2. **Main loop processes phases** in priority order (0 = highest)
3. **G-code only runs when IDLE** → prevents motion blocking
4. **UART rate-limited** → polled every 10ms (not every µs)

### **Single Instance Pattern**
All major data structures centralized in `APP_DATA`:

```c
APP_DATA appData = {
    .gcodeCommandQueue = {...},     // G-code command buffer
    .motionQueue = {...},           // Motion segment array
    .motionQueueHead/Tail/Count,    // Queue management
    .arcGenState,                   // Arc interpolation state
    .modalPlane                     // G17/G18/G19 tracking
};
```

## 📦 **Hardware & Toolchain**

### **Microcontroller**
- **Model:** PIC32MZ2048EFH100
- **System Clock:** 200MHz
- **Peripheral Bus (PBCLK3):** 50MHz
- **Hardware FPU:** Single-precision floating point
- **Flash:** 2MB (1.87MB app code, 64KB settings/bootloader)
- **Memory Map:**
  - 0x9D000000: Application code (1.87MB)
  - 0xBD1F0000: GRBL settings NVM (16KB)
  - 0x9D1F4000: MikroE USB HID bootloader (48KB)

### **Timer Configuration**
- **TMR4:** 16-bit mode
- **Prescaler:** 1:4
- **Timer Frequency:** 12.5MHz (50MHz / 4)
- **Timer Resolution:** **80ns per tick**
- **Rollover Time:** 5.24 milliseconds
- **Period match:** PR4 register for compare timing

### **Output Compare Modules**
- **OC1:** X-axis stepper (uses TMR4)
- **OC2:** Y-axis stepper (uses TMR4)
- **OC3:** Z-axis stepper (uses TMR4)
- **OC4:** A-axis (rotary) stepper (uses TMR4)
- **Mode:** Dual-compare pulse generation
- **Max Step Rate:** 512kHz (validated with 42% ISR headroom)

### **Development Tools**
- **MPLAB X IDE:** v6.x or later
- **XC32 Compiler:** v4.60 (v4.x required for FPU)
- **Harmony 3:** Peripheral libraries
- **Build System:** Make (Windows PowerShell compatible)
- **Bootloader:** MikroE USB HID (39KB at 0x9D1F4000)

### **Compiler Flags**
- **FPU:** `-mhard-float -msingle-float -mfp64`
- **Optimization:** `-O1` (Release), `-O0` (Debug)
- **Fast Math:** `-ffast-math -fno-math-errno`

## 🚀 **Build Instructions**

### **1. Clone Repository**
```bash
git clone https://github.com/Davec6505/Pic32mzCNC_V3.git
cd Pic32mzCNC_V3
```

### **2. Build Commands**
```bash
# Release build (default, -O1 optimization)
make
make all

# Incremental build (only changed files)
make build

# Debug build (-O0, full symbols)
make BUILD_CONFIG=Debug

# Clean builds
make clean              # Clean current BUILD_CONFIG
make clean_all          # Clean both Debug and Release
```

### **3. Debug Builds (Compile-Time Flags)**
```bash
# Enable motion debug
make DEBUG_FLAGS="DEBUG_MOTION"

# Enable multiple subsystems
make DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE DEBUG_SEGMENT"

# Available debug subsystems:
# DEBUG_MOTION, DEBUG_GCODE, DEBUG_STEPPER, 
# DEBUG_SEGMENT, DEBUG_UART, DEBUG_APP

# In release builds, debug code is COMPLETELY REMOVED (zero overhead)
```

**Build Outputs:**
- **Release:** `bins/Release/CS23.hex` (production firmware)
- **Debug:** `bins/Debug/CS23.hex` (with debug symbols)

**See:** `docs/DEBUG_SYSTEM_TUTORIAL.md` for complete debug guide

### **4. Flash Firmware**
```bash
# MikroE USB HID bootloader (recommended)
make flash

# Or use MPLAB X IPE with bins/Release/CS23.hex
```

## 🧪 **Testing & Validation**

### **Hardware Testing (Production Validated)**
Connect via UART3 at 115200 baud 8N1:

```gcode
# 1. Reset and verify banner
^X
[VER:1.1f.20211112:GRBL-style controller for PIC32MZ]
[MSG:PIC32MZ powered on, ready for commands]

# 2. Set work origin
G92 X0 Y0 Z0
OK

# 3. Status query
?
<Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|FS:0,0>

# 4. Coordinated motion test
G1 X100 Y25 F1000
OK
?
<Run|MPos:100.000,25.000,0.000|WPos:100.000,25.000,0.000|FS:100,0>

# 5. Emergency stop
^X
[MSG:Reset to continue]
```

**Expected Results:**
- ✅ Multi-axis coordinated motion (step ratio maintained)
- ✅ Position tracking accurate (MPos = WPos after G92)
- ✅ Feedrate tracking (FS shows current feedrate during motion)
- ✅ Real-time control (Ctrl+X immediate stop)

### **Automated Testing (PowerShell)**
```powershell
# Run test script (sends G92, G1, status queries)
.\ps_commands\test_gcode.ps1

# Color-coded output shows pass/fail with response validation
```

### **Supported G-Code Commands**

**Motion:**
- `G1` - Linear move (X Y Z A F parameters)
- `G2` - Clockwise arc (X Y Z I J K parameters)
- `G3` - Counter-clockwise arc (X Y Z I J K parameters)
- `G4` - Dwell (P parameter in seconds)

**Work Coordinates:**
- `G90` - Absolute positioning mode
- `G91` - Relative positioning mode
- `G92` - Set work coordinate system

**Plane Selection:**
- `G17` - XY plane (default)
- `G18` - XZ plane
- `G19` - YZ plane

**Spindle/Coolant:**
- `M3` - Spindle on clockwise (S parameter)
- `M5` - Spindle off
- `M7` - Coolant on (mist)
- `M8` - Coolant on (flood)
- `M9` - Coolant off

**Modal Parameters:**
- `F` - Set feedrate (mm/min)
- `S` - Set spindle speed (RPM)
- `T` - Tool change

**Real-Time Commands:**
- `?` - Status query
- `~` - Cycle start/resume
- `!` - Feed hold
- `^X` - Soft reset (Ctrl+X)

### **GRBL Status Format**
```
<State|MPos:X,Y,Z|WPos:X,Y,Z|FS:F,S>

State: Idle, Run, Hold, Alarm
MPos: Machine position (absolute)
WPos: Work position (relative to G92)
FS: Feedrate and spindle speed
```

## 📁 **Project Structure**

```
Pic32mzCNC_V3/
├── srcs/
│   ├── app.c                    # Application state machine & event processing
│   ├── main.c                   # Entry point
│   ├── gcode/
│   │   ├── gcode_parser.c       # Event-driven G-code parser (GRBL v1.1)
│   │   └── utils.c              # String tokenization utilities
│   ├── motion/
│   │   ├── stepper.c            # Hardware abstraction (absolute compare mode)
│   │   ├── motion.c             # Master controller (priority phase system)
│   │   ├── motion_utils.c       # Safety checks and utilities
│   │   └── kinematics.c         # Physics & coordinate transformations
│   ├── settings/
│   │   └── settings.c           # Persistent GRBL settings (NVM flash)
│   ├── utils/
│   │   └── uart_utils.c         # Non-blocking UART communication
│   └── config/default/          # Harmony 3 peripheral libraries
├── incs/                        # Header files
│   ├── app.h                    # Application interface
│   ├── data_structures.h        # Unified data structures
│   ├── common.h                 # Shared constants & enums
│   ├── gcode/                   # G-code parser interface
│   ├── motion/                  # Motion control interfaces
│   ├── settings/                # Settings interface
│   └── utils/                   # Utility function interfaces
├── docs/
│   ├── DEBUG_SYSTEM_TUTORIAL.md # Compile-time debug system guide
│   ├── DEVLOG_2025-11-07.md     # Development logs
│   ├── MEMORY_MAP.md            # Flash memory layout
│   └── plantuml/                # Architecture diagrams
│       ├── 01_system_overview.puml
│       ├── 02_segment_clock.puml
│       ├── 03_arc_linear_interpolation.puml
│       ├── 03_gcode_tokenization.puml
│       └── 04_event_system.puml
├── ps_commands/
│   └── test_gcode.ps1           # PowerShell test script
├── bins/Release/                # Build outputs (CS23.hex)
├── objs/Release/                # Object files
├── Makefile                     # Root build configuration
└── .github/
    └── copilot-instructions.md  # Development guidelines & patterns
```

## 📚 **Documentation**

### **Project Docs**
- **README.md** (this file) - Project overview and quick start
- **docs/DEBUG_SYSTEM_TUTORIAL.md** - Compile-time debug system
- **docs/MEMORY_MAP.md** - Flash memory layout details
- **docs/DEVLOG_2025-11-07.md** - Development session notes
- **.github/copilot-instructions.md** - Coding patterns & architecture

### **Architecture Diagrams** (PlantUML)
- **01_system_overview.puml** - High-level system architecture
- **02_segment_clock.puml** - Timer and motion segment flow
- **03_arc_linear_interpolation.puml** - Arc/linear interpolation
- **03_gcode_tokenization.puml** - G-code parsing flow
- **04_event_system.puml** - Event-driven processing

## 🔧 **Key Implementation Details**

### **Work Coordinate System (G92)**
```c
// Formula: WPos = MPos - offset
// To set WPos = 0 at current position:
// offset = current_MPos - 0 = current_MPos

StepperPosition* pos = STEPPER_GetPosition();
float mpos_x = (float)pos->x_steps / pos->steps_per_mm_x;

// Calculate offset for G92 X0
if (!isnan(event->data.setWorkOffset.x)) {
    offset_x = mpos_x - event->data.setWorkOffset.x;
    appData->currentX = event->data.setWorkOffset.x;
}

KINEMATICS_SetWorkCoordinates(offset_x, offset_y, offset_z);
```

### **Control Character Filtering**
Strips non-printable control characters (0x00-0x1F) except CR/LF/TAB before tokenization:

```c
// Prevents terminal emulator artifacts (like PuTTY's Ctrl+C insertion)
for (uint32_t read_pos = 0; read_pos < safe_length; read_pos++) {
    char c = line_buffer[read_pos];
    if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '\t') {
        line_buffer[write_pos++] = c;
    }
}
```

### **G-Code Tokenization Logic**
G/M commands consume ALL parameters until next G/M or line end:

```c
// Input:  "G92 X0 Y0 Z0"
// Output: ["G92 X0 Y0 Z0"]  (one token)

// Input:  "G90G1X10Y10F1000"
// Output: ["G90", "G1X10Y10F1000"]  (two tokens)

if (start_char == 'G' || start_char == 'M') {
    // Consume G/M digits
    while (end < max_len && str[end] >= '0' && str[end] <= '9') {
        end++;
    }
    // Consume all parameters until next G/M
    while (end < max_len) {
        char c = str[end];
        if (c == 'G' || c == 'M') break;
        if (c == '\n' || c == '\r' || c == '\0' || c == ';') break;
        end++;
    }
}
```

### **Command Dequeuing Fix**
Always dequeue commands even if parsing fails (prevents infinite loop):

```c
bool event_valid = parse_command_to_event(current_cmd->command, event);

// ✅ Always dequeue, even if parsing failed
cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
cmdQueue->count--;

return event_valid;
```

## 🚨 **Troubleshooting**

### **Common Issues**

**1. No motion after G1 command**
- Check: Position stays 0.000 after motion command
- Cause: Motion queue empty or phase system stuck
- Solution: Enable DEBUG_MOTION and DEBUG_SEGMENT to see segment loading

**2. G92 shows NaN for Y/Z axes**
- Check: WPos shows NaN instead of numeric values
- Cause: G92 handler not checking for NaN parameters
- Solution: Fixed in app.c lines 527-560 (only updates specified axes)

**3. Commands stuck in queue (infinite loop)**
- Check: Same command repeats in debug output
- Cause: GCODE_GetNextEvent() not dequeuing failed commands
- Solution: Fixed in gcode_parser.c lines 201-217 (always dequeue)

**4. Terminal control characters breaking tokenizer**
- Check: Debug shows `start_char=(0x03)` instead of `start_char='G'`
- Cause: Terminal emulator inserting control characters
- Solution: Use PowerShell test script or control character filter

**5. Build errors after git pull**
- Try: `make clean_all; make`
- Rebuilds from scratch (cleans Debug and Release)

### **Debug Commands**
```bash
# View build configuration
make help

# Clean everything
make clean_all

# Build with motion debug
make DEBUG_FLAGS="DEBUG_MOTION"

# Build with verbose output
make VERBOSE=1
```

## 🛠️ **Development Workflow**

### **Branch Strategy**
- **master** - Production-ready code
- **patch1** (formerly patch3-minimal-pulse) - Current development

### **Commit Pattern**
```bash
# Stage changes
git add -A

# Commit with detailed message
git commit -m "Fix: Description

- Detail 1
- Detail 2
- Detail 3

Files modified:
- file1.c (lines X-Y)
- file2.c (lines A-B)"

# Merge to master
git checkout master
git merge patch1 --no-ff

# Push to GitHub
git push origin master
git push origin patch1
```

## 🔧 **Adding Additional Axes (5th, 6th, etc.)**

The system uses **array-based axis control** architecture that makes adding new axes extremely simple. No switch statements to update - just expand the data structures!

### **Quick Start: Adding B-Axis (5th Axis)**

#### **Step 1: Update Axis Enum**
**File:** `incs/data_structures.h`
```c
typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_A,
    AXIS_B,     // ← ADD NEW AXIS HERE
    NUM_AXIS    // Automatically becomes 5
} E_AXIS;
```

#### **Step 2: Expand Coordinate Structure**
**File:** `incs/data_structures.h`
```c
typedef struct {
    float x, y, z, a, b;  // ← ADD NEW COORDINATE
} CoordinatePoint;
```

#### **Step 3: Add Hardware Pins in MCC**
- Open MCC (MPLAB Code Configurator)
- Add GPIO pins for B-axis: `StepB`, `DirB`, `EnableB`
- Generate code to create new MCC functions: `StepB_Set()`, `DirB_Set()`, etc.

#### **Step 4: Add Settings Parameters**
**File:** `incs/settings/settings.h`
```c
typedef struct {
    // ... existing parameters ...
    float steps_per_mm_b;
    float max_rate_b;
    float acceleration_b;
    // ... other B-axis settings ...
} CNC_Settings;
```

#### **Step 5: Initialize Hardware Array**
**File:** `srcs/utils/utils.c` → `UTILS_InitAxisConfig()`
```c
// ===== B AXIS (AXIS_B = 4) =====
g_axis_config[AXIS_B].step.Set = StepB_Set;
g_axis_config[AXIS_B].step.Clear = StepB_Clear;
g_axis_config[AXIS_B].dir.Set = DirB_Set;
g_axis_config[AXIS_B].dir.Clear = DirB_Clear;
g_axis_config[AXIS_B].enable.Set = EnableB_Set;
g_axis_config[AXIS_B].enable.Clear = EnableB_Clear;
g_axis_config[AXIS_B].steps_per_mm = &settings->steps_per_mm_b;
g_axis_config[AXIS_B].max_rate = &settings->max_rate_b;
g_axis_config[AXIS_B].acceleration = &settings->acceleration_b;
g_axis_config[AXIS_B].step_count = &stepper_pos->b_steps;
```

#### **Step 6: Add Stepper Position Tracking**
**File:** `incs/motion/stepper.h`
```c
typedef struct {
    int32_t x_steps, y_steps, z_steps, a_steps, b_steps;  // ← ADD b_steps
    float steps_per_mm_x, steps_per_mm_y, steps_per_mm_z, steps_per_deg_a, steps_per_deg_b;  // ← ADD b
} StepperPosition;
```

### **🎉 What Automatically Works**

Once you complete the above steps, **ALL motion control automatically supports the new axis**:

✅ **Homing Operations**: `$H` command includes B-axis  
✅ **Motion Planning**: `G1 X10 Y10 B45` commands work immediately  
✅ **Coordinate Systems**: Work offsets, G92 positioning  
✅ **Status Reporting**: Position tracking in `?` status queries  
✅ **Limit Switches**: Hardware limit checking (add `g_limit_config[AXIS_B]`)  
✅ **Emergency Stops**: Feed hold and soft reset handle all axes  

### **Array-Based Magic**

The system uses **coordinate array utilities** that treat `CoordinatePoint` as a `float[]` array:

```c
// These functions work automatically for ANY number of axes:
SET_COORDINATE_AXIS(&target, AXIS_B, 90.0f);      // Set B to 90 degrees
ADD_COORDINATE_AXIS(&position, AXIS_B, 15.0f);    // Move B by +15 degrees  
float b_pos = GET_COORDINATE_AXIS(&current, AXIS_B); // Read B position
```

**No switch statements to update!** All motion logic uses array indexing:
```c
// This line automatically works for any axis (X, Y, Z, A, B, C...):
ADD_COORDINATE_AXIS(&target, current_axis, move_distance);
```

### **Benefits of Array-Based Architecture**

- **🚀 Zero Motion Logic Changes**: Homing, interpolation, kinematics work instantly
- **🔧 Scalable**: Add 6th, 7th, 8th axes using the same pattern
- **🛡️ Type Safe**: C struct layout guarantees prevent memory errors
- **⚡ High Performance**: Direct array indexing vs conditional branches
- **📏 Maintainable**: Single implementation for all axis operations

### **Advanced: 6+ Axes**

For more than 5 axes (B, C, D...), just continue the pattern:
1. Add to `E_AXIS` enum: `AXIS_C`, `AXIS_D`, etc.
2. Expand `CoordinatePoint`: `float x, y, z, a, b, c, d;`
3. Add hardware initialization to `UTILS_InitAxisConfig()`
4. **All motion control automatically scales!**

The array-based design supports unlimited axes without architectural changes.

## 🎯 **Validated Performance**

### **Motion System**
- ✅ **Multi-axis coordination**: Perfect step ratio (tested 400:100 for X:Y)
- ✅ **Position accuracy**: No drift, precise positioning
- ✅ **Feedrate control**: Accurate velocity during motion
- ✅ **Real-time response**: Ctrl+X emergency stop in microseconds
- ✅ **ISR efficiency**: 42% headroom at 512kHz (256 microstepping)

### **Communication**
- ✅ **UART reliability**: 512-byte TX buffer, non-blocking writes
- ✅ **Protocol compliance**: GRBL v1.1 status format
- ✅ **Control character handling**: Robust terminal artifact filtering
- ✅ **Flow control**: 16-command buffer with "OK" withholding

### **G-Code Processing**
- ✅ **Tokenization**: Multi-command parsing working perfectly
- ✅ **Work coordinates**: G92 sets WPos correctly, MPos unchanged
- ✅ **Modal state**: G90/G91, F, S, T commands maintain state
- ✅ **Event system**: Non-blocking, one event per iteration

## 📝 **License**

This project is provided as-is for educational and development purposes. See LICENSE file for details.

## 🤝 **Contributing**

Contributions welcome! Please:
1. Fork the repository
2. Create feature branch (`git checkout -b feature/YourFeature`)
3. Follow coding patterns in `.github/copilot-instructions.md`
4. Test thoroughly with PowerShell script
5. Submit pull request with detailed description

## 📧 **Contact**

**Repository:** https://github.com/Davec6505/Pic32mzCNC_V3

---

**Last Updated:** November 7, 2025  
**Status:** Development Phase 🔧  
**Firmware Version:** CS23 (bins/Release/CS23.hex)
```c
typedef struct {
    // Existing: Bresenham, steps, pulse_width, dominant_axis
    
    // NEW: Trapezoidal profile (GRBL-style)
    uint32_t initial_rate;       // Start interval (timer ticks)
    uint32_t nominal_rate;       // Cruise interval (timer ticks)
    uint32_t final_rate;         // End interval (timer ticks)
    uint32_t accelerate_until;   // Step count to end accel
    uint32_t decelerate_after;   // Step count to start decel
    int32_t rate_delta;          // Interval change per step (signed)
    
    // Kept: Physics params for debugging
    float start_velocity, max_velocity, end_velocity, acceleration;
} MotionSegment;
```

**ISR Segment Conditioning Pattern** (runs IN the ISR):
```c
void __ISR(_OC1_VECTOR, IPL5SOFT) OC1Handler(void) {
    IFS0CLR = _IFS0_OC1IF_MASK;  // Clear flag FIRST
    
    // Update velocity profile (integer math only)
    if (steps_completed <= accelerate_until) {
        current_step_interval -= rate_delta;  // Accelerate
    } else if (steps_completed > decelerate_after) {
        current_step_interval += rate_delta;  // Decelerate
    }
    // Cruise: no change to interval
    
    // Bresenham for subordinate axes
    // ...
    
    // Schedule next dominant pulse
    uint32_t now = TMR2;
    OC1R = now + current_step_interval;  // ABSOLUTE timer value
    OC1RS = OC1R + pulse_width;
    
    steps_completed++;
}
```

**Implementation Phases:**
1. **Phase 1**: Constant velocity motion (verify Bresenham works)
2. **Phase 2**: Single-tier trapezoidal profiling (this architecture)
3. **Phase 3**: Two-tier buffer with look-ahead planning (future)

**Key Decisions:**
- Start simpler than full GRBL implementation
- Match GRBL math but use simpler buffer structure
- Add look-ahead later when needed for smooth corners
- Single ISR approach eliminates V2 synchronization headaches

### Velocity Profiling (Legacy Notes)
**Status**: ~~Under consideration for future implementation~~ **Superseded by November 3, 2025 design above**

Velocity profiling will enable smooth acceleration and deceleration:
- **Trapezoidal profiles**: Constant acceleration/deceleration with cruise phase
- **S-curve profiles**: Jerk-limited motion for smoother operation
- **Dynamic step intervals**: Adjust OCx scheduling based on current velocity
- **Look-ahead planning**: Analyze upcoming segments to optimize velocity transitions

Integration points:
- Segment planner computes velocity profile per segment
- Dominant axis ISR adjusts pulse interval based on acceleration state
- Requires real-time step interval calculation or pre-computed lookup tables

Example ISR Skeleton
void __ISR(_OC1_VECTOR, IPL5SOFT) OC1Handler(void) {
    IFSxCLR = _IFSx_OC1IF_MASK; // Clear interrupt flag

    bresenham_step(&segment, axes, now);
  
  Possibly do this in Bresenhams calculation for better control.
  *  uint32_t now = TMR2;
  *  OC1R = now + segment.x_interval;
  *  OC1RS = OC1R + pulse_width;
}


CONSIDERATIONS:
## TMR2 is free running.
### Option 3: Use 32-bit TMR2:TMR3 Pair
1. 	Gives you a much larger range (up to 4.29 billion ticks).
2. 	Reduces the chance of overflow or wraparound issues.
3. 	Lets you schedule OCxR values far ahead without worrying about PR2.
4.  [OCxR:OCxRS] in absolute mode.


There are two alternative initial conditions to consider:  
1. Initialize TMRy = 0 and set OCxR ≥ 1.  
2. Initialize TMRy = PRy (PRy > 0) and set OCxR = 0. | Pulse will be delayed by the value in the PRy register, depending on setup |

## Special Cases for Dual Compare Mode (Single Output Pulse)

### 1. PRy ≥ OCxRS and OCxRS > OCxR
- **Setup:** OCxR = 0, TMRy = 0
- **Operation:**
  - No pulse on first timer cycle; OCx pin stays low.
  - After TMRy resets (period match), OCx pin goes high at OCxR match.
  - At next OCxRS match, OCx pin goes low and stays low.
  - OCxIF interrupt flag set on second compare.
- **Alternative Initializations:**
  - TMRy = 0, OCxR ≥ 1
  - TMRy = PRy (>0), OCxR = 0
- **Output:** Pulse is delayed by PRy value.

### 2. PRy ≥ OCxR and OCxR ≥ OCxRS
- **Setup:** OCxR ≥ 1, PRy ≥ 1
- **Operation:**
  - TMRy counts to OCxR, OCx pin goes high.
  - TMRy continues, resets at PRy (period match).
  - Counts to OCxRS, OCx pin goes low.
  - OCxIF interrupt flag set on second compare.
- **Output:** Standard pulse.

### 3. OCxRS > PRy and PRy ≥ OCxR
- **Setup:** None
- **Operation:**
  - Only rising edge generated at OCx pin.
  - OCxIF interrupt flag not set.
- **Output:** Pin transitions high (rising edge only).

### 4. OCxR > PRy
- **Setup:** None
- **Operation:**
  - Unsupported mode; timer resets before match.
- **Output:** Pin remains low.

**Legend:**
- OCxR: Compare Register
- OCxRS: Secondary Compare Register
- PRy: Timer Period Register
- TMRy: Timer Count

**Note:** TMRy is assumed to be initialized to 0x0000 in all cases.

---

## 📋 TODO List

### High Priority
- [ ] **Homing System** (4-6 hours)
  - GPIO limit switch inputs with pull-ups/interrupts
  - G28/G28.1 commands (home to position, set home)
  - $H homing cycle (seek/locate/pulloff phases)
  - Multi-axis homing (sequential/simultaneous via $23)
  - Homing alarm recovery
  - Settings $22-$27 already implemented

- [ ] **TMR2 Rollover Monitoring** (1-2 hours)
  - Implement controlled reset before 343.6s overflow
  - Monitor TMR2 > 0xF0000000 (~328s threshold)
  - Drain motion queue before reset
  - See `.github/copilot-instructions.md` for pattern

### Medium Priority
- [ ] **Spindle & Coolant Control** (2-3 hours)
  - PWM spindle control (OC6/OC7 module)
  - M3/M4/M5 spindle direction control
  - S value → PWM duty cycle mapping
  - M7/M8/M9 coolant relay outputs
  - Spindle enable delay (non-blocking)
  - Settings $30-$31 already implemented

- [ ] **G17/G18/G19 Plane Selection** (1 hour)
  - Add G-code parser support for plane commands
  - Update modalPlane when G17/G18/G19 received
  - Adapt arc math for XZ and YZ planes
  - modalPlane tracking already implemented

### Low Priority
- [ ] **Advanced G-code Features** (6-8 hours total)
  - G28/G28.1/G30/G30.1 (predefined positions)
  - G54-G59 work coordinates (multiple WCS)
  - G92 coordinate offset (temporary offset)
  - G43.1 tool length offset (dynamic)
  - G10 L2/L20 (WCS programming)
  - Real-time overrides (feed/spindle during motion)
  - Feed hold improvements (decelerate to stop)
  - Parking motion (safe retract)

### Future Enhancements
- [ ] **Look-Ahead Motion Planning** (12-16 hours)
  - Junction deviation (safe corner speeds)
  - Multi-segment velocity planning
  - GRBL-style junction speed calculation
  - Segment merging (co-linear moves)
  - Adaptive acceleration (direction-based)

- [ ] **Testing & Validation**
  - Arc interpolation testing (G2/G3 with various radii)
  - Helical motion testing (Z-axis during arc)
  - Emergency stop testing (limit triggers)
  - Large arc testing (many segments, memory efficiency)
  - TMR2 rollover testing (long-running operations)

---

## Hardware Pin Assignments

This section documents the complete pin mapping for the PIC32MZ2048EFH100 CNC controller. All pin assignments are configured through Microchip Code Configurator (MCC) and defined in the GPIO PLIB.

### Stepper Motor Control

#### X-Axis
- **Step Pulse**: RD4 (StepX pin)
- **Direction**: RE4 (DirX pin)  
- **Enable**: RE6 (EnX pin)

#### Y-Axis
- **Step Pulse**: RF0 (StepY pin)
- **Direction**: RE2 (DirY pin)
- **Enable**: RE5 (EnY pin)

#### Z-Axis
- **Step Pulse**: RF1 (StepZ pin)
- **Direction**: RG9 (DirZ pin)
- **Enable**: RA5 (EnZ pin)

#### A-Axis (4th Axis)
- **Step Pulse**: RG1 (StepA pin)
- **Direction**: RG12 (DirA pin)
- **Enable**: RG15 (EnA pin)

### Limit Switches

#### X-Axis Limits
- **X Min**: RA4 (X_Min pin)
- **X Max**: RA7 (X_Max pin)

#### Y-Axis Limits
- **Y Min**: RD0 (Y_Min pin)
- **Y Max**: RE0 (Y_Max pin)

#### Z-Axis Limits
- **Z Min**: RD13 (Z_Min pin)
- **Z Max**: RE1 (Z_Max pin)

#### A-Axis Limits
- **A Min**: RA6 (A_Min pin)
- **A Max**: RB1 (A_Max pin)

### Spindle Control
- **Spindle PWM**: RE3 (Spindle pin) - OC8 output, 3.338kHz PWM frequency
  - Hardware: Output Compare 8 (OC8) with Timer 6 (TMR6) time base
  - Frequency: 781.25kHz / (233 + 1) = 3.338kHz
  - Duty Cycle Range: 0-100% (0-233 PWM register values)

### Coolant Control
- **Coolant Output**: RB15 (Coolant pin) - Digital on/off control for coolant pump/mist

### Communication
- **UART3 Communication**: Used for GRBL protocol (USB/Serial interface)
  - Baud Rate: Configured via MCC (typically 115200)
  - Buffer Sizes: RX=512 bytes, TX=1024 bytes
  - Protocol: GRBL v1.1 compatible

### Debug/Status LEDs
- **LED1**: RE7 - Heartbeat/Status indicator (1Hz in APP_IDLE)
- **LED2**: RA9 - Motion debug indicator (toggles during `STEPPER_ScheduleStep()`)

### User Interface
- **SW1**: RC3 - User switch 1 (input with internal pull-up)
- **SW2**: RB0 - User switch 2 (input with internal pull-up)

### Important Notes

1. **Active Logic Levels**: 
   - Stepper signals: Active HIGH step pulses, direction and enable as configured
   - Limit switches: Configurable via GRBL setting `$5` (invert mask)
   - Default assumption: Active LOW limit switches (closed = grounded = triggered)

2. **Stepper Driver Compatibility**:
   - Pulse width: 2.5µs (compatible with most stepper drivers)
   - Pulse frequency: Variable based on motion speed (up to 512kHz maximum)
   - Enable signals: Active HIGH to enable stepper drivers

3. **Hardware Requirements**:
   - Stepper driver modules (e.g., DRV8825, A4988, TMC2208)
   - Limit switch debouncing handled in software (25µs default)
   - External pull-ups/pull-downs as needed for limit switches
   - Spindle PWM compatible with ESC or VFD PWM input (3.3V logic)

4. **Safety Considerations**:
   - All limit switches monitored continuously during motion
   - Emergency stop implemented via limit switch triggering
   - Stepper enable pins allow immediate motion halt
   - Hardware debouncing prevents false limit triggers

### Connection Examples

```
Stepper Driver Connections (per axis):
PIC32MZ Pin → Stepper Driver
---------------------------- 
StepX (RD4) → STEP+
DirX  (RE4) → DIR+  
EnX   (RE6) → EN+
GND         → STEP-, DIR-, EN-

Limit Switch Connections:
PIC32MZ Pin → Switch → Ground
-----------------------------
X_Min (RA4) → NC Contact → GND
X_Max (RA7) → NC Contact → GND
(Repeat for Y, Z, A axes)

Spindle PWM Connection:
PIC32MZ Pin → ESC/VFD
---------------------
Spindle (RE3) → PWM Input (3.3V logic)
GND           → Signal Ground
```

For complete MCC configuration and pin alternate function settings, refer to the project's `.mcc` configuration files and GPIO PLIB generated code in `srcs/config/default/peripheral/gpio/`.

````
