# Pic32mzCNC_V3 - CNC Motion Control System

**CNC Controller for PIC32MZ Microcontrollers**

A CNC motion control system based on the GRBL v1.1 protocol compatibility, 4-axis coordinated motion, and hardware-optimized stepper control.

## 🎯 Key Design Concept

**Virtual Dominant Axis Architecture** - Unlike traditional GRBL implementations that use a fixed 30kHz timer interrupt running continuously, this system uses OC1 continuous pulse mode as a virtual dominant axis. The ISR only fires at the actual step rate needed for motion, eliminating thousands of wasted interrupts per second and dramatically reducing CPU overhead.

---

## 🚀 Status

**Under Active Development** - Testing with UGS, Candle, bCNC  
**Firmware**: `bins/Release/CS23.hex` (264KB)  
**Latest Build**: November 20, 2025

### Current State
- ✅ GRBL v1.1 protocol fully compliant
- ✅ 4-axis coordinated motion (XYZA) - (B,C) can be added.
- ✅ Arc interpolation with radius compensation
- ✅ Flow control with deferred "ok" responses
- ✅ Soft reset recovery (Ctrl+X)
- ✅ NVM flash settings persistence (KSEG0 addressing)
- ✅ File streaming with automatic completion
- ✅ Non-blocking architecture - app state machine runs continuously, zero blocking loops during motion

---

## ⚡ Key Features

### Motion Control
- **4-Axis Coordination**: XYZA stepper control with Bresenham interpolation
- **Virtual Dominant Axis**: OC1 continuous pulse mode drives ISR at step rate - no fixed 30kHz timer, only fires when motion needed
- **Hardware Timing**: TMR4 (781.25kHz) + OC1 variable period based on current step interval
- **Optimal Performance**: 2.5µs pulse width, 256 microstepping support
- **Trapezoidal Profiling**: Acceleration/deceleration with velocity planning
- **Arc Interpolation**: G2/G3 with $13 radius compensation
- **Emergency Stop**: Real-time response with hardware limit support
- **Non-Blocking Design**: Zero blocking loops - app state machine runs continuously during motion

### GRBL v1.1 Protocol
- **Full Compliance**: Status reports, real-time commands, flow control
- **G-Code Support**: G0/G1, G2/G3, G4, G17/G18/G19, G90/G91, G92, G10 L20
- **Modal Commands**: M3/M5 (spindle), M7/M9 (coolant), F (feedrate), S (speed), T (tool)
- **Real-Time Control**: `?` status, `!` feed hold, `~` resume, Ctrl+X soft reset
- **System Commands**: `$$` settings, `$I` info, `$G` state, `$#` offsets, `$H` homing

### Hardware Abstraction
- **Clean GPIO Layer**: Function pointer arrays for all axis/limit operations
- **LED Pattern Design**: Direct register access with zero-overhead inlining
- **Simplified Architecture**: Flat structures, no unnecessary nesting
- **Hardware Validation**: Automatic OC1/TMR4 restart after soft reset

---

## 🏗️ Architecture Overview

### System Flow

```
User Input (UART) → G-Code Parser → Event Queue → Kinematics → Motion Queue → Stepper ISR → Motors
                         ↓              ↓             ↓            ↓
                    Flow Control   Modal State   Velocity     Position
                                                 Profiling    Tracking
```

### Main Components

1. **G-Code Parser** (`srcs/gcode/gcode_parser.c`)
   - Event-driven architecture
   - Tokenization with combined modal splitting
   - Flow control with deferred "ok" responses
   - Real-time character handling

2. **Kinematics** (`srcs/motion/kinematics.c`)
   - Linear move physics calculations
   - Arc interpolation (incremental)
   - Velocity profiling (trapezoidal)
   - Coordinate transformations

3. **Motion Controller** (`srcs/motion/motion.c`)
   - Segment queue management
   - Priority phase system
   - Velocity updates (accel/cruise/decel)

4. **Stepper Control** (`srcs/motion/stepper.c`)
   - Hardware abstraction (TMR4/OC1)
   - Bresenham interpolation
   - ISR for step pulse generation

5. **Settings** (`srcs/settings/settings.c`)
   - Persistent NVM flash storage (KSEG0 @ 0x9D180000)
   - GRBL parameters (29 settings)
   - Callback pattern for flash writes

---

## 🔄 Process Flow

### Motion Execution Pipeline

```
1. UART Receive → rxBuffer accumulates bytes
2. Line Complete → Tokenize and parse
3. Event Created → Add to command queue
4. Event Processed → Generate motion segment
5. Segment Queued → Add to motion queue (16 segment buffer)
6. Segment Loaded → STEPPER_LoadSegment() configures TMR4/OC1
7. ISR Executes → OCP1_ISR fires at step rate
8. Bresenham → Subordinate axes step when error accumulates
9. Velocity Update → Accel/cruise/decel phases (main loop)
10. Completion → Dequeue segment, load next
```

### Flow Control Mechanism

```
Command Received → Check motion queue occupancy
                    ↓
        motionQueueCount > 0?
                    ↓
              Yes ─→ Defer "ok" (set okPending = true)
              No  ─→ Send "ok" immediately
                    ↓
Motion Completes → motionSegmentCompleted flag set
                    ↓
        Check deferred ok in IDLE loop
                    ↓
        motionQueueCount == 0? → Send deferred "ok"
```

---

## 🔧 Hardware Configuration

**Microcontroller**: PIC32MZ2048EFH100  
**System Clock**: 200MHz  
**Peripheral Clock**: 50MHz (PBCLK3)  
**Hardware FPU**: Single-precision enabled

### Timer Configuration
- **TMR4**: 16-bit, 1:64 prescaler (781.25kHz, 1.28µs/tick)
- **OC1**: X-axis continuous pulse mode (dual-compare)
- **OC8/TMR6**: Spindle PWM (3.338kHz)

### Memory Map (Flash)
```
0x9D000000 - 0x9D17FFFF : Application code (~1.5MB)
0x9D180000 - 0x9D183FFF : GRBL settings (16KB, KSEG0 cached)
0x9D1F4000 - 0x9D1FFFFF : MikroE bootloader (48KB)
```

**Critical**: NVM writes MUST use KSEG0 cached addresses (0x9D...), NOT KSEG1 uncached (0xBD...).

---

## 📁 Project Structure

```
Pic32mzCNC_V3/
├── srcs/                      # Source code
│   ├── app.c                  # Main application state machine
│   ├── main.c                 # Entry point
│   ├── gcode/
│   │   └── gcode_parser.c     # GRBL protocol parser
│   ├── motion/
│   │   ├── stepper.c          # Hardware abstraction (TMR4/OC1)
│   │   ├── motion.c           # Motion queue management
│   │   ├── kinematics.c       # Physics & velocity profiling
│   │   ├── homing.c           # $H command
│   │   └── spindle.c          # M3/M5 PWM control
│   ├── settings/
│   │   └── settings.c         # Persistent NVM storage
│   └── utils/
│       ├── utils.c            # GPIO abstraction
│       └── uart_utils.c       # Non-blocking UART
├── incs/                      # Header files
│   ├── data_structures.h      # Unified structures
│   ├── common.h               # Debug system
│   └── ...
├── bins/Release/              # Build outputs
│   └── CS23.hex               # Production firmware
├── docs/                      # Documentation
├── ps_commands/               # PowerShell test scripts
└── Makefile                   # Build system
```

---

## 🚀 Quick Start

### Building

```powershell
# Release build (default, optimized)
make

# Debug build with motion tracing
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"

# Clean build
make clean && make

# Flash firmware
make flash
```

### Testing

```powershell
# Run test script (UART at 115200 baud)
.\ps_commands\test_gcode.ps1 -FilePath .\gcode_tests\01_simple_square.gcode

# Manual testing via PuTTY/terminal
?                    # Status query
$$                   # View all settings
$H                   # Home all axes
G90 G1 X10 F500     # Move 10mm in X
```

### G-Code Sender Setup (UGS)
1. Port: Select COM port (115200 baud)
2. Firmware: GRBL
3. Settings: Auto-connect enabled
4. Visualization: Real-time position updates

---

## 📊 Key Algorithms

### Bresenham Interpolation (ISR)
```c
// Dominant axis always steps
AXIS_StepSet(dominant_axis);

// Subordinate axes step when error accumulates
for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
    error[axis] += delta[axis];
    if (error[axis] >= dominant_delta) {
        AXIS_StepSet(axis);
        error[axis] -= dominant_delta;
    }
}
```

### Trapezoidal Velocity Profiling (Main Loop)
```c
if (steps_completed < accelerate_until) {
    // Acceleration phase: decrease interval (increase speed)
    step_interval -= rate_delta;
} else if (steps_completed > decelerate_after) {
    // Deceleration phase: increase interval (decrease speed)
    step_interval += rate_delta;
}
// Cruise phase: no change to interval
```

### Priority Phase System
```c
// Main loop processes motion phases in order
switch (appData.motionPhase) {
    case MOTION_PHASE_VELOCITY:   // Update step rate
    case MOTION_PHASE_BRESENHAM:  // Update error terms
    case MOTION_PHASE_SCHEDULE:   // Update OCx registers
    case MOTION_PHASE_COMPLETE:   // Check completion
    case MOTION_PHASE_IDLE:       // Safe for G-code processing
}
```

---

## 🎯 Design Principles

### Code Style Best Practices

1. **External Variables**: Always declare in headers, never in function scope
   ```c
   // ✅ GOOD - Header declaration
   extern volatile bool g_hard_limit_alarm;  // stepper.h
   
   // ❌ BAD - Function scope extern
   void MyFunction(void) {
       extern volatile bool g_hard_limit_alarm;  // DON'T!
   }
   ```

2. **Array-Based Axis Control**: No switch statements
   ```c
   // ✅ GOOD - Loop over axes
   for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
       SET_COORDINATE_AXIS(&target, axis, value[axis]);
   }
   
   // ❌ BAD - Individual assignments
   target.x = value_x;
   target.y = value_y;
   ```

3. **Debug System**: Compile-time with zero runtime overhead
   ```c
   // ✅ GOOD - Removed in release builds
   DEBUG_PRINT_MOTION("[MOTION] Message\r\n");
   
   // ❌ BAD - Runtime overhead
   if (debug_enabled) { printf(...); }
   ```

4. **Single Instance Pattern**: All data in APP_DATA
   ```c
   // ✅ GOOD - Centralized in APP_DATA
   appData.motionQueueCount
   
   // ❌ BAD - Static module data
   static uint32_t motionQueueCount;
   ```

---

## 📚 Documentation

- **[Settings Reference](docs/readme/SETTINGS_REFERENCE.md)** - GRBL parameters ($0-$132)
- **[Debug System Tutorial](docs/readme/DEBUG_SYSTEM_TUTORIAL.md)** - Compile-time debugging
- **[Memory Map](docs/readme/MEMORY_MAP.md)** - Flash layout and NVM details
- **[Architecture](docs/readme/ARCHITECTURE.md)** - Technical deep dive

---

## 🧪 Validated Test Results

**Rectangle Test**: ✅ Complete dual iteration (0,0,0 final position)  
**Circle Test**: ✅ 20 segments, 0.025mm final error  
**Arc Compensation**: ✅ $13 tolerance handles CAM rounding  
**Back-to-Back Execution**: ✅ Multiple files without reset  
**Soft Reset Recovery**: ✅ Ctrl+X + motion restart working  

---

## 🔧 Troubleshooting

### Motors don't run after power-up
- Check stepper enable pins (verify $4 invert setting)
- Run `$H` to home if hard limits enabled
- Send `$X` if in alarm state

### Settings lost after power cycle
- Verify flash address is KSEG0 (0x9D...) not KSEG1 (0xBD...)
- Check `SETTINGS_VERSION` matches flash data
- Use `$RST=$` to restore defaults

### Motion jerky or stuttering
- Increase acceleration ($120-$123 settings)
- Verify steps/mm calibration ($100-$103)
- Check microstepping matches driver config

---

## 📄 License

Proprietary - All rights reserved.

---

**Firmware Build**: `1.1h.20251120`  
**Hardware**: PIC32MZ2048EFH100 @ 200MHz  
**Repository**: github.com/Davec6505/Pic32mzCNC_V3
