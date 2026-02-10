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
- **Emergency Stop**: Real-time response with hardware limit support - gpio needs adding to complete this feature.
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
**Peripheral Clock**: 50MHz (PBCLK3) - TMR 
**Hardware FPU**: Single-precision enabled

### Timer Configuration
- **TMR4**: 16-bit, 1:64 prescaler (781.25kHz, 1.28µs/tick)
- **OC1**: X-axis continuous pulse mode (dual-compare)
- **OC8/TMR6**: Spindle PWM (3.338kHz)

### Memory Map (Flash)
```
0x9D000000 - 0x9D17FFFF : Refer to other/Release/memoryfile.xml
0x9D180000 - 0x9D183FFF : GRBL settings (16KB, KSEG0 cached)
0x9D1F4000 - 0x9D1FFFFF : MikroE bootloader (48KB)
```

**Critical**: NVM writes MUST use KSEG0 cached addresses (0x9D...), NOT KSEG1 uncached (0xBD...).

---

## 📞 Call Structure (Runtime Hierarchy)

```
main.c (Entry point)
  └── app.c (Main state machine - APP_Tasks)
      │
      ├── APP_INIT → settings/settings.c (Register NVM callback)
      ├── APP_LOAD_SETTINGS → settings/settings.c (Load from flash)
      │
      ├── APP_IDLE → (Main processing loop)
      │   ├── motion/motion.c (Motion queue management)
      │   ├── gcode/gcode_parser.c (Parse G-code events)
      │   ├── motion/kinematics.c (Generate motion segments)
      │   └── motion/stepper.c (Load segments to hardware)
      │
      ├── APP_HOMING → motion/homing.c ($H command - 4 phase cycle)
      │
      └── APP_ALARM → motion/stepper.c (Emergency stop)

ISR (Hardware interrupts - runs asynchronously)
  ├── stepper.c::OCP1_ISR (Bresenham + step pulses)
  ├── stepper.c::TMR5_Callback (Pulse width timing)
  ├── motion/spindle.c (M3/M5 PWM control)
  └── UART3_ISR (Ring buffer updates - RX/TX)

utils/ (Called from all states - helper functions)
  ├── utils.c (GPIO abstraction - inline functions)
  └── uart_utils.c (Non-blocking UART)
```

## 📁 File Structure (Directory Layout)

```
Pic32mzCNC_V3/
├── srcs/                      # Source code
│   ├── main.c                 # Entry point
│   ├── app.c                  # Main application state machine
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

# Available DEBUG_FLAGS:
# DEBUG_MOTION - Motion planning and segment execution
# DEBUG_GCODE - G-code parsing and event processing
# DEBUG_STEPPER - Low-level stepper ISR
# DEBUG_SEGMENT - Segment loading and queue management
# DEBUG_UART - UART communication
# DEBUG_APP - Application state machine

# build
make

# Clean build
make all

# Flash firmware
use mikroe bootloader.
```

### Testing

```powershell
# Run test script (UART at 115200 baud)
.\ps_commands\test_gcode.ps1 -FilePath .\gcode_tests\01_simple_square.gcode

# Code search utilities
.\ps_commands\search.ps1 -wordToFind "STEPPER_LoadSegment"              # Find function usage
.\ps_commands\search_functions.ps1  -functionName "motion/stepper.c"    # List all functions in file
``


# Manual testing via PuTTY/terminal
?                    # Status query
$$                   # View all settings
$H                   # Home all axes
G90 G1 X10 F500     # Move 10mm in X
`

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

## 🔌 LitePlacer Integration Requirements

**Status**: 🚧 In Progress - Implementation Checklist

This firmware is being integrated with LitePlacer Pick-and-Place software. Below are the requirements that need verification/implementation for seamless integration.

### 📡 Protocol Requirements

#### 1. **Position Reporting Format** ⬜ TODO
**Required**: Standard GRBL status query response

**Send**: `?`

**Expected Response**:
```
<Idle|MPos:43.123,12.456,78.901,0.000|FS:0,0>
```

**Format Details**:
- State: `Idle`, `Run`, `Hold`, `Alarm`, `Home`, `Check`
- MPos: Machine position (X, Y, Z, A in mm)
- FS: Feed rate and spindle speed

**Current Status**: ❓ Need to verify format
- [ ] Test `?` command and capture output
- [ ] Verify all 4 axes (XYZA) are reported
- [ ] Confirm position precision (3 decimal places minimum)

---

#### 2. **Probe Response (G38.2)** ⬜ TODO
**Required**: Probe position feedback after successful trigger

**Send**: `G38.2 Z-100 F300`

**Expected Response Option A** (Preferred):
```
[PRB:43.123,12.456,78.901,0.000:1]
ok
```
- `PRB:` probe result message
- Position where probe triggered (XYZA)
- `:1` = success, `:0` = failed (no contact)

**Expected Response Option B** (Acceptable):
```
ok
<Idle|MPos:43.123,12.456,78.901,0.000>
```
- Position in subsequent status query

**Current Status**: ❓ Need to implement/verify
- [ ] Verify G38.2 is implemented
- [ ] Test probe trigger detection
- [ ] Capture probe response format
- [ ] Verify Z position preservation (no reset to 0)
- [ ] Test probe failure timeout

**Critical for LitePlacer**: 
- Probe MUST preserve Z position where switch triggers
- No automatic backoff/reset during calibration
- Position accuracy within 0.1mm

---

#### 3. **Error and Alarm Handling** ⬜ TODO
**Required**: Clear error/alarm messages for fault recovery

**Expected Formats**:

**Errors** (recoverable):
```
error:9
```
- Numeric error codes (GRBL standard)
- See [GRBL Error Codes](https://github.com/gnea/grbl/wiki/Grbl-v1.1-Interface#grbl-response-messages)

**Alarms** (require reset):
```
ALARM:1
```
- Numeric alarm codes
- Machine enters locked state
- Requires `$X` to unlock

**Current Status**: ❓ Need to verify
- [ ] Test invalid G-code (trigger error)
- [ ] Test limit switch trigger (trigger alarm)
- [ ] Verify `$X` unlock command
- [ ] Capture error/alarm message formats

---

#### 4. **Settings Commands** ✅ CONFIRMED
**Required**: GRBL settings management

| Command | Description | Status |
|---------|-------------|--------|
| `$$` | View all settings | ✅ Verified in README |
| `$100=80` | Set steps/mm for X | ✅ Standard GRBL |
| `$H` | Home all axes | ✅ Verified in README |
| `$X` | Clear alarm state | ✅ Standard GRBL |
| `$RST=$` | Reset to defaults | ✅ Verified in README |

**Current Status**: ✅ Confirmed working
- [x] All commands documented in README

---

#### 5. **Real-Time Commands** ✅ CONFIRMED
**Required**: Non-blocking emergency controls

| Command | Description | Status |
|---------|-------------|--------|
| `?` | Status query | ✅ Verified in README |
| `!` | Feed hold | ✅ Verified in README |
| `~` | Resume | ✅ Verified in README |
| `Ctrl+X` | Soft reset | ✅ Verified in README |

**Current Status**: ✅ Confirmed working

---

### 🔌 Hardware Interface Requirements

#### 6. **Limit Switch Configuration** ⬜ TODO
**Required**: GPIO mapping for limit/probe inputs

**Questions**:
1. Which GPIO pins are limit switches?
   ```c
   // Example needed:
   #define LIMIT_X_PIN     PORTAbits.RA0
   #define LIMIT_Y_PIN     PORTAbits.RA1
   #define LIMIT_Z_MIN_PIN PORTAbits.RA2
   #define LIMIT_Z_MAX_PIN PORTAbits.RA3  // Used as probe input
   ```

2. Is Z-max limit switch used as probe input?
   - [ ] Yes - shared pin (standard GRBL)
   - [ ] No - separate probe pin

3. Active HIGH or LOW?
   - [ ] Active HIGH (switch closes to +5V)
   - [ ] Active LOW (switch closes to GND)

4. Are pull-up/pull-down resistors enabled?
   - [ ] Internal pull-ups enabled
   - [ ] External resistors used

**Current Status**: ❓ Need GPIO documentation
- [ ] Document limit switch pin assignments
- [ ] Confirm probe input configuration
- [ ] Test switch trigger detection

---

#### 7. **Serial Communication Settings** ✅ CONFIRMED
**Required**: UART configuration for host communication

| Setting | Value | Status |
|---------|-------|--------|
| Baud Rate | 115200 | ✅ Confirmed in README |
| Data Bits | 8 | ✅ Standard |
| Stop Bits | 1 | ✅ Standard |
| Parity | None | ✅ Standard |
| Flow Control | None | ❓ Need verification |

**Questions**:
- Do you use XON/XOFF software flow control?
  - [ ] Yes - send XOFF when buffer full
  - [ ] No - rely on deferred "ok" responses

- Do you use RTS/CTS hardware flow control?
  - [ ] Yes - hardware handshaking
  - [ ] No - not implemented

**Current Status**: ✅ Mostly confirmed
- [x] Baud rate verified (115200)
- [ ] Flow control method needs verification

---

### 🧪 Test Cases for Integration

#### Test 1: Status Query Loop ⬜ TODO
```gcode
?
?
?
```
**Expected**: 3 status reports with current position
**Verify**: Position format matches expected

---

#### Test 2: Simple Movement ⬜ TODO
```gcode
G90 G21
G0 X10 Y10
?
```
**Expected**: Machine moves to (10,10), status shows MPos:10.000,10.000,...

---

#### Test 3: Probe Calibration Sequence ⬜ TODO
```gcode
G90 G21
G0 Z50
G38.2 Z-40 F300
?
```
**Expected**: 
1. Probe triggers at ~Z=43mm
2. Either `[PRB:x,y,43.xxx,a:1]` or position in status
3. Z position preserved (not reset to 0)

---

#### Test 4: Error Recovery ⬜ TODO
```gcode
G999
$X
?
```
**Expected**:
1. `error:X` (invalid G-code)
2. Machine recovers
3. Status shows Idle state

---

#### Test 5: Alarm and Clear ⬜ TODO
```gcode
(Trigger limit switch manually)
$X
?
```
**Expected**:
1. `ALARM:X`
2. `$X` clears alarm
3. Status shows Idle, not Alarm

---

### 📋 Implementation Priority

| Priority | Item | Estimated Effort | Status |
|----------|------|------------------|--------|
| 🔴 HIGH | Verify G38.2 probe implementation | 2-4 hours | ⬜ TODO |
| 🔴 HIGH | Test probe position reporting | 1 hour | ⬜ TODO |
| 🔴 HIGH | Capture protocol responses | 1 hour | ⬜ TODO |
| 🟡 MEDIUM | Document GPIO limit switch pins | 30 min | ⬜ TODO |
| 🟡 MEDIUM | Verify error/alarm formats | 1 hour | ⬜ TODO |
| 🟢 LOW | Document flow control method | 15 min | ⬜ TODO |

---

### 🎯 Next Steps

1. **Protocol Capture Session** (30-60 minutes)
   - Connect to firmware via serial terminal (PuTTY/Tera Term)
   - Send test commands from Test Cases above
   - Copy/paste all responses to a text file
   - Share output for LitePlacer integration code

2. **GPIO Documentation** (15-30 minutes)
   - Review `srcs/utils/utils.c` or hardware config
   - Document limit switch pin assignments
   - Note active HIGH/LOW configuration

3. **Integration Development** (LitePlacer side)
   - Create `PIC32MZGrblControl.cs` class
   - Implement GRBL protocol parser
   - Test with captured protocol responses
   - Integrate into LitePlacer CNC abstraction

---

### 📞 Contact for Integration

**Integration Partner**: LitePlacer-DEV project  
**Repository**: https://github.com/Davec6505/LitePlacer-DEV  
**Branch**: patch1  
**Integration Lead**: Dave C

**Questions/Issues**: Create issue in LitePlacer-DEV repo with tag `pic32mz-integration`

---

## 📄 License

Proprietary - All rights reserved.

---

**Firmware Build**: `1.1h.20251120`  
**Hardware**: PIC32MZ2048EFH100 @ 200MHz  
**Repository**: github.com/Davec6505/Pic32mzCNC_V3
