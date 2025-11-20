# Architecture Guide - Pic32mzCNC_V3

## Overview
Comprehensive architectural documentation for the Pic32mzCNC_V3 CNC motion control system.

**Last Updated**: November 15, 2025  
**Status**: Under Active Testing

---

## System Architecture

### High-Level Design
```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  GRBL Parser │→ │  Kinematics  │→ │ Motion Queue │      │
│  │  (G-code)    │  │  (Physics)   │  │  (Segments)  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                   Hardware Abstraction                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Stepper    │  │    Timers    │  │     GPIO     │      │
│  │   Control    │  │  (TMR4/OC)   │  │  (Limits)    │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **LED Pattern GPIO Abstraction**
   - Function pointer arrays for direct hardware access
   - Zero-overhead inline functions
   - No nested structures
   - Compiler optimizes to single instructions

2. **Single Authoritative Data**
   - No data copies between modules
   - Direct references to `appData->motionQueueCount`
   - Eliminates synchronization bugs

3. **Compile-Time Debug System**
   - Zero runtime overhead in release builds
   - Multiple debug subsystems
   - Clean enable/disable via Makefile

4. **Event-Driven Processing**
   - Non-blocking G-code parsing
   - One event per main loop iteration
   - Priority-based motion phases

---

## Module Breakdown

### 1. GRBL Parser (gcode_parser.c)
**Purpose**: Parse GRBL v1.1 protocol commands and generate motion events

**Key Features**:
- Combined modal splitting: "G90G1X10" → ["G90", "G1X10"]
- Blank line handling with proper "ok" responses
- Real-time character processing
- 16-command circular buffer with flow control

**Flow Control Architecture**:
```c
// Defer "ok" when motion queue has content
if (appData->motionQueueCount > 0) {
    okPending = true;  // Defer response
} else {
    UART_SendOK();      // Send immediately
}

// Check deferred ok when motion completes
if (appData->motionSegmentCompleted && appData->motionQueueCount == 0) {
    UART_SendOK();  // Send deferred ok
    okPending = false;
}
```

**Critical Rule**: DO NOT MODIFY - Production validated!

### 2. Kinematics (kinematics.c)
**Purpose**: Convert G-code coordinates to motion segments with physics

**Key Calculations**:
- Trapezoidal velocity profiling
- Acceleration/deceleration planning
- Feedrate limiting to max_rate
- Arc interpolation (G2/G3)

**Velocity Profile**:
```c
// Three phases: accel → cruise → decel
float accel_distance = (feedrate^2) / (2 * acceleration);
float cruise_distance = total_distance - 2*accel_distance;
float decel_distance = accel_distance;

// Step intervals vary during acceleration
uint32_t start_interval = high;  // Slow
uint32_t cruise_interval = low;  // Fast
uint32_t end_interval = high;    // Slow
```

### 3. Motion Control (motion.c)
**Purpose**: Manage motion queue and segment execution

**Queue Architecture**:
```c
typedef struct {
    MotionSegment motionQueue[MAX_MOTION_SEGMENTS];  // 16 segments
    uint32_t motionQueueHead;      // Write index
    uint32_t motionQueueTail;      // Read index
    uint32_t motionQueueCount;     // ✅ AUTHORITATIVE count
    bool motionSegmentCompleted;   // Triggers deferred ok check
} APP_DATA;
```

**Segment Lifecycle**:
1. Kinematics generates segment
2. Added to queue (if space available)
3. Stepper module loads segment
4. ISR executes Bresenham steps
5. Segment completes → set `motionSegmentCompleted` flag
6. Next segment loaded automatically

### 4. Stepper Control (stepper.c)
**Purpose**: Hardware timing and step pulse generation

**Timer Configuration**:
- **TMR4**: 16-bit, 1:64 prescaler (781.25kHz)
- **OC1**: X-axis continuous pulse mode
- **OC2**: Y-axis continuous pulse mode
- **Pulse Width**: 2.5µs (2 ticks)

**ISR Architecture**:
```c
void OCP1_ISR(void) {
    // Clear flag FIRST
    IFS0CLR = _IFS0_OC1IF_MASK;
    
    // Generate step pulse
    AXIS_StepSet(dominant_axis);
    
    // Bresenham for subordinate axes
    for (E_AXIS axis = 0; axis < NUM_AXIS; axis++) {
        if (axis == dominant_axis) continue;
        error[axis] += delta[axis];
        if (error[axis] >= dominant_delta) {
            AXIS_StepSet(axis);
            error[axis] -= dominant_delta;
        }
    }
    
    // Schedule next pulse
    TMR4_PeriodSet(max(7, step_interval + pulse_width + 2));
}
```

**Hardware Validation Guards**:
```c
// Prevent soft reset failures
if(!(OC1CON & _OC1CON_ON_MASK)) {
    OCMP1_Enable();  // Re-enable if disabled
}
if(!(T4CON & _T4CON_ON_MASK)) {
    TMR4_Start();    // Re-start if stopped
}
```

### 5. Hardware Abstraction (utils.c/utils.h)
**Purpose**: Clean GPIO layer with function pointer arrays

**LED Pattern Design**:
```c
// Function pointer arrays (global, not static)
GPIO_SetFunc axis_step_set[NUM_AXIS] = {
    step_x_set, step_y_set, step_z_set, step_a_set
};

GPIO_GetFunc axis_limit_min_get[NUM_AXIS] = {
    x_min_get, y_min_get, z_min_get, a_min_get
};

// Flat settings structures
typedef struct {
    float* max_rate;
    float* acceleration;
    float* steps_per_mm;
    int32_t* step_count;
} AxisSettings;

extern AxisSettings g_axis_settings[NUM_AXIS];
extern HomingSettings g_homing_settings[NUM_AXIS];

// Inline helpers (zero overhead)
static inline void AXIS_StepSet(E_AXIS axis) {
    axis_step_set[axis]();  // → LATDSET = 0x10
}
```

**Compiler Optimization**:
```assembly
; Full inline chain:
; AXIS_StepSet(0) → axis_step_set[0]() → step_x_set() → StepX_Set() → LATDSET
; Results in single instruction:
sw $t1, 0($t0)  ; Store to LATDSET register
```

### 6. Settings (settings.c)
**Purpose**: Persistent GRBL parameter storage in NVM flash

**Storage Layout**:
```c
#define SETTINGS_NVM_ADDRESS 0xBD1F0000  // KSEG1 uncached
#define SETTINGS_VERSION 2                // Structure version

typedef struct {
    // 29 GRBL parameters
    uint8_t pulse_microseconds;     // $0
    uint8_t stepper_idle_lock_time; // $1
    // ... (27 more)
    float arc_tolerance;            // $13
    // Work coordinate offsets
    CoordinatePoint work_offset[6]; // G54-G59
} CNC_Settings;
```

**Write Pattern**:
```c
// 1. Erase page
NVM_PageErase(SETTINGS_NVM_ADDRESS);
while(xferDone == false);

// 2. Write row
NVM_RowWrite(writeData, SETTINGS_NVM_ADDRESS);
while(xferDone == false);
```

**Delayed Initialization**:
```c
// APP_INITIALIZE: Register callback only
SETTINGS_Initialize();

// APP_LOAD_SETTINGS: Read flash after peripherals ready
SETTINGS_LoadFromFlash(SETTINGS_GetCurrent());
```

---

## Data Flow

### G-Code to Motion Pipeline
```
User Input (UGS)
    ↓
UART3 RX (115200 baud)
    ↓
GRBL Parser (line buffering)
    ↓
Tokenization ("G90G1X10" → ["G90", "G1X10"])
    ↓
Event Generation (GCODE_EVENT_LINEAR_MOVE)
    ↓
Kinematics (physics calculations)
    ↓
Motion Queue (segment buffer)
    ↓
Stepper Module (segment loading)
    ↓
ISR (Bresenham + GPIO pulses)
    ↓
Stepper Motors (physical movement)
```

### Flow Control Feedback Loop
```
Motion Queue Count
    ↓
Check: count > 0?
    ├─ YES → Defer "ok" response (okPending = true)
    └─ NO  → Send "ok" immediately
    ↓
Motion Completes (motionSegmentCompleted = true)
    ↓
Check: okPending && count == 0?
    ├─ YES → Send deferred "ok", clear okPending
    └─ NO  → Continue
    ↓
UGS receives "ok" → Sends next command
```

---

## Critical Timing Requirements

### ISR Execution Budget
- **Available**: 1.28µs per tick @ 781.25kHz
- **Used**: ~225 CPU cycles (0.75µs @ 200MHz)
- **Headroom**: 42% at max 512kHz step rate
- **Safety**: Validated with 256 microstepping

### Pulse Timing
- **Pulse Width**: 2.5µs (safe for all stepper drivers)
- **Minimum Period**: 7 ticks (ensures pulse completion)
- **Formula**: `max(7, step_interval + pulse_width + 2)`

### Main Loop Timing
- **Non-blocking**: No delays or while loops
- **Event Processing**: One G-code event per iteration
- **Motion Phases**: Priority-based execution
- **UART Polling**: Rate-limited in IDLE phase

---

## Safety Systems

### Emergency Stop
```c
// Hardware limits trigger alarm
if (MOTION_UTILS_CheckHardLimits()) {
    appData.state = APP_ALARM;
    STEPPER_DisableAll();  // Stop all motion
}

// Soft reset (Ctrl+X)
GCODE_HandleSoftReset(&appData, &cmdQueue);
// Clears queues, resets state, prints banner
```

### Position Tracking
```c
// Machine coordinates (absolute)
int32_t mpos_steps = AXIS_GetSteps(axis);
float mpos_mm = mpos_steps / steps_per_mm;

// Work coordinates (relative to offset)
float wpos_mm = mpos_mm - work_offset[axis];
```

---

## Performance Characteristics

### Motion Capabilities
- **Max Feedrate**: ~8000 mm/min (validated)
- **Acceleration**: Configurable per axis
- **Arc Tolerance**: $13 setting (default 0.002mm)
- **Position Accuracy**: ±0.025mm (20-segment circle test)

### Memory Usage
- **Firmware Size**: 264KB (14% of available 1.87MB)
- **Motion Queue**: 16 segments × ~80 bytes = 1.3KB
- **G-Code Buffer**: 16 commands × ~64 bytes = 1KB
- **Stack**: 131KB allocated

### Build Configurations
- **Release**: Optimized (-O1), no debug (264KB)
- **Debug**: With symbols, debug output (~280KB)
- **Compilation Time**: ~20 seconds clean build

---

## Testing & Validation

### Validated Features (November 15, 2025)
- ✅ Rectangle path (dual iteration)
- ✅ Circle (20 segments, 0.025mm error)
- ✅ Arc compensation (0.001mm tolerance)
- ✅ Back-to-back file execution
- ✅ UGS automatic completion
- ✅ Soft reset recovery
- ✅ Flow control synchronization
- ✅ Array-based architecture

### Test Scripts
```powershell
# Basic motion test
.\ps_commands\test_gcode.ps1 -FilePath .\gcode_tests\01_simple_square.gcode

# Complex path validation
.\ps_commands\test_double_rectangle.ps1

# Arc interpolation test
.\ps_commands\test_gcode.ps1 -FilePath .\tests\03_circle_20segments.gcode
```

---

## Future Enhancements

### Planned Features
- Look-ahead junction planning
- S-curve acceleration
- Production CNC homing validation
- Spindle PWM CNC testing

### Known Limitations
- No junction velocity optimization
- Trapezoidal-only acceleration profile
- Homing tested in development, not production
- Spindle configured but not CNC-validated

---

## References

- [README.md](../../README.md) - Project overview
- [SETTINGS_REFERENCE.md](SETTINGS_REFERENCE.md) - GRBL parameters
- [DEBUG_SYSTEM_TUTORIAL.md](DEBUG_SYSTEM_TUTORIAL.md) - Debug guide
- [MEMORY_MAP.md](MEMORY_MAP.md) - Flash layout
