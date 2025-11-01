# Pic32mzCNC_V3 - Advanced CNC Motion Control System

## Project Status: Core Architecture Implemented ✅

**Pic32mzCNC_V3** is a modular CNC motion control system designed for high-performance, multi-axis stepper motor control using Microchip PIC32MZ microcontrollers. It features **absolute compare mode timer architecture**, **dynamic dominant axis tracking**, and **Bresenham interpolation** with a clean, maintainable architecture suitable for custom CNC machines and automation projects.

### 🚀 **Recent Achievements**
- ✅ **Complete core architecture implementation**
- ✅ **Absolute compare mode with TMR2 continuous operation**
- ✅ **Dynamic dominant axis tracking system**
- ✅ **Single instance pattern in appData for clean separation**
- ✅ **GRBL-compatible G-code parser with real-time control**
- ✅ **Modular physics calculations in kinematics module**
- ✅ **Non-blocking Bresenham state machine architecture**

## 🎯 **Current Implementation Status**

### ✅ **Completed Modules**

#### **G-Code Parser Module**
- **Real-time control characters**: `?` (status), `~` (resume), `!` (hold), `^X` (reset)
- **GRBL protocol compliance**: Status reporting, command acknowledgments
- **Non-blocking command processing**: Event-driven UART reception
- **Command queue management**: Buffered G-code parsing and storage

#### **Kinematics Module** 
- **Physics calculations**: Coordinate transformations (G54/G55 work coordinates)
- **Motion segment generation**: Pre-calculated Bresenham parameters
- **Dominant axis determination**: Dynamic selection based on highest step count
- **Protected coordinate management**: Private static work coordinate system

#### **Stepper Module**
- **Hardware abstraction**: OCx register management with absolute compare mode
- **Position tracking**: Real-time step counting in ISRs
- **Pulse generation control**: TMR2-based absolute timer scheduling
- **Axis disabling**: `OCxR = OCxRS` for clean pulse stopping

#### **Motion Controller** (In Progress)
- **Master execution engine**: Bresenham interpolation state machine 
- **Non-blocking execution**: State-based segment processing
- **Absolute timer integration**: OCx scheduling with TMR2 values
- **Segment management**: Smooth transitions between motion segments

### 🏗️ **Architecture Highlights**

#### **Single Instance Pattern in appData**
```c
APP_DATA appData = {
    .gcodeCommandQueue = {...},     // Single G-code command queue
    .motionQueue = {...},           // Single motion segment array  
    .motionQueueHead/Tail/Count     // Centralized queue management
};
```

#### **Absolute Compare Mode Timer Architecture**
```c
// All timer values are absolute TMR2 counts
uint32_t now = TMR2;                    // Read current timer
OC1R = now + step_interval;             // ABSOLUTE schedule ahead
OC1RS = OC1R + pulse_width;             // ABSOLUTE pulse width
```

#### **Dynamic Dominant Axis Tracking**
```c
// Dominant axis (highest step count) tracks continuously
void OC1Handler(void) {                 // X-axis dominant
    IFS0CLR = _IFS0_OC1IF_MASK;         // Clear flag FIRST
    uint32_t now = TMR2;
    OC1R = now + step_interval;         // Schedule next pulse ahead
    steps_completed++;                  // Update position
}
```

## Hardware & Toolchain Requirements
- **Microcontroller:** Microchip PIC32MZ2048EFH100
- **Timer System:** TMR2/TMR3 32-bit pair @ 10µs resolution
- **Output Compare:** OC1 (X), OC2 (Y), OC3 (Z), OC4 (A) axes
- **Development Tools:** MPLAB X IDE, XC32 Compiler v4.x
- **Communication:** UART2 for G-code commands (GRBL compatible)
- **Build System:** Make utility for modular compilation

## 🚀 **Quick Start / Build Instructions**

### **Current Build Status**: ✅ **Compiles Successfully**

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Davec6505/Pic32mzCNC_V3.git
   cd Pic32mzCNC_V3
   ```

2. **Build the project:**
   ```bash
   make          # Incremental build
   make all      # Clean + full build
   make clean    # Clean build artifacts
   ```

3. **Flash firmware to PIC32MZ:**
   ```bash
   # Use MikroE bootloader or MPLAB X IPE
   # Firmware: bins/Release/CS23.hex
   ```

### **Testing Current Implementation**
The system currently supports:
- **UART G-code commands** via UART2
- **Real-time control**: Send `?` for status, `~` for resume, `!` for hold
- **Position tracking**: Real-time step counting and coordinate reporting
- **GRBL status format**: `<Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|FS:0,0>`

### **Project Structure**
```
Pic32mzCNC_V3/
├── srcs/
│   ├── app.c                    # Application state machine
│   ├── main.c                   # Entry point
│   ├── gcode/
│   │   └── gcode_parser.c       # G-code parsing & GRBL protocol
│   └── motion/
│       ├── stepper.c            # Hardware abstraction layer
│       ├── motion.c             # Master motion controller
│       └── kinematics.c         # Physics & coordinate calculations
├── incs/                        # Header files
├── docs/plantuml/              # Architecture diagrams
└── .github/copilot-instructions.md  # Development guidelines
```

## 📋 **Next Implementation Phases**

### **Phase 1: Motion Controller Completion** (In Progress)
- [ ] Complete Bresenham execution functions in `motion.c`
- [ ] Implement segment transition logic for smooth motion
- [ ] Add velocity profiling (acceleration/deceleration)
- [ ] Test coordinated multi-axis motion

### **Phase 2: G-code Command Processing**
- [ ] Parse G01 linear interpolation commands in `gcode_parser.c`
- [ ] Implement G02/G03 arc interpolation via kinematics
- [ ] Add G54-G59 work coordinate system commands
- [ ] Handle feedrate (F) and spindle (S/M3/M5) control

### **Phase 3: Advanced Features**
- [ ] Look-ahead motion planning for smooth trajectories
- [ ] Emergency stop and safety systems
- [ ] Feed hold/resume with position retention
- [ ] Real-time feed rate and spindle overrides

---

# 🏛️ **CNC Motion Control Architecture**

This section provides detailed technical documentation of the **absolute compare mode** architecture with **dynamic dominant axis tracking** using TMR2 and OCx modules.

## Core Architecture Principles

### Free-Running Timer with Absolute Compare Scheduling
- **TMR2 (32-bit)** runs continuously at 10µs/tick with **no interrupts** on the timer itself
- **Never stops or resets** during operation - provides jitter-free time base
- All OCx modules operate in **absolute compare mode** with TMR2 as the shared time base

### Dynamic Dominant Axis Tracking
The dominant axis (highest step count) is **continuously tracked ahead** of the current TMR2 count:
- The dominant axis OCx pair `[OCxR:OCxRS]` is always scheduled at absolute timer values ahead of TMR2
- This axis drives the motion timing and executes a step on every pulse interval
- **Dominant axis can swap on-the-fly** by changing which axis tracks ahead of TMR2
- No timer resets required - switching happens seamically during motion

### Subordinate Axis On-Demand Scheduling
Subordinate axes are only scheduled for compare match when Bresenham's algorithm determines they need a step:
- OCx pairs are set to absolute timer values **only when a step is required**
- When no step is needed, the compare registers remain inactive (not scheduled)
- This minimizes interrupt overhead and maintains precise timing

### Key Features
- **Absolute positioning**: All compare values are absolute TMR2 counts, not relative offsets
- **Jitter-free operation**: Hardware compare ensures deterministic pulse timing
- **Flexible motion profiles**: Dominant axis can change mid-trajectory for complex paths
- **Minimal ISR overhead**: Bresenham logic runs in state machine; ISRs only schedule next pulse
- **Modular design**: Clear separation of planning, stepping, and pulse scheduling

### Motion Coordination Flow
1. Planner determines dominant axis (highest step count in segment)
2. Dominant axis OCx is scheduled ahead of current TMR2 count
3. On dominant axis ISR: Update Bresenham error terms for all subordinate axes
4. Schedule subordinate axis OCx pairs **only if** error check indicates step required
5. Schedule next dominant axis pulse ahead of TMR2
6. Dominant axis can be swapped by changing tracking to different OCx module

## System Components

| Component | Description |
|-----------|-------------|
| **TMR2 (32-bit)** | Free-running timer with 10µs tick resolution, never stops or resets |
| **OCx (OC1–OC4)** | Output Compare modules for X, Y, Z, A axes in absolute compare mode |
| **OCx ISR** | Minimal interrupt handlers for pulse scheduling and step counting |
| **Bresenham Engine** | State machine-based interpolation (non-ISR) for subordinate axis coordination |
| **Segment Planner** | Preloads motion segments, determines dominant axis, manages acceleration profiles |

## Operation Flow

1. **TMR2 runs continuously** - provides stable time base without interrupts
2. **Dominant axis OCx** - scheduled ahead of TMR2, triggers on compare match
3. **Dominant axis ISR** - executes step, updates Bresenham errors, schedules next pulse
4. **Subordinate axis scheduling** - OCx pairs set only when Bresenham requires a step
5. **Pulse generation** - Hardware compare generates precise step pulses
6. **Segment completion** - When all steps issued, planner loads next segment
7. **Dynamic axis swapping** - Dominant axis can change by switching tracking to different OCx

## Implementation Best Practices

### Timer Configuration
- Use 32-bit timer (TMR2:TMR3 pair) to avoid overflow issues
- Timer runs free continuously - **never stop or reset during operation**
- Configure for 10µs tick resolution for precise timing control

### Compare Register Management
- **Always use absolute timer values** for OCxR and OCxRS (not relative offsets)
- Schedule compare values **ahead of current TMR2 count** to prevent spurious interrupts
- Dominant axis: continuously track ahead by step interval
- Subordinate axes: schedule only when Bresenham error check requires step
- **Disable pulse generation**: Set `OCxR = OCxRS` (equal values prevent compare match pulse)
  - Critical for arc interpolation and Bresenham cycles
  - Prevents spurious pulses when axis is inactive
  - Use for motion completion and emergency stop scenarios

### Interrupt Service Routines
- Clear OCx interrupt flags **immediately** at ISR entry: `IFSxCLR = _IFSx_OCxIF_MASK`
- Keep ISRs **minimal**: schedule next pulse, update counters, exit quickly
- Bresenham calculations run in state machine, **not in ISR**

### Flow Control Options
Choose the method that best suits your motion control requirements:
1. **Disable OCx modules** when motion stops (preferred for dynamic control)
2. **Stop scheduling** ahead of TMR2 (timer keeps running, no pulses generated)
3. **Gate output pins** via GPIO control for emergency stop

### State Tracking
- Use modular structs to track segment state, axis positions, and error terms
- Maintain separate counters for steps issued vs. steps completed
- Track dominant axis designation for dynamic swapping capability

## Future Enhancements

### Velocity Profiling
**Status**: Under consideration for future implementation

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
