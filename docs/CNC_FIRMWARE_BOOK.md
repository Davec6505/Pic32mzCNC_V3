# Building a CNC Brain: A Complete Guide to the Pic32mzCNC_V3 Firmware

### How a Microcontroller Becomes a Machine Controller

---

**Author**: CNC Firmware Project  
**Hardware**: PIC32MZ2048EFH100 (200 MHz, FPU)  
**Firmware Branch**: `scurve_motion`  
**Audience**: Engineers, Makers, and Curious Minds — no prior CNC firmware experience required  

---

## Preface

Somewhere between a G-code command typed into a sender application and the satisfying sound of a stepper motor moving a cutting tool lies a world of mathematics, timing, protocol negotiation, and embedded engineering. This book documents every layer of that world as it exists in the `Pic32mzCNC_V3` firmware project.

You do not need to be an electrical engineer or a software developer to follow this book. Every concept is introduced from first principles in plain language before the code is shown. Where mathematics appears, it is explained geometrically before the formulae are written. Where code appears, its purpose is stated in plain English before any line is shown.

The goal is simple: by the time you finish reading, you should be able to open any file in this codebase — from the interrupt service routine that fires a stepper pulse to the flow control function that decides when to send an "ok" to a PC — and understand what it is doing and why.

---

## Table of Contents

1. [Chapter 1: The Big Picture — What Is a CNC Controller?](#chapter-1-the-big-picture)
2. [Chapter 2: The Hardware Stage — PIC32MZ and Friends](#chapter-2-the-hardware-stage)
3. [Chapter 3: The Language of Machines — G-Code and GRBL](#chapter-3-the-language-of-machines)
4. [Chapter 4: The Mathematics of Motion](#chapter-4-the-mathematics-of-motion)
5. [Chapter 5: Stepper Motors — Turning Pulses into Movement](#chapter-5-stepper-motors)
6. [Chapter 6: The Motion Pipeline — From G-Code to Step Pulse](#chapter-6-the-motion-pipeline)
7. [Chapter 7: Coordinate Systems and Kinematics](#chapter-7-coordinate-systems)
8. [Chapter 8: The Flow Control System — Keeping the Conversation Polite](#chapter-8-flow-control)
9. [Chapter 9: The Application State Machine — The Conductor](#chapter-9-the-application-state-machine)
10. [Chapter 10: Safety, Alarms, and Homing](#chapter-10-safety-alarms-homing)
11. [Chapter 11: Settings — The Machine's Personality](#chapter-11-settings)
12. [Appendix A: GRBL Settings Reference](#appendix-a-settings)
13. [Appendix B: File and Module Map](#appendix-b-module-map)
14. [Appendix C: Timing Reference](#appendix-c-timing)

---

## Chapter 1: The Big Picture — What Is a CNC Controller?

### 1.1 What CNC Actually Means

CNC stands for **Computer Numerical Control**. Strip away the acronym and the idea is simple: a computer moves a machine according to a list of numbers. Those numbers describe positions in space, speeds of travel, and actions like switching a spindle on or off.

Imagine you want to draw a perfect square on a piece of material with a pencil. You could do it by hand — but every corner would be slightly off, every line slightly crooked. Now imagine instead that you give the pencil to a robot and tell it: "Start at position (0, 0). Move to (60, 0) at 5000 mm per minute. Then move to (60, 60). Then to (0, 60). Then back to (0, 0)." The robot does this perfectly, every time, because it follows numbers rather than human muscle memory.

A CNC controller is the "brain" that takes those numbers and turns them into electrical signals that make the motors move.

### 1.2 The Chain from Idea to Cut

```
CAD Software     →   CAM Software    →   G-Code File   →   Sender (UGS)
(Draw the part)      (Plan the tool       (A text file       (Streams commands
                      path)                of commands)       over USB/serial)

         ↓
   CNC Controller (this firmware)
         ↓
   Stepper Motor Drivers
         ↓
   Stepper Motors → Linear Axes → Tool → Material
```

The firmware lives at the centre of this chain. It receives G-code text from a PC, parses it into machine intentions, plans smooth velocity profiles, and drives hardware timers that generate precisely timed electrical pulses to the motor drivers.

### 1.3 What This Firmware Does

The `Pic32mzCNC_V3` firmware implements:

| Capability | Detail |
|-----------|--------|
| GRBL v1.1 protocol | Industry-standard CNC communication protocol |
| 4-axis coordinated motion | X, Y, Z, A axes move simultaneously |
| S-curve velocity profiling | Smooth acceleration — no mechanical shock |
| Arc interpolation | Circular paths (G2/G3) broken into tiny straight segments |
| Work coordinate systems | Multiple reference frames (G54–G59) |
| G4 dwell (pause) | Timed pauses during a program |
| Homing cycle | Automatic reference position finding |
| Spindle control | PWM-based speed control |
| Persistent settings | 29 GRBL parameters stored in flash memory |
| Safety system | Hard/soft limits, emergency stop, alarm states |

### 1.4 The Two Worlds: Real-Time and Non-Real-Time

Everything in an embedded system lives in one of two worlds:

**The Real-Time World (Interrupt Service Routines — ISRs)**  
These are functions that run immediately when hardware signals arrive, interrupting everything else. They must finish in microseconds. Think of them like a fire alarm — when it goes off, everything else stops immediately and the alarm must be handled.

In this firmware, the most critical ISR fires 100,000 times per second (every 10 µs) and decides whether each axis should take a step in that moment.

**The Non-Real-Time World (Main Loop)**  
This is the ordinary program that runs continuously: reading UART characters from the PC, parsing G-code, computing motion plans, and managing state. It runs "in the gaps" between interrupts.

The genius of this firmware's architecture is in how these two worlds communicate cleanly without interfering with each other.

---

## Chapter 2: The Hardware Stage — PIC32MZ and Friends

### 2.1 The Microcontroller: PIC32MZ2048EFH100

The heart of the system is a **PIC32MZ2048EFH100** — a 32-bit microcontroller from Microchip. Let us unpack that name:

- **PIC32**: 32-bit processor based on the MIPS M-class architecture
- **MZ**: The "Majestic" family — Microchip's high-performance PIC32 line
- **2048**: 2,048 KB (2 MB) of internal flash program memory
- **EF**: Has a hardware Floating Point Unit (FPU) — important for smooth motion maths
- **H**: Extended temperature range
- **100**: 100-pin package

**Key specifications:**

| Parameter | Value |
|-----------|-------|
| CPU Speed | 200 MHz |
| Flash | 2 MB |
| RAM | 512 KB |
| Peripheral Bus (PBCLK3) | 50 MHz |
| FPU | Single-precision IEEE 754 |
| UART for G-code | UART3 (115200 baud) |
| Step timer | TMR4 (100 kHz fixed-rate) |
| Step pulse clear timer | TMR5 (one-shot, ~3 µs) |
| OC modules | OC1–OC4 (Output Compare — step pulse generation) |
| SPI2 | TMC5160 driver configuration |

### 2.2 Memory Layout

Flash memory is precious. The 2 MB is carefully partitioned:

```
Flash Address     Virtual Address    Size     Purpose
─────────────     ───────────────    ─────    ──────────────────────────────
0x1D000000        0x9D000000         1.87 MB  Application firmware (this code)
0x1D1F0000        0xBD1F0000         16 KB    GRBL Settings (NVM storage)
0x1D1F4000        0x9D1F4000         48 KB    MikroE USB HID Bootloader
0x1FC00000        0xBFC00000         12 KB    Boot flash (config words)
```

The **bootloader** occupies the top 48 KB. It is a USB bootloader that lets new firmware be flashed over USB without a hardware programmer. The firmware must never write into the bootloader region.

The **NVM settings area** at 0xBD1F0000 stores all 29 GRBL parameters persistently across power cycles. This is raw NOR flash — it must be erased in 16 KB pages before writing, and it must be accessed through the uncached KSEG1 address space (0xBD...) for NVM operations.

### 2.3 Clock Tree

```
External Crystal (24 MHz)
        ↓
   PLL × multiplier
        ↓
System Clock: 200 MHz
        ↓
    ┌───┴───────────────────┐
PBCLK1 (100 MHz)        PBCLK3 (50 MHz)
  CPU, caches              TMR4, TMR5, OC1–4
                           (peripheral timers)
```

The peripheral bus runs at **50 MHz**. TMR4 is configured with no prescaler (1:1) and PR4 = 499, which gives exactly 50,000,000 / 500 = **100,000 ticks per second** (100 kHz). This is the heartbeat of all step generation.

### 2.4 The Stepper Driver Interface

Stepper motors are not driven directly by the microcontroller. Instead, specialised driver ICs sit between the MCU and the motor. The MCU speaks to them in a simple three-wire language:

| Signal | Direction | Meaning |
|--------|-----------|---------|
| **STEP** | MCU → Driver | One pulse = one microstep |
| **DIR** | MCU → Driver | HIGH = forward, LOW = reverse |
| **EN** | MCU → Driver | LOW = driver enabled (motor energised) |

The firmware supports two driver families:

**DRV8825 / A4988 (traditional step-dir drivers)**  
Simple, robust, inexpensive. Configured by DIP switches or resistors on the driver board. The MCU has no visibility into driver status — it just pulses STEP and DIR. The single shared enable pin `EnXYZA` (connected to RE6) controls all four axes simultaneously.

**TMC5160 (intelligent SPI drivers)**  
A more sophisticated driver that can be configured via SPI2. The MCU can set current levels, chopper modes (StealthChop for silence, SpreadCycle for torque), and read back diagnostic information (temperature, stall detection). Future versions of this firmware plan to use StallGuard2 for sensorless homing.

### 2.5 GPIO Pin Assignments

```
Axis    STEP pin    DIR pin     LIMIT MIN    LIMIT MAX
────    ────────    ───────     ─────────    ─────────
 X      RD4         RD5         RA4          RA7
 Y      RD10        RD11        RD0          RE0
 Z      RE2         RE3         RD13         RE1
 A      RF3         RF4         RA6          RB1

Shared Enable:  RE6 (EnXYZA) — active LOW, all drivers

Spindle PWM:    OC8 / TMR6 (3.338 kHz)
LED1:           Status LED
LED2:           Visual debug (step activity)
```

---

## Chapter 3: The Language of Machines — G-Code and GRBL

### 3.1 What Is G-Code?

G-code is the oldest and most widely used language for CNC machines. It was developed in the 1950s at MIT. Despite its age, virtually every CNC machine in the world — from hobby routers to industrial machining centres — understands some dialect of G-code.

A G-code program is a text file. Each line is a command. Here is a real example from the test suite:

```gcode
G21              ; Use millimetres
G90              ; Absolute positioning mode
G17              ; Select XY plane for arcs
G1 F5000         ; Set feedrate to 5000 mm/min
G1 X60 Y0        ; Move to (60, 0)
G1 X60 Y60       ; Move to (60, 60)
G1 X0  Y60       ; Move to (0, 60)
G1 X0  Y0        ; Return to origin
G2 X40 Y30 I-10 J0   ; Clockwise arc to (40,30), centre offset (-10, 0)
```

**G-words** are modal commands — they set a mode that stays active until changed:
- `G0` — Rapid move (fastest safe speed)
- `G1` — Feed move (at programmed feedrate)
- `G2` — Clockwise arc
- `G3` — Counter-clockwise arc
- `G4 P1.0` — Dwell (pause) for 1.0 seconds
- `G90` — Absolute mode (coordinates from machine zero)
- `G91` — Relative mode (coordinates from current position)
- `G20/G21` — Inches / Millimetres

**M-words** are miscellaneous actions:
- `M3 S12000` — Spindle on clockwise at 12000 RPM
- `M5` — Spindle off
- `M7/M8` — Coolant on
- `M9` — Coolant off

**Parameter words** give values to the preceding command:
- `X Y Z` — Target position coordinates
- `I J K` — Arc centre offsets (always relative to start point)
- `F` — Feedrate in mm/min
- `S` — Spindle speed in RPM
- `P` — Dwell time in seconds

### 3.2 The GRBL Protocol

GRBL is an open-source CNC firmware that defined a communication protocol now used as the industry standard for hobby and semi-professional CNC machines. This firmware implements GRBL v1.1.

**The conversation between PC sender (UGS) and firmware:**

```
PC  →  Firmware:   G1 X60 Y0\n
Firmware → PC:     ok
PC  →  Firmware:   G1 X60 Y60\n
Firmware → PC:     ok
```

Every line sent to the firmware must receive exactly one "ok" back before the PC can send the next line in strict (sequential) mode. However, most senders (including UGS) operate in **streaming mode**, where they pipeline many commands — sending several lines without waiting for each ok individually, as long as they have not exceeded their credit window.

**Real-time characters** bypass the normal queue entirely and are handled immediately regardless of what else is happening:

| Character | Hex | Action |
|-----------|-----|--------|
| `?` | 0x3F | Request status report |
| `!` | 0x21 | Feed hold (pause) |
| `~` | 0x7E | Cycle start / resume |
| `Ctrl+X` | 0x18 | Soft reset |

**Status reports** (sent in response to `?`):
```
<Run|MPos:30.013,50.013,0.000|WPos:30.013,50.013,0.000|FS:5000,0>
```
- `Run` — machine state (Idle, Run, Hold, Alarm, etc.)
- `MPos` — machine position in mm
- `WPos` — work position (machine position minus WCS offset)
- `FS` — feed speed : spindle speed

**System commands** start with `$`:

| Command | Meaning |
|---------|---------|
| `$$` | Print all settings |
| `$H` | Run homing cycle |
| `$X` | Clear alarm |
| `$G` | Show modal state |
| `$I` | Build information |
| `$100=80` | Set parameter $100 to 80 |

### 3.3 The G-Code Parser (gcode_parser.c)

The G-code parser is the gateway between the PC world and the firmware's motion system. It:

1. Reads characters from UART3 (the USB-serial link to the PC)
2. Accumulates them into a line buffer until a newline `\n` or carriage return `\r` arrives
3. Routes the completed line to the appropriate handler
4. Sends "ok" (immediately or deferred) back to the PC

**Source file**: `srcs/gcode/gcode_parser.c`  
**Key functions**:

```c
void GCODE_Tasks(APP_DATA* appData, GCODE_CommandQueue* commandQueue);
// Called every APP_IDLE iteration. Reads UART, processes commands.

bool GCODE_GetNextEvent(GCODE_CommandQueue* cmdQueue, GCODE_Event* event);
// Pop one parsed event from the gcode queue for processing by app.c.

void GCODE_ConsumeEvent(GCODE_CommandQueue* cmdQueue);
// Called AFTER an event has been successfully handed to the motion system.

void GCODE_CheckDeferredOk(APP_DATA* appData, GCODE_CommandQueue* q);
// Called every iteration to release deferred "ok" responses when appropriate.
```

**The state machine inside GCODE_Tasks:**

```
GCODE_STATE_IDLE       — Waiting. Read UART, accumulate characters.
GCODE_STATE_CONTROL_CHAR   — Handle ?, !, ~, 0x18 immediately.
GCODE_STATE_QUERY_CHARS    — Handle $ system commands.
GCODE_STATE_GCODE_COMMAND  — Parse G/M/F/S/T commands, queue them.
```

**The command queue** (`GCODE_CommandQueue` in `incs/data_structures.h`) is a 64-slot circular buffer. Each slot holds one G-code command string (up to 80 characters). When the firmware is ready to execute a command, it pops one from this queue, parses it into a motion event, and sends it to the trajectory planner.

```c
// From data_structures.h
#define GCODE_MAX_COMMANDS 64
#define GCODE_BUFFER_SIZE  80

typedef struct {
    GCODE_Command commands[GCODE_MAX_COMMANDS];  // 64 command slots
    uint32_t head;            // where to write next (producer side)
    uint32_t tail;            // where to read next (consumer side)
    uint32_t count;           // number of items currently in queue
    uint32_t commands_consumed; // monotonic counter — incremented at every dequeue
    uint32_t maxMotionSegments; // trajectory queue size (for flow control reference)
} GCODE_CommandQueue;
```

The `commands_consumed` counter is critical to the flow control system explained in Chapter 8.

---

## Chapter 4: The Mathematics of Motion

### 4.1 Coordinate Geometry in the Machine

A CNC machine works in a three-dimensional coordinate space. Every point in space is described by three numbers: X (left–right), Y (front–back), and Z (up–down). This firmware adds a fourth axis A (usually rotary).

The distance from one point to another in 3D space is given by the **Pythagorean theorem** extended to three dimensions:

$$d = \sqrt{(x_2-x_1)^2 + (y_2-y_1)^2 + (z_2-z_1)^2}$$

A **unit vector** is a vector of length exactly 1.0 that describes only a *direction* without any magnitude. To find the unit vector from point A to point B:

$$\hat{u} = \frac{(B-A)}{|B-A|}$$

This is computed in `trajectory.c` and stored in each `SCurveMove`. It is used extensively — every axis component of a move is simply the unit vector component multiplied by the instantaneous speed:

$$v_{axis} = v_{total} \cdot u_{axis}$$

### 4.2 Bresenham's Line Algorithm

When the firmware generates steps, it cannot take fractional steps. Each step is a discrete pulse either forward or backward. But the desired path may require, say, 3 steps in X for every 2 steps in Y. How do we decide the precise order?

**Jack Bresenham** solved this problem in 1962 for drawing lines on pixel grids. The same algorithm is perfect for stepper motors.

Imagine you need to draw a line from (0,0) to (5,3) on a grid — 5 steps right and 3 steps up. The ideal line has slope 3/5 = 0.6. We want each step to be as close to the ideal line as possible.

Bresenham's insight: instead of computing floating-point divisions, keep an **error accumulator** (an integer). For each dominant axis step, add the subordinate axis's delta. When the accumulator exceeds the dominant axis delta, take a subordinate step and subtract.

```c
// Bresenham for two axes (simplified):
int error = dominant_delta / 2;  // start at midpoint

for (step = 0; step < dominant_delta; step++) {
    STEP(dominant_axis);   // always step dominant axis
    
    error += subordinate_delta;
    if (error >= dominant_delta) {
        STEP(subordinate_axis);  // step subordinate when needed
        error -= dominant_delta;
    }
}
```

This firmware uses Bresenham for **all four axes simultaneously**. One axis is the "dominant" (the one with the most steps). The other three are subordinate. The ISR advances the dominant axis on every tick and checks whether each subordinate needs a step.

In `stepper.c`, this is the Bresenham state:

```c
static volatile int32_t  error[NUM_AXIS]     = {0, 0, 0, 0};
static volatile uint32_t abs_delta[NUM_AXIS] = {0, 0, 0, 0};
static volatile int32_t  dominant_delta = 0;
static volatile E_AXIS   dominant_axis  = AXIS_X;
```

### 4.3 The DDS (Direct Digital Synthesis) Step Generator

The interrupt in this firmware fires at a **fixed rate of 100,000 Hz** (every 10 µs). This is different from older GRBL designs where the interrupt rate varied with speed. The advantage of a fixed rate is rock-solid timing and simpler ISR code.

Rather than changing the interrupt rate to control speed, this firmware uses **Direct Digital Synthesis**: a 32-bit accumulator per axis that fills up at a rate proportional to the desired speed. When it overflows (reaches 2³⁰ = 1,073,741,824), a step pulse is generated and the accumulator is reduced by the overflow threshold.

Think of it like a bucket with a hole. Water (the accumulator) fills up at a rate proportional to desired speed. Each time the bucket overflows, it output one step and subtracts the bucket-full amount.

```c
// From interpolator.c — simplified:
#define DDS_SCALE  (1 << 30)   // 2^30 = overflow threshold

// Every 10 µs tick:
for (each axis) {
    dds_acc[axis] += dds_inc[axis];    // fill the bucket
    
    if (dds_acc[axis] >= DDS_SCALE) {  // bucket overflowed?
        fire_step(axis);               // one step
        dds_acc[axis] -= DDS_SCALE;    // drain the bucket
    }
}
```

The increment `dds_inc[axis]` is calculated from the instantaneous velocity:

$$\text{dds\_inc} = v_{total} \cdot |u_{axis}| \cdot \text{steps\_per\_mm} \cdot \frac{\text{DDS\_SCALE}}{\text{TICK\_RATE}}$$

The maximum achievable step rate is `TICK_RATE / 2 = 50,000 steps/second` per axis, which at 80 steps/mm gives `50000 / 80 = 625 mm/s = 37,500 mm/min` — well above our configured 5000 mm/min limit.

### 4.4 S-Curve Velocity Profiling

**Why not just jump to full speed?**

If a motor is commanded to instantly go from rest to 5000 mm/min, it will stall. The mechanical inertia of the tool, carriage, and motor rotor is too great. Additionally, a sudden velocity step causes a violent jerk that rings through the frame, degrading surface finish and potentially skipping steps.

**Trapezoidal profiles** (the old GRBL approach) ramp up linearly, cruise, then ramp down linearly. This is a huge improvement over no ramping at all, but the transition points (where the ramp changes slope) are still sudden — there is an instantaneous change in acceleration, called a **jerk**.

**S-curve profiles** are jerk-limited. The acceleration itself is smoothly ramped, giving a profile that looks like an elongated "S" when you plot velocity versus time. This produces the smoothest possible motion with no abrupt changes at any order of derivative.

**The 7 phases of an S-curve:**

```
Velocity
  ^
  |           _______________
  |          /               \
  |         /                 \
  |        /                   \
  |       /                     \
  +──────/───────────────────────\──────→ Time
  |      P0   P1  P2  P3  P4  P5  P6
  |   (ramp  hold ramp cru ramp hold ramp)
  |   accel)     down    ise) up        down)
```

| Phase | Jerk | Acceleration | Description |
|-------|------|-------------|-------------|
| 0 | +J | 0 → a_peak | Ramp acceleration up |
| 1 | 0 | a_peak | Hold acceleration (if needed) |
| 2 | -J | a_peak → 0 | Ramp acceleration down → reach nominal speed |
| 3 | 0 | 0 | Cruise at nominal speed |
| 4 | -J | 0 → -a_peak | Ramp deceleration up |
| 5 | 0 | -a_peak | Hold deceleration |
| 6 | +J | -a_peak → 0 | Ramp deceleration down → reach exit speed |

The velocity at any moment within phase *i* at local time *dt* is:

$$v(dt) = v_i + a_i \cdot dt + \frac{1}{2} J_i \cdot dt^2$$

Where J is the jerk (rate of change of acceleration, in mm/s³). This is computed in `trajectory.c::TRAJECTORY_VelocityAt()` and called in the ISR once per 10 µs tick to update the DDS increments.

**Jerk magnitude** is derived from the axis settings:

$$J_{axis} = \frac{a_{max} \cdot jerk_{setting}}{2}$$

At default settings (X/Y: a_max = 5000 mm/s², jerk = 500):

$$J = \frac{5000 \cdot 500}{2} = 1{,}250{,}000 \text{ mm/s}^3$$

### 4.5 Junction Speed — Blending Consecutive Moves

When one move ends and the next begins, there is a corner. Rather than decelerating to zero, stopping, and reaccelerating, the firmware can carry some speed through the corner — the amount depends on how sharp the corner is.

GRBL's **centripetal acceleration** approximation computes the maximum safe junction speed by treating the corner as a circular arc with radius derived from the configured junction deviation `$11`:

```
                       v²
a_centripetal = k · ───────
                    junction_deviation
```

The unit vectors of the two moves are used in a **dot product** to find the cosine of the angle between them:

$$\cos\theta = -(\hat{u}_{prev} \cdot \hat{u}_{next})$$

(The negative sign is because the vectors point in opposite directions at a junction — one is the direction arriving, one is the direction leaving.)

From this, the maximum junction speed is:

$$v_j = \sqrt{a_{combined} \cdot J_{dev} \cdot \frac{1 - \cos\theta}{\cos\theta}}$$

This is implemented in `trajectory.c::TRAJECTORY_AddMove()`.

### 4.6 Arc Interpolation Mathematics

G-code arcs (G2/G3) describe circular paths. A microcontroller cannot machine a perfect circle — it approximates it with many tiny straight-line moves. The question is: how many?

**GRBL chord-deviation method (`$12` — arc tolerance):**

Rather than using a fixed segment length, the firmware computes the segment length that keeps the straight-line chord within a specified distance (tolerance) of the true circle:

```
      *──────────────*      ← chord (straight line segment)
     /│              │\
    / │              │ \
   /  │  deviation   │  \
  *   │              │   *  ← circle arc
      ↓              ↓
    tolerance = $12 setting
```

The geometry gives:

$$\theta_{half} = \arccos\left(1 - \frac{\text{tolerance}}{R}\right)$$

$$L_{segment} = 2R \sin(\theta_{half})$$

$$N_{segments} = \left\lceil \frac{L_{arc}}{L_{segment}} \right\rceil$$

With `$12 = 0.500 mm`:
- R = 20mm arc: L_segment ≈ 8.94 mm → only **4 segments** per quarter-circle
- R = 5mm arc: L_segment ≈ 4.36 mm → **3 segments** per quarter-circle

This is dramatically more efficient than a fixed 0.5 mm segment length (which would give 63 segments for a quarter-circle of radius 20 mm, nearly filling the trajectory queue!).

---

## Chapter 5: Stepper Motors — Turning Pulses into Movement

### 5.1 How a Stepper Motor Works

A stepper motor is an electric motor that does not spin continuously when powered. Instead, it moves in discrete **steps**. Each step is a fixed angular increment — typically 1.8° for a standard motor (200 steps per revolution).

Inside a stepper motor are multiple pairs of electromagnets (coils) arranged around the rotor. By energising these coils in sequence, the rotor is pulled from one magnetic alignment to the next. Each alignment is a "step."

```
Coil arrangement (simplified 4-phase stepper):

  Step 1: Coil A energised    →  Rotor aligns to A
  Step 2: Coil B energised    →  Rotor advances to B
  Step 3: Coil A reversed     →  Rotor advances to A'
  Step 4: Coil B reversed     →  Rotor advances to B'
  Step 5: Coil A energised    →  Full revolution step sequence repeats
```

### 5.2 Microstepping

A standard motor has 200 full steps per revolution. **Microstepping** driver ICs (like the DRV8825 or TMC5160) can divide each full step into smaller fractions by partially energising both coils simultaneously and adjusting the ratio of current. This gives dramatically smoother motion.

| Microstep Mode | Steps per Revolution | Angular Resolution |
|---------------|---------------------|-------------------|
| Full step (1x) | 200 | 1.8° |
| Half step (2x) | 400 | 0.9° |
| Eighth step (8x) | 1600 | 0.225° |
| Sixteenth step (16x) | 3200 | 0.1125° |
| 256x | 51,200 | 0.007° |

The firmware setting `$100` (steps per mm for X axis) accounts for microstepping:

$$\text{steps\_per\_mm} = \frac{\text{steps\_per\_rev} \times \text{microstep\_divisor}}{\text{distance\_per\_rev}}$$

For a typical setup with 16x microstepping, 200 steps/rev motor, and 25 mm/rev lead screw:

$$\text{steps\_per\_mm} = \frac{200 \times 16}{25} = 128 \text{ steps/mm}$$

### 5.3 The DRV8825 Driver

The DRV8825 is a discrete step/dir stepper driver that handles all the coil switching internally. The microcontroller only needs to:

1. Set the **DIR** pin to indicate direction
2. Pulse the **STEP** pin to advance one microstep
3. Control **EN** to enable or disable the driver (active LOW)

**Timing requirements** (from the DRV8825 datasheet):

| Parameter | Minimum | Notes |
|-----------|---------|-------|
| STEP pulse width (HIGH) | 1.9 µs | Must be wide enough for driver to recognise |
| DIR setup before STEP | 200 ns | Direction must be stable before stepping |
| STEP pulse LOW period | 1.9 µs | Recovery time between pulses |

This firmware uses **TMR5 in one-shot mode** to clear the step pin after exactly ~3 µs, satisfying the pulse width requirement with margin.

### 5.4 The TMC5160 — An Intelligent Driver

The TMC5160 is a significantly more capable driver. Where the DRV8825 simply switches coil currents, the TMC5160:

- Uses voltage-mode commutation (**StealthChop**) at low speeds — completely silent
- Switches to current-mode (**SpreadCycle**) at high speeds for maximum torque
- Reports driver temperature, short-circuit faults, and open-load conditions via SPI
- Provides **StallGuard2** — a load-sensing system that can detect when the motor stalls, enabling sensorless homing

**SPI Configuration** (40-bit frames):

```
Byte 0:  Bit 7 = Write(1)/Read(0),  Bits 6:0 = Register address
Bytes 1–4: 32-bit data, MSB first

Response from previous transaction returned during this transaction.
```

Key registers configured at startup:

| Register | Address | Purpose |
|---------|---------|---------|
| GCONF | 0x00 | Global config — enables StealthChop mode |
| IHOLD_IRUN | 0x10 | Hold current and run current (0–31 scale) |
| CHOPCONF | 0x6C | Chopper config — microstep resolution |
| PWMCONF | 0x70 | StealthChop PWM parameters |
| DRVSTATUS | 0x6F | Read driver status (stall, overtemp, shorts) |

In the firmware, TMC5160 support is conditionally compiled using `#ifdef HAS_TMC5160_AXIS`. The driver selection per axis is set in `common.h`:

```c
#define AXIS_X_DRIVER   DRIVER_DRV8825   // or DRIVER_TMC5160
#define AXIS_Y_DRIVER   DRIVER_DRV8825
```

### 5.5 The Stepper Enable Pin — An Important Hardware Constraint

**This is a critical hardware fact that every developer on this project must know:**

There is only **one shared enable pin** — `EnXYZA` connected to RE6. There are no separate per-axis enable pins. When you enable the steppers, all four axes are energised simultaneously. When you disable, all four lose hold torque simultaneously.

This is reflected throughout the code:

```c
// From utils.h — these map to the single EnXYZA hardware pin
static inline void enable_all_set(void)   { EnXYZA_Set(); }   // disable all (active LOW)
static inline void enable_all_clear(void) { EnXYZA_Clear(); } // enable all
```

Any code that tries to enable/disable axes individually is **not supported** by the PCB hardware.

---

## Chapter 6: The Motion Pipeline — From G-Code to Step Pulse

### 6.1 Overview of the Pipeline

The journey from a G-code command arriving over UART to a step pulse on a GPIO pin passes through four major software stages:

```
UART Input
    ↓
[1] GCODE_Tasks()          — Parse text, queue commands
    ↓ GCODE_CommandQueue (64 slots)
[2] MOTION_ProcessGcodeEvent()   — Interpret commands, plan geometry
    ↓ SCurveMove trajectory queue (64 slots)
[3] INTERPOLATOR_LoadMove()      — Pre-compute DDS parameters
    ↓ Fixed-rate 100 kHz ISR
[4] ISR: DDS accumulate → fire step → clear step (TMR5)
    ↓
GPIO: STEP pin pulse → Driver → Motor
```

Each stage has its own queue and its own pace. The firmware is non-blocking throughout — nothing waits for anything else. It is a pipeline, not a series of sequential operations.

### 6.2 Stage 1 — The G-Code Parser (gcode_parser.c)

As explained in Chapter 3, the parser reads UART3, accumulates characters, and adds parsed commands to the `GCODE_CommandQueue`. This queue holds text commands that have not yet been handed to the motion system.

**Key point**: the GCODE queue stores text strings, not motion plans. Actual geometry computation happens in Stage 2.

```c
// Each slot in the gcode queue is just a text buffer:
typedef struct {
    char command[GCODE_BUFFER_SIZE];   // e.g. "G1X60Y0"
} GCODE_Command;
```

### 6.3 Stage 2 — The Motion Bridge (motion_bridge.c)

`motion_bridge.c` is where G-code commands are translated into physical motion descriptions. It is called from `app.c`'s event loop whenever the trajectory queue has space and the gcode queue has something to process.

For a **linear move** (G0 / G1):

```c
// Simplified from motion_bridge.c GCODE_EVENT_LINEAR_MOVE:

// 1. Guard: back off if trajectory queue nearly full
if (TRAJECTORY_QueueCount() >= TRAJ_QUEUE_SIZE - 2) {
    return false;  // try again next iteration
}

// 2. Compute start position (from modal state — NOT live position)
CoordinatePoint start = s_planned_position;

// 3. Convert work coords to machine coords
CoordinatePoint end_mach = KINEMATICS_WorkToMachine(target_work);

// 4. Add to trajectory planner
bool added = TRAJECTORY_AddMove(start, end_mach, feedrate, entry_speed, exit_speed);

// 5. Update planned position for next move
s_planned_position = end_mach;

// 6. Trigger planning recalculation
TRAJECTORY_Recalculate();

// 7. If interpolator is idle, start it immediately
if (!INTERPOLATOR_IsActive()) {
    SCurveMove mv;
    if (TRAJECTORY_GetNextMove(&mv)) {
        STEPPERS_Enable();
        INTERPOLATOR_LoadMove(&mv);
    }
}
```

For an **arc move** (G2 / G3), the bridge breaks the arc into many small linear moves. Key parameters from the G-code:

- **I, J**: Centre offsets from start point (always relative)
- Start and end points in machine coordinates
- Feedrate and arc direction

Segment count is determined by the chord-deviation formula (Section 4.6). Each segment becomes one `TRAJECTORY_AddMove` call.

**The planned position** `s_planned_position` is crucial. When UGS pipelines 20 commands before executing any of them, the firmware must compute where each command *will end up*, not where the machine *is right now*. `s_planned_position` tracks the expected end of the last planned move, serving as the start for the next planned move.

### 6.4 Stage 3 — The Trajectory Planner (trajectory.c)

The trajectory planner holds a queue of 64 `SCurveMove` entries. Each entry describes one complete move with full S-curve velocity profile pre-computed:

```c
// SCurveMove (from trajectory.h):
typedef struct {
    float  millimeters;            // distance of this move
    float  unit_vec[NUM_AXIS];     // normalised direction vector
    float  nominal_speed;          // target cruise speed (mm/s)
    float  entry_speed;            // speed at start of move
    float  exit_speed;             // speed at end of move
    float  max_entry_speed;        // max permissible entry speed
    
    // S-curve phase timing (7 phases):
    float  t[8];                   // cumulative time at each phase boundary (s)
    float  v[7];                   // velocity at start of each phase
    float  a[7];                   // acceleration at start of each phase
    float  jk[7];                  // jerk (constant within each phase)
    
    float  steps_per_mm[NUM_AXIS]; // for step counting
} SCurveMove;
```

After each move is added, `TRAJECTORY_Recalculate()` runs the **look-ahead planner**:

**Reverse pass** — scanning from newest to oldest move, it propagates entry speed limits backwards. A move cannot enter faster than physics and geometry allow:

$$v_{entry}^2 \leq v_{next\_entry}^2 + 2 \cdot a_{combined} \cdot L_{move}$$

**Forward pass** — scanning oldest to newest, it ensures no move accelerates faster than the previous exit speed allows:

$$v_{entry} \leq \sqrt{v_{prev\_exit}^2 + 2 \cdot a_{combined} \cdot L_{prev}}$$

After both passes, `KINEMATICS_RecalculateTrapezoid()` (now `TRAJECTORY_SolveSCurve()` in this implementation) computes the full 7-phase S-curve times and jerk values for each move.

### 6.5 Stage 4 — The Interpolator (interpolator.c)

The interpolator is the bridge between the planning world (floating-point, main loop) and the hardware world (integer, ISR). It converts an `SCurveMove` into a sequence of 100,000 DDS ticks.

**LoadMove** (called from main loop):

```c
void INTERPOLATOR_LoadMove(const SCurveMove* mv)
{
    // Disable ISR while writing shared state
    __builtin_disable_interrupts();
    
    active_move = *mv;   // copy entire move profile
    
    // Pre-compute per-axis scale: steps_per_mm / TICK_RATE * DDS_SCALE
    for (int i = 0; i < NUM_AXIS; i++) {
        axis_scale[i] = mv->steps_per_mm[i] / TICK_RATE_HZ * DDS_SCALE_F;
    }
    
    // Compute total ticks needed: millimeters / avg_speed * TICK_RATE
    ticks_remaining = (uint32_t)(mv->millimeters / mv->entry_speed * TICK_RATE_HZ);
    
    // Initialise DDS from entry velocity
    recompute_increments(mv->entry_speed);
    
    interp_active = true;
    __builtin_enable_interrupts();
    
    TMR4_Start();   // Start the 100 kHz timer
}
```

**The ISR** (runs every 10 µs):

```c
// Simplified ISR (OC1_ISR in interpolator.c):
void TMR4_Callback(void) {
    // 1. Clear step pins from PREVIOUS tick
    CLEAR_ALL_STEP_PINS();
    
    // 2. Update velocity from S-curve profile
    elapsed_s = (float)(total_ticks - ticks_remaining) / TICK_RATE_HZ;
    float v_now = TRAJECTORY_VelocityAt(&active_move, elapsed_s);
    
    // 3. Update DDS increments
    for (int ax = 0; ax < NUM_AXIS; ax++) {
        dds_inc[ax] = (int32_t)(v_now * fabsf(unit_vec[ax]) * axis_scale[ax]);
    }
    
    // 4. Accumulate DDS and fire steps
    for (int ax = 0; ax < NUM_AXIS; ax++) {
        dds_acc[ax] += dds_inc[ax];
        if (dds_acc[ax] >= DDS_SCALE) {
            FIRE_STEP(ax, direction[ax]);   // GPIO HIGH
            steps_fired[ax] += direction[ax];
            dds_acc[ax] -= DDS_SCALE;
        }
    }
    
    // 5. Decrement tick counter
    if (--ticks_remaining == 0) {
        // Force any remaining steps for position accuracy
        force_final_steps();
        interp_active = false;
        move_complete = true;
    }
}
```

**ISR timing budget (200 MHz CPU, 10 µs = 2000 cycles):**

| Operation | Cycles |
|-----------|--------|
| Clear step pins | ~20 |
| VelocityAt() — fast path | ~120 |
| 4× DDS accumulate + possible step | ~80 |
| Tick countdown + stop | ~20 |
| **Total** | **~240 (12% of budget)** |

With only 12% of the ISR budget used, there is substantial headroom for future enhancements.

### 6.6 The Step Pulse Width — TMR5 One-Shot

When a step needs to be fired, the ISR sets the STEP pin HIGH. The pin must stay HIGH for at least 1.9 µs (DRV8825 requirement). TMR5 is configured as a one-shot timer. When fired, it counts up to a preset value (corresponding to ~3 µs) and then calls its callback:

```c
void TMR5_PulseWidthCallback(uint32_t status, uintptr_t context) {
    // Clear all step pins after pulse width
    AXIS_StepClear(AXIS_X);
    AXIS_StepClear(AXIS_Y);
    AXIS_StepClear(AXIS_Z);
    AXIS_StepClear(AXIS_A);
}
```

The step pulse clear in the next ISR tick also serves as a backup to guarantee pins are cleared before the next tick's DDS evaluation.

### 6.7 Position Tracking

Every step fired by the ISR is counted:

```c
// In interpolator.c ISR:
if (step_fired) {
    if (direction == +1) AXIS_IncrementSteps(ax);
    else                 AXIS_DecrementSteps(ax);
}
```

`AXIS_IncrementSteps` writes directly to `*g_axis_settings[axis].step_count` — a pointer wired to the `StepperPosition` struct. The current machine position in mm is always:

$$\text{pos\_mm}[axis] = \frac{\text{step\_count}[axis]}{\text{steps\_per\_mm}[axis]}$$

This is computed by `STEPPER_GetPosition()` and used in status reports.

---

## Chapter 7: Coordinate Systems and Kinematics

### 7.1 Why Multiple Coordinate Systems?

A CNC machine has more than one notion of "where things are."

**Machine coordinates (MPos)**: The absolute position of the tool relative to the machine's physical home position. This is the raw count of steps from the home switches. It never changes unless you home the machine or manually override it.

**Work coordinates (WPos)**: The position relative to a **work offset** — usually the corner or centre of the part you want to machine. You can set this to zero wherever you park your tool, making G-code programs portable between different setups.

**G92 offset**: A temporary additional shift, useful for fine adjustments during a job.

The relationship is:

$$\text{WPos} = \text{MPos} - \text{WCS\_offset} - \text{G92\_offset}$$

$$\text{MPos} = \text{WPos} + \text{WCS\_offset} + \text{G92\_offset}$$

### 7.2 Work Coordinate Systems (G54–G59)

GRBL supports six named work coordinate systems:

| G-code | Index | Typical Use |
|--------|-------|-------------|
| G54 | 0 | Default coordinate system |
| G55 | 1 | Second fixture / second side |
| G56 | 2 | Third position |
| G57 | 3 | Fourth position |
| G58 | 4 | Fifth position |
| G59 | 5 | Sixth position |

Each WCS has its own X, Y, Z offset stored in NVM flash. Switching with `G54`, `G55`, etc. applies that WCS's offset to all subsequent coordinate commands.

Setting a WCS offset uses `G10`:

```gcode
G10 P0 L20 X0 Y0 Z0    ; Set current position as (0,0,0) in current WCS
G10 L2 P1 X10 Y20 Z0   ; Set G54 offset so tool is at work (10,20,0)
```

`G92` sets a *temporary* offset without touching the WCS:

```gcode
G92 X0 Y0 Z0    ; Call current position "work zero" without touching WCS offset
```

### 7.3 The Kinematics Module (kinematics.c)

The kinematics module handles all coordinate transformations. It is intentionally simple — just addition and subtraction of offsets:

```c
// machine_pos = work_pos + wcs_offset + g92_offset
CoordinatePoint KINEMATICS_WorkToMachine(CoordinatePoint work_pos)
{
    CoordinatePoint machine;
    for (int i = 0; i < NUM_AXIS; i++) {
        machine.coordinate[i] = work_pos.coordinate[i]
                               + g_wcs.offset.coordinate[i]
                               + g92_offset[i];
    }
    return machine;
}

// work_pos = machine_pos - wcs_offset - g92_offset
CoordinatePoint KINEMATICS_MachineToWork(CoordinatePoint machine_pos)
{
    CoordinatePoint work;
    for (int i = 0; i < NUM_AXIS; i++) {
        work.coordinate[i] = machine_pos.coordinate[i]
                            - g_wcs.offset.coordinate[i]
                            - g92_offset[i];
    }
    return work;
}
```

**Why does this matter for the motion pipeline?**

All G-code coordinates arrive in *work space*. The motion planner works in *machine space* (step counts). Every move conversion involves:

```
G-code coordinates (work)
        ↓
  KINEMATICS_WorkToMachine()
        ↓
Machine coordinates (mm)
        ↓
  × steps_per_mm
        ↓
Step counts (integer)
        ↓
  DDS increment computation
        ↓
Step pulses
```

### 7.4 The CoordinatePoint Structure

All positions in this firmware are stored as:

```c
typedef struct {
    float coordinate[NUM_AXIS];  // [0]=X, [1]=Y, [2]=Z, [3]=A
} CoordinatePoint;
```

Using an array rather than named fields `{float x, y, z, a}` allows iteration over axes in loops — cleaner and less error-prone than four separate variables.

Helper macros enable safe, readable access:

```c
// Get one axis from a CoordinatePoint:
float x_pos = GET_COORDINATE_AXIS(&position, AXIS_X);

// Set one axis:
SET_COORDINATE_AXIS(&target, AXIS_Y, 30.0f);

// Add a delta:
ADD_COORDINATE_AXIS(&target, current_axis, step_distance);
```

---

## Chapter 8: The Flow Control System — Keeping the Conversation Polite

### 8.1 The Problem of Pipelining

A G-code sender like UGS does not wait for each "ok" before sending the next command. Instead, it maintains a **credit window** — it sends commands in advance until the window is fill, then pauses to wait for "ok" responses. When "ok" arrives, one credit is freed and another command can be sent.

This means at any given moment, many commands may already be in the firmware's G-code queue. If the firmware sends "ok" too eagerly (before the queue has room), UGS overfills the queue and commands are dropped. If the firmware withholds "ok" too aggressively, UGS's credit window drains and the machine starves of commands — it will idle mid-program.

### 8.2 The Two-Queue Architecture

This firmware has two queues between the PC and the stepper ISR:

```
PC/UGS → [GCODE Queue: 64 slots] → [TRAJECTORY Queue: 64 slots] → ISR
```

**GCODE Queue** (main flow control gate):
- Holds raw text commands
- Flow control is based entirely on this queue's depth
- HIGH_WATER mark: 48 out of 64 slots

**TRAJECTORY Queue**:
- Holds computed S-curve move profiles
- Managed independently — the motion bridge backs off when it reaches 62/64 full
- Does NOT gate UGS flow control

### 8.3 SendOrDeferOk — The First Decision

When a G-code line arrives and is successfully queued, `SendOrDeferOk` decides whether to send "ok" immediately or defer it:

```c
static void SendOrDeferOk(APP_DATA* appData, GCODE_CommandQueue* q)
{
    // Dwell takes priority: always defer during G4
    if (MOTION_IsDwellActive()) {
        okPendingCount++;
        return;
    }

    // Below HIGH_WATER: send immediately (keep UGS filling the pipeline)
    // At or above: defer (don't let the queue overfill)
    if (q->count >= GCODE_QUEUE_HIGH_WATER) {
        okPendingCount++;     // deferred – will send when commanded
    } else {
        UART_SendOK();        // immediate – UGS can send another command now
    }
}
```

The `okPendingCount` counter accumulates deferred "ok" responses. These will be sent later as the queue drains.

### 8.4 GCODE_CheckDeferredOk — Releasing Deferred OKs

This function is called at the start of every `GCODE_Tasks()` iteration. It checks how many commands have been consumed since the last call, and releases that many deferred "ok" responses:

```c
void GCODE_CheckDeferredOk(APP_DATA* appData, GCODE_CommandQueue* q)
{
    static uint32_t prev_consumed = 0;
    uint32_t curr_consumed = q->commands_consumed;

    // Dwell: never release during G4
    if (MOTION_IsDwellActive()) return;

    // If nothing pending, keep prev_consumed sync'd
    if (okPendingCount == 0) {
        prev_consumed = curr_consumed;
        return;
    }

    if (curr_consumed != prev_consumed) {
        // Commands were consumed — release one ok per command
        uint32_t freed = curr_consumed - prev_consumed;
        prev_consumed = curr_consumed;
        while (freed > 0 && okPendingCount > 0) {
            UART_SendOK();
            okPendingCount--;
            freed--;
        }
    } else if (q->count == 0 && okPendingCount > 0) {
        // Queue fully drained — flush all remaining
        while (okPendingCount > 0) {
            UART_SendOK();
            okPendingCount--;
        }
    }
}
```

### 8.5 Why the commands_consumed Counter Was Critical

**The Bug (commit 28a3dc7 and earlier):**

The original implementation used `q->count` (the current depth of the queue) and detected releases by checking `curr < prev_count`. The logic: if the queue is shallower than it was last call, some commands were consumed, so release that many OKs.

**The flaw**: At moderate throughput, the planner was consuming commands from the gcode queue at nearly the same rate as UGS was adding new ones. The queue depth stayed roughly constant at ~48 slots. `curr < prev_count` was never true. `okPendingCount` never drained. UGS's window exhausted. Machine stopped mid-file.

```
Time →
Queue count: ...48...48...48...48...48...48... (never goes down!)
                                               UGS starves here ↑
```

**The Fix (commit df9d197):**

A `commands_consumed` integer counter is incremented every time a command leaves the queue, regardless of whether new ones are entering simultaneously. The deferred-ok logic reads this counter's *delta* — which correctly counts actual exits independent of concurrent entries.

```
Time →
commands_consumed: ...62...63...64...65...66...67... (always goes up!)
             delta: ...  1 ...1 ...1 ...1 ...1 ...
OKs released:      ...  1   1   1   1   1   1  (correct 1-to-1 release)
```

### 8.6 The HIGH_WATER and LOW_WATER Marks

```c
#define GCODE_QUEUE_HIGH_WATER  48  // defer OKs here (queue 3/4 full)
#define GCODE_QUEUE_LOW_WATER   16  // (retained in code, not currently used)
```

With a 64-slot gcode queue and HIGH_WATER=48:
- When queue < 48: firmware sends ok immediately — UGS keeps sending
- When queue ≥ 48: firmware defers ok — UGS's send window pauses
- As the planner consumes commands, deferred oks are released — UGS resumes

This creates a natural throttle that keeps both sides busy without overflowing.

### 8.7 G4 Dwell Flow Control

A `G4 P1.0` command (dwell for 1 second) requires special handling. The dwell is implemented as:

1. Queue the G4 event normally
2. When the motion bridge sees a G4 event, wait for the trajectory queue to drain to zero (all motion completed)
3. Start a core-timer countdown for the dwell duration
4. During the countdown, suppress all deferred-ok releases
5. After the dwell completes, resume normal ok flow

If OKs were released during the dwell, UGS might send the next batch of commands before the dwell finishes, potentially executing cutting moves before the paused operation is complete.

---

## Chapter 9: The Application State Machine — The Conductor

### 9.1 State Machines Explained

A state machine is a program that can be in exactly one **state** at a time. Each state describes what the program is doing and what events can move it to another state.

Think of a traffic light: it can be in state RED, state YELLOW, or state GREEN. Events (timers) cause transitions between states. The light cannot be simultaneously RED and GREEN.

This firmware's application is structured as a state machine in `srcs/app.c`.

### 9.2 Application States

```c
typedef enum {
    APP_CONFIG = 0,       // Initial hardware configuration
    APP_LOAD_SETTINGS,    // Read settings from NVM flash
    APP_GCODE_INIT,       // Initialise G-code parser
    APP_IDLE,             // Normal operation (most time spent here)
    APP_ALARM,            // Emergency stop / hard limit triggered
    APP_WAIT_FOR_CONFIGURATION,   // Unused
    APP_DEVICE_ATTACHED,          // USB device attached
    APP_WAIT_FOR_DEVICE_ATTACH,   // Waiting for USB
    APP_DEVICE_DETACHED,          // USB disconnected
    APP_ERROR             // Unrecoverable error
} APP_STATES;
```

**State transition diagram (simplified):**

```
APP_CONFIG
    ↓ (hardware initialised)
APP_LOAD_SETTINGS
    ↓ (NVM read complete)
APP_GCODE_INIT
    ↓ (parser ready)
APP_IDLE ←────────────────────┐
    ↓ (limit triggered)        │
APP_ALARM                      │ (soft reset / $X)
    ↓ ($X command)             │
    └──────────────────────────┘
```

### 9.3 The APP_CONFIG State

Called exactly once at startup. Sets up all hardware peripherals:

- Configures GPIO (step pins, direction pins, enable pins, limit switches)
- Starts TMR4 (the 100 kHz base clock — 50 MHz / 500 = 100 kHz)
- Starts TMR5 (step pulse width timer)
- Configures UART3 (115200 baud, ring buffers set to 512 RX / 1024 TX bytes)
- Initialises the stepper module — wires step-count pointers to g_axis_settings
- Initialises the trajectory and interpolator modules

**Critical timing note**: NVM (flash settings) must NOT be read during APP_CONFIG. The peripheral buses are not fully stable yet. Settings reading is deferred to APP_LOAD_SETTINGS.

### 9.4 The APP_IDLE State — The Heart of the System

The overwhelming majority of time is spent in APP_IDLE. Every call to `APP_Tasks()` from `main.c`'s infinite loop lands here. The sequence within one APP_IDLE call:

```c
case APP_IDLE:
    // 1. Check for completed moves and load next from trajectory queue
    MOTION_Tasks(&appData);

    // 2. Check for completed arc generation (feeds trajectory queue)
    //    (handled inside MOTION_Tasks)

    // 3. Process one G-code command from gcode queue → trajectory queue
    //    (one event per call — non-blocking)
    if (GCODE_GetNextEvent(&cmdQ, &event)) {
        bool consumed = MOTION_ProcessGcodeEvent(&appData, &event, &cmdQ);
        if (consumed) GCODE_ConsumeEvent(&cmdQ);
    }

    // 4. Check deferred OKs and release as appropriate
    GCODE_CheckDeferredOk(&appData, &cmdQ);

    // 5. Read UART, process G-code text, queue new commands
    GCODE_Tasks(&appData, &cmdQ);

    // 6. Check hard limits (if enabled)
    MOTION_UTILS_CheckHardLimits(&appData);

    break;
```

**Why one event per call?**

If the firmware processed all queued events in a single call, it might take many milliseconds for long arc programs (which generate many trajectory moves). During that time, the UART buffer could overflow and characters could be lost. Processing one event per call keeps the loop responsive.

### 9.5 The APP_ALARM State

Any safety-critical event (hard limit triggered, estop detected, probe fault) transitions the machine to APP_ALARM. In this state:

- All motor output is disabled immediately
- The GCODE queue is cleared
- No new motion commands are accepted
- UGS shows `<Alarm|...>` status

Clearing the alarm requires either:
- `$X` command (unlock alarm — use with caution if triggered by limit switch)
- Soft reset (`Ctrl+X`) — resets everything and returns to IDLE

---

## Chapter 10: Safety, Alarms, and Homing

### 10.1 Limit Switches

Limit switches are simple electromechanical sensors at the physical travel limits of each axis. When triggered, they indicate the tool has reached the end of safe travel. This firmware supports:

- **Hard limits** (`$21=1`): Immediately triggers APP_ALARM when any limit fires during motion
- **Soft limits** (`$20=1`): Checks G-code targets against configured travel limits (`$130`–`$143`) before executing — rejects moves that would exceed limits

Limit switch logic with inversion:
```c
static inline bool LIMIT_GetMin(E_AXIS axis) {
    bool raw = g_limit_config[axis].limit.GetMin();
    bool inverted = (invert_mask >> axis) & 0x01;
    return raw ^ inverted;  // XOR: flip logic if invert bit set
}
```

The `^` (XOR) operator elegantly handles both active-high and active-low switches. Setting `$5=1` inverts all limit pins.

### 10.2 Homing Cycle ($H)

Homing is the process of moving each axis to its limit switch to establish a known machine zero. The GRBL `$H` command triggers a four-phase process:

**Phase 1 — SEEK (fast):** Move axis at seek rate (`$25`) toward the limit switch until it triggers. This is the coarse approach. Typical speed: 2000 mm/min.

**Phase 2 — BACK OFF:** Move away from the switch by a small amount, so the switch releases.

**Phase 3 — LOCATE (slow):** Move toward the switch again at feed rate (`$24`) for a precise trigger. Typical speed: 500 mm/min. Hardware debouncing (`$26` = 25 ms) ensures false triggers are ignored.

**Phase 4 — PULL-OFF:** Move away from the switch by the pull-off distance (`$27`, default 2.0 mm) so the machine starts from a stable, non-triggered position.

After homing, machine position is set to zero on the homed axes.

### 10.3 Soft Reset (Ctrl+X)

The soft reset is the firmware's "start fresh" command. It:

1. Stops all interrupt-driven motion immediately
2. Clears the trajectory queue — all planned moves are discarded
3. Clears the gcode command queue
4. Resets all modal state (distance mode, units, WCS index)
5. Clears `okPendingCount` to zero
6. Resets `prev_consumed` tracker for flow control
7. Re-enables steppers (hardware leave them powered)
8. Sends the GRBL startup banner: `Grbl 1.1h ['$' for help]`
9. Returns to IDLE state

UGS triggers soft reset at the start of every job and when the user clicks the reset button.

---

## Chapter 11: Settings — The Machine's Personality

### 11.1 How Settings Work

The firmware stores 29 GRBL parameters (and several extended parameters) in a dedicated 16 KB NVM flash page at address `0xBD1F0000`. This survives power cycles.

Settings are accessed uniformly through the `SETTINGS_GetCurrent()` function, which returns a pointer to the current `CNC_Settings` structure loaded into RAM at startup.

```c
typedef struct {
    // Standard GRBL parameters
    float step_pulse;              // $0  — pulse time in µs
    uint32_t step_idle_delay;      // $1  — motor idle delay ms
    uint8_t step_pulse_invert;     // $2  — step invert mask
    uint8_t step_direction_invert; // $3  — direction invert mask
    uint8_t step_enable_invert;    // $4  — enable invert
    uint8_t limit_pin_invert;      // $5  — limit pin invert
    uint8_t probe_pin_invert;      // $6  — probe pin invert
    uint8_t status_report_mask;    // $10 — status report options
    float junction_deviation;      // $11 — junction deviation mm
    float arc_tolerance;           // $12 — arc chord deviation mm
    // ... (29 parameters total)
    float steps_per_mm[NUM_AXIS];  // $100–$103
    float max_rate[NUM_AXIS];      // $110–$113 mm/min
    float acceleration[NUM_AXIS];  // $120–$123 mm/s²
    float max_travel[NUM_AXIS];    // $130–$143 mm
    float jerk[NUM_AXIS];          // (extended — jerk in mm/s³ units)
} CNC_Settings;
```

### 11.2 Reading and Writing

**At startup** (APP_LOAD_SETTINGS):
```c
if (SETTINGS_LoadFromFlash(SETTINGS_GetCurrent())) {
    // loaded successfully from NVM
} else {
    // NVM contained invalid data — defaults were already loaded
}
```

**When a `$100=80` command arrives:**
```c
// In gcode_parser.c $ command handler:
s->steps_per_mm[AXIS_X] = 80.0f;
SETTINGS_SaveToFlash(s);   // Erase page, write row — ~15 ms
UART_SendOK();
```

NVM writes require erasing an entire 16 KB page first, then writing in 2 KB rows. This takes approximately 15 ms. The firmware uses a **callback pattern** via Harmony's NVM driver to do this without blocking the main loop.

```c
// Non-blocking NVM pattern:
static volatile bool xferDone = false;

static void nvm_callback(uintptr_t context) {
    xferDone = true;
}

void SETTINGS_SaveToFlash(CNC_Settings* s) {
    NVM_CallbackRegister(nvm_callback, 0);
    xferDone = false;
    NVM_PageErase(SETTINGS_FLASH_ADDRESS);
    while (!xferDone);    // wait for erase
    xferDone = false;
    NVM_RowWrite((uint32_t*)s, SETTINGS_FLASH_ADDRESS);
    while (!xferDone);    // wait for write
}
```

### 11.3 The Startup Banner and Why It Matters

After every power-on or soft reset, the firmware sends:

```
Grbl 1.1h ['$' for help]
```

This is **not optional**. G-code senders like UGS, Candle, and bCNC identify the controller by this banner. They parse it to confirm they are talking to a GRBL-compatible device, and they use the reset event (which triggers the banner) to synchronise their internal state. If the banner is missing, missing field, or has extra characters, senders may refuse to connect or behave erratically.

The banner is configured in `common.h`:
```c
#define GRBL_FIRMWARE_VERSION   "1.1h"
#define STARTUP_BANNER_STRING   "Grbl " GRBL_FIRMWARE_VERSION " ['$' for help]\r\n"
```

---

## Appendix A: GRBL Settings Reference

| Setting | Parameter | Default | Description |
|---------|-----------|---------|-------------|
| `$0` | Step pulse time | 10 µs | Duration of STEP pin HIGH pulse |
| `$1` | Step idle delay | 25 ms | Time before steppers power down after motion |
| `$2` | Step invert mask | 0 | Bitmask to invert step signals per axis |
| `$3` | Dir invert mask | 0 | Bitmask to invert DIR signals per axis |
| `$4` | Enable invert | 0 | Invert EN pin polarity (0=active LOW) |
| `$5` | Limit invert | 0 | Invert limit switch logic |
| `$6` | Probe invert | 0 | Invert probe pin logic |
| `$10` | Status options | 3 | Bit 0=MPos, bit 1=WPos in status reports |
| `$11` | Junction deviation | 0.010 mm | Controls corner blending speed |
| `$12` | Arc tolerance | 0.500 mm | Maximum chord deviation for arc segments |
| `$13` | Report inches | 0 | 0=mm, 1=inches in status reports |
| `$20` | Soft limits | 0 | Enable software travel limits |
| `$21` | Hard limits | 0 | Enable hardware limit switches |
| `$22` | Homing enable | 1 | Enable `$H` homing command |
| `$23` | Homing dir mask | 0 | Bitmask: 1=home in positive direction |
| `$24` | Homing feed rate | 500 mm/min | Slow precision homing speed |
| `$25` | Homing seek rate | 2000 mm/min | Fast initial search speed |
| `$26` | Homing debounce | 25 ms | Limit switch debounce delay |
| `$27` | Homing pull-off | 2.0 mm | Distance to back off after homing |
| `$30` | Max spindle RPM | 24000 | Maps to 100% PWM duty |
| `$31` | Min spindle RPM | 0 | Maps to 0% PWM duty |
| `$32` | Laser mode | 0 | Enables laser-mode PWM behaviour |
| `$100` | X steps/mm | 80 | Step resolution for X axis |
| `$101` | Y steps/mm | 80 | Step resolution for Y axis |
| `$102` | Z steps/mm | 3200 | Step resolution for Z axis |
| `$103` | A steps/mm | 80 | Step resolution for A axis |
| `$110` | X max rate | 5000 mm/min | Maximum X velocity |
| `$111` | Y max rate | 5000 mm/min | Maximum Y velocity |
| `$112` | Z max rate | 300 mm/min | Maximum Z velocity |
| `$113` | A max rate | 5000 mm/min | Maximum A velocity |
| `$120` | X acceleration | 5000 mm/s² | X axis acceleration limit |
| `$121` | Y acceleration | 5000 mm/s² | Y axis acceleration limit |
| `$122` | Z acceleration | 200 mm/s² | Z axis acceleration limit |
| `$123` | A acceleration | 500 mm/s² | A axis acceleration limit |
| `$130` | X max travel | 300 mm | X soft limit (requires `$20=1`) |
| `$131` | Y max travel | 300 mm | Y soft limit |
| `$132` | Z max travel | 100 mm | Z soft limit |
| `$133–$143` | A / extended | various | A-axis limits and jerk settings |

### Tuning Tips

**Junction deviation (`$11`):** Lower values give sharper corners but slower corner speed. Higher values give faster operation but may cause path deviation at corners. Start at 0.010 mm and increase if cornering takes too long.

**Arc tolerance (`$12`):** Lower values give smoother arcs (more segments, smaller chords). Higher values give fewer segments. At 0.500 mm, typical arcs use 3–10 segments per quarter-circle, which is a good balance between smoothness and queue efficiency.

**Acceleration (`$120`–`$123`):** Setting too high causes missed steps. Too low wastes time. Tuning: increase until occasional missed steps appear, then back off 20%.

---

## Appendix B: File and Module Map

```
Pic32mzCNC_V3/
├── incs/
│   ├── app.h                    Application data structure declaration
│   ├── common.h                 Hardware config, driver selection, GRBL constants
│   ├── data_structures.h        ALL shared data structures (single source of truth)
│   ├── gcode/
│   │   └── gcode_parser.h       G-code parser API
│   ├── motion/
│   │   ├── stepper.h            Stepper hardware API + global alarm flags
│   │   ├── kinematics.h         Coordinate transform API
│   │   ├── trajectory.h         S-curve planner API + SCurveMove definition
│   │   ├── interpolator.h       DDS interpolator API
│   │   ├── motion_bridge.h      Bridge shim API
│   │   └── homing.h             Homing system API
│   ├── settings/
│   │   └── settings.h           CNC_Settings struct + settings API
│   └── utils/
│       ├── utils.h              GPIO function pointers, inline helpers
│       └── uart_utils.h         Non-blocking UART helpers
│
├── srcs/
│   ├── main.c                   Entry point: calls SYS_Initialize() + APP_Tasks() loop
│   ├── app.c                    Application state machine — the conductor
│   ├── gcode/
│   │   ├── gcode_parser.c       GRBL protocol + G-code parsing + flow control
│   │   └── utils.c              String tokenisation utilities
│   ├── motion/
│   │   ├── stepper.c            Low-level hardware config (ISR registration, TMR4)
│   │   ├── interpolator.c       Fixed-rate DDS step generator + ISR
│   │   ├── trajectory.c         S-curve velocity planner + lookahead queue
│   │   ├── kinematics.c         Coordinate transforms (Work ↔ Machine)
│   │   ├── motion_bridge.c      G-code event → trajectory move bridge
│   │   ├── motion_utils.c       Limit checking, direction helpers
│   │   ├── homing.c             $H homing cycle state machine
│   │   ├── spindle.c            M3/M5 spindle PWM control
│   │   └── tmc5160.c            TMC5160 SPI driver (conditional)
│   ├── settings/
│   │   └── settings.c           NVM flash read/write, default values
│   └── utils/
│       ├── utils.c              GPIO function pointer array initialisation
│       └── uart_utils.c         UART_Printf, UART_SendOK, etc.
│
├── bins/
│   └── CNC_V3.hex               Built firmware binary (flash this to the MCU)
│
├── Makefile                     Build system (always run from repo root)
└── docs/
    ├── readme/                  Individual feature documentation
    └── CNC_FIRMWARE_BOOK.md     This document
```

**Data flow between modules:**

```
gcode_parser.c → [GCODE_CommandQueue] → app.c
app.c          → motion_bridge.c → trajectory.c [SCurveMove queue]
trajectory.c   → interpolator.c [active_move]
interpolator.c → ISR → GPIO → Driver → Motor
kinematics.c   ← settings.c (WCS offsets)
                 ↑ called by motion_bridge.c for coord transform
```

---

## Appendix C: Timing Reference

### Timer Configuration Summary

| Timer | Prescaler | Base Clock | Frequency | Period | Purpose |
|-------|-----------|------------|-----------|--------|---------|
| TMR4 | 1:1 | 50 MHz | 100 kHz | 10 µs | Interpolator ISR heartbeat |
| TMR5 | 1:1 | 50 MHz | one-shot | ~3 µs | Step pulse width (GPIO clear) |
| Core Timer | — | 100 MHz | — | — | Dwell timing, homing debounce |
| TMR6 / OC8 | — | 50 MHz | 3.338 kHz | — | Spindle PWM |

### ISR Budget Analysis (TMR4, 100 kHz, 10 µs per tick)

At 200 MHz system clock, 10 µs = **2,000 CPU cycles** available per tick.

| Operation | Cycle Estimate | % Budget |
|-----------|---------------|---------|
| ISR entry overhead (MIPS) | 20 cycles | 1% |
| Clear previous step pins | 20 cycles | 1% |
| TRAJECTORY_VelocityAt() | 120 cycles | 6% |
| 4× DDS accumulate + test | 80 cycles | 4% |
| Step pin SET (if needed) | 20 cycles | 1% |
| Tick countdown + stop | 20 cycles | 1% |
| ISR exit overhead | 20 cycles | 1% |
| **Total** | **~300 cycles** | **~15%** |

**Headroom: ~85%** — substantial room for future enhancements like StallGuard, encoder feedback, or additional axes.

### Maximum Step Rates

| Axis | steps/mm | Max DDS Rate | Max Speed |
|------|----------|-------------|-----------|
| X, Y | 80 | 50,000 steps/s | 37,500 mm/min |
| Z | 3200 | 50,000 steps/s | 937.5 mm/min |
| A | 80 | 50,000 steps/s | 37,500 mm/min |

Configured limits are far below hardware limits:
- XY: 5,000 mm/min (uses 13.3% of step rate)
- Z: 300 mm/min (uses 32% of step rate)

### UART Configuration

| Parameter | Value |
|-----------|-------|
| Baud rate | 115,200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| TX buffer | 1,024 bytes |
| RX buffer | 512 bytes |

**Why 1024 bytes TX?** The `$$` settings dump is ~500 bytes. This must fit in one TX buffer fill. A 256-byte buffer (MCC default) causes `$$` to truncate — senders disconnect. Always verify the TX buffer size is 1024 after any MCC regeneration.

---

## Closing Thoughts

### What Makes This Firmware Unusual

Most hobbyist CNC firmware is a straight port of trapezoidal-profile GRBL. This firmware takes a significantly more sophisticated approach:

**S-curve velocity profiling** eliminates mechanical shock at acceleration transitions. For a cutting machine this means better surface finish and less wear. For a plotter or laser it means the tool velocity is continuous and smooth — critical for consistent mark quality.

**Fixed-rate DDS interpolation** decouples the step generation rate from the motion speed, providing rock-solid 10 µs tick spacing regardless of velocity. This approach is used in high-end industrial servo drives and is now available in an open-source hobby implementation.

**Clean layered architecture** — each module has a single responsibility. The trajectory planner does not touch hardware. The interpolator does not parse G-code. The parser does not know about steps per mm. This separation makes each component testable and modifiable independently.

**Monotonic consumption tracking** (`commands_consumed`) for flow control solves a subtle but fundamental problem with depth-delta approaches: correctly tracking releases in the face of concurrent arrivals. This took several iterations to get right and the solution is now robust.

### Where the Firmware Is Going

The roadmap (in the `copilot-instructions.md`) lays out the planned evolution:

1. **Probing (G38.x)** — Tool contact detection for auto-zeroing
2. **Tool length offsets (G43/G49)** — Automatic Z compensation per tool
3. **Multi-WCS (G54–G59)** — Complete six-WCS implementation
4. **Rigid tapping and canned cycles (G81–G89)** — Standard CNC drilling patterns
5. **CAN / EtherCAT drive communication** — Replace step-dir GPIO with fieldbus
6. **Closed-loop encoder feedback** — Full servo with following-error alarm

Each of these builds on the clean foundation this firmware already provides.

### A Note on Reading the Code

The best way to deepen your understanding after reading this book is to follow a single G-code line from entry to step pulse:

1. Open `srcs/gcode/gcode_parser.c`, find the `GCODE_STATE_GCODE_COMMAND` handler. Trace how `G1 X60 Y0` becomes an entry in `cmdQueue`.

2. Open `srcs/app.c`, find `APP_IDLE`. Trace the call to `GCODE_GetNextEvent` and then to `MOTION_ProcessGcodeEvent`.

3. Open `srcs/motion/motion_bridge.c`, find `GCODE_EVENT_LINEAR_MOVE`. Trace the geometry calculation and `TRAJECTORY_AddMove` call.

4. Open `srcs/motion/trajectory.c`, find `TRAJECTORY_AddMove`. Trace the unit vector computation, junction speed calculation, and S-curve solve.

5. Open `srcs/motion/interpolator.c`, find `INTERPOLATOR_LoadMove` and the TMR4 callback. Watch the DDS accumulator count up and fire a step.

By the time you have followed this thread, you will have read the most critical path in the entire firmware — and you will understand exactly why each piece is where it is.

---

*This book documents the Pic32mzCNC_V3 firmware as of commit `72423f1` on branch `scurve_motion`, March 2026.*

*For corrections, additions, or questions, open an issue on the GitHub repository.*
