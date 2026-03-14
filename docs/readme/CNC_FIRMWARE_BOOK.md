# Building a CNC Brain: The Complete Pic32mzCNC_V3 Firmware Reference

### A Technical Deep-Dive from G-Code Character to Motor Step Pulse

---

**Hardware**: PIC32MZ2048EFH100 — 200 MHz MIPS M-class with hardware FPU  
**Firmware Branch**: `scurve_motion`  
**Protocol**: GRBL v1.1  
**Compiler**: Microchip XC32  
**Last Validated**: March 2026 — hardware stress test, 1:59 duration, exact origin return

---

## Preface

This book documents every layer of the Pic32mzCNC_V3 firmware: from the moment a
G-code character arrives over the serial link to the moment a GPIO pin fires a step
pulse to a stepper motor driver. It is written for the engineer who wants to
understand not just *what* the code does, but *why* each design decision was made and
*how* the PIC32MZ hardware is specifically exploited for maximum performance.

Every code listing is exact or closely paraphrased from the real source files. Every
timing number has been verified against the actual hardware configuration. Where the
firmware makes a non-obvious choice — deferred `ok` flow control, inline arc retry,
DDS step generation, S-curve jerk limiting — the reasoning is explained in full.

---

## Table of Contents

1. Chapter 1: System Overview
2. Chapter 2: The PIC32MZ Hardware Platform
3. Chapter 3: GRBL Protocol and G-Code Parsing
4. Chapter 4: The Mathematics of CNC Motion
5. Chapter 5: The DDS Step Generator and ISR
6. Chapter 6: S-Curve Velocity Profiling and Lookahead
7. Chapter 7: The Motion Pipeline — From G-Code to Steps
8. Chapter 8: Coordinate Systems and Kinematics
9. Chapter 9: Flow Control — Keeping Senders in Sync
10. Chapter 10: The Application State Machine
11. Chapter 11: Safety — Limits, Alarms, and Homing
12. Chapter 12: Spindle and Laser Control
13. Chapter 13: Settings and NVM Persistence
14. Chapter 14: The Debug Infrastructure
15. Chapter 15: Build System and Toolchain
16. Appendix A: GRBL Settings Reference (`$$`)
17. Appendix B: GPIO Pin Table
18. Appendix C: Timer Configuration Reference
19. Appendix D: Module and File Map
20. Appendix E: ISR Budget Analysis

---

## Chapter 1: System Overview

### 1.1 What the Firmware Does

The Pic32mzCNC_V3 firmware turns a PIC32MZ microcontroller into a four-axis CNC
motion controller. It speaks the GRBL v1.1 protocol — the same protocol understood
by Universal G-Code Sender (UGS), Candle, and bCNC. A sender streams G-code text
over USB serial; the firmware translates each command into precisely timed GPIO pulses
to stepper motor drivers.
____________________________________________________________________________________________________
| Capability                                    | Implementation                                    |
|-----------------------------------------------|---------------------------------------------------|
| GRBL v1.1 protocol                            | `gcode_parser.c`                                  |
| 4-axis coordinated motion                     | `interpolator.c` (DDS) + `trajectory.c` (S-curve) |
| S-curve jerk-limited velocity profiles        | `trajectory.c`                                    |
| Arc interpolation (G2/G3)                     | `motion_bridge.c`                                 |
| Junction speed blending                       | `trajectory.c::TRAJECTORY_AddMove()`              |
| Homing cycle (`$H`)                           | `homing.c`                                        |
| Six work coordinate systems (G54–G59)         | `kinematics.c`                                    |
| Tool length offsets (G43/G43.1/G49)           | `kinematics.c`                                    |
| Canned drilling cycles (G81/G83)              | `motion_bridge.c`                                 |
| Software jog (`$J=`)                          | `gcode_parser.c` / `motion_bridge.c`              |
| Spindle PWM (M3/M5)                           | `spindle.c`                                       |
| Laser power-velocity tracking (`$32=1`)       | `interpolator.c`                                  |
| Persistent settings (NVM flash)               | `settings.c`                                      |
| Real-time overrides (feed / rapid / spindle)  | `gcode_parser.c` / `interpolator.c`               |
| Smooth feed-hold deceleration                 | `interpolator.c::INTERPOLATOR_SoftStop()`         |
| TMC5160 SPI driver (conditional)              | `tmc5160.c` (`#ifdef HAS_TMC5160_AXIS`)           |
|_______________________________________________|___________________________________________________|
### 1.2 The Two Worlds: ISR and Main Loop

**The real-time world (ISR):**  
Timer TMR4 fires 100,000 times per second. Every 10 µs, execution is interrupted and
`INTERPOLATOR_Tick()` runs. It computes instantaneous velocity, runs the DDS
accumulator for each axis, and fires GPIO step pulses. It must complete in much less
than 10 µs.

**The main-loop world:**  
Between ISR invocations, `APP_Tasks()` runs freely. It reads UART characters, parses
G-code, plans S-curve velocity profiles, manages coordinate transforms, and decides
what to do next. These operations involve floating-point mathematics, string parsing,
and state machines — unsuitable for an ISR.

The two worlds share data through `volatile` variables. **The ISR only reads these
variables; only the main loop writes them (in `INTERPOLATOR_LoadMove()`).** This
one-directional contract requires no locking.

### 1.3 The Motion Pipeline

```
PC / UGS
    |   USB-serial 115200 baud
    v
UART3 ring buffer  (1024 B RX + 1024 B TX)   [plib_uart3.c]
    |
GCODE_Tasks()  <── every APP_IDLE iteration
    |   text parsing, flow control, command queuing
    v
GCODE_CommandQueue  (64 slots x 80 bytes)     [data_structures.h]
    |
MOTION_ProcessGcodeEvent()  <── one event per APP_IDLE iteration
    |   geometry, arc expansion, S-curve planning
    v
SCurveMove trajectory queue  (64 entries)     [trajectory.c]
    |
INTERPOLATOR_LoadMove()  <── when interpolator is idle
    |   pre-computes DDS axis scales, dir pins, expected steps
    v
TMR4 ISR @ 100 kHz  (10 µs period)           [interpolator.c]
    |   DDS accumulate → fire GPIO step pulses
    v
GPIO STEP pin HIGH on tick N
GPIO STEP pin cleared at top of tick N+1  (10 µs pulse width)
    |
Stepper driver (DRV8825 or TMC5160)
    |
Motor microstep
```

### 1.4 Key Architectural Principles

**Single instance pattern.** All major data lives in `APP_DATA`, passed by reference
through every function call. No hidden module-level static state.

**No blocking in the main loop.** Every operation completes or yields within one
`APP_Tasks()` iteration. Arc generation produces one segment per call; dwell checks
one deadline per call.

**Flat, direct array access.** All switch statements on axis index are replaced by
function pointer arrays. `AXIS_StepSet(AXIS_X)` compiles to a single GPIO LAT
register write.

**Aggressive flow control.** The firmware never drops G-code commands. `ok` is
withheld when the trajectory queue is near-full so the sender paces itself.

---

## Chapter 2: The PIC32MZ Hardware Platform

### 2.1 The Microcontroller — PIC32MZ2048EFH100
_____________________________________________________________________
| Sub-field | Meaning                                                |
|-----------|--------------------------------------------------------|
| PIC32     | 32-bit MIPS M-class processor                          |
| MZ        | "Majestic" — Microchip's high-performance PIC32 family |
| 2048      | 2,048 KB (2 MB) internal flash                         |
| EF        | Hardware Single-Precision FPU (IEEE 754)               |
| H         | Extended temperature range                             |
| 100       | 100-pin QFP package                                    |
|___________|________________________________________________________|
________________________________________________________________________________________
| Parameter |   Value           | Notes                                                 |
|-----------|-------------------|-------------------------------------------------------|
| CPU clock | 200 MHz           | 5-stage MIPS pipeline                                 |
| Flash     | 2 MB              | Application + bootloader + NVM settings               |
| RAM       | 512 KB            | ~128 KB in use                                        |
| FPU       | Single-precision  | `-mhard-float -msingle-float -mfp64`                  |
| PBCLK3    | 50 MHz            | Peripheral bus — source for all timers and OC modules |
| UART3     | 115,200 baud      | G-code serial link                                    |
| SPI2      | Up to 25 MHz      | TMC5160 configuration (conditional)                   |
|___________|___________________|_______________________________________________________|

### 2.2 Clock Tree

```
External oscillator  →  PLL
                          |
              System: 200 MHz
                          |
              +───────────+───────────+
              |                       |
          PBCLK1: 100 MHz         PBCLK3: 50 MHz
          (CPU, cache, DMA)       (TMR4, TMR5, TMR6, OC1–OC8)
```

All step timing derives from **PBCLK3 = 50 MHz**.

### 2.3 TMR4 — The Interpolator Heartbeat

TMR4 is a 16-bit timer clocked from PBCLK3 with 1:1 prescaler.  
Period register PR4 = 499, giving:

```
f_tick = PBCLK3 / (PR4 + 1) = 50,000,000 / 500 = 100,000 Hz  (100 kHz)
T_tick = 10 µs
```

Defined in `srcs/motion/interpolator.c`:

```c
#define INTERP_PR4_VALUE        (499u)   // (50 MHz / 100 kHz) - 1
#define INTERPOLATOR_TICK_RATE_HZ  100000u
```

The ISR callback is registered as:

```c
// interpolator.c — INTERPOLATOR_Initialize()
TMR4_CallbackRegister(INTERPOLATOR_Tick, (uintptr_t)NULL);
TMR4_PeriodSet(INTERP_PR4_VALUE);
```

**Step pulse width = 10 µs (inherent, no secondary timer needed)**

Step pins are raised at the end of the ISR body in tick N.  
At the very first action of the ISR in tick N+1:

```c
// interpolator.c — INTERPOLATOR_Tick(), first executable lines:
for (int i = 0; i < NUM_AXIS; i++) {
    AXIS_StepClear(i);   // Clear ALL step pins from previous tick
}
```

Pulse duration = one full tick period = **10 µs** — guaranteed regardless of step rate.

> There is **no TMR5** used for step pulse width. Earlier firmware revisions used TMR5.
> The fixed-rate DDS architecture makes it completely unnecessary.

### 2.4 TMR6 + OC8 — Spindle PWM

```
PBCLK3    = 50,000,000 Hz
Prescaler = 1:64  (T6CONbits.TCKPS = 6)
Timer f   = 781,250 Hz
PR6       = 233

PWM f     = 781,250 / (233 + 1) = 3,338 Hz  (3.338 kHz)
Duty      = OC8RS / 234  —  range 0 (0%) to 233 (99.6%)
```

From `srcs/motion/spindle.c`:

```c
#define PWM_TIMER_FREQ_HZ   781250UL   // after 1:64 prescaler
#define PWM_PERIOD_TICKS    233U       // PR6
// f_PWM = 781250 / 234 = 3338.0 Hz
```

OC8 uses `OCTSEL = 0` (TMR6 time base), running in PWM mode.
Setting `OCMP8_CompareSecondaryValueSet(duty)` writes OC8RS directly —
an ISR-safe single-register write.

### 2.5 Memory Map

```
Virtual (KSEG)         Physical         Size     Purpose
─────────────────────  ───────────      ──────   ─────────────────────────────
0x9D000000  (KSEG0)    0x1D000000       1.87 MB  Application code (cached)
0xBD1F0000  (KSEG1)    0x1D1F0000       16 KB    GRBL Settings NVM (uncached)
0x9D1F4000  (KSEG0)    0x1D1F4000       48 KB    MikroE USB HID Bootloader
0xBFC00000  (KSEG1)    0x1FC00000       12 KB    Boot flash / configuration words
```

**KSEG0** (0x9D…) = cached — used for normal firmware execution.  
**KSEG1** (0xBD…) = uncached — **mandatory for all NVM read/write operations**.  
Writing to NVM through the KSEG0 alias produces undefined behaviour on PIC32MZ.

### 2.6 GPIO Pin Assignments

```
Function        Pin   Port Register   Macro in plib_gpio.h
──────────────  ────  ─────────────── ─────────────────────────────────
X STEP          RD4   LATD  bit  4     RD4_Set()  / RD4_Clear()
X DIR           RD5   LATD  bit  5     RD5_Set()  / RD5_Clear()
Y STEP          RD10  LATD  bit 10    RD10_Set() / RD10_Clear()
Y DIR           RD11  LATD  bit 11    RD11_Set() / RD11_Clear()
Z STEP          RE2   LATE  bit  2     RE2_Set()  / RE2_Clear()
Z DIR           RE3   LATE  bit  3     RE3_Set()  / RE3_Clear()
A STEP          RF3   LATF  bit  3     RF3_Set()  / RF3_Clear()
A DIR           RF4   LATF  bit  4     RF4_Set()  / RF4_Clear()

Shared ENABLE   RE6   LATE  bit  6     EnXYZA_Clear() → enable all
                                       EnXYZA_Set()   → disable all

X Limit MIN     RA4   PORTA bit  4     RA4_Get()
X Limit MAX     RA7   PORTA bit  7     RA7_Get()
Y Limit MIN     RD0   PORTD bit  0     RD0_Get()
Y Limit MAX     RE0   PORTE bit  0     RE0_Get()
Z Limit MIN     RD13  PORTD bit 13    RD13_Get()
Z Limit MAX     RE1   PORTE bit  1     RE1_Get()  (also probe input)
A Limit MIN     RA6   PORTA bit  6     RA6_Get()
A Limit MAX     RB1   PORTB bit  1     RB1_Get()

Spindle PWM     OC8   (automatic OC output — no direct LAT write)
```

### 2.7 The Single Shared Enable Pin

> **CRITICAL HARDWARE CONSTRAINT:**  
> The PCB has **one shared enable pin** for all four axes: `EnXYZA` on RE6.  
> There are NO per-axis enable GPIO macros. Any attempt to create `EnX_Set()`,
> `EnY_Clear()` etc. will fail — these macros do not exist in `plib_gpio.h`.

Enable is **active LOW**: Clear the pin to energise motors; Set the pin to de-energise.

```c
// incs/utils/utils.h
static inline void __attribute__((always_inline)) STEPPERS_Enable(void)  { EnXYZA_Clear(); }
static inline void __attribute__((always_inline)) STEPPERS_Disable(void) { EnXYZA_Set();   }

// Per-axis wrappers all resolve to the same shared pin:
static inline void __attribute__((always_inline)) AXIS_EnableSet(E_AXIS axis) {
    (void)axis;
    EnXYZA_Clear();   // axis parameter ignored — hardware has no per-axis enable
}
```

### 2.8 Function Pointer Arrays — Zero-Overhead Axis Abstraction

All GPIO operations access axes through function pointer arrays.
The `__attribute__((always_inline))` wrappers ensure no function call overhead.

```c
// incs/utils/utils.h  (extern declarations)
extern GPIO_SetFunc   axis_step_set[NUM_AXIS];   // RD4_Set, RD10_Set, RE2_Set, RF3_Set
extern GPIO_ClearFunc axis_step_clear[NUM_AXIS];
extern GPIO_SetFunc   axis_dir_set[NUM_AXIS];
extern GPIO_ClearFunc axis_dir_clear[NUM_AXIS];
extern GPIO_GetFunc   axis_limit_min_get[NUM_AXIS]; // RA4_Get, RD0_Get, RD13_Get, RA6_Get
extern GPIO_GetFunc   axis_limit_max_get[NUM_AXIS]; // RA7_Get, RE0_Get, RE1_Get,  RB1_Get

static inline void __attribute__((always_inline)) AXIS_StepSet(E_AXIS axis) {
    axis_step_set[axis]();
}
static inline void __attribute__((always_inline)) AXIS_StepClear(E_AXIS axis) {
    axis_step_clear[axis]();
}
static inline void __attribute__((always_inline)) AXIS_IncrementSteps(E_AXIS axis) {
    (*g_axis_settings[axis].step_count)++;
}
static inline void __attribute__((always_inline)) AXIS_DecrementSteps(E_AXIS axis) {
    (*g_axis_settings[axis].step_count)--;
}
```

In a Release build (`-O2`), the compiler resolves each `axis_step_set[axis]()` call
to a single LAT register write — identical to calling `RD4_Set()` directly.

---

## Chapter 3: GRBL Protocol and G-Code Parsing

**Source**: `srcs/gcode/gcode_parser.c`

### 3.1 The GRBL v1.1 Protocol

GRBL v1.1 is the standard protocol for amateur and semi-professional CNC. The
firmware claims full GRBL v1.1 compliance and has been tested with UGS, Candle,
and bCNC in both single-step and pipelined streaming modes.

**Startup banner** — sent on power-up and every soft reset:

```
Pic32mzCNC v1.1h ['$' for help]


```

This exact string is how senders identify the controller. It is defined in
`incs/common.h`:

```c
#define GRBL_FIRMWARE_VERSION   "Pic32mzCNC v1.1h ['$' for help]\r\n"
```

> Note: The banner says **Pic32mzCNC**, not "Grbl". Senders that require the exact
> string "Grbl" may need their detection regex loosened.

**The ok/error handshake:**  
Every complete line sent to the firmware receives exactly one response: `ok

`
on success, or `error:N

` on parse error. The firmware's flow control (Chapter 9)
determines whether `ok` is sent immediately or deferred until the motion queue drains.

**Status report** — sent in response to `?` with no `ok`:

```
<Run|MPos:30.013,50.013,0.000,0.000|WPos:30.013,50.013,0.000,0.000|FS:5000,12000|Ov:100,100,100>
```

| Field | Meaning |
|---|---|
| State | `Idle`, `Run`, `Hold`, `Alarm`, `Jog`, `Check` |
| `MPos` | Machine position (step counts ÷ steps_per_mm) |
| `WPos` | Work position (MPos − active WCS offset − G92 offset) |
| `FS` | Feed speed (mm/min) : spindle speed (RPM) |
| `Ov` | Feed / rapid / spindle overrides (%) |
| `|Cm:1` | Check mode is active |

### 3.2 Real-Time Characters

Real-time control bytes are processed **immediately**, bypassing the line buffer and
all queues. They may arrive at any time — even in the middle of a G-code line.

| Byte  | Decimal | Action |
|---|---|---|
| `?`   | 63  | Send status report (no `ok`) |
| `!`   | 33  | Feed hold — smooth deceleration ramp |
| `~`   | 126 | Cycle start / resume from hold |
| 0x18  | 24  | Soft reset (Ctrl+X) — full system reinitialisation |
| 0x85  | 133 | Jog cancel — flush jog segments only |
| 0x90  | 144 | Feed override: 100% |
| 0x91  | 145 | Feed override: +10% |
| 0x92  | 146 | Feed override: −10% |
| 0x93  | 147 | Feed override: +1% |
| 0x94  | 148 | Feed override: −1% |
| 0x95  | 149 | Rapid override: 100% |
| 0x96  | 150 | Rapid override: 50% |
| 0x97  | 151 | Rapid override: 25% |
| 0x99  | 153 | Spindle override: 100% |
| 0x9A  | 154 | Spindle override: +10% |
| 0x9B  | 155 | Spindle override: −10% |
| 0x9C  | 156 | Spindle override: +1% |
| 0x9D  | 157 | Spindle override: −1% |

The predicate in `gcode_parser.c`:

```c
static bool is_control_char(uint8_t c) {
    return (c == '?') || (c == '!') || (c == '~') || (c == 0x18) ||
           (c == 0x85) ||
           (c >= 0x90u && c <= 0x9Du);   // all 14 override bytes in one range
}
```

The range `0x90`–`0x9D` captures all 14 override bytes efficiently. Note that 0x98
is not assigned — it falls in the range but is silently discarded if sent.

A **push-back slot** (`s_rxPushback`, `s_rxHasPushback`) exists so that a G-code
first byte saved during an arc-segment retry is never lost when a `?` status request
arrives mid-retry.

### 3.3 System Commands

System commands begin with `$` and are handled directly in the parser without
being queued to the G-code command queue.

| Command | Description |
|---|---|
| `$$` | Print all settings (ensure TX buf ≥ 1024 bytes) |
| `$I` | Build info: `[VER:1.1h.date]` + `[OPT:VHM,63,1023,4]` |
| `$G` | Modal state: `[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]` |
| `$#` | Work offsets: G54–G59, G28, G30, G92, TLO, PRB |
| `$H` | Run homing cycle; `ok` is **deferred** until homing completes |
| `$X` | Clear alarm state |
| `$C` | Toggle check mode (parse without motion) |
| `$Nn` | Read startup line N (0 or 1) |
| `$Nn=<gcode>` | Write and persist startup line |
| `$<n>=<v>` | Write setting n to value v |
| `$RST=*` | Restore all defaults; `$RST=$` resets settings; `$RST=#` resets WCS |
| `$J=G91 X10 F500` | Jog command |
| `$SLP` | Not supported → `error:2` |

### 3.4 Supported G-Codes

| G-Code | Action |
|---|---|
| G0 / G1 | Rapid / linear feed move |
| G2 / G3 | CW / CCW arc (I,J,K centre; or R radius) |
| G4 Pn | Dwell n seconds |
| G10 L2 Pn / L20 Pn | Set / copy work coordinate system offset |
| G17 / G18 / G19 | Plane selection (XY / XZ / YZ) |
| G20 / G21 | Inch / millimetre units |
| G28 / G28.1 | Rapid to stored position / save current position |
| G30 / G30.1 | Rapid to second stored position / save |
| G38.2 / G38.3 / G38.4 / G38.5 | Probe toward/away with/without alarm on fail |
| G43 / G43.1 Zn | Activate TLO (stored / dynamic) |
| G49 | Cancel TLO |
| G54–G59 | Select work coordinate system |
| G80 | Cancel canned cycle |
| G81 | Simple drill canned cycle |
| G83 Qn | Peck drill canned cycle |
| G90 / G91 | Absolute / incremental distance mode |
| G92 / G92.1 | Set / clear coordinate offset |
| G98 / G99 | Canned cycle retract: initial Z / R-plane |

### 3.5 Combined Modal Splitting

The tokeniser in `srcs/gcode/utils.c` handles combined modal tokens as GRBL v1.1
requires. Parameters (X, Y, Z, F, S…) attach to the **last modal** in a combined
string.

```
Input:   "G21G90 G0Z5
"
Tokens:  ["G21", "G90", "G0Z5"]

Input:   "G90G1X10Y10F500
"
Tokens:  ["G90", "G1X10Y10F500"]

Input:   "G83G91Z-5Q1F100
"
Tokens:  ["G83G91Z-5Q1F100"]  (single modal, G91 absorbed by peck-drill parser)
```

### 3.6 The Command Queue

```c
// incs/data_structures.h
#define GCODE_MAX_COMMANDS   64
#define GCODE_BUFFER_SIZE    80

typedef struct {
    char command[GCODE_BUFFER_SIZE];
} GCODE_Command;

typedef struct {
    GCODE_Command commands[GCODE_MAX_COMMANDS];
    uint32_t head;               // write index (producer = parser)
    uint32_t tail;               // read index  (consumer = motion bridge)
    uint32_t count;              // current occupancy
    uint32_t commands_consumed;  // monotonic — basis of deferred-ok logic
    uint32_t maxMotionSegments;  // trajectory queue capacity (reference only)
} GCODE_CommandQueue;
```

`commands_consumed` is a monotonic counter that is incremented every time the motion
bridge dequeues and processes one command. The flow controller compares
`commands_consumed` to the expected `ok` count to decide when a deferred `ok`
may be released.

---

## Chapter 4: The Mathematics of CNC Motion

### 4.1 Coordinate Geometry

Positions in 4D space are represented as arrays wrapped in `CoordinatePoint`:

```c
// incs/data_structures.h
typedef struct {
    float coordinate[NUM_AXIS];   // [0]=X  [1]=Y  [2]=Z  [3]=A
} CoordinatePoint;
```

Array indexing lets all axis operations be written as loops:

```c
// incs/utils/utils.h — inline helpers
static inline float GET_COORDINATE_AXIS(const CoordinatePoint *p, E_AXIS ax) {
    return p->coordinate[ax];
}
static inline void SET_COORDINATE_AXIS(CoordinatePoint *p, E_AXIS ax, float v) {
    p->coordinate[ax] = v;
}
static inline void ADD_COORDINATE_AXIS(CoordinatePoint *p, E_AXIS ax, float d) {
    p->coordinate[ax] += d;
}
```

**Euclidean distance** between two points:

```
d = sqrt( sum_i( delta_i^2 ) )    for i in {X, Y, Z, A}
```

**Unit vector** (direction only, magnitude = 1):

```
u_i = delta_i / d
```

Every `SCurveMove` stores `unit_vec[NUM_AXIS]`. The instantaneous velocity on any
axis at total speed `v` is:

```
v_axis_i = v * |unit_vec_i|
```

### 4.2 Direct Digital Synthesis (DDS) — The Step Algorithm

The ISR must produce a precise fractional number of steps per tick without floating
point division. **Direct Digital Synthesis** achieves this with a 32-bit integer
accumulator and an overflow threshold.

**Principle:**
- Each tick: add increment `dds_inc` to accumulator `dds_acc`
- When accumulator ≥ threshold `DDS_SCALE`: fire one step, subtract `DDS_SCALE`

```c
// interpolator.c
#define DDS_SCALE   (1 << 30)    // 1,073,741,824

// Per-axis in ISR:
dds_acc[i] += dds_inc[i];
if (dds_acc[i] >= DDS_SCALE) {
    AXIS_StepSet(i);
    steps_fired[i]++;
    dds_acc[i] -= DDS_SCALE;
    if (dir_positive[i]) AXIS_IncrementSteps(i);
    else                 AXIS_DecrementSteps(i);
}
```

**Computing the increment — done in `LoadMove` (main-loop context):**

Axis scale (pre-computed once per move, no ISR division):
```c
// interpolator.c — INTERPOLATOR_LoadMove()
axis_scale[i] = ((double)steps_per_mm[i] / INTERPOLATOR_TICK_RATE_HZ)
                * (double)DDS_SCALE;
```

In the ISR, each tick:
```c
dds_inc[i] = (int32_t)(v_now * fabsf(unit_vec[i]) * axis_scale[i]);
```

Where `v_now` [mm/s] comes from the S-curve evaluator.

**Maximum step rate per axis:**
At `dds_inc = DDS_SCALE / 2`, one step fires every two ticks: 50,000 steps/s.
At 80 steps/mm (16× microstepping, 4 mm/rev lead, 200-step motor):

```
v_max = 50,000 / 80 = 625 mm/s = 37,500 mm/min
```

This far exceeds the configured $110 maximum (typically 3,000–5,000 mm/min).

### 4.3 Penultimate-Tick Deficit Flush — Exact Position Guarantee

As velocity approaches zero at move end, DDS increments shrink and integer rounding
can leave 1–2 steps un-fired. The firmware pre-computes the **expected total steps**
at `LoadMove` time:

```c
// interpolator.c — INTERPOLATOR_LoadMove()
for (int i = 0; i < NUM_AXIS; i++) {
    expected_steps[i] = (int32_t)(
        move->millimeters * fabsf(move->unit_vec[i]) * steps_per_mm[i] + 0.5f
    );
    steps_fired[i] = 0;
}
```

On the **penultimate tick** (`ticks_remaining == 1`), any deficit is force-fired:

```c
// interpolator.c — inside INTERPOLATOR_Tick(), ticks_remaining == 1 path:
for (int i = 0; i < NUM_AXIS; i++) {
    int32_t deficit = expected_steps[i] - steps_fired[i];
    while (deficit > 0) { AXIS_StepSet(i); AXIS_IncrementSteps(i); deficit--; steps_fired[i]++; }
    while (deficit < 0) { AXIS_StepSet(i); AXIS_DecrementSteps(i); deficit++; steps_fired[i]--; }
}
```

These deficit steps are **set on the penultimate tick** and **cleared at the top of
the final tick** — guaranteeing a full 10 µs pulse width. This mechanism produces
the exact `MPos:0.000,0.000,0.000,0.000` return observed in hardware testing after
1:59 of mixed motion.

### 4.4 Arc Interpolation Mathematics

G2/G3 arcs are approximated by short linear chords. Chord length is determined by
the GRBL chord-deviation formula using `$12` (arc tolerance in mm):

```
theta_half = arccos(1 - arc_tolerance / radius)
chord_length = 2 * radius * sin(theta_half)
N_segments = ceil(arc_length / chord_length)
```

With default `$12 = 0.500 mm` and radius = 20 mm:
- `theta_half = arccos(0.975) ≈ 12.7°`
- `chord_length ≈ 8.78 mm`
- Quarter circle (31.4 mm arc): only ~4 segments needed

Each segment is a small `KINEMATICS_LinearMove()` call with fixed start and end
coordinates. The arc centre remains fixed throughout.

Arc parameters from I, J, K:
- Centre: `cx = start.x + I`, `cy = start.y + J` (K for XZ/YZ planes)
- Radius check: `|r_start - r_end| < $12 * 2` (tolerance, else error:33)
- Start angle: `atan2(start.y - cy, start.x - cx)`
- End angle: `atan2(end.y - cy, end.x - cx)` with CW/CCW wrap

---

## Chapter 5: The DDS Step Generator and ISR

**Source**: `srcs/motion/interpolator.c`

The interpolator is the lowest-level layer of the firmware. It transforms abstract
S-curve velocity profiles into precisely timed step GPIO pulses. All computation
here is in the interrupt context.

### 5.1 ISR Entry and Controls

The ISR function is `INTERPOLATOR_Tick()`, registered as the TMR4 callback:

```c
// interpolator.c
static void INTERPOLATOR_Tick(uint32_t status, uintptr_t context) {
    // ── Guard 1: feed hold ──────────────────────────────────────────────────
    if (feed_hold) {
        return;   // Stop immediately, no deceleration (hard hold)
    }

    // ── Guard 2: soft-stop deceleration ramp ───────────────────────────────
    if (hold_decel_active) {
        // ... separate decel path (see §5.4) ...
        return;
    }

    // ── Step 1: Clear previous tick's step pins ────────────────────────────
    for (int i = 0; i < NUM_AXIS; i++) {
        AXIS_StepClear(i);
    }

    // ── Step 2: If no move active, return ─────────────────────────────────
    if (!interp_active) return;
    // ... rest of ISR body ...
}
```

`feed_hold` is set by the `!` real-time command via `INTERPOLATOR_FeedHold()`.
This immediately freezes all step output without a deceleration ramp.

`hold_decel_active` is set by `INTERPOLATOR_SoftStop()` (feed hold `!` in the
GRBL sense — a smooth deceleration). See §5.4.

### 5.2 Velocity Evaluation

The instantaneous velocity is evaluated by calling `TRAJECTORY_VelocityAt()`:

```c
// Compute elapsed time within the current move:
float elapsed_s = ((float)(move_total_ticks - ticks_remaining) + 0.5f)
                  / (float)INTERPOLATOR_TICK_RATE_HZ;

// Get velocity from S-curve (main computation):
float v_now = TRAJECTORY_VelocityAt(&active_move, elapsed_s);

// Apply feed override (0.1 to 2.0, updated from UART parsing):
v_now *= feed_override_val;
```

`TRAJECTORY_VelocityAt()` identifies which of the 7 S-curve phases covers
`elapsed_s` and evaluates the quadratic velocity polynomial for that phase:

```
v(dt) = v_phase + a_phase * dt + 0.5 * jk_phase * dt^2
```

Where `dt = elapsed_s - t[phase_start]`. This is 2 multiplies + 2 adds per call.

### 5.3 The DDS Step Loop

```c
// interpolator.c — inside INTERPOLATOR_Tick():
for (int i = 0; i < NUM_AXIS; i++) {
    dds_inc[i] = (int32_t)(v_now * fabsf(unit_vec[i]) * axis_scale[i]);
    dds_acc[i] += dds_inc[i];
    if (dds_acc[i] >= DDS_SCALE) {
        AXIS_StepSet(i);
        steps_fired[i]++;
        dds_acc[i] -= DDS_SCALE;
        if (dir_positive[i]) AXIS_IncrementSteps(i);
        else                 AXIS_DecrementSteps(i);
    }
}
```

All four axes are independent. There is no dominant-axis concept, no Bresenham
integer accumulator. Each axis fires steps at exactly the rate demanded by its
component of the velocity vector. This is the key difference from earlier firmware
revisions and from many open-source CNC controllers.

> **The ISR uses DDS only — not Bresenham.**  
> Bresenham requires a dominant axis and cascades subordinate steps from the
> dominant-axis ISR. DDS handles all axes independently with equal precision.

### 5.4 Soft-Stop — Feed Hold with Deceleration

`INTERPOLATOR_SoftStop()` is called by the `!` G-code real-time command.
It initiates a controlled deceleration instead of an abrupt freeze.

```c
// interpolator.c — INTERPOLATOR_SoftStop()
void INTERPOLATOR_SoftStop(void) {
    if (!interp_active) return;
    hold_decel_v_start = current_v;          // v at moment stop was requested
    hold_decel_rate    = combined_accel;     // decel rate (mm/s²)
    hold_decel_ticks   = 0;
    hold_decel_active  = true;
    // feed_hold will be set when v drops to zero inside the ISR
}
```

Inside the ISR, the `hold_decel_active` path runs a simple linear decel:

```c
if (hold_decel_active) {
    float t = (float)hold_decel_ticks / INTERPOLATOR_TICK_RATE_HZ;
    float v = hold_decel_v_start - hold_decel_rate * t;
    hold_decel_ticks++;

    if (v <= 0.0f) {
        // Reached zero velocity — park steppers
        v = 0.0f;
        hold_decel_active = false;
        feed_hold = true;
        interp_active = false;
        STEPPERS_Disable();
    }

    // Fire steps at decelerated speed (laser scaling applies here too):
    if (laser_mode_active) {
        float scale = (v > 0.0f) ? (v / laser_nominal_speed) : 0.0f;
        SPINDLE_LaserScale(scale);
    }
    // [DDS loop using v — same structure as normal path]
    return;
}
```

The laser is scaled during hold deceleration so that laser power tracks velocity
continuously — including the decel ramp.

### 5.5 Move Loading — INTERPOLATOR_LoadMove()

`INTERPOLATOR_LoadMove()` is called from the main loop when the interpolator is
idle and a new `SCurveMove` is available in the trajectory queue.

This function runs in **main-loop context**, so it may use floating-point freely.
It pre-computes everything the ISR will need so the ISR does minimal work.

```c
void INTERPOLATOR_LoadMove(const SCurveMove *move) {
    // 1. Stop timer to prevent ISR firing before setup is complete
    TMR4_Stop();

    // 2. Copy S-curve move into ISR-visible volatile storage
    memcpy(&active_move, move, sizeof(SCurveMove));
    move_total_ticks = (uint32_t)(move->t[7] * INTERPOLATOR_TICK_RATE_HZ + 0.5f) + 2;
    ticks_remaining  = move_total_ticks;

    // 3. Pre-compute per-axis DDS scale factors
    for (int i = 0; i < NUM_AXIS; i++) {
        axis_scale[i] = ((double)steps_per_mm[i] / INTERPOLATOR_TICK_RATE_HZ)
                        * (double)DDS_SCALE;
    }

    // 4. Set direction pins (with $3 invert mask applied)
    for (int i = 0; i < NUM_AXIS; i++) {
        bool forward = (move->unit_vec[i] >= 0.0f);
        bool invert  = (dir_invert_mask >> i) & 1u;
        dir_positive[i] = forward;
        if (forward ^ invert) AXIS_DirSet(i);
        else                  AXIS_DirClear(i);
    }

    // 5. Reset DDS accumulators and step counters
    for (int i = 0; i < NUM_AXIS; i++) {
        dds_acc[i]    = 0;
        steps_fired[i] = 0;
        expected_steps[i] = (int32_t)(
            move->millimeters * fabsf(move->unit_vec[i]) * steps_per_mm[i] + 0.5f
        );
    }

    // 6. Cache laser parameters (ISR reads these as read-only volatile)
    laser_mode_active   = (settings.laser_mode != 0);
    laser_duty_cmd      = SPINDLE_GetCommandedDuty();
    laser_nominal_speed = move->nominal_speed;

    // 7. Set interp_active flag and start timer
    interp_active = true;
    move_complete = false;
    TMR4_Start();
}
```

The `+ 2` on `move_total_ticks` adds two guard ticks: one for the penultimate-tick
deficit flush and one for the final clear tick.

### 5.6 Move Completion

When `ticks_remaining` reaches zero:

```c
// interpolator.c
ticks_remaining--;
if (ticks_remaining == 0u) {
    interp_active = false;
    move_complete = true;
    TMR4_Stop();
}
```

The main loop polls `INTERPOLATOR_IsMoveComplete()` and calls
`INTERPOLATOR_LoadMove()` with the next queued segment.

---

## Chapter 6: S-Curve Velocity Profiling and Lookahead

**Source**: `srcs/motion/trajectory.c`

### 6.1 Why S-Curves?

A trapezoidal velocity profile changes acceleration instantaneously at the start of
each ramp. This instantaneous change in acceleration (infinite jerk) mechanically
shocks the machine: the frame flexes, the lead screw vibrates, and surface finish
quality degrades. At high speeds, the shock can cause missed steps.

An S-curve profile limits jerk — the rate of change of acceleration — to a finite
maximum. Acceleration ramps up and down smoothly. The resulting motion is:
- **Quieter**: less mechanical resonance and acoustic noise
- **Faster**: higher sustainable peak speeds without missed steps
- **More accurate**: reduced ringing in the final position

### 6.2 The 7-Phase S-Curve Structure

The S-curve profile is divided into seven phases, indexed 0–6:

```
Velocity
^
|         _______________  v_nominal (cruise)
|        /               |       / (phases 0,1,2)  \  (phases 4,5,6)
|      /                   |  v_entry                  v_exit
+──────────────────────────────────────> Time
   P0  P1    P2   P3  P4    P5   P6

Phase | Jerk jk | Acceleration a          | v changes
──────┼─────────┼─────────────────────────┼────────────────────
  0   |   +J    | 0 → a_peak              | v_entry → partial
  1   |    0    | a_peak (constant)       | linear ramp up
  2   |   -J    | a_peak → 0              | reach v_nominal
  3   |    0    | 0                       | cruise (constant v)
  4   |   -J    | 0 → -a_peak             | begin decel
  5   |    0    | -a_peak (constant)      | linear decel
  6   |   +J    | -a_peak → 0             | reach v_exit
```

Within phase i, at local time dt from phase start:
```
v(dt) = v[i] + a[i]*dt + 0.5*jk[i]*dt^2
a(dt) = a[i] + jk[i]*dt
```

The 8-element time array `t[8]` stores cumulative phase boundary times:
- `t[0] = 0` (move start)
- `t[1]` = end of phase 0 / start of phase 1
- ...
- `t[7]` = total move duration

### 6.3 The SCurveMove Struct

```c
// incs/motion/trajectory.h
typedef struct SCurveMove_t {
    // Geometry
    float millimeters;              // total Euclidean distance
    float unit_vec[NUM_AXIS];       // normalised direction vector
    float steps_per_mm[NUM_AXIS];   // for expected_steps calculation

    // Speeds
    float nominal_speed;            // target cruise speed (mm/s)
    float v_entry;                  // actual entry speed (set by planner)
    float v_exit;                   // actual exit speed (set by planner)
    float max_entry_speed;          // junction limit (set at enqueue time)
    float acceleration;             // combined acceleration (mm/s^2)

    // S-curve phase data (7 phases, 8 boundaries):
    float t[8];    // cumulative phase boundary times [s];  t[7] = total duration
    float v[7];    // velocity at start of each phase [mm/s]
    float a[7];    // acceleration at start of each phase [mm/s^2]
    float jk[7];   // jerk within each phase [mm/s^3]

    // Planner bookkeeping
    bool  recalculate;              // needs forward/reverse pass
    bool  speed_locked;             // skip in planner passes (e.g. arc segments)
} SCurveMove;
```

### 6.4 Jerk Limit Computation

The jerk magnitude J [mm/s³] is the most restrictive across all active axes.
From `trajectory.c::limit_jerk()`:

```c
static float limit_jerk(const SCurveMove *m, const CNC_Settings *s) {
    float J = FLT_MAX;
    for (int i = 0; i < NUM_AXIS; i++) {
        float uv = fabsf(m->unit_vec[i]);
        if (uv < 1e-6f) continue;
        // Formula: J_axis = (acceleration * jerk_setting) * 0.5 / unit_vec
        float J_axis = (s->acceleration[i] * s->jerk[i]) * 0.5f / uv;
        if (J_axis < J) J = J_axis;
    }
    return J;
}
```

The `jerk` setting (`$140`–`$143`) is a dimensionless "aggression factor", not
directly in mm/s³ units. The combined J is derived from acceleration and jerk setting.
A higher value gives a shorter, snappier S-curve ramp.

Example with default values (X/Y: accel=5000 mm/s², jerk=500) on a pure X move:
```
J = (5000 * 500) * 0.5 / 1.0 = 1,250,000 mm/s^3
```

### 6.5 Degenerate Phase Handling

If a move is too short to reach nominal speed, or if acceleration is very high
relative to the jerk limit, phases 1 and/or 5 collapse to zero duration.

```c
// trajectory.c — scurve_fill_phases():
if (t[2] <= t[1]) {
    // Phase 1 zero-duration: no constant-acceleration segment
    t[1] = t[2];     // store equal boundaries
    // ISR skips this phase because t[i+1] == t[i]
}
```

`TRAJECTORY_VelocityAt()` handles this by phase-scanning:
```c
for (int p = 0; p < 7; p++) {
    if (elapsed_s < move->t[p+1]) {
        float dt = elapsed_s - move->t[p];
        return move->v[p] + move->a[p]*dt + 0.5f*move->jk[p]*dt*dt;
    }
}
```

If `move->t[p] == move->t[p+1]`, the condition `elapsed_s < t[p+1]` is never true
for that phase and it is skipped cleanly.

### 6.6 Three-Pass Lookahead Planner

Every time a new move is added to the trajectory queue via `TRAJECTORY_AddMove()`,
the planner re-runs three passes over all buffered, unlocked moves.

**Reverse pass** (newest → oldest): propagates maximum entry speed backwards.
A move cannot enter at a speed it couldn't have decelerated from:

```
v_entry^2 = min(max_entry^2, v_next_entry^2 + 2 * accel * distance)
```

**Forward pass** (oldest → newest): prevents over-optimistic entry speeds that
the previous move couldn't have accelerated to:

```
v_entry[i] = min(v_entry[i], sqrt(v_exit[i-1]^2 + 2 * accel[i-1] * dist[i-1]))
```

**S-curve solve**: calls `scurve_fill_phases()` on every un-locked move
with its settled `v_entry` and `v_exit` values.

`speed_locked` moves (e.g. arc segments with fixed cruise speed) are skipped
in the reverse and forward passes but still receive their S-curve phases.

### 6.7 Junction Speed Limit

The maximum speed that may be carried through a directional change is:

```c
// trajectory.c — TRAJECTORY_AddMove():
double cos_theta = 0.0;
for (int i = 0; i < NUM_AXIS; i++) {
    cos_theta -= (double)prev_unit_vec[i] * (double)m->unit_vec[i];
}
float sin_half_sq = (float)((1.0 - cos_theta) * 0.5);
// Centripetal approximation (GRBL-exact):
float v_junct_sq = settings.acceleration * settings.junction_deviation
                   * sin_half_sq / (1.0f - sin_half_sq);
m->max_entry_speed = sqrtf(v_junct_sq);
```

`junction_deviation` corresponds to GRBL's `$11`. Higher value = faster cornering
(with more arc at the corner); lower value = slower but sharper corners.

A perfect 180° reversal gives `cos_theta = 1.0`, `sin_half_sq = 1.0`, and
`max_entry_speed = 0` (forced stop). A straight line gives `cos_theta = -1.0`,
`sin_half_sq = 0`, and `max_entry_speed → ∞` (limited only by nominal speed).

---

## Chapter 7: The Motion Pipeline — From G-Code to Steps

**Sources**: `srcs/motion/motion_bridge.c`, `srcs/motion/kinematics.c`

### 7.1 The Three-Layer Motion Stack

```
Layer 1: Parser        gcode_parser.c
          ↓ string command (e.g. "G1X10Y20F500")
Layer 2: Motion Bridge  motion_bridge.c
          ↓ coordinate transforms, arc expansion, dwell, jog, canned cycles
Layer 3: Trajectory     trajectory.c
          ↓ SCurveMove structs with S-curve phase data
         Interpolator   interpolator.c
          ↓ GPIO step pulses @ 100 kHz
         Motor drivers  DRV8825 / TMC5160
```

### 7.2 The Motion Bridge — motion_bridge.c

The motion bridge is the adapter between the G-code event system and the trajectory
planner. It handles:

- Coordinate frame transforms (work → machine coordinates)
- Arc expansion into linear chord segments
- Dwell timing (non-blocking loop with `CORETIMER_CounterGet()`)
- G28 / G30 stored position rapids
- G38.x probing state machine
- G81 / G83 canned drilling state machine
- G43/G49 tool length offset application

**Key function: `MOTION_ProcessGcodeEvent()`**

Called from `APP_Tasks()` every `APP_IDLE` iteration. Processes one event at a time
from the `GCODE_CommandQueue`.

```c
// motion_bridge.c (paraphrased)
void MOTION_ProcessGcodeEvent(APP_DATA *app, GCODE_CommandQueue *q) {
    if (!GCODE_GetNextEvent(q, &event)) return;  // nothing to process

    switch (event.type) {
        case GCODE_EVENT_LINEAR_MOVE:
            // Convert work coords to machine coords (WCS + G92 + TLO)
            KINEMATICS_WorkToMachine(&event.target, &machine_target);
            KINEMATICS_LinearMove(machine_current, machine_target,
                                   event.feedrate, &move);
            TRAJECTORY_AddMove(&move);
            machine_current = machine_target;
            break;

        case GCODE_EVENT_ARC_MOVE:
            // Expand arc into N chord segments, each a LinearMove
            arc_expand_and_queue(&event);
            break;

        case GCODE_EVENT_DWELL:
            // Non-blocking dwell via CoreTimer
            dwell_until = CORETIMER_CounterGet() + (uint32_t)(event.p * CPU_FREQ_HZ);
            app->state = APP_DWELL;  // handled in APP_Tasks() dwell state
            break;

        case GCODE_EVENT_SELECT_WCS:
            KINEMATICS_SelectWCS(event.wcs_index);
            break;

        case GCODE_EVENT_SET_TLO:
            KINEMATICS_SetToolLengthOffset(event.tlo_value);
            break;

        // ... spindle, coolant, probing, etc.
    }
}
```

### 7.3 Linear Move Planning — KINEMATICS_LinearMove()

Converts a machine-coordinate linear move into an `SCurveMove` ready for the
trajectory planner. This is a port of GRBL's `plan_buffer_line()`.

```c
// kinematics.c (paraphrased)
bool KINEMATICS_LinearMove(CoordinatePoint start, CoordinatePoint end,
                             float feedrate, SCurveMove *out) {
    // 1. Compute delta and distance
    float delta[NUM_AXIS], dist = 0.0f;
    for (int i = 0; i < NUM_AXIS; i++) {
        delta[i] = end.coordinate[i] - start.coordinate[i];
        dist += delta[i] * delta[i];
    }
    dist = sqrtf(dist);
    if (dist < 1e-6f) return false;  // zero-length move

    // 2. Normalise to unit vector
    for (int i = 0; i < NUM_AXIS; i++) out->unit_vec[i] = delta[i] / dist;
    out->millimeters = dist;

    // 3. Axis-limited combined feed rate
    float rate = feedrate / 60.0f;  // mm/s
    for (int i = 0; i < NUM_AXIS; i++) {
        float axis_rate = settings.max_rate[i] / 60.0f;
        float needed = fabsf(out->unit_vec[i]) * rate;
        if (needed > axis_rate) rate = rate * axis_rate / needed;
    }
    out->nominal_speed = rate;

    // 4. Axis-limited combined acceleration
    // 5. Junction speed from previous move unit vector
    // 6. Copy steps_per_mm, fill planner bookkeeping
    // 7. Call TRAJECTORY_AddMove()
}
```

### 7.4 Arc Expansion

Arc expansion runs **inline** in the motion bridge — one chord per
`MOTION_ProcessGcodeEvent()` call. This prevents a long arc from filling the
trajectory queue before the next G-code command can be processed.

The expansion state is tracked in `app->arc_state`:

```c
typedef struct {
    bool      active;
    float     theta;            // current angle [rad]
    float     d_theta;          // angle per segment [rad]
    float     theta_end;
    float     cx, cy;           // arc centre (machine coords)
    float     radius;
    bool      clockwise;
    CoordinatePoint end_point;
    float     feedrate;
} ArcState;
```

Each call to `MOTION_ProcessGcodeEvent()` while `arc_state.active`:
1. Advances `theta` by `d_theta`
2. Computes next X/Y on the circle
3. Calls `KINEMATICS_LinearMove()` for that chord
4. If this is the last segment, uses exact `end_point` (no accumulated error)
5. If the trajectory queue is full, returns without advancing — retried next call

This backpressure mechanism ensures the trajectory queue never overflows regardless
of arc length.

### 7.5 Dwell (G4)

Dwell uses the PIC32MZ **Core Timer** (a 200 MHz free-running counter, incrementing
at CPU frequency / 2 = 100 MHz) for non-blocking timing.

```c
// motion_bridge.c — GCODE_EVENT_DWELL handling:
uint64_t dwell_ticks = (uint64_t)(event.p_seconds
                        * (float)CORETIMER_FrequencyGet());
dwell_end = CORETIMER_CounterGet() + (uint32_t)dwell_ticks;
app->state = APP_DWELL;
```

In `APP_Tasks()`, `APP_DWELL` state:
```c
case APP_DWELL:
    if ((int32_t)(CORETIMER_CounterGet() - dwell_end) >= 0) {
        app->state = APP_IDLE;
    }
    break;
```

The cast to `int32_t` handles 32-bit counter wrap correctly. A signed comparison
of `(counter - deadline)` works for any interval up to 2^31 ticks (~21 seconds).

### 7.6 Canned Drilling Cycles — G81 and G83

Canned cycles are implemented as a state machine inside `motion_bridge.c`.
Both G81 and G83 share the same top-level state enum:

```c
typedef enum {
    CANNED_IDLE,
    CANNED_RAPID_TO_XY,
    CANNED_RAPID_TO_R,
    CANNED_DRILL_FEED,
    CANNED_PECK_RETRACT,  // G83 only
    CANNED_RAPID_RETRACT,
    CANNED_DONE
} CannedState;
```

G83 adds a Q-depth loop: each iteration drills Q mm deeper, rapid retracts to R-plane,
then rapids back to the current peck depth + small clearance before the next feed.

Soft reset clears the canned state immediately.

### 7.7 Probing — G38.x

The probing state machine checks the probe pin (Z-Max, RE1) each `APP_IDLE`
iteration while a probe move is in progress. When the probe triggers:

1. `INTERPOLATOR_SoftStop()` — smooth deceleration
2. Main loop monitors `INTERPOLATOR_IsMoveComplete()`
3. On completion, read `STEPPER_GetPosition()` → latch as probe result
4. Report: `[PRB:x.xxx,y.xxx,z.xxx:1]

`  (`:0` on miss, ALARM:5 for G38.2 miss)

`$6` (probe pin invert) is applied in the probe-pin read:
```c
bool probe_triggered = (RE1_Get() != 0) ^ ((settings.probe_invert >> 0) & 1u);
```

---

## Chapter 8: Coordinate Systems and Kinematics

**Source**: `srcs/motion/kinematics.c`

### 8.1 The Coordinate Stack

Position is stored in four layers, applied cumulatively:

```
Machine position (MPos)
        ↕  WCS offset (G54–G59 per-axis)
Work position
        ↕  G92 offset (compensation, not persistent across reset)
Compensated position
        ↕  Tool Length Offset (TLO, Z-axis only)
Displayed work position (WPos in ? report)
```

All offsets are stored in `CNC_Settings` and persist to NVM.

### 8.2 Work Coordinate Systems — G54 to G59

Six WCS are supported (G54 = index 0, G59 = index 5). The **active WCS index** is
a static variable in `kinematics.c`.

```c
// kinematics.c
static int active_wcs = 0;  // 0 = G54, ..., 5 = G59

void KINEMATICS_SelectWCS(int wcs_index) {
    if (wcs_index >= 0 && wcs_index < 6) active_wcs = wcs_index;
}
```

Offsets are stored in `settings.wcs_offset[6][3]` (6 WCS × 3 axes XYZ).
The A-axis has no WCS offset (GRBL v1.1 standard).

**Setting a WCS offset (G10 L2 P1 X10 Y20 Z0):**
Sets the offset of WCS index `P1` so that machine position − offset = work position.

```c
// G10 L2 Pn: directly sets the WCS offset
settings.wcs_offset[n][axis] = event.wcs_value[axis];

// G10 L20 Pn: sets offset so current machine position = specified target position
settings.wcs_offset[n][axis] = machine_current.coordinate[axis]
                                 - event.wcs_value[axis];
```

`P0` in G10 is a sentinel for "current WCS" — resolved to `active_wcs` at event
processing time.

### 8.3 G92 Offset

G92 is a temporary overlay, not persistent across power cycle (it is persisted only
until `$RST=#` is sent or G92.1 clears it). It is stored in
`settings.g92_offset[3]`.

```c
// G92 X0 Y0 Z0: set offsets so current work position = specified value
settings.g92_offset[axis] = (work_position[axis] + current_g92[axis])
                              - commanded_value[axis];
```

### 8.4 Tool Length Offset — G43, G43.1, G49

TLO is applied to the **Z axis only** (GRBL v1.1 standard).

```c
// kinematics.c
static float tool_length_offset = 0.0f;   // active TLO in machine mm, Z only
static bool  tlo_active = false;

// G43: activate stored TLO from settings
void KINEMATICS_ActivateTLO(void) {
    tool_length_offset = settings.tool_length_offset;
    tlo_active = true;
}

// G43.1 Zn: activate dynamic (non-persisted) TLO
void KINEMATICS_SetDynamicTLO(float value) {
    tool_length_offset = value;
    tlo_active = true;
}

// G49: cancel
void KINEMATICS_CancelTLO(void) {
    tlo_active = false;
    tool_length_offset = 0.0f;
}
```

In `$G` modal state: `G43` is shown if TLO is active, `G49` otherwise.
In `$#` report: `[TLO:x.xxx]` shows the active value.

### 8.5 Work-to-Machine Coordinate Transform

```c
// kinematics.c — KINEMATICS_WorkToMachine()
void KINEMATICS_WorkToMachine(const CoordinatePoint *work, CoordinatePoint *machine) {
    for (int i = 0; i < 3; i++) {  // XYZ only
        machine->coordinate[i] = work->coordinate[i]
            + settings.wcs_offset[active_wcs][i]
            + settings.g92_offset[i];
    }
    // Apply TLO to Z axis only:
    if (tlo_active) {
        machine->coordinate[AXIS_Z] += tool_length_offset;
    }
    // A axis: no WCS offset
    machine->coordinate[AXIS_A] = work->coordinate[AXIS_A];
}
```

### 8.6 Machine-to-Work (for Status Report)

```c
// kinematics.c — KINEMATICS_MachineToWork()
void KINEMATICS_MachineToWork(const CoordinatePoint *machine, CoordinatePoint *work) {
    for (int i = 0; i < 3; i++) {
        work->coordinate[i] = machine->coordinate[i]
            - settings.wcs_offset[active_wcs][i]
            - settings.g92_offset[i];
    }
    if (tlo_active) work->coordinate[AXIS_Z] -= tool_length_offset;
    work->coordinate[AXIS_A] = machine->coordinate[AXIS_A];
}
```

This is called every time a `?` status report is generated.

---

## Chapter 9: Flow Control — Keeping Senders in Sync

**Source**: `srcs/gcode/gcode_parser.c`

### 9.1 The Problem: Queue Mismatch

Every G-code line must receive exactly one `ok`. The sender counts `ok` responses and
uses that count to decide how many more lines to send. If `ok` is sent too early
(before the motion for that line completes), the sender may believe it is done when
motion is still executing — this was the cause of UGS "premature Finished" state.

If `ok` is never sent, the sender stalls waiting.

The firmware must send `ok` **at the right time** — after it has accepted
responsibility for the command and has capacity to accept the next one.

### 9.2 Trajectory Queue High/Low Water Marks

```c
// gcode_parser.c
#define TRAJECTORY_QUEUE_HIGH_WATER   48   // defer 'ok' above this occupancy
#define TRAJECTORY_QUEUE_LOW_WATER    16   // release deferred 'ok' below this
#define TRAJECTORY_QUEUE_SIZE         64   // max SCurveMove entries
```

When the trajectory queue has fewer than `HIGH_WATER` entries, the firmware
sends `ok` immediately. When it exceeds `HIGH_WATER`, `ok` is deferred and
`okPendingCount` is incremented. When the queue drains below `LOW_WATER`, all
pending `ok` responses are released.

```c
// gcode_parser.c — SendOrDeferOk()
static void SendOrDeferOk(APP_DATA *app) {
    uint32_t qd = TRAJECTORY_GetCount();   // live trajectory queue depth
    if (qd < TRAJECTORY_QUEUE_HIGH_WATER) {
        UART_SendOK();
    } else {
        okPendingCount++;
    }
}

// Called from APP_IDLE when no new data:
void GCODE_CheckDeferredOk(APP_DATA *app) {
    if (okPendingCount > 0) {
        uint32_t qd = TRAJECTORY_GetCount();
        while (okPendingCount > 0 && qd < TRAJECTORY_QUEUE_LOW_WATER) {
            UART_SendOK();
            okPendingCount--;
        }
    }
}
```

### 9.3 Homing Deferred Ok

When `$H` is received, the `ok` for that command is held until homing completely
finishes:

```c
// gcode_parser.c
case '$H':
    s_homing_pending = true;   // block ok until homing done
    HOMING_Start();
    break;
```

In `APP_Tasks()`, after `HOMING_Tasks()` returns complete:
```c
if (s_homing_pending && !HOMING_IsActive()) {
    s_homing_pending = false;
    UART_SendOK();
}
```

### 9.4 Arc Inline Retry and Push-Back

When a `?` real-time status request arrives in the middle of an arc-segment retry
(trajectory queue full), the first byte of the G-code command being retried must
be preserved. This is the **push-back slot**:

```c
static uint8_t s_rxPushback = 0;
static bool    s_rxHasPushback = false;
```

The next `GCODE_Tasks()` call injects the push-back byte at the front of `rxBuffer`
before reading new UART data:

```c
if (s_rxHasPushback) {
    rxBuffer[0] = s_rxPushback;
    nBytesRead  = 1;
    s_rxHasPushback = false;
}
```

The `?` status is always answered immediately regardless of parser state.
UGS never times out waiting for a status response during a long arc fill.

### 9.5 UART Buffer Requirements

The TX ring buffer in `plib_uart3.c` must be at least **1024 bytes** for `$$` to
respond without overflow. The `$$` response is approximately 400–500 bytes of
settings text.

```c
// srcs/config/default/peripheral/uart/plib_uart3.c
#define UART3_READ_BUFFER_SIZE   (512U)
#define UART3_WRITE_BUFFER_SIZE  (1024U)
```

> **Warning:** MCC (MPLAB Code Configurator) regeneration may revert the TX buffer
> to 256 bytes. After **any** MCC regeneration, verify these constants.
> Symptom of wrong size: `?` and `$I` work, but `$$` causes sender disconnect.

---

## Chapter 10: The Application State Machine

**Source**: `srcs/app.c`

### 10.1 States Overview

The top-level `APP_Tasks()` function is called from `main.c`'s `while(1)` loop.
It implements a cooperative state machine with these states:

```c
// incs/app.h
typedef enum {
    APP_STATE_INIT,           // one-shot initialisation
    APP_STATE_CONFIG,         // peripheral enable, TMC config
    APP_STATE_LOAD_SETTINGS,  // read settings from NVM flash
    APP_STATE_IDLE,           // normal operation — G-code + motion
    APP_STATE_DWELL,          // waiting for G4 dwell timer
    APP_STATE_HOMING,         // $H homing cycle running
    APP_STATE_ALARM,          // limit hit or hard error
    APP_STATE_RESET           // soft reset in progress
} APP_STATES;
```

### 10.2 State Transitions

```
INIT → CONFIG → LOAD_SETTINGS → IDLE ←──────────────────────┐
                                  |                           |
                              G4 dwell ──→ DWELL ────────────┘
                                  |
                              $H ─────→ HOMING ──────────────┘ (on complete)
                                  |
                        limit hit ──→ ALARM ──→ ($X) ──→ IDLE
                                  |
                      Ctrl+X ──────→ RESET ───────────────→ INIT
```

### 10.3 APP_STATE_LOAD_SETTINGS

NVM flash must not be read until all peripheral drivers are initialised.
Reading flash during `APP_STATE_INIT` or `APP_STATE_CONFIG` can hang the firmware.
The settings load is therefore deferred to `APP_STATE_LOAD_SETTINGS`, which runs
after `APP_STATE_CONFIG` completes:

```c
case APP_STATE_LOAD_SETTINGS:
    if (SETTINGS_LoadFromFlash(SETTINGS_GetCurrent())) {
        // Successfully loaded from flash
        UART_Printf("Loaded settings from flash

");
    } else {
        // CRC mismatch / version change: use defaults already in RAM
        UART_Printf("Using default settings

");
    }
    app->state = APP_STATE_IDLE;
    break;
```

### 10.4 APP_STATE_IDLE — The Main Work Loop

The `IDLE` state is where the firmware spends the vast majority of its time. Each
iteration:

1. Check for deferred `ok` releases (`GCODE_CheckDeferredOk()`)
2. Check for pending homing-ok release
3. Check for probe trigger (if probing active)
4. Check if interpolator has finished a move (load next if available)
5. Call `GCODE_Tasks()` — read UART, parse one line or character
6. Call `MOTION_ProcessGcodeEvent()` — process one G-code event

All six steps complete in one `APP_Tasks()` iteration. The loop runs at hundreds of
kHz between G-code commands, ensuring real-time responsiveness.

### 10.5 Soft Reset — Ctrl+X

Soft reset reinitialises the firmware to a clean state without rebooting:

```c
// app.c — soft reset sequence:
INTERPOLATOR_Stop();            // stop ISR immediately
TRAJECTORY_Reset();             // flush trajectory queue
GCODE_ResetQueue(&cmdQueue);    // flush G-code command queue
KINEMATICS_ResetG92();          // clear G92 offset
memset(&rxBuffer, 0, sizeof(rxBuffer));  // flush UART accumulation buffer
nBytesRead = 0;
okPendingCount = 0;
// Clear modal state: distance mode, plane, units (retain WCS offsets and settings)
app->state = APP_STATE_INIT;    // re-init but NOT APP_STATE_RESET to flash
UART_Printf(STARTUP_BANNER_STRING);   // "Pic32mzCNC v1.1h ['\$' for help]

"
```

The banner printed on soft reset identifies the firmware version to the sender.
UGS and other senders detect this banner and reset their internal state.

---

## Chapter 11: Safety — Limits, Alarms, and Homing

**Sources**: `srcs/motion/homing.c`, `srcs/motion/motion_bridge.c`

### 11.1 Limit Switch Hardware

Each axis has two limit switches (MIN and MAX). All are read as GPIO inputs via
`axis_limit_min_get[]` and `axis_limit_max_get[]` function pointer arrays.

Limit invert mask `$5` (`settings.limit_invert`) allows active-HIGH or active-LOW
switches per axis. The invert is applied when reading:

```c
static bool LIMIT_IsTriggered(E_AXIS axis, bool check_max) {
    bool raw = check_max ? LIMIT_GetMax(axis) : LIMIT_GetMin(axis);
    bool inverted = (settings.limit_invert >> axis) & 1u;
    return raw ^ inverted;
}
```

**Soft limits** (`$20`, `$21`): checked before each motion command by comparing
the target machine position against `settings.max_travel[]`. Triggered before any
motion starts — no physical switch required.

**Hard limits**: the main loop checks limit switches each `APP_IDLE` iteration:
```c
// app.c — in APP_STATE_IDLE:
if (settings.hard_limits_enabled) {
    for (E_AXIS ax = 0; ax < NUM_AXIS; ax++) {
        if (LIMIT_IsTriggered(ax, false) || LIMIT_IsTriggered(ax, true)) {
            INTERPOLATOR_Stop();
            STEPPERS_Disable();
            app->state = APP_STATE_ALARM;
            UART_Printf("ALARM:1

");
            break;
        }
    }
}
```

### 11.2 Alarm States

| Alarm | Meaning |
|---|---|
| ALARM:1 | Hard limit triggered |
| ALARM:2 | Soft limit triggered |
| ALARM:3 | Abort during cycle |
| ALARM:4 | Probe fail — did not trigger |
| ALARM:5 | Probe fail — triggered on G38.3/5 |
| ALARM:8 | Homing fail — $H called in alarm |
| ALARM:9 | Homing fail — limit not triggered within max_travel |

Clear alarm state with `$X`. Hard limits remain checked in alarm state — the machine
will re-alarm immediately if limits are still triggered.

### 11.3 Homing Cycle — $H

**Source**: `srcs/motion/homing.c`

#### 11.3.1 Homing Order — Safety First

The homing order is:

```c
// homing.c
static const E_AXIS homing_order[] = { AXIS_Z, AXIS_Y, AXIS_X, AXIS_A };
```

**Z homes first** to lift the tool away from the workpiece before X and Y move.
This prevents crashing a milling tool into the work surface during homing.

> **Never change this order without physical analysis of the machine geometry.**

#### 11.3.2 The Four-Phase Homing Cycle

Each axis goes through four phases:

```
SEEK        Fast move toward limit switch at $25 (homing_seek_rate mm/min)
            until limit pin triggers
            ↓
BACKOFF     Short reverse move to clear the switch
            ↓
LOCATE      Slow move back toward switch at $24 (homing_feed_rate mm/min)
            until switch triggers again (precise location)
            ↓
PULLOFF     Move away by $27 (homing_pull_off mm)
            — machine position set to 0.0 on this axis at this point
```

Debounce timing uses the PIC32MZ Core Timer:
```c
// homing.c
static uint32_t debounce_start;

// On switch trigger:
debounce_start = CORETIMER_CounterGet();
homing_state = HOMING_DEBOUNCE;

// In HOMING_DEBOUNCE state:
uint32_t elapsed = CORETIMER_CounterGet() - debounce_start;
if (elapsed >= (uint32_t)(settings.homing_debounce * 1000UL)) {
    // Debounce period expired, advance to next phase
}
```

#### 11.3.3 The $22 Setting

`$22` (homing_enable) in this firmware is a **single boolean flag** (0 = disable,
non-zero = enable). It is **not** a per-axis bitmask as in stock GRBL.

The axes to home are the first 3 in `homing_order[]` that have limit switches
fitted on the PCB (Z, Y, X). The A-axis is last and has no limit switch on this
board. Sending `$22=1` enables homing for Z, Y, and X.

> **Critical lesson from debugging:** Never hardcode `axes_mask = 0x0F` in
> `parse_command_to_event()`. This would include the A-axis (bit 3), which has no
> limit switch, causing ALARM:9 on every `$H`. Always read
> `settings.homing_enable` and home only the axes listed in `homing_order[]`
> that have physical switches.

#### 11.3.4 Homing Deferred `ok`

The `ok` for the `$H` command is not sent until homing fully completes.
The parser sets `s_homing_pending = true`; the main loop releases it when
`HOMING_IsActive()` returns false.

If homing is triggered while the machine is in alarm state (`$X` not yet sent),
`error:8` is returned immediately.

---

## Chapter 12: Spindle and Laser Control

**Source**: `srcs/motion/spindle.c`

### 12.1 Spindle PWM Hardware

Spindle speed is controlled by OC8 (Output Compare 8) using TMR6 as its time base.

```
PBCLK3 = 50 MHz
TMR6:  1:64 prescaler → 781,250 Hz
PR6:   233
OC8:   PWM mode, OCTSEL=0 (TMR6)

f_PWM = 781,250 / 234 = 3,338 Hz
Duty  = OC8RS / 234  (0 = off, 233 = full speed)
```

The 3.338 kHz PWM frequency is chosen to be above audible resonance on most spindle
drivers while being low enough that their PWM input is happy.

### 12.2 M3/M5 and S-word Commands

M3 (spindle CW on) sets OC8RS based on spindle speed:
```c
// spindle.c
void SPINDLE_SetSpeed(uint32_t rpm) {
    // Clamp to $30 max RPM:
    if (rpm > settings.max_spindle_speed) rpm = settings.max_spindle_speed;
    // Map RPM to duty count:
    uint16_t duty = (uint16_t)((uint32_t)rpm * PWM_PERIOD_TICKS
                                / settings.max_spindle_speed);
    spindle_state.current_pwm_duty = duty;
    if (spindle_state.enabled) {
        OCMP8_CompareSecondaryValueSet(duty);
    }
}

void SPINDLE_Enable(bool enable) {
    spindle_state.enabled = enable;
    if (enable) {
        OCMP8_CompareSecondaryValueSet(spindle_state.current_pwm_duty);
    } else {
        OCMP8_CompareSecondaryValueSet(0);
    }
}
```

`$30` defines the maximum RPM corresponding to 100% duty (OC8RS = 233).

### 12.3 Laser Mode — $32=1

When `$32 = 1` (laser mode), the spindle PWM tracks cutting velocity. A laser cutter
must fire at reduced power when the machine is moving slowly (at the start and end of
a move, during cornering) to prevent over-burning.

#### 12.3.1 How Laser Scaling Works

At `INTERPOLATOR_LoadMove()` time, the commanded duty and laser mode flag are
**cached into ISR-visible volatile variables**:

```c
// interpolator.c — INTERPOLATOR_LoadMove()
laser_mode_active   = (settings.laser_mode != 0);
laser_duty_cmd      = SPINDLE_GetCommandedDuty();   // cached at load time
laser_nominal_speed = move->nominal_speed;           // cruise speed for this move
```

These are `volatile` because the ISR reads them. They are written only in
main-loop context (LoadMove), so no locking is needed.

Inside the ISR, every tick when laser mode is active:
```c
// interpolator.c — INTERPOLATOR_Tick():
if (laser_mode_active) {
    float scale = (laser_nominal_speed > 1e-6f)
                  ? (v_now / laser_nominal_speed) : 0.0f;
    // Clamp scale 0.0..1.0:
    if (scale > 1.0f) scale = 1.0f;
    SPINDLE_LaserScale(scale);
}
```

`SPINDLE_LaserScale()` in `spindle.c`:
```c
// spindle.c
void SPINDLE_LaserScale(float scale) {
    uint16_t duty = (uint16_t)((float)spindle_state.current_pwm_duty * scale + 0.5f);
    OCMP8_CompareSecondaryValueSet(duty);
}
```

`OCMP8_CompareSecondaryValueSet()` is a direct OC8RS register write — ISR-safe with
no internal state, no locking, and no latency.

#### 12.3.2 Laser Scaling During Feed Hold Deceleration

When `INTERPOLATOR_SoftStop()` triggers a smooth deceleration, the laser ISR path
also applies scaling during the decel ramp:

```c
// interpolator.c — hold_decel_active path:
if (laser_mode_active) {
    float scale = (v > 0.0f && laser_nominal_speed > 1e-6f)
                  ? (v / laser_nominal_speed) : 0.0f;
    SPINDLE_LaserScale(scale);
}
```

This ensures laser power ramps down proportionally as velocity drops to zero during
a feed hold. When velocity reaches zero, `scale = 0.0` → OC8RS = 0 → laser off.

#### 12.3.3 Why Cached Values — the ISR-Safety Design

The ISR runs every 10 µs. It must not call functions that read `settings` (which
might be mid-write from the main loop) or call `SPINDLE_GetCommandedDuty()` (which
reads `spindle_state.current_pwm_duty`, also modifiable from the main loop).

Caching `laser_duty_cmd` and `laser_mode_active` at `LoadMove` time — once per move,
in main-loop context — provides a stable snapshot that the ISR can safely read for
the entire duration of that move.

If the user changes `S` speed or `M5` during a move (not typical), the change takes
effect at the next `LoadMove` call.

---

## Chapter 13: Settings and NVM Persistence

**Sources**: `srcs/settings/settings.c`, `incs/settings/settings.h`

### 13.1 The CNC_Settings Struct

All GRBL parameters are stored in a single `CNC_Settings` struct. The struct layout
directly mirrors the `$$` parameter numbers where possible.

```c
// incs/settings/settings.h (selected fields):
typedef struct {
    // ── Step/direction behaviour ───────────────────────────────────────────
    uint8_t  step_invert;          // $0  — step pulse invert mask
    uint8_t  dir_invert;           // $3  — direction invert mask
    uint8_t  steppers_idle_lock;   // $1  — stepper idle delay (ms)
    uint8_t  step_pulse_us;        // $0  — step pulse width (legacy, informational)

    // ── Limit switches ─────────────────────────────────────────────────────
    uint8_t  hard_limits_enabled;  // $21
    uint8_t  limit_invert;         // $5  — per-axis invert bitmask
    uint8_t  probe_invert;         // $6  — probe pin invert

    // ── Homing ─────────────────────────────────────────────────────────────
    uint8_t  homing_enable;        // $22 — boolean (0/1); used as global enable
    uint8_t  homing_dir_mask;      // $23 — per-axis seek direction
    float    homing_feed_rate;     // $24 — slow locate rate (mm/min)
    float    homing_seek_rate;     // $25 — fast seek rate (mm/min)
    uint32_t homing_debounce;      // $26 — debounce time (ms)
    float    homing_pull_off;      // $27 — pull-off distance (mm)

    // ── Reporting ──────────────────────────────────────────────────────────
    uint8_t  report_inches;        // $13
    uint8_t  status_report_mask;   // $10

    // ── Motion per-axis (index 0=X, 1=Y, 2=Z, 3=A) ────────────────────────
    float    steps_per_mm[4];      // $100–$103
    float    max_rate[4];          // $110–$113  (mm/min)
    float    acceleration[4];      // $120–$123  (mm/s^2)
    float    max_travel[4];        // $130–$133  (mm)
    float    jerk[4];              // $140–$143  (dimensionless aggression factor)

    // ── Soft limits ────────────────────────────────────────────────────────
    uint8_t  soft_limits_enabled;  // $20

    // ── Arc ────────────────────────────────────────────────────────────────
    float    arc_tolerance;        // $12 (mm, chord deviation)
    float    junction_deviation;   // $11 (mm, corner speed blend)

    // ── Spindle / laser ────────────────────────────────────────────────────
    float    max_spindle_speed;    // $30 (RPM at 100% duty)
    float    min_spindle_speed;    // $31 (RPM at lowest non-zero duty)
    uint8_t  laser_mode;           // $32 (0=spindle, 1=laser)

    // ── Work coordinate offsets ────────────────────────────────────────────
    float    wcs_offset[6][3];     // G54–G59 offsets, XYZ per WCS
    float    g92_offset[3];        // G92 overlay, XYZ
    float    tool_length_offset;   // G43 stored TLO (Z only)

    // ── Stored positions ───────────────────────────────────────────────────
    float    g28_position[4];      // G28.1 stored position (XYZA)
    float    g30_position[4];      // G30.1 stored position (XYZA)

    // ── Startup lines ──────────────────────────────────────────────────────
    char     startup_line[2][80];  // $N0, $N1 — persisted G-code strings

    // ── TMC5160 settings (conditional) ────────────────────────────────────
    #ifdef HAS_TMC5160_AXIS
    uint8_t  tmc_mode[4];          // $200–$203 (1=StealthChop, 2=SpreadCycle, 3=Mixed)
    uint8_t  tmc_irun[4];          // $210–$213 (0–31 run current scale)
    uint8_t  tmc_ihold[4];         // $220–$223 (0–31 hold current scale)
    uint8_t  tmc_mres[4];          // $230–$233 (0=256µ, 1=128µ ... 8=full-step)
    uint32_t tmc_tpwm_thrs[4];     // $240–$243 (StealthChop threshold velocity)
    uint32_t tmc_tcoolthrs[4];     // $250–$253 (CoolStep threshold)
    #endif

    // ── Integrity ──────────────────────────────────────────────────────────
    uint32_t signature;            // 0x47524231 ('GRB1') — version marker
    uint32_t crc32;                // CRC32 of all preceding bytes
} CNC_Settings;
```

### 13.2 NVM Flash Storage

Settings are stored in one 2 KB flash **row** within the 16 KB settings page at
`0xBD1F0000` (KSEG1). The Harmony NVM PLIB handles all low-level flash operations.

**Write sequence:**
1. Erase the 16 KB page (`NVM_PageErase(0xBD1F0000)`)
2. Wait for erase to complete (callback or `NVM_IsBusy()` polling)
3. Write one 2 KB row (`NVM_RowWrite(buffer, 0xBD1F0000)`)
4. Wait for write to complete
5. Update CRC32 in struct before writing

**Read sequence** (in `APP_STATE_LOAD_SETTINGS`):
1. `memcpy` from `0xBD1F0000` into RAM struct (uncached KSEG1 address)
2. Verify `signature == 0x47524231`
3. Compute CRC32 over content and compare to stored `crc32`
4. On mismatch: restore defaults

**Why no flash read in `SETTINGS_Initialize()`:**
Reading flash during early init (before `SYS_Initialize()` fully completes) hangs
the firmware on PIC32MZ. The NVM controller requires PBCLK6 to be running, which
only happens after full peripheral init. Settings load is therefore deferred to
`APP_STATE_LOAD_SETTINGS`.

### 13.3 Default Settings

Defaults are hard-coded in `SETTINGS_RestoreDefaults()`:

```c
// settings.c — typical defaults:
s->steps_per_mm[AXIS_X] = 80.0f;   // 200 step/rev, 16x µstep, 4mm/rev lead
s->steps_per_mm[AXIS_Y] = 80.0f;
s->steps_per_mm[AXIS_Z] = 80.0f;
s->steps_per_mm[AXIS_A] = 80.0f;
s->max_rate[AXIS_X]     = 5000.0f; // mm/min
s->acceleration[AXIS_X] = 5000.0f; // mm/s^2
s->jerk[AXIS_X]         = 500.0f;  // dimensionless
s->arc_tolerance        = 0.500f;  // mm
s->junction_deviation   = 0.010f;  // mm
s->max_spindle_speed    = 12000.0f; // RPM
```

### 13.4 Settings Version Management

`SETTINGS_VERSION` (defined in `settings.h`) is incremented whenever the
`CNC_Settings` struct layout changes. If the version in flash does not match the
compiled version, defaults are loaded.

```c
#define SETTINGS_VERSION   3   // TMC5160 settings added
```

The `signature` field `0x47524231` ('GRB1') identifies that the settings block
was written by this firmware (not random data or a different firmware's settings).

---

## Chapter 14: The Debug Infrastructure

**Source**: `incs/common.h`, `incs/utils/uart_utils.h`, `srcs/utils/uart_utils.c`

### 14.1 Zero-Cost Debug Macros

The debug system is built on compile-time conditional compilation. Debug code
is completely absent from Release builds — not merely disabled or skipped at
runtime.

Flags are passed on the `make` command line:

```powershell
# Release build (default — no debug output, zero overhead):
make

# Debug build with motion and G-code tracing:
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"

# Multiple subsystems:
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE DEBUG_STEPPER DEBUG_SEGMENT"
```

### 14.2 Available Debug Flags

| Flag | Subsystem |
|---|---|
| `DEBUG_MOTION` | Motion planning, segment loading, velocity profiling |
| `DEBUG_GCODE` | G-code parsing, tokenisation, event generation |
| `DEBUG_STEPPER` | Low-level ISR, timer state, step/dir GPIO |
| `DEBUG_SEGMENT` | Trajectory segment queue management |
| `DEBUG_UART` | UART TX/RX and flow control |
| `DEBUG_APP` | Application state machine transitions |

### 14.3 Macro Definitions

In `incs/common.h`:

```c
// DEBUG_PRINT_MOTION: compile to UART_Printf in debug, to ((void)0) in release
#ifdef DEBUG_MOTION
  #define DEBUG_PRINT_MOTION(fmt, ...) UART_Printf(fmt, ##__VA_ARGS__)
  #define DEBUG_EXEC_MOTION(code)      do { code } while(0)
#else
  #define DEBUG_PRINT_MOTION(fmt, ...) ((void)0)
  #define DEBUG_EXEC_MOTION(code)      ((void)0)
#endif

// Equivalent macros for GCODE, STEPPER, SEGMENT, UART, APP subsystems
```

Usage:

```c
// In motion planning code:
DEBUG_PRINT_MOTION("[MOTION] Loading segment: dist=%.3f mm, v_entry=%.1f mm/s

",
                   move->millimeters, move->v_entry);

// In ISR (LED toggle — visible on oscilloscope):
DEBUG_EXEC_STEPPER(LED2_Toggle());

// At trajectory queue boundaries:
DEBUG_PRINT_SEGMENT("[SEG] Queue: count=%u, head=%u, tail=%u

",
                    count, head, tail);
```

In a Release build, all three of the above compile to `((void)0)` — zero bytes
of code, zero stack usage, zero CPU time.

### 14.4 UART Utilities

```c
// incs/utils/uart_utils.h
bool UART_Printf(const char *fmt, ...);      // non-blocking formatted output
bool UART_Write(const uint8_t *msg, size_t len);  // raw bytes
bool UART_SendOK(void);                      // send "ok

"
bool UART_IsReady(void);                     // TX buffer not full
```

All functions use the Harmony UART3 ring buffer. They return `false` if the TX
buffer is full (fire-and-forget; no blocking or retry). The 1024-byte TX buffer
is large enough that `$$` never drops characters.

### 14.5 Debug Workflow

1. Add `DEBUG_PRINT_XXX()` calls in the area under investigation
2. Build with `make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_XXX"`
3. Flash the hex file from `bins/Debug/CNC_V3.hex`
4. Connect PuTTY at 115200,8,N,1 — debug output appears inline with GRBL responses
5. When done: `make clean && make` (Release build — all debug removed)

---

## Chapter 15: Build System and Toolchain

### 15.1 Compiler and Tools

| Tool | Version | Purpose |
|---|---|---|
| XC32 | 4.x | MIPS C/C++ compiler for PIC32 |
| GNU make | 4.x | Build orchestration |
| Microchip Harmony 3 | 3.x | Peripheral library (PLIB) source |
| MikroE HID Bootloader | — | USB firmware flashing |
| MPLAB X IDE | optional | Code navigation (not required to build) |

### 15.2 Build Commands

All `make` commands must be run from the **repository root** (where the top-level
`Makefile` lives). Never `cd srcs` before running make — relative paths in the
Makefile assume the root as the working directory.

```powershell
# Release build (default):
make

# Incremental rebuild:
make build

# Clean then full Release build:
make clean && make

# Debug build with tracing:
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"

# Clean build artifacts for current BUILD_CONFIG:
make clean
```

### 15.3 Build Output

| Path | Contents |
|---|---|
| `bins/Release/CNC_V3.hex` | Release firmware (flash this for production) |
| `bins/Debug/CNC_V3.hex` | Debug firmware (contains UART trace output) |
| `objs/Release/` | Release object files (.o) |
| `objs/Debug/` | Debug object files (.o) |
| `libs/Release/` | Release static libraries (.a) |
| `other/Release/` | Map file, XML |

### 15.4 Compiler Flags (Key Flags)

```makefile
# Math — use hardware FPU:
-mhard-float -msingle-float -mfp64
# Math optimisation (safe for CNC motion):
-ffast-math -fno-math-errno
# Code size/speed:
-O2 -funroll-loops
# MIPS M-class architecture:
-mprocessor=32MZ2048EFH100
```

### 15.5 Flashing the Firmware

The MikroE USB HID bootloader lives at 0x9D1F4000. To flash:

1. Power the board
2. Press and hold the bootloader entry button (or apply the entry pattern)
3. Run the bootloader tool:

```powershell
# From VS Code task "Flash MikroC Bootloader (USB)":
C:/Users/davec/GIT/MikroC_bootloader/bins/mikro_hb.exe bins/Release/CNC_V3.hex
```

The bootloader identifies itself as a USB HID device. No USB driver installation
is required on Windows 10/11.

### 15.6 Linker Script and Memory Layout

The linker script places the application at 0x9D000000. The bootloader at
0x9D1F4000 has a hardware protection mechanism — an attempt to erase or write to
its region will be silently ignored by the flash controller.

The settings NVM at 0x9D1F0000 (physical 0x1D1F0000) is specifically excluded
from the linker output — it is never overwritten by a firmware flash. Settings
persist across firmware updates until `$RST=*` is sent.

---

## Appendix A: GRBL Settings Reference (`$$`)

All settings persist to NVM flash. Read with `$$`. Write with `$N=value`.

| Param | Key | Default | Units | Description |
|---|---|---|---|---|
| $0 | step_pulse_us | 10 | µs | Step pulse width (informational; hardware is 10 µs fixed) |
| $1 | steppers_idle_lock | 25 | ms | Stepper idle lock delay |
| $2 | step_invert | 0 | mask | Step pin invert bitmask (bit0=X, bit1=Y, bit2=Z, bit3=A) |
| $3 | dir_invert | 0 | mask | Direction pin invert bitmask |
| $5 | limit_invert | 0 | mask | Limit switch invert bitmask (per-axis) |
| $6 | probe_invert | 0 | bool | Probe pin invert (0=active LOW, 1=active HIGH) |
| $10 | status_report_mask | 3 | mask | Status report fields |
| $11 | junction_deviation | 0.010 | mm | Corner speed blend aggressiveness |
| $12 | arc_tolerance | 0.500 | mm | Arc chord deviation (G2/G3 segment size) |
| $13 | report_inches | 0 | bool | Report positions in inches (1) or mm (0) |
| $20 | soft_limits | 0 | bool | Enable soft limits (requires homing) |
| $21 | hard_limits | 0 | bool | Enable hard limit switch checking |
| $22 | homing_enable | 0 | bool | Enable homing cycle (`$H`) |
| $23 | homing_dir | 0 | mask | Homing seek direction (bit=1: seek positive) |
| $24 | homing_feed_rate | 100 | mm/min | Slow locate rate |
| $25 | homing_seek_rate | 2000 | mm/min | Fast seek rate |
| $26 | homing_debounce | 250 | ms | Limit switch debounce time |
| $27 | homing_pull_off | 1.0 | mm | Pull-off distance after homing |
| $30 | max_spindle_speed | 12000 | RPM | Speed at 100% PWM duty |
| $31 | min_spindle_speed | 0 | RPM | Minimum non-zero speed |
| $32 | laser_mode | 0 | bool | Laser mode (1 = power tracks velocity) |
| $100 | steps_per_mm[X] | 80.0 | steps/mm | |
| $101 | steps_per_mm[Y] | 80.0 | steps/mm | |
| $102 | steps_per_mm[Z] | 80.0 | steps/mm | |
| $103 | steps_per_mm[A] | 80.0 | steps/mm | |
| $110 | max_rate[X] | 5000 | mm/min | |
| $111 | max_rate[Y] | 5000 | mm/min | |
| $112 | max_rate[Z] | 3000 | mm/min | |
| $113 | max_rate[A] | 5000 | mm/min | |
| $120 | acceleration[X] | 5000 | mm/s² | |
| $121 | acceleration[Y] | 5000 | mm/s² | |
| $122 | acceleration[Z] | 3000 | mm/s² | |
| $123 | acceleration[A] | 5000 | mm/s² | |
| $130 | max_travel[X] | 200 | mm | Soft limit bounds |
| $131 | max_travel[Y] | 200 | mm | |
| $132 | max_travel[Z] | 100 | mm | |
| $133 | max_travel[A] | 360 | mm | |
| $140 | jerk[X] | 500 | factor | S-curve jerk aggression |
| $141 | jerk[Y] | 500 | factor | |
| $142 | jerk[Z] | 300 | factor | |
| $143 | jerk[A] | 500 | factor | |

**TMC5160 settings** (only present when `HAS_TMC5160_AXIS` is defined):

| Param | Description | Range |
|---|---|---|
| $200–$203 | Chopper mode (X/Y/Z/A): 1=StealthChop, 2=SpreadCycle, 3=Mixed, 4=CoolStep | 1–4 |
| $210–$213 | Run current scale (X/Y/Z/A) | 0–31 |
| $220–$223 | Hold current scale (X/Y/Z/A) | 0–31 |
| $230–$233 | Microstep resolution (0=256µ, 1=128µ … 8=full-step) | 0–8 |
| $240–$243 | TPWMTHRS — StealthChop to SpreadCycle crossover velocity | 0–1048575 |
| $250–$253 | TCOOLTHRS — CoolStep threshold velocity | 0–1048575 |

---

## Appendix B: GPIO Pin Table

| Signal | PIC32MZ Pin | Port.Bit | PLIB Macro | Notes |
|---|---|---|---|---|
| X STEP | RD4 | LATD.4 | `RD4_Set()` / `RD4_Clear()` | |
| X DIR | RD5 | LATD.5 | `RD5_Set()` / `RD5_Clear()` | |
| Y STEP | RD10 | LATD.10 | `RD10_Set()` / `RD10_Clear()` | |
| Y DIR | RD11 | LATD.11 | `RD11_Set()` / `RD11_Clear()` | |
| Z STEP | RE2 | LATE.2 | `RE2_Set()` / `RE2_Clear()` | |
| Z DIR | RE3 | LATE.3 | `RE3_Set()` / `RE3_Clear()` | |
| A STEP | RF3 | LATF.3 | `RF3_Set()` / `RF3_Clear()` | |
| A DIR | RF4 | LATF.4 | `RF4_Set()` / `RF4_Clear()` | |
| ENABLE (all) | RE6 | LATE.6 | `EnXYZA_Clear()` / `EnXYZA_Set()` | Active LOW; shared all 4 axes |
| X Limit MIN | RA4 | PORTA.4 | `RA4_Get()` | |
| X Limit MAX | RA7 | PORTA.7 | `RA7_Get()` | |
| Y Limit MIN | RD0 | PORTD.0 | `RD0_Get()` | |
| Y Limit MAX | RE0 | PORTE.0 | `RE0_Get()` | |
| Z Limit MIN | RD13 | PORTD.13 | `RD13_Get()` | |
| Z Limit MAX / Probe | RE1 | PORTE.1 | `RE1_Get()` | G38.x probe input |
| A Limit MIN | RA6 | PORTA.6 | `RA6_Get()` | No limit switch on this PCB |
| A Limit MAX | RB1 | PORTB.1 | `RB1_Get()` | No limit switch on this PCB |
| Spindle PWM | OC8 output | (auto) | `OCMP8_CompareSecondaryValueSet(duty)` | 3.338 kHz |
| TMC5160 SCK | SPI2 | (auto) | — | |
| TMC5160 MOSI | SPI2 | (auto) | — | |
| TMC5160 MISO | SPI2 | (auto) | — | |

---

## Appendix C: Timer Configuration Reference

| Timer | Prescaler | Base Freq | PR Register | Output Freq | Purpose |
|---|---|---|---|---|---|
| TMR4 | 1:1 | 50 MHz | PR4=499 | **100 kHz** | Step ISR heartbeat |
| TMR6 | 1:64 | 781.25 kHz | PR6=233 | **3.338 kHz** | Spindle PWM time base |
| Core Timer | — | 100 MHz | — | free-running | Dwell, homing debounce |

**Step pulse characteristics:**
- Period: 10 µs (1 / 100 kHz)
- Pulse width: 10 µs (one full tick — cleared at top of next tick)
- No secondary pulse-width timer (TMR5 not used for this purpose)

**DRV8825 compatibility:**
- Min STEP pulse width: 1.9 µs → actual 10 µs → 5.3× margin ✓
- Min DIR setup before STEP: 200 ns → DIR set in LoadMove → ∞ margin ✓

**TMC5160 compatibility:**
- Min STEP pulse width: 100 ns → actual 10 µs → 100× margin ✓
- Min DIR setup before STEP: 20 ns → DIR set in LoadMove → ∞ margin ✓

**Maximum step rate:**  
50,000 steps/s per axis (DDS fires maximum one step every 2 ticks).  
At 80 steps/mm: 50,000 / 80 = 625 mm/s = 37,500 mm/min.

---

## Appendix D: Module and File Map

```
c:/Users/davec/GIT/Pic32mzCNC_V3/
├── Makefile                        Top-level build (run make from here)
├── README.md                       Project overview and quick-start
├── STATUS.md                       Change log and hardware test results
│
├── incs/
│   ├── app.h                       APP_DATA struct, APP_STATES enum
│   ├── common.h                    Debug flags, driver config, banner string
│   ├── data_structures.h           CoordinatePoint, GCODE_CommandQueue, etc.
│   ├── settings/
│   │   └── settings.h              CNC_Settings struct, SETTINGS_VERSION
│   ├── motion/
│   │   ├── interpolator.h          INTERPOLATOR_* public API
│   │   ├── trajectory.h            SCurveMove struct, TRAJECTORY_* API
│   │   ├── kinematics.h            KINEMATICS_* API
│   │   ├── motion_bridge.h         MOTION_ProcessGcodeEvent API
│   │   ├── homing.h                HOMING_* API
│   │   └── spindle.h               SPINDLE_* API
│   └── utils/
│       └── utils.h                 GPIO arrays, inline helpers, AxisSettings
│
├── srcs/
│   ├── Makefile                    Inner build (sources, flags, dependencies)
│   ├── main.c                      Entry point, SYS_Initialize, APP_Tasks loop
│   ├── app.c                       Application state machine (APP_Tasks)
│   ├── gcode/
│   │   ├── gcode_parser.c          GRBL protocol, line parsing, flow control
│   │   └── utils.c                 Tokeniser, modal splitting, string helpers
│   ├── motion/
│   │   ├── interpolator.c          TMR4 ISR, DDS step generator, LoadMove
│   │   ├── trajectory.c            S-curve planner, 3-pass lookahead
│   │   ├── kinematics.c            Coord transforms, LinearMove, junction
│   │   ├── motion_bridge.c         G-code event → trajectory move bridge
│   │   ├── homing.c                4-phase homing state machine
│   │   └── spindle.c               OC8/TMR6 PWM, laser scaling
│   ├── settings/
│   │   └── settings.c              NVM flash read/write, defaults, CRC32
│   └── utils/
│       ├── utils.c                 GPIO array definitions, AxisSettings init
│       └── uart_utils.c            Non-blocking UART helpers
│
├── docs/
│   └── readme/
│       ├── CNC_FIRMWARE_BOOK.md    This document
│       ├── ARCHITECTURE.md         System architecture overview
│       ├── SETTINGS_REFERENCE.md   Quick settings lookup
│       └── DEBUG_SYSTEM_TUTORIAL.md  Debug macro usage guide
│
├── tests/                          Hardware-validated G-code test files
│   ├── 02_rectangle_path.gcode     Simple rectangle (corners, origin return)
│   ├── 03_circle_20segments.gcode  Circle, <0.025 mm error
│   ├── 05_three_arcs_simple.gcode  CW/CCW arc handoff
│   ├── 07_complex_long_run_fast.gcode  Full pipeline test
│   ├── 08_arc_cw_ccw_stress.gcode  36 arcs, 2:18 run time
│   └── 09_concentric_semicircles.gcode  X error <0.031 mm
│
└── ps_commands/
    ├── test_gcode.ps1              Stream a G-code file over serial
    ├── test_homing.ps1             Exercise homing cycle
    └── stream_gcode.ps1            General streaming script
```

---

## Appendix E: ISR Budget Analysis

The TMR4 ISR (`INTERPOLATOR_Tick()`) runs every 10 µs.
At 200 MHz CPU and 5-stage MIPS pipeline, the available budget is:

```
Budget = 200,000,000 Hz × 10 µs = 2,000 CPU cycles per tick
```

**Approximate ISR cycle count (Release build, -O2):**

| Operation | Cycles (approx) |
|---|---|
| ISR entry (context save, interrupt clear) | 30 |
| Step pin clear loop (4 axes) | 16 |
| Elapsed time computation | 15 |
| `TRAJECTORY_VelocityAt()` (7-phase scan + quadratic) | 40 |
| Feed override multiply | 5 |
| Laser scale (when active) | 20 |
| DDS increment computation (4 × float multiply + cast) | 40 |
| DDS accumulate and fire (4 axes) | 60 |
| Ticks-remaining decrement + end-condition check | 10 |
| ISR exit (context restore, return) | 20 |
| **Total (normal path, no laser)** | **~236 cycles** |
| **Total (laser mode active)** | **~256 cycles** |
| **Total (penultimate tick, deficit flush)** | **~300 cycles** |
| **Available budget** | **2,000 cycles** |
| **Headroom (normal path)** | **~88% (1,764 cycles unused)** |

The ISR uses approximately **12% of the available CPU budget** in the normal path.
This leaves ample headroom for:
- Higher step rates above 100 kHz if needed
- Additional axes
- More complex velocity profiles
- Any future ISR additions

The penultimate-tick deficit flush may fire up to 8 extra steps (2 per axis × 4 axes)
in the worst case, each costing ~8 cycles — well within budget.

---

## Production Validation Status

The following test files have been validated on real hardware (March 2026).
Tests run in both UGS single-step and pipelined streaming modes.

| Test file | Mode | Duration | Result |
|---|---|---|---|
| `08_arc_cw_ccw_stress.gcode` | Both | 2:18 min | PASS — 36 arcs, exact origin return |
| `09_concentric_semicircles.gcode` | Pipeline | 56 s / 55 s | PASS — X error < 0.031 mm |
| `05_three_arcs_simple.gcode` | Both | — | PASS — CW/CCW arc handoff stable |
| `02_rectangle_path.gcode` | Both | — | PASS — sharp corners, origin return |
| `03_circle_20segments.gcode` | Both | — | PASS — < 0.025 mm error |
| `07_complex_long_run_fast.gcode` | Both | 1:59 min | PASS — arcs, linears, G4, overrides, origin |

---

*End of Pic32mzCNC_V3 Firmware Reference*

