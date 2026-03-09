# Pic32mzCNC_V3 — CNC Motion Control System

GRBL v1.1 compatible 4-axis CNC motion controller for the PIC32MZ2048EFH100, targeting LitePlacer pick-and-place and general CNC applications. Supports mixed TMC5160 (SPI) and DRV8825/A4988/TMC2208 (step-dir) stepper drivers on the same board.

---

## 🚀 Status

**Branch**: `scurve_motion` (active development mainline)
**Firmware**: `bins/CNC_V3.hex`
**Last build**: March 9, 2026

| Feature | Status |
|---------|--------|
| GRBL v1.1 protocol | ✅ Complete |
| 4-axis coordinated motion (XYZA) | ✅ Complete |
| S-curve velocity profiling (jerk-limited, 7-phase) | ✅ Complete |
| GRBL-exact look-ahead planner (reverse+forward+junction) | ✅ Complete |
| Arc interpolation G2/G3 + radius compensation | ✅ Complete |
| Homing ($H) — 4-phase cycle | ✅ Complete |
| Spindle PWM (M3/M5 via OC8/TMR6) | ✅ Complete |
| NVM flash settings persistence | ✅ Complete |
| Flow control — deferred "ok" | ✅ Complete |
| Soft reset recovery (Ctrl+X) | ✅ Complete |
| Feed Hold (`!`) / Resume (`~`) — graceful drain | ✅ Complete |
| E-Stop hardware interrupt (RF4 IPL7, Hold:1→Hold:0) | ✅ Complete |
| G38.2/3/4/5 probe commands | ✅ Implemented — testing pending |
| Per-axis driver selection (TMC5160 / DRV8825) | ✅ Complete |
| TMC5160 SPI driver + runtime settings ($200–$253) | ✅ Complete |
| StallGuard2 / sensorless homing | 🔲 Phase 2 (deferred) |

---

## ⚙️ Hardware

| Item | Detail |
|------|--------|
| MCU | PIC32MZ2048EFH100 |
| CPU clock | 200 MHz (MIPS32r2) |
| Peripheral clock | 50 MHz (PBCLK3) |
| CoreTimer | 100 MHz (CPU/2) |
| FPU | Single-precision hardware (`-mhard-float -msingle-float`) |
| Step timer | TMR4 1:64 → 781.25 kHz (1.28 µs/tick) — overflow fires step ISR |
| Step pulse | OC1 dual-compare auto-generates GPIO pulse in hardware |
| Spindle PWM | OC8 / TMR6 @ 3.338 kHz |
| SPI2 | Mode 3, ~1.923 MHz (BRG=12), 8-bit, MSSEN=0 |
| Bootloader | MikroE USB HID @ 0x9D1F4000 (48 KB) |

### GPIO Pin Assignments

| Signal | Pin | Notes |
|--------|-----|-------|
| EnXYZA | RE6 | Shared enable for all axes (active LOW) |
| SPI2_CS_X | RA1 | TMC5160 chip-select X |
| SPI2_CS_Y | RF13 | TMC5160 chip-select Y |
| SPI2_CS_Z | RF12 | TMC5160 chip-select Z |
| SPI2_CS_A | RB12 | TMC5160 chip-select A |
| X Min/Max | RA4 / RA7 | Limit switches |
| Y Min/Max | RD0 / RE0 | Limit switches |
| Z Min/Max | RD13 / RE1 | Limit / probe (Z-Max = probe input) |
| A Min/Max | RA6 / RB1 | Limit switches |

---

## 🔧 Driver Configuration

Per-axis driver type is set at compile time in [`incs/common.h`](incs/common.h). Edit the four `AXIS_x_DRIVER` lines to match physical wiring — everything else is auto-derived.

```c
// ── Per-axis driver assignment ────────────────────────────────────────────────
#define AXIS_X_DRIVER   DRIVER_DRV8825   // change to DRIVER_TMC5160 when fitted
#define AXIS_Y_DRIVER   DRIVER_DRV8825
#define AXIS_Z_DRIVER   DRIVER_DRV8825
#define AXIS_A_DRIVER   DRIVER_DRV8825
```

**Valid values**: `DRIVER_TMC5160` or `DRIVER_DRV8825`
Mixing is fully supported (e.g. X+Y = TMC5160, Z+A = DRV8825).

**Auto-derived flags** (do not edit):
- `HAS_TMC5160_AXIS` — defined when ≥1 axis is TMC5160; gates all SPI / TMC register code
- `HAS_DRV8825_AXIS` — defined when ≥1 axis is DRV8825; gates per-axis enable arrays
- `TMC5160_AXIS_MASK` — compile-time bitmask used by mixed-driver enable inlines

> **PCB note**: There is **one shared EN pin** (`EnXYZA` RE6) for all axes. There are no per-axis `EnX/EnY/EnZ/EnA` GPIO macros — they do not exist in `plib_gpio.h`. All enable logic routes through `enable_all_set()` / `enable_all_clear()`.

---

## 🏗️ Architecture

### Key Design Concept

**Virtual Dominant Axis** — `TIMER_4_InterruptHandler` fires only at the actual step rate needed, not at a fixed background rate. CPU load scales with motion speed, not a fixed overhead.

### Motion Pipeline

```
UART RX → Parser (inline dispatch) → Trajectory Planner → Interpolator → TIMER_4_ISR
               ↓                           ↓                   ↓               ↓
          Flow Control             S-curve velocity       DDS step        Bresenham
        (backpressure/ok)         (64-slot lookahead)    generator     subordinate axes
```

G-code commands are dispatched **inline** — bypassing the gcode queue for normal streaming — so arc backpressure is applied immediately at the trajectory level, not deferred.

### Call Hierarchy

```
main.c
  └── app.c — APP_Tasks() state machine
        ├── APP_INIT        → settings.c (register NVM callback)
        ├── APP_LOAD_SETTINGS → settings.c (read flash after peripherals ready)
        ├── APP_CONFIG      → TMC5160_Initialize() [#ifdef HAS_TMC5160_AXIS]
        ├── APP_IDLE
        │     ├── gcode_parser.c — parse/dispatch G-code, flow control
        │     ├── motion_bridge.c — G-code event → trajectory move bridge
        │     ├── trajectory.c — S-curve planner, 64-slot lookahead queue
        │     ├── interpolator.c — fixed-rate DDS step generator (TMR4 ISR)
        │     ├── kinematics.c — coordinate transforms (Work ↔ Machine)
        │     └── TMC5160_Tasks() [rate-limited 10 Hz, #ifdef HAS_TMC5160_AXIS]
        ├── APP_HOMING      → homing.c ($H — 4-phase seek/locate/pulloff/complete)
        └── APP_ALARM       → stepper.c (emergency stop, STEPPER_DisableAll)

ISR (asynchronous)
  ├── TIMER_4_ISR  — Bresenham step pulses (stepper.c, IPL6)
  ├── TMR5_Callback — pulse width timing (stepper.c)
  ├── OC8 / TMR6 — spindle PWM (spindle.c)
  ├── UART3_ISR — ring buffer RX/TX
  ├── INT1_ISR  — Probe contact RF3 (app.c, IPL6)
  └── INT3_ISR  — E-Stop button RF4 (app.c, IPL7 — highest priority)
```

### ISR Architecture

All Bresenham interpolation and velocity profiling execute **inside** `TIMER_4_InterruptHandler` — there is no separate main-loop phase system. The PIC32MZ M14K shadow register set at IPL6 swaps the entire CPU register file in 1 cycle on ISR entry/exit, giving zero push/pop stack cost.

**Per-step sequence inside `TIMER_4_InterruptHandler`:**

1. Cache `seg = currentSegment` (NULL guard — immediate return if not loaded)
2. **Dominant axis STEP HIGH** — `switch(dominant_axis)` → Harmony `#define` macro → single `LATxSET` write. Compiler emits a jump table; no function call, no pointer indirection.
3. Position counter update (`AXIS_IncrementSteps` / `AXIS_DecrementSteps` — genuinely inlined; address never placed in a pointer array)
4. **Bresenham loop** (0–3 subordinate axes, bounded worst-case): `error[axis] += abs_delta[axis]` → step if `error >= dominant_delta` → same `switch`/Harmony macro pattern
5. `T5CONSET = _T5CON_ON_MASK` — start one-shot pulse-width timer (direct SFR)
6. **Velocity profiling** — integer arithmetic only, no float, no division:
   - Accel: `step_interval -= rate_delta` (floor at `nominal_rate`)
   - Decel: `step_interval += decel_rate_delta` (ceil at `final_rate`)
7. `PR4 = period` — direct SFR write sets next TMR4 rollover interval. TMR4 silicon never changes board-to-board so no Harmony abstraction is needed here.
8. `seg->steps_completed++`
9. **Segment completion check** — if `steps_completed >= steps_remaining`: write `T4CONCLR = _T4CON_ON_MASK` directly (stops TMR4 in silicon *inside the ISR*, before returning) → set `motionSegmentCompleted = true` → `return`. This eliminates phantom ISR calls that would otherwise occur on the next TMR4 rollover after the final step.

**TMR5 callback** (pulse width ~2.56 µs later): `T5CONCLR` stops timer; four unrolled Harmony macros `StepX_Clear()…StepA_Clear()` cleared unconditionally.

**GPIO vs Timer/OC split:**
- **GPIO** (board-specific, MCC-managed): Harmony `#define` macros (`StepX_Set()` etc.) — MCC pin renames propagate automatically.
- **Timer** (silicon, never board-specific): direct SFR registers (`PR4`, `T5CONSET`, `T5CONCLR`).

### Dwell (G4)

Dwell segments (`SEGMENT_TYPE_DWELL`) bypass the TMR4 step ISR entirely. `STEPPER_LoadSegment()` records a CoreTimer start value and duration (CoreTimer runs at 100 MHz = CPU/2), then returns immediately. The segment is held in `currentSegment` until `STEPPER_IsDwellComplete()` confirms the elapsed CoreTimer ticks exceed `dwell_duration`. No step pulses are generated; the motion queue does not advance until the dwell expires.

### Safety Flags

Three volatile flags gate motion and new command processing at runtime:

| Flag | Set by | Cleared by | Effect |
|------|--------|------------|--------|
| `g_feed_hold_active` | `!` real-time char | `~` resume or soft reset | Gates new event processing and arc generation; current segment drains completely |
| `g_feed_hold_pending` | `!` real-time char | motion queue drains to 0 | Status reports `Hold:1` (decelerating); transitions to `Hold:0` when queue empty |
| `g_suppress_hard_limits` | soft reset (Ctrl+X) | all limit pins physically release | Allows jogging off a triggered limit after reset without ALARM |
| `g_estop_pending` | `ESTOP_Callback` ISR (RF4 IPL7) | soft reset / main loop | Triggers immediate `STEPPER_DisableAll()` and `APP_ALARM` state |

### GRBL-Exact Look-Ahead Planner

`MOTION_PlannerRecalculate()` executes a **three-pass algorithm** (exact port of GRBL's `planner_recalculate()`) after every segment is enqueued:

1. **Reverse pass** (newest → oldest): propagates deceleration constraints backward — each segment's `entry_speed_sqr` is capped by how fast it can decelerate to the next segment's planned entry.
2. **Forward pass** (oldest → newest): propagates acceleration constraints forward — caps any entry that the preceding segment cannot reach from a standing start given its distance.
3. **Trapezoid pass**: calls `KINEMATICS_RecalculateTrapezoid()` on every buffered segment to write the final ISR timing fields.

Junction speed uses GRBL's centripetal approximation (dot-product only, no trig):  
$v_{junction}^2 = \frac{a \cdot e_{d} \cdot \sin(\theta/2)}{1 - \sin(\theta/2)}$
where $e_d$ is `junction_deviation` ($11) and $a$ is the axis-limited acceleration for the bisector direction.

`KINEMATICS_LinearMove()` uses `convert_to_unit_vector()` + `limit_acceleration_by_axis()` + `limit_rate_by_axis()` (GRBL's `limit_value_by_axis_maximum()`) so that any multi-axis move is correctly constrained to the slowest limiting axis, not just the dominant axis.

Per-segment planner fields stored in `MotionSegment`: `unit_vec[NUM_AXIS]`, `entry_speed_sqr`, `max_entry_speed_sqr`, `max_junction_speed_sqr`, `millimeters`. ISR timing fields written by `RecalculateTrapezoid`: `initial_rate`, `nominal_rate`, `final_rate`, `accelerate_until`, `decelerate_after`. All ISR reads are integers — no float, no division inside the ISR.

### Flow Control

```
Command received
  gcodeQ >= HIGH_WATER (48)? → defer "ok"
  gcodeQ <  HIGH_WATER (48)? → send "ok" immediately

During streaming
  gcodeQ <= LOW_WATER (16)? → burst deferred "ok" responses
                               until sender can refill back toward HIGH_WATER

Queue drained to zero
  only flush final deferred "ok" responses after:
    - trajectory/interpolator motion is complete
    - arc generation is idle
```

---

## 📡 GRBL v1.1 Protocol

### Supported G-Codes

| Code | Description |
|------|-------------|
| G0, G1 | Rapid / linear move |
| G2, G3 | Arc (CW / CCW), helical |
| G4 | Dwell (P seconds) |
| G17, G18, G19 | Plane selection |
| G20, G21 | Inches / mm |
| G28, G30 | Go to predefined position |
| G38.2–G38.5 | Probe toward/away |
| G90, G91 | Absolute / incremental |
| G92 | Set current position |
| G10 L20 | Set work coordinate offset |

### Supported M-Codes

| Code | Description |
|------|-------------|
| M3, M4, M5 | Spindle CW / CCW / off |
| M7, M8, M9 | Coolant mist / flood / off |

### Real-Time Commands

| Char | Action |
|------|--------|
| `?` | Status report (no "ok") |
| `!` | Feed hold — current segment completes (Hold:1), then hardware stops (Hold:0) |
| `~` | Resume from Hold:0 or cancel pending Hold:1 drain |
| Ctrl+X | Soft reset + GRBL banner |

### System Commands

| Command | Description |
|---------|-------------|
| `$$` | Print all settings |
| `$n=v` | Set parameter n to value v |
| `$I` | Build info |
| `$G` | Modal state |
| `$#` | Work offsets |
| `$H` | Home all axes |
| `$X` | Clear alarm |
| `$C` | Toggle check mode |
| `$RST=*` / `$RST=$` / `$RST=#` | Reset all / settings / WCS |

---

## ⚙️ Settings Reference

### Standard GRBL Parameters ($0–$132)

| Param | Description | Default |
|-------|-------------|---------|
| $0 | Step pulse width (µs) | 10 |
| $1 | Step idle delay (ms) | 25 |
| $2 | Step port invert mask | 0 |
| $3 | Direction port invert mask | 0 |
| $4 | Step enable invert | 0 |
| $5 | Limit pins invert | 0 |
| $6 | Probe pin invert | 0 |
| $10 | Status report mask | 1 |
| $11 | Junction deviation (mm) | 0.010 |
| $12 | Arc tolerance (mm/segment) | 0.002 |
| $13 | Arc radius compensation tolerance (mm) | 0.002 |
| $20 | Soft limits enable | 0 |
| $21 | Hard limits enable | 0 |
| $22 | Homing cycle enable | 0 |
| $23 | Homing direction mask | 0 |
| $24 | Homing feed rate (mm/min) | 500 |
| $25 | Homing seek rate (mm/min) | 2000 |
| $26 | Homing debounce (µs) | 250 |
| $27 | Homing pull-off (mm) | 2.0 |
| $100–$103 | Steps/mm — X Y Z A | 80 |
| $110–$113 | Max rate (mm/min) — X Y Z A | 5000 |
| $120–$123 | Acceleration (mm/s²) — X Y Z A | 200 |
| $130–$133 | Max travel (mm) — X Y Z A | 200 |

### TMC5160 Parameters ($200–$253) — active only when `HAS_TMC5160_AXIS`

Applied immediately via SPI on write; persisted to NVM flash (SETTINGS_VERSION = 3).

| Params | Axis | Description | Default |
|--------|------|-------------|---------|
| $200–$203 | X–A | Chopper mode: 1=StealthChop 2=SpreadCycle 3=Mixed 4=CoolStep | 1 |
| $210–$213 | X–A | Run current (0–31) | 20 |
| $220–$223 | X–A | Hold current (0–31) | 10 |
| $230–$233 | X–A | Microstep resolution (0=256µ … 8=full step) | 4 (16µ) |
| $240–$243 | X–A | TPWMTHRS — StealthChop→SpreadCycle crossover | 500 |
| $250–$253 | X–A | TCOOLTHRS — CoolStep lower velocity threshold | 0 |

Format: `$2XY=value` — X = tens digit (0=mode, 1=irun, 2=ihold, 3=mres, 4=pwmthrs, 5=coolthrs), Y = axis (0=X, 1=Y, 2=Z, 3=A).

**Example**: `$211=25` → set X-axis run current to 25.

---

## 🔍 G38.x Probe

Implemented using Z-Max limit pin as probe input. The `$6` setting inverts the polarity for NO/NC switch wiring.

| Command | Behaviour |
|---------|-----------|
| G38.2 | Probe toward — alarm on no contact |
| G38.3 | Probe toward — no alarm on miss |
| G38.4 | Probe away — alarm on no contact |
| G38.5 | Probe away — no alarm on miss |

**Response on success**: `[PRB:x,y,z,a:1]` then `ok`
**Response on failure**: `[PRB:x,y,z,a:0]` then `ok` (or `ALARM:5` for G38.2/G38.4)

Position is captured at the exact moment of trigger — no automatic backoff.

---

## 🗂️ File Structure

```
Pic32mzCNC_V3/
├── srcs/
│   ├── app.c                  # Main state machine
│   ├── main.c                 # Entry point
│   ├── gcode/
│   │   └── gcode_parser.c     # GRBL protocol, flow control, G38.x
│   ├── motion/
        ├── stepper.c          # TMR4 step ISR, Bresenham
│   │   ├── motion.c           # Segment queue, phase system
│   │   ├── kinematics.c       # Velocity profiling, arc math
│   │   ├── homing.c           # $H 4-phase cycle
│   │   ├── spindle.c          # M3/M5 OC8 PWM
│   │   ├── motion_utils.c     # Enable/limit helpers
│   │   └── tmc5160.c          # TMC5160 SPI driver [HAS_TMC5160_AXIS]
│   ├── settings/
│   │   └── settings.c         # NVM flash persistence
│   └── utils/
│       ├── utils.c            # GPIO abstraction (step/dir/enable/limit arrays)
│       └── uart_utils.c       # Non-blocking UART helpers
├── incs/
│   ├── common.h               # Driver selection, debug flags
│   ├── data_structures.h      # APP_DATA, MotionSegment, GCODE_CommandQueue
│   ├── motion/tmc5160.h       # TMC5160 register map and API
│   └── utils/utils.h          # Inline GPIO functions, PROBE_Get()
├── bins/
│   └── CNC_V3.hex             # Production firmware
├── docs/readme/               # Detailed reference documents
├── gcode_tests/               # Test G-code files
├── tests/                     # Validation G-code files
└── ps_commands/               # PowerShell test scripts
```

---

## 🔨 Build

Always run `make` from the repository root.

```powershell
# Release build (default)
make

# With serial debug output
make DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"

# Clean then build
make clean && make
```

**Output**: `bins/CNC_V3.hex`

**Available `DEBUG_FLAGS`** (compile-time, zero release overhead):

| Flag | Traces |
|------|--------|
| `DEBUG_MOTION` | Motion planning, segment execution |
| `DEBUG_GCODE` | G-code parsing, event processing |
| `DEBUG_STEPPER` | ISR, pulse generation |
| `DEBUG_SEGMENT` | Segment loading, queue management |
| `DEBUG_UART` | UART communication |
| `DEBUG_APP` | Application state machine |

### Flash

```powershell
# USB HID bootloader
C:\Users\davec\GIT\MikroC_bootloader\bins\mikro_hb.exe bins\CNC_V3.hex
```

Or use the **Flash MikroC Bootloader (USB)** task in VS Code.

---

## 🧪 Testing

```powershell
# Run a G-code file
.\ps_commands\test_gcode.ps1 -FilePath .\gcode_tests\01_simple_square.gcode

# Rectangle (double iteration)
.\ps_commands\test_double_rectangle.ps1
```

**Manual via PuTTY** (115200, 8N1):

```
?              # Status report
$$             # View all settings
$H             # Home all axes
$X             # Clear alarm
G90 G21        # Absolute, mm
G1 X10 F500    # Move X 10 mm
G38.2 Z-10 F100  # Probe toward Z
```

### Validated Results

All tests validated in hardware (`49e1bf6`, `scurve_motion` branch) — both UGS single-step and pipelined streaming modes.

| Test file | Mode | Result | Notes |
|-----------|------|--------|-------|
| `08_arc_cw_ccw_stress.gcode` | Single-step | ✅ Pass | 36 arcs, 2:18 min, return to origin |
| `08_arc_cw_ccw_stress.gcode` | Pipeline | ✅ Pass | Smooth continuous, zero stop-starts |
| `09_concentric_semicircles.gcode` | Pipeline | ✅ Pass ×2 | 56s/55s, X error <0.031 mm |
| `05_three_arcs_simple.gcode` | Both | ✅ Pass | CW/CCW arc handoff stable |
| `02_rectangle_path.gcode` | Both | ✅ Pass | Corners sharp, return to origin |
| `03_circle_20segments.gcode` | Both | ✅ Pass | 20-segment circle, <0.025 mm error |
| `07_complex_long_run_fast.gcode` | Both | ✅ Pass | Full pipeline: arcs, linears, G4, return |
| G38.x probe | — | ⏳ Pending hardware test | |

---

## 💾 Memory Map

```
0x9D000000 – 0x9D17FFFF   Application code (~1.5 MB)
0x9D180000 – 0x9D183FFF   GRBL settings NVM (16 KB, KSEG0 0x9D...)
0x9D1F4000 – 0x9D1FFFFF   MikroE USB HID bootloader (48 KB)
0xBFC00000 – 0xBFC02FFF   Boot flash / config words (12 KB)
```

> NVM writes use KSEG0 (0x9D…) addresses — **not** KSEG1 (0xBD…).

---

## 📚 Reference Documents

- [Settings Reference](docs/readme/SETTINGS_REFERENCE.md)
- [Debug System Tutorial](docs/readme/DEBUG_SYSTEM_TUTORIAL.md)
- [Memory Map](docs/readme/MEMORY_MAP.md)
- [Architecture](docs/readme/ARCHITECTURE.md)
- [LitePlacer GRBL Implementation](docs/readme/LITEPLACER_GRBL_IMPLEMENTATION.md)
- [Development Log](STATUS.md)

---

**Hardware**: PIC32MZ2048EFH100 @ 200 MHz
**Compiler**: XC32 v4.60
**Repository**: github.com/Davec6505/Pic32mzCNC_V3
**Integration target**: [LitePlacer-DEV](https://github.com/Davec6505/LitePlacer-DEV)
