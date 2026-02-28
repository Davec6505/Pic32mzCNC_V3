# Pic32mzCNC_V3 — CNC Motion Control System

GRBL v1.1 compatible 4-axis CNC motion controller for the PIC32MZ2048EFH100, targeting LitePlacer pick-and-place and general CNC applications. Supports mixed TMC5160 (SPI) and DRV8825/A4988/TMC2208 (step-dir) stepper drivers on the same board.

---

## 🚀 Status

**Branch**: `tmc5160` (active development mainline)
**Firmware**: `bins/CNC_V3.hex`
**Last build**: February 28, 2026

| Feature | Status |
|---------|--------|
| GRBL v1.1 protocol | ✅ Complete |
| 4-axis coordinated motion (XYZA) | ✅ Complete |
| Trapezoidal velocity profiling | ✅ Complete |
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
| Step timer | TMR4 1:64 → 781.25 kHz (1.28 µs/tick) |
| Step output | OC1 continuous-pulse dual-compare mode |
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

**Virtual Dominant Axis** — OC1 fires the ISR only at the actual step rate needed, not at a fixed 30 kHz background rate. CPU load scales with motion speed, not a fixed overhead.

### Motion Pipeline

```
UART RX → Parser → Event Queue → Kinematics → Motion Queue → STEPPER_LoadSegment → OC1 ISR
                        ↓              ↓              ↓                               ↓
                   Flow Control   Velocity       Position                      Bresenham
                   (defer "ok")   Profile        Tracking                   subordinate axes
```

### Call Hierarchy

```
main.c
  └── app.c — APP_Tasks() state machine
        ├── APP_INIT        → settings.c (register NVM callback)
        ├── APP_LOAD_SETTINGS → settings.c (read flash after peripherals ready)
        ├── APP_CONFIG      → TMC5160_Initialize() [#ifdef HAS_TMC5160_AXIS]
        ├── APP_IDLE
        │     ├── motion.c  — motion queue, phase system
        │     ├── gcode_parser.c — parse events, flow control
        │     ├── kinematics.c — generate motion segments
        │     ├── stepper.c — load segments to TMR4/OC1
        │     └── TMC5160_Tasks() [rate-limited 10 Hz, #ifdef HAS_TMC5160_AXIS]
        ├── APP_HOMING      → homing.c ($H — 4-phase seek/locate/pulloff/complete)
        └── APP_ALARM       → stepper.c (emergency stop, STEPPER_DisableAll)

ISR (asynchronous)
  ├── OCP1_ISR  — Bresenham step pulses (stepper.c, IPL5)
  ├── TMR5_Callback — pulse width timing (stepper.c)
  ├── OC8 / TMR6 — spindle PWM (spindle.c)
  ├── UART3_ISR — ring buffer RX/TX
  └── CN-F_ISR  — E-Stop button RF4 (stepper.c, IPL7 — highest priority)
```

### Priority Phase System

The main loop processes motion phases in strict priority order so G-code parsing never blocks step timing:

```c
MOTION_PHASE_VELOCITY  = 0   // update step rate (accel/cruise/decel)
MOTION_PHASE_BRESENHAM = 1   // accumulate error terms
MOTION_PHASE_SCHEDULE  = 2   // write OCx registers
MOTION_PHASE_COMPLETE  = 3   // check segment done, load next
MOTION_PHASE_IDLE      = 255 // safe to process G-code / UART
```

### Flow Control

```
Command received
  motionQueueCount > 0? → defer "ok" (okPending = true)
  motionQueueCount == 0? → send "ok" immediately

Segment completes → motionSegmentCompleted flag
  → CheckDeferredOk(): if okPending && count==0 → send "ok"
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
│   │   ├── stepper.c          # TMR4/OC1 hardware, ISR, Bresenham
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

| Test | Result |
|------|--------|
| Rectangle — dual iteration | ✅ Pass |
| Circle — 20 segments, 0.025 mm error | ✅ Pass |
| Arc radius compensation | ✅ Pass |
| Back-to-back file execution | ✅ Pass |
| Soft reset recovery (Ctrl+X) | ✅ Pass |
| G38.x probe | ⏳ Pending hardware test |

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
