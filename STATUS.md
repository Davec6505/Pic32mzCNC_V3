# Pic32mzCNC_V3 - Development Status Tracker

**Branch**: `lookahead`  
**Feature**: LitePlacer G38.x Probe + TMC5160 SPI Driver  
**Started**: February 10, 2026  
**Last Updated**: March 1, 2026

---

## ✅ Documentation: UML + README Updated for lookahead Architecture — March 1, 2026

Updated all architecture documentation to reflect current `lookahead` branch implementation. Removed all references to the deleted Priority Phase System and old TMR2/OC2/OC3/OC4 per-axis ISR architecture.

- `docs/plantuml/02_segment_clock.puml` — **Complete rewrite**: Now documents period-based TMR4/OC1/TMR5 single-ISR architecture, M14K shadow register swap, switch/Harmony macro GPIO pattern, direct-SFR Timer/OC, Bresenham inside ISR, velocity profiling inside ISR, pulse-width TMR5 callback, race window note
- `docs/plantuml/01_system_overview.puml` — **Complete rewrite**: Removed TMR2 free-running, OC2/OC3/OC4 per-axis ISRs, Priority Phase enum. Added `OCP1_ISR` single-ISR with shadow regs, look-ahead planner in kinematics, arc incremental generator
- `docs/plantuml/03_arc_linear_interpolation.puml` — **Targeted fix**: Replaced `[Priority Phase\nSystem]` note with `[MOTION_Tasks Segment Queue]`. Replaced `[Hardware Timers OC1/OC2/OC3/OC4]` with `[OCP1_ISR TMR4/OC1/TMR5]`
- `README.md:9` — Branch updated: `tmc5160` → `lookahead`
- `README.md:11` — Last build date updated: February 28, 2026 → March 1, 2026
- `README.md:19` — Feature table: Added "Look-ahead backward-pass planner (junction velocity) ✅ Complete"
- `README.md:112` — Call hierarchy: "motion.c — motion queue, phase system" → "motion.c — segment queue, look-ahead planner"
- `README.md:128–143` — **Deleted** "### Priority Phase System" section (5-phase enum no longer exists in codebase)
- `README.md:128` — **Added** "### ISR Architecture" section: M14K shadow regs, switch/Harmony macro rationale, GPIO vs Timer/OC split, 8-step ISR sequence, TMR5 callback
- `README.md` — **Added** "### Look-Ahead Planner" section: backward-pass junction velocity, pre-computed per-segment integer deltas

---

## ✅ Look-Ahead Single-Step Retroactive Planner — February 28, 2026

Implements single-step retroactive velocity planning across `data_structures.h`, `kinematics.c`, and `motion.c`.

**Core change**: when segment N arrives, the planner retroactively patches segment N-1's exit profile to the computed junction speed instead of always decelerating to safe-start (8.33 mm/s).

- `incs/data_structures.h` — `MotionSegment`: Added `decel_rate_delta` (separate accel/decel deltas), `entry_speed_mms`, `exit_speed_mms`, `speed_locked`
- `srcs/motion/kinematics.c` — `KINEMATICS_LinearMove()`: Initialise all new fields; compute `decel_rate_delta = (final_rate - nominal_rate) / decel_steps` independently from `rate_delta`
- `srcs/motion/motion.c` — Decel phase: use `seg->decel_rate_delta` instead of `abs(seg->rate_delta)` (correct for asymmetric entry/exit speeds)
- `srcs/motion/motion.c` — Added `MOTION_RecomputeExit()`: patches `final_rate`, `decelerate_after`, `decel_rate_delta` from a new exit speed
- `srcs/motion/motion.c` — `MOTION_ProcessGcodeEvent()` G1 handler: hoisted `junction_speed`; after queuing N, calls `MOTION_RecomputeExit(N-1, junction_speed)` — retroactive patch guarded by `motionQueueTail` and `speed_locked`
- `srcs/motion/motion.c` — `MOTION_Arc()`: sets `segment->speed_locked = true` on every arc segment
- `srcs/motion/motion.c` — `GCODE_EVENT_ARC_MOVE` handler: computes `arc_cruise = min(feedrate, sqrt(accel * mm_per_arc_segment))` and patches preceding G1 exit to `arc_cruise` for smooth G1→arc entry

**Effect**:
- Consecutive same-direction G1 moves glide through without decelerating
- Corner G1 moves decelerate to the correct angle-computed junction speed
- G1→arc: preceding G1 decelerates to arc cruise speed not to near-stop
- Arc segments remain speed-locked, immune to further patching

---

## ✅ Arc Cruise Speed Fix — February 28, 2026

- `srcs/motion/kinematics.c:500` — `KINEMATICS_LinearMoveSimple()` — Replaced hardcoded `8.33 mm/s` entry/exit velocity with computed triangle-peak cruise speed: `arc_cruise = min(feedrate, sqrt(min_accel_xy * chord_mm))`. Passes `arc_cruise` as feedrate AND entry/exit so `nominal=initial=final` — trapezoidal profiler is fully inactive for arc segments. Consecutive arc segments run back-to-back at constant speed with no inter-segment deceleration. Fixes circles running at ~11 mm/min instead of commanded feedrate.

---

## ✅ Velocity Profiling Restored — February 28, 2026

Restores the GRBL-style trapezoidal acceleration that was disabled in commit `07939e8`.
All three fixes applied simultaneously to avoid race conditions.

- `srcs/motion/kinematics.c:258` — `KINEMATICS_LinearMove()` — Fix 1: `min_steps_per_sec` now computed as GRBL safe-start `sqrtf(2*a/steps_per_mm)` clamped to `[1, cruise]`. Replaces incorrect `= steps_per_sec` that forced `rate_delta=0`.
- `srcs/motion/kinematics.c:299` — `KINEMATICS_LinearMove()` — Fix 2: `step_interval = initial_rate` (safe-start speed). Removes `⚠️ TEMPORARY` that hardwired cruise from step 1.
- `srcs/motion/stepper.c:291` — `STEPPER_SetStepRate()` — Fix 3: Removed `TMR4_PeriodSet` / `OCMP1_CompareValueSet` / `OCMP1_CompareSecondaryValueSet` from main loop. ISR already writes these registers every step from `step_interval` — sole hardware writer is now the ISR, eliminating the race condition that caused X/Y noise in prior attempts.

**Tag before changes**: `v1.1h-20260228-pre-accel`

---

## ✅ Full GRBL Feed Hold / Resume (Graceful Drain) — February 28, 2026

Full GRBL v1.1 compliant `!` (feed hold) / `~` (cycle start) with **two-flag graceful drain** —
in-flight segments complete naturally before the hardware is stopped.

### Two-Flag Architecture (defined in `stepper.c`, declared in `stepper.h`)

| Flag | Meaning | GRBL Status | Hardware |
|------|---------|-------------|----------|
| `g_feed_hold_pending` | `!` received, queue draining | `Hold:1` | TMR4/OC1 running |
| `g_feed_hold_active` | queue empty, fully parked | `Hold:0` | TMR4/OC1 stopped |

### Changed files

- `srcs/motion/stepper.c:413` — `STEPPER_PauseMotion()` — if queue non-empty sets `g_feed_hold_pending` only (drain); if queue already 0 goes straight to `g_feed_hold_active` (immediate stop)
- `srcs/motion/stepper.c:447` — `STEPPER_FinalizeHold()` — NEW: clears pending, sets active, stops TMR4/OC1/TMR5; called by `MOTION_Tasks` when `motionQueueCount` hits 0
- `srcs/motion/stepper.c:461` — `STEPPER_ResumeMotion()` — cancels `g_feed_hold_pending` (motion continues) OR clears `g_feed_hold_active` and restarts TMR4/OC1
- `incs/motion/stepper.h:30` — Added `extern volatile bool g_feed_hold_pending` and `void STEPPER_FinalizeHold(void)`
- `srcs/motion/motion.c:300` — `MOTION_Tasks()` — at queue-drain (`motionQueueCount == 0`) calls `STEPPER_FinalizeHold()` if pending, else normal `TMR4_Stop()`
- `srcs/gcode/gcode_parser.c:876` — status `?` returns `Hold:1` when pending, `Hold:0` when active
- `srcs/gcode/gcode_parser.c:915` — `~` calls `STEPPER_ResumeMotion()` when either flag set
- `srcs/gcode/gcode_parser.c:174` — soft reset clears both flags
- `srcs/app.c:326` — `MOTION_Tasks` gated on `!g_feed_hold_active` only (runs during drain)
- `srcs/app.c:344` — arc generation gated on `!g_feed_hold_pending` (inside the active gate)
- `srcs/app.c:434` — event processing gated on `!g_feed_hold_active && !g_feed_hold_pending`

**Soft reset (`Ctrl+X`)**: clears `g_feed_hold_active` without resuming motion (calls `STEPPER_StopMotion` separately)

**Files changed**:
- `srcs/motion/stepper.c:67` — Added `volatile bool g_feed_hold_active = false;`
- `incs/motion/stepper.h:30` — Added `extern volatile bool g_feed_hold_active;`
- `srcs/motion/stepper.c:407` — `STEPPER_PauseMotion()` sets flag before stopping hardware
- `srcs/motion/stepper.c:426` — `STEPPER_ResumeMotion()` clears flag + handles NULL-segment resume
- `srcs/gcode/gcode_parser.c:69` — Removed `static bool feedHoldActive`; replaced all 5 refs with `g_feed_hold_active`
- `srcs/app.c:322` — `if (!g_feed_hold_active)` gates `MOTION_Tasks` + arc generation
- `srcs/app.c:432` — `if(appData.state != APP_ALARM && !g_feed_hold_active)` gates event processing

Build result: ✅ **BUILD COMPLETE** (`bins/CNC_V3.hex`)

---

## 🔧 Linker Fix: `g_estop_pending` moved to stepper.c — February 28, 2026

After `make clean`, the linker reported `undefined reference to 'g_estop_pending'` from
`gcode_parser.c:161`. Root cause: `g_estop_pending` was defined in `app.c` but the linker
(with `--gc-sections` + `-fdata-sections`) failed to resolve it across the `app.o` → `gcode_parser.o`
dependency. Fixed by moving the definition to `stepper.c` to match the established pattern
used by `g_hard_limit_alarm` and `g_suppress_hard_limits` (both defined in `stepper.c`,
declared in `stepper.h`, which is directly included by all consumers).

- `srcs/motion/stepper.c:63` — Added `volatile bool g_estop_pending = false;`
- `incs/motion/stepper.h:26` — Added `extern volatile bool g_estop_pending;`
- `srcs/app.c:59` — Removed definition (replaced with comment referencing stepper.c)
- `incs/app.h:22` — Removed `extern volatile bool g_estop_pending;` (now in stepper.h)

Build result: ✅ **BUILD COMPLETE** (`bins/CNC_V3.hex`)

---

## ✅ E-Stop Hardware Interrupt — February 27, 2026

E-Stop button on **RF4** (Change Notice Port F, IPL7) — highest priority ISR, beats OC1 at IPL5.

**MCC Configuration**:
- RF4 = digital input, internal pullup enabled, CN-F interrupt, IPL7SRS
- SPI2 = IPL1 (lowest — only used at startup/idle, never during motion)
- Interrupt vector: `CHANGE_NOTICE_F_Handler` → `CHANGE_NOTICE_F_InterruptHandler()` → `ESTOP_Callback()`

**Changes**:
- `incs/data_structures.h:191` — `alarmCode` comment updated; added `volatile bool eStopTriggered`
- `srcs/app.c:57` — Added `ESTOP_Callback()` static function: disables steppers, stops TMR4, flushes motion queue, sets `alarmCode=10`, transitions to `APP_ALARM`
- `srcs/app.c` (APP_CONFIG) — Registered `GPIO_PinInterruptCallbackRegister(ESTOP_PIN, ESTOP_Callback, NULL)` + `ESTOP_InterruptEnable()`
- `srcs/app.c` (APP_ALARM) — Reports `ALARM:10\r\n` once on E-Stop; recovery via Ctrl+X soft reset from host

**Recovery sequence**: Button released → host sends `Ctrl+X` → soft reset → `APP_IDLE`



## 🔧 UART3 TX Buffer Restored — February 26, 2026

MCC regeneration (during SPI2/TMC5160 addition) reverted `UART3_WRITE_BUFFER_SIZE` from 1024 back
to 256. This caused UGS to disconnect immediately after sending `$$` — the settings response
(~400-500 bytes) overflowed the 256-byte TX ring buffer.

- `srcs/config/default/peripheral/uart/plib_uart3.c:60` — `UART3_WRITE_BUFFER_SIZE` restored to `1024U`
- `srcs/config/default/peripheral/uart/plib_uart3.c:61` — `UART3_WRITE_BUFFER_SIZE_9BIT` restored to `1024U >> 1`

> ⚠️ After **any** MCC regeneration always verify `UART3_WRITE_BUFFER_SIZE = 1024U` in this file.

---

## 🔧 Build System Flattened — February 26, 2026

Removed `Debug`/`Release` subdirectory split from the build system. Single output path for all
builds — no hardware JTAG debugger is available so the distinction was meaningless.

**Output change**:
- Before: `bins/Release/CNC_V3.hex`, `objs/Release/`, `other/Release/`, `libs/Release/`
- After:  `bins/CNC_V3.hex`, `objs/`, `other/`, `libs/`

**Deleted directories**: `bins/Release`, `bins/Debug`, `objs/Release`, `objs/Debug`,
`other/Release`, `other/Debug`, `libs/Release`, `libs/Debug`

**Files modified**:
- `Makefile` — Removed `BUILD_CONFIG ?= Release`, all `$(BUILD_CONFIG)` references,
  `clean_all` target; `build` target now does `cd bins && xc32-bin2hex`
- `srcs/Makefile` — Removed `BUILD_CONFIG ?= Default`; `BIN_DIR/OBJ_DIR/OUT_DIR/LIB_DIR`
  now flat; replaced `ifeq Debug/Release/error` block with single `OPT_FLAGS := -g -O$(OPT_LEVEL)`;
  `build_dir` no longer creates `bins/Debug` and `bins/Release`; map file is `CS23.map`

**Serial debug output** still controlled by `DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"` — same
compile-time zero-overhead mechanism, just no longer tied to a build config name.

**Build verified**: `bins/CNC_V3.hex` (262637 bytes), only pre-existing bootloader pragma warning.

---

## 🔧 TMC5160 Runtime Settings via $= — February 26, 2026

Added `$200–$253` parameter range for TMC5160 runtime motor tuning. Parameters persist in NVM
flash and take effect immediately (re-configures the driver via SPI without rebooting).
All code is guarded `#ifdef HAS_TMC5160_AXIS` — DRV8825-only builds compile identically to before.

**Parameter Map** (values are applied per-axis: 0=X, 1=Y, 2=Z, 3=A):

| Param  | Axis | Description                                               | Default |
|--------|------|-----------------------------------------------------------|---------|
| $200–$203 | X–A | Chopper mode: 1=StealthChop 2=SpreadCycle 3=Mixed 4=CoolStep | 1 |
| $210–$213 | X–A | Run current 0–31 (linear % of Vref)                      | 20      |
| $220–$223 | X–A | Hold current 0–31                                         | 10      |
| $230–$233 | X–A | Microstep resolution (0=256 … 8=full-step)                | 4 (16µ) |
| $240–$243 | X–A | TPWMTHRS — StealthChop→SpreadCycle crossover (Mixed mode) | 500     |
| $250–$253 | X–A | TCOOLTHRS — CoolStep lower velocity threshold             | 0       |

**Files changed:**

- `incs/settings/settings.h:7` — Added `#include "common.h"` so `HAS_TMC5160_AXIS` is visible
- `incs/settings/settings.h:58–65` — Added `#ifdef HAS_TMC5160_AXIS` block with 6 arrays inside `CNC_Settings`
- `incs/settings/settings.h:SETTINGS_VERSION` — Bumped 2 → 3 (struct changed; old flash data is invalid and triggers defaults)
- `srcs/settings/settings.c:default_settings` — Added `#ifdef HAS_TMC5160_AXIS` block with sensible defaults matching `g_default_cfg` in `tmc5160.c`
- `srcs/settings/settings.c:SETTINGS_SetValue()` — Added `case 200–253` inside `#ifdef HAS_TMC5160_AXIS`
- `srcs/settings/settings.c:SETTINGS_GetValue()` — Added `case 200–253` inside `#ifdef HAS_TMC5160_AXIS`
- `srcs/settings/settings.c:SETTINGS_PrintAll()` — Added 24 `sprintf` lines for `$200–$253` inside `#ifdef HAS_TMC5160_AXIS`
- `incs/motion/tmc5160.h:24` — Added `#include "settings/settings.h"` for `CNC_Settings` type
- `incs/motion/tmc5160.h:TMC5160_ApplySettings()` — New public API function declaration
- `srcs/motion/tmc5160.c:TMC5160_ApplySettings()` — Implementation: validates axis is TMC5160, builds `TMC5160_AxisConfig` from settings arrays, calls `TMC5160_ConfigAxis()`
- `srcs/gcode/gcode_parser.c:includes` — Added `#ifdef HAS_TMC5160_AXIS` guard around `#include "motion/tmc5160.h"`
- `srcs/gcode/gcode_parser.c:$n=v handler` — Added hook: when param in [200,253], calls `TMC5160_ApplySettings((E_AXIS)(param % 10), s)` immediately after flash save

**Build**: ✅ Clean Release — `bins/Release/CS23.hex` (316 KB, February 26, 2026)

---

## 🔧 Per-Axis Mixed Driver System — February 26, 2026

### Correction: enable wrappers must delegate to enable_all_set/clear
- `srcs/utils/utils.c:enable_x/y/z/a_set/clear` - Fixed: per-axis DRV8825 enable wrappers now call `enable_all_set()` / `enable_all_clear()` (the user-defined single-pin wrapper) instead of calling `EnXYZA_Set/Clear` directly. This respects the existing `enable_all_*` abstraction layer and keeps all future EN pin changes in one place.
- `incs/common.h:DRIVER_DRV8825 comment` - Updated comment to note shared `EnXYZA` pin (no per-axis EN on this PCB)
- `.github/copilot-instructions.md` - Added **🔴 CRITICAL: PCB Hardware Constraints** section at top documenting the single shared `EnXYZA` (RE6) enable pin, the absence of `EnX/EnY/EnZ/EnA` macros, and the per-axis driver define system — so this is never accidentally broken again

Replaced the global `STEPPER_DRIVER_TMC5160` / `STEPPER_DRIVER_LEGACY` compile switch with a
fully configurable per-axis driver assignment system supporting mixed TMC5160 + DRV8825 on the
same board (e.g. 2+2 or 3+1). All selection is compile-time; zero runtime overhead.

- `incs/common.h:17-60` - **Replaced** global driver `#define` with:
  - `DRIVER_TMC5160 1` / `DRIVER_DRV8825 2` — driver type tokens
  - `AXIS_X_DRIVER` / `AXIS_Y_DRIVER` / `AXIS_Z_DRIVER` / `AXIS_A_DRIVER` — per-axis assignment (edit to match wiring)
  - `HAS_TMC5160_AXIS` — auto-derived; defined if ≥1 axis is TMC5160; gates all SPI code
  - `HAS_DRV8825_AXIS` — auto-derived; defined if ≥1 axis is DRV8825; gates per-axis enable arrays
  - `TMC5160_AXIS_MASK` — compile-time bitmask (bit N = axis N is TMC5160); used by mixed-driver enable inlines
- `incs/motion/tmc5160.h:22,119` - Changed `#ifdef STEPPER_DRIVER_TMC5160` guard → `#ifdef HAS_TMC5160_AXIS`
- `srcs/motion/tmc5160.c:17` - Changed file-level guard → `#ifdef HAS_TMC5160_AXIS`
- `srcs/motion/tmc5160.c:TMC5160_Initialize()` - Replaced 4-axis loop with per-axis `#if (AXIS_x_DRIVER == DRIVER_TMC5160)` blocks; CS deassert and ConfigAxis only called for assigned TMC5160 axes
- `srcs/motion/tmc5160.c:TMC5160_Tasks()` - Replaced 4-axis loop with per-axis `#if` blocks using `POLL_TMC_AXIS()` local macro; DRV8825 axes never polled over SPI
- `srcs/motion/tmc5160.c:268` - Updated closing `#endif` comment
- `incs/utils/utils.h:27-31` - Changed enable extern guard `#ifndef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_DRV8825_AXIS`
- `incs/utils/utils.h:STEPPERS_Enable/Disable` - **Replaced** binary TMC5160/legacy block with mixed-driver implementation: TMC5160 block (`#ifdef HAS_TMC5160_AXIS`) + DRV8825 per-axis `#if` blocks (`#ifdef HAS_DRV8825_AXIS`)
- `incs/utils/utils.h:AXIS_EnableSet/Clear` - **Replaced** with three-way `#if defined(HAS_TMC5160_AXIS) && defined(HAS_DRV8825_AXIS)` / `elif TMC5160` / `else DRV8825`; mixed path uses `TMC5160_AXIS_MASK` bitmask for single-compare runtime dispatch (compiler dead-strips one branch for pure configs)
- `srcs/utils/utils.c:enable wrappers` - Changed guard `#ifndef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_DRV8825_AXIS`
- `srcs/utils/utils.c:axis_enable_set[] arrays` - Changed guard `#ifndef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_DRV8825_AXIS`; full 4-entry arrays always compiled when DRV8825 present (TMC5160 entries unreachable via bitmask)
- `srcs/app.c:APP_CONFIG` - Changed `#ifdef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_TMC5160_AXIS`
- `srcs/app.c:APP_IDLE` - Changed `#ifdef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_TMC5160_AXIS`

**Example configurations** (edit `AXIS_x_DRIVER` lines in `incs/common.h`):
```
X+Y=TMC5160, Z+A=DRV8825:  AXIS_X=TMC5160 AXIS_Y=TMC5160 AXIS_Z=DRV8825 AXIS_A=DRV8825
X+Y+Z=TMC5160, A=DRV8825:  AXIS_X=TMC5160 AXIS_Y=TMC5160 AXIS_Z=TMC5160 AXIS_A=DRV8825
All TMC5160:                all four = DRIVER_TMC5160
All DRV8825:                all four = DRIVER_DRV8825  (previous STEPPER_DRIVER_LEGACY behaviour)
```

---

## 🔧 Infrastructure Changes - February 23, 2026

### TMC5160 SPI Driver - Infrastructure Setup

- **MCC**: SPI2 peripheral added and configured (SPI Master mode)
- `srcs/config/config/default/peripheral/spi/spi_master/plib_spi2_master.c` - MCC generated SPI2 PLIB (source)
- `incs/config/config/default/peripheral/spi/spi_master/plib_spi2_master.h` - MCC generated SPI2 PLIB (header, moved to incs)
- `incs/config/config/default/peripheral/spi/spi_master/plib_spi_master_common.h` - MCC common SPI types (header, moved to incs)

### Build System Fixes

- `Makefile:124` - `build_dir` target now forwards `DRY_RUN=$(DRY_RUN)` to inner make
- `srcs/Makefile:380` - `SRC_DIRS` wildcard extended from 4 to 6 levels deep (covers `spi/spi_master/` nesting)

---

## ✅ LitePlacer GRBL Probe Implementation — February 10, 2026

G38.2/3/4/5 probe commands. Planning: [LITEPLACER_GRBL_IMPLEMENTATION.md](docs/readme/LITEPLACER_GRBL_IMPLEMENTATION.md)

---

## 🔧 Code Changes - February 10, 2026

### Phase 1: Probe Event Types (Initial Implementation)

#### File: `incs/gcode/gcode_parser.h`

**Line 35** - `GCODE_EventType enum`  
- **Added**: `GCODE_EVENT_PROBE_TOWARD` - Event type for G38.2, G38.3 probe toward commands
- **Added**: `GCODE_EVENT_PROBE_AWAY` - Event type for G38.4, G38.5 probe away commands

**Line 100** - `GCODE_Event data union`  
- **Added**: `probe` struct member with fields:
  - `float x, y, z, a` - Target coordinates for probe move
  - `float feedrate` - Probe feedrate
  - `bool alarm_on_fail` - true for G38.2/G38.4, false for G38.3/G38.5
  - `bool probe_toward` - true = toward (G38.2/G38.3), false = away (G38.4/G38.5)

### Phase 2: Parser Implementation ✅ COMPLETE

#### File: `srcs/gcode/gcode_parser.c`

**Line 418** - `parse_command_to_event()` function  
- **Added**: G38.x probe command parsing block (57 lines)
- Parses G38.2, G38.3, G38.4, G38.5 with decimal subcode detection
- Determines probe direction: toward (G38.2/G38.3) or away (G38.4/G38.5)
- Sets alarm behavior: true for G38.2/G38.4, false for G38.3/G38.5
- Parses XYZAF parameters with unit scaling (mm/inches)
- Returns `GCODE_EVENT_PROBE_TOWARD` or `GCODE_EVENT_PROBE_AWAY`
- Debug logging for probe parameters

---

## 📝 Pending Implementation

### Phase 3: Probe State Machine (80% COMPLETE ✅)
- [x] `incs/data_structures.h:145` - Add `ProbeState` enum (IDLE, MOVING, TRIGGERED, FAILED)
- [x] `incs/data_structures.h:210` - Add probe state fields to APP_DATA (probeState, probeSuccess, probeAlarmOnFail, probePosition)
- [x] `srcs/motion/motion.c:812` - `MOTION_ProcessGcodeEvent()` - Add GCODE_EVENT_PROBE_TOWARD/AWAY handling (68 lines)
  - Builds start/end coordinates from probe event data
  - Calls `KINEMATICS_LinearMove()` with probe_speed (8.33 mm/s)
  - Initializes `appData->probeState = PROBE_STATE_MOVING`
- [x] `srcs/app.c:278` - `APP_Tasks()` - Add probe trigger monitoring (70 lines)
  - Checks `LIMIT_GetMax(AXIS_Z)` during PROBE_STATE_MOVING
  - Applies $6 probe invert setting
  - On trigger: stops motion, saves position to probePosition, sets PROBE_STATE_TRIGGERED
  - On completion without trigger: sets PROBE_STATE_FAILED
- [x] `srcs/app.c:290` - Add probe result reporting for PROBE_STATE_TRIGGERED
  - Sends `[PRB:x,y,z,a:1]` and "ok"
- [x] `srcs/app.c:302` - Add probe failure handling for PROBE_STATE_FAILED
  - If probeAlarmOnFail: triggers `ALARM:5`, enters APP_ALARM state
  - Else: sends `[PRB:x,y,z,a:0]` and "ok"

### Phase 4: Hardware Configuration (100% COMPLETE ✅)
- [x] `incs/utils/utils.h:155` - Add `PROBE_Get()` inline function
  - Uses Z-max limit switch as probe input (GRBL standard)
  - Applies $6 probe invert setting (NO/NC switch configuration)
  - Returns true when probe contact made
- [x] `srcs/app.c:283` - Updated probe trigger detection to use `PROBE_Get()`
  - Replaced manual `LIMIT_GetMax(AXIS_Z)` + invert logic
  - Now uses hardware abstraction layer for cleaner code

### Phase 5: Testing (PENDING)
- [ ] Test G38.2 probe with trigger detection
- [ ] Test G38.3 probe without alarm on failure
- [ ] Verify `[PRB:...]` response format
- [ ] Test position preservation at trigger point

---

## 🏷️ Git History

### Commits

**Tag**: `v1.1h-20260210-pre-probe` - Stable build before LitePlacer probe implementation

**Commit**: `c37eca6` - "Add LitePlacer GRBL probe implementation plan and initial event types"
- Added comprehensive implementation document
- Added GCODE_EVENT_PROBE_TOWARD and GCODE_EVENT_PROBE_AWAY event types
- Added probe data structure to GCODE_Event union

---

## 📊 Implementation Progress

| Phase | Status | Description |
|-------|--------|-------------|
| Phase 1: Event Types | ✅ Complete | `GCODE_EVENT_PROBE_TOWARD/AWAY` added to parser.h |
| Phase 2: Parser | ✅ Complete | G38.2/3/4/5 parsing in gcode_parser.c |
| Phase 3: State Machine | ✅ Complete | `MOTION_ProcessGcodeEvent`, probe monitoring in APP_Tasks |
| Phase 4: Hardware Config | ✅ Complete | `PROBE_Get()` inline, Z-max as probe input, $6 invert |
| Phase 5: Testing | ⬜ Pending | Flash to dev rig and verify [PRB:...] response |

**Total Progress**: 80% (4/5 phases complete — Phase 5 hardware test pending)

---

## 📚 Reference Documents

- [LITEPLACER_GRBL_IMPLEMENTATION.md](docs/readme/LITEPLACER_GRBL_IMPLEMENTATION.md) - Complete implementation plan
- [SETTINGS_REFERENCE.md](docs/readme/SETTINGS_REFERENCE.md) - GRBL settings reference
- [ARCHITECTURE.md](docs/readme/ARCHITECTURE.md) - System architecture overview

---

**Note**: This file tracks all code changes. Planning documents are separate in `docs/readme/`. No other change tracking files should be created.
