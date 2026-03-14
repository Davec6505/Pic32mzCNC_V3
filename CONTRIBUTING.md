# Contributing to Pic32mzCNC_V3

Thank you for your interest in contributing! This project is a hardware-validated CNC motion
controller running on a PIC32MZ2048EFH100. Because changes can affect real stepper motors and
CNC machines, the bar for correctness is high. Please read this guide before opening a PR.

---

## Table of Contents

- [Contributing to Pic32mzCNC\_V3](#contributing-to-pic32mzcnc_v3)
  - [Table of Contents](#table-of-contents)
  - [Branch Convention](#branch-convention)
  - [Build Requirements](#build-requirements)
  - [Code Style](#code-style)
    - [GPIO access](#gpio-access)
    - [Coordinate access](#coordinate-access)
  - [Hardware Testing](#hardware-testing)
    - [Minimum test suite (run all with UGS in pipelined streaming mode)](#minimum-test-suite-run-all-with-ugs-in-pipelined-streaming-mode)
    - [ISR-critical files](#isr-critical-files)
  - [Submitting a Pull Request](#submitting-a-pull-request)
  - [Reporting Bugs](#reporting-bugs)
  - [Requesting Features](#requesting-features)
  - [Safe Files to Modify](#safe-files-to-modify)
  - [Files That Require Extra Care](#files-that-require-extra-care)
  - [Questions?](#questions)

---

## Branch Convention

- **`master`** — stable, hardware-validated releases only. Do **not** target PRs here directly.
- **`scurve_motion`** — active development mainline. All PRs must target this branch.

```
git checkout scurve_motion
git pull origin scurve_motion
git checkout -b feature/my-feature
# ... make changes ...
git push origin feature/my-feature
# Open PR → base: scurve_motion
```

---

## Build Requirements

| Tool | Version | Notes |
|------|---------|-------|
| XC32 compiler | v4.x | Must be on PATH; `xc32-gcc --version` to verify |
| GNU Make | any | Run from **repo root only** |
| MPLAB Harmony v3 | 3.x | Only needed if regenerating MCC peripheral drivers |

**Always run `make` from the repository root:**

```powershell
# CORRECT
make
make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"
make clean && make

# WRONG — breaks relative paths
cd srcs && make
```

Output files:
- `bins/Release/CNC_V3.hex` — production firmware
- `bins/Debug/CNC_V3.hex` — debug firmware (larger, slower)

---

## Code Style

Match the style of the file you are editing. In general:

- **Language standard**: C99
- **Naming**: `snake_case` for functions and variables, `UPPER_CASE` for macros and enum values,
  `PascalCase` only for type names that mirror Harmony PLIB conventions
- **Braces**: Allman style (opening brace on new line)
- **Indentation**: 4 spaces — no tabs
- **Line endings**: LF (`\n`), not CRLF
- **Include guards**: `#ifndef FILENAME_H` / `#define FILENAME_H` / `#endif`
- **Debug output**: Always use the compile-time `DEBUG_PRINT_xxx` macros from `incs/common.h`.
  Never add raw `printf` / `UART3_Write` calls.

```c
// Good
DEBUG_PRINT_MOTION("[MOTION] Steps: %lu\r\n", (unsigned long)steps);

// Bad — leaks debug output into release builds
char buf[64];
snprintf(buf, sizeof(buf), "Steps: %lu\r\n", steps);
UART3_Write((uint8_t*)buf, strlen(buf));
```

### GPIO access

Use the inline helpers in `incs/utils/utils.h` — never write directly to LAT/PORT registers:

```c
AXIS_StepSet(AXIS_X);        // ✅ correct
asm("LATDSET = 0x10");       // ❌ wrong
```

### Coordinate access

Use the coordinate array utilities:

```c
ADD_COORDINATE_AXIS(&target, axis, delta);   // ✅
target.x = delta_x;                          // ❌ (breaks multi-axis loops)
```

---

## Hardware Testing

**Any PR that touches motion code must be tested on real hardware.** Simulation is not sufficient.

### Minimum test suite (run all with UGS in pipelined streaming mode)

| File | What it validates |
|------|------------------|
| `tests/02_rectangle_path.gcode` | Linear moves, corner accuracy, return to origin |
| `tests/03_circle_20segments.gcode` | 20-segment circle, <0.025 mm error |
| `tests/05_three_arcs_simple.gcode` | CW/CCW arc handoff |
| `tests/08_arc_cw_ccw_stress.gcode` | 36 arcs continuous (~2:18 min) |
| `tests/09_concentric_semicircles.gcode` | Pipeline continuity, X error <0.031 mm |
| `tests/07_complex_long_run_fast.gcode` | Mixed arcs, linears, G4 dwell |

All six tests must **PASS** (machine returns to within 0.05 mm of origin) before a PR is mergeable.

### ISR-critical files

If your change touches `srcs/motion/interpolator.c`, `srcs/motion/trajectory.c`, or the
`TIMER_4_InterruptHandler`, run the stress test (`08_arc_cw_ccw_stress.gcode`) **twice** in a row
without resetting the machine. A single pass is not conclusive for timing regressions.

---

## Submitting a Pull Request

1. Rebase on the latest `scurve_motion` before opening the PR.
2. Fill in the PR template checklist completely — empty checkboxes will delay review.
3. If `CNC_Settings` struct changed, bump `SETTINGS_VERSION` and document the new field in
   `docs/readme/SETTINGS_REFERENCE.md`.
4. If the ISR architecture, call hierarchy, or step pulse timing changed, update the relevant
   section of `README.md` and `docs/CNC_FIRMWARE_BOOK.md`.
5. Always update `STATUS.md` with the file, line, function, and description of every change.

---

## Reporting Bugs

Use the **Bug Report** issue template. The most useful information is:

- Exact firmware commit hash (`$I` response)
- G-code sender name and version
- The `.gcode` file or minimal reproducer
- Output of `$$` (all settings)
- Serial log showing the failure (copy from your terminal/UGS console)

---

## Requesting Features

Use the **Feature Request** issue template. Describe the use case first — what CNC operation
requires this, and why the current behaviour is insufficient.

---

## Safe Files to Modify

These files have well-defined interfaces and are lower-risk for changes:

| File | What it controls |
|------|-----------------|
| `srcs/settings/settings.c` | Persistent GRBL settings (NVM) |
| `srcs/motion/homing.c` | 4-phase homing cycle |
| `srcs/motion/spindle.c` | Spindle PWM |
| `srcs/utils/uart_utils.c` | UART helpers |

---

## Files That Require Extra Care

| File | Reason |
|------|--------|
| `srcs/gcode/gcode_parser.c` | Production-ready; any change risks GRBL protocol regression |
| `srcs/motion/interpolator.c` | 100 kHz ISR; timing errors cause missed steps or crashes |
| `srcs/motion/trajectory.c` | S-curve planner; velocity math is subtle |
| `srcs/motion/motion_bridge.c` | Glue between G-code events and trajectory — subtle state machine |
| `srcs/config/default/peripheral/` | Harmony PLIB auto-generated; touch only via MCC |
| `srcs/config/default/peripheral/uart/plib_uart3.c` | Buffer sizes must stay 512 RX / 1024 TX |

---

## Questions?

Open a **Discussion** (GitHub Discussions tab) for questions that are not bug reports or feature
requests. Hardware-specific questions about the PCB, motor drivers, or stepper tuning are welcome.
