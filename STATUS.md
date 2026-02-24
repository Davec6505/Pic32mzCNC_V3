# Pic32mzCNC_V3 - Development Status Tracker

**Branch**: `liteplacer`  
**Feature**: LitePlacer G38.x Probe + TMC5160 SPI Driver  
**Started**: February 10, 2026  
**Last Updated**: February 23, 2026

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

### Next Steps for TMC5160

- **TODO**: Confirm CS GPIO pin assignments from PCB/MCC pin manager
- **TODO**: Create `srcs/motion/tmc5160.c` and `incs/motion/tmc5160.h`
- **TODO**: Add CS wrappers to `srcs/utils/utils.c` following LED pattern
- **TODO**: Call `TMC5160_Initialize()` from `APP_CONFIG` in `srcs/app.c`
- **TODO**: Add rate-limited `TMC5160_CheckStatus()` call in `APP_IDLE` in `srcs/app.c`

**Planning Document**: See `TMC5160 SPI Driver` section in `.github/copilot-instructions.md`

---

## 📋 Current Development Session

### LitePlacer GRBL Probe Implementation

**Objective**: Implement GRBL v1.1 standard G38.x probe commands for LitePlacer pick-and-place machine integration

**Planning Document**: [docs/readme/LITEPLACER_GRBL_IMPLEMENTATION.md](docs/readme/LITEPLACER_GRBL_IMPLEMENTATION.md)

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

| Phase | Status | Files Modified | Lines Changed |
|-------|--------|----------------|---------------|
| Phase 1: Event Types | ✅ Complete | 1 | +10 |
| Phase 2: Parser | ✅ Complete | 1 | +57 |
| Phase 3: State Machine | 🔄 In Progress | - | - |
| Phase 4: Hardware Config | ⬜ Pending | - | - |
| Phase 5: Testing | ⬜ Pending | - | - |

**Total Progress**: 40% (2/5 phases complete)

---

## 🎯 Next Action

**Continue with Phase 2**: Implement G38.x parsing in `srcs/gcode/gcode_parser.c`

**Expected Changes**:
- Parse G38.2, G38.3, G38.4, G38.5 commands
- Extract XYZAF parameters
- Create probe event with appropriate flags
- Return event to main loop for processing

---

## 📚 Reference Documents

- [LITEPLACER_GRBL_IMPLEMENTATION.md](docs/readme/LITEPLACER_GRBL_IMPLEMENTATION.md) - Complete implementation plan
- [SETTINGS_REFERENCE.md](docs/readme/SETTINGS_REFERENCE.md) - GRBL settings reference
- [ARCHITECTURE.md](docs/readme/ARCHITECTURE.md) - System architecture overview

---

**Note**: This file tracks all code changes. Planning documents are separate in `docs/readme/`. No other change tracking files should be created.
