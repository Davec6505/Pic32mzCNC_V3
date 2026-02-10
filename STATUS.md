# Pic32mzCNC_V3 - Development Status Tracker

**Branch**: `liteplacer`  
**Feature**: LitePlacer G38.x Probe Implementation  
**Started**: February 10, 2026  
**Last Updated**: February 10, 2026

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

---

## 📝 Pending Implementation

### Phase 2: Parser Implementation
- [ ] `srcs/gcode/gcode_parser.c` - `parse_command_to_event()` - Add G38.2/G38.3/G38.4/G38.5 parsing
- [ ] Parse probe parameters (XYZAF)
- [ ] Set alarm_on_fail and probe_toward flags based on subcode

### Phase 3: Probe State Machine
- [ ] `incs/data_structures.h` - Add probe state fields to APP_DATA
- [ ] `srcs/app.c` - `APP_Tasks()` - Add probe event handling
- [ ] `srcs/app.c` - Add probe trigger monitoring
- [ ] `srcs/app.c` - Add probe result reporting `[PRB:x,y,z,a:1]`
- [ ] `srcs/app.c` - Add probe failure handling (ALARM:5)

### Phase 4: Hardware Configuration
- [ ] `srcs/utils/utils.c` - Configure Z-max limit as probe input
- [ ] `incs/utils/utils.h` - Add `PROBE_Get()` inline function
- [ ] Apply $6 probe invert setting

### Phase 5: Testing
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
| Phase 2: Parser | ⬜ Pending | - | - |
| Phase 3: State Machine | ⬜ Pending | - | - |
| Phase 4: Hardware Config | ⬜ Pending | - | - |
| Phase 5: Testing | ⬜ Pending | - | - |

**Total Progress**: 20% (1/5 phases complete)

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
