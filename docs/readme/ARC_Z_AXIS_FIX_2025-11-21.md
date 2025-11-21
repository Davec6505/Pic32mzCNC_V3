# Arc Z-Axis Coordinate Fix (November 21, 2025)

## Problem Description

**Symptom**: Z-axis commands (e.g., `G1 Z0 F300`) would not execute after arc moves (G2/G3) in G-code files.

**User Report**: 
- Z-axis worked perfectly up to the first arc in the toolpath
- After any arc move (G2/G3), subsequent Z commands had no effect
- Motion in X/Y continued working normally

## Root Cause Analysis

### Debug Investigation

1. **Initial Observation**: Debug output showed:
   ```
   [MOTION] Z event: value=nan, current=-50.000, mode=ABS
   [KIN] Z: start=10.477 end=nan delta=nan
   ```

2. **Parser Validation**: Modified `parse_float_after()` to properly return NAN for failed conversions:
   ```c
   // Before: returned 0.0f on parse failure
   // After: returns NAN to distinguish from actual zero values
   if (!start || !start[1]) return NAN;
   char* endptr = NULL;
   float value = (float)strtof(start + 1, &endptr);
   if (endptr == start + 1) return NAN;  // Parse failed
   return value;
   ```

3. **Arc Coordinate Handling**: Discovered that arc moves (G2/G3) were setting Z coordinate to NAN when Z parameter not specified in the command.

### The Bug

**Location**: `srcs/motion/motion.c` lines 623-638 (arc move event processing)

**Problem**: When processing arc moves, the code set coordinates directly from event values without checking for NAN:
```c
// OLD CODE - Bug
if (appData->modal_distance_mode == DISTANCE_MODE_ABSOLUTE) {
    SET_COORDINATE_AXIS(&end, axis, event_value);  // Sets Z to NAN!
}
```

**Propagation**:
1. Arc command like `G2 X10 Y10 I5 J0` (no Z specified) → Z event_value = NAN
2. Arc start/end points built with Z = NAN
3. Arc generator stores NAN in `appData->current[AXIS_Z]`
4. Subsequent Z commands inherit NAN as starting position
5. Motion calculation: `delta = target - current = 0.0 - NAN = NAN`
6. No steps generated because `isnan(delta)` → no movement

## Solution Implemented

**Fix**: Added `isnan()` validation in arc coordinate building:

```c
// File: srcs/motion/motion.c, lines 623-638
for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
    float event_value = GET_COORDINATE_AXIS(&event->data.arcMove.end, axis);
    
    if (appData->modal_distance_mode == DISTANCE_MODE_ABSOLUTE) {
        // Use event value if valid, otherwise preserve current position
        SET_COORDINATE_AXIS(&end, axis, 
            isnan(event_value) ? appData->current[axis] : event_value);
    } else { // DISTANCE_MODE_INCREMENTAL
        // Add delta if valid, otherwise no change (delta=0)
        SET_COORDINATE_AXIS(&end, axis, 
            appData->current[axis] + (isnan(event_value) ? 0.0f : event_value));
    }
}
```

**Logic**:
- **Absolute Mode** (G90): If arc doesn't specify Z, use current Z position (no change)
- **Incremental Mode** (G91): If arc doesn't specify Z, treat as zero delta (no change)
- Prevents NAN contamination of position tracking

## Validation

**Test Case**: G-code file with arcs followed by Z moves
```gcode
G21 G90         ; Metric, absolute
G1 Z-50 F300    ; Lower Z (works)
G2 X10 Y10 I5 J0 F500  ; Arc move (no Z specified)
G1 Z0 F300      ; Raise Z (NOW WORKS!)
```

**Results**:
- ✅ Z-axis moves correctly before arcs
- ✅ Z-axis maintains position during arcs (no Z specified)
- ✅ Z-axis moves correctly after arcs
- ✅ Release build tested and validated

## Files Modified

1. **srcs/gcode/gcode_parser.c** (lines 95-101)
   - Enhanced `parse_float_after()` to return NAN on parse failure
   - Validates `strtof()` conversion with `endptr` check

2. **srcs/motion/motion.c** (lines 623-638)
   - Added `isnan()` checks in arc move coordinate building
   - Preserves current position when coordinate not specified in arc command

## Impact

- **Bug Severity**: Critical - prevented Z-axis motion after any arc
- **Affected Use Cases**: Pen plotters, 3D paths with arcs and Z moves
- **Fix Complexity**: Simple validation check
- **Regression Risk**: Low - only affects arc coordinate handling

## Lessons Learned

1. **NAN Propagation**: Once NAN enters position tracking, it corrupts all subsequent moves
2. **Parser Validation**: Distinguish between "not specified" (NAN) and "zero" (0.0)
3. **Arc Behavior**: GRBL standard - unspecified axes maintain current position during arcs
4. **Debug Output**: Excessive debug can overflow UART buffer and cause motion issues

## Deployment

- **Branch**: patch1
- **Build**: Release (no debug overhead)
- **Testing**: User validated complete toolpath execution
- **Status**: Ready for merge to master

---

**Fixed By**: GitHub Copilot  
**Validated By**: Davec6505  
**Date**: November 21, 2025
