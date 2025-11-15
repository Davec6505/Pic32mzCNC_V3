# Arc Motion Debug Session - November 7, 2025

## Problem Summary
Arc interpolation (G2/G3 commands) was stopping prematurely at approximately 40-60% completion. Additionally, helical arcs showed Z-axis oscillation instead of smooth ramping.

## Root Causes Identified

### Issue 1: Z-Axis Interpolation Bug ✅ FIXED
**Location:** `srcs/motion/motion.c` MOTION_Arc() function (lines 314-331)

**Problem:**
The Z/A interpolation code was recalculating `start_angle` from `arcCurrent` on EVERY segment generation:
```c
float start_angle = atan2f(appData->arcCurrent.y - appData->arcCenter.y,
                           appData->arcCurrent.x - appData->arcCenter.x);
```

Since `arcCurrent` was updated after each segment, the "start" angle kept changing, making progress calculations completely wrong.

**Fix:**
Added persistent start tracking:
- `arcThetaStart` - Stores initial angle (never changes)
- `arcStartPoint` - Stores initial X/Y/Z/A position (never changes)
- Progress now calculated from fixed start point to end point

**Result:** TEST 7 (Helical Arc) Z motion now smooth: 0.375 → 1.3 → 2.35 → 3.25 → 3.925mm ✅

### Issue 2: Angle Wrap-Around in Termination Logic ✅ FIXED
**Location:** `srcs/motion/motion.c` MOTION_Arc() function (lines 293-299)

**Problem:**
Arc termination was comparing angles directly:
```c
if(appData->arcThetaIncrement < 0.0f) {
    is_last_segment = (next_theta <= appData->arcThetaEnd);
}
```

Angles wrap around at ±π, causing premature termination when crossing the wrap boundary.

**Fix:**
Replaced angle comparison with segment counter:
- Added `arcSegmentCurrent` (0-based counter)
- Added `arcSegmentTotal` (calculated from arc length / mm_per_segment)
- Termination: `is_last_segment = (arcSegmentCurrent >= arcSegmentTotal - 1)`

**Result:** More reliable arc completion, TEST 3 completed fully with 13 segments ✅

## Test Results (November 7, 2025 - Final Run)

### ✅ Working Tests:
- **TEST 1:** Setup and zeroing - ✅ Perfect
- **TEST 3:** 90° CCW Arc - ✅ **COMPLETED FULLY** (13 position updates)
- **TEST 4:** Full Circle CW - ✅ **COMPLETED** (9 position updates, minor endpoint error)
- **TEST 7:** Helical Arc with Z - ✅ **Z interpolation smooth**

### ⚠️ Partially Working Tests:
- **TEST 2:** 90° CW Arc - ⚠️ Stopped at (6.125, 7.725) instead of (10, 10)
- **TEST 5:** 180° Semi-Circle CW - ⚠️ Stopped at (12.25, -0.075) instead of (20, 0)
- **TEST 6:** 180° Semi-Circle CCW - ⚠️ Stopped at (0.075, -0.1) instead of (0, 0)
- **TEST 8:** Small Arc - ⚠️ Stopped at (7.475, 9.175) instead of (12, 12)

### ❌ Not Working:
- **TEST 9:** Fast Arc (F3000) - ❌ No motion detected

## Files Modified

### 1. `incs/data_structures.h`
**Added fields to APP_DATA:**
```c
float arcThetaStart;               // Initial start angle (radians, fixed)
uint32_t arcSegmentCurrent;        // Current segment number (0-based)
uint32_t arcSegmentTotal;          // Total number of segments
CoordinatePoint arcStartPoint;     // Initial arc start point (for Z/A interpolation)
```

### 2. `srcs/motion/motion.c`

**MOTION_ProcessGcodeEvent() - Arc initialization (lines 503-530):**
```c
// Initialize arc state
appData->arcGenState = ARC_GEN_ACTIVE;
appData->arcTheta = start_angle;
appData->arcThetaStart = start_angle;  // Store initial angle
appData->arcSegmentCurrent = 0;
appData->arcSegmentTotal = num_segments;
appData->arcCurrent = start;
appData->arcStartPoint = start;  // Store initial position
```

**MOTION_Arc() - Segment generation (lines 275-328):**
```c
// Segment counter-based termination
is_last_segment = (appData->arcSegmentCurrent >= (appData->arcSegmentTotal - 1));

if (is_last_segment) {
    next = appData->arcEndPoint;  // Exact endpoint
    appData->arcGenState = ARC_GEN_IDLE;
} else {
    // Calculate XY from current theta
    next.x = appData->arcCenter.x + appData->arcRadius * cosf(appData->arcTheta);
    next.y = appData->arcCenter.y + appData->arcRadius * sinf(appData->arcTheta);
    
    // Z/A interpolation using segment-based progress
    float progress = (float)appData->arcSegmentCurrent / (float)(appData->arcSegmentTotal - 1);
    next.z = appData->arcStartPoint.z + (appData->arcEndPoint.z - appData->arcStartPoint.z) * progress;
    next.a = appData->arcStartPoint.a + (appData->arcEndPoint.a - appData->arcStartPoint.a) * progress;
    
    appData->arcTheta += appData->arcThetaIncrement;
}

// Increment segment counter after each generation
appData->arcSegmentCurrent++;
```

## Remaining Issues

### 1. Some Arcs Still Terminate Early
**Tests Affected:** TEST 2, 5, 6, 8

**Observations:**
- Segment counter approach improved reliability significantly
- Some arcs complete (TEST 3, 4), others still stop early
- Pattern unclear - not strictly related to arc size or direction

**Hypotheses:**
1. **Motion queue starvation?** Arc generation may be blocked when queue fills
2. **Segment calculation error?** num_segments may be too low for some arcs
3. **Feed rate issues?** Some arcs may execute too slowly causing timeout
4. **Position tracking mismatch?** Work coordinates may not update correctly

**Debug Steps for Tomorrow:**
```bash
# Build with debug output to see segment generation
make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION"

# Monitor segment counter and queue status during arc execution
# Expected output: "seg=0/157", "seg=1/157", ... "seg=156/157"
```

### 2. Fast Arc No Motion (TEST 9)
**Command:** `G3 X0 Y0 Z0 I-12 J-12 F3000`

**Observation:** No position updates detected

**Hypotheses:**
1. Arc too long for current segment size?
2. Feed rate calculation issue at F3000?
3. Large I/J values causing radius error?

**Debug:** Check if arc initialization succeeds (no ALARM:33)

## Next Steps (Priority Order)

### High Priority:
1. **Enable DEBUG_MOTION build** - See actual segment counts vs expected
2. **Verify segment calculation** - Check `num_segments` for various arc lengths
3. **Monitor motion queue** - Ensure arc generator not starved by full queue
4. **Test feed rate scaling** - Try same arcs at different speeds (F100, F500, F1000, F3000)

### Medium Priority:
5. **Add telemetry** - Log segment_current/segment_total during arc execution
6. **Verify position updates** - Ensure work coordinates update when arcGenState = IDLE
7. **Test edge cases** - Very small arcs, very large arcs, full circles at different radii

### Low Priority:
8. **Code cleanup** - Remove unused angle comparison code
9. **Documentation** - Update copilot-instructions.md with arc fixes
10. **Performance tuning** - Optimize mm_per_arc_segment setting ($12)

## Key Learnings

### What Worked:
✅ **Segment counter approach** - More reliable than angle comparison
✅ **Fixed start point tracking** - Solved Z interpolation oscillation
✅ **Non-blocking arc generation** - One segment per main loop iteration
✅ **Release build testing** - No debug UART interference

### What Didn't Work:
❌ **Angle wrap-around handling** - Too complex, segment counter simpler
❌ **Recalculating start angle** - Breaks progress calculation
❌ **Large motion queue (64 segments)** - Causes firmware crash (stack overflow)

### Architecture Insights:
- **16-segment motion queue sufficient** - Incremental arc generation works
- **Segment-based termination robust** - No wrap-around edge cases
- **Progress calculation must use fixed reference** - Moving reference breaks interpolation
- **Compile-time debug system valuable** - Zero runtime overhead in release builds

## Build Configuration

### Release Build (Current):
```bash
make BUILD_CONFIG=Release
# Output: bins/Release/CS23.hex
# No debug output, clean UART for G-code
```

### Debug Build (For Tomorrow):
```bash
make clean && make BUILD_CONFIG=Debug DEBUG_FLAGS="DEBUG_MOTION"
# Output: bins/Debug/CS23.hex
# UART debug output enabled for motion subsystem
```

### Settings:
- `$12` (mm_per_arc_segment): 0.1mm (default)
- Motion queue: 16 segments (MAX_MOTION_SEGMENTS)
- Stack: 128KB (doubled from 64KB to prevent overflow)
- Heap: 64KB (unchanged)

## Test Commands for Tomorrow

### Basic Arc Verification:
```gcode
G17 G90 G21          ; XY plane, absolute, metric
G92 X0 Y0 Z0         ; Zero all axes
G1 F100              ; Slow feedrate
G2 X10 Y0 I5 J0      ; 180° semicircle, r=5mm
?                    ; Check final position (should be 10,0)
```

### Segment Count Test:
```gcode
G92 X0 Y0            ; Zero
G2 X10 Y10 I10 J0    ; 90° arc, r=10mm
                     ; Expected segments: π/2 * 10 / 0.1 ≈ 157 segments
```

### Feed Rate Scaling:
```gcode
G92 X0 Y0
G2 X10 Y10 I10 J0 F100    ; Slow
G92 X0 Y0
G2 X10 Y10 I10 J0 F1000   ; Medium
G92 X0 Y0
G2 X10 Y10 I10 J0 F3000   ; Fast
```

## References
- GRBL v1.1 Documentation: https://github.com/gnea/grbl/wiki
- Arc Interpolation Algorithm: Bresenham-based incremental
- Settings: `docs/GRBL_SETTINGS.md`
- Debug System: `docs/DEBUG_SYSTEM_TUTORIAL.md`

---
**Session End:** November 7, 2025, 11:30 PM
**Next Session:** November 8, 2025
**Status:** Partial success - Z interpolation fixed, some arcs complete, termination logic improved but not perfect
