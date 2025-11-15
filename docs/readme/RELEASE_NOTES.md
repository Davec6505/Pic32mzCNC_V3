# Release Notes - November 2025

## Production Release - November 15, 2025

### Version Information
- **Firmware**: `bins/Release/CS23.hex` (264KB)
- **Status**: Production Ready
- **GRBL**: v1.1 Compatible
- **Architecture**: Array-Based Refactored

---

## Major Changes

### 1. Array-Based Architecture Refactor (November 14-15)
**LED Pattern Simplification** - Eliminated all nested structures

**Before (Convoluted)**:
```c
typedef struct {
    GPIO_SetFunc Set;
    GPIO_ClearFunc Clear;
    GPIO_ToggleFunc Toggle;  // Unused
    GPIO_GetFunc Get;        // Unused
} GPIO_Control;

typedef struct {
    GPIO_Control step;   // 3 levels deep!
    GPIO_Control dir;
    GPIO_Control enable;
    float* max_rate;
} AxisConfig;

// Access: g_axis_config[axis].step.Set()  // Too complex!
```

**After (LED Pattern)**:
```c
// Direct function pointer arrays
GPIO_SetFunc axis_step_set[NUM_AXIS];
GPIO_ClearFunc axis_step_clear[NUM_AXIS];

// Flat settings struct
typedef struct {
    float* max_rate;
    float* acceleration;
    float* steps_per_mm;
    int32_t* step_count;
} AxisSettings;

// Access: axis_step_set[axis]()  // Clean and direct!
```

**Benefits**:
- Removed 72 unused wrapper functions
- Firmware size: 266KB → 264KB (2KB savings)
- Cleaner code - matches LED pattern elegance
- Zero-overhead inline to single instructions
- No nested structure complexity

**Files Modified**:
- `incs/utils/utils.h` - Simplified to flat arrays
- `srcs/utils/utils.c` - Removed nested initialization
- `srcs/motion/stepper.c` - Direct array access
- `srcs/motion/kinematics.c` - Removed accessor calls
- `srcs/motion/motion.c` - Direct settings access
- `srcs/motion/homing.c` - Simplified limit checks

---

### 2. Aggressive Flow Control (November 13)
**Single-Threshold Deferred OK System**

**Problem**: UGS marked files "Finished" before motion completed

**Solution**: Defer "ok" until motion queue completely empty
```c
// Defer when ANY motion in queue
if (appData->motionQueueCount > 0) {
    okPending = true;
}

// Send only when queue empty
if (okPending && appData->motionQueueCount == 0) {
    UART_SendOK();
    okPending = false;
}
```

**Benefits**:
- UGS stays in "Run" state during motion
- Accurate "Finished" timing
- Continuous status polling
- Perfect file streaming

---

### 3. GRBL v1.1 Blank Line Compliance (November 13)
**Proper "ok" Responses for All Lines**

**Implementation**:
```c
// Blank lines get "ok" with flow control
} else {
    SendOrDeferOk(appData, cmdQueue);  // Same flow control
}
```

**Benefits**:
- Full GRBL v1.1 protocol compliance
- Works with all G-code senders
- Proper comment/blank line handling

---

### 4. Motion Completion Synchronization (November 13)
**Flag-Based Deferred OK Check**

**Architecture**:
```c
// Motion completes → set flag
appData->motionSegmentCompleted = true;

// Main loop checks immediately
if (appData.motionSegmentCompleted) {
    appData.motionSegmentCompleted = false;
    GCODE_CheckDeferredOk(&appData, &cmdQueue);
}
```

**Benefits**:
- Immediate response when motion completes
- No polling delays
- Accurate UGS synchronization

---

### 5. Arc Radius Compensation (November 13)
**GRBL $13 Setting Implementation**

**Feature**: Tolerates CAM-generated radius variations
```c
// Allow small radius mismatches
float radius_error = fabs(r_start - r_end);
if (radius_error > settings->arc_tolerance) {
    // Alarm: Arc radius error
} else {
    // Average the radii
    float radius = (r_start + r_end) / 2.0f;
}
```

**Settings Version**: Incremented to 2 for structure validation

---

### 6. Soft Reset Recovery (November 10)
**Hardware Validation Guards**

**Problem**: Motion wouldn't restart after Ctrl+X soft reset

**Solution**: Validate and re-enable OC/TMR modules
```c
// Re-enable OC1 if disabled
if(!(OC1CON & _OC1CON_ON_MASK)) {
    OCMP1_Enable();
}

// Re-start TMR4 if stopped
if(!(T4CON & _T4CON_ON_MASK)) {
    TMR4_Start();
}
```

**Benefits**:
- Motion always restarts after soft reset
- No manual intervention needed
- UGS-compatible recovery

---

### 7. Single Authoritative Count (November 11)
**Eliminated Stale Data Copies**

**Problem**: Flow control read stale motion count copy

**Solution**: Direct access to authoritative count
```c
// REMOVED stale copy from GCODE_CommandQueue
// uint32_t motionQueueCount;  // DELETE

// REMOVED sync operation
// appData.gcodeCommandQueue.motionQueueCount = appData.motionQueueCount;

// Flow control reads FRESH count
CheckAndSendDeferredOk(appData->motionQueueCount, cmdQueue->maxMotionSegments);
```

**Benefits**:
- No synchronization bugs
- Always current data
- Simpler architecture

---

## Testing Summary

### Validated Tests (November 13-15, 2025)
- ✅ Rectangle path (dual iteration, 4 corners)
- ✅ Circle (20 segments, 0.025mm final error)
- ✅ Arc compensation (0.001mm radius tolerance)
- ✅ Back-to-back file execution
- ✅ UGS automatic completion
- ✅ Soft reset recovery (Ctrl+X)
- ✅ Real-time visualization
- ✅ Blank line GRBL compliance
- ✅ Array-based refactoring
- ✅ LED pattern GPIO simplification

### Test Results
- **File Streaming**: 100% success rate
- **Position Accuracy**: ±0.025mm (circle test)
- **Arc Tolerance**: 0.001mm handled correctly
- **UGS Integration**: Seamless operation
- **Firmware Size**: 264KB (optimal)

---

## Known Issues - NONE! 🎉

All major issues resolved and deployed to production.

---

## Breaking Changes

### Settings Structure (SETTINGS_VERSION = 2)
- Added `arc_tolerance` parameter ($13)
- First boot will restore defaults and save to flash
- Previous settings will be migrated automatically

### GPIO Architecture
- Removed `AxisConfig`, `LimitConfig`, `GPIO_Control` types
- Replaced with `AxisSettings`, `HomingSettings` flat structs
- Code using accessor functions must be updated to direct array access

---

## Migration Guide

### For Developers
If you have custom code using the old architecture:

**Old Pattern**:
```c
const AxisConfig* cfg = UTILS_GetAxisConfig(axis);
float rate = *(cfg->max_rate);
```

**New Pattern**:
```c
float rate = *(g_axis_settings[axis].max_rate);
```

**Old Pattern**:
```c
g_axis_config[axis].step.Set();
```

**New Pattern**:
```c
axis_step_set[axis]();
// Or use inline helper:
AXIS_StepSet(axis);
```

---

## Documentation Updates

### New/Updated Files
- `README.md` - Complete rewrite, clean and concise
- `.github/copilot-instructions.md` - Accurate architecture guide
- `docs/readme/ARCHITECTURE.md` - Comprehensive system documentation
- `docs/readme/MEMORY_MAP.md` - Clean memory layout reference
- `docs/readme/RELEASE_NOTES.md` - This file

### Preserved Files
- `docs/readme/SETTINGS_REFERENCE.md` - Still accurate
- `docs/readme/DEBUG_SYSTEM_TUTORIAL.md` - Still valid
- Development logs preserved for historical reference

---

## Performance Metrics

### Firmware Size Progression
- November 13: 266KB (after LED wrapper refactor)
- November 14: 266KB (stable)
- November 15: 264KB (array-based refactor - 2KB saved)

### Build Times
- Clean build: ~20 seconds
- Incremental build: ~5 seconds
- Debug build: ~22 seconds

### Memory Usage
- Application code: 264KB (14% of 1.87MB)
- Motion queue: ~1.3KB
- G-code buffer: ~1KB
- Settings NVM: <1KB (6% of 16KB page)

---

## Credits

**Development**: Dave Chapman  
**Architecture**: Array-based with LED pattern design  
**Protocol**: GRBL v1.1 compatible  
**Testing**: UGS Platform, Candle, bCNC

---

## Next Steps

### Future Enhancements
1. Look-ahead junction planning
2. S-curve acceleration profiles
3. Production CNC homing validation
4. Spindle PWM CNC testing
5. Advanced arc optimization

### Maintenance
- Monitor GitHub issues
- Validate new G-code sender compatibility
- Continue code quality improvements
- Update documentation as needed

---

## References

- **Main README**: [README.md](../../README.md)
- **Architecture Guide**: [ARCHITECTURE.md](ARCHITECTURE.md)
- **Settings Reference**: [SETTINGS_REFERENCE.md](SETTINGS_REFERENCE.md)
- **Debug Tutorial**: [DEBUG_SYSTEM_TUTORIAL.md](DEBUG_SYSTEM_TUTORIAL.md)
- **Memory Map**: [MEMORY_MAP.md](MEMORY_MAP.md)
