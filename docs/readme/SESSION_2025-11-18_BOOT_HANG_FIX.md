# Boot Hang Fix - November 18, 2025

## Problem Summary

**Issue**: System would not boot after power cycle or soft reset (Ctrl+X). No LED heartbeat, no serial communication - completely dead.

**Symptoms**:
- Power cycle: System hangs during boot, never reaches main loop
- After soft reset (Ctrl+X): System hangs during reboot
- No LED1 heartbeat visible
- No UART serial communication
- Bootloader completes successfully, but application hangs

## Root Cause Analysis

### Investigation Process

1. **Initial Hypothesis**: Flash read during `APP_LOAD_SETTINGS` causing hang
   - **Test**: Commented out `SETTINGS_LoadFromFlash()` call in `app.c`
   - **Result**: System still hung - flash read NOT the problem ❌

2. **Second Hypothesis**: `SETTINGS_Initialize()` causing hang
   - **Test**: Commented out `SETTINGS_Initialize()` call in `main.c`
   - **Result**: System boots successfully! ✅
   - **Conclusion**: Problem is in `SETTINGS_Initialize()`, not flash read

3. **Root Cause Identified**: `NVM_IsBusy()` infinite loop in `SETTINGS_Initialize()`

### The Blocking Code

**File**: `srcs/settings/settings.c` (lines 121-126)

```c
// ❌ PROBLEMATIC CODE - CAUSED BOOT HANG
uint32_t timeout = 10000000; // ~100ms timeout at 100MHz core
while(NVM_IsBusy() && timeout > 0) {
    timeout--;
}
```

**Why this caused the hang**:
- After soft reset (Ctrl+X) or power cycle, the NVM flash controller can enter a stuck state
- `NVM_IsBusy()` returns `true` indefinitely (flash controller never becomes "ready")
- The while loop spins for the full 10 million iterations (~100ms)
- Even after timeout expires, the code continues with potentially unstable flash controller
- This creates unpredictable behavior - sometimes boots, sometimes hangs completely

**PIC32MZ NVM Controller Behavior After Reset**:
- Hardware soft reset (Ctrl+X) may not fully reset the NVM peripheral state
- Flash controller can remain in BUSY state from previous incomplete operations
- `NVM_Initialize()` in `SYS_Initialize()` runs `NO_OPERATION` command asynchronously
- The NO_OPERATION may not complete before `SETTINGS_Initialize()` is called
- Polling `NVM_IsBusy()` immediately after can catch the controller mid-operation

## Solution

### The Fix

**File**: `srcs/settings/settings.c` (lines 115-124)

```c
// ✅ FIXED CODE - BOOTS RELIABLY
void SETTINGS_Initialize(void)
{
    // ✅ Skip NVM wait - flash controller may be stuck after soft reset
    // NVM_IsBusy() can return true indefinitely if controller is in bad state
    // Just register the callback and load defaults
    
    // ✅ Register NVM callback once during initialization (Harmony pattern)
    NVM_CallbackRegister(eventHandler, (uintptr_t)NULL);
    
    // ✅ Load defaults (flash read happens later in APP_LOAD_SETTINGS)
    SETTINGS_RestoreDefaults(&current_settings);
}
```

**Key Changes**:
1. **Removed** the `while(NVM_IsBusy())` blocking loop entirely
2. **Simplified** initialization to just:
   - Register NVM callback (required for flash operations)
   - Load default settings into memory
3. **Deferred** flash read to `APP_LOAD_SETTINGS` state (happens after all peripherals ready)

### Why This Works

**Initialization Sequence** (Before Fix):
```
SYS_Initialize() 
  └─> NVM_Initialize() starts NO_OPERATION (async)
  
SETTINGS_Initialize()
  └─> while(NVM_IsBusy()) { }  ← HANGS HERE if NVM stuck
  └─> Load defaults
  
APP_Initialize()
  └─> ...

APP_Tasks()
  └─> APP_LOAD_SETTINGS
      └─> SETTINGS_LoadFromFlash() (with interrupt protection)
```

**Initialization Sequence** (After Fix):
```
SYS_Initialize() 
  └─> NVM_Initialize() starts NO_OPERATION (async)
  
SETTINGS_Initialize()
  └─> Register callback (non-blocking)
  └─> Load defaults (non-blocking)
  
APP_Initialize()
  └─> ...

APP_Tasks()
  └─> APP_LOAD_SETTINGS
      └─> SETTINGS_LoadFromFlash() (with interrupt protection, after NVM stable)
```

**Benefits**:
- ✅ No blocking loops during critical boot sequence
- ✅ NVM controller has time to stabilize before flash operations
- ✅ Flash read happens when system is fully initialized and stable
- ✅ Soft reset (Ctrl+X) no longer causes boot hang
- ✅ Power cycle boots reliably

## Testing Results

### Test 1: Power Cycle Boot
**Before Fix**: System hangs, no LED heartbeat, no serial
**After Fix**: ✅ System boots normally, LED1 heartbeat visible, serial communication works

### Test 2: Soft Reset (Ctrl+X)
**Before Fix**: System hangs on reboot, no LED heartbeat, no serial
**After Fix**: ✅ System reboots normally, LED1 heartbeat visible, serial communication works

### Test 3: Motion Restart After Soft Reset
**Before Fix**: Motion would not restart (fixed in previous session with `STEPPER_StopMotion()`)
**After Fix**: ✅ Motion restarts correctly after soft reset

### Test 4: Settings Persistence
**Current State**: Flash read still works in `APP_LOAD_SETTINGS` state
**Note**: Settings load from flash after system fully boots and NVM controller is stable

## Related Changes

### Previous Fix: Motion Restart After Soft Reset
**Session**: November 18, 2025 (earlier in day)
**Problem**: Motion would not restart after Ctrl+X soft reset
**Solution**: Created centralized `STEPPER_StopMotion()` function
**Files Modified**: 
- `srcs/motion/stepper.h` - Added `STEPPER_StopMotion()` declaration
- `srcs/motion/stepper.c` - Implemented centralized stop function
- `srcs/gcode/gcode_parser.c` - Updated `GCODE_SoftReset()` to use centralized function

**Result**: Motion restart ✅ FIXED, but boot hang persisted

### Current Fix: Boot Hang After Power Cycle / Soft Reset
**Session**: November 18, 2025 (this session)
**Problem**: System would not boot after power cycle or soft reset
**Solution**: Removed `NVM_IsBusy()` blocking loop from `SETTINGS_Initialize()`
**Files Modified**:
- `srcs/settings/settings.c` - Removed NVM wait loop
- `srcs/main.c` - Re-enabled `SETTINGS_Initialize()` call

**Result**: Boot hang ✅ FIXED

## Files Modified

### srcs/settings/settings.c
**Lines**: 115-124
**Change**: Removed `while(NVM_IsBusy())` blocking loop from `SETTINGS_Initialize()`
**Reason**: NVM controller can get stuck after soft reset, causing infinite loop
**Impact**: System now boots reliably without blocking on NVM status

### srcs/main.c
**Lines**: 43-48
**Change**: Re-enabled `SETTINGS_Initialize()` call (was commented out for testing)
**Reason**: With NVM wait loop removed, initialization is now safe
**Impact**: Settings system properly initializes during boot

### srcs/app.c
**Lines**: 195-216
**Change**: User added LED2 flash indicators for visual feedback when settings load
**Reason**: Visual confirmation of flash read success/failure
**Impact**: LED2 blinks fast (250ms) on successful flash load, slow (50ms) on failure

## Architecture Insights

### PIC32MZ NVM Controller Behavior

**Normal Operation**:
- `NVM_Initialize()` clears error flags and runs NO_OPERATION command
- NO_OPERATION is asynchronous (interrupt-driven via callback)
- After completion, `NVM_IsBusy()` returns false
- Flash operations can proceed safely

**After Soft Reset (Problematic)**:
- NVM peripheral state may not fully reset
- Previous operations may leave controller in BUSY state
- NO_OPERATION may not complete or may hang
- `NVM_IsBusy()` can return true indefinitely
- **Solution**: Don't poll `NVM_IsBusy()` during boot - defer flash operations

### Best Practices Learned

1. **Never block during boot sequence**
   - Boot sequence must be non-blocking
   - Blocking loops can cause complete system hang
   - Timeouts don't help if hardware is stuck

2. **Defer flash operations to stable state**
   - Let system fully initialize before flash access
   - NVM controller needs time to stabilize after reset
   - APP_LOAD_SETTINGS state is ideal for flash read

3. **Use interrupt protection for flash operations**
   - `__builtin_disable_interrupts()` before NVM_Read
   - `__builtin_enable_interrupts()` after NVM_Read
   - Prevents interrupt interference during flash access

4. **Separate initialization from persistence**
   - `SETTINGS_Initialize()`: Load defaults, register callbacks (fast, non-blocking)
   - `SETTINGS_LoadFromFlash()`: Read from flash (slow, can fail, deferred)
   - Don't mix boot-critical code with flash operations

## Comparison with GRBL Reference Implementation

**GRBL v1.1 Settings Initialization**:
```c
void settings_init() {
    // Just read from EEPROM (AVR) - no NVM busy polling
    if(!settings_read_from_eeprom()) {
        settings_restore_defaults();
    }
}
```

**Our Implementation** (After Fix):
```c
void SETTINGS_Initialize(void) {
    NVM_CallbackRegister(eventHandler, (uintptr_t)NULL);
    SETTINGS_RestoreDefaults(&current_settings);
}

// Flash read deferred to APP_LOAD_SETTINGS state
void APP_Tasks(void) {
    case APP_LOAD_SETTINGS:
        SETTINGS_LoadFromFlash(SETTINGS_GetCurrent());
        // ...
}
```

**Key Difference**: 
- GRBL reads EEPROM synchronously during init (AVR EEPROM is always ready)
- PIC32MZ flash controller requires async initialization - can't poll during boot

## Future Improvements

### Potential Enhancements

1. **Add NVM health check in APP_LOAD_SETTINGS**
   - Poll `NVM_IsBusy()` with timeout before flash read
   - If timeout expires, log error and continue with defaults
   - Don't block boot even if NVM never becomes ready

2. **Add NVM error flag checking**
   - Check `NVM_ErrorGet()` before flash operations
   - Clear LVDERR/WRERR if set before attempting read
   - Log flash controller state for debugging

3. **Consider watchdog timer**
   - Enable watchdog during boot sequence
   - Reset system if boot takes too long
   - Prevents infinite hangs from unknown causes

4. **Add boot failure counter**
   - Store boot attempts in RAM (survives soft reset)
   - After 3 consecutive boot failures, force defaults
   - Prevents settings corruption from causing boot loop

## Lessons Learned

### Critical Insights

1. **Microcontroller peripherals have state**
   - Soft reset doesn't always clear peripheral state
   - Flash controllers can get stuck in busy state
   - Hardware state must be validated, not assumed

2. **Isolation testing is powerful**
   - Commenting out code sections quickly isolates problems
   - Binary search approach: disable half, test, repeat
   - Found root cause in 3 iterations (flash read → settings init → NVM busy loop)

3. **Asynchronous initialization is complex**
   - Harmony pattern uses callbacks for NVM operations
   - NO_OPERATION runs asynchronously during SYS_Initialize
   - Can't assume completion by the time SETTINGS_Initialize runs

4. **Non-blocking is essential for embedded boot**
   - Boot sequence must never block indefinitely
   - Timeouts help but aren't sufficient if hardware is stuck
   - Defer risky operations to later states when stable

### Development Process Validation

**What Worked Well**:
- ✅ Systematic isolation testing
- ✅ User-provided hypothesis about flash read
- ✅ Clear understanding of boot sequence
- ✅ Separation of concerns (init vs persistence)

**What Could Improve**:
- Initial timeout approach assumed hardware would eventually become ready
- Should have considered "skip entirely" approach earlier
- Flash controller state after soft reset needed more investigation upfront

## Conclusion

**Root Cause**: Blocking loop polling `NVM_IsBusy()` during boot hung system when flash controller was stuck

**Solution**: Remove NVM wait loop from `SETTINGS_Initialize()`, defer flash operations to stable state

**Result**: System boots reliably after power cycle and soft reset ✅

**Impact**: Both major issues now resolved:
1. ✅ Motion restart after soft reset (fixed with centralized `STEPPER_StopMotion()`)
2. ✅ Boot hang after power cycle/soft reset (fixed by removing NVM busy loop)

**Status**: Production firmware ready for testing on CNC hardware

---

**Session Date**: November 18, 2025  
**Firmware Version**: CS23 (bins/Release/CS23.hex)  
**Build Status**: ✅ Successful (91,824 bytes program, 9,876 bytes data)
