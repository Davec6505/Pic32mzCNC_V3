# PIC32MZ Memory Map Reference

## Overview
PIC32MZ2048EFH100 flash memory layout for Pic32mzCNC_V3 CNC controller with MikroE USB HID bootloader.

**Total Flash**: 2MB (2048KB)  
**Available for Application**: ~1.87MB  
**Reserved for Bootloader**: 48KB  
**Settings Storage**: 16KB

---

## Memory Layout

### Physical Address Map
```
Physical Address    Virtual (KSEG1)     Size        Purpose
----------------    ---------------     ----        -------
0x1D000000          0x9D000000          1.87MB      Application Code
0x1D1F0000          0xBD1F0000          16KB        GRBL Settings (NVM)
0x1D1F4000          0x9D1F4000          48KB        MikroE Bootloader
0x1FC00000          0xBFC00000          12KB        Config Words
```

### Critical Memory Rules
- **Settings NVM**: `0xBD1F0000` (KSEG1 virtual for both reads and writes)
- **Page-aligned**: 16KB boundaries (0x4000)
- **Row-aligned**: 2048-byte boundaries (0x800)
- **Safe margin**: 64KB (0x10000) before bootloader at 0xBD1F4000
- **Never write to**: 0x9D1F4000 / 0xBD1F4000 (bootloader region)
- **Never write to**: 0xBFC00000 (boot flash config)

---

## Address Space (MIPS Architecture)

### Virtual Memory Regions
- **Physical (0x1D...)**: Internal flash controller addressing
- **Virtual KSEG1 (0xBD...)**: Uncached - **REQUIRED** for NVM operations (reads and writes)
- **Virtual KSEG0 (0x9D...)**: Cached - used for code execution

### Why KSEG1 for NVM?
The PIC32MZ flash controller requires uncached access for write operations. Using KSEG0 (cached) addresses will cause write failures.

**Correct NVM Address Usage:**
```c
// ✅ CORRECT - KSEG1 uncached address
#define SETTINGS_NVM_ADDRESS 0xBD1F0000

// ❌ WRONG - KSEG0 cached address  
#define SETTINGS_NVM_ADDRESS 0x9D1F0000  // Will fail on writes!
```

---

## NVM Operations (Harmony Pattern)

### Flash Specifications
- **Page size**: 16KB (must erase entire page before writing)
- **Row size**: 2048 bytes (512 words) - unit of RowWrite operations
- **Cache-aligned buffers REQUIRED**: Use `CACHE_ALIGN` attribute
- **Callback pattern**: Register handler, wait on `xferDone` flag
- **RowWrite preferred**: One operation vs many WordWrite operations

### NVM Write Pattern
```c
// Cache-aligned buffer (CRITICAL for PIC32MZ)
static uint32_t writeData[BUFFER_SIZE] CACHE_ALIGN;

// 1. Page Erase
NVM_PageErase(SETTINGS_NVM_ADDRESS);
while(xferDone == false);  // Wait for erase completion
xferDone = false;

// 2. Row Write
NVM_RowWrite((uint32_t *)writeData, SETTINGS_NVM_ADDRESS);
while(xferDone == false);  // Wait for write completion
xferDone = false;
```

### NVM Read Pattern
```c
// Simple direct read (no special handling needed)
CNC_Settings* flash_settings = (CNC_Settings*)SETTINGS_NVM_ADDRESS;
memcpy(&current_settings, flash_settings, sizeof(CNC_Settings));
```

---

## Bootloader Integration

### MikroE USB HID Bootloader
- **Location**: 0x9D1F4000 - 0x9D1FBFFF (48KB)
- **Entry Point**: Last 39KB of flash
- **Protected Region**: Never write to bootloader area
- **Reset Vector**: Bootloader controls startup, jumps to app

### Application Constraints
- **Max Application Size**: ~1.87MB (0x9D000000 - 0x9D1EFFFF)
- **Reserved NVM**: 16KB page-aligned at 0xBD1F0000
- **Safe Upper Limit**: 0x9D1EFFFF (64KB margin before bootloader)

### Linker Script Configuration
The linker script (`p32MZ2048EFH100.ld`) defines memory regions:
```
kseg0_program_mem : ORIGIN = 0x9D000000, LENGTH = 0x1F0000  /* 1.87MB */
kseg1_boot_mem    : ORIGIN = 0x9D1F4000, LENGTH = 0x8000    /* 48KB bootloader */
```

---

## Settings Storage Architecture

### GRBL Settings Structure
```c
typedef struct {
    // 29 GRBL parameters
    uint8_t pulse_microseconds;         // $0
    uint8_t stepper_idle_lock_time;     // $1
    // ... (26 more parameters)
    float arc_tolerance;                // $13
    // Work coordinate offsets (6 systems)
    CoordinatePoint work_offset[6];     // G54-G59
} CNC_Settings;
```

### Storage Location
- **Address**: `0xBD1F0000` (KSEG1 uncached)
- **Size**: < 512 bytes (fits in single row)
- **Alignment**: 16KB page boundary
- **Margin**: 64KB before bootloader

### Version Control
```c
#define SETTINGS_VERSION 2  // Increment when structure changes
```
When version mismatch detected, restore defaults and save to flash.

---

## Safety Guidelines

### ✅ DO
- Use KSEG1 addresses (0xBD...) for all NVM write operations
- Verify NVM_IsBusy() before operations
- Use CACHE_ALIGN attribute on write buffers
- Erase full page before writing
- Wait for xferDone callback completion
- Check address bounds before writes

### ❌ DON'T
- Write to KSEG0 addresses (0x9D...) for NVM operations
- Write to bootloader region (0x9D1F4000+)
- Write to config words (0xBFC00000+)
- Skip page erase before row write
- Use non-aligned buffers
- Assume immediate completion (always wait for callback)

---

## Common Issues & Solutions

### Problem: NVM Write Fails Silently
**Cause**: Using KSEG0 (cached) address instead of KSEG1 (uncached)  
**Solution**: Change address from 0x9D... to 0xBD...

### Problem: Settings Lost After Reset
**Cause**: Writing during peripheral initialization before NVM ready  
**Solution**: Delay NVM operations until APP_LOAD_SETTINGS state

### Problem: Flash Write Hangs
**Cause**: Not polling NVM_IsBusy() or xferDone flag  
**Solution**: Add proper wait loops with timeout protection

### Problem: Partial Data Written
**Cause**: Non-cache-aligned buffer  
**Solution**: Add `CACHE_ALIGN` attribute to buffer declaration

---

## Memory Usage Summary

| Region | Address | Size | Usage |
|--------|---------|------|-------|
| Application | 0x9D000000 | 1.87MB | Firmware code/data |
| Settings NVM | 0xBD1F0000 | 16KB | GRBL parameters |
| Bootloader | 0x9D1F4000 | 48KB | USB HID loader |
| Config Words | 0xBFC00000 | 12KB | Fuses/config bits |

**Current Firmware Size**: 264KB (14% of available space)  
**NVM Usage**: < 1KB (6% of reserved page)  
**Remaining Space**: ~1.6MB for future features

---

## References

- [PIC32MZ Datasheet](https://www.microchip.com/en-us/product/PIC32MZ2048EFH100)
- [Harmony NVM PLIB Documentation](https://microchip-mplab-harmony.github.io/core/)
- [MIPS Memory Management](https://www.mips.com/products/architectures/mips32-2/)
