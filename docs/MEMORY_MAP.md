# PIC32MZ2048EFH100 Memory Map

## Flash Memory Layout (2MB Total)

### Program Flash (2,097,152 bytes)

| Physical Address | KSEG1 Address | Size    | Region                  | Usage                          |
|------------------|---------------|---------|-------------------------|--------------------------------|
| 0x1D000000       | 0x9D000000    | 1.87MB  | Application Code        | Main CNC controller firmware   |
| 0x1D1F0000       | 0x9D1F0000    | 16KB    | **Settings Storage**    | GRBL settings (NVM)            |
| 0x1D1F4000       | 0x9D1F4000    | 48KB    | **MikroE Bootloader**   | USB HID bootloader (protected) |

### Boot Flash (12KB)

| Physical Address | KSEG1 Address | Size    | Region                  | Usage                          |
|------------------|---------------|---------|-------------------------|--------------------------------|
| 0x1FC00000       | 0xBFC00000    | 12KB    | Boot Flash              | Reset vector, config words     |

## MikroE Bootloader Details

**From `Config.c` (MikroE USB HID Bootloader):**
```c
const unsigned long BOOTLOADER_SIZE = 39000;  // 39KB (~0x9858 bytes)
```

**Bootloader Start Calculation:**
```
BOOTLOADER_START_PHY = 0x1D000000 + ((__FLASH_SIZE - BOOTLOADER_SIZE) / _FLASH_ERASE) * _FLASH_ERASE
                     = 0x1D000000 + ((2097152 - 39000) / 16384) * 16384
                     = 0x1D000000 + (125 * 16384)
                     = 0x1D000000 + 0x1F4000
                     = 0x1D1F4000 (physical)
                     = 0x9D1F4000 (KSEG1 cached)
```

## GRBL Settings Storage

**Settings Structure Size:** ~140 bytes (fits in single 16KB page with CRC validation)

**Storage Location:**
- **Physical Address (for NVM writes):** `0x1D1F0000`
- **KSEG1 Virtual Address (for reads):** `0x9D1F0000`
- **Page Size:** 16KB (0x4000 bytes)
- **Pages Reserved:** 1 page (16KB)
- **Safety Margin:** 4 pages (64KB) before bootloader

**NVM Operations:**
```c
// Erase settings page - MUST use physical address
NVM_PageErase(0x1D1F0000);  // Physical address required by Flash controller

// Write settings (quad-word aligned) - MUST use physical address
NVM_QuadWordWrite(&data[i], 0x1D1F0000 + offset);  // Physical address for NVMADDR

// Read settings - Use virtual address (uncached for coherency)
memcpy(&settings, (void*)0x9D1F0000, sizeof(GRBL_Settings));  // KSEG1 virtual address
```

## Memory Protection Rules

### ✅ Safe Operations
- **Application Code (virtual):** 0x9D000000 - 0x9D1EFFFF (1.87MB available)
- **Settings Storage (physical):** 0x1D1F0000 - 0x1D1F3FFF (16KB page for NVM ops)
- **Settings Storage (virtual):** 0x9D1F0000 - 0x9D1F3FFF (16KB page for reads)
- **Reading anywhere:** Virtual addresses (0x8D.../0x9D...) are safe for reads

### ❌ Forbidden Operations
- **Never erase/write (physical):** 0x1D1F4000 - 0x1D1FFFFF (bootloader region)
- **Never erase/write (virtual):** 0x9D1F4000 - 0x9D1FFFFF (bootloader region)
- **Never erase/write:** 0xBFC00000 - 0xBFC02FFF (boot flash config words)
- **Never exceed:** Application code must stay below 0x1D1F0000 (physical) / 0x9D1F0000 (virtual)

## Address Space Segments (MIPS)

| Segment | Address Range           | Cached  | Type    | Usage                                    |
|---------|-------------------------|---------|---------|------------------------------------------|
| KSEG0   | 0x80000000 - 0x9FFFFFFF | Yes     | Virtual | Cached memory access (fast execution)    |
| KSEG1   | 0xA0000000 - 0xBFFFFFFF | No      | Virtual | Uncached access (peripherals, I/O)       |
| Physical| 0x00000000 - 0x1FFFFFFF | N/A     | Physical| Used by DMA, Flash controller, NVM ops   |

**Program Flash Memory Mapping (PIC32MZ2048EFH100):**
- **Physical Address:** 0x1D000000 - 0x1D1FFFFF (2MB)
- **KSEG1 Virtual (uncached):** 0x9D000000 - 0x9D1FFFFF (2MB)
- **KSEG0 Virtual (cached):** 0x8D000000 - 0x8D1FFFFF (2MB)

**Address Usage Best Practices:**
- Use **physical addresses (0x1D...)** for NVM erase/write operations (Flash controller requires physical)
- Use **KSEG1 virtual (0x9D...)** for reading settings and data (uncached, coherent)
- Use **KSEG0 virtual (0x8D...)** for application code execution (cached, faster)

**Important:** The CPU uses virtual addresses (0x8D.../0x9D...), but peripherals like the NVM Flash controller use physical addresses (0x1D...). The Flash Programming Reference Manual requires physical addresses for NVMADDR register.

## Flash Erase/Write Characteristics

| Parameter            | Value          | Notes                                    |
|----------------------|----------------|------------------------------------------|
| Page Size            | 16KB (0x4000)  | Minimum erase block                      |
| Write Block          | 128 bits       | Quad-word (4 words = 16 bytes)           |
| Erase Time           | ~40ms typical  | Per page                                 |
| Write Time           | ~2ms typical   | Per quad-word                            |
| Endurance            | 10,000 cycles  | Typical flash endurance                  |

## Settings Storage Strategy

**GRBL Settings Structure:**
```c
typedef struct {
    uint32_t signature;      // 0x47524231 = "GRB1"
    uint16_t version;        // Settings version
    // ... motion parameters ...
    uint32_t checksum;       // CRC32 for validation
} GRBL_Settings;
```

**Validation on Load:**
1. Check signature matches `0x47524231`
2. Verify CRC32 checksum
3. If invalid, restore defaults and save to flash

**Wear Leveling Considerations:**
- Settings changes are infrequent (only on `$x=val` commands)
- 10,000 cycle endurance = 10,000 complete settings saves
- With ~10 settings changes per day = 2.7+ years lifespan
- No additional wear leveling needed for this application

## Bootloader Update Procedure

⚠️ **WARNING:** Updating the bootloader requires specialized tools and can brick the device!

**MikroE Bootloader Update:**
1. Use MikroE PIC32 Bootloader programming tool
2. Connect via ICSP/PICkit
3. Erase entire flash (including bootloader region)
4. Program new bootloader firmware
5. Verify bootloader operation before programming application

**Application Update (via Bootloader):**
1. Connect USB cable
2. Run MikroE HID Bootloader PC application
3. Select `.hex` file from `bins/Release/` folder
4. Click "Write" - bootloader programs application region only
5. Settings at 0x9D1F0000 are preserved ✅

## Memory Map Visualization

```
Program Flash (2MB)
┌────────────────────────────────────────┐ 0x9D000000
│                                        │
│         Application Code               │
│         (1,966,080 bytes)              │ 1.87MB
│                                        │
├────────────────────────────────────────┤ 0x9D1F0000
│  GRBL Settings Storage (16KB)          │ ✅ SAFE NVM
├────────────────────────────────────────┤ 0x9D1F4000
│  MikroE USB HID Bootloader (48KB)      │ ❌ PROTECTED
└────────────────────────────────────────┘ 0x9D200000

Boot Flash (12KB)
┌────────────────────────────────────────┐ 0xBFC00000
│  Reset Vector + Config Words           │ ❌ PROTECTED
└────────────────────────────────────────┘ 0xBFC03000
```

## References

- **Microchip DS60001320:** PIC32MZ Embedded Connectivity (EF) Family Datasheet
- **Microchip DS60001214:** PIC32 Flash Programming Specification
- **MikroE:** USB HID Bootloader for PIC32MZ
- **GRBL v1.1:** Settings Storage Specification

---

**Last Updated:** November 2, 2025  
**Author:** CNC Controller Development Team  
**Hardware:** PIC32MZ2048EFH100 Clicker 2 Board
