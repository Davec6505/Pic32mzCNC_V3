#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"   // For HAS_TMC5160_AXIS, NUM_AXIS

// CNC Controller Settings Structure
typedef struct {
    // Signature for validation
    uint32_t signature;  // 0x47524231 = "GRB1"
    uint16_t version;    // Settings version number
    uint16_t padding;    // Alignment padding
    
    // Step configuration ($0-$5)
    uint32_t step_pulse_time;      // $0 - microseconds (1-255)
    uint32_t step_idle_delay;      // $1 - milliseconds (0-255)
    uint8_t step_pulse_invert;     // $2 - step port invert mask (0-255)
    uint8_t step_direction_invert; // $3 - direction port invert mask (0-255)
    uint8_t step_enable_invert;    // $4 - invert step enable pin (bool as uint8)
    uint8_t limit_pins_invert;     // $5 - invert limit pins (bool as uint8)
    uint8_t probe_invert;          // $6  - probe pin invert (0=active low, 1=active high)
    uint8_t status_report_mask;    // $10 - status report mask (default 3: MPos + buf state)
    uint8_t report_inches;         // $13 - 0=mm output, 1=inches output (GRBL v1.1)
    uint8_t soft_limits_enable;    // $20 - soft limit enable
    // NOTE: the 3 bytes above fill what was implicit compiler padding after probe_invert,
    // so no existing field offsets are shifted.

    // Arc configuration ($12)
    float mm_per_arc_segment;      // $12 - Arc segment chord length in mm (default 0.5mm)
    float arc_tolerance;           // internal - radius mismatch guard (not user-settable, not printed)
    
    // Motion configuration ($100-$132) - Array-based for scalability
    float steps_per_mm[4];         // $100-$103 - Steps per mm [X, Y, Z, A]
    float max_rate[4];             // $110-$113 - Max rate (mm/min) [X, Y, Z, A]
    float acceleration[4];         // $120-$123 - Acceleration (mm/sec^2) [X, Y, Z, A]
    float max_travel[4];           // $130-$133 - Max travel (mm) [X, Y, Z, A]

    // Jerk control — limits how fast acceleration can change ($140-$143)
    // Units: mm/s/step (effective jerk aggressiveness; higher = shorter S-curve ramp)
    // Formula: jerk_ramp_steps = acceleration * steps_per_mm / jerk
    // Low value  → long S-curve ramp  → very smooth, slightly slower acceleration
    // High value → short S-curve ramp → snappier, closer to pure trapezoidal
    float jerk[4];                 // $140-$143 - Jerk [X, Y, Z, A]

    // Spindle configuration
    float spindle_max_rpm;         // $30 - Max spindle speed (RPM)
    float spindle_min_rpm;         // $31 - Min spindle speed (RPM)
    
    // Homing configuration ($20-$27)
    uint8_t hard_limits_enable;    // $21 - Hard limit enable (bool as uint8)
    uint8_t homing_enable;         // $22 - Homing enable (DUAL MODE: UGS boolean 0/1, internal bitmask 0-15)
                                    //       UGS writes $22=1 → firmware uses 0x07 (XYZ enabled)
                                    //       UGS reads $$ → firmware reports 1 if any axis enabled
                                    //       Manual $22=7 works for per-axis control (advanced users)
    uint8_t homing_dir_mask;       // $23 - Homing dir invert mask
    uint8_t laser_mode;            // $32 - 0=CNC mode, 1=laser mode (replaces padding2, same offset)
    float homing_feed_rate;        // $24 - Homing locate feed rate (mm/min)
    float homing_seek_rate;        // $25 - Homing search seek rate (mm/min)
    uint32_t homing_debounce;      // $26 - Homing switch debounce (ms)
    float homing_pull_off;         // $27 - Homing switch pull-off distance (mm)
    
    // Junction deviation for smooth cornering
    float junction_deviation;      // $11 - Junction deviation (mm)
    
    // Work coordinate systems (G54-G59, G92 offset, Tool length offset)
    // GRBL v1.1 standard: 6 work coordinate systems + G92 offset + TLO
    // Array-based for scalability: [WCS_INDEX][AXIS]
    float wcs_g54[3];              // G54 work coordinate system [X, Y, Z]
    float wcs_g55[3];              // G55 work coordinate system [X, Y, Z]
    float wcs_g56[3];              // G56 work coordinate system [X, Y, Z]
    float wcs_g57[3];              // G57 work coordinate system [X, Y, Z]
    float wcs_g58[3];              // G58 work coordinate system [X, Y, Z]
    float wcs_g59[3];              // G59 work coordinate system [X, Y, Z]
    float g92_offset[3];           // G92 coordinate offset [X, Y, Z]
    float tool_length_offset;      // Tool length offset (TLO)

    // G28 / G30 stored machine positions (GRBL v1.1 — persist to NVM)
    // G28.1 / G30.1 saves current machine position; G28 / G30 rapids to it.
    float g28_position[4];         // G28 stored machine position [X, Y, Z, A]
    float g30_position[4];         // G30 stored machine position [X, Y, Z, A]
    
    // TMC5160 runtime tuning ($200-$253) — one slot per axis (0=X,1=Y,2=Z,3=A)
    // Always compiled in so the UI/serial interface can configure TMC5160 params
    // regardless of the compile-time AXIS_x_DRIVER selection.
    // Values are only written to hardware when HAS_TMC5160_AXIS is defined.
    uint8_t  tmc_mode[4];        // $200-$203: 1=StealthChop 2=SpreadCycle 3=Mixed 4=CoolStep
    uint8_t  tmc_irun[4];        // $210-$213: run current 0-31
    uint8_t  tmc_ihold[4];       // $220-$223: hold current 0-31
    uint8_t  tmc_mres[4];        // $230-$233: microstep resolution (TMC5160_MRES_xxx)
    uint32_t tmc_tpwm_thrs[4];   // $240-$243: Mixed mode StealthChop→SpreadCycle velocity threshold
    uint32_t tmc_tcoolthrs[4];   // $250-$253: CoolStep lower velocity threshold

    // Per-axis driver type — runtime selection exposed via serial ($260-$263)
    // 1 = DRIVER_TMC5160 (SPI + StepDir)   2 = DRIVER_DRV8825 (StepDir only)
    // Changing this persists to flash; a firmware rebuild may still be needed
    // before the hardware driver code for that axis is active.
    uint8_t  axis_driver_type[4]; // $260-$263: axis driver type per axis [X, Y, Z, A]

    // Startup lines ($N0 / $N1) — executed at boot after settings load
    char startup_line[2][80];

    // CRC32 checksum (for validation)
    uint32_t checksum;
} CNC_Settings;

// Default settings
#define SETTINGS_SIGNATURE 0x47524231  // "GRB1"
// Version tracks the BINARY layout of CNC_Settings.
// v8: TMC5160 fields ($200-$253) and axis_driver_type ($260-$263) are now always
//     compiled in (no #ifdef guard) so the UI/serial interface always exposes them.
#define SETTINGS_VERSION   8

// ✅ CRITICAL: Safe NVM storage location based on MikroE bootloader
// PIC32MZ2048EFH100 Program Flash with MikroE Bootloader:
// - Total: 2MB (0x200000 bytes)
// - Application Range: 0x9D000000 - 0x9D1EFFFF (1,966,080 bytes)
// - Safe Settings: 0xBD1F0000 - 0xBD1F3FFF (16KB, 64KB before bootloader)
// - MikroE Bootloader: 0xBD1F4000 - 0xBD1FFFFF (48KB)
//
// Address Space (MIPS Architecture):
// - Physical (0x1D...): Used by Flash controller for NVM operations
// - Virtual KSEG0 (0x9D...): Cached - USE THIS for NVM writes (Microchip pattern)
// - Virtual KSEG1 (0xBD...): Uncached - NOT for NVM writes!

//#define SETTINGS_FLASH_PAGE_SIZE    0x4000      // 16KB page
//#define SETTINGS_NVM_ADDRESS        0x9D180000  // ✅ KSEG0 cached for NVM writes (64KB before bootloader)
//#define SETTINGS_READ_ADDRESS       0x9D180000  // ✅ KSEG0 cached for reads

#define NVM_FLASH_START_ADDRESS    (0x9d000000U)
#define NVM_FLASH_SIZE             (0x200000U)
#define NVM_FLASH_ROWSIZE          (2048U)
#define NVM_FLASH_PAGESIZE         (16384U)



// Function prototypes
void SETTINGS_Initialize(void);
bool SETTINGS_LoadFromFlash(CNC_Settings* settings);
bool SETTINGS_SaveToFlash(const CNC_Settings* settings);
void SETTINGS_RestoreDefaults(CNC_Settings* settings);
bool SETTINGS_SetValue(CNC_Settings* settings, uint32_t parameter, float value);
float SETTINGS_GetValue(const CNC_Settings* settings, uint32_t parameter);
void SETTINGS_PrintAll(const CNC_Settings* settings);
void SETTINGS_PrintBuildInfo(void);
uint32_t SETTINGS_CalculateCRC32(const CNC_Settings* settings);
CNC_Settings* SETTINGS_GetCurrent(void);

// Work coordinate system functions
bool SETTINGS_GetWorkCoordinateSystem(uint8_t wcs_number, float* x, float* y, float* z);  // Get WCS (0=G54, 1=G55, etc.)
bool SETTINGS_SetWorkCoordinateSystem(uint8_t wcs_number, float x, float y, float z);     // Set WCS and save to flash
void SETTINGS_GetG92Offset(float* x, float* y, float* z);                                // Get G92 offset
void SETTINGS_SetG92Offset(float x, float y, float z);                                   // Set G92 offset
float SETTINGS_GetToolLengthOffset(void);                                                // Get TLO
void SETTINGS_SetToolLengthOffset(float offset);                                         // Set TLO

// G28 / G30 stored machine positions
void SETTINGS_GetG28Position(float out[4]);               // Get stored G28 machine position
void SETTINGS_SetG28Position(const float pos[4]);         // Save current pos as G28 (persists to NVM)
void SETTINGS_GetG30Position(float out[4]);               // Get stored G30 machine position
void SETTINGS_SetG30Position(const float pos[4]);         // Save current pos as G30 (persists to NVM)

#endif // SETTINGS_H