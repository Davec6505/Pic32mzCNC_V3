#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

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
    
    // Arc configuration ($12-$13)
    float mm_per_arc_segment;      // $12 - Arc segment length in mm (default 0.1mm)
    float arc_tolerance;           // $13 - Arc tolerance in mm (default 0.002mm, GRBL v1.1)
    
    // Motion configuration ($100-$132) - Array-based for scalability
    float steps_per_mm[4];         // $100-$103 - Steps per mm [X, Y, Z, A]
    float max_rate[4];             // $110-$113 - Max rate (mm/min) [X, Y, Z, A]
    float acceleration[4];         // $120-$123 - Acceleration (mm/sec^2) [X, Y, Z, A]
    float max_travel[4];           // $130-$133 - Max travel (mm) [X, Y, Z, A]
    
    // Spindle configuration
    float spindle_max_rpm;         // $30 - Max spindle speed (RPM)
    float spindle_min_rpm;         // $31 - Min spindle speed (RPM)
    
    // Homing configuration ($20-$27)
    uint8_t hard_limits_enable;    // $21 - Hard limit enable (bool as uint8)
    uint8_t homing_enable;         // $22 - Homing cycle enable (bool as uint8)
    uint8_t homing_dir_mask;       // $23 - Homing dir invert mask
    uint8_t padding2;              // Alignment padding
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
    
    // CRC32 checksum (for validation)
    uint32_t checksum;
} CNC_Settings;

// Default settings
#define SETTINGS_SIGNATURE 0x47524231  // "GRB1"
#define SETTINGS_VERSION   2           // Incremented when structure changes (was 1)

// ✅ CRITICAL: Safe NVM storage location based on MikroE bootloader
// PIC32MZ2048EFH100 Program Flash with MikroE Bootloader:
// - Total: 2MB (0x200000 bytes)
// - Application Range: 0x9D000000 - 0x9D1EFFFF (1,966,080 bytes)
// - Safe Settings: 0xBD1F0000 - 0xBD1F3FFF (16KB, 64KB before bootloader)
// - MikroE Bootloader: 0xBD1F4000 - 0xBD1FFFFF (48KB)
//
// Address Space (MIPS Architecture):
// - Physical (0x1D...): Used by Flash controller for NVM operations
// - Virtual KSEG1 (0xBD...): Uncached, used for NVM and CPU reads/execution
// - Virtual KSEG0 (0x9D...): Cached, used by CPU for fast execution

#define SETTINGS_FLASH_PAGE_SIZE    0x4000      // 16KB page
#define SETTINGS_NVM_ADDRESS        0xBD1F0000  // ✅ Virtual KSEG1 for NVM writes (64KB before bootloader)
#define SETTINGS_READ_ADDRESS       0xBD1F0000  // ✅ Virtual KSEG1 for reads

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

#endif // SETTINGS_H