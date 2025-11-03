#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

// GRBL v1.1 Settings (subset for CNC controller)
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
    
    // Motion configuration ($100-$132)
    float steps_per_mm_x;          // $100 - X-axis steps per mm
    float steps_per_mm_y;          // $101 - Y-axis steps per mm
    float steps_per_mm_z;          // $102 - Z-axis steps per mm
    float steps_per_mm_a;          // $103 - A-axis steps per mm
    
    float max_rate_x;              // $110 - X-axis max rate (mm/min)
    float max_rate_y;              // $111 - Y-axis max rate (mm/min)
    float max_rate_z;              // $112 - Z-axis max rate (mm/min)
    float max_rate_a;              // $113 - A-axis max rate (mm/min)
    
    float acceleration_x;          // $120 - X-axis acceleration (mm/sec^2)
    float acceleration_y;          // $121 - Y-axis acceleration (mm/sec^2)
    float acceleration_z;          // $122 - Z-axis acceleration (mm/sec^2)
    float acceleration_a;          // $123 - A-axis acceleration (mm/sec^2)
    
    float max_travel_x;            // $130 - X-axis max travel (mm)
    float max_travel_y;            // $131 - Y-axis max travel (mm)
    float max_travel_z;            // $132 - Z-axis max travel (mm)
    
    // Spindle configuration
    float spindle_max_rpm;         // $30 - Max spindle speed (RPM)
    float spindle_min_rpm;         // $31 - Min spindle speed (RPM)
    
    // Homing configuration ($20-$27)
    uint8_t homing_enable;         // $22 - Homing cycle enable (bool as uint8)
    uint8_t homing_dir_mask;       // $23 - Homing dir invert mask
    uint16_t padding2;             // Alignment padding
    float homing_feed_rate;        // $24 - Homing locate feed rate (mm/min)
    float homing_seek_rate;        // $25 - Homing search seek rate (mm/min)
    uint32_t homing_debounce;      // $26 - Homing switch debounce (ms)
    float homing_pull_off;         // $27 - Homing switch pull-off distance (mm)
    
    // CRC32 checksum (for validation)
    uint32_t checksum;
} GRBL_Settings;

// Default settings
#define SETTINGS_SIGNATURE 0x47524231  // "GRB1"
#define SETTINGS_VERSION   1

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
bool SETTINGS_LoadFromFlash(GRBL_Settings* settings);
bool SETTINGS_SaveToFlash(const GRBL_Settings* settings);
void SETTINGS_RestoreDefaults(GRBL_Settings* settings);
bool SETTINGS_SetValue(GRBL_Settings* settings, uint32_t parameter, float value);
float SETTINGS_GetValue(const GRBL_Settings* settings, uint32_t parameter);
void SETTINGS_PrintAll(const GRBL_Settings* settings);
void SETTINGS_PrintBuildInfo(void);
uint32_t SETTINGS_CalculateCRC32(const GRBL_Settings* settings);
GRBL_Settings* SETTINGS_GetCurrent(void);

#endif // SETTINGS_H