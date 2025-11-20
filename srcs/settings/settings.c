#include "settings.h"
#include "peripheral/nvm/plib_nvm.h"
#include "peripheral/uart/plib_uart3.h"
#include "peripheral/coretimer/plib_coretimer.h"  // ✅ For CORETIMER_DelayMs/Us
#include "utils/uart_utils.h"  // ✅ For non-blocking UART_Printf()
#include "definitions.h"
#include <string.h>
#include <stdio.h>

// Global settings instance (loaded at startup)
static CNC_Settings current_settings;

// ✅ CRITICAL: Cache-aligned buffer for NVM operations (Harmony pattern)
// PIC32MZ requires cache-aligned buffers for flash read/write operations
// Buffer size MUST match NVM_FLASH_ROWSIZE for NVM_RowWrite() operations
//#define READ_WRITE_SIZE         (NVM_FLASH_PAGESIZE)
//#define BUFFER_SIZE             (READ_WRITE_SIZE / sizeof(uint32_t))
#define READ_WRITE_SIZE         (NVM_FLASH_PAGESIZE)
#define BUFFER_SIZE             (READ_WRITE_SIZE / sizeof(uint32_t))
#define HALF_SIZE               (NVM_FLASH_SIZE / 2)
#define QUARTER_SIZE            (NVM_FLASH_SIZE / 4)
#define EIGHTH_SIZE             (NVM_FLASH_SIZE / 8)
#define APP_FLASH_ADDRESS       (NVM_FLASH_START_ADDRESS + (HALF_SIZE) + (QUARTER_SIZE) + (EIGHTH_SIZE))
//                              = 0x9D000000 + 0x100000 + 0x80000 + 0x40000
//                              = 0x9D1C0000  (1.75MB mark, 87.5% of flash)


static uint32_t writeData[BUFFER_SIZE] CACHE_ALIGN;

// ✅ CRITICAL: Harmony pattern - callback flag for NVM operations
static volatile bool xferDone = false;

// ✅ CRITICAL: Harmony pattern - event handler callback
static void eventHandler(uintptr_t context)
{
    xferDone = true;
}

// Default settings values
static const CNC_Settings default_settings = {
    .signature = SETTINGS_SIGNATURE,
    .version = SETTINGS_VERSION,
    .padding = 0,
    
    // Step configuration
    .step_pulse_time = 10,         // 10 microseconds
    .step_idle_delay = 25,         // 25 milliseconds
    .step_pulse_invert = 0,
    .step_direction_invert = 0,
    .step_enable_invert = 0,       // false as uint8
    .limit_pins_invert = 0,        // false as uint8
    
    // Steps per mm (typical for 1/8 microstepping, 200 steps/rev, 5mm pitch) - Array-based
    .steps_per_mm = {156.0f, 156.0f, 156.0f, 156.0f},  // [X, Y, Z, A]
    
    // Max rates (mm/min) - Array-based
    .max_rate = {5000.0f, 5000.0f, 2000.0f, 5000.0f},  // [X, Y, Z, A]
    
    // Acceleration (mm/sec^2) - Array-based
    .acceleration = {500.0f, 500.0f, 200.0f, 500.0f},  // [X, Y, Z, A]
    
    // Max travel (mm) - Array-based
    .max_travel = {300.0f, 300.0f, 100.0f, 0.0f},      // [X, Y, Z, A] (A=0 for rotary)
    
    // Spindle
    .spindle_max_rpm = 24000.0f,
    .spindle_min_rpm = 0.0f,
    
    // Homing
    .hard_limits_enable = 0,       // false as uint8 (disabled by default for safety)
    .homing_enable = 0x07,         // Bit mask: X=bit0, Y=bit1, Z=bit2, A=bit3 (default: XYZ enabled)
    .homing_dir_mask = 0,
    .padding2 = 0,
    .homing_feed_rate = 100.0f,
    .homing_seek_rate = 500.0f,
    .homing_debounce = 25,
    .homing_pull_off = 2.0f,
    
    // Junction deviation for smooth cornering  
    .junction_deviation = 0.01f,   // GRBL default: 0.01mm
    
    // Work coordinate systems (G54-G59) - Array-based, all initialized to zero
    .wcs_g54 = {0.0f, 0.0f, 0.0f},  // [X, Y, Z]
    .wcs_g55 = {0.0f, 0.0f, 0.0f},
    .wcs_g56 = {0.0f, 0.0f, 0.0f},
    .wcs_g57 = {0.0f, 0.0f, 0.0f},
    .wcs_g58 = {0.0f, 0.0f, 0.0f},
    .wcs_g59 = {0.0f, 0.0f, 0.0f},
    
    // G92 coordinate offset - Array-based, initialized to zero
    .g92_offset = {0.0f, 0.0f, 0.0f},  // [X, Y, Z]
    
    // Tool length offset - initialized to zero
    .tool_length_offset = 0.0f,
    
    // Arc configuration ($12-$13)
    .mm_per_arc_segment = 0.5f,    // $12 - Increased from 0.1mm - creates larger segments with reliable step counts
    .arc_tolerance = 0.002f,       // $13 - GRBL v1.1 default: 0.002mm (2 microns) for radius error compensation
    
    .checksum = 0  // Will be calculated
};

/* Calculate CRC32 checksum for settings validation */
uint32_t SETTINGS_CalculateCRC32(const CNC_Settings* settings)
{
    // Simple CRC32 implementation
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t* data = (const uint8_t*)settings;
    size_t length = sizeof(CNC_Settings) - sizeof(uint32_t); // Exclude checksum field
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    
    return ~crc;
}

/* Initialize settings module */
void SETTINGS_Initialize(void)
{
    // ✅ TESTING: Skip NVM wait - may be hanging after soft reset
    // NVM_IsBusy() might never return false if flash controller is stuck
    // Just register the callback and load defaults
    
    // ✅ CRITICAL: Register NVM callback once during initialization (Harmony pattern)
    NVM_CallbackRegister(eventHandler, (uintptr_t)NULL);
    
    // ✅ SIMPLIFIED: Just use defaults, don't read flash during boot
    // Flash read happens later in APP_LOAD_SETTINGS after all peripherals ready
    SETTINGS_RestoreDefaults(&current_settings);
}

/* Load settings from NVM flash - EXACT Microchip pattern */
bool SETTINGS_LoadFromFlash(CNC_Settings* settings)
{
    if (!settings) return false;
    
    // Microchip pattern: Direct read into cache-aligned buffer
    NVM_Read(writeData, sizeof(writeData), APP_FLASH_ADDRESS);
    
    // Validate before copying to settings
    CNC_Settings temp_settings;
    memcpy(&temp_settings, writeData, sizeof(CNC_Settings));

    // Validate signature
    if (temp_settings.signature != SETTINGS_SIGNATURE) {
        return false;
    }
    
    // Validate version
    if (temp_settings.version != SETTINGS_VERSION) {
        return false;
    }
    
    // Validate checksum
    uint32_t calculated_crc = SETTINGS_CalculateCRC32(&temp_settings);
    if (calculated_crc != temp_settings.checksum) {
        return false;
    }
    
    // Copy if validation passed
    memcpy(settings, &temp_settings, sizeof(CNC_Settings));

    return true;
}

/* Save settings to NVM flash - EXACT Microchip pattern */
bool SETTINGS_SaveToFlash(const CNC_Settings* settings)
{
    if (!settings) return false;

    // Calculate checksum first
    CNC_Settings temp_settings;
    memcpy(&temp_settings, settings, sizeof(CNC_Settings));
    temp_settings.checksum = SETTINGS_CalculateCRC32(&temp_settings);
    
    // Populate cache-aligned buffer (Harmony pattern)
    memcpy(writeData, &temp_settings, sizeof(CNC_Settings));
    
    // Microchip pattern variables
    uint32_t address =  APP_FLASH_ADDRESS;//SETTINGS_NVM_ADDRESS;
    uint8_t *writePtr = (uint8_t *)writeData;
    uint32_t i = 0;
    
    // Wait for NVM ready before starting
    while(NVM_IsBusy() == true);
    
    /* Erase the Page */
    NVM_PageErase(address);
    
    while(xferDone == false);
    
    xferDone = false;

    for (i = 0; i < sizeof(CNC_Settings); i+= NVM_FLASH_ROWSIZE)
    {
        /* Program a row of data */
        NVM_RowWrite((uint32_t *)writePtr, address);

        while(xferDone == false);

        xferDone = false;

        writePtr += NVM_FLASH_ROWSIZE;
        address  += NVM_FLASH_ROWSIZE;
    }
    
    return true;
}

/* Restore default settings */
void SETTINGS_RestoreDefaults(CNC_Settings* settings)
{
    if (!settings) return;
    
    memcpy(settings, &default_settings, sizeof(CNC_Settings));
    settings->checksum = SETTINGS_CalculateCRC32(settings);
}

/* Set a settings value by parameter number */
bool SETTINGS_SetValue(CNC_Settings* settings, uint32_t parameter, float value)
{
    if (!settings) return false;
    
    switch (parameter) {
        // Step configuration
        case 0: settings->step_pulse_time = (uint32_t)value; break;
        case 1: settings->step_idle_delay = (uint32_t)value; break;
        case 2: settings->step_pulse_invert = (uint8_t)value; break;
        case 3: settings->step_direction_invert = (uint8_t)value; break;
        case 4: settings->step_enable_invert = (uint8_t)value; break;
        case 5: settings->limit_pins_invert = (uint8_t)value; break;
        
        // Junction deviation
        case 11: settings->junction_deviation = value; break;
        
        // Arc configuration
        case 12: settings->mm_per_arc_segment = value; break;
        case 13: settings->arc_tolerance = value; break;
        
        // Steps per mm ($100-$103) - Array-based
        case 100: settings->steps_per_mm[AXIS_X] = value; break;
        case 101: settings->steps_per_mm[AXIS_Y] = value; break;
        case 102: settings->steps_per_mm[AXIS_Z] = value; break;
        case 103: settings->steps_per_mm[AXIS_A] = value; break;
        
        // Max rates ($110-$113) - Array-based
        case 110: settings->max_rate[AXIS_X] = value; break;
        case 111: settings->max_rate[AXIS_Y] = value; break;
        case 112: settings->max_rate[AXIS_Z] = value; break;
        case 113: settings->max_rate[AXIS_A] = value; break;
        
        // Acceleration ($120-$123) - Array-based
        case 120: settings->acceleration[AXIS_X] = value; break;
        case 121: settings->acceleration[AXIS_Y] = value; break;
        case 122: settings->acceleration[AXIS_Z] = value; break;
        case 123: settings->acceleration[AXIS_A] = value; break;
        
        // Max travel ($130-$132) - Array-based
        case 130: settings->max_travel[AXIS_X] = value; break;
        case 131: settings->max_travel[AXIS_Y] = value; break;
        case 132: settings->max_travel[AXIS_Z] = value; break;
        
        // Spindle
        case 30: settings->spindle_max_rpm = value; break;
        case 31: settings->spindle_min_rpm = value; break;
        
        // Homing & Limits
        case 21: settings->hard_limits_enable = (uint8_t)value; break;
        case 22: settings->homing_enable = (uint8_t)value; break;
        case 23: settings->homing_dir_mask = (uint8_t)value; break;
        case 24: settings->homing_feed_rate = value; break;
        case 25: settings->homing_seek_rate = value; break;
        case 26: settings->homing_debounce = (uint32_t)value; break;
        case 27: settings->homing_pull_off = value; break;
        
        default:
            return false;  // Invalid parameter
    }
    
    return true;
}

/* Get a settings value by parameter number */
float SETTINGS_GetValue(const CNC_Settings* settings, uint32_t parameter)
{
    if (!settings) return 0.0f;
    
    switch (parameter) {
        case 0: return (float)settings->step_pulse_time;
        case 1: return (float)settings->step_idle_delay;
        case 2: return (float)settings->step_pulse_invert;
        case 3: return (float)settings->step_direction_invert;
        case 4: return (float)settings->step_enable_invert;
        case 5: return (float)settings->limit_pins_invert;
        
        case 11: return settings->junction_deviation;
        case 12: return settings->mm_per_arc_segment;
        case 13: return settings->arc_tolerance;
        // Steps per mm ($100-$103) - Array-based
        case 100: return settings->steps_per_mm[AXIS_X];
        case 101: return settings->steps_per_mm[AXIS_Y];
        case 102: return settings->steps_per_mm[AXIS_Z];
        case 103: return settings->steps_per_mm[AXIS_A];
        
        // Max rates ($110-$113) - Array-based
        case 110: return settings->max_rate[AXIS_X];
        case 111: return settings->max_rate[AXIS_Y];
        case 112: return settings->max_rate[AXIS_Z];
        case 113: return settings->max_rate[AXIS_A];
        
        // Acceleration ($120-$123) - Array-based
        case 120: return settings->acceleration[AXIS_X];
        case 121: return settings->acceleration[AXIS_Y];
        case 122: return settings->acceleration[AXIS_Z];
        case 123: return settings->acceleration[AXIS_A];
        
        // Max travel ($130-$132) - Array-based
        case 130: return settings->max_travel[AXIS_X];
        case 131: return settings->max_travel[AXIS_Y];
        case 132: return settings->max_travel[AXIS_Z];
        
        case 30: return settings->spindle_max_rpm;
        case 31: return settings->spindle_min_rpm;
        
        case 21: return (float)settings->hard_limits_enable;
        case 22: return (float)settings->homing_enable;
        case 23: return (float)settings->homing_dir_mask;
        case 24: return settings->homing_feed_rate;
        case 25: return settings->homing_seek_rate;
        case 26: return (float)settings->homing_debounce;
        case 27: return settings->homing_pull_off;
        
        default:
            return 0.0f;
    }
}

/* Print all settings (GRBL $$ command) */
void SETTINGS_PrintAll(const CNC_Settings* settings)
{
    if (!settings) return;
    
    // ✅ Buffer entire settings response for reliable transmission
    // Non-blocking UART_Printf() drops messages when TX buffer full
    static char settings_buffer[2048];  // Large enough for all 30 settings + formatting
    int len = 0;
    
    // Format: $<param>=<value> (lowercase 'ok' per GRBL protocol)
    len += sprintf(&settings_buffer[len], "$0=%u\r\n", (unsigned int)settings->step_pulse_time);
    len += sprintf(&settings_buffer[len], "$1=%u\r\n", (unsigned int)settings->step_idle_delay);
    len += sprintf(&settings_buffer[len], "$2=%u\r\n", settings->step_pulse_invert);
    len += sprintf(&settings_buffer[len], "$3=%u\r\n", settings->step_direction_invert);
    len += sprintf(&settings_buffer[len], "$4=%u\r\n", settings->step_enable_invert);
    len += sprintf(&settings_buffer[len], "$5=%u\r\n", settings->limit_pins_invert);
    len += sprintf(&settings_buffer[len], "$11=%.3f\r\n", settings->junction_deviation);
    len += sprintf(&settings_buffer[len], "$12=%.3f\r\n", settings->mm_per_arc_segment);
    len += sprintf(&settings_buffer[len], "$13=%.3f\r\n", settings->arc_tolerance);
        
    len += sprintf(&settings_buffer[len], "$21=%u\r\n", settings->hard_limits_enable);
    len += sprintf(&settings_buffer[len], "$22=%u\r\n", settings->homing_enable);
    len += sprintf(&settings_buffer[len], "$23=%u\r\n", settings->homing_dir_mask);
    len += sprintf(&settings_buffer[len], "$24=%.3f\r\n", settings->homing_feed_rate);
    len += sprintf(&settings_buffer[len], "$25=%.3f\r\n", settings->homing_seek_rate);
    len += sprintf(&settings_buffer[len], "$26=%u\r\n", (unsigned int)settings->homing_debounce);
    len += sprintf(&settings_buffer[len], "$27=%.3f\r\n", settings->homing_pull_off);

    len += sprintf(&settings_buffer[len], "$30=%.3f\r\n", settings->spindle_max_rpm);
    len += sprintf(&settings_buffer[len], "$31=%.3f\r\n", settings->spindle_min_rpm);
    
    // Steps per mm ($100-$103) - Array-based
    len += sprintf(&settings_buffer[len], "$100=%.3f\r\n", settings->steps_per_mm[AXIS_X]);
    len += sprintf(&settings_buffer[len], "$101=%.3f\r\n", settings->steps_per_mm[AXIS_Y]);
    len += sprintf(&settings_buffer[len], "$102=%.3f\r\n", settings->steps_per_mm[AXIS_Z]);
    len += sprintf(&settings_buffer[len], "$103=%.3f\r\n", settings->steps_per_mm[AXIS_A]);
    
    // Max rates ($110-$113) - Array-based
    len += sprintf(&settings_buffer[len], "$110=%.3f\r\n", settings->max_rate[AXIS_X]);
    len += sprintf(&settings_buffer[len], "$111=%.3f\r\n", settings->max_rate[AXIS_Y]);
    len += sprintf(&settings_buffer[len], "$112=%.3f\r\n", settings->max_rate[AXIS_Z]);
    len += sprintf(&settings_buffer[len], "$113=%.3f\r\n", settings->max_rate[AXIS_A]);
    
    // Acceleration ($120-$123) - Array-based
    len += sprintf(&settings_buffer[len], "$120=%.3f\r\n", settings->acceleration[AXIS_X]);
    len += sprintf(&settings_buffer[len], "$121=%.3f\r\n", settings->acceleration[AXIS_Y]);
    len += sprintf(&settings_buffer[len], "$122=%.3f\r\n", settings->acceleration[AXIS_Z]);
    len += sprintf(&settings_buffer[len], "$123=%.3f\r\n", settings->acceleration[AXIS_A]);
    
    // Max travel ($130-$132) - Array-based
    len += sprintf(&settings_buffer[len], "$130=%.3f\r\n", settings->max_travel[AXIS_X]);
    len += sprintf(&settings_buffer[len], "$131=%.3f\r\n", settings->max_travel[AXIS_Y]);
    len += sprintf(&settings_buffer[len], "$132=%.3f\r\n", settings->max_travel[AXIS_Z]);
    



    // ✅ GRBL protocol: blank line + ok
    len += sprintf(&settings_buffer[len], "\r\nok\r\n");
    
    // ✅ Write to PLIB TX ring buffer (1024 bytes)
    // Non-blocking - ISR handles transmission in background
    UART3_Write((uint8_t*)settings_buffer, len);
}

/* Print build info (GRBL $I command) */
void SETTINGS_PrintBuildInfo(void)
{
    // ✅ GRBL v1.1 format - UGS expects specific format!
    // Format: [VER:version] [OPT:options,blockbuffersize,rxbuffersize]
    const char build_info[] = "[VER:1.1h.20251102]\r\n[OPT:VHM,35,1024,4]\r\nok\r\n";
    
    // ✅ Write to PLIB TX ring buffer - ISR transmits in background
    UART3_Write((uint8_t*)build_info, sizeof(build_info) - 1);  // -1 excludes null terminator
}

/* Get pointer to current settings */
CNC_Settings* SETTINGS_GetCurrent(void)
{
    return &current_settings;
}

/* Get work coordinate system offset (G54=0, G55=1, ..., G59=5) */
bool SETTINGS_GetWorkCoordinateSystem(uint8_t wcs_number, float* x, float* y, float* z) {
    if (wcs_number > 5 || !x || !y || !z) return false;  // G54-G59 only
    
    // Array-based access - cleaner and scalable
    const float* wcs = NULL;
    switch (wcs_number) {
        case 0: wcs = current_settings.wcs_g54; break;  // G54
        case 1: wcs = current_settings.wcs_g55; break;  // G55
        case 2: wcs = current_settings.wcs_g56; break;  // G56
        case 3: wcs = current_settings.wcs_g57; break;  // G57
        case 4: wcs = current_settings.wcs_g58; break;  // G58
        case 5: wcs = current_settings.wcs_g59; break;  // G59
    }
    
    if (wcs) {
        *x = wcs[0];  // X
        *y = wcs[1];  // Y
        *z = wcs[2];  // Z
        return true;
    }
    return false;
}

/* Set work coordinate system offset and save to flash */
bool SETTINGS_SetWorkCoordinateSystem(uint8_t wcs_number, float x, float y, float z) {
    if (wcs_number > 5) return false;  // G54-G59 only
    
    // Array-based access - cleaner and scalable
    float* wcs = NULL;
    switch (wcs_number) {
        case 0: wcs = current_settings.wcs_g54; break;  // G54
        case 1: wcs = current_settings.wcs_g55; break;  // G55
        case 2: wcs = current_settings.wcs_g56; break;  // G56
        case 3: wcs = current_settings.wcs_g57; break;  // G57
        case 4: wcs = current_settings.wcs_g58; break;  // G58
        case 5: wcs = current_settings.wcs_g59; break;  // G59
    }
    
    if (wcs) {
        wcs[0] = x;  // X
        wcs[1] = y;  // Y
        wcs[2] = z;  // Z
        return SETTINGS_SaveToFlash(&current_settings);  // Save to flash immediately
    }
    return false;
}

/* Get G92 coordinate offset */
void SETTINGS_GetG92Offset(float* x, float* y, float* z) {
    if (x) *x = current_settings.g92_offset[0];  // X
    if (y) *y = current_settings.g92_offset[1];  // Y
    if (z) *z = current_settings.g92_offset[2];  // Z
}

/* Set G92 coordinate offset */
void SETTINGS_SetG92Offset(float x, float y, float z) {
    current_settings.g92_offset[0] = x;  // X
    current_settings.g92_offset[1] = y;  // Y
    current_settings.g92_offset[2] = z;  // Z
    // Note: G92 is typically not saved to flash (temporary offset)
}

/* Get tool length offset */
float SETTINGS_GetToolLengthOffset(void) {
    return current_settings.tool_length_offset;
}

/* Set tool length offset and save to flash */
void SETTINGS_SetToolLengthOffset(float offset) {
    current_settings.tool_length_offset = offset;
    SETTINGS_SaveToFlash(&current_settings);  // Save TLO to flash
}
