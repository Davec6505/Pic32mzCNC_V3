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
#define READ_WRITE_SIZE         (NVM_FLASH_PAGESIZE)
#define BUFFER_SIZE             (READ_WRITE_SIZE / sizeof(uint32_t))

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
    
    // Steps per mm (typical for 1/8 microstepping, 200 steps/rev, 5mm pitch)
    .steps_per_mm_x = 40.0f,       // 200 * 8 / 5 / 8 = 40
    .steps_per_mm_y = 40.0f,
    .steps_per_mm_z = 40.0f,
    .steps_per_mm_a = 40.0f,
    
    // Max rates (mm/min)
    .max_rate_x = 5000.0f,
    .max_rate_y = 5000.0f,
    .max_rate_z = 2000.0f,
    .max_rate_a = 5000.0f,
    
    // Acceleration (mm/sec^2)
    .acceleration_x = 500.0f,
    .acceleration_y = 500.0f,
    .acceleration_z = 200.0f,
    .acceleration_a = 500.0f,
    
    // Max travel (mm)
    .max_travel_x = 300.0f,
    .max_travel_y = 300.0f,
    .max_travel_z = 100.0f,
    
    // Spindle
    .spindle_max_rpm = 24000.0f,
    .spindle_min_rpm = 0.0f,
    
    // Homing
    .homing_enable = 1,            // true as uint8
    .homing_dir_mask = 0,
    .padding2 = 0,
    .homing_feed_rate = 100.0f,
    .homing_seek_rate = 500.0f,
    .homing_debounce = 25,
    .homing_pull_off = 2.0f,
    
    // Arc configuration
    .mm_per_arc_segment = 0.1f,    // GRBL default: 0.1mm per arc segment
    
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
    // ✅ CRITICAL: Register NVM callback once during initialization (Harmony pattern)
    NVM_CallbackRegister(eventHandler, (uintptr_t)NULL);
    
    // ✅ SIMPLIFIED: Just use defaults, don't read flash during boot
    // User can read flash later with $ commands if needed
    SETTINGS_RestoreDefaults(&current_settings);
}

/* Load settings from NVM flash */
bool SETTINGS_LoadFromFlash(CNC_Settings* settings)
{
    if (!settings) return false;
    
    // ✅ PURE HARMONY PATTERN: Read into cache-aligned buffer
    NVM_Read(writeData, sizeof(writeData), SETTINGS_READ_ADDRESS);
    
    // ✅ CRITICAL: Validate BEFORE copying to settings (don't corrupt defaults!)
    CNC_Settings temp_settings;
    memcpy(&temp_settings, writeData, sizeof(CNC_Settings));
    
    // Validate signature first
    if (temp_settings.signature != SETTINGS_SIGNATURE) {
        return false;  // Flash empty or invalid - keep current settings (defaults)
    }
    
    // Validate checksum
    uint32_t calculated_crc = SETTINGS_CalculateCRC32(&temp_settings);
    if (calculated_crc != temp_settings.checksum) {
        return false;  // Checksum mismatch - keep current settings (defaults)
    }
    
    // ✅ Only copy if validation passed
    memcpy(settings, &temp_settings, sizeof(CNC_Settings));
    
    return true;
}

/* Save settings to NVM flash */
bool SETTINGS_SaveToFlash(const CNC_Settings* settings)
{
    if (!settings) return false;
    
    // ✅ CRITICAL: Calculate checksum first
    CNC_Settings temp_settings;
    memcpy(&temp_settings, settings, sizeof(CNC_Settings));
    temp_settings.checksum = SETTINGS_CalculateCRC32(&temp_settings);
    
    // ✅ CRITICAL: Populate cache-aligned buffer (Harmony pattern)
    memcpy(writeData, &temp_settings, sizeof(CNC_Settings));
    
    // ✅ Harmony pattern variables
    uint32_t address = SETTINGS_NVM_ADDRESS;
    uint8_t *writePtr = (uint8_t *)writeData;
    uint32_t i = 0;
    
    // ✅ CRITICAL: Address MUST be row-aligned for RowWrite
    if ((address & 0x7FF) != 0) {
        return false;  // Address not row-aligned!
    }
    
    // ✅ Wait for NVM ready (Harmony pattern)
    while(NVM_IsBusy() == true);
    
    // ✅ Erase the Page (Harmony pattern)
    NVM_PageErase(address);
    
    // ✅ Wait for erase complete using callback flag
    while(xferDone == false);
    
    xferDone = false;
    
    // ✅ Check for erase errors
    if (NVM_ErrorGet() != NVM_ERROR_NONE) {
        return false;
    }
    
    // ✅ Write data row-by-row (Harmony pattern)
    for (i = 0; i < sizeof(CNC_Settings); i+= NVM_FLASH_ROWSIZE)
    {
        // Program a row of data
        NVM_RowWrite((uint32_t *)writePtr, address);

        // Wait for write complete using callback flag
        while(xferDone == false);
        
        xferDone = false;
        
        // Check for write errors
        if (NVM_ErrorGet() != NVM_ERROR_NONE) {
            return false;
        }

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
        
        // Arc configuration
        case 12: settings->mm_per_arc_segment = value; break;
        
        // Steps per mm
        case 100: settings->steps_per_mm_x = value; break;
        case 101: settings->steps_per_mm_y = value; break;
        case 102: settings->steps_per_mm_z = value; break;
        case 103: settings->steps_per_mm_a = value; break;
        
        // Max rates
        case 110: settings->max_rate_x = value; break;
        case 111: settings->max_rate_y = value; break;
        case 112: settings->max_rate_z = value; break;
        case 113: settings->max_rate_a = value; break;
        
        // Acceleration
        case 120: settings->acceleration_x = value; break;
        case 121: settings->acceleration_y = value; break;
        case 122: settings->acceleration_z = value; break;
        case 123: settings->acceleration_a = value; break;
        
        // Max travel
        case 130: settings->max_travel_x = value; break;
        case 131: settings->max_travel_y = value; break;
        case 132: settings->max_travel_z = value; break;
        
        // Spindle
        case 30: settings->spindle_max_rpm = value; break;
        case 31: settings->spindle_min_rpm = value; break;
        
        // Homing
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
        
        case 12: return settings->mm_per_arc_segment;
        
        case 100: return settings->steps_per_mm_x;
        case 101: return settings->steps_per_mm_y;
        case 102: return settings->steps_per_mm_z;
        case 103: return settings->steps_per_mm_a;
        
        case 110: return settings->max_rate_x;
        case 111: return settings->max_rate_y;
        case 112: return settings->max_rate_z;
        case 113: return settings->max_rate_a;
        
        case 120: return settings->acceleration_x;
        case 121: return settings->acceleration_y;
        case 122: return settings->acceleration_z;
        case 123: return settings->acceleration_a;
        
        case 130: return settings->max_travel_x;
        case 131: return settings->max_travel_y;
        case 132: return settings->max_travel_z;
        
        case 30: return settings->spindle_max_rpm;
        case 31: return settings->spindle_min_rpm;
        
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
    static char settings_buffer[2048];  // Large enough for all 29 settings + formatting
    int len = 0;
    
    // Format: $<param>=<value> (lowercase 'ok' per GRBL protocol)
    len += sprintf(&settings_buffer[len], "$0=%u\r\n", (unsigned int)settings->step_pulse_time);
    len += sprintf(&settings_buffer[len], "$1=%u\r\n", (unsigned int)settings->step_idle_delay);
    len += sprintf(&settings_buffer[len], "$2=%u\r\n", settings->step_pulse_invert);
    len += sprintf(&settings_buffer[len], "$3=%u\r\n", settings->step_direction_invert);
    len += sprintf(&settings_buffer[len], "$4=%u\r\n", settings->step_enable_invert);
    len += sprintf(&settings_buffer[len], "$5=%u\r\n", settings->limit_pins_invert);
    len += sprintf(&settings_buffer[len], "$12=%.3f\r\n", settings->mm_per_arc_segment);
    
    len += sprintf(&settings_buffer[len], "$100=%.3f\r\n", settings->steps_per_mm_x);
    len += sprintf(&settings_buffer[len], "$101=%.3f\r\n", settings->steps_per_mm_y);
    len += sprintf(&settings_buffer[len], "$102=%.3f\r\n", settings->steps_per_mm_z);
    len += sprintf(&settings_buffer[len], "$103=%.3f\r\n", settings->steps_per_mm_a);
    
    len += sprintf(&settings_buffer[len], "$110=%.3f\r\n", settings->max_rate_x);
    len += sprintf(&settings_buffer[len], "$111=%.3f\r\n", settings->max_rate_y);
    len += sprintf(&settings_buffer[len], "$112=%.3f\r\n", settings->max_rate_z);
    len += sprintf(&settings_buffer[len], "$113=%.3f\r\n", settings->max_rate_a);
    
    len += sprintf(&settings_buffer[len], "$120=%.3f\r\n", settings->acceleration_x);
    len += sprintf(&settings_buffer[len], "$121=%.3f\r\n", settings->acceleration_y);
    len += sprintf(&settings_buffer[len], "$122=%.3f\r\n", settings->acceleration_z);
    len += sprintf(&settings_buffer[len], "$123=%.3f\r\n", settings->acceleration_a);
    
    len += sprintf(&settings_buffer[len], "$130=%.3f\r\n", settings->max_travel_x);
    len += sprintf(&settings_buffer[len], "$131=%.3f\r\n", settings->max_travel_y);
    len += sprintf(&settings_buffer[len], "$132=%.3f\r\n", settings->max_travel_z);
    
    len += sprintf(&settings_buffer[len], "$30=%.3f\r\n", settings->spindle_max_rpm);
    len += sprintf(&settings_buffer[len], "$31=%.3f\r\n", settings->spindle_min_rpm);
    
    len += sprintf(&settings_buffer[len], "$22=%u\r\n", settings->homing_enable);
    len += sprintf(&settings_buffer[len], "$23=%u\r\n", settings->homing_dir_mask);
    len += sprintf(&settings_buffer[len], "$24=%.3f\r\n", settings->homing_feed_rate);
    len += sprintf(&settings_buffer[len], "$25=%.3f\r\n", settings->homing_seek_rate);
    len += sprintf(&settings_buffer[len], "$26=%u\r\n", (unsigned int)settings->homing_debounce);
    len += sprintf(&settings_buffer[len], "$27=%.3f\r\n", settings->homing_pull_off);

    // ✅ GRBL protocol: blank line + ok
    len += sprintf(&settings_buffer[len], "\r\nok\r\n");
    
    // ✅ Write to PLIB TX ring buffer (1024 bytes)
    // Non-blocking - ISR handles transmission in background
    UART3_Write((uint8_t*)settings_buffer, len);
}

/* Print build info (GRBL $I command) */
void SETTINGS_PrintBuildInfo(void)
{
    // ✅ GRBL v1.1 exact format - MUST have space after colons!
    // Format: [VER: version] [OPT: options,blockbuffersize,rxbuffersize,axiscount]
    const char build_info[] = "[VER: 1.1h.20251102:]\r\n[OPT: VHM,35,1024,4]\r\nok\r\n";
    
    // ✅ Write to PLIB TX ring buffer - ISR transmits in background
    UART3_Write((uint8_t*)build_info, sizeof(build_info) - 1);  // -1 excludes null terminator
}

/* Get pointer to current settings */
CNC_Settings* SETTINGS_GetCurrent(void)
{
    return &current_settings;
}
