/*******************************************************************************
 * tmc5160.h - TMC5160 SPI Stepper Driver
 *
 * Architecture:
 *   - SPI2 used exclusively for startup config and runtime diagnostics
 *   - Step/Dir/EnXYZA GPIO motion system is UNCHANGED (stepper.c untouched)
 *   - All SPI transactions in main-loop context only (never in ISR)
 *   - One CS pin per axis: SPI2_CS_X/Y/Z/A
 *
 * Usage:
 *   APP_CONFIG  -> TMC5160_Initialize()           (configure all 4 drivers)
 *   APP_IDLE    -> TMC5160_Tasks() rate-limited    (poll DRVSTATUS diagnostics)
 ******************************************************************************/

#ifndef TMC5160_H
#define TMC5160_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"           // For STEPPER_DRIVER_TMC5160 define
#include "data_structures.h"  // For E_AXIS

#ifdef STEPPER_DRIVER_TMC5160

// ===== TMC5160 REGISTER MAP =====
#define TMC5160_REG_GCONF       0x00  // Global configuration
#define TMC5160_REG_GSTAT       0x01  // Global status (R+C)
#define TMC5160_REG_IOIN        0x04  // Inputs
#define TMC5160_REG_IHOLD_IRUN  0x10  // Current control
#define TMC5160_REG_TPOWERDOWN  0x11  // Power-down delay after standstill
#define TMC5160_REG_TPWMTHRS    0x13  // StealthChop upper velocity threshold
#define TMC5160_REG_CHOPCONF    0x6C  // Chopper configuration + microstep
#define TMC5160_REG_DRVSTATUS   0x6F  // Driver status flags (read-only)
#define TMC5160_REG_PWMCONF     0x70  // StealthChop PWM configuration
#define TMC5160_REG_PWM_SCALE   0x71  // StealthChop amplitude (read-only)

// ===== SPI FRAME FLAGS =====
#define TMC5160_WRITE_FLAG      0x80  // OR with register address for write
#define TMC5160_READ_FLAG       0x00  // Register address as-is for read

// ===== GCONF BITS =====
#define TMC5160_GCONF_EN_PWM_MODE   (1UL << 2)  // StealthChop enable

// ===== GSTAT BITS (write 1 to clear) =====
#define TMC5160_GSTAT_RESET     (1UL << 0)  // Driver has been reset
#define TMC5160_GSTAT_DRV_ERR   (1UL << 1)  // Driver error (overtemp/short)
#define TMC5160_GSTAT_UV_CP     (1UL << 2)  // Charge pump undervoltage

// ===== CHOPCONF MICROSTEP RESOLUTION =====
// MRES field bits[27:24]: 0=256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2, 8=1
#define TMC5160_MRES_256        0x00
#define TMC5160_MRES_128        0x01
#define TMC5160_MRES_64         0x02
#define TMC5160_MRES_32         0x03
#define TMC5160_MRES_16         0x04
#define TMC5160_MRES_8          0x05
#define TMC5160_MRES_4          0x06
#define TMC5160_MRES_2          0x07
#define TMC5160_MRES_1          0x08  // Full step

// ===== DRVSTATUS BITS =====
#define TMC5160_DRVSTATUS_OT        (1UL << 25)  // Overtemperature shutdown
#define TMC5160_DRVSTATUS_OTPW      (1UL << 26)  // Overtemperature pre-warning
#define TMC5160_DRVSTATUS_S2GA      (1UL << 27)  // Short to GND phase A
#define TMC5160_DRVSTATUS_S2GB      (1UL << 28)  // Short to GND phase B
#define TMC5160_DRVSTATUS_S2VSA     (1UL << 29)  // Short to VS phase A
#define TMC5160_DRVSTATUS_S2VSB     (1UL << 30)  // Short to VS phase B
#define TMC5160_DRVSTATUS_STALLGUARD (1UL << 24) // StallGuard2 stall indicator
// bits 9:0 = SG_RESULT (StallGuard2 load measurement)
#define TMC5160_DRVSTATUS_SG_RESULT_MASK  0x3FFUL

// ===== DRIVER CONFIGURATION =====
// Adjust these per your hardware (motor current, microstepping)
typedef struct {
    uint8_t  irun;        // Run current  0-31 (maps to ~32 steps of Vref)
    uint8_t  ihold;       // Hold current 0-31 (typically 50% of irun)
    uint8_t  iholddelay;  // Hold current ramp-down delay (0-15, x2^18 clk)
    uint8_t  mres;        // Microstep resolution (TMC5160_MRES_xxx)
    bool     stealthchop; // Enable StealthChop (quiet) mode
    uint32_t tpwm_thrs;   // StealthChop speed threshold (0=always on)
} TMC5160_AxisConfig;

// ===== DIAGNOSTIC STATUS =====
typedef struct {
    uint32_t drv_status;   // Raw DRVSTATUS register
    bool     overtemp;     // OT bit
    bool     overtemp_warn;// OTPW bit
    bool     short_gnd;    // S2GA or S2GB
    bool     short_vs;     // S2VSA or S2VSB
    bool     stall;        // StallGuard stall
    uint16_t sg_result;    // StallGuard load value
} TMC5160_Status;

// ===== PUBLIC API =====

// Call once from APP_CONFIG after SPI2_Initialize()
void TMC5160_Initialize(void);

// Call rate-limited from APP_IDLE (~10Hz) to poll DRVSTATUS on all axes
// Returns true if any fault detected (caller can trigger alarm)
bool TMC5160_Tasks(void);

// Low-level register access (blocking poll on SPI2_IsBusy)
bool     TMC5160_WriteRegister(E_AXIS axis, uint8_t reg, uint32_t data);
uint32_t TMC5160_ReadRegister(E_AXIS axis, uint8_t reg);

// Per-axis configuration (called from TMC5160_Initialize)
void TMC5160_ConfigAxis(E_AXIS axis, const TMC5160_AxisConfig* cfg);

// Get last polled status for an axis
const TMC5160_Status* TMC5160_GetStatus(E_AXIS axis);

// Current control (runtime adjustment)
void TMC5160_SetRunCurrent(E_AXIS axis, uint8_t irun);
void TMC5160_SetHoldCurrent(E_AXIS axis, uint8_t ihold);

#endif /* STEPPER_DRIVER_TMC5160 */
#endif /* TMC5160_H */
