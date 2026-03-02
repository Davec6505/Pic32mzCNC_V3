/*******************************************************************************
 * tmc5160.c - TMC5160 SPI Stepper Driver Implementation
 *
 * SPI Protocol (TMC5160):
 *   40-bit frame per transaction (5 bytes, MSB first)
 *   Byte 0:  bit7 = Write(1)/Read(0), bits[6:0] = register address
 *   Bytes 1-4: 32-bit data MSB first
 *   Received byte 0: SPI status (mirrors GSTAT reset/error/uv bits)
 *   Received bytes 1-4: data from PREVIOUS read request (double-transaction read)
 *
 * All SPI transactions are blocking-polled (SPI2_IsBusy) since we only call
 * from APP_CONFIG or rate-limited APP_IDLE — never from ISR context.
 ******************************************************************************/

#include "motion/tmc5160.h"

#ifdef HAS_TMC5160_AXIS

#include <string.h>
#include "common.h"
#include "utils/uart_utils.h"
#include "plib_spi2_master.h"
#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/coretimer/plib_coretimer.h"

// ===== CS PIN FUNCTION POINTER ARRAYS (LED/axis pattern) =====
typedef void (*CS_Func)(void);

static void cs_x_assert(void)    { SPI2_CS_X_Clear(); }
static void cs_x_deassert(void)  { SPI2_CS_X_Set(); }
static void cs_y_assert(void)    { SPI2_CS_Y_Clear(); }
static void cs_y_deassert(void)  { SPI2_CS_Y_Set(); }
static void cs_z_assert(void)    { SPI2_CS_Z_Clear(); }
static void cs_z_deassert(void)  { SPI2_CS_Z_Set(); }
static void cs_a_assert(void)    { SPI2_CS_A_Clear(); }
static void cs_a_deassert(void)  { SPI2_CS_A_Set(); }

static const CS_Func cs_assert[NUM_AXIS]   = { cs_x_assert,   cs_y_assert,   cs_z_assert,   cs_a_assert   };
static const CS_Func cs_deassert[NUM_AXIS] = { cs_x_deassert, cs_y_deassert, cs_z_deassert, cs_a_deassert };

// ===== DEFAULT AXIS CONFIGURATION =====
// Tune irun/ihold to your motor + driver Vref sense resistors.
// irun 0-31 maps linearly to 0-100% of the set Vref current.
// mode is overridden per-axis in TMC5160_Initialize() from AXIS_x_TMC_MODE.
static const TMC5160_AxisConfig g_default_cfg = {
    .irun        = 20,                       // ~63% run current
    .ihold       = 10,                       // ~31% hold current
    .iholddelay  = 6,                        // Ramp-down delay
    .mres        = TMC5160_MRES_16,          // 16 microsteps
    .mode        = TMC5160_MODE_STEALTHCHOP, // overridden per-axis at init
    .tpwm_thrs   = 500,                      // MIXED: crossover ~mid speed (tune per motor)
    .tcoolthrs   = 0,                        // COOLSTEP: 0 = active at all speeds (tune per motor)
};

// CHOPCONF for SpreadCycle: TOFF=3, TBL=2, HSTRT=5, HEND=3
// MRES is overlaid at write time from cfg->mres
#define CHOPCONF_BASE  0x000100C3UL

// PWMCONF: StealthChop2 defaults from TMC5160 datasheet section 22
#define PWMCONF_DEFAULT  0xC40C001EUL

// COOLCONF: CoolStep defaults
//   semin[3:0]=2  CoolStep activates when SG_RESULT < semin*32 (load high)
//   seup[6:5]=0   Increment by 1 current step
//   semax[11:8]=5 Deactivates when SG_RESULT > (semin+semax+1)*32 (light load)
//   sedn[14:13]=1 Decrement after 32 StallGuard measurements
//   seimin[15]=0  Minimum = 1/4 IRUN
// Requires per-motor tuning of TCOOLTHRS and semin/semax before enabling.
#define COOLCONF_DEFAULT  0x00002502UL

// ===== MODULE STATE =====
static TMC5160_Status g_status[NUM_AXIS];
static bool           g_initialized = false;

// ===== INTERNAL HELPERS =====

// Block until SPI2 finishes the current transfer (~5 bytes @ PBCLK/4 = fast)
static void spi_wait(void)
{
    while (SPI2_IsBusy()) { /* spin */ }
}

/*
 * Send one 40-bit TMC5160 frame to the selected axis.
 * rxbuf must point to a 5-byte buffer; receives the reply frame.
 * Handles CS assert/deassert and busy-wait.
 */
static bool spi_transfer(E_AXIS axis, uint8_t* txbuf, uint8_t* rxbuf)
{
    cs_assert[axis]();

    bool ok = SPI2_WriteRead(txbuf, 5, rxbuf, 5);

    if (ok) {
        spi_wait();
    }

    cs_deassert[axis]();
    return ok;
}

// Build a 5-byte write frame in buf
static void build_write_frame(uint8_t* buf, uint8_t reg, uint32_t data)
{
    buf[0] = (uint8_t)(reg | TMC5160_WRITE_FLAG);
    buf[1] = (uint8_t)((data >> 24) & 0xFF);
    buf[2] = (uint8_t)((data >> 16) & 0xFF);
    buf[3] = (uint8_t)((data >>  8) & 0xFF);
    buf[4] = (uint8_t)( data        & 0xFF);
}

// Build a 5-byte read frame in buf (data bytes are don't-care / 0)
static void build_read_frame(uint8_t* buf, uint8_t reg)
{
    buf[0] = (uint8_t)(reg & ~TMC5160_WRITE_FLAG);
    buf[1] = 0; buf[2] = 0; buf[3] = 0; buf[4] = 0;
}

// Extract 32-bit data from a received 5-byte frame (bytes 1-4)
static uint32_t extract_data(const uint8_t* rxbuf)
{
    return ((uint32_t)rxbuf[1] << 24)
         | ((uint32_t)rxbuf[2] << 16)
         | ((uint32_t)rxbuf[3] <<  8)
         |  (uint32_t)rxbuf[4];
}

// ===== LOW-LEVEL REGISTER ACCESS =====

bool TMC5160_WriteRegister(E_AXIS axis, uint8_t reg, uint32_t data)
{
    uint8_t tx[5], rx[5];
    build_write_frame(tx, reg, data);
    return spi_transfer(axis, tx, rx);
}

/*
 * TMC5160 read requires TWO transactions (SPI response is one frame behind):
 *   Transaction 1: Send read request  → response is stale (discard)
 *   Transaction 2: Send anything      → response contains the requested data
 * We re-send the same read request for transaction 2 (harmless).
 */
uint32_t TMC5160_ReadRegister(E_AXIS axis, uint8_t reg)
{
    uint8_t tx[5], rx[5];

    // Transaction 1: issue read request
    build_read_frame(tx, reg);
    spi_transfer(axis, tx, rx);

    // Small gap between CS cycles (TMC5160 needs >= 1µs)
    CORETIMER_DelayUs(2);

    // Transaction 2: clock out the data
    spi_transfer(axis, tx, rx);

    return extract_data(rx);
}

// ===== PER-AXIS CONFIGURATION =====

void TMC5160_ConfigAxis(E_AXIS axis, const TMC5160_AxisConfig* cfg)
{
    // 1. Clear GSTAT (reset/error flags)
    TMC5160_WriteRegister(axis, TMC5160_REG_GSTAT,
                          TMC5160_GSTAT_RESET | TMC5160_GSTAT_DRV_ERR | TMC5160_GSTAT_UV_CP);

    // 2. GCONF: StealthChop modes need en_pwm_mode=1; SpreadCycle needs 0
    uint32_t gconf = 0;
    if (cfg->mode == TMC5160_MODE_STEALTHCHOP ||
        cfg->mode == TMC5160_MODE_MIXED       ||
        cfg->mode == TMC5160_MODE_COOLSTEP) {
        gconf |= TMC5160_GCONF_EN_PWM_MODE;
    }
    TMC5160_WriteRegister(axis, TMC5160_REG_GCONF, gconf);

    // 3. IHOLD_IRUN: motor current
    uint32_t ihold_irun = ((uint32_t)(cfg->iholddelay & 0x0F) << 16)
                        | ((uint32_t)(cfg->irun        & 0x1F) <<  8)
                        | ((uint32_t)(cfg->ihold       & 0x1F));
    TMC5160_WriteRegister(axis, TMC5160_REG_IHOLD_IRUN, ihold_irun);

    // 4. TPOWERDOWN: hold current ramp-down (~160ms)
    TMC5160_WriteRegister(axis, TMC5160_REG_TPOWERDOWN, 10);

    // 5. TPWMTHRS: StealthChop→SpreadCycle crossover velocity
    //    STEALTHCHOP: 0 = always StealthChop (no crossover)
    //    SPREADCYCLE: 0 = irrelevant (en_pwm_mode=0)
    //    MIXED/COOLSTEP: crossover at cfg->tpwm_thrs
    uint32_t tpwm = (cfg->mode == TMC5160_MODE_MIXED ||
                     cfg->mode == TMC5160_MODE_COOLSTEP) ? cfg->tpwm_thrs : 0;
    TMC5160_WriteRegister(axis, TMC5160_REG_TPWMTHRS, tpwm);

    // 6. CHOPCONF: SpreadCycle config + microstep resolution
    //    Written for all modes (used as fallback at high speed in MIXED/COOLSTEP)
    uint32_t chopconf = CHOPCONF_BASE | ((uint32_t)(cfg->mres & 0x0F) << 24);
    TMC5160_WriteRegister(axis, TMC5160_REG_CHOPCONF, chopconf);

    // 7. PWMCONF: StealthChop tuning (not needed for pure SpreadCycle)
    if (cfg->mode != TMC5160_MODE_SPREADCYCLE) {
        TMC5160_WriteRegister(axis, TMC5160_REG_PWMCONF, PWMCONF_DEFAULT);
    }

    // 8. TCOOLTHRS + COOLCONF: CoolStep / StallGuard2 (COOLSTEP mode only)
    if (cfg->mode == TMC5160_MODE_COOLSTEP) {
        TMC5160_WriteRegister(axis, TMC5160_REG_TCOOLTHRS, cfg->tcoolthrs);
        TMC5160_WriteRegister(axis, TMC5160_REG_COOLCONF,  COOLCONF_DEFAULT);
    } else {
        TMC5160_WriteRegister(axis, TMC5160_REG_TCOOLTHRS, 0); // disable CoolStep
        TMC5160_WriteRegister(axis, TMC5160_REG_COOLCONF,  0); // disable CoolStep
    }

    const char* mode_str[] = { "?", "StealthChop", "SpreadCycle", "Mixed", "CoolStep" };
    uint8_t mode_idx = (cfg->mode >= 1 && cfg->mode <= 4) ? cfg->mode : 0;
    (void)mode_str; (void)mode_idx; // suppress unused-variable in Release (-Werror)
    DEBUG_PRINT_MOTION("[TMC5160] Axis %d: mode=%s irun=%d ihold=%d mres=%d tpwm=%lu\r\n",
                       (int)axis, mode_str[mode_idx],
                       cfg->irun, cfg->ihold, cfg->mres, (unsigned long)tpwm);
}

// ===== PUBLIC API =====

void TMC5160_Initialize(void)
{
    memset(g_status, 0, sizeof(g_status));

    // Deassert CS lines for all configured TMC5160 axes before SPI begins
#if (AXIS_X_DRIVER == DRIVER_TMC5160)
    cs_deassert[AXIS_X]();
#endif
#if (AXIS_Y_DRIVER == DRIVER_TMC5160)
    cs_deassert[AXIS_Y]();
#endif
#if (AXIS_Z_DRIVER == DRIVER_TMC5160)
    cs_deassert[AXIS_Z]();
#endif
#if (AXIS_A_DRIVER == DRIVER_TMC5160)
    cs_deassert[AXIS_A]();
#endif

    // Small settle time after power-on (TMC5160 needs ~1ms after ENN high → low)
    CORETIMER_DelayMs(2);

    // Configure only the axes assigned as TMC5160, applying per-axis mode
#if (AXIS_X_DRIVER == DRIVER_TMC5160)
    { TMC5160_AxisConfig cfg = g_default_cfg; cfg.mode = AXIS_X_TMC_MODE; TMC5160_ConfigAxis(AXIS_X, &cfg); CORETIMER_DelayUs(10); }
#endif
#if (AXIS_Y_DRIVER == DRIVER_TMC5160)
    { TMC5160_AxisConfig cfg = g_default_cfg; cfg.mode = AXIS_Y_TMC_MODE; TMC5160_ConfigAxis(AXIS_Y, &cfg); CORETIMER_DelayUs(10); }
#endif
#if (AXIS_Z_DRIVER == DRIVER_TMC5160)
    { TMC5160_AxisConfig cfg = g_default_cfg; cfg.mode = AXIS_Z_TMC_MODE; TMC5160_ConfigAxis(AXIS_Z, &cfg); CORETIMER_DelayUs(10); }
#endif
#if (AXIS_A_DRIVER == DRIVER_TMC5160)
    { TMC5160_AxisConfig cfg = g_default_cfg; cfg.mode = AXIS_A_TMC_MODE; TMC5160_ConfigAxis(AXIS_A, &cfg); CORETIMER_DelayUs(10); }
#endif

    g_initialized = true;
    UART_Printf("[TMC5160] Initialized (X=%s Y=%s Z=%s A=%s)\r\n",
                (AXIS_X_DRIVER == DRIVER_TMC5160) ? "TMC" : "DRV",
                (AXIS_Y_DRIVER == DRIVER_TMC5160) ? "TMC" : "DRV",
                (AXIS_Z_DRIVER == DRIVER_TMC5160) ? "TMC" : "DRV",
                (AXIS_A_DRIVER == DRIVER_TMC5160) ? "TMC" : "DRV");
}

/*
 * Poll DRVSTATUS for all axes and update g_status[].
 * Call rate-limited from APP_IDLE (e.g. every ~100ms).
 * Returns true if any fault is active.
 */
bool TMC5160_Tasks(void)
{
    if (!g_initialized) return false;

    bool fault = false;

    // Poll DRVSTATUS only for axes assigned as TMC5160
#define POLL_TMC_AXIS(ax) \
    { \
        uint32_t drv = TMC5160_ReadRegister(ax, TMC5160_REG_DRVSTATUS); \
        TMC5160_Status* s = &g_status[ax]; \
        s->drv_status    = drv; \
        s->overtemp      = (drv & TMC5160_DRVSTATUS_OT)    != 0; \
        s->overtemp_warn = (drv & TMC5160_DRVSTATUS_OTPW)  != 0; \
        s->short_gnd     = (drv & (TMC5160_DRVSTATUS_S2GA  | TMC5160_DRVSTATUS_S2GB))  != 0; \
        s->short_vs      = (drv & (TMC5160_DRVSTATUS_S2VSA | TMC5160_DRVSTATUS_S2VSB)) != 0; \
        s->stall         = (drv & TMC5160_DRVSTATUS_STALLGUARD) != 0; \
        s->sg_result     = (uint16_t)(drv & TMC5160_DRVSTATUS_SG_RESULT_MASK); \
        if (s->overtemp || s->short_gnd || s->short_vs) { \
            fault = true; \
            DEBUG_PRINT_MOTION("[TMC5160] FAULT axis %d: OT=%d S2G=%d S2VS=%d\r\n", \
                               (int)(ax), (int)s->overtemp, \
                               (int)s->short_gnd, (int)s->short_vs); \
        } \
    }
#if (AXIS_X_DRIVER == DRIVER_TMC5160)
    POLL_TMC_AXIS(AXIS_X)
#endif
#if (AXIS_Y_DRIVER == DRIVER_TMC5160)
    POLL_TMC_AXIS(AXIS_Y)
#endif
#if (AXIS_Z_DRIVER == DRIVER_TMC5160)
    POLL_TMC_AXIS(AXIS_Z)
#endif
#if (AXIS_A_DRIVER == DRIVER_TMC5160)
    POLL_TMC_AXIS(AXIS_A)
#endif
#undef POLL_TMC_AXIS

    return fault;
}

const TMC5160_Status* TMC5160_GetStatus(E_AXIS axis)
{
    return &g_status[axis];
}

void TMC5160_SetRunCurrent(E_AXIS axis, uint8_t irun)
{
    // Read current IHOLD_IRUN, update IRUN field, write back
    uint32_t reg = TMC5160_ReadRegister(axis, TMC5160_REG_IHOLD_IRUN);
    reg = (reg & ~0x00001F00UL) | ((uint32_t)(irun & 0x1F) << 8);
    TMC5160_WriteRegister(axis, TMC5160_REG_IHOLD_IRUN, reg);
}

void TMC5160_SetHoldCurrent(E_AXIS axis, uint8_t ihold)
{
    uint32_t reg = TMC5160_ReadRegister(axis, TMC5160_REG_IHOLD_IRUN);
    reg = (reg & ~0x0000001FUL) | (uint32_t)(ihold & 0x1F);
    TMC5160_WriteRegister(axis, TMC5160_REG_IHOLD_IRUN, reg);
}

/**
 * @brief Apply runtime settings from CNC_Settings to a single TMC5160 axis.
 *
 * Called by gcode_parser.c when any $200-$253 parameter is written via $n=v.
 * Silently ignores axes that are not configured as DRIVER_TMC5160.
 *
 * @param axis     Target axis (AXIS_X .. AXIS_A)
 * @param settings Pointer to current CNC_Settings (read-only)
 */
void TMC5160_ApplySettings(E_AXIS axis, const CNC_Settings* settings)
{
    if (!settings) return;

    // Only reconfigure axes that are actually wired as TMC5160
    if (!(TMC5160_AXIS_MASK & (1u << (uint8_t)axis))) return;

    // Build the per-axis config from persisted runtime fields
    TMC5160_AxisConfig cfg;
    cfg.irun       = settings->tmc_irun[axis];
    cfg.ihold      = settings->tmc_ihold[axis];
    cfg.iholddelay = g_default_cfg.iholddelay;   // Not yet a $-param; use compile default
    cfg.mres       = settings->tmc_mres[axis];
    cfg.mode       = settings->tmc_mode[axis];
    cfg.tpwm_thrs  = settings->tmc_tpwm_thrs[axis];
    cfg.tcoolthrs  = settings->tmc_tcoolthrs[axis];

    TMC5160_ConfigAxis(axis, &cfg);
}

#endif /* HAS_TMC5160_AXIS */
