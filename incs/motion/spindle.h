#ifndef SPINDLE_H
#define SPINDLE_H

#include <stdint.h>
#include <stdbool.h>

// Spindle control functions
void SPINDLE_Initialize(void);
void SPINDLE_SetSpeed(uint32_t rpm);
void SPINDLE_Start(void);
void SPINDLE_Stop(void);
uint32_t SPINDLE_GetCurrentRPM(void);
bool SPINDLE_IsRunning(void);

// PWM calculation helpers
uint16_t SPINDLE_RPMToPWMDuty(uint32_t rpm);
uint32_t SPINDLE_PWMDutyToRPM(uint16_t duty);

// Spindle speed override (GRBL real-time override bytes 0x99-0x9D)
// pct: percentage 10-200 (100 = nominal speed)
void SPINDLE_SetOverridePct(uint8_t pct);
uint8_t SPINDLE_GetOverridePct(void);

#endif // SPINDLE_H