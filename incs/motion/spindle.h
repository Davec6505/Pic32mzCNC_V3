
#ifndef SPINDLE_H
#define SPINDLE_H

#include <stdint.h>
#include <stdbool.h>
#include "../data_structures.h"

// Spindle control functions
void SPINDLE_Initialize(APP_DATA* appData);
void SPINDLE_SetSpeed(APP_DATA* appData, uint32_t rpm);
void SPINDLE_Start(APP_DATA* appData);
void SPINDLE_Stop(APP_DATA* appData);
uint32_t SPINDLE_GetCurrentRPM(APP_DATA* appData);
bool SPINDLE_IsRunning(APP_DATA* appData);

// PWM calculation helpers
uint16_t SPINDLE_RPMToPWMDuty(uint32_t rpm);
uint32_t SPINDLE_PWMDutyToRPM(uint16_t duty);

#endif // SPINDLE_H