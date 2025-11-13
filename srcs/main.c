/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <string.h>
#include <stdio.h>
#include "definitions.h"                // SYS function prototypes

#include "app.h"                // GCODE parser function prototypes
#include "settings.h"           // Settings module function prototypes


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
  /* Initialize all modules */
  SYS_Initialize ( NULL );

  // ✅ CRITICAL: Load settings from flash FIRST
  // This must happen before APP_Initialize() so settings are available
  // for any module that needs them during initialization
  SETTINGS_Initialize();

  /* Initialize application */
  APP_Initialize();

  // Use Core Timer for a stable 1 Hz LED1 heartbeat (independent of loop load)
  uint32_t hb_last = CORETIMER_CounterGet();
  const uint32_t HB_INTERVAL = 100000000U; // ~1s @ 100MHz Core Timer (200MHz CPU / 2)

  while ( true )
  {
    /* Maintain state machines of all polled MPLAB Harmony modules. */
    APP_Tasks();

    // LED1 heartbeat: slow blink when idle; during motion, ISR toggles LED1 rapidly
    uint32_t now_ticks = CORETIMER_CounterGet();
    if ((uint32_t)(now_ticks - hb_last) >= HB_INTERVAL) {
      hb_last = now_ticks;
      LED1_Toggle();
    }
  }

  /* Execution should not come here during normal operation */
  return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

