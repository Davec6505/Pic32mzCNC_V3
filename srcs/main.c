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


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
  static volatile uint32_t app_indicator = 0;
    /* Initialize all modules */
    SYS_Initialize ( NULL );

    /* Initialize application */
    APP_Initialize();

    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        APP_Tasks();

        /* Toggle LED1 every 1 second for app status indication at the end of the loop
         * This indicates that the application is running.
         */
        if(++app_indicator >= 1000000){
            app_indicator = 0;
            LED1_Toggle();
        }
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

