
/*
 * The application does the following:
 * 1. Sets up UART2 to communicate with the host.
 * 2. After the setup it polls for data in the ring buffer.
 * 3. If there is no error in any of the above steps then the application will
 *    go into Idle state.
 * 4. If there is an error then the application will go into Error state.
 * */

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <string.h>
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 * 1) Strip out gcode commands received from UART and add them to the command queue.
 *    place them in appData.gcodeCommandQueue for processing in APP_Tasks function.
 *    From there place them into look ahead buffer for execution calculations, like
 *    acceleration, jerk, junction deviation etc.
 * 2) 
 * 
*/



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Initialize the app state to wait for media attach. */
    appData.state = APP_CONFIG;

    memset((void*)&appData.gcodeCommandQueue, 0, sizeof(GCODE_CommandQueue));

}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
   /* Check the application's current state. */
   static uint32_t idle_indicator = 0;



    switch ( appData.state )
    {
        case APP_CONFIG:
        {
            GCODE_USART_Initialize(5);

            // Initialize subsystems ONCE during configuration
            STEPPER_Initialize();                           // Hardware timer setup
            MOTION_Initialize();                           // Motion planning initialization  
            KINEMATICS_Initialize(&appData.workCoordinates); // Initialize work coordinates
            

            appData.state = APP_IDLE;
            break;
        }
        case APP_IDLE:
        {

            // Only state machines that need continuous execution
            MOTION_Tasks(appData.motionQueue, &appData.motionQueueHead, 
                        &appData.motionQueueTail, &appData.motionQueueCount);


            /* idle status LED */
             if(++idle_indicator >= 500000)
            {
                LED2_Toggle();
                idle_indicator = 0;
            }
            break;
        }

        case APP_ERROR:
        {
            /* The application comes here when the demo has failed. */
            break;
        }
        default:
        {
            break;
        }
    }
        // Always call the GCODE task routine to process incoming GCODE commands
        GCODE_Tasks(&appData.gcodeCommandQueue);

} //End of APP_Tasks
