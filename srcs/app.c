
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
#include "settings.h"

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

    // Initialize all single instances in appData (centralized pattern)
    memset((void*)&appData.motionQueue, 0, sizeof(appData.motionQueue));
    appData.motionQueueHead = 0;
    appData.motionQueueTail = 0;
    appData.motionQueueCount = 0;
    
    // Initialize G-code command queue
    memset((void*)&appData.gcodeCommandQueue, 0, sizeof(GCODE_CommandQueue));
    appData.gcodeCommandQueue.head = 0;
    appData.gcodeCommandQueue.tail = 0;
    appData.gcodeCommandQueue.count = 0;
    
    // ✅ Initialize nested motion queue info for flow control
    appData.gcodeCommandQueue.motionQueueCount = 0;
    appData.gcodeCommandQueue.maxMotionSegments = MAX_MOTION_SEGMENTS;


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
            KINEMATICS_Initialize(); // Initialize work coordinates
            

            appData.state = APP_LOAD_SETTINGS;
            break;
        }
        
        case APP_LOAD_SETTINGS:
        {
            // ✅ Delay flash read until after all peripherals initialized
            // Try to load from flash - if invalid signature/CRC, use defaults (already loaded)
            if (SETTINGS_LoadFromFlash(SETTINGS_GetCurrent())) {
                // Flash settings loaded successfully
            } else {
                // Flash empty or invalid - defaults already loaded in SETTINGS_Initialize()
            }
            
            appData.state = APP_IDLE;
            break;
        }
        
        case APP_IDLE:
        {
            // ✅ Sync motion queue count into G-code queue for flow control
            appData.gcodeCommandQueue.motionQueueCount = appData.motionQueueCount;

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
        
        // Process G-code events through clean interface (respects APP_DATA abstraction)
        GCODE_Event event;
        while (GCODE_GetNextEvent(&appData.gcodeCommandQueue, &event)) {
            switch (event.type) {
                case GCODE_EVENT_LINEAR_MOVE:
                    // Convert to motion segment using kinematics
                    // MotionSegment segment;
                    // KINEMATICS_LinearMove(currentPos, targetPos, event.data.linearMove.feedrate, &segment);
                    // Add to motion queue through YOUR abstraction layer
                    // TODO: Implement motion segment queueing
                    break;
                    
                case GCODE_EVENT_SPINDLE_ON:
                    // Handle spindle control
                    // SPINDLE_SetSpeed(event.data.spindle.rpm);
                    break;
                    
                case GCODE_EVENT_SET_ABSOLUTE:
                    // Set coordinate mode
                    break;
                    
                default:
                    // Handle other events or ignore
                    break;
            }
        }

} //End of APP_Tasks
