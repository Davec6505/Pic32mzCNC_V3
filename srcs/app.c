
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
#include <math.h>  // For fabsf in limit checks
#include "app.h"
#include "settings.h"
#include "kinematics.h"  // For KINEMATICS_LinearMove and CoordinatePoint
#include "motion_utils.h"  // For hard limit checking

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
    
    // ✅ Initialize G-code command queue
    memset((void*)&appData.gcodeCommandQueue, 0, sizeof(GCODE_CommandQueue));
    appData.gcodeCommandQueue.head = 0;
    appData.gcodeCommandQueue.tail = 0;
    appData.gcodeCommandQueue.count = 0;
    
    // ✅ Initialize nested motion queue info for flow control
    appData.gcodeCommandQueue.motionQueueCount = 0;
    appData.gcodeCommandQueue.maxMotionSegments = MAX_MOTION_SEGMENTS;
    
    // ✅ Initialize current position (work coordinates) to origin
    appData.currentX = 0.0f;
    appData.currentY = 0.0f;
    appData.currentZ = 0.0f;
    appData.currentA = 0.0f;
    
    // ✅ Initialize modal state (GRBL defaults)
    appData.modalFeedrate = 0.0f;      // No default feedrate (must be set explicitly)
    appData.modalSpindleRPM = 0;       // Spindle off
    appData.modalToolNumber = 0;       // No tool selected
    appData.absoluteMode = true;       // G90 absolute mode (GRBL default)
    
    // ✅ Initialize alarm state
    appData.alarmCode = 0;             // No alarm


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

    // ✅ Following Harmony pattern: Protocol handlers run INSIDE states
    // This ensures they only execute when subsystems are ready

    switch ( appData.state )
    {
        case APP_CONFIG:
        {
            // Initialize subsystems ONCE during configuration
            STEPPER_Initialize();                           // Hardware timer setup
            MOTION_Initialize();                           // Motion planning initialization  
            KINEMATICS_Initialize();                        // Initialize work coordinates
            
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
            
            appData.state = APP_GCODE_INIT;
            break;
        }
        
        case APP_GCODE_INIT:
        {
            // ✅ Initialize UART and G-code parser after subsystems ready
            GCODE_USART_Initialize(5);
            
            appData.state = APP_IDLE;
            break;
        }
        
        case APP_IDLE:
        {
            // ✅ STEP 1: Poll serial protocol handler (Harmony pattern)
            // Only runs when UART is ready (after APP_GCODE_INIT)
            GCODE_Tasks(&appData.gcodeCommandQueue);
            
            // ✅ STEP 2: Sync motion queue status for flow control
            appData.gcodeCommandQueue.motionQueueCount = appData.motionQueueCount;
            appData.gcodeCommandQueue.maxMotionSegments = MAX_MOTION_SEGMENTS;

            // ✅ STEP 3: Motion controller processes queued segments
            MOTION_Tasks(appData.motionQueue, &appData.motionQueueHead, 
                        &appData.motionQueueTail, &appData.motionQueueCount);

            // ✅ STEP 4: HARD LIMIT CHECK (GRBL safety feature)
            // Check limit switches with inversion mask from settings
            GRBL_Settings* settings = SETTINGS_GetCurrent();
            if(MOTION_UTILS_CheckHardLimits(settings->limit_pins_invert)) {
                // ⚠️ HARD LIMIT TRIGGERED - Emergency stop!
                appData.alarmCode = 1;  // Alarm code 1 = hard limit
                appData.state = APP_ALARM;
                break;  // Immediate transition to alarm state
            }

            // ✅ STEP 5: Process ONE G-code event per iteration (non-blocking!)
            GCODE_Event event;
            if (GCODE_GetNextEvent(&appData.gcodeCommandQueue, &event)) {
                switch (event.type) {
                    case GCODE_EVENT_LINEAR_MOVE:
                    {
                        // Check if motion queue has space before processing
                        if(appData.motionQueueCount < MAX_MOTION_SEGMENTS) {
                            // Build start and end coordinates
                            CoordinatePoint start = {appData.currentX, appData.currentY, appData.currentZ, appData.currentA};
                            CoordinatePoint end;
                            
                            if(appData.absoluteMode) {
                                // G90 absolute mode - use coordinates directly
                                end.x = event.data.linearMove.x;
                                end.y = event.data.linearMove.y;
                                end.z = event.data.linearMove.z;
                                end.a = event.data.linearMove.a;
                            } else {
                                // G91 relative mode - add to current position
                                end.x = appData.currentX + event.data.linearMove.x;
                                end.y = appData.currentY + event.data.linearMove.y;
                                end.z = appData.currentZ + event.data.linearMove.z;
                                end.a = appData.currentA + event.data.linearMove.a;
                            }
                            
                            // ✅ SOFT LIMIT CHECK (non-blocking GRBL implementation)
                            // Check target position against max_travel settings
                            GRBL_Settings* settings = SETTINGS_GetCurrent();
                            bool limit_violation = false;
                            
                            // Machine coordinates are negative (GRBL convention: 0,0,0 is max travel position)
                            // Work coordinates typically positive, but we check absolute values
                            if(fabsf(end.x) > settings->max_travel_x) {
                                limit_violation = true;
                            }
                            if(fabsf(end.y) > settings->max_travel_y) {
                                limit_violation = true;
                            }
                            if(fabsf(end.z) > settings->max_travel_z) {
                                limit_violation = true;
                            }
                            
                            // If limit violated, trigger alarm (GRBL behavior)
                            if(limit_violation) {
                                appData.alarmCode = 2;  // Soft limit alarm
                                appData.state = APP_ALARM;
                                break;  // Exit event processing immediately
                            }
                            
                            // Use feedrate from event, or modal feedrate if not specified
                            float feedrate = event.data.linearMove.feedrate;
                            if(feedrate == 0.0f) {
                                feedrate = appData.modalFeedrate;
                            } else {
                                // Update modal feedrate
                                appData.modalFeedrate = feedrate;
                            }
                            
                            // Get next queue slot
                            MotionSegment* segment = &appData.motionQueue[appData.motionQueueHead];
                            
                            // Convert to motion segment using kinematics (reuse existing function)
                            KINEMATICS_LinearMove(start, end, feedrate, segment);
                            
                            // Add to motion queue
                            appData.motionQueueHead = (appData.motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
                            appData.motionQueueCount++;
                            
                            // Update current position (work coordinates)
                            appData.currentX = end.x;
                            appData.currentY = end.y;
                            appData.currentZ = end.z;
                            appData.currentA = end.a;
                        }
                        break;
                    }
                        
                    case GCODE_EVENT_SPINDLE_ON:
                        // Update modal spindle RPM
                        appData.modalSpindleRPM = event.data.spindle.rpm;
                        // TODO: Implement spindle control hardware interface
                        break;
                        
                    case GCODE_EVENT_SPINDLE_OFF:
                        appData.modalSpindleRPM = 0;
                        // TODO: Implement spindle control hardware interface
                        break;
                        
                    case GCODE_EVENT_SET_ABSOLUTE:
                        // G90 - absolute positioning mode
                        appData.absoluteMode = true;
                        break;
                        
                    case GCODE_EVENT_SET_RELATIVE:
                        // G91 - relative positioning mode
                        appData.absoluteMode = false;
                        break;
                        
                    case GCODE_EVENT_SET_FEEDRATE:
                        // Standalone F command - update modal feedrate (GRBL v1.1 compliant)
                        appData.modalFeedrate = event.data.setFeedrate.feedrate;
                        break;
                        
                    case GCODE_EVENT_SET_SPINDLE_SPEED:
                        // Standalone S command - update modal spindle speed (GRBL v1.1 compliant)
                        appData.modalSpindleRPM = event.data.setSpindleSpeed.rpm;
                        break;
                        
                    case GCODE_EVENT_SET_TOOL:
                        // Tool change command (T command)
                        appData.modalToolNumber = event.data.setTool.toolNumber;
                        // TODO: Implement tool change logic
                        break;
                        
                    case GCODE_EVENT_DWELL:
                        // G4 dwell command
                        // TODO: Implement non-blocking dwell using CORETIMER
                        break;
                        
                    case GCODE_EVENT_ARC_MOVE:
                        // G2/G3 arc commands
                        // TODO: Implement arc interpolation via KINEMATICS_ArcMove
                        break;
                        
                    case GCODE_EVENT_COOLANT_ON:
                    case GCODE_EVENT_COOLANT_OFF:
                        // M7/M8/M9 coolant commands
                        // TODO: Implement coolant control
                        break;
                        
                    default:
                        // Unknown or unhandled event
                        break;
                }
            }
            // ✅ Only ONE event processed - returns quickly for LED toggle

            /* idle status LED */
             if(++idle_indicator >= 500000)
            {
                LED2_Toggle();
                idle_indicator = 0;
            }
            break;
        }

        case APP_ALARM:
        {
            // ⚠️ EMERGENCY STOP STATE - Hard limit or alarm triggered
            // System halted - requires soft reset (^X) to clear
            // TODO: Implement $X unlock command for alarm clear without position loss
            
            // Execute emergency stop ONCE when entering this state
            static bool alarm_handled = false;
            if (!alarm_handled) {
                // 1. Disable all stepper axes immediately (cut power + stop pulses)
                STEPPER_DisableAll();
                
                // 2. Clear motion queue (discard all pending moves)
                appData.motionQueueCount = 0;
                appData.motionQueueHead = 0;
                appData.motionQueueTail = 0;
                
                // 3. Clear G-code command queue
                appData.gcodeCommandQueue.count = 0;
                appData.gcodeCommandQueue.head = 0;
                appData.gcodeCommandQueue.tail = 0;
                
                // 4. Send GRBL alarm message to host
                switch(appData.alarmCode) {
                    case 1:
                        UART2_Write((uint8_t*)"ALARM:1 Hard limit triggered\r\n", 31);
                        break;
                    case 2:
                        UART2_Write((uint8_t*)"ALARM:2 Soft limit exceeded\r\n", 30);
                        break;
                    case 3:
                        UART2_Write((uint8_t*)"ALARM:3 Abort during cycle\r\n", 29);
                        break;
                    default:
                        UART2_Write((uint8_t*)"ALARM:9 Unknown alarm\r\n", 24);
                        break;
                }
                
                alarm_handled = true;
            }
            
            // Stay in alarm state until cleared
            // TODO: Implement unlock ($X) or soft reset (^X) commands
            // For now, must reset device to clear alarm
            
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


} //End of APP_Tasks
