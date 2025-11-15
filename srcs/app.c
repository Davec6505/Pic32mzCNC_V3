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
#include <stdio.h>
#include <math.h>  // For fabsf in limit checks
#include "app.h"
#include "settings.h"
#include "kinematics.h"  // For KINEMATICS_LinearMove and CoordinatePoint
#include "motion/homing.h"  // For homing state machine
#include "motion/spindle.h"  // For spindle PWM control
#include "motion_utils.h"  // For hard limit checking
#include "config/default/peripheral/coretimer/plib_coretimer.h"  // For CORETIMER heartbeat counter
#include "utils/uart_utils.h"  // Non-blocking UART utilities
#include "utils/utils.h"       // For UTILS_InitAxisConfig
#include "motion.h"

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
    
  // ✅ Initialize flow control capacity (single source of truth is appData.motionQueueCount)
  appData.gcodeCommandQueue.maxMotionSegments = MAX_MOTION_SEGMENTS;
    
    // ✅ ARRAY-BASED: Initialize current position (work coordinates) to zero
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        appData.current[axis] = 0.0f;
    }
    
    // ✅ Initialize modal state (GRBL defaults)
    appData.modalFeedrate = 0.0f;      // No default feedrate (must be set explicitly)
    appData.modalSpindleRPM = 0;       // Spindle off
    appData.modalToolNumber = 0;       // No tool selected
    appData.absoluteMode = true;       // G90 absolute mode (GRBL default)
    appData.modalPlane = 0;            // G17 XY plane (GRBL default)
    appData.activeWCS = 0;             // G54 default work coordinate system
    
    // ✅ Initialize alarm state
    appData.alarmCode = 0;             // No alarm
    
    // ✅ Initialize motion phase system (priority-based task scheduler)
    appData.motionPhase = MOTION_PHASE_IDLE;   // Safe for G-code processing
    appData.dominantAxis = AXIS_X;             // Default dominant axis
    appData.currentStepInterval = 0;           // No active motion
    appData.currentSegment = NULL;             // No active segment
    
    // ✅ ARRAY-BASED: Initialize Bresenham errors for all axes
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        appData.bresenham_error[axis] = 0;
    }
    
    appData.uartPollCounter = 0;               // Rate limiting counter
    
    // ✅ Initialize arc generation state (non-blocking incremental)
    appData.arcGenState = ARC_GEN_IDLE;
    appData.arcTheta = 0.0f;
    appData.arcThetaEnd = 0.0f;
    appData.arcThetaIncrement = 0.0f;
    appData.arcRadius = 0.0f;
    appData.arcClockwise = false;
    appData.arcPlane = 0;  // XY plane
    appData.arcFeedrate = 0.0f;
    
    // ✅ Initialize UART utilities (callback-based non-blocking output)
    UART_Initialize();
    
    // ✅ Initialize axis hardware configuration (must be after SETTINGS and STEPPER init)
    // NOTE: This will be finalized in APP_CONFIG state after all peripherals are ready
    UTILS_InitAxisConfig();
    
    // ✅ Initialize limit switch configuration (must be after SETTINGS init)
    UTILS_InitLimitConfig();
    
    // ✅ Initialize homing system
    HOMING_Initialize();
    
    // ✅ Initialize spindle PWM control (OC8/TMR6)
    SPINDLE_Initialize();

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

    // ✅ Following Harmony pattern: Protocol handlers run INSIDE states
    // This ensures they only execute when subsystems are ready

    switch ( appData.state )
    {
        case APP_CONFIG:
        {
            // Initialize subsystems ONCE during configuration
            STEPPER_Initialize(&appData);                   // ✅ Pass APP_DATA reference for ISR phase signaling
            MOTION_Initialize();                            // Motion planning initialization  
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
            
            // ✅ CRITICAL: Update stepper cached values after settings loaded
            // STEPPER_Initialize() ran before settings were loaded, so steps_per_mm might be stale
            CNC_Settings* settings = SETTINGS_GetCurrent();
            StepperPosition* stepper_pos = STEPPER_GetPosition();
            
            // ✅ ARRAY-BASED: Update steps_per_mm from settings (loop for scalability)
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                stepper_pos->steps_per_mm[axis] = settings->steps_per_mm[axis];
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

            // ===== LED2 STATUS INDICATOR (CORETIMER-TIMED, NOT LOOP-COUNT) =====
            // Maintain stable heartbeat independent of loop workload
            // Toggle about once per second when idle (no motion)
            static uint32_t hb_last = 0;
            const uint32_t HB_INTERVAL = 100000000U; // ~1s @ 100MHz Core Timer (200MHz CPU / 2)
            if(appData.motionQueueCount == 0) {
                uint32_t now_ticks = CORETIMER_CounterGet();
                if ((uint32_t)(now_ticks - hb_last) >= HB_INTERVAL) {
                  //  LED2_Toggle();
                    hb_last = now_ticks;
                }
            }
            // During motion, ISR toggles LED1 on each step (fast blink)
            // LED1 will blink at step rate, visible motion indicator
            // ===== PROCESS G-CODE FIRST (EVERY ITERATION) =====
            // Flow control uses appData.motionQueueCount directly (no sync needed)

            // ✅ ALWAYS process G-code (even in ALARM) for status queries and $X clear
            // Read bytes, tokenize, and queue commands continuously
            GCODE_Tasks(&appData, &appData.gcodeCommandQueue);

            // ===== MOTION CONTROLLER - RUNS BEFORE EVENT PROCESSING =====
            // ⚠️ CRITICAL: Motion must run EVERY iteration to keep ISR fed with segments
            // This runs even during dwell to complete existing motion before timer starts
            MOTION_Tasks(&appData);

            // ✅ CRITICAL: Check deferred OKs immediately when segment completes
            // Motion frees a queue slot → check if we should send deferred "ok"
            if (appData.motionSegmentCompleted) {
                appData.motionSegmentCompleted = false;
                GCODE_CheckDeferredOk(&appData, &appData.gcodeCommandQueue);
            }

            // ===== INCREMENTAL ARC GENERATION (NON-BLOCKING) =====
            // Generate arc segments one at a time when arc is active
            if(appData.arcGenState == ARC_GEN_ACTIVE) {
                MOTION_Arc(&appData);
            }
            
            // ===== HOMING STATE MACHINE (NON-BLOCKING) =====
            // Process homing cycle if active
            HOMING_Tasks(&appData);

            // ===== EVENT PROCESSING - CONVERT GCODE TO MOTION =====
            // ⚠️ Only process events if NOT in alarm state
            if(appData.state != APP_ALARM) {
                
                // Process next event - convert one queued command into motion event
                // Dwell is now handled as a motion segment (no special checking needed)
                GCODE_Event event;
                if (GCODE_GetNextEvent(&appData.gcodeCommandQueue, &event)) {
                    DEBUG_PRINT_GCODE("[APP] Event retrieved: type=%d (1=LINEAR,2=ARC)\r\n", event.type);
                    
                    // ⚠️ CRITICAL: Don't process ARC events while another arc is generating
                    // But allow non-arc commands (dwell, linear, etc.) to process
                    bool should_process = true;
                    if (event.type == GCODE_EVENT_ARC_MOVE && appData.arcGenState == ARC_GEN_ACTIVE) {
                        should_process = false;  // Defer arc until current arc completes
                    }
                    
                    if (should_process) {
                        // Process event - only consume if successful
                        if (MOTION_ProcessGcodeEvent(&appData, &event)) {
                            // Event processed successfully - consume it from queue
                            GCODE_ConsumeEvent(&appData.gcodeCommandQueue);
                        }
                        // If processing failed (queue full), leave event in queue for next iteration
                    }
                }
            }

            // ===== HARD LIMIT CHECK (TEMP DISABLED) =====
            // Check limit switches with inversion mask from settings
            // ⚠️ TEMPORARILY DISABLED FOR UART TESTING
            /*
            CNC_Settings* settings = SETTINGS_GetCurrent();
            if(MOTION_UTILS_CheckHardLimits(settings->limit_pins_invert)) {
                // ⚠️ HARD LIMIT TRIGGERED - Emergency stop!
                appData.alarmCode = 1;  // Alarm code 1 = hard limit
                appData.state = APP_ALARM;
                break;  // Immediate transition to alarm state
            }
            */

            break;
        }
        
        case APP_ALARM:
        {
            // Emergency stop state - all motion halted
            // LED2 constant ON to indicate alarm
            LED2_Set();
            
            // User must acknowledge alarm and reset
            STEPPER_DisableAll();
            break;
        }
        
        case APP_WAIT_FOR_CONFIGURATION:
        case APP_DEVICE_ATTACHED:
        case APP_WAIT_FOR_DEVICE_ATTACH:
        case APP_DEVICE_DETACHED:
        case APP_ERROR:
        default:
            break;
    }
}
