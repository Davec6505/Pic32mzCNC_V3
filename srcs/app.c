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
#include "config/default/peripheral/ocmp/plib_ocmp5.h"  // ✅ DEBUG: For OC5 test
#include "config/default/peripheral/tmr/plib_tmr2.h"    // ✅ DEBUG: For TMR2 counter
#include "motion.h"  // Make sure this is included

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
    appData.modalPlane = 0;            // G17 XY plane (GRBL default)
    
    // ✅ Initialize alarm state
    appData.alarmCode = 0;             // No alarm
    
    // ✅ Initialize motion phase system (priority-based task scheduler)
    appData.motionPhase = MOTION_PHASE_IDLE;   // Safe for G-code processing
    appData.dominantAxis = AXIS_X;             // Default dominant axis
    appData.currentStepInterval = 0;           // No active motion
    appData.currentSegment = NULL;             // No active segment
    appData.bresenham_error_y = 0;
    appData.bresenham_error_z = 0;
    appData.bresenham_error_a = 0;
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
            GRBL_Settings* settings = SETTINGS_GetCurrent();
            StepperPosition* stepper_pos = STEPPER_GetPosition();
            stepper_pos->steps_per_mm_x = settings->steps_per_mm_x;
            stepper_pos->steps_per_mm_y = settings->steps_per_mm_y;
            stepper_pos->steps_per_mm_z = settings->steps_per_mm_z;
            stepper_pos->steps_per_deg_a = settings->steps_per_mm_a;
            
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
            // ===== DEBUG TEST DISABLED - ENABLE SERIAL ONLY =====
            // (Debug test code removed to restore serial communication)
            
            // ===== MOTION PHASE PROCESSING (HIGHEST PRIORITY) =====
            // ✅ Following Harmony pattern: Protocol handlers run INSIDE states
            // This ensures they only execute when subsystems are ready

            // ✅ TMR2 ROLLOVER MONITORING (CRITICAL - PREVENTS 343.6s OVERFLOW)
            STEPPER_CheckTimerRollover(&appData);

            // ✅ MOTION CONTROLLER (HIGHEST PRIORITY - LOAD NEXT SEGMENT) =====
            MOTION_Tasks(&appData);

            // ===== INCREMENTAL ARC GENERATION (NON-BLOCKING) =====
            // ✅ Moved to motion.c for clean separation
            MOTION_Arc(&appData);
            
            // ===== RATE-LIMITED UART POLLING (EVERY ~10ms) =====
            appData.uartPollCounter++;
            if(appData.uartPollCounter >= 1250) {  // ~10ms @ 100kHz scan rate
                GCODE_Tasks(&appData.gcodeCommandQueue);
                appData.uartPollCounter = 0;
            }
            
            // ✅ STEP 2: Sync motion queue status for flow control
            appData.gcodeCommandQueue.motionQueueCount = appData.motionQueueCount;
            appData.gcodeCommandQueue.maxMotionSegments = MAX_MOTION_SEGMENTS;

            // ✅ STEP 3: HARD LIMIT CHECK (GRBL safety feature)
            // Check limit switches with inversion mask from settings
            GRBL_Settings* settings = SETTINGS_GetCurrent();
            if(MOTION_UTILS_CheckHardLimits(settings->limit_pins_invert)) {
                // ⚠️ HARD LIMIT TRIGGERED - Emergency stop!
                appData.alarmCode = 1;  // Alarm code 1 = hard limit
                appData.state = APP_ALARM;
                break;  // Immediate transition to alarm state
            }

            // ✅ STEP 4: Process ONE G-code event per iteration (non-blocking!)
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
                                // G90 absolute mode - use coordinates directly (or current if NAN)
                                end.x = isnan(event.data.linearMove.x) ? appData.currentX : event.data.linearMove.x;
                                end.y = isnan(event.data.linearMove.y) ? appData.currentY : event.data.linearMove.y;
                                end.z = isnan(event.data.linearMove.z) ? appData.currentZ : event.data.linearMove.z;
                                end.a = isnan(event.data.linearMove.a) ? appData.currentA : event.data.linearMove.a;
                            } else {
                                // G91 relative mode - add to current position (or keep current if NAN)
                                end.x = isnan(event.data.linearMove.x) ? appData.currentX : (appData.currentX + event.data.linearMove.x);
                                end.y = isnan(event.data.linearMove.y) ? appData.currentY : (appData.currentY + event.data.linearMove.y);
                                end.z = isnan(event.data.linearMove.z) ? appData.currentZ : (appData.currentZ + event.data.linearMove.z);
                                end.a = isnan(event.data.linearMove.a) ? appData.currentA : (appData.currentA + event.data.linearMove.a);
                            }
                            
                            // ✅ SOFT LIMIT CHECK (non-blocking GRBL implementation)
                            // Check target position against max_travel settings
                            GRBL_Settings* settings = SETTINGS_GetCurrent();
                            bool limit_violation = false;
                            
                            // ✅ CRITICAL: Only check limits if settings are valid (not NAN)
                            // If settings corrupted, allow motion (fail-safe operation)
                            if(!isnan(settings->max_travel_x) && !isnan(settings->max_travel_y) && !isnan(settings->max_travel_z)) {
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
                    {
                        // G2/G3 arc commands - Initialize incremental arc generation
                        // Calculate arc parameters, then stream segments over multiple iterations
                        
                        // Build start and end coordinates
                        CoordinatePoint start = {appData.currentX, appData.currentY, appData.currentZ, appData.currentA};
                        CoordinatePoint end;
                        
                        if(appData.absoluteMode) {
                            // G90 absolute mode
                            end.x = event.data.arcMove.x;
                            end.y = event.data.arcMove.y;
                            end.z = event.data.arcMove.z;
                            end.a = event.data.arcMove.a;
                        } else {
                            // G91 relative mode
                            end.x = appData.currentX + event.data.arcMove.x;
                            end.y = appData.currentY + event.data.arcMove.y;
                            end.z = appData.currentZ + event.data.arcMove.z;
                            end.a = appData.currentA + event.data.arcMove.a;
                        }
                        
                        // Center is ALWAYS incremental (centerX/centerY offsets from start)
                        CoordinatePoint center;
                        center.x = start.x + event.data.arcMove.centerX;
                        center.y = start.y + event.data.arcMove.centerY;
                        center.z = start.z;  // Z doesn't participate in arc center
                        center.a = 0.0f;
                        
                        // Verify radius (GRBL arc validation)
                        float r_start = sqrtf(event.data.arcMove.centerX * event.data.arcMove.centerX + 
                                             event.data.arcMove.centerY * event.data.arcMove.centerY);
                        float r_end = sqrtf((end.x - center.x) * (end.x - center.x) + 
                                           (end.y - center.y) * (end.y - center.y));
                        
                        if(fabsf(r_start - r_end) > 0.005f) {
                            // Radius error - abort arc
                            appData.alarmCode = 33;  // GRBL alarm: arc radius error
                            appData.state = APP_ALARM;
                            break;
                        }
                        
                        // Calculate angles
                        float start_angle = atan2f(start.y - center.y, start.x - center.x);
                        float end_angle = atan2f(end.y - center.y, end.x - center.x);
                        
                        // Calculate total angle (handle wrap-around)
                        float total_angle;
                        if(event.data.arcMove.clockwise) {
                            // G2 clockwise
                            total_angle = start_angle - end_angle;
                            if(total_angle <= 0.0f) total_angle += 2.0f * M_PI;
                        } else {
                            // G3 counter-clockwise
                            total_angle = end_angle - start_angle;
                            if(total_angle <= 0.0f) total_angle += 2.0f * M_PI;
                        }
                        
                        // ✅ Initialize arc generation state (incremental streaming)
                        // Arc generation happens in MOTION_Arc() called from main loop
                        appData.arcGenState = ARC_GEN_ACTIVE;
                        appData.arcTheta = start_angle;
                        appData.arcThetaEnd = end_angle;
                        appData.arcCenter.x = center.x;
                        appData.arcCenter.y = center.y;
                        appData.arcCurrent = start;
                        appData.arcEndPoint = end;
                        appData.arcRadius = r_start;
                        appData.arcClockwise = event.data.arcMove.clockwise;
                        appData.arcPlane = appData.modalPlane;
                        appData.arcFeedrate = event.data.arcMove.feedrate;
                        
                        // Calculate arc segment increment
                        GRBL_Settings* settings = SETTINGS_GetCurrent();
                        float arc_length = r_start * total_angle;
                        uint32_t num_segments = (uint32_t)ceilf(arc_length / settings->mm_per_arc_segment);
                        if(num_segments < 1) num_segments = 1;
                        appData.arcThetaIncrement = total_angle / (float)num_segments;
                        if(!event.data.arcMove.clockwise) {
                            // CCW = positive angle increment
                            appData.arcThetaIncrement = fabsf(appData.arcThetaIncrement);
                        } else {
                            // CW = negative angle increment
                            appData.arcThetaIncrement = -fabsf(appData.arcThetaIncrement);
                        }
                        
                        break;
                    }
                    
                    case GCODE_EVENT_COOLANT_ON:
                        // M7/M8 coolant on
                        // TODO: Implement coolant control hardware interface
                        break;
                    
                    case GCODE_EVENT_COOLANT_OFF:
                        // M9 coolant off
                        // TODO: Implement coolant control hardware interface
                        break;
                    
                    case GCODE_EVENT_NONE:
                    default:
                        // No action for unknown events
                        break;
                }
            }
            
            break;
        }
        
        case APP_ALARM:
        {
            // Emergency stop state - all motion halted
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
