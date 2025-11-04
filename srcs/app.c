
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
   static uint32_t idle_indicator = 0;

    // ✅ Following Harmony pattern: Protocol handlers run INSIDE states
    // This ensures they only execute when subsystems are ready

    switch ( appData.state )
    {
        case APP_CONFIG:
        {
            // Initialize subsystems ONCE during configuration
            STEPPER_Initialize(&appData);                   // ✅ Pass APP_DATA reference for ISR phase signaling
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
            // ✅ PRIORITY PHASE SYSTEM: Process motion phases BEFORE G-code
            // ISR sets motionPhase flag when dominant axis fires
            // Main loop processes phases in priority order (0 = highest)
            // G-code only runs when motionPhase == IDLE (prevents blocking)
            
            // ✅ STEP 0: Load next segment from queue (BEFORE phase processing)
            // This provides look-ahead and prevents jitter between segments
            // MOTION_Tasks manages the pipeline: load next while executing current
            MOTION_Tasks(appData.motionQueue, &appData.motionQueueHead, 
                        &appData.motionQueueTail, &appData.motionQueueCount);
            
            switch(appData.motionPhase) {
                case MOTION_PHASE_VELOCITY:
                {
                    // Phase 0: Velocity conditioning (highest priority)
                    // Update step interval based on acceleration/cruise/decel state
                    if(appData.currentSegment != NULL) {
                        MotionSegment* seg = appData.currentSegment;
                        
                        if(seg->steps_completed < seg->accelerate_until) {
                            // Acceleration phase - interval decreases (speed increases)
                            appData.currentStepInterval += seg->rate_delta;  // rate_delta is negative
                            if(appData.currentStepInterval < seg->nominal_rate) {
                                appData.currentStepInterval = seg->nominal_rate;  // Clamp to max speed
                            }
                        } else if(seg->steps_completed > seg->decelerate_after) {
                            // Deceleration phase - interval increases (speed decreases)
                            appData.currentStepInterval += seg->rate_delta;  // rate_delta is positive
                            if(appData.currentStepInterval > seg->final_rate) {
                                appData.currentStepInterval = seg->final_rate;  // Clamp to min speed
                            }
                        } else {
                            // Cruise phase - maintain nominal rate
                            appData.currentStepInterval = seg->nominal_rate;
                        }
                    }
                    
                    appData.motionPhase = MOTION_PHASE_BRESENHAM;
                    // Fall through to next phase
                }
                    
                case MOTION_PHASE_BRESENHAM:
                {
                    // Phase 1: Bresenham error accumulation
                    // Determine which subordinate axes need steps
                    if(appData.currentSegment != NULL) {
                        MotionSegment* seg = appData.currentSegment;
                        
                        // Accumulate errors for subordinate axes (not dominant)
                        if(seg->dominant_axis != AXIS_Y && seg->delta_y != 0) {
                            appData.bresenham_error_y += seg->delta_y;
                        }
                        if(seg->dominant_axis != AXIS_Z && seg->delta_z != 0) {
                            appData.bresenham_error_z += seg->delta_z;
                        }
                        if(seg->dominant_axis != AXIS_A && seg->delta_a != 0) {
                            appData.bresenham_error_a += seg->delta_a;
                        }
                    }
                    
                    appData.motionPhase = MOTION_PHASE_SCHEDULE;
                    // Fall through to next phase
                }
                    
                case MOTION_PHASE_SCHEDULE:
                {
                    // Phase 2: OCx register scheduling
                    // Write OCxR/OCxRS with absolute TMR2 values
                    if(appData.currentSegment != NULL) {
                        MotionSegment* seg = appData.currentSegment;
                        
                        // Get dominant axis delta for comparison
                        int32_t dominant_delta = 0;
                        switch(seg->dominant_axis) {
                            case AXIS_X: dominant_delta = seg->delta_x; break;
                            case AXIS_Y: dominant_delta = seg->delta_y; break;
                            case AXIS_Z: dominant_delta = seg->delta_z; break;
                            case AXIS_A: dominant_delta = seg->delta_a; break;
                            default: break;
                        }
                        
                        // Schedule dominant axis (always steps)
                        STEPPER_ScheduleStep(seg->dominant_axis, appData.currentStepInterval);
                        
                        // Schedule subordinate axes if Bresenham says they need a step
                        if(seg->dominant_axis != AXIS_Y && seg->delta_y != 0) {
                            if(appData.bresenham_error_y >= dominant_delta) {
                                STEPPER_ScheduleStep(AXIS_Y, appData.currentStepInterval);
                                appData.bresenham_error_y -= dominant_delta;
                            } else {
                                STEPPER_DisableAxis(AXIS_Y);  // No step this cycle
                            }
                        }
                        
                        if(seg->dominant_axis != AXIS_Z && seg->delta_z != 0) {
                            if(appData.bresenham_error_z >= dominant_delta) {
                                STEPPER_ScheduleStep(AXIS_Z, appData.currentStepInterval);
                                appData.bresenham_error_z -= dominant_delta;
                            } else {
                                STEPPER_DisableAxis(AXIS_Z);
                            }
                        }
                        
                        if(seg->dominant_axis != AXIS_A && seg->delta_a != 0) {
                            if(appData.bresenham_error_a >= dominant_delta) {
                                STEPPER_ScheduleStep(AXIS_A, appData.currentStepInterval);
                                appData.bresenham_error_a -= dominant_delta;
                            } else {
                                STEPPER_DisableAxis(AXIS_A);
                            }
                        }
                        
                        // Increment step counter
                        seg->steps_completed++;
                    }
                    
                    appData.motionPhase = MOTION_PHASE_COMPLETE;
                    // Fall through to next phase
                }
                    
                case MOTION_PHASE_COMPLETE:
                {
                    // Phase 3: Segment completion check
                    // Check if current segment done
                    if(appData.currentSegment != NULL) {
                        MotionSegment* seg = appData.currentSegment;
                        
                        if(seg->steps_completed >= seg->steps_remaining) {
                            // Segment complete - signal MOTION_Tasks to load next
                            // Reset Bresenham errors for next segment
                            appData.bresenham_error_y = 0;
                            appData.bresenham_error_z = 0;
                            appData.bresenham_error_a = 0;
                            appData.currentSegment = NULL;  // MOTION_Tasks will set next
                        }
                    }
                    
                    // Return to IDLE (will be re-triggered by ISR on next dominant axis step)
                    appData.motionPhase = MOTION_PHASE_IDLE;
                    break;  // Exit phase processing
                }
                    
                case MOTION_PHASE_IDLE:
                    // No motion processing needed - fall through to other tasks
                    break;
            }
            
            // ✅ Rate-limited polling for both real-time chars AND G-code (every ~10ms)
            // Real-time commands (?!~^X) get 100Hz response rate - plenty fast for humans
            // Full G-code processing also runs at 10ms intervals when safe
            appData.uartPollCounter++;
            if(appData.uartPollCounter >= 1250) {  // ~10ms @ 125kHz loop rate
                appData.uartPollCounter = 0;
                
                // ✅ STEP 1: Poll serial protocol handler (Harmony pattern)
                // Handles both real-time chars and G-code lines
                // Only runs when UART is ready (after APP_GCODE_INIT)
                GCODE_Tasks(&appData.gcodeCommandQueue);
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
                        
                        // Calculate arc length and segment count
                        float arc_length = r_start * total_angle;  // mm
                        GRBL_Settings* settings = SETTINGS_GetCurrent();
                        uint32_t segment_count = (uint32_t)ceilf(arc_length / settings->mm_per_arc_segment);
                        if(segment_count == 0) segment_count = 1;
                        
                        // Initialize arc generation state
                        appData.arcGenState = ARC_GEN_ACTIVE;
                        appData.arcTheta = start_angle;
                        appData.arcThetaEnd = event.data.arcMove.clockwise ? 
                                             (start_angle - total_angle) : 
                                             (start_angle + total_angle);
                        appData.arcThetaIncrement = total_angle / segment_count;
                        if(event.data.arcMove.clockwise) {
                            appData.arcThetaIncrement = -appData.arcThetaIncrement;  // Negative for CW
                        }
                        appData.arcCenter = center;
                        appData.arcCurrent = start;
                        appData.arcEndPoint = end;
                        appData.arcRadius = r_start;
                        appData.arcClockwise = event.data.arcMove.clockwise;
                        appData.arcPlane = appData.modalPlane;  // Use current modal plane (G17/G18/G19)
                        appData.arcFeedrate = event.data.arcMove.feedrate > 0.0f ? 
                                             event.data.arcMove.feedrate : appData.modalFeedrate;
                        
                        // Arc will be generated incrementally in main loop (see below)
                        // Don't update currentX/Y/Z yet - will update as segments complete
                        break;
                    }
                        
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
            
            // ✅ INCREMENTAL ARC GENERATION (Non-Blocking)
            // Generate ONE arc segment per iteration if arc active and motion queue has space
            if(appData.arcGenState == ARC_GEN_ACTIVE && appData.motionQueueCount < MAX_MOTION_SEGMENTS) {
                // Calculate next point on arc
                appData.arcTheta += appData.arcThetaIncrement;
                
                // Check if this is the last segment
                bool is_last_segment = false;
                if(appData.arcClockwise) {
                    is_last_segment = (appData.arcTheta <= appData.arcThetaEnd);
                } else {
                    is_last_segment = (appData.arcTheta >= appData.arcThetaEnd);
                }
                
                CoordinatePoint next;
                if(is_last_segment) {
                    // Use exact end point for final segment (avoid accumulated error)
                    next = appData.arcEndPoint;
                    appData.arcGenState = ARC_GEN_IDLE;  // Arc complete
                } else {
                    // Calculate intermediate point using sin/cos (FPU accelerated)
                    next.x = appData.arcCenter.x + appData.arcRadius * cosf(appData.arcTheta);
                    next.y = appData.arcCenter.y + appData.arcRadius * sinf(appData.arcTheta);
                    
                    // Linear interpolation for Z and A axes
                    float progress = fabsf(appData.arcTheta - atan2f(appData.arcCurrent.y - appData.arcCenter.y,
                                                                      appData.arcCurrent.x - appData.arcCenter.x));
                    float total_angle = fabsf(appData.arcThetaEnd - atan2f(appData.arcCurrent.y - appData.arcCenter.y,
                                                                            appData.arcCurrent.x - appData.arcCenter.x));
                    float ratio = (total_angle > 0.0f) ? (progress / total_angle) : 0.0f;
                    
                    next.z = appData.arcCurrent.z + (appData.arcEndPoint.z - appData.arcCurrent.z) * ratio;
                    next.a = appData.arcCurrent.a + (appData.arcEndPoint.a - appData.arcCurrent.a) * ratio;
                }
                
                // Convert arc segment to linear motion segment
                MotionSegment* segment = &appData.motionQueue[appData.motionQueueHead];
                KINEMATICS_LinearMove(appData.arcCurrent, next, appData.arcFeedrate, segment);
                
                // Add to motion queue
                appData.motionQueueHead = (appData.motionQueueHead + 1) % MAX_MOTION_SEGMENTS;
                appData.motionQueueCount++;
                
                // Update current position for next iteration
                appData.arcCurrent = next;
                
                // Update work coordinates when arc completes
                if(appData.arcGenState == ARC_GEN_IDLE) {
                    appData.currentX = appData.arcEndPoint.x;
                    appData.currentY = appData.arcEndPoint.y;
                    appData.currentZ = appData.arcEndPoint.z;
                    appData.currentA = appData.arcEndPoint.a;
                }
            }

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
