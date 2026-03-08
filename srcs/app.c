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
#include "stepper.h"  // For g_hard_limit_alarm flag
#include "config/default/peripheral/coretimer/plib_coretimer.h"  // For CORETIMER heartbeat counter
#include "utils/uart_utils.h"  // Non-blocking UART utilities
#include "utils/utils.h"       // For UTILS_InitAxisConfig
#include "motion.h"
#include "motion/tmc5160.h"    // TMC5160 SPI stepper driver

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
// E-Stop Callback — fired by EXTERNAL_3_InterruptHandler at IPL7
// RF4 pulled LOW (falling edge) when E-Stop button pressed
// *****************************************************************************
static void ESTOP_Callback(EXTERNAL_INT_PIN pin, uintptr_t context)
{
    (void)pin;
    (void)context;

    // Debounce: ignore if pin bounced back HIGH
    if (ESTOP_Get() != 0U) {
        return;
    }

    // 1. Kill all steppers immediately (hardware level)
    STEPPER_DisableAll();

    // 2. Stop step timer
    TMR4_Stop();

    // 3. Kill spindle PWM
    OCMP8_Disable();

    // 4. Flush motion queue
    appData.motionQueueCount = 0;
    appData.motionQueueHead  = 0;
    appData.motionQueueTail  = 0;
    appData.currentSegment   = NULL;

    // 5. Signal application state machine
    appData.eStopTriggered = true;
    appData.alarmCode      = 10;   // GRBL ALARM:10 = E-Stop
    appData.state          = APP_ALARM;
}

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
    
    appData.dominantAxis = AXIS_X;             // Default dominant axis
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

#ifdef HAS_TMC5160_AXIS
            TMC5160_Initialize();                          // Configure TMC5160 drivers via SPI2
#endif

            // Register E-Stop callback on INT3 (RF4 via PPS, IPL7)
            // Button pressed → RF4 pulled LOW → falling edge fires EXTERNAL_3_Handler
            EVIC_ExternalInterruptCallbackRegister(EXTERNAL_INT_3, ESTOP_Callback, (uintptr_t)NULL);
            EVIC_SourceStatusClear(INT_SOURCE_EXTERNAL_3);  // Clear any flag set during boot
            EVIC_ExternalInterruptEnable(EXTERNAL_INT_3);

            appData.state = APP_LOAD_SETTINGS;
            break;
        }
        
        case APP_LOAD_SETTINGS:
        {
            // ✅ Delay flash read until after all peripherals initialized
            // Try to load from flash - if invalid signature/CRC, use defaults (already loaded)
            if (SETTINGS_LoadFromFlash(SETTINGS_GetCurrent())) {
                // Flash settings loaded successfully
                uint32_t led2_flash_loaded = 0;
                while(led2_flash_loaded < 10){
                    LED2_Toggle();
                    led2_flash_loaded++;
                    CORETIMER_DelayMs(250);
                }
          
            } else {
                    // Flash empty or invalid - defaults already loaded in SETTINGS_Initialize()
                        // Try to load from flash - if invalid signature/CRC, use defaults (already loaded)
                if (SETTINGS_LoadFromFlash(SETTINGS_GetCurrent())) {
                    // Flash settings loaded successfully
                    uint32_t led2_flash_loaded = 0;
                    while(led2_flash_loaded < 10){
                        LED2_Toggle();
                        led2_flash_loaded++;
                        CORETIMER_DelayMs(50);
                    }
                }
            }
            
            // ✅ CRITICAL: Update stepper cached values after settings loaded
            // STEPPER_Initialize() ran before settings were loaded, so ALL cached values are stale
            CNC_Settings* settings = SETTINGS_GetCurrent();
            StepperPosition* stepper_pos = STEPPER_GetPosition();
            
            // ✅ ARRAY-BASED: Update steps_per_mm from settings (loop for scalability)
            for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
                stepper_pos->steps_per_mm[axis] = settings->steps_per_mm[axis];
            }
            
            // ✅ CRITICAL: Reload stepper invert masks ($0-$5) after settings loaded from flash
            // This ensures direction_invert_mask, step_pulse_invert_mask, enable_invert are current
            STEPPER_ReloadSettings();

#ifdef HAS_TMC5160_AXIS
            // ✅ Re-apply TMC5160 driver config from flash-loaded settings
            // TMC5160_Initialize() ran in APP_CONFIG with compile-time defaults;
            // now apply whatever the user saved via $200-$253 at runtime.
            {
                E_AXIS ax;
                for (ax = AXIS_X; ax < NUM_AXIS; ax++) {
                    TMC5160_ApplySettings(ax, settings);
                }
            }
#endif

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

#ifdef HAS_TMC5160_AXIS
            // Rate-limited TMC5160 DRVSTATUS diagnostic poll (~10Hz)
            // Uses CoreTimer tick difference to avoid blocking motion
            static uint32_t tmc_last_poll = 0;
            uint32_t tmc_now = CORETIMER_CounterGet();
            // CoreTimer runs at CPU/2 = 100MHz. 100,000,000 ticks = 1s -> 10Hz = 10,000,000
            if ((tmc_now - tmc_last_poll) >= 10000000UL) {
                tmc_last_poll = tmc_now;
                if (TMC5160_Tasks()) {
                    // Fault detected — log (alarm handling can be added here later)
                    UART_Printf("[TMC5160] Driver fault detected!\r\n");
                }
            }
#endif

            // ===== PROCESS G-CODE FIRST (EVERY ITERATION) =====
            // Flow control uses appData.motionQueueCount directly (no sync needed)

            // ✅ ALWAYS process G-code (even in ALARM) for status queries and $X clear
            // Read bytes, tokenize, and queue commands continuously
            GCODE_Tasks(&appData, &appData.gcodeCommandQueue);

            // Release deferred oks based on gcode queue space (once per main-loop
            // iteration). Space-based: if q->count < HIGH_WATER and okPendingCount > 0,
            // send ok. This is the primary mechanism that keeps UGS streaming during
            // arc generation (where commands_consumed is frozen so delta-based
            // release would not fire).
            GCODE_CheckDeferredOk(&appData, &appData.gcodeCommandQueue);


            // ===== HOMING STATE MACHINE (NON-BLOCKING) =====
            // Process homing cycle if active
            HOMING_Tasks(&appData);



            // ===== INCREMENTAL ARC GENERATION (NON-BLOCKING) =====
            // Generate arc segments one at a time when arc is active
            // needs to run before motion tasks to keep the queue fed and avoid underrun during long arcs
            if(appData.arcGenState == ARC_GEN_ACTIVE) {
                MOTION_Arc(&appData);
            }
            

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


            // ===== PROBE MONITORING (G38.x COMMANDS) =====
            // Check probe input during motion and handle trigger/failure
            if (appData.probeState == PROBE_STATE_MOVING) {
                // Check if probe input triggered (Z-max with $6 invert applied)
                CNC_Settings* settings = SETTINGS_GetCurrent();
                bool probe_triggered = PROBE_Get(settings->probe_invert);
                
                if (probe_triggered) {
                    // ✅ PROBE TRIGGERED - Stop motion immediately
                    DEBUG_PRINT_MOTION("[PROBE] Triggered! Stopping motion\r\n");
                    
                    // Stop all motion
                    TMR4_Stop();
                    STEPPER_DisableAll();
                    
                    // Clear motion queue
                    appData.motionQueueHead = 0;
                    appData.motionQueueTail = 0;
                    appData.motionQueueCount = 0;
                    appData.currentSegment = NULL;
                    
                    // Save trigger position (current position is exact trigger point)
                    appData.probePosition.coordinate[AXIS_X] = appData.current[AXIS_X];
                    appData.probePosition.coordinate[AXIS_Y] = appData.current[AXIS_Y];
                    appData.probePosition.coordinate[AXIS_Z] = appData.current[AXIS_Z];
                    appData.probePosition.coordinate[AXIS_A] = appData.current[AXIS_A];
                    
                    // Mark success and transition to triggered state
                    appData.probeSuccess = true;
                    appData.probeState = PROBE_STATE_TRIGGERED;
                    
                } else if (appData.motionQueueCount == 0 && !appData.probeSuccess) {
                    // ✅ PROBE FAILED - Reached target without trigger
                    DEBUG_PRINT_MOTION("[PROBE] Failed - no trigger detected\r\n");
                    appData.probeState = PROBE_STATE_FAILED;
                }
            }
            
            // Handle probe completion (triggered or failed)
            if (appData.probeState == PROBE_STATE_TRIGGERED) {
                // Send probe result: [PRB:x,y,z,a:1]
                UART_Printf("[PRB:%.3f,%.3f,%.3f,%.3f:1]\r\n",
                           appData.probePosition.coordinate[AXIS_X],
                           appData.probePosition.coordinate[AXIS_Y],
                           appData.probePosition.coordinate[AXIS_Z],
                           appData.probePosition.coordinate[AXIS_A]);
                UART_SendOK();
                
                // Reset probe state
                appData.probeState = PROBE_STATE_IDLE;
                
                DEBUG_PRINT_MOTION("[PROBE] Success reported\r\n");
                
            } else if (appData.probeState == PROBE_STATE_FAILED) {
                if (appData.probeAlarmOnFail) {
                    // G38.2 or G38.4 - Trigger ALARM
                    UART_Printf("ALARM:5\r\n");  // Probe fail alarm
                    appData.state = APP_ALARM;
                    appData.alarmCode = 5;
                    
                    DEBUG_PRINT_MOTION("[PROBE] ALARM:5 triggered (probe failed)\r\n");
                } else {
                    // G38.3 or G38.5 - No alarm, just report failure
                    UART_Printf("[PRB:%.3f,%.3f,%.3f,%.3f:0]\r\n",
                               appData.current[AXIS_X],
                               appData.current[AXIS_Y],
                               appData.current[AXIS_Z],
                               appData.current[AXIS_A]);
                    UART_SendOK();
                    
                    DEBUG_PRINT_MOTION("[PROBE] Failure reported (no alarm)\r\n");
                }
                
                // Reset probe state
                appData.probeState = PROBE_STATE_IDLE;
            }
            

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
                            // ✅ Causal ok release: fire BEFORE UGS can refill the freed slot.
                            // Without this, UGS immediately sends another command on the same
                            // iteration, q->count stays at HIGH_WATER, and the deferred ok for
                            // arc 2 is never released → deterministic arc-to-arc stall on run 2.
                            GCODE_CheckDeferredOk(&appData, &appData.gcodeCommandQueue);
                            // ✅ CRITICAL FIX: If new arc just started, generate first segment IMMEDIATELY
                            // This prevents 1-iteration gap between arcs which drains queue
                            if (event.type == GCODE_EVENT_ARC_MOVE && 
                                appData.arcGenState == ARC_GEN_ACTIVE) {
                                MOTION_Arc(&appData);
                            }
                        } else if (event.type == GCODE_EVENT_PROGRAM_END) {
                            // ✅ CRITICAL FIX: M0, M2, M30 (program end) - not handled by motion
                            // Consume the event and remain in IDLE state
                            // This prevents machine from getting stuck after file completion
                            DEBUG_PRINT_APP("[APP] Program end (M0/M2/M30) - file complete\r\n");
                            GCODE_MarkProgramEnd();                         // ← hold ok until motion drains
                            GCODE_ConsumeEvent(&appData.gcodeCommandQueue); // ← increments commands_consumed
                            GCODE_CheckDeferredOk(&appData, &appData.gcodeCommandQueue);                       
                        }
                        // If processing failed (queue full), leave event in queue for next iteration
                    }
                }
            }

            // ===== HARD LIMIT CHECK =====
            // Check hard limits in main loop (non-blocking, state-aware)
            // Note: homing_active already calculated above for LED2 indicator
            CNC_Settings* settings = SETTINGS_GetCurrent();
   
            
            
            // ===== HOMING STATE CHECK (USED FOR LED2 AND HARD LIMITS) =====
            // g_homing declared in motion/homing.h (already included at top)
            bool homing_active = (g_homing.state != HOMING_STATE_IDLE && 
                                  g_homing.state != HOMING_STATE_COMPLETE &&
                                  g_homing.state != HOMING_STATE_ALARM);
            

            // During homing: Use limit check to drive state transitions
            if (homing_active) {
                // ===== LED2 STATE INDICATOR: HOMING =====
                static uint32_t led2_homing_last = 0;
                const uint32_t LED2_HOMING_INTERVAL = 100000000U; // 1 second @ 100MHz Core Timer
                uint32_t now = CORETIMER_CounterGet();
                if ((uint32_t)(now - led2_homing_last) >= LED2_HOMING_INTERVAL) {
                    LED2_Toggle();
                    led2_homing_last = now;
                }

                // Per-state limit handling — only sample when meaningful for that state.
                switch (g_homing.state) {

                    case HOMING_STATE_SEEK:
                        // Fast approach: sample limit, looking for first contact.
                        UTILS_HomingLimitUpdate(HOMING_IsLimitActiveNow());
                        if (UTILS_HomingLimitRisingEdge()) {
                            DEBUG_PRINT_MOTION("[APP_HOMING] SEEK: switch hit → LOCATE backoff\r\n");
                            STEPPER_StopMotion();   // stops interpolator + flushes trajectory queue
                            g_homing.motion_active = false;
                            HOMING_StartLocate(&appData);   // queues backoff, kicks interpolator
                        }
                        break;

                    case HOMING_STATE_LOCATE_BACKOFF:
                        // Switch still physically closed — do NOT sample, would corrupt edge tracker.
                        // HOMING_Tasks() watches queue-empty → calls HOMING_StartLocateReapproach().
                        break;

                    case HOMING_STATE_LOCATE_REAPPROACH:
                        // Slow re-approach: sample limit, looking for precision contact.
                        UTILS_HomingLimitUpdate(HOMING_IsLimitActiveNow());
                        if (UTILS_HomingLimitRisingEdge()) {
                            DEBUG_PRINT_MOTION("[APP_HOMING] LOCATE: precision hit → PULLOFF\r\n");
                            STEPPER_StopMotion();   // stops interpolator + flushes trajectory queue
                            g_homing.motion_active = false;
                            // Zero step counter at the exact home point.
                            {
                                CNC_Settings* hset = SETTINGS_GetCurrent();
                                bool home_positive  = (*g_homing_settings[g_homing.current_axis].homing_dir_mask
                                                       >> g_homing.current_axis) & 0x01;
                                float home_zero = home_positive
                                                  ? hset->max_travel[g_homing.current_axis] : 0.0f;
                                KINEMATICS_SetAxisMachinePosition(g_homing.current_axis, home_zero);
                            }
                            g_homing.state = HOMING_STATE_PULLOFF;
                            HOMING_StartPulloff(&appData);
                        }
                        break;

                    default:
                        // HOMING_Tasks() drives PULLOFF → COMPLETE via queue-empty.
                        // No limit sampling needed in remaining states.
                        break;
                }

                // ✅ Clear hard limit suppression when all limits physically released
                if (g_suppress_hard_limits && !MOTION_UTILS_CheckHardLimits(settings->limit_pins_invert)) {
                    g_suppress_hard_limits = false;
                    DEBUG_PRINT_APP("[APP] Hard limit suppression cleared - all limits released\r\n");
                }
                    
            } else {
                // ===== HARD LIMIT CHECK (normal motion, non-homing) =====
                if (settings->hard_limits_enable && !g_suppress_hard_limits) {
                    if (MOTION_UTILS_CheckHardLimits(settings->limit_pins_invert)) {
                        STEPPER_StopMotion();
                        g_hard_limit_alarm = true;
                        UART_Printf("ALARM:1\r\n");  // Hard limit triggered
                        appData.alarmCode = 1;
                        appData.state = APP_ALARM;
                        DEBUG_PRINT_APP("[APP] Hard limit triggered during motion - ALARM:1\r\n");
                    }
                }
                // Clear hard limit suppression when limits physically released
                if (g_suppress_hard_limits && !MOTION_UTILS_CheckHardLimits(settings->limit_pins_invert)) {
                    g_suppress_hard_limits = false;
                }
            }

            break;
        }
        
        case APP_ALARM:
        {
            // Emergency stop state - all motion halted
            
            // ===== LED2 STATE INDICATOR: ALARM =====
            // Toggle LED2 at 10Hz (100ms) during alarm
            static uint32_t led2_alarm_last = 0;
            const uint32_t LED2_ALARM_INTERVAL = 10000000U; // 100ms @ 100MHz Core Timer
            uint32_t now_ticks = CORETIMER_CounterGet();
            if ((now_ticks - led2_alarm_last) >= LED2_ALARM_INTERVAL) {
                LED2_Toggle();
                led2_alarm_last = now_ticks;
            }
            
            // User must acknowledge alarm and reset
            STEPPER_DisableAll();
            
            // ✅ CRITICAL FIX: Clear hard limit suppression when limits physically released
            // This allows $X to work after user moves machine off limit switch
            CNC_Settings* settings = SETTINGS_GetCurrent();
            if (g_suppress_hard_limits && !MOTION_UTILS_CheckHardLimits(settings->limit_pins_invert)) {
                g_suppress_hard_limits = false;
                DEBUG_PRINT_APP("[APP_ALARM] Hard limit suppression cleared - all limits released\r\n");
            }
            
            // ✅ Report E-Stop alarm once to host
            if (appData.eStopTriggered) {
                UART_Printf("ALARM:10\r\n");  // E-Stop asserted
                appData.eStopTriggered = false;
                // Recovery: host must send Ctrl+X (soft reset) to clear
            }

            // ✅ Check if alarm cleared by $X command (hard limit)
            if (!g_hard_limit_alarm && appData.alarmCode == 1) {
                // Hard limit alarm cleared - return to IDLE
                DEBUG_PRINT_APP("[APP] Hard limit alarm cleared, returning to IDLE\r\n");
                appData.alarmCode = 0;
                appData.state = APP_IDLE;
                LED2_Clear();  // Turn off alarm LED
            }
            
            // Continue processing G-code (allows $X, $$, $#, ?, etc.)
            GCODE_Tasks(&appData, &appData.gcodeCommandQueue);
            
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
