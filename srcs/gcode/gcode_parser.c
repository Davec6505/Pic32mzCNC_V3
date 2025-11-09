/*
 * All G-code command prefixes that UGS sends:
 * - G commands: G0, G1, G2, G3, G4, G10, G17, G18, G19, G20, G21, G28, G30, G38.x, G43, G49, G53, G54-G59, G80, G90, G91, G92, G93, G94
 * - M commands: M0, M1, M2, M3, M4, M5, M7, M8, M9, M30
 * - S commands: S1000 (spindle speed)
 * - F commands: F1000 (feed rate)
 * - T commands: T1 (tool select)
 * - P commands: P0.5 (dwell time for G4)
 * - Coordinate commands: X10 Y20 Z5 (modal coordinate moves)
 * - Comment lines: (This is a comment)
 * - Empty lines: Just whitespace and line terminators
 
 */



#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <string.h>
#include <stdio.h>
#include <ctype.h>                      // For isdigit()
#include <math.h>                       // ✅ For NAN and isnan()

#include "gcode_parser.h"             
#include "common.h"
#include "stepper.h"
#include "motion.h"
#include "kinematics.h"
#include "motion/homing.h"
#include "utils.h"
#include "settings.h"

/* ========== CONFIGURATION ========== */
#include "data_structures.h"  // ✅ Access to GCODE_CommandQueue with nested motion info
#include "utils/uart_utils.h"        // ✅ Non-blocking UART utilities
#include "../config/default/peripheral/uart/plib_uart3.h"  // ✅ UART3 PLIB (for baud, init)



// ✅ ENABLE startup banner for debugging UART TX
#define ENABLE_STARTUP_BANNER

// ✅ Flow control threshold - leave 2 slots free for safety
#define MOTION_BUFFER_THRESHOLD 2

/* USART Buffers */
static uint8_t txBuffer[250];
static uint8_t rxBuffer[250];
static volatile uint32_t nBytesRead = 0;
static volatile bool rxThresholdEventReceived = false;

/* local datatypes */
GCODE_Data gcodeData = {
    .state = GCODE_STATE_IDLE,
};

// ✅ Add static variable for pending OK
static bool okPending = false;

/* local scope prototypes */
GCODE_CommandQueue* Extract_CommandLineFrom_Buffer(uint8_t* buffer, uint32_t length, GCODE_CommandQueue* commandQueue);
void GCODE_ProcessQueuedCommand(GCODE_CommandQueue* cmdQueue);
CoordinatePoint parse_coordinate_values(const char* command);
float parse_feedrate(const char* command);
bool parse_command_to_event(const char* command, GCODE_Event* event);

const char GRBL_CONTROL_CHARS[] = {'?', '~', 0x18, '!', 0x84, '$'}; // add more as needed


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

void GCODE_USART_Initialize( uint32_t RD_thresholds)
{
    
    // ✅ Initialize UART utilities
    UART_Initialize();
    
    // ✅ PLIB already enables RX ISR in UART3_Initialize()
    // Ring buffer fills automatically - we just poll UART3_ReadCountGet()
    
    // *******************************************************************************************
    // * Print the size of the read buffer on the terminal Dummy test
    // *******************************************************************************************
#if  DEBUG_ == DBG_LEVEL_UART
    UART_Printf("RX Buffer Size = %d\r\n", (int)UART3_ReadBufferSizeGet());
    UART_Printf("TX Buffer Size = %d\r\n", (int)UART3_WriteBufferSizeGet());
#endif

#ifdef ENABLE_STARTUP_BANNER
    // Startup banner - can be disabled to reduce UART traffic at boot
    UART3_Write((uint8_t*)GRBL_FIRMWARE_VERSION, (uint32_t)strlen(GRBL_FIRMWARE_VERSION));
    UART3_Write((uint8_t*)GRBL_BUILD_DATE, (uint32_t)strlen(GRBL_BUILD_DATE));
    UART3_Write((uint8_t*)GRBL_BUILD_TIME, (uint32_t)strlen(GRBL_BUILD_TIME));
#endif
    
    // ✅ Clear any garbage from RX buffer at startup (prevents spurious OK on first command)
    nBytesRead = 0;
    memset((void*)rxBuffer, 0, sizeof(rxBuffer));
    okPending = false;
}


/* @brief Extract G-code command line from buffer and queue it
 * @param buffer Pointer to input buffer containing G-code line
 * @param length Length of the input buffer
 * @param commandQueue Pointer to GCODE_CommandQueue to store parsed commands
 * @return Pointer to updated GCODE_CommandQueue
 */

GCODE_CommandQueue* Extract_CommandLineFrom_Buffer(uint8_t* buffer, uint32_t length, GCODE_CommandQueue* commandQueue)
{
    GCODE_CommandQueue* cmdQueue = commandQueue;
    
    // Null-terminate buffer for string processing
    char line_buffer[256];
    uint32_t safe_length = (length < sizeof(line_buffer) - 1) ? length : sizeof(line_buffer) - 1;
    memcpy(line_buffer, buffer, safe_length);
    line_buffer[safe_length] = '\0';

    // Normalize to uppercase to simplify parsing (G/M/X/Y/Z/A/F/S/T/I/J/K/P)
    for (uint32_t i = 0; i < safe_length; i++) {
        char c = line_buffer[i];
        if (c >= 'a' && c <= 'z') {
            line_buffer[i] = (char)(c - 'a' + 'A');
        }
    }
    
    // ✅ Strip non-printable control characters (except CR/LF/TAB/SPACE)
    // This prevents issues with terminal emulators inserting Ctrl+C (0x03) etc.
    uint32_t write_pos = 0;
    for (uint32_t read_pos = 0; read_pos < safe_length; read_pos++) {
        char c = line_buffer[read_pos];
        // Keep printable chars (32-126) and CR/LF/TAB
        if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '\t') {
            line_buffer[write_pos++] = c;
        }
        // Skip control characters (0x00-0x1F except CR/LF/TAB)
    }
    line_buffer[write_pos] = '\0';
    safe_length = write_pos;
    
    // ✅ Tokenize properly: "G90G1X10Y10F1000" → ["G90", "G1X10Y10F1000"]
    // Each G/M command gets ALL its parameters
    TokenArray tokens;
    uint32_t token_count = UTILS_TokenizeGcodeLine(line_buffer, &tokens);

    // Add each token to the command queue
    for (uint32_t i = 0; i < token_count; i++) {
        // Skip empty tokens and comments
        if (UTILS_IsEmptyString(tokens.tokens[i]) || UTILS_IsComment(tokens.tokens[i])) {
            continue;
        }
        
        // Check if queue has space (circular buffer protection)
        if (((cmdQueue->head + 1) % GCODE_MAX_COMMANDS) != cmdQueue->tail) {
            // Copy token to queue
            uint32_t token_len = UTILS_SafeStrlen(tokens.tokens[i], GCODE_BUFFER_SIZE - 1);
            if (token_len > 0) {
                memcpy(cmdQueue->commands[cmdQueue->head].command, 
                       tokens.tokens[i], token_len);
                cmdQueue->commands[cmdQueue->head].command[token_len] = '\0';

                // Advance queue
                cmdQueue->head = (cmdQueue->head + 1) % GCODE_MAX_COMMANDS;
                cmdQueue->count++;
            }
        }
        // Note: If queue full, remaining tokens are dropped (GRBL behavior)
    }
    
    return cmdQueue;
}


/* @brief Process G-code commands from the queue
 * @param cmdQueue Pointer to GCODE_CommandQueue
 * Post process queued gcode commands from UGS
 * @return void
 */

void GCODE_ProcessQueuedCommand(GCODE_CommandQueue* cmdQueue)
{
    // Legacy function - deprecated in favor of GCODE_GetNextEvent()
    // Keeping for backward compatibility during transition
    if (cmdQueue->count == 0) return;
    
    // Simply remove command from queue - event processing handles parsing
    cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
    cmdQueue->count--;
}

/* @brief Get next G-code event from command queue
 * @param cmdQueue Pointer to GCODE_CommandQueue
 * @param event Pointer to GCODE_Event to populate
 * @return true if event available, false if queue empty
 * Clean interface - no APP_DATA exposure, respects abstraction layer
 */
bool GCODE_GetNextEvent(GCODE_CommandQueue* cmdQueue, GCODE_Event* event)
{
    if (!cmdQueue || !event || cmdQueue->count == 0) {
        return false;
    }
    
    // Get next command from queue
    GCODE_Command* current_cmd = &cmdQueue->commands[cmdQueue->tail];

    // Parse command into event
    bool event_valid = parse_command_to_event(current_cmd->command, event);

    // ✅ CRITICAL: Always dequeue command, even if parsing failed
    // Otherwise unparseable commands stay in queue forever causing infinite loops
    cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
    cmdQueue->count--;
    
    return event_valid;
}

void GCODE_Tasks(GCODE_CommandQueue* commandQueue)
{
    GCODE_CommandQueue* cmdQueue = commandQueue;
    uint32_t nBytesAvailable = 0;
    uint32_t cmd_end = 0;  // Track terminator position for buffer shifting
    
    switch (gcodeData.state)
    {
    case GCODE_STATE_IDLE:
        // ✅ CRITICAL: Only read from ring buffer if rxBuffer is EMPTY
        // This prevents overwriting unprocessed data
        if (nBytesRead == 0) {
            nBytesAvailable = UART3_ReadCountGet();
            
            if(nBytesAvailable > 0) {
                // ✅ Only read available bytes (up to buffer limit)
                // rxBuffer size is 250 bytes now
                //only read what uart ring buffer has available to avoid corrupting plib 
                // ring buffer data flow.
                nBytesRead = UART3_Read(rxBuffer, nBytesAvailable);
                rxBuffer[nBytesRead] = '\0';  // Null terminate
            }
        }
        
        // ✅ Process whatever is in rxBuffer (if anything)
        if (nBytesRead > 0) {

            // REMOVED: Debug print was causing infinite loop due to TX buffer overflow
            // DEBUG_EXEC_GCODE({ UART_Printf("[RX: nBytes=%u, first=0x%02X]\r\n", ...); });
            
            // ✅ CRITICAL: Check for control characters FIRST (before terminator check)
            // Real-time chars (`?`, `~`, `!`, 0x18) don't need terminators
            // Settings/query chars (`$`, `G`) need terminators but go to CONTROL_CHAR state
            if(rxBuffer[0] == '?' || rxBuffer[0] == '~' || rxBuffer[0] == '!' || rxBuffer[0] == 0x18 ||
               rxBuffer[0] == '$' || rxBuffer[0] == 'G' || rxBuffer[0] == 'g') {
                DEBUG_PRINT_GCODE("[IDLE] Control/command char detected: 0x%02X\r\n", (unsigned)rxBuffer[0]);
                gcodeData.state = GCODE_STATE_CONTROL_CHAR;
                // DON'T clear nBytesRead here - CONTROL_CHAR state needs the buffer
                // Exit IDLE case immediately - will process in CONTROL_CHAR on next GCODE_Tasks() call
                break;
            }
            
            // ✅ Only process regular commands from here on (control chars already handled above)
            
            // ✅ Find FIRST line terminator (for commands that need it)
            bool has_terminator = false;
            uint32_t terminator_pos = 0;
            for(uint32_t i = 0; i < nBytesRead; i++) {
                if(rxBuffer[i] == '\n' || rxBuffer[i] == '\r') {
                    has_terminator = true;
                    terminator_pos = i;
                    break;
                }
            }
            
            // If no terminator, wait for more data
            if(!has_terminator) {
                break;  // Exit, rxBuffer keeps current data for next iteration
            }
            
            // ✅ CRITICAL: Only process up to FIRST terminator
            // Null-terminate at first terminator (isolates first command)
            rxBuffer[terminator_pos] = '\0';
            
            // ✅ Process first complete command based on first character
            bool command_processed = false;
            
            if(rxBuffer[0] == '$' || rxBuffer[0] == 'G' || rxBuffer[0] == 'g') {
                // Multi-byte command - process immediately
                command_processed = true;
                gcodeData.state = GCODE_STATE_CONTROL_CHAR;
                
            } else {
                // Regular G-code or empty line
                bool has_content = false;
                for(uint32_t i = 0; i < terminator_pos; i++) {
                    if(rxBuffer[i] != ' ' && rxBuffer[i] != '\t' && rxBuffer[i] != 0) {
                        has_content = true;
                        break;
                    }
                }
                
                if(has_content) {
                    cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, terminator_pos, cmdQueue);
                    
                    if (!okPending && cmdQueue->motionQueueCount < (cmdQueue->maxMotionSegments - MOTION_BUFFER_THRESHOLD)) {
                        UART_SendOK();
                    } else {
                        okPending = true;
                    }
                }
                command_processed = true;
            }
            
            if(command_processed) {
                // ✅ CRITICAL: Only shift buffer if staying in IDLE state
                // If state changed (e.g., CONTROL_CHAR), leave buffer intact for target state
                if(gcodeData.state == GCODE_STATE_IDLE) {
                    // ✅ Shift remaining bytes to start of buffer
                    // Skip past terminator (and any following CR/LF)
                    uint32_t skip_pos = terminator_pos + 1;
                    while(skip_pos < nBytesRead && (rxBuffer[skip_pos] == '\r' || rxBuffer[skip_pos] == '\n')) {
                        skip_pos++;
                    }
                    
                    uint32_t remaining_bytes = nBytesRead - skip_pos;
                    if(remaining_bytes > 0) {
                        // Move remaining data to start of buffer
                        memmove(rxBuffer, &rxBuffer[skip_pos], remaining_bytes);
                        nBytesRead = remaining_bytes;
                        rxBuffer[nBytesRead] = '\0';
                    } else {
                        // No remaining data - clear buffer
                        nBytesRead = 0;
                        memset(rxBuffer, 0, sizeof(rxBuffer));
                    }
                }
                // If state changed to CONTROL_CHAR, buffer preserved for processing
            }
            break;
            
        } else {
            // No bytes in rxBuffer - check for pending OK
            if (okPending && 
                cmdQueue->motionQueueCount < (cmdQueue->maxMotionSegments - MOTION_BUFFER_THRESHOLD)) {
                UART_SendOK();
                okPending = false;
            }
            
            gcodeData.state = GCODE_STATE_IDLE;
            break;
        }
        break;  // End of GCODE_STATE_IDLE
    
case GCODE_STATE_CONTROL_CHAR:
    {
    /* Handle specific control characters per GRBL protocol */
    DEBUG_PRINT_GCODE("[CTRL] rxBuffer[0]=0x%02X\r\n", (unsigned)rxBuffer[0]);
    bool buffer_already_cleared = false;  // Flag to skip shared buffer shifting
    switch(rxBuffer[0]) {
        case '?':  // Status query - NO "OK" response
        {
            DEBUG_PRINT_GCODE("[?] Processing status query\r\n");
            StepperPosition* pos = STEPPER_GetPosition();
            WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();
            
            DEBUG_PRINT_GCODE("[STATUS] Raw steps: X=%ld Y=%ld Z=%ld\r\n", 
                             (long)pos->x_steps, (long)pos->y_steps, (long)pos->z_steps);
            DEBUG_PRINT_GCODE("[STATUS] Steps/mm: X=%.2f Y=%.2f Z=%.2f\r\n",
                             pos->steps_per_mm_x, pos->steps_per_mm_y, pos->steps_per_mm_z);
            
            float mpos_x = (float)pos->x_steps / pos->steps_per_mm_x;
            float mpos_y = (float)pos->y_steps / pos->steps_per_mm_y;
            float mpos_z = (float)pos->z_steps / pos->steps_per_mm_z;
            
            DEBUG_PRINT_GCODE("[STATUS] MPos floats: X=%.3f Y=%.3f Z=%.3f\r\n",
                             mpos_x, mpos_y, mpos_z);
            
            float wpos_x = mpos_x - wcs->offset.x;
            float wpos_y = mpos_y - wcs->offset.y;
            float wpos_z = mpos_z - wcs->offset.z;

            // Derive state and FS from appData runtime info
            extern APP_DATA appData;
            // Refined state derivation: Only report Run if an active segment still has steps remaining.
            const char* state = "Idle";
            if (appData.currentSegment != NULL) {
                if (appData.currentSegment->steps_completed < appData.currentSegment->steps_remaining) {
                    state = "Run";
                } else {
                    // Segment pointer not cleared yet but logically complete.
                    // Opportunistically clear to keep status accurate.
                    appData.currentSegment = NULL;
                    appData.motionPhase = MOTION_PHASE_IDLE;
                }
            } else if (appData.motionQueueCount > 0) {
                state = "Run"; // Pending segments waiting to load
            }
            float feedrate_mm_min = (state[0] == 'R') ? appData.modalFeedrate : 0.0f;
            // If we are paradoxically in Run with zero feed (e.g. first move sans F), apply modal fallback.
            if (state[0] == 'R' && feedrate_mm_min <= 0.0f) {
                feedrate_mm_min = (appData.modalFeedrate > 0.0f) ? appData.modalFeedrate : 600.0f;
                appData.modalFeedrate = feedrate_mm_min; // Persist the chosen default
            }
            uint32_t spindle_rpm = appData.modalSpindleRPM;
            
            // Send GRBL-compatible status response (NO "OK")
            nBytesRead = snprintf((char*)txBuffer, sizeof(txBuffer),
                                "<%s|MPos:%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f|FS:%.0f,%u>\r\n",
                                state, mpos_x, mpos_y, mpos_z, wpos_x, wpos_y, wpos_z,
                                feedrate_mm_min, (unsigned)spindle_rpm);
            UART3_Write((uint8_t*)txBuffer, nBytesRead);
            
            // ✅ CRITICAL: Clear buffer after status query (real-time command)
            nBytesRead = 0;
            memset(rxBuffer, 0, sizeof(rxBuffer));
            gcodeData.state = GCODE_STATE_IDLE;
            buffer_already_cleared = true;
            break;
        }
        case '~':  // Cycle start/resume - NO response (real-time command)
            // TODO: Implement cycle start functionality
            
            // ✅ Clear buffer after real-time command
            nBytesRead = 0;
            memset(rxBuffer, 0, sizeof(rxBuffer));
            gcodeData.state = GCODE_STATE_IDLE;
            buffer_already_cleared = true;
            break;
        case '!':  // Feed hold - NO response (real-time command)
            // TODO: Implement feed hold functionality
            
            // ✅ Clear buffer after real-time command
            nBytesRead = 0;
            memset(rxBuffer, 0, sizeof(rxBuffer));
            gcodeData.state = GCODE_STATE_IDLE;
            buffer_already_cleared = true;
            break;
        case 0x18: // Soft reset (Ctrl+X)
        
            // ✅ CRITICAL: Stop all motion immediately
            STEPPER_DisableAll();  // Disable stepper outputs
            
            // ✅ Clear motion queue
            extern APP_DATA appData;  // Access app data
            appData.motionQueueCount = 0;
            appData.motionQueueHead = 0;
            appData.motionQueueTail = 0;
            appData.currentSegment = NULL;  // ✅ CRITICAL: Clear segment pointer!
            
            // ✅ Clear G-code queue
            cmdQueue->count = 0;
            cmdQueue->head = 0;
            cmdQueue->tail = 0;
            
            // ✅ Stop arc generation if active
            appData.arcGenState = ARC_GEN_IDLE;
            
            // ✅ Reset motion phase
            appData.motionPhase = MOTION_PHASE_IDLE;

            // ✅ Reset modal and coordinate state for a clean start
            appData.absoluteMode = true;          // G90
            appData.modalFeedrate = 0.0f;         // No feed until set
            appData.modalSpindleRPM = 0;
            appData.modalToolNumber = 0;
            appData.currentX = 0.0f;
            appData.currentY = 0.0f;
            appData.currentZ = 0.0f;
            appData.currentA = 0.0f;
            // Reset work coordinate offset
            KINEMATICS_SetWorkOffset(0.0f, 0.0f, 0.0f);

            // ✅ Disable OC interrupts explicitly (safety) - use correct modules
            IEC0CLR = _IEC0_OC5IE_MASK;  // X
            IEC0CLR = _IEC0_OC1IE_MASK;  // Y
            IEC0CLR = _IEC0_OC3IE_MASK;  // Z
            IEC0CLR = _IEC0_OC4IE_MASK;  // A
            
            // ✅ Send startup banner (PLIB ring buffer handles transmission)
            UART3_Write((uint8_t*)GRBL_FIRMWARE_VERSION, sizeof(GRBL_FIRMWARE_VERSION));
            
            // ✅ Transition to safe state
            appData.state = APP_IDLE;
            
            break;
            
        // ✅ NEW: Settings commands handler
        case '$':  // Settings commands
        {
            // ✅ Settings commands need complete line - check if we have terminator
            bool has_terminator = false;
            
            // Check if we already have terminator in buffer
            for(uint32_t i = 0; i < nBytesRead; i++){
                if(rxBuffer[i] == '\n' || rxBuffer[i] == '\r'){
                    has_terminator = true;
                    break;
                }
            }
            
            // If no terminator yet, try to read more (but don't block)
            if(!has_terminator){
                uint32_t bytes_available = UART3_ReadCountGet();
                
                if(bytes_available > 0){
                    // Read more bytes into rxBuffer
                    uint32_t space_available = sizeof(rxBuffer) - nBytesRead - 1;
                    uint32_t bytes_to_read = (bytes_available < space_available) ? bytes_available : space_available;
                    
                    if(bytes_to_read > 0){
                        uint32_t new_bytes = UART3_Read((uint8_t*)&rxBuffer[nBytesRead], bytes_to_read);
                        nBytesRead += new_bytes;
                        
                        // Check again for terminator
                        for(uint32_t i = 0; i < nBytesRead; i++){
                            if(rxBuffer[i] == '\n' || rxBuffer[i] == '\r'){
                                has_terminator = true;
                                break;
                            }
                        }
                    }
                }
                
                // If still no terminator, return and wait for more data
                if(!has_terminator){
                    return;  // Keep state as CONTROL_CHAR, will resume on next call
                }
            }
            
            // Now we have complete command, parse it
            // ✅ CRITICAL: Find the terminator position and only process up to it
            // UGS sends commands rapidly - multiple commands may be in buffer
            cmd_end = 0;  // Reset for this command
            for(uint32_t i = 0; i < nBytesRead; i++){
                if(rxBuffer[i] == '\r' || rxBuffer[i] == '\n'){
                    rxBuffer[i] = '\0';  // Null-terminate at first CR/LF
                    cmd_end = i;
                    break;
                }
            }
            
            if(strncmp((char*)rxBuffer, "$$", 2) == 0 && (rxBuffer[2] == '\0' || rxBuffer[2] == '\r' || rxBuffer[2] == '\n')){
                SETTINGS_PrintAll(SETTINGS_GetCurrent());
                // Note: SETTINGS_PrintAll() sends "ok" internally
                
                // ✅ Clear buffer after settings command
                nBytesRead = 0;
                memset(rxBuffer, 0, sizeof(rxBuffer));
                gcodeData.state = GCODE_STATE_IDLE;
                buffer_already_cleared = true;
                
            } else if(strncmp((char*)rxBuffer, "$I", 2) == 0 && (rxBuffer[2] == '\0' || rxBuffer[2] == '\r' || rxBuffer[2] == '\n')){
                SETTINGS_PrintBuildInfo();
                // Note: SETTINGS_PrintBuildInfo() sends "ok" internally
                
                // ✅ Clear buffer after build info command
                nBytesRead = 0;
                memset(rxBuffer, 0, sizeof(rxBuffer));
                gcodeData.state = GCODE_STATE_IDLE;
                buffer_already_cleared = true;
                
            } else if(strncmp((char*)rxBuffer, "$G", 2) == 0 && (rxBuffer[2] == '\0' || rxBuffer[2] == '\r' || rxBuffer[2] == '\n')){
                // ✅ GRBL $G command - print parser state (modal settings)
                extern APP_DATA appData;
                char state_buffer[128];
                int len = 0;
                
                // Format: [GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]
                len += sprintf(&state_buffer[len], "[GC:");
                len += sprintf(&state_buffer[len], "G0 ");  // Motion mode (G0=rapid, G1=linear)
                len += sprintf(&state_buffer[len], "G54 "); // Work coordinate system
                len += sprintf(&state_buffer[len], "G%d ", appData.modalPlane == 0 ? 17 : (appData.modalPlane == 1 ? 18 : 19)); // Plane
                len += sprintf(&state_buffer[len], "G21 "); // Units (mm)
                len += sprintf(&state_buffer[len], "G%d ", appData.absoluteMode ? 90 : 91); // Distance mode
                len += sprintf(&state_buffer[len], "G94 "); // Feed rate mode
                len += sprintf(&state_buffer[len], "M5 ");  // Spindle off
                len += sprintf(&state_buffer[len], "M9 ");  // Coolant off
                len += sprintf(&state_buffer[len], "T%u ", appData.modalToolNumber); // Tool
                len += sprintf(&state_buffer[len], "F%.1f ", appData.modalFeedrate); // Feedrate
                len += sprintf(&state_buffer[len], "S%u", appData.modalSpindleRPM); // Spindle RPM
                len += sprintf(&state_buffer[len], "]\r\n");
                
                // ✅ Ensure complete transmission
                size_t total_sent = 0;
                while(total_sent < len) {
                    size_t sent = UART3_Write((uint8_t*)&state_buffer[total_sent], len - total_sent);
                    total_sent += sent;
                    if(sent == 0 && total_sent < len) {
                        for(volatile uint32_t i = 0; i < 1000; i++);
                    }
                }
                UART_SendOK();  // ✅ GRBL system.c returns STATUS_OK for $G command
                
            } else if(strncmp((char*)rxBuffer, "$H", 2) == 0){
                // ✅ GRBL $H command - start homing cycle
                extern APP_DATA appData;
                
                // Check if homing is enabled ($22)
                CNC_Settings* settings = SETTINGS_GetCurrent();
                if(!settings->homing_enable){
                    UART3_Write((uint8_t*)"error:22\r\n", 10);  // Error 22 = Homing not enabled
                } else {
                    // Start homing for all axes enabled in $23 mask
                    uint32_t axes_to_home = 0;
                    
                    // Build axis mask from $23 homing_dir_mask
                    // If bit is set in $23, home that axis
                    // GRBL homes axes with non-zero bits in $23
                    // For simplicity, home all axes (X=bit0, Y=bit1, Z=bit2, A=bit3)
                    // TODO: Make this configurable via $23 interpretation
                    axes_to_home = 0x07;  // Home X, Y, Z (default GRBL behavior)
                    
                    if(HOMING_Start(axes_to_home)){
                        UART_SendOK();  // Homing started
                    } else {
                        UART3_Write((uint8_t*)"error:23\r\n", 10);  // Error 23 = Homing fail
                    }
                }
                
            } else if(strncmp((char*)rxBuffer, "$RST=$", 6) == 0){
                // ✅ Restore defaults to RAM only
                SETTINGS_RestoreDefaults(SETTINGS_GetCurrent());
                UART_SendOK();
                
            } else if(nBytesRead >= 4 && strncmp((char*)rxBuffer, "$WR", 3) == 0){
                // ✅ NEW: Explicit write command to save settings to NVM
                if(SETTINGS_SaveToFlash(SETTINGS_GetCurrent())){
                    UART_SendOK();
                } else {
                    UART3_Write((uint8_t*)"error:9\r\n", 9);  // Error 9 = Settings write failed
                }
                
            } else {
                // Parse $<n>=<val> or $<n>
                uint32_t param = 0;
                float value = 0.0f;
                char* equals = strchr((char*)rxBuffer, '=');
                
                if(equals){
                    // ✅ Set parameter in RAM only - NO auto-save
                    sscanf((char*)&rxBuffer[1], "%u=%f", (unsigned int*)&param, &value);
                    
                    if(SETTINGS_SetValue(SETTINGS_GetCurrent(), param, value)){
                        // ✅ Value updated in RAM only - user must send $WR to persist
                        UART_SendOK();
                    } else {
                        UART3_Write((uint8_t*)"error:3\r\n", 9);  // Error 3 = Invalid parameter
                    }
                    
                } else {
                    // Get parameter value (read from RAM)
                    sscanf((char*)&rxBuffer[1], "%u", (unsigned int*)&param);
                    
                    char buffer[64];
                    float val = SETTINGS_GetValue(SETTINGS_GetCurrent(), param);
                    
                    // Check if parameter is valid
                    if(val != 0.0f || param == 31 || param == 4 || param == 5 || param == 2 || param == 3){
                        snprintf(buffer, sizeof(buffer), "$%u=%.3f\r\n", (unsigned int)param, val);
                        UART3_Write((uint8_t*)buffer, strlen(buffer));
                        UART_SendOK();
                    } else {
                        UART3_Write((uint8_t*)"error:3\r\n", 9);
                    }
                }
            }
            break;
        }
            
        case 'G':
        case 'g':
        case 'M':
        case 'm':
        {
            // ✅ Non-blocking full-line read for G/M commands (UGS pipelines without relying on first-char)
            bool has_terminator = false;
            for(uint32_t i = 0; i < nBytesRead; i++){
                if(rxBuffer[i] == '\n' || rxBuffer[i] == '\r'){
                    has_terminator = true;
                    break;
                }
            }

            if(!has_terminator){
                uint32_t bytes_available = UART3_ReadCountGet();
                if(bytes_available > 0){
                    uint32_t space_available = sizeof(rxBuffer) - nBytesRead - 1;
                    uint32_t bytes_to_read = (bytes_available < space_available) ? bytes_available : space_available;
                    if(bytes_to_read > 0){
                        uint32_t new_bytes = UART3_Read((uint8_t*)&rxBuffer[nBytesRead], bytes_to_read);
                        nBytesRead += new_bytes;
                        for(uint32_t i = 0; i < nBytesRead; i++){
                            if(rxBuffer[i] == '\n' || rxBuffer[i] == '\r'){
                                has_terminator = true;
                                break;
                            }
                        }
                    }
                }

                if(!has_terminator){
                    // Wait for more data - keep CONTROL_CHAR state
                    return;
                }
            }

            // Find terminator and isolate command
            cmd_end = 0;
            for(uint32_t i = 0; i < nBytesRead; i++){
                if(rxBuffer[i] == '\r' || rxBuffer[i] == '\n'){
                    rxBuffer[i] = '\0';
                    cmd_end = i;
                    break;
                }
            }

            // Queue the full G/M line (handles multi-token G/M lines)
            cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, cmd_end, cmdQueue);

            if (!okPending && cmdQueue->motionQueueCount < (cmdQueue->maxMotionSegments - MOTION_BUFFER_THRESHOLD)) {
                UART_SendOK();
            } else {
                okPending = true;
            }

            // Shift remaining bytes (if any)
            if(nBytesRead > (cmd_end + 1)){
                uint32_t bytes_after_cmd = nBytesRead - (cmd_end + 1);
                memmove(rxBuffer, &rxBuffer[cmd_end + 1], bytes_after_cmd);
                nBytesRead = bytes_after_cmd;
            } else {
                nBytesRead = 0;
            }

            gcodeData.state = GCODE_STATE_IDLE;
            break;
        }

        default:
            // Unknown control char - silently ignore (GRBL behavior)
            break;
    }

    // ✅ CRITICAL: Shift remaining bytes after the processed command
    // UGS sends commands rapidly - next command may already be in buffer
    // Skip this if buffer was already cleared by real-time command handler
    if(!buffer_already_cleared) {
        uint32_t bytes_after_cmd = nBytesRead - (cmd_end + 1);  // Skip past terminator
        if(bytes_after_cmd > 0) {
        // Move remaining bytes to start of buffer
        memmove(rxBuffer, &rxBuffer[cmd_end + 1], bytes_after_cmd);
        nBytesRead = bytes_after_cmd;
        
        // ✅ Check if remaining bytes are only whitespace/terminators
        bool only_whitespace = true;
        for(uint32_t i = 0; i < nBytesRead; i++) {
            if(rxBuffer[i] != '\r' && rxBuffer[i] != '\n' && 
               rxBuffer[i] != ' ' && rxBuffer[i] != '\t' && rxBuffer[i] != 0) {
                only_whitespace = false;
                break;
            }
        }
        
        if(only_whitespace) {
            // Only whitespace left - clear buffer completely
            nBytesRead = 0;
            memset(rxBuffer, 0, sizeof(rxBuffer));
        }
        
        // ✅ CRITICAL: Return to IDLE so next iteration re-evaluates command type
        gcodeData.state = GCODE_STATE_IDLE;
    } else {
        // No remaining data - clear buffer and return to idle
        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        gcodeData.state = GCODE_STATE_IDLE;
    }
    }  // End of buffer_already_cleared check
    }  // End of GCODE_STATE_CONTROL_CHAR scope
    break;

    case GCODE_STATE_ERROR:
        // Send GRBL-style error message
        nBytesRead = sprintf((char*)txBuffer, "error:1\r\n"); // Error code 1 = G-code syntax error
        UART3_Write((uint8_t*)txBuffer, nBytesRead);
        
        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        gcodeData.state = GCODE_STATE_IDLE;
    break;
    default:
        break;
    }
}

/* Parse coordinate values from G-code command
 * @param command G-code command string
 * @return CoordinatePoint with extracted values
 */
CoordinatePoint parse_coordinate_values(const char* command)
{
    // ✅ Use NAN to indicate "not specified" - allows detection in event handler
    CoordinatePoint coords = {NAN, NAN, NAN, NAN};  // Initialize to NAN
    
    char* x_ptr = strstr(command, "X");
    char* y_ptr = strstr(command, "Y"); 
    char* z_ptr = strstr(command, "Z");
    char* a_ptr = strstr(command, "A");
    
    if (x_ptr) coords.x = atof(x_ptr + 1);
    if (y_ptr) coords.y = atof(y_ptr + 1);
    if (z_ptr) coords.z = atof(z_ptr + 1);
    if (a_ptr) coords.a = atof(a_ptr + 1);
    
    return coords;
}

/* Parse feedrate from G-code command  
 * @param command G-code command string
 * @return Feedrate value in units/min
 */
float parse_feedrate(const char* command)
{
    char* f_ptr = strstr(command, "F");
    if (f_ptr) {
        return atof(f_ptr + 1);
    }
    return 0.0f;  // Default feedrate if not specified
}

/* Parse G-code command into event structure
 * @param command G-code command string
 * @param event Pointer to GCODE_Event to populate
 * @return true if command parsed successfully, false otherwise
 */
bool parse_command_to_event(const char* command, GCODE_Event* event)
{
    if (!command || !event) {
        return false;
    }
    
    DEBUG_PRINT_GCODE("[GCODE] parse_command_to_event: '%s'\r\n", command);
    
    // Initialize event
    event->type = GCODE_EVENT_NONE;
    
    // Parse G-code commands
    if (strstr(command, "G90")) {
        event->type = GCODE_EVENT_SET_ABSOLUTE;
        return true;
    }
    else if (strstr(command, "G91")) {
        event->type = GCODE_EVENT_SET_RELATIVE;
        return true;
    }
    else if (strstr(command, "G10")) {
        // G10 L2 Pn - Set coordinate system offset (L2 = use current position, L20 = use specified position)
        // G10 L2 P0 - Sets G54 origin (P0=G54, P1=G55, etc.)
        // G10 L20 P0 X0 Y0 Z0 - Sets G54 origin to specified values in current work coordinate system
        
        // Parse L value (operation type)
        const char* l_ptr = strchr(command, 'L');
        uint32_t l_value = 2;  // Default to L2
        if (l_ptr != NULL) {
            l_value = (uint32_t)strtol(l_ptr + 1, NULL, 10);
        }
        
        // Parse P value (coordinate system selector: P0=G54, P1=G55, etc.)
        const char* p_ptr = strchr(command, 'P');
        uint32_t p_value = 0;  // Default to P0 (G54)
        if (p_ptr != NULL) {
            p_value = (uint32_t)strtol(p_ptr + 1, NULL, 10);
        }
        
        // For now, only support P0 (G54) - main work coordinate system
        if (p_value != 0) {
            return false;  // Unsupported coordinate system (G55-G59 not implemented yet)
        }
        
        // L2 = Set origin to current position (like G92 but for coordinate system)
        // L20 = Set origin to specified position
        if (l_value == 2 || l_value == 20) {
            event->type = GCODE_EVENT_SET_WORK_OFFSET;
            
            // Extract coordinates
            CoordinatePoint coords = parse_coordinate_values(command);
            event->data.setWorkOffset.x = coords.x;
            event->data.setWorkOffset.y = coords.y;
            event->data.setWorkOffset.z = coords.z;
            event->data.setWorkOffset.a = coords.a;
            
            return true;
        }
        
        return false;  // Unsupported L value
    }
    else if (strstr(command, "G92")) {
        // G92 - Set work coordinate system (set current position)
        event->type = GCODE_EVENT_SET_WORK_OFFSET;
        
        // Extract coordinates
        CoordinatePoint coords = parse_coordinate_values(command);
        event->data.setWorkOffset.x = coords.x;
        event->data.setWorkOffset.y = coords.y;
        event->data.setWorkOffset.z = coords.z;
        event->data.setWorkOffset.a = coords.a;
        
        return true;
    }
    else if (strstr(command, "G1") || strstr(command, "G01")) {
        // Linear motion command
        event->type = GCODE_EVENT_LINEAR_MOVE;
        
        // Extract coordinates
        CoordinatePoint coords = parse_coordinate_values(command);
        event->data.linearMove.x = coords.x;
        event->data.linearMove.y = coords.y;
        event->data.linearMove.z = coords.z;
        event->data.linearMove.a = coords.a;
        
        // Extract feedrate
        event->data.linearMove.feedrate = parse_feedrate(command);
        
        return true;
    }
    else if (strstr(command, "G2") || strstr(command, "G02")) {
        // Clockwise arc
        event->type = GCODE_EVENT_ARC_MOVE;
        event->data.arcMove.clockwise = true;
        
        // Extract coordinates
        CoordinatePoint coords = parse_coordinate_values(command);
        event->data.arcMove.x = coords.x;
        event->data.arcMove.y = coords.y;
        event->data.arcMove.z = coords.z;
        event->data.arcMove.a = coords.a;
        
        // Extract arc center (I, J parameters)
        char* i_ptr = strstr(command, "I");
        char* j_ptr = strstr(command, "J");
        event->data.arcMove.centerX = i_ptr ? atof(i_ptr + 1) : 0.0f;
        event->data.arcMove.centerY = j_ptr ? atof(j_ptr + 1) : 0.0f;
        
        event->data.arcMove.feedrate = parse_feedrate(command);
        return true;
    }
    else if (strstr(command, "G3") || strstr(command, "G03")) {
        // Counter-clockwise arc
        event->type = GCODE_EVENT_ARC_MOVE;
        event->data.arcMove.clockwise = false;
        
        // Extract coordinates  
        CoordinatePoint coords = parse_coordinate_values(command);
        event->data.arcMove.x = coords.x;
        event->data.arcMove.y = coords.y;
        event->data.arcMove.z = coords.z;
        event->data.arcMove.a = coords.a;
        
        // Extract arc center
        char* i_ptr = strstr(command, "I");
        char* j_ptr = strstr(command, "J");
        event->data.arcMove.centerX = i_ptr ? atof(i_ptr + 1) : 0.0f;
        event->data.arcMove.centerY = j_ptr ? atof(j_ptr + 1) : 0.0f;
        
        event->data.arcMove.feedrate = parse_feedrate(command);
        return true;
    }
    else if (strstr(command, "G4") || strstr(command, "G04")) {
        // Dwell command
        event->type = GCODE_EVENT_DWELL;
        char* p_ptr = strstr(command, "P");
        event->data.dwell.seconds = p_ptr ? atof(p_ptr + 1) : 0.0f;
        return true;
    }
    else if (strstr(command, "M3") || strstr(command, "M03")) {
        // Spindle on clockwise
        event->type = GCODE_EVENT_SPINDLE_ON;
        char* s_ptr = strstr(command, "S");
        event->data.spindle.rpm = s_ptr ? (uint32_t)atoi(s_ptr + 1) : 1000;
        return true;
    }
    else if (strstr(command, "M5") || strstr(command, "M05")) {
        // Spindle off
        event->type = GCODE_EVENT_SPINDLE_OFF;
        return true;
    }
    else if (strstr(command, "M7") || strstr(command, "M07")) {
        // Coolant on
        event->type = GCODE_EVENT_COOLANT_ON;
        return true;
    }
    else if (strstr(command, "M9") || strstr(command, "M09")) {
        // Coolant off
        event->type = GCODE_EVENT_COOLANT_OFF;
        return true;
    }
    // ✅ Standalone modal parameter commands (LinuxCNC/GRBL compatible)
    else if (command[0] == 'F') {
        // Standalone feedrate change: "F1500"
        event->type = GCODE_EVENT_SET_FEEDRATE;
        event->data.setFeedrate.feedrate = atof(command + 1);
        return true;
    }
    else if (command[0] == 'S') {
        // Standalone spindle speed change: "S2000"
        event->type = GCODE_EVENT_SET_SPINDLE_SPEED;
        event->data.setSpindleSpeed.rpm = (uint32_t)atoi(command + 1);
        return true;
    }
    else if (command[0] == 'T') {
        // Tool change: "T1"
        event->type = GCODE_EVENT_SET_TOOL;
        event->data.setTool.toolNumber = (uint32_t)atoi(command + 1);
        return true;
    }
    
    // Command not recognized or not implemented
    DEBUG_PRINT_GCODE("[GCODE] parse_command_to_event: FAILED to parse '%s'\r\n", command);
    return false;
}

