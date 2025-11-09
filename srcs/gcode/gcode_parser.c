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
    // NOTE: This function is currently unused - $ commands are handled directly in GCODE_Tasks
    // Kept for future use if we need event-based processing
    (void)cmdQueue;  // Suppress unused warning
    (void)event;
    return false;
    
#if 0  // Disabled - not using event-based parsing anymore
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
#endif
}

void GCODE_Tasks(GCODE_CommandQueue* commandQueue)
{
    GCODE_CommandQueue* cmdQueue = commandQueue;
    uint32_t nBytesAvailable = 0;
    
    switch (gcodeData.state)
    {
    case GCODE_STATE_IDLE:
        // ✅ CRITICAL: Only read from ring buffer if rxBuffer is EMPTY
        // This prevents overwriting unprocessed data
        if (nBytesRead == 0) {
            nBytesAvailable = UART3_ReadCountGet();
            
            if(nBytesAvailable > 0) {
                LED2_Toggle();
                // ✅ Only read available bytes (up to buffer limit)
                // rxBuffer size is 250 bytes now
                //only read what uart ring buffer has available to avoid corrupting plib 
                // ring buffer data flow.
                nBytesRead = UART3_Read(rxBuffer, nBytesAvailable);
               // rxBuffer[nBytesRead] = '\0';  // Null terminate
            }
        }
        
        // ✅ Process whatever is in rxBuffer (if anything)
        if (nBytesRead > 0) {

            // REMOVED: Debug print was causing infinite loop due to TX buffer overflow
            // DEBUG_EXEC_GCODE({ UART_Printf("[RX: nBytes=%u, first=0x%02X]\r\n", ...); });
            
            // ✅ CRITICAL: Check for REAL-TIME control characters FIRST (no terminator needed)
            // Real-time chars (`?`, `~`, `!`, 0x18) process immediately without waiting for terminators
            // NOTE: `$` is NOT real-time - it needs terminator to distinguish $, $$, $I, $G, etc.
            if(rxBuffer[0] == '?' || rxBuffer[0] == '~' || rxBuffer[0] == '!' || rxBuffer[0] == 0x18) {
                gcodeData.state = GCODE_STATE_CONTROL_CHAR;            
            }
            else if(rxBuffer[0] == '$') {
                // `$` commands need terminators - fall through to line buffering
                // Handled in next block
                gcodeData.state = GCODE_STATE_QUERY_CHARS;
                break;
            }
            // ✅ All other commands (including `$`) need terminators - fall through to line buffering
            else {
                
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
    
                // ✅ CRITICAL: Null-terminate at first terminator BEFORE processing
                rxBuffer[terminator_pos] = '\0';
                
                // ✅ Process first complete command based on first character
                bool command_processed = false;
                
                if(rxBuffer[0] == '$') {
                    // Query command - process in QUERY_CHARS state (needs terminator)
                    command_processed = true;
                    gcodeData.state = GCODE_STATE_QUERY_CHARS;
                    
                } else if(rxBuffer[0] == 'G' || rxBuffer[0] == 'g' || rxBuffer[0] == 'M' || rxBuffer[0] == 'm') {
                    // G/M command - process in GCODE_COMMAND state
                    command_processed = true;
                    gcodeData.state = GCODE_STATE_GCODE_COMMAND;
                    
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
                    // If state changed (e.g., QUERY_CHARS, GCODE_COMMAND), leave buffer intact for target state
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
                    // If state changed to QUERY_CHARS or GCODE_COMMAND, buffer preserved for processing
                
                
                // Check for pending OK when no data to process
                if (okPending && 
                    cmdQueue->motionQueueCount < (cmdQueue->maxMotionSegments - MOTION_BUFFER_THRESHOLD)) {
                    UART_SendOK();
                    okPending = false;
                }
                
                break;  // Exit IDLE case
            }
        }  // Close else block
    }  // Close if (nBytesRead > 0)
        
    case GCODE_STATE_CONTROL_CHAR:
    {
    /* Handle specific control characters per GRBL protocol */
    
    // ✅ CRITICAL: Declare flag BEFORE switch so all cases can set it
    bool buffer_already_cleared __attribute__((unused)) = false;
    
    switch(rxBuffer[0]) {
        case '?':  // Status query - NO "OK" response
        {
            StepperPosition* pos = STEPPER_GetPosition();
            WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();
            
            float mpos_x = (float)pos->x_steps / pos->steps_per_mm_x;
            float mpos_y = (float)pos->y_steps / pos->steps_per_mm_y;
            float mpos_z = (float)pos->z_steps / pos->steps_per_mm_z;
            
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
            uint32_t response_len = snprintf((char*)txBuffer, sizeof(txBuffer),
                                "<%s|MPos:%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f|FS:%.0f,%u>\r\n",
                                state, mpos_x, mpos_y, mpos_z, wpos_x, wpos_y, wpos_z,
                                feedrate_mm_min, (unsigned)spindle_rpm);
            UART3_Write((uint8_t*)txBuffer, response_len);
            
            break;
        }
        case '~':  // Cycle start/resume - NO response (real-time command)
            // TODO: Implement cycle start functionality
            
            break;
        case '!':  // Feed hold - NO response (real-time command)
            // TODO: Implement feed hold functionality
            
            break;
        case 0x18: // Soft reset (Ctrl+X)
        
            // ✅ CRITICAL: Stop all motion immediately
            STEPPER_DisableAll();  // Disable stepper outputs
            extern APP_DATA appData;
            UART_SoftReset(&appData,cmdQueue);  // Reset UART buffers and state
            
            // ✅ Send startup banner (PLIB ring buffer handles transmission)
            UART3_Write((uint8_t*)GRBL_FIRMWARE_VERSION, sizeof(GRBL_FIRMWARE_VERSION));
            
            // ✅ Transition to safe state
            appData.state = APP_IDLE;
            break;
            
        default:
            // Unknown control char - silently ignore (GRBL behavior
            break;
        }  // Close switch(rxBuffer[0])

        // ✅ CRITICAL: Clear buffer after status query (real-time command)
            nBytesRead = 0;
            memset(rxBuffer, 0, sizeof(rxBuffer));
            gcodeData.state = GCODE_STATE_IDLE;
            buffer_already_cleared = true;
        
        // End of GCODE_STATE_CONTROL_CHAR scope
        break;
    }  // Close case GCODE_STATE_CONTROL_CHAR

    case GCODE_STATE_QUERY_CHARS:
    {
        // ✅ Query commands handler ($, $$, $I, $G, etc.)
        // NOTE: Buffer already prepared by IDLE state - has terminator and is null-terminated
        // These are NOT real-time - they need terminators and can send multi-line responses
        
       // if(rxBuffer[0] != '$') {
       //     break;  // Safety check - should not happen
       // }
            // Send GRBL-compatible status response (NO "OK")
            uint32_t response_len = snprintf((char*)txBuffer, sizeof(txBuffer),
                                "<%s>\r\n",
                                rxBuffer);
            UART3_Write((uint8_t*)txBuffer, response_len);

        switch(rxBuffer[1]) {
            case '$':  // List all settings
                SETTINGS_PrintAll(SETTINGS_GetCurrent());
                // Note: SETTINGS_PrintAll() sends "ok" internally

                break;

            case '\0':  // Single $ = help command
            case '\r':
            case '\n':
              //  UART_PrintHelp();

                break;

            case 'I':  // Build info
                SETTINGS_PrintBuildInfo();
                // Note: SETTINGS_PrintBuildInfo() sends "ok" internally
                break;
                
            case 'G':  // Print parser state (modal settings)
            {
                extern APP_DATA appData;
                char state_buffer[128];
                int len = 0;
                
                len += sprintf(&state_buffer[len], "[GC:");
                len += sprintf(&state_buffer[len], "G0 ");
                len += sprintf(&state_buffer[len], "G54 ");
                len += sprintf(&state_buffer[len], "G%d ", appData.modalPlane == 0 ? 17 : (appData.modalPlane == 1 ? 18 : 19));
                len += sprintf(&state_buffer[len], "G21 ");
                len += sprintf(&state_buffer[len], "G%d ", appData.absoluteMode ? 90 : 91);
                len += sprintf(&state_buffer[len], "G94 ");
                len += sprintf(&state_buffer[len], "M5 ");
                len += sprintf(&state_buffer[len], "M9 ");
                len += sprintf(&state_buffer[len], "T%u ", appData.modalToolNumber);
                len += sprintf(&state_buffer[len], "F%.1f ", appData.modalFeedrate);
                len += sprintf(&state_buffer[len], "S%u", appData.modalSpindleRPM);
                len += sprintf(&state_buffer[len], "]\r\n");
                
                size_t total_sent = 0;
                while(total_sent < len) {
                    size_t sent = UART3_Write((uint8_t*)&state_buffer[total_sent], len - total_sent);
                    total_sent += sent;
                    if(sent == 0 && total_sent < len) {
                        for(volatile uint32_t i = 0; i < 1000; i++);
                    }
                }
                UART_SendOK();
                
                break;
            }
            
            case 'H':  // Start homing cycle
            {
                extern APP_DATA appData;
                CNC_Settings* settings = SETTINGS_GetCurrent();
                if(!settings->homing_enable){
                    UART3_Write((uint8_t*)"error:22\r\n", 10);
                } else {
                    // Homing implementation here
                }               

                break;
            }

            case 'R':  // $RST command
               if(rxBuffer[2] == 'S' && rxBuffer[3] == 'T'){
                    SETTINGS_RestoreDefaults(SETTINGS_GetCurrent());
                    UART_SendOK();
                } else {
                    UART3_Write((uint8_t*)"error:4\r\n", 10);
                }
                
                break;
                
            case 'W':  // $WR command
              if(rxBuffer[2] == 'R'){
                    if(SETTINGS_SaveToFlash(SETTINGS_GetCurrent())){
                        UART_SendOK();
                    } else {
                        UART3_Write((uint8_t*)"error:9\r\n", 9);
                    }
                } else {
                    UART3_Write((uint8_t*)"error:4\r\n", 10);
                }
                
                break;
            
            default:
                // Unknown $ command
                UART3_Write((uint8_t*)"error:4\r\n", 10);
                
                break;
        }

                        
        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        gcodeData.state = GCODE_STATE_IDLE;
        
        break;
    }  // Close case GCODE_STATE_QUERY_CHARS

    case GCODE_STATE_GCODE_COMMAND:
    {
        // ✅ Buffer already prepared by IDLE state - has terminator and is null-terminated
        // Find the null terminator position
        uint32_t cmd_end = 0;
        for(uint32_t i = 0; i < nBytesRead; i++){
            if(rxBuffer[i] == '\0'){
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

        // Clear buffer after processing
        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        
        gcodeData.state = GCODE_STATE_IDLE;
        break;
    }  // Close case GCODE_STATE_GCODE_COMMAND

    case GCODE_STATE_ERROR:
    {
        // Send GRBL-style error message
        uint32_t error_len = sprintf((char*)txBuffer, "error:1\r\n"); // Error code 1 = G-code syntax error
        UART3_Write((uint8_t*)txBuffer, error_len);
        
        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        gcodeData.state = GCODE_STATE_IDLE;
        break;
    }
    
    default:
        break;
   }  // Close switch(gcodeData.state)
}  // Close GCODE_Tasks()


#if 0  // Disabled - helper functions not currently used
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
#endif  // End of disabled helper functions

/* UNUSED - Parse G-code command into event structure
 * Kept for future reference but not currently used
 * @param command G-code command string
 * @param event Pointer to GCODE_Event to populate
 * @return true if command parsed successfully, false otherwise
 */
#if 0  // Disabled - simplified $ command handler doesn't use events
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
            event->data.workOffset.x = coords.x;
            event->data.workOffset.y = coords.y;
            event->data.workOffset.z = coords.z;
            event->data.workOffset.a = coords.a;
            event->data.workOffset.l_value = l_value;  // Store L value for handler
            
            return true;
        }
        
        return false;  // Unsupported L value
    }
    else if (strstr(command, "G92")) {
        // G92 - Set work coordinate system (set current position)
        event->type = GCODE_EVENT_SET_WORK_OFFSET;
        
        // Extract coordinates
        CoordinatePoint coords = parse_coordinate_values(command);
        event->data.workOffset.x = coords.x;
        event->data.workOffset.y = coords.y;
        event->data.workOffset.z = coords.z;
        event->data.workOffset.a = coords.a;
        event->data.workOffset.l_value = 0;  // G92 has no L value
        
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
#endif  // End of disabled parse_command_to_event


