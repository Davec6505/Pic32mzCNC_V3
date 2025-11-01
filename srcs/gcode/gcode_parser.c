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

#include "gcode_parser.h"             
#include "common.h"
#include "stepper.h"
#include "motion.h"
#include "kinematics.h"
#include "utils.h"
// Note: APP_DATA structure access available through parameter - no include needed

/* USART Buffers */
static uint8_t txBuffer[128];
static uint8_t rxBuffer[50];
static volatile uint32_t nBytesRead = 0;
static volatile bool txThresholdEventReceived = false;
static volatile bool rxThresholdEventReceived = false;

/* local datatypes */
GCODE_Data gcodeData = {
    .state = GCODE_STATE_IDLE,
};


/* local scope prototypes */
void usartReadEventHandler(UART_EVENT event, uintptr_t context );
void usartWriteEventHandler(UART_EVENT event, uintptr_t context );
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

void usartReadEventHandler(UART_EVENT event, uintptr_t context )
{
    uint32_t nBytesAvailable = 0;
    
    if (event == UART_EVENT_READ_THRESHOLD_REACHED)
    {
        /* Receiver should atleast have the thershold number of bytes in the ring buffer */
        nBytesAvailable = UART2_ReadCountGet();
        
        nBytesRead += UART2_Read((uint8_t*)&rxBuffer[nBytesRead], nBytesAvailable);                          
    }
}

void usartWriteEventHandler(UART_EVENT event, uintptr_t context )
{
    txThresholdEventReceived = true;
}


void GCODE_USART_Initialize( uint32_t RD_thresholds)
{
    uint32_t nBytes = 0;        


    /* Register a callback for write events */
    UART2_WriteCallbackRegister(usartWriteEventHandler, (uintptr_t) NULL);
    
    /* Register a callback for read events */
    UART2_ReadCallbackRegister(usartReadEventHandler, (uintptr_t) NULL);    
    
    
    // *******************************************************************************************
    // * Print the size of the read buffer on the terminal Dummy test
    // *******************************************************************************************
    nBytes = sprintf((char*)txBuffer, "RX Buffer Size = %d\r\n", (int)UART2_ReadBufferSizeGet());
    
    UART2_Write((uint8_t*)txBuffer, nBytes);  
    
    /* Print the size of the write buffer on the terminal */
    nBytes = sprintf((char*)txBuffer, "TX Buffer Size = %d\r\n", (int)UART2_WriteBufferSizeGet());
    
    UART2_Write((uint8_t*)txBuffer, nBytes);    
    
    UART2_Write((uint8_t*)GRBL_FIRMWARE_VERSION, sizeof(GRBL_FIRMWARE_VERSION));    
    UART2_Write((uint8_t*)GRBL_BUILD_DATE, sizeof(GRBL_BUILD_DATE));    
    UART2_Write((uint8_t*)GRBL_BUILD_TIME, sizeof(GRBL_BUILD_TIME));    

    /* set write threshold to indicate when the TX buffer is full */
    /* enable notifications to get notified when the TX buffer is empty */
    UART2_WriteThresholdSet(UART2_WriteBufferSizeGet());   
    
    /* Enable notifications, disabled for now */
    UART2_WriteNotificationEnable(false, false);

   /* set a threshold value to receive a callback after every 1 characters are received */
    UART2_ReadThresholdSet(RD_thresholds);

    /* Disable RX event notifications */
    UART2_ReadNotificationEnable(false, false);
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
    
    // Tokenize the line using utils
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
    
    if (event_valid) {
        // Remove processed command from queue
        cmdQueue->tail = (cmdQueue->tail + 1) % GCODE_MAX_COMMANDS;
        cmdQueue->count--;
        return true;
    }
    
    return false;
}

void GCODE_Tasks(GCODE_CommandQueue* commandQueue)
{
    GCODE_CommandQueue* cmdQueue = commandQueue;
    uint32_t nBytesAvailable = 0;
    
    switch (gcodeData.state)
    {
    case GCODE_STATE_IDLE:
        /* check if command has been recieved */
        nBytesAvailable = UART2_ReadCountGet();

        if(nBytesAvailable > 0){
            // Read all bytes from ring buffer.
           nBytesRead = UART2_Read((uint8_t*)&rxBuffer[nBytesRead], nBytesAvailable);  
           
           // We need to check the 1st char for control characters like ?,~,^X,!,etc.
            for(uint16_t i=0; i<sizeof(GRBL_CONTROL_CHARS); i++){
                if(rxBuffer[0] == GRBL_CONTROL_CHARS[i]){
                    gcodeData.state = GCODE_STATE_CONTROL_CHAR;
                    break;
                }
            }

            // If we reach here, no control chars found = G-code command
            // Process the buffer as G-code (null-terminated string)
            cmdQueue = Extract_CommandLineFrom_Buffer(rxBuffer, nBytesRead, cmdQueue);
         
           // 🎯 EVENT PROCESSING - Commands ready for processing
            // Events will be consumed by APP_Tasks via GCODE_GetNextEvent()
            // This maintains clean separation between parsing and execution

            // Send acknowledgment for next gcode line, stop if buffer is full
            // wait for free space.
            if(cmdQueue->count < GCODE_MAX_COMMANDS - 1){
                     UART2_Write((uint8_t*)"OK\r\n", 4);
            } else {
                // Buffer full, do not send OK, wait for space
            }

            // reset rxBuffer for next line
            nBytesRead = 0; 
            memset(rxBuffer, 0, sizeof(rxBuffer));
            // If line not complete, keep accumulating in rxBuffer
            break;

        }
        // if we make it here chars read were not recognized.
        gcodeData.state = GCODE_STATE_IDLE;
        break;

case GCODE_STATE_CONTROL_CHAR:
        /* Handle specific control characters per GRBL protocol */
        switch(rxBuffer[0]) {
            case '?':  // Status query - immediate response required
{
    // Get current position from stepper module
    StepperPosition* pos = STEPPER_GetPosition();
    WorkCoordinateSystem* wcs = KINEMATICS_GetWorkCoordinates();  // Fixed: No parameter needed
    
    // Convert step counts to real coordinates
    float mpos_x = (float)pos->x_steps / pos->steps_per_mm_x;
    float mpos_y = (float)pos->y_steps / pos->steps_per_mm_y;
    float mpos_z = (float)pos->z_steps / pos->steps_per_mm_z;
    
    // Calculate work coordinates using kinematics internal coordinates
    float wpos_x = mpos_x - wcs->offset.x;
    float wpos_y = mpos_y - wcs->offset.y;
    float wpos_z = mpos_z - wcs->offset.z;
    
    // Send GRBL status response
    nBytesRead = snprintf((char*)txBuffer, sizeof(txBuffer),
                         "<Idle|MPos:%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f|FS:0,0>\r\n",
                         mpos_x, mpos_y, mpos_z, wpos_x, wpos_y, wpos_z);
    UART2_Write((uint8_t*)txBuffer, nBytesRead);
    break;
}
            case '~':  // Cycle start/resume
                // Resume motion, send "OK\r\n"
                UART2_Write((uint8_t*)"OK\r\n", 4);
                break;
            case '!':  // Feed hold
                // Pause motion, send "OK\r\n"
                UART2_Write((uint8_t*)"OK\r\n", 4);
                break;
            case 0x18: // Soft reset (Ctrl+X)
                // Reset system, send GRBL startup message
                UART2_Write((uint8_t*)GRBL_FIRMWARE_VERSION, sizeof(GRBL_FIRMWARE_VERSION));
                break;
            default:
                UART2_Write((uint8_t*)rxBuffer, 1); // Echo unknown control char
                break;
        }
        nBytesRead = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        gcodeData.state = GCODE_STATE_IDLE;
        break;

    case GCODE_STATE_ERROR:
        // Send GRBL-style error message
        nBytesRead = sprintf((char*)txBuffer, "error:1\r\n"); // Error code 1 = G-code syntax error
        UART2_Write((uint8_t*)txBuffer, nBytesRead);
        
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
    CoordinatePoint coords = {0.0f, 0.0f, 0.0f, 0.0f};  // Initialize to zero
    
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
    
    // Command not recognized or not implemented
    return false;
}
