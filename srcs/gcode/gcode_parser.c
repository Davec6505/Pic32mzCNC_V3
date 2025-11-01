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

void usartReadEventHandler(UART_EVENT event, uintptr_t context );
void usartWriteEventHandler(UART_EVENT event, uintptr_t context );

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


GCODE_CommandQueue* Extract_CommandLineFrom_Buffer(uint8_t* buffer, uint32_t length, GCODE_CommandQueue* commandQueue)
{
    GCODE_CommandQueue* cmdQueue = commandQueue;
    uint32_t cmdStart = 0;

    for(uint32_t j = 0; j < length; j++){
        // Skip whitespace at start of potential command
        if(buffer[j] == ' ' || buffer[j] == '\t'){
            cmdStart = j + 1;
            continue;
        }
        
        // Check for G or M command start
        if(buffer[j] == 'G' || buffer[j] == 'M'){
            cmdStart = j;  // Mark start of this command
            
            // Find end of current command (next G/M or line terminator)
            uint32_t cmdEnd = j + 1;
            while(cmdEnd < length && 
                    buffer[cmdEnd] != 'G' && 
                    buffer[cmdEnd] != 'M' && 
                    buffer[cmdEnd] != '\n' && 
                    buffer[cmdEnd] != '\r' &&
                    buffer[cmdEnd] != ';' &&
                    buffer[cmdEnd] != '#' &&
                    buffer[cmdEnd] != '\0') { // also stop at comment char
                cmdEnd++;   
            }
            
            // Copy command to queue if there's space
            if(((cmdQueue->head + 1) % GCODE_MAX_COMMANDS) != cmdQueue->tail){
                uint32_t cmdLen = cmdEnd - cmdStart;
                if(cmdLen < GCODE_BUFFER_SIZE){
                    memcpy(cmdQueue->commands[cmdQueue->head].command, 
                            &buffer[cmdStart], cmdLen);
                    cmdQueue->commands[cmdQueue->head].command[cmdLen] = '\0';
                    cmdQueue->head = (cmdQueue->head + 1) % GCODE_MAX_COMMANDS;
                }
            }
            
            // Move to end of this command for next iteration
            j = cmdEnd - 1;  // -1 because for loop will increment
            continue;
        }

        if((buffer[j] == '\n') || (buffer[j] == '\r' || buffer[j] == '\0')){
            break;
        }
    }
    return cmdQueue;
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
         
            // Send acknowledgment for G-code line
            UART2_Write((uint8_t*)"OK\r\n", 4);

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
