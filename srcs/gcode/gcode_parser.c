#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <string.h>
#include <stdio.h>
#include "gcode_parser.h"                // SYS function prototypes



/* USART Buffers */
static uint8_t txBuffer[50];
static uint8_t rxBuffer[50];
static volatile uint32_t nBytesRead = 0;
static volatile bool txThresholdEventReceived = false;
static volatile bool rxThresholdEventReceived = false;

/* local datatypes */
GCODE_Data gcodeData = {
    .state = GCODE_STATE_IDLE,
    .commandQueue = {
        .head = 0,
        .tail = 0
    }
};

void usartReadEventHandler(UART_EVENT event, uintptr_t context );
void usartWriteEventHandler(UART_EVENT event, uintptr_t context );

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





void GCODE_Tasks(void)
{
    static volatile uint32_t nBytesRead = 0;
    uint32_t nBytesAvailable = 0;

    switch (gcodeData.state)
    {
    case GCODE_STATE_IDLE:
        /* code */
        nBytesAvailable = UART2_ReadCountGet();
        if(nBytesAvailable > 0){
            nBytesRead += UART2_Read((uint8_t*)&rxBuffer[nBytesRead], nBytesAvailable);  
            gcodeData.state = GCODE_STATE_PROCESSING;
        }
        break;
    case GCODE_STATE_PROCESSING:
        /* process GCODE commands from rxBuffer and split into array of strings for processing */
        UART2_Write((uint8_t*)rxBuffer, nBytesRead);  
        nBytesRead = 0;  // reset for next read
        gcodeData.state = GCODE_STATE_IDLE;
        break;
    case GCODE_STATE_ERROR:
        /* code */
        gcodeData.state = GCODE_STATE_IDLE; 
        break;
    default:
        break;
    }
}
