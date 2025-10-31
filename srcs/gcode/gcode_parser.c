#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <string.h>
#include <stdio.h>
#include "gcode_parser.h"                // SYS function prototypes

static uint8_t txBuffer[50];
static uint8_t rxBuffer[50];
static volatile uint32_t nBytesRead = 0;
static volatile bool txThresholdEventReceived = false;
static volatile bool rxThresholdEventReceived = false;

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


void GCODE_USART_Initialize(void)
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
    
    UART2_Write((uint8_t*)"Adding 10 characters to the TX buffer - ", sizeof("Adding 10 characters to the TX buffer - "));    
    

    /* Disable notifications to get notified when the TX buffer is empty */
    UART2_WriteThresholdSet(UART2_WriteBufferSizeGet());   
    
    /* Enable notifications */
    UART2_WriteNotificationEnable(false, false);

   /* set a threshold value to receive a callback after every 1 characters are received */
    UART2_ReadThresholdSet(1);

    /* Disable RX event notifications */
    UART2_ReadNotificationEnable(false, false);
}
