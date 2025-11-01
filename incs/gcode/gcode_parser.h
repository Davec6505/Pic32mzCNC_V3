#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H


#include "definitions.h"
#include "plib_uart2.h"
#include "plib_uart_common.h"

/* definitions for GCODE parsing */
#define GCODE_BUFFER_SIZE 50
#define GCODE_MAX_COMMANDS 16

/* GCODE command structure */
typedef struct {
    char command[GCODE_BUFFER_SIZE];
    uint32_t line_number;
} GCODE_Command;

/* GCODE command queue */
typedef struct {
    GCODE_Command commands[GCODE_MAX_COMMANDS];
    uint32_t head;
    uint32_t tail;
} GCODE_CommandQueue;

/* GCODE USART function prototypes */

void usartReadEventHandler(UART_EVENT event, uintptr_t context );
void usartWriteEventHandler(UART_EVENT event, uintptr_t context );
void GCODE_USART_Initialize(void);

#endif // GCODE_PARSER_H