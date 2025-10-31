#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H


#include "definitions.h"
#include "plib_uart2.h"
#include "plib_uart_common.h"



void usartReadEventHandler(UART_EVENT event, uintptr_t context );
void usartWriteEventHandler(UART_EVENT event, uintptr_t context );
void GCODE_USART_Initialize(void);

#endif // GCODE_PARSER_H