#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H


#include "definitions.h"
#include "plib_uart2.h"
#include "plib_uart_common.h"

/* string literals for GRBL firmware commands */
#ifndef WindowsOS
#define NEWLINE "\r\n"
#else
#define NEWLINE "\n"
#endif



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



/* GCODE State Machine */
typedef enum {
    GCODE_STATE_IDLE,
    GCODE_STATE_CONTROL_CHAR,
    GCODE_STATE_ERROR
} GCODE_State;

typedef struct{
    GCODE_State state;
} GCODE_Data;

/* GCODE USART function prototypes */




void GCODE_USART_Initialize(uint32_t RD_thresholds);
void GCODE_Tasks(GCODE_CommandQueue* commandQueue);
#endif // GCODE_PARSER_H