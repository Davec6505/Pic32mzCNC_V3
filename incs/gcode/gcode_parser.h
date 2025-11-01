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

/* grbl firmware commands */
#define FIRMWARE_VERSION "1.1"
// Macro to convert a macro value to a string literal
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Concatenate GRBL prefix with FIRMWARE_VERSION from gcode_parser.h
#define GRBL_FIRMWARE_VERSION "GRBL v" FIRMWARE_VERSION NEWLINE
#define GRBL_BUILD_DATE "Build Date: " __DATE__ NEWLINE
#define GRBL_BUILD_TIME "Build Time: " __TIME__ NEWLINE

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
    GCODE_STATE_PROCESSING,
    GCODE_STATE_ERROR
} GCODE_State;

typedef struct{
    GCODE_State state;
    GCODE_CommandQueue commandQueue;
} GCODE_Data;

/* GCODE USART function prototypes */




void GCODE_USART_Initialize(uint32_t RD_thresholds);
void GCODE_Tasks(void);
#endif // GCODE_PARSER_H