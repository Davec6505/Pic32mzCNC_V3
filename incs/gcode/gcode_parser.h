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
    uint32_t count;
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

/* G-code Event System - Clean interface without APP_DATA exposure */
typedef enum {
    GCODE_EVENT_NONE,
    GCODE_EVENT_LINEAR_MOVE,
    GCODE_EVENT_ARC_MOVE,
    GCODE_EVENT_DWELL,
    GCODE_EVENT_SPINDLE_ON,
    GCODE_EVENT_SPINDLE_OFF,
    GCODE_EVENT_COOLANT_ON,
    GCODE_EVENT_COOLANT_OFF,
    GCODE_EVENT_SET_ABSOLUTE,
    GCODE_EVENT_SET_RELATIVE
} GCODE_EventType;

typedef struct {
    GCODE_EventType type;
    union {
        struct {
            float x, y, z, a;    // Target coordinates
            float feedrate;      // Feed rate in units/min
        } linearMove;
        
        struct {
            float centerX, centerY;  // Arc center relative to start
            float x, y, z, a;        // Target coordinates  
            bool clockwise;          // Direction
            float feedrate;          // Feed rate
        } arcMove;
        
        struct {
            float seconds;           // Dwell time
        } dwell;
        
        struct {
            uint32_t rpm;           // Spindle speed
        } spindle;
    } data;
} GCODE_Event;

/* GCODE USART function prototypes */
void GCODE_USART_Initialize(uint32_t RD_thresholds);
void GCODE_Tasks(GCODE_CommandQueue* commandQueue);
bool GCODE_GetNextEvent(GCODE_CommandQueue* cmdQueue, GCODE_Event* event);

#endif // GCODE_PARSER_H