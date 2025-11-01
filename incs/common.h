// common definef must go here


#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* grbl firmware commands */
#define FIRMWARE_VERSION "1.1"
// Macro to convert a macro value to a string literal
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Concatenate GRBL prefix with FIRMWARE_VERSION from gcode_parser.h
#define GRBL_FIRMWARE_VERSION "GRBL v" FIRMWARE_VERSION NEWLINE
#define GRBL_BUILD_DATE "Build Date: " __DATE__ NEWLINE
#define GRBL_BUILD_TIME "Build Time: " __TIME__ NEWLINE


// Define number of axes
#define NUM_OF_AXIS 4  // X, Y, Z, A

// Motion planning configuration
#define MAX_MOTION_SEGMENTS 16  // Look-ahead planning buffer size


typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_A,
    AXIS_COUNT = NUM_OF_AXIS
} E_AXIS;

// Simple validation - no function overhead needed
#define IS_VALID_AXIS(axis) ((axis) < AXIS_COUNT)

#endif /* COMMON_H */