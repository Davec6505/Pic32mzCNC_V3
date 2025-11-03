// common definef must go here


#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "data_structures.h"  // ✅ For E_AXIS and NUM_AXIS

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

// ✅ E_AXIS now defined in data_structures.h
// ✅ MAX_MOTION_SEGMENTS now defined in data_structures.h

// Simple validation - no function overhead needed  
#define IS_VALID_AXIS(axis) ((axis) < NUM_AXIS)

#endif /* COMMON_H */