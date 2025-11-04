#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NUM_OF_AXIS 4  // X, Y, Z, A

// E_AXIS enum is defined in data_structures.h to avoid circular dependencies
// Include data_structures.h if you need the enum

// Simple validation - no function overhead needed
#define IS_VALID_AXIS(axis) ((axis) < NUM_OF_AXIS)

// GRBL Protocol Constants
#define GRBL_FIRMWARE_VERSION "Grbl v1.1 [PIC32MZ CNC v3.0]\r\n"
#define GRBL_BUILD_DATE "[Build Date: " __DATE__ "]\r\n"
#define GRBL_BUILD_TIME "[Build Time: " __TIME__ "]\r\n"

#endif /* COMMON_H */