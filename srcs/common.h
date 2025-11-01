#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NUM_OF_AXIS 4  // X, Y, Z, A

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