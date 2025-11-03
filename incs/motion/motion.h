#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"
#include "data_structures.h"  // ✅ Use unified data structures (parent directory)

// ✅ MotionSegment now defined in data_structures.h

void MOTION_Initialize(void);
void MOTION_Tasks(MotionSegment motionQueue[], uint32_t* head, uint32_t* tail, uint32_t* count);



#endif /* MOTION_H */