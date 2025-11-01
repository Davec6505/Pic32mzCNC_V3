#include "motion.h"
#include "kinematics.h"
#include "stepper.h"
#include "common.h"
#include <stdlib.h>
#include <math.h>

// Motion state management
typedef enum {
    MOTION_STATE_IDLE,
    MOTION_STATE_BRESENHAM_ACTIVE,    // Bresenham interpolation state
    MOTION_STATE_SEGMENT_TRANSITION,  // Moving to next segment
    MOTION_STATE_COMPLETE
} MotionState;



static MotionState motion_state = MOTION_STATE_IDLE;
static MotionSegment* current_segment = NULL;

void MOTION_Initialize(void) {
    motion_state = MOTION_STATE_IDLE;
    current_segment = NULL;
    
    // Initialize stepper system
    STEPPER_Initialize();
}

void MOTION_Tasks(MotionSegment motionQueue[], uint32_t* head, uint32_t* tail, uint32_t* count) {
    
    switch(motion_state) {
        case MOTION_STATE_IDLE:
            if (*count > 0) {
                // Start executing first segment
                current_segment = &motionQueue[*tail];
               // setup_dominant_axis();  // Begin continuous tracking ahead of TMR2
                motion_state = MOTION_STATE_BRESENHAM_ACTIVE;
            }
            break;
            
        case MOTION_STATE_BRESENHAM_ACTIVE:
            // NON-BLOCKING Bresenham execution
            // execute_bresenham_step();  // Updates error terms, schedules subordinate axes
            
            if (current_segment->steps_remaining == 0) {
                motion_state = MOTION_STATE_SEGMENT_TRANSITION;
            }
            break;
            
        case MOTION_STATE_SEGMENT_TRANSITION:
            // Move to next segment without stopping motion
            // advance_to_next_segment();
            
            if (*count > 0) {
                motion_state = MOTION_STATE_BRESENHAM_ACTIVE;  // Continue with next segment
            } else {
               //disable_all_axes();  // OCxR = OCxRS to stop pulse generation
                motion_state = MOTION_STATE_IDLE;
            }
            break;

        case MOTION_STATE_COMPLETE:
           break;
           
    }
}