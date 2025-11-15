#include "kinematics.h"
#include "motion.h"
#include "stepper.h"
#include "settings.h"  // For CNC_Settings access
#include "common.h"
#include "utils/uart_utils.h"  // For DEBUG_PRINT_MOTION
#include "utils/utils.h"       // For AxisConfig
#include <stdlib.h>
#include <math.h>

// Timer configuration for step interval calculations
// Timer frequency now queried dynamically (TMR4 in 16-bit mode)
#define TIMER_TICKS_PER_SECOND_DYNAMIC() (TMR4_FrequencyGet())

// Single instance of work coordinates managed by kinematics (physics module)
static WorkCoordinateSystem work_coordinates;

void KINEMATICS_Initialize(void) {
    // Initialize work coordinate system to default (G54)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        SET_COORDINATE_AXIS(&work_coordinates.offset, axis, 0.0f);
    }
}

WorkCoordinateSystem* KINEMATICS_GetWorkCoordinates(void) {
    return &work_coordinates;  // Return reference to internal single instance
}

void KINEMATICS_SetWorkOffset(float x_offset, float y_offset, float z_offset) {
    // Array-based assignment for easy axis scaling
    float offsets[NUM_AXIS] = {x_offset, y_offset, z_offset, 0.0f};
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        SET_COORDINATE_AXIS(&work_coordinates.offset, axis, offsets[axis]);
    }
}

void KINEMATICS_SetWorkCoordinates(float x, float y, float z) {
    // Array-based assignment for easy axis scaling
    float coords[NUM_AXIS] = {x, y, z, 0.0f};
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        SET_COORDINATE_AXIS(&work_coordinates.offset, axis, coords[axis]);
    }
}

// Get active work coordinate system offset based on current modal state
void KINEMATICS_GetActiveWCSOffset(uint8_t activeWCS, float* x_offset, float* y_offset, float* z_offset) {
    if (!x_offset || !y_offset || !z_offset) return;
    
    // Get current active work coordinate system from settings
    if (!SETTINGS_GetWorkCoordinateSystem(activeWCS, x_offset, y_offset, z_offset)) {
        // Fallback to zeros if invalid WCS number
        *x_offset = 0.0f;
        *y_offset = 0.0f;
        *z_offset = 0.0f;
    }
    
    // Add G92 offset (temporary coordinate offset)
    float g92_x, g92_y, g92_z;
    SETTINGS_GetG92Offset(&g92_x, &g92_y, &g92_z);
    *x_offset += g92_x;
    *y_offset += g92_y;
    *z_offset += g92_z;
}

// Physics calculations using internal work coordinates
CoordinatePoint KINEMATICS_WorkToMachine(CoordinatePoint work_pos) {
    CoordinatePoint machine_pos;
    
    // Machine position = Work position + Work coordinate offset
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        if (axis == AXIS_A) {
            // A axis typically not affected by work coordinates
            SET_COORDINATE_AXIS(&machine_pos, axis, GET_COORDINATE_AXIS(&work_pos, axis));
        } else {
            SET_COORDINATE_AXIS(&machine_pos, axis,
                GET_COORDINATE_AXIS(&work_pos, axis) + GET_COORDINATE_AXIS(&work_coordinates.offset, axis));
        }
    }
    
    return machine_pos;
}

CoordinatePoint KINEMATICS_MachineToWork(CoordinatePoint machine_pos) {
    CoordinatePoint work_pos;
    
    // Work position = Machine position - Work coordinate offset
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        if (axis == AXIS_A) {
            // A axis typically not affected by work coordinates
            SET_COORDINATE_AXIS(&work_pos, axis, GET_COORDINATE_AXIS(&machine_pos, axis));
        } else {
            SET_COORDINATE_AXIS(&work_pos, axis,
                GET_COORDINATE_AXIS(&machine_pos, axis) - GET_COORDINATE_AXIS(&work_coordinates.offset, axis));
        }
    }
    
    return work_pos;
}

// Enhanced coordinate conversion functions using active WCS
CoordinatePoint KINEMATICS_WorkToMachineWithWCS(CoordinatePoint work_pos, uint8_t activeWCS) {
    CoordinatePoint machine_pos;
    float x_offset, y_offset, z_offset;
    
    // Get active WCS offset (includes G92 offset)
    KINEMATICS_GetActiveWCSOffset(activeWCS, &x_offset, &y_offset, &z_offset);
    
    // Array-based conversion with loop for easy axis scaling
    float offsets[NUM_AXIS] = {x_offset, y_offset, z_offset, 0.0f};
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        SET_COORDINATE_AXIS(&machine_pos, axis, 
            GET_COORDINATE_AXIS(&work_pos, axis) + offsets[axis]);
    }
    
    return machine_pos;
}

CoordinatePoint KINEMATICS_MachineToWorkWithWCS(CoordinatePoint machine_pos, uint8_t activeWCS) {
    CoordinatePoint work_pos;
    float x_offset, y_offset, z_offset;
    
    // Get active WCS offset (includes G92 offset)
    KINEMATICS_GetActiveWCSOffset(activeWCS, &x_offset, &y_offset, &z_offset);
    
    // Array-based conversion with loop for easy axis scaling
    float offsets[NUM_AXIS] = {x_offset, y_offset, z_offset, 0.0f};
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        SET_COORDINATE_AXIS(&work_pos, axis, 
            GET_COORDINATE_AXIS(&machine_pos, axis) - offsets[axis]);
    }
    
    return work_pos;
}

// Physics & profiling calculations for linear moves
MotionSegment* KINEMATICS_LinearMove(CoordinatePoint start, CoordinatePoint end, float feedrate, 
                                   MotionSegment* segment_buffer,
                                   float entry_velocity, float exit_velocity) {
    // ✅ Set segment type (CRITICAL: Must be first!)
    segment_buffer->type = SEGMENT_TYPE_LINEAR;
    
    // Get settings and stepper position (reuse existing modules - no duplication)
    CNC_Settings* settings = SETTINGS_GetCurrent();
    StepperPosition* stepper = STEPPER_GetPosition();
    
    // Convert to machine coordinates using internal work coordinates
    CoordinatePoint machine_start = KINEMATICS_WorkToMachine(start);
    CoordinatePoint machine_end = KINEMATICS_WorkToMachine(end);
    
    // ✅ ARRAY-BASED: Calculate distance in mm (preserve sign for direction)
    float delta_mm[NUM_AXIS];
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        delta_mm[axis] = GET_COORDINATE_AXIS(&machine_end, axis) - GET_COORDINATE_AXIS(&machine_start, axis);
    }
    
    // Convert mm to steps WITH ACCUMULATION to preserve fractional steps
    // Using static file-scope accumulators
    static float step_accumulator[NUM_AXIS] = {0.0f, 0.0f, 0.0f, 0.0f};
    
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        step_accumulator[axis] += delta_mm[axis] * stepper->steps_per_mm[axis];
        segment_buffer->delta[axis] = (int32_t)step_accumulator[axis];
        step_accumulator[axis] -= (float)segment_buffer->delta[axis];
    }
    
    DEBUG_PRINT_MOTION("[KINEMATICS] dx=%.4f dy=%.4f → steps: X=%ld Y=%ld Z=%ld A=%ld (acc: %.3f,%.3f)\r\n",
                      delta_mm[AXIS_X], delta_mm[AXIS_Y], 
                      segment_buffer->delta[AXIS_X], segment_buffer->delta[AXIS_Y],
                      segment_buffer->delta[AXIS_Z], segment_buffer->delta[AXIS_A],
                      step_accumulator[AXIS_X], step_accumulator[AXIS_Y]);
    
    // ✅ ARRAY-BASED: Determine dominant axis (highest ABSOLUTE step count) - for Bresenham and timing
    int32_t max_delta = 0;
    segment_buffer->dominant_axis = AXIS_X;
    E_AXIS limiting_axis = AXIS_X;
    
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        int32_t current_delta = abs(segment_buffer->delta[axis]);
        if (current_delta > max_delta) {
            max_delta = current_delta;
            segment_buffer->dominant_axis = axis;
            limiting_axis = axis;
        }
    }
    
    // Store dominant delta (used by Bresenham in ISR)
    segment_buffer->dominant_delta = max_delta;
    
    // Convert feedrate from mm/min to mm/sec
    float feedrate_mm_sec = feedrate / 60.0f;
    // Guard: if no feed specified (0 or negative), fall back to a safe default
    // This prevents divide-by-zero when computing nominal_rate and ensures motion proceeds after reset.
    if (feedrate_mm_sec <= 0.0f) {
        // Use a conservative default of 600 mm/min (10 mm/sec)
        feedrate_mm_sec = 600.0f / 60.0f;
    }
    
    // Get max_rate and acceleration for limiting axis using direct array access
    E_AXIS cfg_axis = limiting_axis;
    if (cfg_axis >= NUM_AXIS) {
        cfg_axis = AXIS_X;  // Fallback to X axis if invalid
    }
    
    float max_rate_mm_min = *(g_axis_settings[cfg_axis].max_rate);
    float acceleration_mm_sec2 = *(g_axis_settings[cfg_axis].acceleration);
    
    // Clamp feedrate to max rate
    float max_rate_mm_sec = max_rate_mm_min / 60.0f;
    if(feedrate_mm_sec > max_rate_mm_sec) {
        feedrate_mm_sec = max_rate_mm_sec;
    }
    
    // Timer frequency (TMR4 in 16-bit mode)
    const float TIMER_FREQ = (float)TMR4_FrequencyGet();
    
    // Array-based steps_per_mm lookup (replaces switch statement)
    float steps_per_mm_dominant = *g_axis_settings[segment_buffer->dominant_axis].steps_per_mm;
    
    // Calculate nominal step interval (cruise speed)
    // steps_per_sec = feedrate_mm_sec * steps_per_mm
    float steps_per_sec = feedrate_mm_sec * steps_per_mm_dominant;
    if (steps_per_sec < 1.0f) {
        // Ensure at least 1 step/sec to avoid INF/NaN conversions
        steps_per_sec = 1.0f;
    }
    segment_buffer->nominal_rate = (uint32_t)(TIMER_FREQ / steps_per_sec);
    
    // ✅ DEBUG: Print timing calculations
    DEBUG_PRINT_MOTION("[KINEMATICS] F=%.1f mm/min, steps/mm=%.1f, steps/sec=%.1f, nominal_rate=%lu ticks (%.2fms)\r\n",
        feedrate, steps_per_mm_dominant, steps_per_sec, 
        segment_buffer->nominal_rate, (float)segment_buffer->nominal_rate / TIMER_FREQ * 1000.0f);
    
    // Calculate acceleration profile (GRBL-style trapezoidal)
    // Distance to accelerate: d = v² / (2*a)
    float accel_distance_mm = (feedrate_mm_sec * feedrate_mm_sec) / (2.0f * acceleration_mm_sec2);
    uint32_t accel_steps = (uint32_t)(accel_distance_mm * steps_per_mm_dominant);
    
    // Check if we have room for full accel + decel
    if(accel_steps * 2 > (uint32_t)max_delta) {
        // Triangle profile - can't reach full speed
        accel_steps = max_delta / 2;
        segment_buffer->accelerate_until = accel_steps;
        segment_buffer->decelerate_after = accel_steps;
    } else {
        // Trapezoid profile - accel, cruise, decel
        segment_buffer->accelerate_until = accel_steps;
        segment_buffer->decelerate_after = max_delta - accel_steps;
    }
    
    // Junction-aware velocity planning
    // ✅ FIX: Use actual feedrate as minimum, not arbitrary slow value
    // This prevents starting segments at half speed unnecessarily
    float min_steps_per_sec = steps_per_sec;  // Use target speed as minimum
    
    // Calculate entry and exit step rates from junction velocities
    float entry_steps_per_sec = fmaxf(entry_velocity * steps_per_mm_dominant, min_steps_per_sec);
    float exit_steps_per_sec = fmaxf(exit_velocity * steps_per_mm_dominant, min_steps_per_sec);
    
    segment_buffer->initial_rate = (uint32_t)(TIMER_FREQ / entry_steps_per_sec);
    segment_buffer->final_rate = (uint32_t)(TIMER_FREQ / exit_steps_per_sec);

    // ✅ FIX: Ensure rates don't go faster than nominal (LOWER rate = FASTER speed in timer ticks)
    // nominal_rate is the FASTEST allowed speed (minimum ticks between steps)
    if (segment_buffer->initial_rate < segment_buffer->nominal_rate) {
        segment_buffer->initial_rate = segment_buffer->nominal_rate;  // Start at cruise speed
    }
    if (segment_buffer->final_rate < segment_buffer->nominal_rate) {
        segment_buffer->final_rate = segment_buffer->nominal_rate;    // End at cruise speed
    }
    
    // Rate delta per step (for integer ISR math)
    if(accel_steps > 0) {
        segment_buffer->rate_delta = (int32_t)((segment_buffer->initial_rate - segment_buffer->nominal_rate) / accel_steps);
    } else {
        segment_buffer->rate_delta = 0;
    }
    
    // Physics parameters (for debugging/reference) - use actual junction velocities
    segment_buffer->start_velocity = entry_velocity;
    segment_buffer->max_velocity = feedrate_mm_sec;
    segment_buffer->end_velocity = exit_velocity;
    segment_buffer->acceleration = acceleration_mm_sec2;
    
    // Motion state initialization
    segment_buffer->steps_remaining = max_delta;
    segment_buffer->steps_completed = 0;
    // ⚠️ TEMPORARY: Use nominal_rate (cruise speed) until velocity profiling is connected
    // TODO: Implement proper acceleration by updating step_interval in MOTION_PHASE_VELOCITY
    segment_buffer->step_interval = segment_buffer->nominal_rate;
    
    // Pulse width from settings (reuse existing calculation from stepper.c)
    segment_buffer->pulse_width = (uint32_t)(settings->step_pulse_time * 12.5f);  // µs to timer ticks
    
    // Initialize Bresenham error terms (symmetric rounding)
    for (E_AXIS axis = AXIS_Y; axis < NUM_AXIS; axis++) {
        segment_buffer->error[axis] = max_delta / 2;
    }
    
    return segment_buffer;
}

// Arc interpolation - generates a single linear segment for an arc
MotionSegment* KINEMATICS_ArcMove(CoordinatePoint start, CoordinatePoint end, CoordinatePoint center, 
                                 bool clockwise, float feedrate, MotionSegment* segment_buffer) {
    // Guard against invalid arc parameters
    if (!segment_buffer) {
        return NULL;
    }
    
    // Calculate radius from start point to center
    float dx_center = GET_COORDINATE_AXIS(&start, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X);
    float dy_center = GET_COORDINATE_AXIS(&start, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y);
    float radius = sqrtf(dx_center * dx_center + dy_center * dy_center);
    
    // Validate radius (must be > 0)
    if (radius < 0.001f) {
        DEBUG_PRINT_MOTION("[KINEMATICS_ARC] Invalid radius: %.6f\r\n", radius);
        return NULL;
    }
    
    // Validate end point radius matches start point radius (within tolerance)
    float dx_end = GET_COORDINATE_AXIS(&end, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X);
    float dy_end = GET_COORDINATE_AXIS(&end, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y);
    float end_radius = sqrtf(dx_end * dx_end + dy_end * dy_end);
    
    if (fabsf(radius - end_radius) > 0.005f) {
        DEBUG_PRINT_MOTION("[KINEMATICS_ARC] Radius mismatch: start=%.6f, end=%.6f\r\n", radius, end_radius);
        return NULL;
    }
    
    // Calculate arc length and choose appropriate feedrate
    float start_angle = atan2f(dy_center, dx_center);
    float end_angle = atan2f(dy_end, dx_end);
    
    float arc_angle;
    if (clockwise) {
        arc_angle = start_angle - end_angle;
        if (arc_angle <= 0.0f) arc_angle += 2.0f * M_PI;
    } else {
        arc_angle = end_angle - start_angle;
        if (arc_angle <= 0.0f) arc_angle += 2.0f * M_PI;
    }
    
#ifdef DEBUG_MOTION
    float arc_length = radius * arc_angle;
    DEBUG_PRINT_MOTION("[KINEMATICS_ARC] Arc segment: radius=%.3f, angle=%.3f rad, length=%.3f mm\r\n",
                      radius, arc_angle, arc_length);
#endif
    
    // Use linear move to generate the segment (arcs are interpolated as short linear segments)
    // This leverages all the existing motion planning including junction deviation
    float entry_velocity = 5.0f;  // mm/sec - conservative entry for arc segments
    float exit_velocity = 5.0f;   // mm/sec - conservative exit for arc segments
    
    return KINEMATICS_LinearMove(start, end, feedrate, segment_buffer, entry_velocity, exit_velocity);
}

// High-level arc planning function - validates and calculates arc parameters
bool KINEMATICS_PlanArc(CoordinatePoint start, CoordinatePoint end, CoordinatePoint center,
                       bool clockwise, float feedrate, float* out_radius, float* out_total_angle,
                       uint32_t* out_num_segments) {
    // Validate input parameters
    if (!out_radius || !out_total_angle || !out_num_segments) {
        return false;
    }
    
    // Calculate radius from start and end points
    float dx_start = GET_COORDINATE_AXIS(&start, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X);
    float dy_start = GET_COORDINATE_AXIS(&start, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y);
    float r_start = sqrtf(dx_start * dx_start + dy_start * dy_start);
    
    float dx_end = GET_COORDINATE_AXIS(&end, AXIS_X) - GET_COORDINATE_AXIS(&center, AXIS_X);
    float dy_end = GET_COORDINATE_AXIS(&end, AXIS_Y) - GET_COORDINATE_AXIS(&center, AXIS_Y);
    float r_end = sqrtf(dx_end * dx_end + dy_end * dy_end);
    
    // Radius validation (GRBL standard tolerance)
    if (r_start < 0.001f || fabsf(r_start - r_end) > 0.005f) {
        DEBUG_PRINT_MOTION("[KINEMATICS_PLAN_ARC] Radius error: start=%.6f, end=%.6f\r\n", r_start, r_end);
        return false;
    }
    
    // Calculate angles and total arc angle
    float start_angle = atan2f(dy_start, dx_start);
    float end_angle = atan2f(dy_end, dx_end);
    
    float total_angle;
    if (clockwise) {
        total_angle = start_angle - end_angle;
        if (total_angle <= 0.0f) total_angle += 2.0f * M_PI;
    } else {
        total_angle = end_angle - start_angle;
        if (total_angle <= 0.0f) total_angle += 2.0f * M_PI;
    }
    
    // Calculate arc length and number of segments
    float arc_length = r_start * total_angle;
    CNC_Settings* settings = SETTINGS_GetCurrent();
    uint32_t num_segments = (uint32_t)ceilf(arc_length / settings->mm_per_arc_segment);
    if (num_segments < 1) num_segments = 1;
    
    // Limit maximum segments to prevent memory issues
    if (num_segments > 1000) {
        DEBUG_PRINT_MOTION("[KINEMATICS_PLAN_ARC] Too many segments (%lu), limiting to 1000\r\n", num_segments);
        num_segments = 1000;
    }
    
    // Return calculated values
    *out_radius = r_start;
    *out_total_angle = total_angle;
    *out_num_segments = num_segments;
    
    DEBUG_PRINT_MOTION("[KINEMATICS_PLAN_ARC] Arc planned: R=%.3f, angle=%.3f rad (%.1f°), segments=%lu\r\n",
                      r_start, total_angle, total_angle * 180.0f / M_PI, num_segments);
    
    return true;
}

// Reset step accumulators - call when starting new arc or after position reset (G92)
void KINEMATICS_ResetAccumulators(void) {
    // Accumulators are now static inside KINEMATICS_LinearMove
    // This function kept for API compatibility but does nothing
    // TODO: Refactor to expose accumulator reset properly if needed
}
// Get current position from stepper counts
CoordinatePoint KINEMATICS_GetCurrentPosition(void) {
    CoordinatePoint current;
    
    // Use direct array access for clean access
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        float position = (float)AXIS_GetSteps(axis) / (*g_axis_settings[axis].steps_per_mm);
        
        // Array-based coordinate setting (replaces switch statement)
        SET_COORDINATE_AXIS(&current, axis, position);
    }
    
    return current;
}

// Set machine position for a single axis (used during homing)
void KINEMATICS_SetAxisMachinePosition(E_AXIS axis, float position) {
    if (axis >= NUM_AXIS) return;  // Invalid axis
    
    // Convert position to steps and update using abstracted inline helper
    int32_t steps = (int32_t)(position * (*g_axis_settings[axis].steps_per_mm));
    AXIS_SetSteps(axis, steps);
}

// Junction deviation calculation for smooth cornering between segments
float KINEMATICS_CalculateJunctionSpeed(CoordinatePoint prev_dir, CoordinatePoint curr_dir,
                                       float junction_deviation, float acceleration) {
    // Guard against invalid inputs
    if (junction_deviation <= 0.0f || acceleration <= 0.0f) {
        return 0.0f;  // Dead stop for invalid parameters
    }
    
    // ✅ ARRAY-BASED: Calculate dot product of normalized direction vectors
    float dot_product = 0.0f;
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        dot_product += GET_COORDINATE_AXIS(&prev_dir, axis) * GET_COORDINATE_AXIS(&curr_dir, axis);
    }
    
    // Clamp to [-1, 1] to prevent acos domain errors
    if (dot_product > 1.0f) dot_product = 1.0f;
    if (dot_product < -1.0f) dot_product = -1.0f;
    
    // Straight line (no significant direction change)
    if (dot_product > 0.999f) {
        return 1000.0f;  // High speed - virtually no corner
    }
    
    // Sharp reverse direction (>90 degrees)
    if (dot_product < -0.999f) {
        return 0.0f;  // Dead stop for reverse direction
    }
    
    // GRBL junction deviation formula
    // Safe junction speed based on acceleration capability and deviation tolerance
    float cosine_angle = dot_product;
    float junction_speed = sqrtf(junction_deviation * acceleration * 
                                (2.0f / (1.0f - cosine_angle)));
    
    DEBUG_PRINT_MOTION("[JUNCTION] dot=%.3f, cosine=%.3f, dev=%.4f, accel=%.1f → speed=%.1f mm/sec\r\n",
                      dot_product, cosine_angle, junction_deviation, acceleration, junction_speed);
    
    return junction_speed;
}

// Backward compatibility wrapper for single segments (no junction planning)
MotionSegment* KINEMATICS_LinearMoveSimple(CoordinatePoint start, CoordinatePoint end, float feedrate, 
                                          MotionSegment* segment_buffer) {
    // Use minimum entry/exit velocities (traditional behavior - accelerate from stop, decelerate to stop)
    float min_velocity = 8.33f;  // 500 mm/min minimum (matches min_steps_per_sec / steps_per_mm)
    return KINEMATICS_LinearMove(start, end, feedrate, segment_buffer, min_velocity, min_velocity);
}
