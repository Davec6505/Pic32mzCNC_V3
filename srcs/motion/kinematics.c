#include "kinematics.h"
#include "motion.h"
#include "stepper.h"
#include "settings.h"  // For CNC_Settings access
#include "common.h"
#include "utils/uart_utils.h"  // For DEBUG_PRINT_MOTION
#include "utils/utils.h"       // For AxisConfig
#include <stdlib.h>
#include <math.h>

// Single instance of work coordinates managed by kinematics (physics module)
static WorkCoordinateSystem work_coordinates;

// TMR4 frequency cached at KINEMATICS_Initialize() — value is fixed at runtime.
// Avoids a JAL (function call) on every KINEMATICS_LinearMove() invocation.
static float g_timer_freq = 0.0f;

// ============================================================================
// GRBL-exact planner state  (mirrors GRBL's planner_t struct fields)
// Updated inside KINEMATICS_LinearMove, consumed by MOTION_PlannerRecalculate.
// ============================================================================
static float pl_previous_unit_vec[NUM_AXIS]; // Unit vector of previous planned segment
static float pl_previous_nominal_speed;      // Nominal speed of previous segment (mm/s)

// Normalise vec[n] to unit length in-place. Returns original magnitude (mm).
// Equivalent to GRBL's convert_delta_vector_to_unit_vector().
static float convert_to_unit_vector(float *vec, uint8_t n) {
    double mag_sq = 0.0;
    uint8_t i;
    for (i = 0; i < n; i++) { mag_sq += (double)vec[i] * (double)vec[i]; }
    if (mag_sq < 1e-18) { return 0.0f; }
    float mag = (float)sqrt(mag_sq);
    float inv = 1.0f / mag;
    for (i = 0; i < n; i++) { vec[i] *= inv; }
    return mag;
}

// Returns the maximum combined acceleration such that no single axis exceeds its
// individual limit when moving along unit_vec.
// Equivalent to GRBL's limit_value_by_axis_maximum(settings.acceleration, unit_vec).
static float limit_acceleration_by_axis(const float *unit_vec) {
    float limit = 1.0E18f;
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        float uv = fabsf(unit_vec[axis]);
        if (uv > 0.0f) {
            float a = *(g_axis_settings[axis].acceleration) / uv;
            if (a < limit) limit = a;
        }
    }
    return (limit >= 1.0E18f) ? 1.0f : limit;
}

// Returns the maximum combined feed rate such that no single axis exceeds its limit.
// Equivalent to GRBL's limit_value_by_axis_maximum(settings.max_rate, unit_vec).
static float limit_rate_by_axis(const float *unit_vec) {
    float limit = 1.0E18f;
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        float uv = fabsf(unit_vec[axis]);
        if (uv > 0.0f) {
            float r = (*(g_axis_settings[axis].max_rate) / 60.0f) / uv; // mm/s
            if (r < limit) limit = r;
        }
    }
    return (limit >= 1.0E18f) ? 1.0f : limit;
}

void KINEMATICS_ResetPlannerState(void) {
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        pl_previous_unit_vec[axis] = 0.0f;
    }
    pl_previous_nominal_speed = 0.0f;
}

void KINEMATICS_Initialize(void) {
    // Cache timer frequency once — TMR4 prescaler is fixed by MCC configuration
    g_timer_freq = (float)TMR4_FrequencyGet();

    // Initialize work coordinate system to default (G54)
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        SET_COORDINATE_AXIS(&work_coordinates.offset, axis, 0.0f);
    }

    // Reset GRBL planner state
    KINEMATICS_ResetPlannerState();
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
    
    DEBUG_PRINT_MOTION("[KIN] Z: start=%.3f end=%.3f delta=%.3f steps/mm=%.1f\r\n",
                      GET_COORDINATE_AXIS(&machine_start, AXIS_Z),
                      GET_COORDINATE_AXIS(&machine_end, AXIS_Z),
                      delta_mm[AXIS_Z],
                      stepper->steps_per_mm[AXIS_Z]);
    
    // Convert mm to steps WITH ACCUMULATION to preserve fractional steps,
    // and determine dominant axis (highest ABSOLUTE step count) in the same pass.
    static float step_accumulator[NUM_AXIS] = {0.0f, 0.0f, 0.0f, 0.0f};

    int32_t max_delta = 0;
    segment_buffer->dominant_axis = AXIS_X;
    E_AXIS limiting_axis = AXIS_X;

    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        step_accumulator[axis] += delta_mm[axis] * stepper->steps_per_mm[axis];
        segment_buffer->delta[axis] = (int32_t)step_accumulator[axis];
        step_accumulator[axis] -= (float)segment_buffer->delta[axis];

        int32_t current_delta = abs(segment_buffer->delta[axis]);
        if (current_delta > max_delta) {
            max_delta = current_delta;
            segment_buffer->dominant_axis = axis;
            limiting_axis = axis;
        }
    }

    DEBUG_PRINT_MOTION("[KIN] Z: steps=%ld\r\n", segment_buffer->delta[AXIS_Z]);
    
    // Store dominant delta (used by Bresenham in ISR)
    segment_buffer->dominant_delta = max_delta;

    // -------------------------------------------------------------------------
    // ZERO-STEP GUARD — no movement, skip all physics calculations.
    // The caller (motion.c) checks steps_remaining == 0 and handles the
    // position update + ok response; returning here avoids wasted computation
    // and spurious debug output for commands like bare "G0" or "G1 F500".
    // -------------------------------------------------------------------------
    if (max_delta == 0) {
        DEBUG_PRINT_MOTION("[KIN] Zero-step move — skipping physics\r\n");
        segment_buffer->steps_remaining = 0;
        return segment_buffer;
    }

    // =========================================================================
    // GRBL-exact: unit vector, axis-limited acceleration and rate
    // Mirrors plan_buffer_line() in GRBL's planner.c.
    // =========================================================================
    // Build unit vector from delta_mm (same signed values computed above).
    // convert_to_unit_vector normalises in-place and returns distance in mm.
    float local_unit_vec[NUM_AXIS];
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        local_unit_vec[axis] = delta_mm[axis];
    }
    float seg_millimeters = convert_to_unit_vector(local_unit_vec, NUM_AXIS);

    // Axis-limited acceleration and rapid rate (GRBL exact).
    // Scales the per-axis limits to the combined move direction so no single
    // axis ever exceeds its maximum, regardless of move angle.
    float acceleration_mm_sec2 = limit_acceleration_by_axis(local_unit_vec);
    float max_rate_mm_sec      = limit_rate_by_axis(local_unit_vec);

    // Feedrate: mm/min → mm/s, guard zero, clamp to axis-limited max
    float feedrate_mm_sec = feedrate / 60.0f;
    if (feedrate_mm_sec <= 0.0f) {
        feedrate_mm_sec = 600.0f / 60.0f;  // Fallback: 600 mm/min
    }
    if (feedrate_mm_sec > max_rate_mm_sec) {
        feedrate_mm_sec = max_rate_mm_sec;
    }

    // Store nominal speed in mm/s (segment generator will convert to PR4)
    (void)limiting_axis; // no longer used — limit_acceleration_by_axis handles this
    segment_buffer->nominal_speed = feedrate_mm_sec;
    
    DEBUG_PRINT_MOTION("[KIN] F=%.0f mm/min, nominal=%.1f mm/s, a=%.1f mm/s2\r\n",
        feedrate, segment_buffer->nominal_speed, acceleration_mm_sec2);
    
    // =========================================================================
    // JUNCTION-AWARE TRAPEZOIDAL VELOCITY PROFILE
    // =========================================================================
    // Store speeds in mm/s - segment generator will convert to PR4 values
    // Accel ramp: entry_v  →  cruise   uses (v_cruise² - v_entry²) / (2a)
    // Decel ramp: cruise   →  exit_v   uses (v_cruise² - v_exit²)  / (2a)
    segment_buffer->initial_speed = entry_velocity;
    segment_buffer->final_speed = exit_velocity;
    
    // Calculate trapezoid boundaries in mm (not steps)
    float accel_dist_mm = (feedrate_mm_sec * feedrate_mm_sec - entry_velocity * entry_velocity)
                          / (2.0f * acceleration_mm_sec2);
    float decel_dist_mm = (feedrate_mm_sec * feedrate_mm_sec - exit_velocity  * exit_velocity)
                          / (2.0f * acceleration_mm_sec2);
    if (accel_dist_mm < 0.0f) accel_dist_mm = 0.0f;
    if (decel_dist_mm < 0.0f) decel_dist_mm = 0.0f;

    if (accel_dist_mm + decel_dist_mm > seg_millimeters) {
        // Triangle: ramps exceed total distance — find peak speed where they just fit.
        float v_peak_sq = acceleration_mm_sec2 * seg_millimeters
                          + 0.5f * (entry_velocity * entry_velocity
                                    + exit_velocity  * exit_velocity);
        if (v_peak_sq < 0.0f) v_peak_sq = 0.0f;
        float v_peak = sqrtf(v_peak_sq);
        if (v_peak > feedrate_mm_sec) v_peak = feedrate_mm_sec;

        accel_dist_mm = (v_peak * v_peak - entry_velocity * entry_velocity)
                        / (2.0f * acceleration_mm_sec2);
        decel_dist_mm = (v_peak * v_peak - exit_velocity  * exit_velocity)
                        / (2.0f * acceleration_mm_sec2);
        if (accel_dist_mm < 0.0f) accel_dist_mm = 0.0f;
        if (decel_dist_mm < 0.0f) decel_dist_mm = 0.0f;

        // Triangle: decel starts immediately where accel ends
        segment_buffer->accelerate_until = accel_dist_mm;
        segment_buffer->decelerate_after = accel_dist_mm;
    } else {
        // Trapezoid: independent accel and decel with cruise in between
        segment_buffer->accelerate_until = accel_dist_mm;
        segment_buffer->decelerate_after = seg_millimeters - decel_dist_mm;
    }
    
    // =========================================================================
    // GRBL SEGMENT BUFFER ARCHITECTURE - NO TAYLOR CODE IN KINEMATICS
    // =========================================================================
    // Kinematics only stores speeds in mm/s and trapezoid boundaries.
    // The segment generator (segment_buffer.c) will convert speeds to PR4 values
    // and break the trapezoid into constant-velocity segments.
    // No ISR profiling code here - pure planner calculations only.
    
    segment_buffer->speed_locked = false;

    // =========================================================================
    // GRBL-exact planner fields
    // Mirrors the planner state update at the end of plan_buffer_line().
    // =========================================================================
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        segment_buffer->unit_vec[axis] = local_unit_vec[axis];
    }
    segment_buffer->millimeters = seg_millimeters;

    // Max junction speed — GRBL centripetal approximation (no trig required).
    // A circle of radius r = junction_deviation is inscribed at the corner;
    // the max speed through the corner is constrained by centripetal accel.
#define MINIMUM_JUNCTION_SPEED_SQR  0.0f
#define SOME_LARGE_VALUE_SQR        1.0E18f
    float max_junction_speed_sqr;
    if (pl_previous_nominal_speed <= 0.0f) {
        // First segment — no previous direction available, start from rest.
        max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED_SQR;
    } else {
        float junction_cos_theta = 0.0f;
        float junction_unit_vec[NUM_AXIS];
        for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
            junction_cos_theta -= pl_previous_unit_vec[axis] * local_unit_vec[axis];
            junction_unit_vec[axis] = local_unit_vec[axis] - pl_previous_unit_vec[axis];
        }
        if (junction_cos_theta > 0.999999f) {
            // Near 0° — sharp corner.
            max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED_SQR;
        } else if (junction_cos_theta < -0.999999f) {
            // 180° — straight-line continuation, no corner.
            max_junction_speed_sqr = SOME_LARGE_VALUE_SQR;
        } else {
            convert_to_unit_vector(junction_unit_vec, NUM_AXIS);
            float junction_accel = limit_acceleration_by_axis(junction_unit_vec);
            float jd = settings->junction_deviation;
            float sin_theta_d2 = sqrtf(0.5f * (1.0f - junction_cos_theta));
            max_junction_speed_sqr = fmaxf(MINIMUM_JUNCTION_SPEED_SQR,
                (junction_accel * jd * sin_theta_d2) / (1.0f - sin_theta_d2));
        }
    }
    segment_buffer->max_junction_speed_sqr = max_junction_speed_sqr;

    // max_entry_speed_sqr = min(junction limit, min of neighboring nominal speeds)
    float nominal_speed_sqr      = feedrate_mm_sec * feedrate_mm_sec;
    float prev_nominal_speed_sqr = pl_previous_nominal_speed * pl_previous_nominal_speed;
    float max_entry_speed_sqr = (nominal_speed_sqr < prev_nominal_speed_sqr)
                                 ? nominal_speed_sqr : prev_nominal_speed_sqr;
    if (max_entry_speed_sqr > max_junction_speed_sqr) {
        max_entry_speed_sqr = max_junction_speed_sqr;
    }
    segment_buffer->max_entry_speed_sqr = max_entry_speed_sqr;

    // entry_speed_sqr: initial conservative estimate (max achievable from full stop).
    // MOTION_PlannerRecalculate() will update this via the reverse+forward pass.
    float stop_entry_sqr = 2.0f * acceleration_mm_sec2 * seg_millimeters;
    segment_buffer->entry_speed_sqr = (max_entry_speed_sqr < stop_entry_sqr)
                                       ? max_entry_speed_sqr : stop_entry_sqr;

    // Update planner state for the NEXT segment's junction calculation.
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        pl_previous_unit_vec[axis] = local_unit_vec[axis];
    }
    pl_previous_nominal_speed = feedrate_mm_sec;

#undef MINIMUM_JUNCTION_SPEED_SQR
#undef SOME_LARGE_VALUE_SQR

    // Physics parameters
    segment_buffer->acceleration = acceleration_mm_sec2;
    
    // Motion state initialization
    segment_buffer->steps_remaining = max_delta;
    
    // Pulse width from settings (reuse existing calculation from stepper.c)
    segment_buffer->pulse_width = (uint32_t)(settings->step_pulse_time * 12.5f);  // µs to timer ticks
    
    return segment_buffer;
}

// ============================================================================
// KINEMATICS_RecalculateTrapezoid
//
// Re-computes the trapezoid profile (entry/nominal/exit speeds in mm/s and
// accel/decel boundaries in mm) for a segment given its planned speeds.
//
// Called by MOTION_PlannerRecalculate() after the reverse+forward pass has
// determined the correct entry and exit speeds for every buffered segment.
// Exact port of GRBL's calculate_trapezoid_for_block().
//
// NOTE: This only stores speeds in mm/s. The segment generator (segment_buffer.c)
// will convert these to PR4 values when breaking the block into segments.
// ============================================================================
void KINEMATICS_RecalculateTrapezoid(MotionSegment *seg,
                                     float entry_mms,
                                     float exit_mms)
{
    if (!seg || seg->steps_remaining == 0) { return; }

    float accel          = seg->acceleration;          // mm/s²
    float nominal_mms    = seg->nominal_speed;         // feedrate stored at plan time
    float distance_mm    = seg->millimeters;

    // Clamp to nominal so we never exceed programmed feedrate
    if (entry_mms > nominal_mms) { entry_mms = nominal_mms; }
    if (exit_mms  > nominal_mms) { exit_mms  = nominal_mms; }

    // ---- Trapezoidal intersection point in mm (GRBL calc_trapezoid) ----
    // Distance needed to accelerate from entry to nominal
    float accel_dist_mm   = (nominal_mms * nominal_mms - entry_mms * entry_mms)
                             / (2.0f * accel);
    // Distance needed to decelerate from nominal to exit
    float decel_dist_mm   = (nominal_mms * nominal_mms - exit_mms  * exit_mms)
                             / (2.0f * accel);

    if (accel_dist_mm < 0.0f) accel_dist_mm = 0.0f;
    if (decel_dist_mm < 0.0f) decel_dist_mm = 0.0f;

    // Handle case where accel+decel exceeds available distance (no cruise plateau)
    if (accel_dist_mm + decel_dist_mm > distance_mm) {
        // Solve for the intersection point: v_peak where ramp fits exactly
        // v_peak² = (2*a*d + v_entry² + v_exit²) / 2   (both ramps same accel)
        float v_peak_sq = (2.0f * accel * distance_mm
                           + entry_mms * entry_mms
                           + exit_mms  * exit_mms) / 2.0f;
        if (v_peak_sq < 0.0f) v_peak_sq = 0.0f;
        float v_peak = sqrtf(v_peak_sq);
        if (v_peak > nominal_mms) v_peak = nominal_mms;

        accel_dist_mm = (v_peak * v_peak - entry_mms * entry_mms) /(2.0f * accel);
        decel_dist_mm = (v_peak * v_peak - exit_mms  * exit_mms)  / (2.0f * accel);
        if (accel_dist_mm < 0.0f) accel_dist_mm = 0.0f;
        if (decel_dist_mm < 0.0f) decel_dist_mm = 0.0f;

        // Triangle: decel starts where accel ends
        seg->accelerate_until = accel_dist_mm;
        seg->decelerate_after = accel_dist_mm;
    } else {
        // Trapezoid: independent accel and decel with cruise in between
        seg->accelerate_until = accel_dist_mm;
        seg->decelerate_after = distance_mm - decel_dist_mm;
    }

    // Store speeds in mm/s (segment generator will convert to PR4)
    seg->initial_speed = entry_mms;
    seg->nominal_speed = nominal_mms;
    seg->final_speed   = exit_mms;
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

// Arc segment wrapper: computes achievable constant cruise speed for the chord length.
// A short arc chord cannot reach feedrate from rest in a triangle profile — the actual
// peak of that triangle is sqrt(accel * chord_mm). Clamping to this value and passing it
// as the feedrate AND entry/exit velocity produces a flat (no-ramp) segment at arc_cruise,
// which is the correct behaviour. Consecutive arc segments therefore run back-to-back at
// the same rate with no inter-segment deceleration.
MotionSegment* KINEMATICS_LinearMoveSimple(CoordinatePoint start, CoordinatePoint end, float feedrate,
                                          MotionSegment* segment_buffer) {
    CNC_Settings* settings = SETTINGS_GetCurrent();

    // Chord length (XY plane — arcs are always XY in G17)
    float dx = end.coordinate[AXIS_X] - start.coordinate[AXIS_X];
    float dy = end.coordinate[AXIS_Y] - start.coordinate[AXIS_Y];
    float dz = end.coordinate[AXIS_Z] - start.coordinate[AXIS_Z];
    float chord_mm = sqrtf(dx*dx + dy*dy + dz*dz);
    if (chord_mm < 0.0001f) chord_mm = 0.0001f;  // Avoid sqrt(0)

    // Use conservative (lowest) XY acceleration
    float arc_accel = fminf(settings->acceleration[AXIS_X], settings->acceleration[AXIS_Y]);
    if (arc_accel < 1.0f) arc_accel = 1.0f;

    // Triangle-peak speed: maximum speed achievable over chord_mm with full accel+decel
    // v_peak = sqrt(accel * chord_mm)  [from v² = 2*a*(d/2) accel + same decel]
    // arc_accel is mm/s², chord_mm is mm → v_peak is mm/s → convert to mm/min for comparison
    float v_peak_mm_min = sqrtf(arc_accel * chord_mm) * 60.0f;
    float arc_cruise = fminf(feedrate, v_peak_mm_min);

    // Pass arc_cruise (mm/min) as feedrate, convert to mm/s for entry/exit velocities.
    // entry = exit = cruise → nominal = initial = final → constant speed, no Taylor ramps,
    // segments run seamlessly back-to-back at full speed.
    float arc_cruise_mm_s = arc_cruise / 60.0f;
    return KINEMATICS_LinearMove(start, end, arc_cruise, segment_buffer, arc_cruise_mm_s, arc_cruise_mm_s);
}

// ============================================================================
// PURE GRBL CONSTANT-SPEED HOMING SEGMENT (No Taylor, No Planner)
// ============================================================================
// Builds a pure constant-speed segment for homing operations per GRBL spec:
//   - initial_rate = nominal_rate = final_rate (instant cruise start/stop)
//   - accelerate_until = 0, decelerate_after = steps_remaining (no ramps)
//   - speed_locked = true (planner must not modify)
//   - Zero Taylor state (accel_count=0, rest=0, jerk_count=0)
// Used by HOMING_StartSeek/Locate/Pulloff to avoid Taylor profiler interference.
MotionSegment* KINEMATICS_HomingMove(CoordinatePoint start, CoordinatePoint end,
                                     float feedrate_mm_min, MotionSegment* segment_buffer) {
    // Set segment type
    segment_buffer->type = SEGMENT_TYPE_LINEAR;
    
    // Get settings and stepper position
    StepperPosition* stepper = STEPPER_GetPosition();
    
    // Convert to machine coordinates
    CoordinatePoint machine_start = KINEMATICS_WorkToMachine(start);
    CoordinatePoint machine_end = KINEMATICS_WorkToMachine(end);
    
    // Calculate distance in mm (preserve sign for direction)
    float delta_mm[NUM_AXIS];
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        delta_mm[axis] = GET_COORDINATE_AXIS(&machine_end, axis) - GET_COORDINATE_AXIS(&machine_start, axis);
    }
    
    // Convert mm to steps WITH ACCUMULATION and determine dominant axis
    static float step_accumulator[NUM_AXIS] = {0.0f, 0.0f, 0.0f, 0.0f};
    int32_t max_delta = 0;
    segment_buffer->dominant_axis = AXIS_X;
    
    for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
        step_accumulator[axis] += delta_mm[axis] * stepper->steps_per_mm[axis];
        segment_buffer->delta[axis] = (int32_t)step_accumulator[axis];
        step_accumulator[axis] -= (float)segment_buffer->delta[axis];
        
        int32_t current_delta = abs(segment_buffer->delta[axis]);
        if (current_delta > max_delta) {
            max_delta = current_delta;
            segment_buffer->dominant_axis = axis;
        }
    }
    
    segment_buffer->dominant_delta = max_delta;
    segment_buffer->steps_remaining = (uint32_t)max_delta;
    
    // Zero-step guard
    if (max_delta == 0) {
        DEBUG_PRINT_MOTION("[HOMING] Zero-step move\r\n");
        return segment_buffer;
    }
    
    // Calculate constant speed (pure cruise, no ramps)
    // Clamp feedrate to axis max_rate (critical for Z-axis homing)
    float max_rate_mm_min = *g_axis_settings[segment_buffer->dominant_axis].max_rate;
    if (feedrate_mm_min > max_rate_mm_min) {
        DEBUG_PRINT_MOTION("[HOMING] Clamping feedrate %.0f to max_rate %.0f for axis %d\r\n",
                          feedrate_mm_min, max_rate_mm_min, segment_buffer->dominant_axis);
        feedrate_mm_min = max_rate_mm_min;
    }
    
    float feedrate_mm_sec = feedrate_mm_min / 60.0f;
    if (feedrate_mm_sec <= 0.0f) feedrate_mm_sec = 10.0f;  // Fallback: 600 mm/min
    
    // Pure constant-speed: all speeds equal (segment generator will handle)
    segment_buffer->initial_speed = feedrate_mm_sec;
    segment_buffer->nominal_speed = feedrate_mm_sec;
    segment_buffer->final_speed = feedrate_mm_sec;
    
    // No acceleration phases (pure cruise from start to end)
    float distance_mm = (float)max_delta / *g_axis_settings[segment_buffer->dominant_axis].steps_per_mm;
    segment_buffer->accelerate_until = 0.0f;
    segment_buffer->decelerate_after = distance_mm;
    segment_buffer->millimeters = distance_mm;
    segment_buffer->acceleration = 0.0f;  // No acceleration needed
    
    // Mark as speed-locked (planner must not modify)
    segment_buffer->speed_locked = true;
    
    DEBUG_PRINT_MOTION("[HOMING] axis=%d steps=%lu speed=%.1f mm/s (%.0f mm/min)\r\n",
                      segment_buffer->dominant_axis, segment_buffer->steps_remaining,
                      feedrate_mm_sec, feedrate_mm_min);
    
    return segment_buffer;
}
