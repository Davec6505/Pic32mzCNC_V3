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
        segment_buffer->steps_completed = 0;
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

    // Array-based steps_per_mm lookup for dominant axis
    float steps_per_mm_dominant = *g_axis_settings[segment_buffer->dominant_axis].steps_per_mm;
    (void)limiting_axis; // no longer used — limit_acceleration_by_axis handles this
    
    // Calculate nominal step interval (cruise speed)
    // steps_per_sec = feedrate_mm_sec * steps_per_mm
    float steps_per_sec = feedrate_mm_sec * steps_per_mm_dominant;
    if (steps_per_sec < 1.0f) {
        // Ensure at least 1 step/sec to avoid INF/NaN conversions
        steps_per_sec = 1.0f;
    }
    segment_buffer->nominal_rate = (uint32_t)(g_timer_freq / steps_per_sec);
    
    DEBUG_PRINT_MOTION("[KIN] F=%.0f mm/min, spm=%.1f, nominal=%lu ticks, a=%.1f mm/s2\r\n",
        feedrate, steps_per_mm_dominant,
        segment_buffer->nominal_rate, acceleration_mm_sec2);
    
    // =========================================================================
    // JUNCTION-AWARE TRAPEZOIDAL VELOCITY PROFILE
    // =========================================================================
    // Accel ramp: entry_v  →  cruise   uses (v_cruise² - v_entry²) / (2a)
    // Decel ramp: cruise   →  exit_v   uses (v_cruise² - v_exit²)  / (2a)
    // These are independent — asymmetric ramps when entry_v ≠ exit_v.
    float total_dist_mm = (float)max_delta / steps_per_mm_dominant;

    float accel_dist_mm = (feedrate_mm_sec * feedrate_mm_sec - entry_velocity * entry_velocity)
                          / (2.0f * acceleration_mm_sec2);
    float decel_dist_mm = (feedrate_mm_sec * feedrate_mm_sec - exit_velocity  * exit_velocity)
                          / (2.0f * acceleration_mm_sec2);
    if (accel_dist_mm < 0.0f) accel_dist_mm = 0.0f;
    if (decel_dist_mm < 0.0f) decel_dist_mm = 0.0f;

    uint32_t accel_steps = (uint32_t)(accel_dist_mm * steps_per_mm_dominant);
    uint32_t decel_steps = (uint32_t)(decel_dist_mm * steps_per_mm_dominant);

    if (accel_steps + decel_steps > (uint32_t)max_delta) {
        // Triangle: ramps exceed total steps — find peak speed where they just fit.
        // v_peak² = a * total_dist + (v_entry² + v_exit²) / 2
        float v_peak_sq = acceleration_mm_sec2 * total_dist_mm
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

        accel_steps = (uint32_t)(accel_dist_mm * steps_per_mm_dominant);
        decel_steps = (uint32_t)(decel_dist_mm * steps_per_mm_dominant);

        // Hard clamp: floating-point rounding can push sum 1 step over
        if (accel_steps + decel_steps > (uint32_t)max_delta) {
            accel_steps = (uint32_t)max_delta / 2;
            decel_steps = (uint32_t)max_delta - accel_steps;
        }

        // Triangle: decel starts immediately where accel ends
        segment_buffer->accelerate_until = accel_steps;
        segment_buffer->decelerate_after = accel_steps;
    } else {
        // Trapezoid: independent accel and decel with cruise in between
        segment_buffer->accelerate_until = accel_steps;
        segment_buffer->decelerate_after = (uint32_t)max_delta - decel_steps;
    }
    
    // =========================================================================
    // TAYLOR SERIES INITIAL CONDITIONS  (Austin/Aryzen algorithm)
    // =========================================================================
    // c₀ = TIMER_FREQ · √(2 / (a · spm))  — physics first-step interval from rest.
    //
    // Problem: raw c₀ can be thousands of ticks (< 200 Hz step rate).  At those
    // rates most stepper motors/drivers won't produce enough torque to actually
    // move, especially under load.  We therefore clamp c₀ to a minimum step
    // frequency (maximum timer interval).
    //
    // Taylor index formula:   n = 2·v²·spm/a − 1
    // KEY RULE: n_entry MUST be derived from the ACTUAL initial_rate used,
    // not from a separate velocity.  Inconsistent (rate, n) pairs make the
    // denominator (4n+1) wrong from step 1 → tiny deltas → frozen acceleration.
    //
    // Unified derivation (works for clamped and unclamped c₀):
    //   effective_c  = actual timer interval to be used
    //   v_effective  = TIMER_FREQ / (effective_c · spm)
    //   n            = 2·v_effective²·spm/a − 1  (floor, min 0)
    // When c₀ is NOT clamped this gives n=0 exactly (entry from true rest).
    // When c₀ IS clamped this gives n>0 — correctly reflecting that the motor
    // starts mid-ramp at the minimum practical speed.
    //
    // ACCEL: ISR starts at n_entry, counts UP → shrinking interval → faster.
    // DECEL: ISR starts at -(decel_steps + n_exit), counts UP toward −n_exit.
    //        abs_n shrinks → denominator shrinks → delta grows → strong braking.
    // =========================================================================

    // c₀ = TIMER_FREQ · √(2 / (a · spm))  — physics first-step interval from rest.
    float c0 = g_timer_freq * sqrtf(2.0f / (acceleration_mm_sec2 * steps_per_mm_dominant));
    if (c0 < 1.0f) c0 = 1.0f;

    // -------------------------------------------------------------------------
    // FEEDRATE-RELATIVE START CLAMP
    // -------------------------------------------------------------------------
    // Problem: at high feedrates c0 >> nominal_rate.  Example at F2500 on X:
    //   c0=5328 ticks (146Hz), nominal=219 ticks (3.6kHz) — ratio 24:1.
    //   Taylor needs ~200 steps to converge → visible delay before axis moves.
    //
    // Fix: cap initial_rate at MAX_START_RATIO × nominal_rate.
    //   - Ratio 4 means the axis always starts at ≥25% of cruise speed.
    //   - Derive n_entry via Austin approximation so Taylor continues smoothly
    //     from mid-ramp rather than from true zero (n=0 at a clamped rate would
    //     give den=5 → violent 40% jump on first step).
    //   - This is feedrate- and axis-aware: Z with tiny nominal gets a small
    //     initial_rate (close to cruise), X at F500 is barely clamped.
    // -------------------------------------------------------------------------
    const float MAX_START_RATIO = 4.0f;   // start at most 4× slower than cruise

    // Austin approximation: given c0 (rest) and actual start interval c_start,
    // the mid-ramp Taylor index is  n ≈ (c0/c_start)² − 0.5  (floor, min 0).
    // When c_start == c0 (no clamp) → n = 0.5 → floor 0.  Correct.
    // When c_start < c0 (clamped)  → n > 0.   Correct: ISR starts mid-ramp.
    #define AUSTIN_N(c_start) \
        ((int32_t)fmaxf(0.0f, (c0 / (c_start)) * (c0 / (c_start)) - 0.5f))

    // --- ENTRY (accel start) ---
    // Ceiling: never start slower than max(c0, MAX_START_RATIO × cruise).
    // Applied to BOTH rest-start and junction cases so that a near-zero
    // junction velocity from the planner can't produce a pathologically large
    // initial_rate that breaks the Austin n_entry calculation.
    float max_entry_c = fminf(c0, MAX_START_RATIO * (float)segment_buffer->nominal_rate);

    DEBUG_PRINT_MOTION("[KIN] c0=%.1f 4xnom=%.1f max_entry_c=%.1f entry_vel=%.3f mm/s\r\n",
        c0, MAX_START_RATIO * (float)segment_buffer->nominal_rate,
        max_entry_c, entry_velocity);

    float entry_c;
    if (entry_velocity <= 0.0f) {
        // From rest.
        entry_c = max_entry_c;
    } else {
        // Junction velocity: convert to timer interval, then clamp.
        entry_c = g_timer_freq / (entry_velocity * steps_per_mm_dominant);
        if (entry_c > max_entry_c) entry_c = max_entry_c;
    }
    segment_buffer->initial_rate = (uint32_t)entry_c;
    // Hard floor: can't start faster than cruise.
    if (segment_buffer->initial_rate < segment_buffer->nominal_rate)
        segment_buffer->initial_rate = segment_buffer->nominal_rate;

    DEBUG_PRINT_MOTION("[KIN] initial_rate=%lu accel_count_pre=%ld\r\n",
        segment_buffer->initial_rate,
        AUSTIN_N((float)segment_buffer->initial_rate));

    segment_buffer->accel_count = AUSTIN_N((float)segment_buffer->initial_rate);
    segment_buffer->rest = 0;

    // --- EXIT (decel end) ---
    // Decel stop case: use c0 directly (full deceleration to physical rest).
    //   Do NOT apply the 4× clamp here — that would truncate decel too early,
    //   leaving the motor coasting at 4× cruise speed when it should be at rest.
    // Junction exit case: convert to ticks; clamp only if near-zero junction
    //   would produce a pathologically large exit_c (same issue as entry path).
    //   The backward-pass planner (MOTION_RecomputeExit) will retroactively
    //   correct final_rate once the next segment is known, so the initial value
    //   just needs to be physically plausible.
    float exit_c;
    if (exit_velocity <= 0.0f) {
        // To rest: full c0 — let Taylor decel all the way to near-zero speed.
        exit_c = c0;
    } else {
        // Junction: clamp pathological near-zero junction (same ceiling as entry).
        exit_c = g_timer_freq / (exit_velocity * steps_per_mm_dominant);
        if (exit_c > max_entry_c) exit_c = max_entry_c;
    }
    segment_buffer->final_rate = (uint32_t)exit_c;
    // Hard floor: decel can't end faster than cruise.
    if (segment_buffer->final_rate < segment_buffer->nominal_rate)
        segment_buffer->final_rate = segment_buffer->nominal_rate;

    int32_t n_exit = AUSTIN_N((float)segment_buffer->final_rate);

    #undef AUSTIN_N

    segment_buffer->accel_count_decel = -((int32_t)decel_steps + n_exit);

    // S-curve jerk disabled — perfecting junction-velocity trapezoidal profile first.
    segment_buffer->jerk_steps      = 0;
    segment_buffer->jerk_steps_log2 = 0;
    segment_buffer->jerk_count      = 0;

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

    // Physics parameters (for debugging/reference)
    segment_buffer->start_velocity = entry_velocity;
    segment_buffer->max_velocity = feedrate_mm_sec;
    segment_buffer->end_velocity = exit_velocity;
    segment_buffer->acceleration = acceleration_mm_sec2;
    
    // Motion state initialization
    segment_buffer->steps_remaining = max_delta;
    segment_buffer->steps_completed = 0;
    // Start at initial_rate (safe-start speed). ISR reads step_interval each tick.
    // Main loop velocity profiler updates step_interval to ramp toward nominal_rate.
    segment_buffer->step_interval = segment_buffer->initial_rate;
    
    // Pulse width from settings (reuse existing calculation from stepper.c)
    segment_buffer->pulse_width = (uint32_t)(settings->step_pulse_time * 12.5f);  // µs to timer ticks
    
    // Initialize Bresenham error terms (symmetric rounding)
    for (E_AXIS axis = AXIS_Y; axis < NUM_AXIS; axis++) {
        segment_buffer->error[axis] = max_delta / 2;
    }
    
    return segment_buffer;
}

// ============================================================================
// KINEMATICS_RecalculateTrapezoid
//
// Re-computes the ISR timing parameters (initial_rate, nominal_rate, final_rate,
// accelerate_until, decelerate_after) for a segment given its planned
// entry_mms and exit_mms speeds.
//
// Called by MOTION_PlannerRecalculate() after the reverse+forward pass has
// determined the correct entry and exit speeds for every buffered segment.
// Exact port of GRBL's calculate_trapezoid_for_block().
// ============================================================================
void KINEMATICS_RecalculateTrapezoid(MotionSegment *seg,
                                     float entry_mms,
                                     float exit_mms)
{
    if (!seg || seg->steps_remaining == 0) { return; }

    float accel          = seg->acceleration;          // mm/s²
    float nominal_mms    = seg->max_velocity;          // feedrate stored at plan time
    float distance_mm    = seg->millimeters;

    // Clamp to nominal so we never exceed programmed feedrate
    if (entry_mms > nominal_mms) { entry_mms = nominal_mms; }
    if (exit_mms  > nominal_mms) { exit_mms  = nominal_mms; }

    // Total steps in this block
    uint32_t total_steps  = seg->steps_remaining + seg->steps_completed; // full count
    float    steps_per_mm = (distance_mm > 0.0f)
                            ? ((float)total_steps / distance_mm)
                            : 1.0f;

    // Convert speeds to step rates (steps/s)
    float entry_rate   = entry_mms   * steps_per_mm;
    float nominal_rate = nominal_mms * steps_per_mm;
    float exit_rate    = exit_mms    * steps_per_mm;
    float accel_steps  = accel       * steps_per_mm; // steps/s²

    // ---- Trapezoidal intersection point in step space (GRBL calc_trapezoid) ----
    // Steps needed to accelerate from entry to nominal
    float accel_steps_f   = (nominal_rate * nominal_rate - entry_rate * entry_rate)
                             / (2.0f * accel_steps);
    // Steps needed to decelerate from nominal to exit
    float decel_steps_f   = (nominal_rate * nominal_rate - exit_rate  * exit_rate)
                             / (2.0f * accel_steps);

    int32_t accelerate_steps = (int32_t)ceilf(accel_steps_f);
    int32_t decelerate_steps = (int32_t)floorf(decel_steps_f);

    // Handle case where accel+decel exceeds available steps (no cruise plateau)
    int32_t plateau = (int32_t)total_steps - accelerate_steps - decelerate_steps;
    if (plateau < 0) {
        // Solve for the intersection point: v_peak where ramp fits exactly
        // v_peak² = (2*a*d + v_entry² + v_exit²) / 2   (both ramps same accel)
        float intersect = (2.0f * accel_steps * (float)total_steps
                           + entry_rate * entry_rate
                           + exit_rate  * exit_rate) / (4.0f * accel_steps);
        accelerate_steps = (int32_t)ceilf(
            (intersect - entry_rate * entry_rate) / (2.0f * accel_steps));
        accelerate_steps = (accelerate_steps < 0)           ? 0
                         : (accelerate_steps > (int32_t)total_steps) ? (int32_t)total_steps
                         : accelerate_steps;
        decelerate_steps = (int32_t)total_steps - accelerate_steps;
    }

    // ---- Convert step rates to timer-tick periods (same units as KINEMATICS_LinearMove) ----
    // CRITICAL: All seg->*_rate fields MUST be timer-tick periods, not steps/sec, because:
    //   • STEPPER_LoadSegment reads initial_rate directly as PR4 (period register)
    //   • ISR uses nominal_rate as step_interval floor, final_rate as decel ceiling
    // Storing steps/sec here causes the ISR to clamp step_interval to a huge period,
    // making the motor run at ~55 mm/min instead of 8000 mm/min (unit mismatch bug).
    float c_nominal = (nominal_rate > 0.0f) ? (g_timer_freq / nominal_rate) : g_timer_freq;
    float c_entry   = (entry_rate   > 0.0f) ? (g_timer_freq / entry_rate)   : g_timer_freq;
    float c_exit    = (exit_rate    > 0.0f) ? (g_timer_freq / exit_rate)    : g_timer_freq;

    // Entry/exit periods cannot be smaller (faster) than the cruise period.
    if (c_entry < c_nominal) c_entry = c_nominal;
    if (c_exit  < c_nominal) c_exit  = c_nominal;

    // Apply MAX_START_RATIO=4 clamp — same as KINEMATICS_LinearMove — so a short replan
    // doesn't produce a pathologically slow entry (> 4× cruise period).
    float max_start_c = 4.0f * c_nominal;
    if (c_entry > max_start_c) c_entry = max_start_c;
    if (c_exit  > max_start_c) c_exit  = max_start_c;

    // Hardware minimum: 7 ticks (pulse-width guard, same as STEPPER_LoadSegment).
    if (c_nominal < 7.0f) c_nominal = 7.0f;
    if (c_entry   < 7.0f) c_entry   = 7.0f;
    if (c_exit    < 7.0f) c_exit    = 7.0f;

    seg->initial_rate     = (uint32_t)c_entry;
    seg->nominal_rate     = (uint32_t)c_nominal;
    seg->final_rate       = (uint32_t)c_exit;
    seg->accelerate_until = (uint32_t)accelerate_steps;
    seg->decelerate_after = (uint32_t)((int32_t)total_steps - decelerate_steps);

    // Recompute Austin/Aryzen n_entry so the ISR Taylor ramp starts at the correct index.
    // n ≈ (c0 / c_entry)² − 0.5  (same formula as the AUSTIN_N macro in LinearMove).
    // Without this, accel_count is stale from LinearMove — wrong denominator from step 1.
    float c0_rt = g_timer_freq * sqrtf(2.0f / (accel * steps_per_mm));
    if (c0_rt < 1.0f) c0_rt = 1.0f;

    float ratio_entry      = c0_rt / (float)seg->initial_rate;
    seg->accel_count       = (int32_t)fmaxf(0.0f, ratio_entry * ratio_entry - 0.5f);
    seg->rest              = 0;
    seg->jerk_count        = 0;

    // Recompute decel starting index (symmetric mirror of entry).
    float ratio_exit       = c0_rt / (float)seg->final_rate;
    int32_t n_exit_rt      = (int32_t)fmaxf(0.0f, ratio_exit * ratio_exit - 0.5f);
    seg->accel_count_decel = -((int32_t)decelerate_steps + n_exit_rt);

    // step_interval starts at entry rate (profiler updates it each ISR tick).
    seg->step_interval = seg->initial_rate;
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

    // Pass arc_cruise as feedrate AND entry/exit → nominal = initial = final → flat cruise,
    // profiler does nothing, segments run seamlessly back-to-back.
    return KINEMATICS_LinearMove(start, end, arc_cruise, segment_buffer, arc_cruise, arc_cruise);
}
