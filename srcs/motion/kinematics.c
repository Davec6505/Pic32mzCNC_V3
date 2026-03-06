/*
 * kinematics.c — Work Coordinate System (WCS) transforms  (S-curve build)
 *
 * This file is ONLY responsible for coordinate-space mathematics:
 *   • Machine ↔ work coordinate conversion
 *   • Active WCS offset management (G54–G59)
 *   • G92 temporary work offset
 *   • Reading / setting current machine position from stepper step counts
 *   • Per-axis machine position override (used by homing)
 *
 * All velocity planning, trajectory math, and motion profiling has been
 * moved to trajectory.c / interpolator.c.  Stub implementations of the
 * old planning API are retained here so that any remaining callers link
 * without error while they are migrated; they all return NULL / false / 0.0.
 *
 * WCS model (GRBL-compatible)
 * ─────────────────────────────
 *   machine_pos = work_pos + wcs_offset[active]
 *   work_pos    = machine_pos - wcs_offset[active]
 *
 * Offsets are stored in flash via SETTINGS_Get/SetWorkCoordinateSystem().
 * A separate in-memory g92_offset[] holds the G92 temporary shift (not
 * persisted — cleared on reset).
 */

#include "motion/kinematics.h"
#include "settings/settings.h"
#include "utils/utils.h"
#include "data_structures.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

// ─── Work coordinate state ────────────────────────────────────────────────────

// Number of supported work coordinate systems (G54..G59 = 0..5)
#define NUM_WCS  6u

// Single internal work-coordinate instance (returned by GetWorkCoordinates)
static WorkCoordinateSystem g_wcs;

// G92 temporary offset (machine coordinates, not flash-backed)
static float g92_offset[NUM_AXIS];

// Tool Length Offset — active value in mm, applied to Z only (GRBL G43/G49 semantics)
// Not flash-backed; cleared on soft reset via KINEMATICS_ClearTLO().
static float  s_tlo_value  = 0.0f;
static bool   s_tlo_active = false;

// Active Work Coordinate System index (0=G54 … 5=G59)
// Loaded from flash into g_wcs at init and whenever KINEMATICS_SetActiveWCS() is called.
static uint8_t s_active_wcs = 0;

// ─── Initialisation ────────────────────────────────────────────────────────────

void KINEMATICS_Initialize(void)
{
    memset(&g_wcs, 0, sizeof(g_wcs));
    memset(g92_offset, 0, sizeof(g92_offset));
    s_tlo_value  = 0.0f;
    s_tlo_active = false;
    s_active_wcs = 0;
    // Load G54 (slot 0) as the default active WCS
    float ox = 0.0f, oy = 0.0f, oz = 0.0f;
    SETTINGS_GetWorkCoordinateSystem(0u, &ox, &oy, &oz);
    g_wcs.offset.coordinate[AXIS_X] = ox;
    g_wcs.offset.coordinate[AXIS_Y] = oy;
    g_wcs.offset.coordinate[AXIS_Z] = oz;
    g_wcs.offset.coordinate[AXIS_A] = 0.0f;
}

// ─── Simple (single-WCS) coordinate transforms ────────────────────────────────
//
// These use whatever offsets are currently cached in g_wcs.

CoordinatePoint KINEMATICS_WorkToMachine(CoordinatePoint work_pos)
{
    CoordinatePoint machine;
    for (int i = 0; i < NUM_AXIS; i++) {
        machine.coordinate[i] = work_pos.coordinate[i]
                               + g_wcs.offset.coordinate[i]
                               + g92_offset[i];
    }
    if (s_tlo_active) {
        machine.coordinate[AXIS_Z] += s_tlo_value;
    }
    return machine;
}

CoordinatePoint KINEMATICS_MachineToWork(CoordinatePoint machine_pos)
{
    CoordinatePoint work;
    for (int i = 0; i < NUM_AXIS; i++) {
        work.coordinate[i] = machine_pos.coordinate[i]
                            - g_wcs.offset.coordinate[i]
                            - g92_offset[i];
    }
    if (s_tlo_active) {
        work.coordinate[AXIS_Z] -= s_tlo_value;
    }
    return work;
}

// ─── Multi-WCS transforms ─────────────────────────────────────────────────────
//
// activeWCS: 0=G54, 1=G55, 2=G56, 3=G57, 4=G58, 5=G59.
// Offsets are read directly from flash settings every time to stay consistent
// with any $10x= commands that may have updated them.

CoordinatePoint KINEMATICS_WorkToMachineWithWCS(CoordinatePoint work_pos,
                                                 uint8_t         activeWCS)
{
    float ox = 0.0f, oy = 0.0f, oz = 0.0f;
    if (activeWCS < NUM_WCS) {
        SETTINGS_GetWorkCoordinateSystem(activeWCS, &ox, &oy, &oz);
    }
    CoordinatePoint machine;
    machine.coordinate[AXIS_X] = work_pos.coordinate[AXIS_X] + ox + g92_offset[AXIS_X];
    machine.coordinate[AXIS_Y] = work_pos.coordinate[AXIS_Y] + oy + g92_offset[AXIS_Y];
    machine.coordinate[AXIS_Z] = work_pos.coordinate[AXIS_Z] + oz + g92_offset[AXIS_Z];
    if (s_tlo_active) {
        machine.coordinate[AXIS_Z] += s_tlo_value;
    }
    machine.coordinate[AXIS_A] = work_pos.coordinate[AXIS_A]      + g92_offset[AXIS_A];
    return machine;
}

CoordinatePoint KINEMATICS_MachineToWorkWithWCS(CoordinatePoint machine_pos,
                                                 uint8_t         activeWCS)
{
    float ox = 0.0f, oy = 0.0f, oz = 0.0f;
    if (activeWCS < NUM_WCS) {
        SETTINGS_GetWorkCoordinateSystem(activeWCS, &ox, &oy, &oz);
    }
    CoordinatePoint work;
    work.coordinate[AXIS_X] = machine_pos.coordinate[AXIS_X] - ox - g92_offset[AXIS_X];
    work.coordinate[AXIS_Y] = machine_pos.coordinate[AXIS_Y] - oy - g92_offset[AXIS_Y];
    work.coordinate[AXIS_Z] = machine_pos.coordinate[AXIS_Z] - oz - g92_offset[AXIS_Z];
    if (s_tlo_active) {
        work.coordinate[AXIS_Z] -= s_tlo_value;
    }
    work.coordinate[AXIS_A] = machine_pos.coordinate[AXIS_A]      - g92_offset[AXIS_A];
    return work;
}

// ─── WCS offset management ────────────────────────────────────────────────────

void KINEMATICS_SetWorkOffset(float x_offset, float y_offset, float z_offset)
{
    g_wcs.offset.coordinate[AXIS_X] = x_offset;
    g_wcs.offset.coordinate[AXIS_Y] = y_offset;
    g_wcs.offset.coordinate[AXIS_Z] = z_offset;
    g_wcs.offset.coordinate[AXIS_A] = 0.0f;
}

void KINEMATICS_GetActiveWCSOffset(uint8_t  activeWCS,
                                    float   *x_offset,
                                    float   *y_offset,
                                    float   *z_offset)
{
    float ox = 0.0f, oy = 0.0f, oz = 0.0f;
    if (activeWCS < NUM_WCS) {
        SETTINGS_GetWorkCoordinateSystem(activeWCS, &ox, &oy, &oz);
    }
    if (x_offset) *x_offset = ox + g92_offset[AXIS_X];
    if (y_offset) *y_offset = oy + g92_offset[AXIS_Y];
    if (z_offset) *z_offset = oz + g92_offset[AXIS_Z];
}

WorkCoordinateSystem *KINEMATICS_GetWorkCoordinates(void)
{
    return &g_wcs;
}

void KINEMATICS_SetWorkCoordinates(float x, float y, float z)
{
    g_wcs.offset.coordinate[AXIS_X] = x;
    g_wcs.offset.coordinate[AXIS_Y] = y;
    g_wcs.offset.coordinate[AXIS_Z] = z;
}

// ─── G92 temporary offset ─────────────────────────────────────────────────────
//
// GRBL G92 sets work position at current machine position.
// Offset = machine_pos - desired_work_pos.
// Called from app.c when a G92 event arrives.

void KINEMATICS_SetG92Offset(const float machine[NUM_AXIS],
                              const float desired_work[NUM_AXIS])
{
    for (int i = 0; i < NUM_AXIS; i++) {
        g92_offset[i] = machine[i] - desired_work[i];
    }
}

void KINEMATICS_ClearG92Offset(void)
{
    memset(g92_offset, 0, sizeof(g92_offset));
}

// ─── Active WCS management (G54–G59) ──────────────────────────────────────────────
//
// G54=0, G55=1 … G59=5.  Reloading from flash keeps g_wcs consistent with
// any $10x= writes or G10 commands that may have modified a slot.

void KINEMATICS_SetActiveWCS(uint8_t wcs_number)
{
    if (wcs_number > 5u) return;  // Only G54–G59 supported
    s_active_wcs = wcs_number;
    float ox = 0.0f, oy = 0.0f, oz = 0.0f;
    SETTINGS_GetWorkCoordinateSystem(wcs_number, &ox, &oy, &oz);
    g_wcs.offset.coordinate[AXIS_X] = ox;
    g_wcs.offset.coordinate[AXIS_Y] = oy;
    g_wcs.offset.coordinate[AXIS_Z] = oz;
    g_wcs.offset.coordinate[AXIS_A] = 0.0f;  // A-axis offset not stored (always 0)
}

uint8_t KINEMATICS_GetActiveWCS(void)
{
    return s_active_wcs;
}

// ─── Tool Length Offset (G43 / G43.1 / G49) ──────────────────────────────────
//
// TLO is applied to the Z axis only (GRBL convention).
// It shifts the machine Z target so that Z=0 in work space corresponds to the
// tip of the current tool rather than the reference gauge-length plane.
// Positive TLO = tool is longer than reference → spindle lifts to compensate.

void KINEMATICS_SetTLO(float offset_mm)
{
    s_tlo_value  = offset_mm;
    s_tlo_active = true;
}

void KINEMATICS_ClearTLO(void)
{
    s_tlo_value  = 0.0f;
    s_tlo_active = false;
}

float KINEMATICS_GetTLO(bool *active_out)
{
    if (active_out) *active_out = s_tlo_active;
    return s_tlo_active ? s_tlo_value : 0.0f;
}

// ─── Position from stepper step counts ────────────────────────────────────────
//
// The authoritative machine position is the step count maintained by the ISR
// for each axis (via AXIS_IncrementSteps/AXIS_DecrementSteps).
// Dividing by steps_per_mm gives position in mm.

CoordinatePoint KINEMATICS_GetCurrentPosition(void)
{
    CoordinatePoint pos;
    for (int i = 0; i < NUM_AXIS; i++) {
        float spm = *g_axis_settings[i].steps_per_mm;
        int32_t sc = *g_axis_settings[i].step_count;
        pos.coordinate[i] = (spm > 1e-9f) ? ((float)sc / spm) : 0.0f;
    }
    return pos;
}

void KINEMATICS_SetAxisMachinePosition(E_AXIS axis, float position)
{
    float spm = *g_axis_settings[axis].steps_per_mm;
    *g_axis_settings[axis].step_count = (int32_t)(position * spm);
}

// ─── No-op / Stub resets ──────────────────────────────────────────────────────
//
// These had meaning in the old Bresenham / trapezoid engine.
// The new engine has no equivalent state to reset here.

void KINEMATICS_ResetAccumulators(void)
{
    // Nothing to reset in the DDS engine — accumulators live in interpolator.c
}

void KINEMATICS_ResetPlannerState(void)
{
    // Nothing to reset — planner state is inside trajectory.c (static)
    // Call TRAJECTORY_Reset() instead if a full queue flush is needed.
}

// ─── Old planning API stubs ───────────────────────────────────────────────────
//
// Retained for link compatibility only.  Do NOT call these from new code.
// All planning now goes through TRAJECTORY_AddMove() + INTERPOLATOR_LoadMove().

MotionSegment *KINEMATICS_LinearMove(CoordinatePoint start,
                                      CoordinatePoint end,
                                      float           feedrate,
                                      MotionSegment  *segment_buffer,
                                      float           entry_velocity,
                                      float           exit_velocity)
{
    (void)start; (void)end; (void)feedrate;
    (void)segment_buffer; (void)entry_velocity; (void)exit_velocity;
    return NULL;   // Replaced by TRAJECTORY_AddMove()
}

MotionSegment *KINEMATICS_LinearMoveSimple(CoordinatePoint start,
                                            CoordinatePoint end,
                                            float           feedrate,
                                            MotionSegment  *segment_buffer)
{
    (void)start; (void)end; (void)feedrate; (void)segment_buffer;
    return NULL;
}

MotionSegment *KINEMATICS_HomingMove(CoordinatePoint start,
                                      CoordinatePoint end,
                                      float           feedrate_mm_min,
                                      MotionSegment  *segment_buffer)
{
    (void)start; (void)end; (void)feedrate_mm_min; (void)segment_buffer;
    return NULL;
}

MotionSegment *KINEMATICS_ArcMove(CoordinatePoint start,
                                   CoordinatePoint end,
                                   CoordinatePoint center,
                                   bool            clockwise,
                                   float           feedrate,
                                   MotionSegment  *segment_buffer)
{
    (void)start; (void)end; (void)center;
    (void)clockwise; (void)feedrate; (void)segment_buffer;
    return NULL;
}

bool KINEMATICS_PlanArc(CoordinatePoint  start,
                         CoordinatePoint  end,
                         CoordinatePoint  center,
                         bool             clockwise,
                         float            feedrate,
                         float           *out_radius,
                         float           *out_total_angle,
                         uint32_t        *out_num_segments)
{
    (void)start; (void)end; (void)center;
    (void)clockwise; (void)feedrate;
    if (out_radius)       *out_radius       = 0.0f;
    if (out_total_angle)  *out_total_angle  = 0.0f;
    if (out_num_segments) *out_num_segments = 0u;
    return false;
}

float KINEMATICS_CalculateJunctionSpeed(CoordinatePoint prev_dir,
                                         CoordinatePoint curr_dir,
                                         float           junction_deviation,
                                         float           acceleration)
{
    (void)prev_dir; (void)curr_dir;
    (void)junction_deviation; (void)acceleration;
    return 0.0f;
}

void KINEMATICS_RecalculateTrapezoid(MotionSegment *seg,
                                      float          entry_mms,
                                      float          exit_mms)
{
    (void)seg; (void)entry_mms; (void)exit_mms;
}
