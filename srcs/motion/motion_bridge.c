/*
 * motion_bridge.c — Compatibility shim for the S-curve motion engine
 *
 * Provides stub implementations of the old `stepper.c` and `motion.c` APIs
 * so that `app.c`, `gcode_parser.c`, `utils.c`, and `homing.c` continue to
 * link without modification while the full wiring migration is in progress.
 *
 * Key bridges (old → new):
 *   STEPPER_Initialize      → INTERPOLATOR_Initialize + TRAJECTORY_Initialize
 *   STEPPER_DisableAll      → STEPPERS_Disable (GPIO macro from utils.h)
 *   STEPPER_EnableAll       → STEPPERS_Enable
 *   STEPPER_StopMotion      → INTERPOLATOR_Stop
 *   STEPPER_PauseMotion     → INTERPOLATOR_Stop
 *   STEPPER_ResumeMotion    → INTERPOLATOR_Resume
 *   MOTION_Initialize       → TRAJECTORY_Initialize
 *   MOTION_Tasks            → INTERPOLATOR_MoveComplete poll → INTERPOLATOR_LoadMove
 *   STEPPER_GetPosition     → internally maintained StepperPosition from step counts
 *   STEPPER_GetPositionPointer → same struct (used by utils.c to wire step_count pointers)
 *
 * Global flags (g_hard_limit_alarm, etc.) live here and are shared with
 * app.c, gcode_parser.c, and homing.c via `extern` declarations in stepper.h.
 */

#include "motion/stepper.h"
#include "motion/motion.h"
#include "motion/trajectory.h"
#include "motion/interpolator.h"
#include "motion/kinematics.h"
#include "settings/settings.h"
#include "utils/utils.h"
#include "data_structures.h"
#include "gcode/gcode_parser.h"
#include <string.h>
#include <math.h>

// ─── Shared volatile flags (declared extern in stepper.h) ────────────────────

volatile bool g_hard_limit_alarm    = false;
volatile bool g_suppress_hard_limits= false;
volatile bool g_estop_pending       = false;
volatile bool g_feed_hold_active    = false;
volatile bool g_feed_hold_pending   = false;

// ─── StepperPosition instance ────────────────────────────────────────────────
//
// utils.c calls STEPPER_GetPositionPointer() once during APP_CONFIG and stores
// pointers into g_axis_settings[axis].step_count from this struct.
// The S-curve interpolator in interpolator.c calls AXIS_IncrementSteps/
// AXIS_DecrementSteps which write directly to *g_axis_settings[axis].step_count,
// so these arrays stay in sync automatically.

static StepperPosition s_stepper_pos;

// ─── STEPPER API bridges ──────────────────────────────────────────────────────

void STEPPER_Initialize(APP_DATA *appData)
{
    (void)appData;
    memset(&s_stepper_pos, 0, sizeof(s_stepper_pos));
    // Initialise steps_per_mm from settings (same as old stepper.c did)
    const CNC_Settings *s = SETTINGS_GetCurrent();
    for (int i = 0; i < NUM_AXIS; i++) {
        s_stepper_pos.steps_per_mm[i] = s->steps_per_mm[i];
    }
    TRAJECTORY_Initialize();
    INTERPOLATOR_Initialize();
}

void STEPPER_DisableAll(void)
{
    // Single shared enable pin (EnXYZA / RE6), active-LOW.
    // STEPPERS_Disable() from utils.h drives the correct GPIO macro.
    STEPPERS_Disable();
}

void STEPPER_EnableAll(void)
{
    STEPPERS_Enable();
}

void STEPPER_StopMotion(void)
{
    INTERPOLATOR_Stop();
    TRAJECTORY_Reset();
}

void STEPPER_PauseMotion(void)
{
    if (!INTERPOLATOR_IsActive()) {
        // Already idle — go straight to hold
        g_feed_hold_pending = false;
        g_feed_hold_active  = true;
    } else {
        g_feed_hold_pending = true;
        INTERPOLATOR_Stop();
        g_feed_hold_pending = false;
        g_feed_hold_active  = true;
    }
}

void STEPPER_FinalizeHold(void)
{
    g_feed_hold_pending = false;
    g_feed_hold_active  = true;
}

void STEPPER_ResumeMotion(void)
{
    g_feed_hold_active  = false;
    g_feed_hold_pending = false;
    INTERPOLATOR_Resume();
}

void STEPPER_ReloadSettings(void)
{
    // Steps_per_mm cached in s_stepper_pos; refresh from flash
    const CNC_Settings *s = SETTINGS_GetCurrent();
    for (int i = 0; i < NUM_AXIS; i++) {
        s_stepper_pos.steps_per_mm[i] = s->steps_per_mm[i];
    }
}

StepperPosition *STEPPER_GetPosition(void)
{
    // Sync steps[] from the live step_count pointers (kept up to date by ISR)
    for (int i = 0; i < NUM_AXIS; i++) {
        if (g_axis_settings[i].step_count != NULL) {
            s_stepper_pos.steps[i] = *g_axis_settings[i].step_count;
        }
    }
    return &s_stepper_pos;
}

StepperPosition *STEPPER_GetPositionPointer(void)
{
    // utils.c calls this to wire g_axis_settings[axis].step_count
    return &s_stepper_pos;
}

float STEPPER_GetCurrentFeedrateMmMin(void)
{
    // Return instantaneous velocity from the DDS engine converted to mm/min
    return INTERPOLATOR_GetInstantVelocity() * 60.0f;
}

bool STEPPER_IsEnabled(void)
{
    return INTERPOLATOR_IsActive();
}

bool STEPPER_IsDwellComplete(void)
{
    // Dwell segments not yet implemented in new engine — always report complete
    return true;
}

void STEPPER_LoadSegment(MotionSegment *segment)
{
    // Old interface — not used by new pipeline (trajectory.c → interpolator.c)
    (void)segment;
}

void STEPPER_SetDirection(E_AXIS axis, bool forward)
{
    (void)axis; (void)forward;
    // Direction is handled per-move inside INTERPOLATOR_LoadMove
}

// ─── MOTION API bridges ───────────────────────────────────────────────────────

void MOTION_Initialize(void)
{
    TRAJECTORY_Initialize();
}

// MOTION_Tasks — the key pipeline pump.
// Called from APP_IDLE every main-loop iteration.
// Polls the interpolator; when a move finishes, loads the next one.
void MOTION_Tasks(APP_DATA *appData)
{
    (void)appData;

    // Feed-hold gating
    if (g_feed_hold_active) return;

    // If interpolator just finished a move, pop the next one
    if (INTERPOLATOR_MoveComplete()) {
        SCurveMove next_move;
        if (TRAJECTORY_GetNextMove(&next_move)) {
            INTERPOLATOR_LoadMove(&next_move);
        }
    }
}

// MOTION_Arc — incremental arc segment generation (one segment per call).
// Called by app.c every idle iteration while appData->arcGenState == ARC_GEN_ACTIVE.
// Generates exactly one trajectory segment per invocation so the main loop
// is never blocked, and backs off gracefully when the queue is full.
void MOTION_Arc(APP_DATA *appData)
{
    if (appData == NULL || appData->arcGenState != ARC_GEN_ACTIVE) return;

    // Back off if trajectory queue is almost full (leave 2 slots headroom)
    if (TRAJECTORY_QueueCount() >= (uint32_t)(TRAJ_QUEUE_SIZE - 2)) return;

    // Advance segment counter; determine whether this is the final segment
    appData->arcSegmentCurrent++;
    bool is_last = (appData->arcSegmentCurrent >= appData->arcSegmentTotal);

    CoordinatePoint next;

    if (is_last) {
        // Use the exact programmed end point to avoid accumulated rounding error
        next = appData->arcEndPoint;
    } else {
        // Advance angle and compute new XY position
        appData->arcTheta += appData->arcThetaIncrement;
        float cx = appData->arcCenter.coordinate[AXIS_X];
        float cy = appData->arcCenter.coordinate[AXIS_Y];
        next.coordinate[AXIS_X] = cx + appData->arcRadius * cosf(appData->arcTheta);
        next.coordinate[AXIS_Y] = cy + appData->arcRadius * sinf(appData->arcTheta);

        // Linear interpolation for Z and A axes (helical / 4-axis motion)
        float progress = (float)appData->arcSegmentCurrent / (float)appData->arcSegmentTotal;
        float sz = appData->arcStartPoint.coordinate[AXIS_Z];
        float ez = appData->arcEndPoint.coordinate[AXIS_Z];
        float sa = appData->arcStartPoint.coordinate[AXIS_A];
        float ea = appData->arcEndPoint.coordinate[AXIS_A];
        next.coordinate[AXIS_Z] = sz + (ez - sz) * progress;
        next.coordinate[AXIS_A] = sa + (ea - sa) * progress;
    }

    // Attempt to add the segment to the trajectory queue
    if (TRAJECTORY_AddMove(appData->arcCurrent, next, appData->arcFeedrate, 0.0f, 0.0f)) {
        TRAJECTORY_Recalculate();

        appData->arcCurrent = next;

        if (is_last) {
            appData->arcGenState = ARC_GEN_IDLE;
        }

        // Pre-fill gate: only release the interpolator once the queue is deep enough
        // to absorb any main-loop timing jitter without starving the ISR.
        //
        // For short arcs  (total < queue capacity): wait until fully generated.
        // For long arcs   (total ≥ queue capacity): wait until queue is nearly full.
        //
        // After this initial kick, MOTION_Tasks() keeps the interpolator fed by
        // popping the next SCurveMove each time MoveComplete() fires.
        if (!INTERPOLATOR_IsActive()) {
            uint32_t prefill_target =
                (appData->arcSegmentTotal < (uint32_t)(TRAJ_QUEUE_SIZE - 2))
                ? appData->arcSegmentTotal          // short arc: wait for full generation
                : (uint32_t)(TRAJ_QUEUE_SIZE - 2);  // long arc:  wait for near-full queue

            bool prefill_ready = (appData->arcGenState == ARC_GEN_IDLE) ||
                                 (TRAJECTORY_QueueCount() >= prefill_target);

            if (prefill_ready) {
                SCurveMove mv;
                if (TRAJECTORY_GetNextMove(&mv)) {
                    STEPPERS_Enable();
                    INTERPOLATOR_LoadMove(&mv);
                }
            }
        }
    } else {
        // Queue unexpectedly full — roll back and retry next iteration
        appData->arcSegmentCurrent--;
        if (!is_last) {
            appData->arcTheta -= appData->arcThetaIncrement;
        }
    }
}

// MOTION_ProcessGcodeEvent — convert a parsed G-code event to motion.
// Returns true if the event was consumed (even if only queued).
bool MOTION_ProcessGcodeEvent(APP_DATA *appData, GCODE_Event *event)
{
    if (appData == NULL || event == NULL) return false;

    const CNC_Settings *s = SETTINGS_GetCurrent();

    switch (event->type) {

        case GCODE_EVENT_LINEAR_MOVE: {
            // Build start and end CoordinatePoints from machine-space positions.
            CoordinatePoint start = KINEMATICS_GetCurrentPosition();
            CoordinatePoint end;
            // gcode_parser linearMove uses named fields x,y,z,a (not an array)
            end.coordinate[AXIS_X] = event->data.linearMove.x;
            end.coordinate[AXIS_Y] = event->data.linearMove.y;
            end.coordinate[AXIS_Z] = event->data.linearMove.z;
            end.coordinate[AXIS_A] = event->data.linearMove.a;
            float fr = event->data.linearMove.feedrate;
            if (fr < 1.0f) fr = s->max_rate[0];  // fallback

            // Add to trajectory queue; Recalculate blends junction speeds.
            if (TRAJECTORY_AddMove(start, end, fr, 0.0f, 0.0f)) {
                TRAJECTORY_Recalculate();
                // Pump the first move if interpolator is idle
                if (!INTERPOLATOR_IsActive()) {
                    SCurveMove mv;
                    if (TRAJECTORY_GetNextMove(&mv)) {
                        STEPPERS_Enable();
                        INTERPOLATOR_LoadMove(&mv);
                    }
                }
            }
            return true;
        }

        case GCODE_EVENT_ARC_MOVE: {
            // Guard: only one arc at a time
            if (appData->arcGenState == ARC_GEN_ACTIVE) return false;

            CoordinatePoint start = KINEMATICS_GetCurrentPosition();

            // Center in absolute coordinates (I/J are always relative to start per GRBL)
            float cx = start.coordinate[AXIS_X] + event->data.arcMove.centerX;
            float cy = start.coordinate[AXIS_Y] + event->data.arcMove.centerY;

            // Absolute end point
            float ex = event->data.arcMove.x;
            float ey = event->data.arcMove.y;
            float ez = event->data.arcMove.z;
            float ea = event->data.arcMove.a;

            float fr = event->data.arcMove.feedrate;
            if (fr < 1.0f) fr = s->max_rate[AXIS_X];

            // Radius from start to center
            float dx_s = start.coordinate[AXIS_X] - cx;
            float dy_s = start.coordinate[AXIS_Y] - cy;
            float radius = sqrtf(dx_s * dx_s + dy_s * dy_s);
            if (radius < 0.001f) return false; // Degenerate arc

            // Angular sweep in the correct direction
            float theta_start = atan2f(dy_s, dx_s);
            float dx_e = ex - cx;
            float dy_e = ey - cy;
            float theta_end = atan2f(dy_e, dx_e);

            bool cw = event->data.arcMove.clockwise;
            float sweep;
            if (cw) {
                // G2 clockwise: angle decreases
                sweep = theta_start - theta_end;
                if (sweep <= 0.0f) sweep += 2.0f * 3.14159265358979f;
            } else {
                // G3 counter-clockwise: angle increases
                sweep = theta_end - theta_start;
                if (sweep <= 0.0f) sweep += 2.0f * 3.14159265358979f;
            }

            // Number of linear segments
            float arc_length = radius * sweep;
            float mm_per_seg = s->mm_per_arc_segment;
            if (mm_per_seg < 0.001f) mm_per_seg = 0.1f;
            uint32_t n_seg = (uint32_t)ceilf(arc_length / mm_per_seg);
            if (n_seg < 1) n_seg = 1;

            float theta_inc = cw ? -(sweep / (float)n_seg)
                                 :  (sweep / (float)n_seg);

            // Store arc state into APP_DATA — MOTION_Arc() will consume it
            appData->arcCenter.coordinate[AXIS_X]    = cx;
            appData->arcCenter.coordinate[AXIS_Y]    = cy;
            appData->arcCenter.coordinate[AXIS_Z]    = start.coordinate[AXIS_Z];
            appData->arcCenter.coordinate[AXIS_A]    = start.coordinate[AXIS_A];
            appData->arcCurrent                      = start;
            appData->arcStartPoint                   = start;
            appData->arcEndPoint.coordinate[AXIS_X]  = ex;
            appData->arcEndPoint.coordinate[AXIS_Y]  = ey;
            appData->arcEndPoint.coordinate[AXIS_Z]  = ez;
            appData->arcEndPoint.coordinate[AXIS_A]  = ea;
            appData->arcRadius                       = radius;
            appData->arcClockwise                    = cw;
            appData->arcFeedrate                     = fr;
            appData->arcTheta                        = theta_start;
            appData->arcThetaStart                   = theta_start;
            appData->arcThetaEnd                     = theta_end;
            appData->arcThetaIncrement               = theta_inc;
            appData->arcSegmentCurrent               = 0;
            appData->arcSegmentTotal                 = n_seg;
            appData->arcGenState                     = ARC_GEN_ACTIVE;
            return true;
        }

        case GCODE_EVENT_SPINDLE_ON:
        case GCODE_EVENT_SPINDLE_OFF:
        case GCODE_EVENT_COOLANT_ON:
        case GCODE_EVENT_COOLANT_OFF:
        case GCODE_EVENT_SET_FEEDRATE:
        case GCODE_EVENT_SET_SPINDLE_SPEED:
        case GCODE_EVENT_SET_ABSOLUTE:
        case GCODE_EVENT_SET_RELATIVE:
        case GCODE_EVENT_SET_WORK_OFFSET:
        case GCODE_EVENT_SET_WCS:
            // Modal-state changes handled in app.c before this point; nothing to do here
            return true;

        case GCODE_EVENT_DWELL:
            // TODO: implement dwell via CoreTimer delay; for now just drain queue
            return true;

        default:
            return false;
    }
}
