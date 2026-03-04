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
#include "utils/uart_utils.h"
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

// ─── Modal state ─────────────────────────────────────────────────────────────
// Remembers the last programmed feedrate across G-code lines (F-word is modal).
static float s_modal_feedrate_mm_min = 0.0f;

// ─── Planned position ────────────────────────────────────────────────────────
// Tracks the logical end position of the last move queued to the trajectory.
// This is used as the `start` for the next queued move rather than the live
// step-counter position, because UGS/senders typically pipeline many commands
// before the first one executes.  Updated each time TRAJECTORY_AddMove succeeds.
// Reset to the live step-counter position on soft-reset or queue drain.
static CoordinatePoint s_planned_position;
static bool            s_planned_position_valid = false;

// ─── STEPPER API bridges ──────────────────────────────────────────────────────

void STEPPER_Initialize(APP_DATA *appData)
{
    memset(&s_stepper_pos, 0, sizeof(s_stepper_pos));
    // Initialise steps_per_mm from settings (same as old stepper.c did)
    const CNC_Settings *s = SETTINGS_GetCurrent();
    for (int i = 0; i < NUM_AXIS; i++) {
        s_stepper_pos.steps_per_mm[i] = s->steps_per_mm[i];
    }
    // Tell the G-code flow-controller the real queue depth so water marks
    // (HIGH/LOW_WATER) are calculated against 64 slots, not the old 16.
    appData->gcodeCommandQueue.maxMotionSegments = TRAJ_QUEUE_SIZE;
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
    // Sync the queue depth visible to the G-code flow controller.
    // gcode_parser.c reads appData->motionQueueCount to decide when to
    // send deferred "ok" — it must reflect the trajectory queue, not the
    // old 16-slot segment buffer.
    appData->motionQueueCount = TRAJECTORY_QueueCount();

    // Feed-hold gating
    if (g_feed_hold_active) return;

    // If interpolator just finished a move, pop the next one and signal
    // app.c so it triggers GCODE_CheckDeferredOk() immediately.
    // Also handles the kickstart case: interpolator idle but queue has entries
    // (first move after a command, or after a feed hold clear).
    bool move_done = INTERPOLATOR_MoveComplete();
    bool needs_kickstart = !INTERPOLATOR_IsActive() && (TRAJECTORY_QueueCount() > 0u);

    if (move_done || needs_kickstart) {
        SCurveMove next_move;
        if (TRAJECTORY_GetNextMove(&next_move)) {
            INTERPOLATOR_LoadMove(&next_move);
        }
        // Fire even if queue is now empty — the count sync above already
        // reflects the new (lower) occupancy, so CheckDeferredOk will
        // see the freed slot and release the next "ok" to UGS.
        if (move_done) {
            appData->motionSegmentCompleted = true;
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

    // DEBUG: Log first and last segment
    if (appData->arcSegmentCurrent == 1) {
        UART_Printf("[ARC_SEG1] total=%lu from=(%.3f,%.3f) to=(%.3f,%.3f) theta=%.4f\r\n",
            (unsigned long)appData->arcSegmentTotal,
            appData->arcCurrent.coordinate[AXIS_X], appData->arcCurrent.coordinate[AXIS_Y],
            next.coordinate[AXIS_X], next.coordinate[AXIS_Y],
            appData->arcTheta);
    }

    // Attempt to add the segment to the trajectory queue
    if (TRAJECTORY_AddMove(appData->arcCurrent, next, appData->arcFeedrate, 0.0f, 0.0f)) {
        TRAJECTORY_Recalculate();

        appData->arcCurrent = next;

        if (is_last) {
            appData->arcGenState = ARC_GEN_IDLE;
            UART_Printf("[ARC_DONE] end=(%.3f,%.3f,%.3f)\r\n",
                next.coordinate[AXIS_X], next.coordinate[AXIS_Y], next.coordinate[AXIS_Z]);
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
            // Build start/end in MACHINE coordinates.
            // Use the planned (queued) end position as start so that back-to-back
            // commands chained by the sender don't accumulate position before the
            // motor has moved (step-counter hasn't updated yet between commands).
            // When the trajectory queue is empty the step-counter IS authoritative,
            // so re-anchor from it in that case (covers soft-reset / queue drain).
            // Use the planned (logical) end of the last queued move as start.
            // This is correct even when the interpolator is mid-execute and the
            // trajectory queue has already been drained to feed the interpolator.
            // Re-anchor from the step counter only when all motion has stopped.
            CoordinatePoint start;
            if (s_planned_position_valid &&
                (TRAJECTORY_QueueCount() > 0 || INTERPOLATOR_IsActive())) {
                start = s_planned_position;
            } else {
                start = KINEMATICS_GetCurrentPosition();
                s_planned_position_valid = false;  // will be set true on first AddMove
            }
            // Build end in work space first, defaulting to current work position
            CoordinatePoint start_work = KINEMATICS_MachineToWork(start);
            CoordinatePoint end_work   = start_work;

            float ex = event->data.linearMove.x;
            float ey = event->data.linearMove.y;
            float ez = event->data.linearMove.z;
            float ea = event->data.linearMove.a;
            if (!isnan(ex)) end_work.coordinate[AXIS_X] = ex;
            if (!isnan(ey)) end_work.coordinate[AXIS_Y] = ey;
            if (!isnan(ez)) end_work.coordinate[AXIS_Z] = ez;
            if (!isnan(ea)) end_work.coordinate[AXIS_A] = ea;

            // Convert work-space end to machine space
            CoordinatePoint end = KINEMATICS_WorkToMachine(end_work);

            // G0 rapid: use axis max_rate; G1 feed: use programmed F-word (modal)
            float fr = event->data.linearMove.feedrate;
            bool  rapid = event->data.linearMove.isRapid;
            // Save non-zero feedrate as modal state
            if (fr >= 1.0f) s_modal_feedrate_mm_min = fr;
            // Use modal feedrate if current event has none
            if (!rapid && fr < 1.0f && s_modal_feedrate_mm_min >= 1.0f)
                fr = s_modal_feedrate_mm_min;
            if (rapid || fr < 1.0f) {
                // Rapid: pick the most-restrictive axis max_rate for all moving axes
                fr = 1e30f;
                for (int i = 0; i < NUM_AXIS; i++) {
                    if (fabsf(end.coordinate[i] - start.coordinate[i]) > 1e-6f) {
                        if (s->max_rate[i] < fr) fr = s->max_rate[i];
                    }
                }
                if (fr > 1e29f) fr = s->max_rate[0];  // no axis moved — fallback
            }

            // Add to trajectory queue; Recalculate blends junction speeds.
            bool added = TRAJECTORY_AddMove(start, end, fr, 0.0f, 0.0f);
            if (added) {
                // Advance the planned position to this move's end so the next
                // queued command uses the correct absolute start coordinate.
                s_planned_position       = end;
                s_planned_position_valid = true;

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

            // Use planned position (same logic as LINEAR_MOVE) so arcs chained
            // after queued linear moves start from the correct position.
            CoordinatePoint start;
            bool used_planned = false;
            if (s_planned_position_valid &&
                (TRAJECTORY_QueueCount() > 0 || INTERPOLATOR_IsActive())) {
                start = s_planned_position;
                used_planned = true;
            } else {
                start = KINEMATICS_GetCurrentPosition();
                s_planned_position_valid = false;
            }

            // Center in absolute coordinates (I/J are always relative to start per GRBL)
            float cx = start.coordinate[AXIS_X] + event->data.arcMove.centerX;
            float cy = start.coordinate[AXIS_Y] + event->data.arcMove.centerY;

            // Absolute end point
            float ex = event->data.arcMove.x;
            float ey = event->data.arcMove.y;
            float ez = event->data.arcMove.z;
            float ea = event->data.arcMove.a;

            // DEBUG: Log arc setup
            UART_Printf("[ARC_SETUP] src=%s start=(%.3f,%.3f,%.3f) ev.xy=(%.3f,%.3f) ij=(%.3f,%.3f) ctr=(%.3f,%.3f) traj=%lu isr=%d\r\n",
                used_planned ? "plan" : "step",
                start.coordinate[AXIS_X], start.coordinate[AXIS_Y], start.coordinate[AXIS_Z],
                ex, ey,
                event->data.arcMove.centerX, event->data.arcMove.centerY,
                cx, cy,
                (unsigned long)TRAJECTORY_QueueCount(),
                (int)INTERPOLATOR_IsActive());

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
            // Z and A are optional in arc commands. NAN means "same as start"
            // (no helical motion). Replace NAN now so MOTION_Arc() never passes
            // NAN into TRAJECTORY_AddMove, which would corrupt dist/millimeters.
            appData->arcEndPoint.coordinate[AXIS_Z]  = isnan(ez) ? start.coordinate[AXIS_Z] : ez;
            appData->arcEndPoint.coordinate[AXIS_A]  = isnan(ea) ? start.coordinate[AXIS_A] : ea;
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

            // Advance planned position to arc end so the next queued command
            // chains from the correct position even before MOTION_Arc() runs.
            s_planned_position.coordinate[AXIS_X] = ex;
            s_planned_position.coordinate[AXIS_Y] = ey;
            s_planned_position.coordinate[AXIS_Z] = appData->arcEndPoint.coordinate[AXIS_Z];
            s_planned_position.coordinate[AXIS_A] = appData->arcEndPoint.coordinate[AXIS_A];
            s_planned_position_valid = true;

            return true;
        }

        case GCODE_EVENT_SET_FEEDRATE:
            // Save standalone F-word as modal feedrate
            if (event->data.setFeedrate.feedrate >= 1.0f)
                s_modal_feedrate_mm_min = event->data.setFeedrate.feedrate;
            return true;

        case GCODE_EVENT_SPINDLE_ON:
        case GCODE_EVENT_SPINDLE_OFF:
        case GCODE_EVENT_COOLANT_ON:
        case GCODE_EVENT_COOLANT_OFF:
        case GCODE_EVENT_SET_SPINDLE_SPEED:
        case GCODE_EVENT_SET_ABSOLUTE:
        case GCODE_EVENT_SET_RELATIVE:
        case GCODE_EVENT_SET_WORK_OFFSET:
        case GCODE_EVENT_SET_WCS:
        case GCODE_EVENT_SET_TOOL:
        case GCODE_EVENT_PROGRAM_END:
        case GCODE_EVENT_HOMING:
        case GCODE_EVENT_PROBE_TOWARD:
        case GCODE_EVENT_PROBE_AWAY:
            // Modal-state / system events handled elsewhere; consume silently
            return true;

        case GCODE_EVENT_DWELL:
            // TODO: implement dwell via CoreTimer delay; for now just drain queue
            return true;

        default:
            // Unknown event — consume it to prevent infinite replay
            return true;
    }
}

// ─── PLANNED POSITION SYNC ────────────────────────────────────────────────────

// Re-anchor the planned position to the live step-counter (machine position).
// Must be called after a soft reset, feed hold + queue drain, or any event that
// discards queued trajectory moves without executing them, so that the next
// queued command starts from the correct machine position.
void MOTION_SyncPlannedPosition(void)
{
    s_planned_position       = KINEMATICS_GetCurrentPosition();
    s_planned_position_valid = true;
}

// ─── HOMING BRIDGE ────────────────────────────────────────────────────────────

// Submit one constant-speed homing move to the trajectory queue.
// Homing uses a fixed cruise speed with no junction blending (entry/exit = 0
// so TRAJECTORY_Recalculate leaves the move's speed untouched).
// Kicks the interpolator if it is currently idle.
bool MOTION_HomingMove(APP_DATA *appData,
                       CoordinatePoint start,
                       CoordinatePoint end,
                       float feedrate_mm_min)
{
    if (appData == NULL) return false;
    if (TRAJECTORY_QueueCount() >= (uint32_t)TRAJ_QUEUE_SIZE) return false;

    if (!TRAJECTORY_AddMove(start, end, feedrate_mm_min, 0.0f, 0.0f)) {
        return false;  // Queue full
    }
    TRAJECTORY_Recalculate();

    // Kick interpolator if it went idle between homing segments
    if (!INTERPOLATOR_IsActive()) {
        SCurveMove mv;
        if (TRAJECTORY_GetNextMove(&mv)) {
            STEPPERS_Enable();
            INTERPOLATOR_LoadMove(&mv);
        }
    }
    return true;
}
