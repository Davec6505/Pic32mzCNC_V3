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

// MOTION_Arc — incremental arc segment generation.
// In the old engine this pushed one segment per call.  In the new engine, arc
// geometry is converted to many TRAJECTORY_AddMove calls upfront by the G-code
// event handler.  This stub handles the residual calls from app.c.
void MOTION_Arc(APP_DATA *appData)
{
    (void)appData;
    // No-op: arcs are generated entirely by MOTION_ProcessGcodeEvent
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
