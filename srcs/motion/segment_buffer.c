// ============================================================================
// segment_buffer.c — GRBL exact port of st_prep_buffer()
// ============================================================================
// Faithful port of GRBL 1.1's st_prep_buffer() from stepper.c.
// Key algorithm: do-while inner loop extends DT_SEGMENT until ≥1 step is
// guaranteed. dt_remainder carries the sub-step fractional time forward so
// the next segment's step_interval absorbs it, preserving exact timing.
// This eliminates all drift, jumps, and sub-step forcing issues.

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "common.h"
#include "data_structures.h"
#include "settings.h"
#include "utils/uart_utils.h"
#include "../config/default/peripheral/tmr/plib_tmr4.h"

// Base time slice per segment iteration (1ms, same as GRBL).
// The do-while loop may extend this to 2ms, 3ms, etc. for slow speeds.
#define DT_SEGMENT              0.001f   // seconds per segment iteration

// Minimum mm-travel per segment = 1.25 × (1/step_per_mm).
// Guarantees at least 1 step per emitted segment (GRBL's REQ_MM_INCREMENT_SCALAR).
#define REQ_MM_INCREMENT_SCALAR 1.25f

// Ramp types (matches GRBL)
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2

// NOTE: prep_dt_remainder, prep_steps_remaining, prep_accelerate_until,
// prep_decelerate_after, and prep_maximum_speed are stored in APP_DATA
// (added to data_structures.h) so they reset correctly on soft reset.

// ============================================================================
// Segment Buffer Preparation
// ============================================================================
// Called from main loop to fill segment buffer from planner blocks.
// Continuously generates small constant-velocity segments.

void SEGMENT_PrepBuffer(APP_DATA* appData) {
    // ---- GRBL-exact: fill ALL available slots in one call (while loop) -----
    // GRBL's st_prep_buffer() also uses a while loop so the segment buffer is
    // always kept as full as possible.  Without this loop, each call fills only
    // one slot, the ISR drains it in ~3 ms and stalls while waiting for the
    // next call — producing the "slow then burst" artefact.
    while (true) {

    // Don't prep if buffer is full (ring buffer convention: full when next_head==tail).
    uint8_t next_head = (uint8_t)((appData->segmentBufferHead + 1u) % SEGMENT_BUFFER_SIZE);
    if (next_head == appData->segmentBufferTail) { return; }

    // ---- Load a new planner block if needed ---------------------------------
    if (appData->prep_pl_block == NULL) {
        if (appData->motionQueueCount == 0) { return; }

        appData->prep_pl_block = &appData->motionQueue[appData->motionQueueTail];

        if (appData->prep_pl_block->type == SEGMENT_TYPE_DWELL) {
            appData->prep_pl_block = NULL;
            return;
        }

        appData->prep_st_block_index = appData->motionQueueTail;
        appData->prep_mm_remaining   = appData->prep_pl_block->millimeters;

        // Float step counter (GRBL: prep.steps_remaining).
        // Ceil-difference accounting: n_steps = ceil(now) - ceil(after).
        appData->prep_steps_remaining = (float)appData->prep_pl_block->steps_remaining;
        appData->prep_dt_remainder    = 0.0f;
        appData->prep_mm_complete     = 0.0f;
        appData->prep_exit_speed      = appData->prep_pl_block->final_speed;
        appData->prep_mm_remaining    = appData->prep_pl_block->millimeters;  // ✅ FIX: must start at block total distance, not 0

        // Convert our trapezoid fields (distance-from-start) to GRBL mm_remaining notation
        // (counts DOWN from total_mm to 0). Stored once at block load so they remain
        // correct even after pl_block->millimeters is mutated each segment.
        float total_mm_bl = appData->prep_pl_block->millimeters;
        appData->prep_step_per_mm      = appData->prep_steps_remaining / total_mm_bl;
        appData->prep_req_mm_increment = REQ_MM_INCREMENT_SCALAR / appData->prep_step_per_mm;
        // prep_accelerate_until = mm_remaining when accel ramp ends
        appData->prep_accelerate_until = total_mm_bl - appData->prep_pl_block->accelerate_until;
        // prep_decelerate_after = mm_remaining when decel ramp starts
        appData->prep_decelerate_after = total_mm_bl - appData->prep_pl_block->decelerate_after;
        if (appData->prep_accelerate_until < 0.0f) { appData->prep_accelerate_until = 0.0f; }
        if (appData->prep_decelerate_after < 0.0f) { appData->prep_decelerate_after = 0.0f; }

        // Actual peak speed: for trapezoid = nominal_speed, for triangle = v_peak < nominal.
        // Compute from entry speed + accel distance (exact, handles both cases).
        float entry_speed = appData->prep_pl_block->initial_speed;
        float v_pk_sq = entry_speed * entry_speed
                        + 2.0f * appData->prep_pl_block->acceleration
                        * appData->prep_pl_block->accelerate_until;
        if (v_pk_sq < 0.0f) { v_pk_sq = 0.0f; }
        float v_peak = sqrtf(v_pk_sq);
        if (v_peak > appData->prep_pl_block->nominal_speed) {
            v_peak = appData->prep_pl_block->nominal_speed;
        }
        appData->prep_maximum_speed = v_peak;

        // Entry speed → initial ramp.
        if (entry_speed >= appData->prep_maximum_speed) {
            appData->prep_current_speed = appData->prep_maximum_speed;
            appData->prep_ramp_type     = RAMP_CRUISE;
        } else {
            appData->prep_current_speed = entry_speed;
            appData->prep_ramp_type     = RAMP_ACCEL;
        }

        // Copy Bresenham data to stepper block (unchanged).
        StepperBlock* st_block = &appData->stepperBlocks[appData->prep_st_block_index];
        for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
            st_block->steps[axis] = (uint32_t)abs(appData->prep_pl_block->delta[axis]);
        }
        st_block->step_event_count = appData->prep_pl_block->steps_remaining;
        uint8_t dir_bits = 0;
        for (E_AXIS axis = AXIS_X; axis < NUM_AXIS; axis++) {
            if (appData->prep_pl_block->delta[axis] < 0) { dir_bits |= (1u << axis); }
        }
        st_block->direction_bits = dir_bits;

        DEBUG_PRINT_MOTION("[SEGMENT] New block: %.2f mm, entry=%.1f mm/s, nominal=%.1f mm/s\r\n",
            appData->prep_pl_block->millimeters,
            appData->prep_current_speed,
            appData->prep_pl_block->nominal_speed);
    }

    // ---- References to current block fields ---------------------------------
    MotionSegment* pl_block  = appData->prep_pl_block;
    // Use the pre-computed prep fields (set at block load, not re-derived from
    // pl_block->millimeters which may already be mutated from a previous segment).
    float step_per_mm      = appData->prep_step_per_mm;
    float accel_until_mm   = appData->prep_accelerate_until;  // mm_rem at accel end
    float decel_after_mm   = appData->prep_decelerate_after;  // mm_rem at decel start
    float maximum_speed    = appData->prep_maximum_speed;     // actual peak speed

    // ---- GRBL inner do-while loop -------------------------------------------
    float dt_max     = DT_SEGMENT;
    float dt         = 0.0f;
    float time_var   = dt_max;
    float mm_remaining = appData->prep_mm_remaining;
    float minimum_mm   = mm_remaining - appData->prep_req_mm_increment;
    if (minimum_mm < 0.0f) { minimum_mm = 0.0f; }

    do {
        switch (appData->prep_ramp_type) {
            case RAMP_ACCEL: {
                float speed_var = pl_block->acceleration * time_var;
                mm_remaining   -= time_var * (appData->prep_current_speed + 0.5f * speed_var);
                if (mm_remaining < accel_until_mm) {
                    // Crossed end of acceleration ramp — snap to exact boundary.
                    mm_remaining = accel_until_mm;
                    time_var = 2.0f * (appData->prep_mm_remaining - mm_remaining)
                               / (appData->prep_current_speed + maximum_speed);
                    // Triangle (no cruise): apex == decel start → DECEL immediately
                    if (mm_remaining == decel_after_mm) { appData->prep_ramp_type = RAMP_DECEL; }
                    else                                { appData->prep_ramp_type = RAMP_CRUISE; }
                    appData->prep_current_speed = maximum_speed;
                } else {
                    appData->prep_current_speed += speed_var;
                }
                break;
            }
            case RAMP_CRUISE: {
                float mm_var = mm_remaining - maximum_speed * time_var;
                if (mm_var < decel_after_mm) {
                    // Crossed start of deceleration — snap to boundary.
                    time_var     = (mm_remaining - decel_after_mm) / maximum_speed;
                    mm_remaining = decel_after_mm;
                    appData->prep_ramp_type = RAMP_DECEL;
                } else {
                    mm_remaining = mm_var;
                }
                break;
            }
            default: /* RAMP_DECEL */ {
                float speed_var = pl_block->acceleration * time_var;
                if (appData->prep_current_speed > speed_var) {
                    float mm_var = mm_remaining
                                   - time_var * (appData->prep_current_speed - 0.5f * speed_var);
                    if (mm_var > appData->prep_mm_complete) {
                        mm_remaining = mm_var;
                        appData->prep_current_speed -= speed_var;
                        break;             // still decelerating
                    }
                }
                // At or below exit speed — snap to block end.
                time_var = 2.0f * (mm_remaining - appData->prep_mm_complete)
                           / (appData->prep_current_speed + appData->prep_exit_speed);
                mm_remaining = appData->prep_mm_complete;
                appData->prep_current_speed = appData->prep_exit_speed;
                break;
            }
        }

        dt += time_var;
        if (dt < dt_max) {
            // Ramp junction mid-slice: fill the remaining time in the new ramp.
            time_var = dt_max - dt;
        } else {
            if (mm_remaining > minimum_mm) {
                // Speed too slow for ≥1 step in dt_max — extend the time slice.
                // GRBL: dt_max += DT_SEGMENT until minimum_mm is satisfied.
                dt_max  += DT_SEGMENT;
                time_var = dt_max - dt;
            } else {
                break;   // Segment complete: ≥1 step guaranteed or block end reached.
            }
        }
    } while (mm_remaining > appData->prep_mm_complete);

    // ---- Compute step count (GRBL ceil-difference) --------------------------
    float step_dist_remaining    = step_per_mm * mm_remaining;
    float n_steps_remaining_f    = ceilf(step_dist_remaining);
    float last_n_steps_remaining = ceilf(appData->prep_steps_remaining);
    uint16_t n_steps = (uint16_t)(last_n_steps_remaining - n_steps_remaining_f);

    if (n_steps == 0u) {
        // Zero-step: not enough travel for a step yet — update state and
        // loop back immediately so the while loop can extend and retry.
        appData->prep_mm_remaining     = mm_remaining;
        appData->prep_steps_remaining  = n_steps_remaining_f;
        continue;
    }

    // ---- Compute step interval (GRBL: cycles_per_tick) ----------------------
    // Add fractional time credit from previous segment before dividing.
    float total_dt   = dt + appData->prep_dt_remainder;
    float inv_rate   = total_dt / (last_n_steps_remaining - step_dist_remaining);
    uint32_t step_interval = (uint32_t)ceilf((float)TMR4_FrequencyGet() * inv_rate);
    if (step_interval < 7u)      { step_interval = 7u; }
    if (step_interval > 0xFFFFu) { step_interval = 0xFFFFu; }

    // ---- Commit prep state -------------------------------------------------
    appData->prep_mm_remaining    = mm_remaining;
    appData->prep_steps_remaining = n_steps_remaining_f;
    // Save fractional step time for next segment
    appData->prep_dt_remainder    = (n_steps_remaining_f - step_dist_remaining) * inv_rate;

    // ---- Write segment to buffer -------------------------------------------
    StepSegment* segment        = &appData->segmentBuffer[appData->segmentBufferHead];
    segment->n_step             = n_steps;
    segment->step_interval      = step_interval;
    segment->st_block_index     = appData->prep_st_block_index;

    appData->segmentBufferHead = next_head;

    DEBUG_PRINT_MOTION("[SEGMENT] n=%u PR4=%lu dt=%.3fms remain=%.3fmm\r\n",
        n_steps, step_interval, dt * 1000.0f, mm_remaining);

    // ---- Advance motion queue when block is fully consumed -----------------
    if (mm_remaining <= appData->prep_mm_complete) {
        appData->motionQueueTail  = (appData->motionQueueTail + 1u) % MAX_MOTION_SEGMENTS;
        appData->motionQueueCount--;
        appData->prep_pl_block    = NULL;
        DEBUG_PRINT_MOTION("[SEGMENT] Block complete\r\n");
    }

    } // while(true) — loop back to fill next available slot
}

// ============================================================================
// SEGMENT_Initialize
// ============================================================================
void SEGMENT_Initialize(APP_DATA* appData)
{
    appData->segmentBufferHead   = 0u;
    appData->segmentBufferTail   = 0u;
    appData->segmentNextHead     = 1u;
    appData->prep_pl_block       = NULL;
    appData->prep_mm_remaining   = 0.0f;
    appData->prep_current_speed  = 0.0f;
    appData->prep_ramp_type      = RAMP_ACCEL;
    appData->prep_st_block_index = 0u;

    // GRBL prep state (now in APP_DATA, reset here)
    appData->prep_steps_remaining  = 0.0f;
    appData->prep_step_per_mm      = 0.0f;
    appData->prep_req_mm_increment = 0.0f;
    appData->prep_dt_remainder     = 0.0f;
    appData->prep_accelerate_until = 0.0f;
    appData->prep_decelerate_after = 0.0f;
    appData->prep_maximum_speed    = 0.0f;
    appData->prep_mm_complete      = 0.0f;
    appData->prep_exit_speed       = 0.0f;
}
