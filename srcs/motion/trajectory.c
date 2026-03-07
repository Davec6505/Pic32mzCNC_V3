/*
 * trajectory.c — S-curve trajectory planner
 *
 * Implements a jerk-limited 7-phase velocity profile solver plus a 64-entry
 * lookahead queue with GRBL-style reverse+forward junction blending.
 *
 * All code in this file runs in main-loop context with full FPU access.
 * Nothing here touches ISR-visible state directly.
 *
 * S-curve math summary
 * ─────────────────────
 * Seven motion phases, each with a CONSTANT jerk jk[i].
 * Within phase i, at local time dt = (t_now - t[i]):
 *
 *   v(dt) = v[i] + a[i]*dt + 0.5*jk[i]*dt^2
 *   a(dt) = a[i] + jk[i]*dt
 *   s(dt) = v[i]*dt + 0.5*a[i]*dt^2 + (1/6)*jk[i]*dt^3
 *
 * Phase sequence (ph 0..6):
 *   0: jk=+J, a: 0→a_peak_accel          (ramp acceleration up)
 *   1: jk=0,  a=a_peak_accel             (hold acceleration if needed)
 *   2: jk=-J, a: a_peak_accel→0          (ramp acceleration down → v_nom)
 *   3: jk=0,  a=0, v=v_nom              (cruise, may be zero length)
 *   4: jk=-J, a: 0→-a_peak_decel        (ramp deceleration up)
 *   5: jk=0,  a=-a_peak_decel           (hold deceleration)
 *   6: jk=+J, a: -a_peak_decel→0        (ramp deceleration down → v_exit)
 *
 * Jerk magnitude per move:
 *   From CNC_Settings.jerk[axis] (units: see settings.h comment).
 *   The planner converts to mm/s³ via:
 *     J = a_max * jerk_setting / 2
 *   This gives J=1,250,000 mm/s³ for XY at default settings (smooth but fast).
 *
 * Degenerate cases:
 *   • If the move is too short to reach v_nom, v_peak is reduced (binary search).
 *   • If v_peak * 2 < a_max²/J, phases 1 and/or 5 collapse to zero duration.
 *   • Zero-duration phases are stored as equal t[i]==t[i+1]; the ISR skips them.
 */

#include "motion/trajectory.h"
#include "settings/settings.h"
#include "utils/utils.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

// ─── Module state ─────────────────────────────────────────────────────────────

static SCurveMove traj_queue[TRAJ_QUEUE_SIZE];
static uint32_t   traj_head  = 0;   // next write position
static uint32_t   traj_tail  = 0;   // next read position
static uint32_t   traj_count = 0;   // number of moves in queue

// Previous move direction and speed (for junction blending)
static float prev_unit_vec[NUM_AXIS] = {0};
static float prev_nominal_speed = 0.0f;

// ─── Internal helpers ─────────────────────────────────────────────────────────

// Clamp a float to [lo, hi].
static inline float fclampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Normalise a 4-element vector in-place.  Returns magnitude.
static float __attribute__((unused)) normalise_vec(float *vec)
{
    double mag_sq = 0.0;   // double accumulator for precision
    for (int i = 0; i < NUM_AXIS; i++) {
        mag_sq += (double)vec[i] * (double)vec[i];
    }
    float mag = sqrtf((float)mag_sq);
    if (mag > 1e-9f) {
        float inv_mag = 1.0f / mag;
        for (int i = 0; i < NUM_AXIS; i++) vec[i] *= inv_mag;
    }
    return mag;
}

// Compute the worst-case (most restrictive) combined acceleration limit given a
// unit direction vector.  Mirrors GRBL's plan_compute_profile_parameters logic.
static float limit_acceleration(const float *unit_vec)
{
    const CNC_Settings *s = SETTINGS_GetCurrent();
    float a_lim = 1e30f;
    for (int i = 0; i < NUM_AXIS; i++) {
        float uv = fabsf(unit_vec[i]);
        if (uv > 1e-6f) {
            float axis_a = s->acceleration[i] / uv;
            if (axis_a < a_lim) a_lim = axis_a;
        }
    }
    return a_lim;
}

// Compute the worst-case combined feedrate limit.
static float limit_rate(const float *unit_vec)
{
    const CNC_Settings *s = SETTINGS_GetCurrent();
    float r_lim = 1e30f;
    for (int i = 0; i < NUM_AXIS; i++) {
        float uv = fabsf(unit_vec[i]);
        if (uv > 1e-6f) {
            float axis_r = (s->max_rate[i] / 60.0f) / uv;  // convert mm/min→mm/s
            if (axis_r < r_lim) r_lim = axis_r;
        }
    }
    return r_lim;
}

// Compute the worst-case jerk magnitude J [mm/s³] for a given unit vector.
// Formula: J = a_max_axis * jerk_setting_axis / 2  (see file header)
// Then take the minimum across contributing axes (most conservative).
static float limit_jerk(const float *unit_vec, float a_combined)
{
    const CNC_Settings *s = SETTINGS_GetCurrent();
    float j_best = 1e30f;
    for (int i = 0; i < NUM_AXIS; i++) {
        float uv = fabsf(unit_vec[i]);
        if (uv > 1e-6f) {
            // Per-axis jerk setting in mm/s^2 units relative to per-axis acceleration
            float j_axis = (s->acceleration[i] * s->jerk[i]) * 0.5f;
            // Scale by direction component — same weighting as for acceleration
            float j_scaled = j_axis / uv;
            if (j_scaled < j_best) j_best = j_scaled;
        }
    }
    // Ensure J is positive and finite
    if (j_best < 1.0f || j_best > 1e12f) {
        // Fallback: J that gives 50 ms jerk-phase at a_combined
        j_best = a_combined / 0.05f;
    }
    return j_best;
}

// ─── S-curve block-distance helper ───────────────────────────────────────────
//
// Returns the distance [mm] needed to go from v_start to v_end using
// acceleration a_max and jerk J.
// Works for both acceleration (v_end > v_start) and deceleration (v_end < v_start).
// dv is always treated as positive; the direction is handled by the caller.

static float scurve_block_dist(float v_start, float v_end, float a_max, float J)
{
    float dv = fabsf(v_end - v_start);
    if (dv < 1e-9f) return 0.0f;

    // Velocity change achievable with a pure double-jerk ramp (no constant accel):
    //   dv_jerk = a_max² / J  (velocity gain from one pair of jerk phases)
    float dv_jerk = (a_max * a_max) / J;

    float tj, ta, v_lo, d;

    if (dv >= dv_jerk) {
        // Full 3-phase ramp: jerk-up, constant accel, jerk-down
        tj   = a_max / J;
        ta   = (dv - dv_jerk) / a_max;
        v_lo = fminf(v_start, v_end);          // lower velocity
        // v at end of first jerk phase:
        float v1  = v_lo + 0.5f * a_max * a_max / J;
        float v2  = v1 + a_max * ta;
        // s1 = v_lo*tj + (1/6)*J*tj³
        float s1 = v_lo * tj + (J * tj * tj * tj) * (1.0f / 6.0f);
        // s2 = v1*ta + 0.5*a_max*ta²
        float s2 = v1 * ta + 0.5f * a_max * ta * ta;
        // s3 = v2*tj + 0.5*a_max*tj² - (1/6)*J*tj³
        float s3 = v2 * tj + 0.5f * a_max * tj * tj
                   - (J * tj * tj * tj) * (1.0f / 6.0f);
        d = s1 + s2 + s3;
    } else {
        // 2-phase ramp: only jerk-up + jerk-down (peak accel < a_max)
        // a_peak = sqrt(J * dv),  tj = a_peak / J = sqrt(dv / J)
        float tj2 = sqrtf(dv / J);
        // Average velocity × total time  (analytical result for symmetric 2-pjase)
        d = (v_start + v_end) * tj2;
    }
    return d;
}

// ─── S-curve profile fill ─────────────────────────────────────────────────────
//
// Fills move->t[], v[], a[], jk[] for a move from v_entry to v_exit through
// peak speed v_peak (≤ v_nom).
// Called only when the geometry has already been validated.

static void scurve_fill_phases(SCurveMove *m,
                                float v_entry, float v_peak, float v_exit,
                                float a_max, float J, float dist)
{
    // ── Acceleration block (v_entry → v_peak) ─────────────────────────────────
    float dv_a   = v_peak - v_entry;
    float dv_jerk_a = (a_max * a_max) / J;
    float tj_a, ta_a, a_pk_a;

    if (dv_a <= 1e-9f) {
        // No acceleration needed
        tj_a = 0.0f; ta_a = 0.0f; a_pk_a = 0.0f;
    } else if (dv_a >= dv_jerk_a) {
        tj_a  = a_max / J;
        ta_a  = (dv_a - dv_jerk_a) / a_max;
        a_pk_a = a_max;
    } else {
        // Reduced peak — 2-phase only
        a_pk_a = sqrtf(J * dv_a);
        tj_a   = a_pk_a / J;
        ta_a   = 0.0f;
    }

    // ── Deceleration block (v_peak → v_exit) ──────────────────────────────────
    float dv_d   = v_peak - v_exit;
    float dv_jerk_d = (a_max * a_max) / J;
    float tj_d, ta_d, a_pk_d;

    if (dv_d <= 1e-9f) {
        tj_d = 0.0f; ta_d = 0.0f; a_pk_d = 0.0f;
    } else if (dv_d >= dv_jerk_d) {
        tj_d  = a_max / J;
        ta_d  = (dv_d - dv_jerk_d) / a_max;
        a_pk_d = a_max;
    } else {
        a_pk_d = sqrtf(J * dv_d);
        tj_d   = a_pk_d / J;
        ta_d   = 0.0f;
    }

    // ── Cruise ────────────────────────────────────────────────────────────────
    float d_accel = scurve_block_dist(v_entry, v_peak, a_max, J);
    float d_decel = scurve_block_dist(v_peak,  v_exit, a_max, J);
    float d_cruise = dist - d_accel - d_decel;
    if (d_cruise < 0.0f) d_cruise = 0.0f;
    float tc = (v_peak > 1e-9f) ? (d_cruise / v_peak) : 0.0f;

    // ── Fill phase arrays ─────────────────────────────────────────────────────
    // Phase 0: jerk-up (accel side)
    float t = 0.0f;
    m->t[0]  = t;
    m->v[0]  = v_entry;
    m->a[0]  = 0.0f;
    m->jk[0] = +J;
    t += tj_a;

    // Phase 1: constant accel
    float v1 = v_entry + 0.5f * a_pk_a * a_pk_a / J;  // v after jerk-up
    if (dv_a <= 1e-9f) v1 = v_entry;
    m->t[1]  = t;
    m->v[1]  = v1;
    m->a[1]  = a_pk_a;
    m->jk[1] = 0.0f;
    t += ta_a;

    // Phase 2: jerk-down (accel side, a_pk_a → 0)
    float v2 = v1 + a_pk_a * ta_a;
    m->t[2]  = t;
    m->v[2]  = v2;
    m->a[2]  = a_pk_a;
    m->jk[2] = -J;
    t += tj_a;

    // Phase 3: cruise
    m->t[3]  = t;
    m->v[3]  = v_peak;
    m->a[3]  = 0.0f;
    m->jk[3] = 0.0f;
    t += tc;

    // Phase 4: jerk-down (decel side, 0 → -a_pk_d)
    m->t[4]  = t;
    m->v[4]  = v_peak;
    m->a[4]  = 0.0f;
    m->jk[4] = -J;
    t += tj_d;

    // Phase 5: constant decel
    float v5 = v_peak - 0.5f * a_pk_d * a_pk_d / J;
    if (dv_d <= 1e-9f) v5 = v_peak;
    m->t[5]  = t;
    m->v[5]  = v5;
    m->a[5]  = -a_pk_d;
    m->jk[5] = 0.0f;
    t += ta_d;

    // Phase 6: jerk-up back to 0 accel (decel side)
    float v6 = v5 - a_pk_d * ta_d;
    m->t[6]  = t;
    m->v[6]  = v6;
    m->a[6]  = -a_pk_d;
    m->jk[6] = +J;
    t += tj_d;

    // t[7] = total duration
    m->t[7]  = t;
    m->jk[7] = 0.0f;  // unused

    m->v_entry = v_entry;
    m->v_exit  = v_exit;
}

// ─── S-curve solver (main entry point) ───────────────────────────────────────
//
// Given move geometry and physics limits, solves the 7-phase profile and fills
// the SCurveMove struct.
//
// Strategy:
//   1. Try v_nom as peak speed.
//   2. If d_accel + d_decel > dist, binary-search for a reduced v_peak.
//   3. Fill all 7 phase arrays.

static void scurve_solve(SCurveMove *m, float v_entry, float v_exit,
                          float v_nom, float a_max, float J)
{
    float dist = m->millimeters;

    // Clamp entries to  v_nom
    v_entry = fclampf(v_entry, 0.0f, v_nom);
    v_exit  = fclampf(v_exit,  0.0f, v_nom);

    // Distance needed for direct entry→exit transition (no cruise at all)
    float d_need = scurve_block_dist(v_entry, v_nom, a_max, J)
                 + scurve_block_dist(v_nom,   v_exit, a_max, J);

    float v_peak;

    if (d_need <= dist) {
        // Full profile fits: use v_nom as cruise speed
        v_peak = v_nom;
    } else {
        // Move too short: binary-search for v_peak in [max(v_entry,v_exit), v_nom]
        float v_lo = fmaxf(v_entry, v_exit);
        float v_hi = v_nom;

        for (int iter = 0; iter < 32; iter++) {
            v_peak = 0.5f * (v_lo + v_hi);
            float d = scurve_block_dist(v_entry, v_peak, a_max, J)
                    + scurve_block_dist(v_peak,  v_exit,  a_max, J);
            if (d > dist) {
                v_hi = v_peak;
            } else {
                v_lo = v_peak;
            }
            if ((v_hi - v_lo) < 0.001f) break;
        }
        v_peak = v_lo;
    }

    m->nominal_speed = v_nom;
    m->J             = J;
    m->acceleration  = a_max;

    scurve_fill_phases(m, v_entry, v_peak, v_exit, a_max, J, dist);
}

// ─── Public API implementation ────────────────────────────────────────────────

void TRAJECTORY_Initialize(void)
{
    traj_head  = 0;
    traj_tail  = 0;
    traj_count = 0;
    memset(traj_queue, 0, sizeof(traj_queue));
    memset(prev_unit_vec, 0, sizeof(prev_unit_vec));
    prev_nominal_speed = 0.0f;
}

void TRAJECTORY_Reset(void)
{
    TRAJECTORY_Initialize();
}

bool TRAJECTORY_AddMove(CoordinatePoint start,
                         CoordinatePoint end,
                         float           feedrate_mm_min,
                         float           entry_v,
                         float           exit_v)
{
    if (traj_count >= TRAJ_QUEUE_SIZE) return false;

    SCurveMove *m = &traj_queue[traj_head];
    memset(m, 0, sizeof(SCurveMove));

    // ── Geometry: direction vector and distance ───────────────────────────────
    float delta[NUM_AXIS];
    float dist_sq = 0.0f;
    for (int i = 0; i < NUM_AXIS; i++) {
        delta[i] = end.coordinate[i] - start.coordinate[i];
        dist_sq += delta[i] * delta[i];
    }
    float dist = sqrtf(dist_sq);
    if (dist < 1e-6f) return false;   // zero-length move — discard

    m->millimeters = dist;
    for (int i = 0; i < NUM_AXIS; i++) {
        m->unit_vec[i] = delta[i] / dist;
    }

    // ── Physics limits ────────────────────────────────────────────────────────
    float a_max = limit_acceleration(m->unit_vec);
    float v_max_rate = limit_rate(m->unit_vec);
    float J     = limit_jerk(m->unit_vec, a_max);

    // Clamp feedrate
    float v_nom = fclampf(feedrate_mm_min / 60.0f, 0.0f, v_max_rate);
    if (v_nom < 1e-6f) return false;   // zero feedrate — discard

    // ── Junction blending (max entry speed from centripetal approximation) ────
    // GRBL formula: max_junction_speed² = J_accel * jd * sin(θ/2) / (1-sin(θ/2))
    // where jd = junction_deviation setting.
    const CNC_Settings *s = SETTINGS_GetCurrent();
    float jd = s->junction_deviation;

    float cos_theta = 0.0f;
    for (int i = 0; i < NUM_AXIS; i++) {
        cos_theta -= prev_unit_vec[i] * m->unit_vec[i];  // -dot product
    }
    cos_theta = fclampf(cos_theta, -1.0f, 1.0f);

    float max_junction_speed_sqr;
    if (cos_theta > (1.0f - 1e-4f)) {
        // 180-degree turn — full stop
        max_junction_speed_sqr = 0.0f;
    } else {
        float sin_theta_d2 = sqrtf(0.5f * (1.0f - cos_theta));
        if (sin_theta_d2 < 1e-6f) {
            max_junction_speed_sqr = (v_nom * v_nom);  // straight line
        } else {
            float j_accel = a_max;   // use combined accel as junction acceleration
            max_junction_speed_sqr = (j_accel * jd * sin_theta_d2)
                                     / (1.0f - sin_theta_d2);
        }
    }
    // Also limit by previous block's nominal speed (can't enter faster than the previous move was going)
    float prev_nom_sq = prev_nominal_speed * prev_nominal_speed;
    max_junction_speed_sqr = fminf(max_junction_speed_sqr, prev_nom_sq);
    max_junction_speed_sqr = fminf(max_junction_speed_sqr, v_nom * v_nom);

    m->max_junction_speed_sqr = max_junction_speed_sqr;
    m->max_entry_speed_sqr    = max_junction_speed_sqr;
    m->entry_speed_sqr        = 0.0f;    // will be set by Recalculate
    m->speed_locked           = false;
    m->nominal_speed          = v_nom;
    m->acceleration           = a_max;
    m->J                      = J;

    // Initial profile — conservative, entry=0, exit=0.  Recalculate will fix.
    scurve_solve(m, 0.0f, 0.0f, v_nom, a_max, J);

    // Update planner previous-block state
    prev_nominal_speed = v_nom;
    for (int i = 0; i < NUM_AXIS; i++) prev_unit_vec[i] = m->unit_vec[i];

    traj_head = (traj_head + 1u) % TRAJ_QUEUE_SIZE;
    traj_count++;
    return true;
}

// ─── Lookahead junction blending ──────────────────────────────────────────────
//
// GRBL-exact three-pass algorithm adapted for SCurveMove entries:
//   Pass 1 — Reverse  (newest → oldest): propagate maximum deceleration backward
//   Pass 2 — Forward  (oldest → newest): propagate maximum acceleration forward
//   Pass 3 — Re-solve: call scurve_solve on every modified block with the
//                      settled entry/exit speeds.
//
// The entry_speed_sqr field stores v² at the START of each block.
// The exit speed of block[i] = entry speed of block[i+1].

void TRAJECTORY_Recalculate(void)
{
    if (traj_count < 2u) return;   // nothing to blend

    // Helper: index from tail
    // tail is oldest, (tail + traj_count - 1) % SIZE is newest

    uint32_t n = traj_count;

    // ── Pass 1: Reverse (newest → oldest) ────────────────────────────────────
    // Propagate max achievable entry speed backward through the queue.
    {
        // Newest block must stop at exit (entry of block after last = 0)
        uint32_t idx = (traj_tail + n - 1u) % TRAJ_QUEUE_SIZE;
        SCurveMove *blk = &traj_queue[idx];
        // Exit speed of newest block is 0 (last move — decelerate to stop)
        float next_entry_sqr = 0.0f;

        for (uint32_t i = n; i-- > 0u; ) {
            idx = (traj_tail + i) % TRAJ_QUEUE_SIZE;
            blk = &traj_queue[idx];
            if (blk->speed_locked) {
                next_entry_sqr = blk->entry_speed_sqr;
                continue;
            }
            // Max entry speed reachable given exit speed and distance
            // v_entry² = v_exit² + 2*a*d
            float max_via_next = next_entry_sqr
                               + 2.0f * blk->acceleration * blk->millimeters;
            blk->entry_speed_sqr = fminf(blk->max_entry_speed_sqr, max_via_next);
            next_entry_sqr       = blk->entry_speed_sqr;
        }
    }

    // ── Pass 2: Forward (oldest → newest) ────────────────────────────────────
    // Propagate maximum achievable acceleration forward.
    {
        float prev_entry_sqr = 0.0f;  // entry of queue start = 0 (machine at rest)
        float prev_accel     = 0.0f;
        float prev_dist      = 0.0f;

        for (uint32_t i = 0; i < n; i++) {
            uint32_t idx = (traj_tail + i) % TRAJ_QUEUE_SIZE;
            SCurveMove *blk = &traj_queue[idx];
            if (blk->speed_locked) {
                prev_entry_sqr = blk->entry_speed_sqr;
                prev_accel     = blk->acceleration;
                prev_dist      = blk->millimeters;
                continue;
            }
            // Max entry speed reachable given previous block's entry speed and accel
            float max_via_prev = prev_entry_sqr
                               + 2.0f * prev_accel * prev_dist;
            if (max_via_prev < blk->entry_speed_sqr) {
                blk->entry_speed_sqr = max_via_prev;
            }
            prev_entry_sqr = blk->entry_speed_sqr;
            prev_accel     = blk->acceleration;
            prev_dist      = blk->millimeters;
        }
    }

    // ── Pass 3: Re-solve every block with settled junction speeds ─────────────
    for (uint32_t i = 0; i < n; i++) {
        uint32_t idx     = (traj_tail + i) % TRAJ_QUEUE_SIZE;
        uint32_t idx_next= (traj_tail + i + 1u) % TRAJ_QUEUE_SIZE;
        SCurveMove *blk  = &traj_queue[idx];
        if (blk->speed_locked) continue;

        float v_entry = sqrtf(blk->entry_speed_sqr);

        float exit_speed_sqr;
        if (i + 1u < n) {
            exit_speed_sqr = traj_queue[idx_next].entry_speed_sqr;
        } else {
            exit_speed_sqr = 0.0f;   // last block decelerates to stop
        }
        float v_exit = sqrtf(exit_speed_sqr);

        scurve_solve(blk, v_entry, v_exit,
                     blk->nominal_speed, blk->acceleration, blk->J);
    }
}

// ─── Queue access ─────────────────────────────────────────────────────────────

bool TRAJECTORY_GetNextMove(SCurveMove *out)
{
    if (traj_count == 0u) return false;
    *out = traj_queue[traj_tail];
    traj_tail  = (traj_tail + 1u) % TRAJ_QUEUE_SIZE;
    traj_count--;
    return true;
}

uint32_t TRAJECTORY_QueueCount(void)
{
    return traj_count;
}

void TRAJECTORY_LockLastMove(void)
{
    if (traj_count == 0u) return;
    uint32_t newest = (traj_head == 0u) ? (TRAJ_QUEUE_SIZE - 1u) : (traj_head - 1u);
    SCurveMove *m = &traj_queue[newest];
    // Raise the junction entry limit to allow full-speed arc-to-arc transitions.
    // At a=5000 mm/s² a typical arc chord (~1.8mm) provides >1mm of braking room,
    // so the reverse pass will still arrive at v_nom for interior segments, giving
    // constant-speed arcs.  Crucially we do NOT set speed_locked or force
    // entry_speed_sqr — the planner reverse/forward passes remain active and can
    // correctly compute the deceleration ramp at the arc-start (if approaching from
    // rest) and arc-end junction with following linear/arc moves.
    m->max_entry_speed_sqr    = m->nominal_speed * m->nominal_speed;
    m->max_junction_speed_sqr = m->nominal_speed * m->nominal_speed;
}

// ─── Jog support ─────────────────────────────────────────────────────────────

void TRAJECTORY_TagLastAsJog(void)
{
    if (traj_count == 0u) return;
    uint32_t newest = (traj_head == 0u) ? (TRAJ_QUEUE_SIZE - 1u) : (traj_head - 1u);
    traj_queue[newest].is_jog     = true;
    traj_queue[newest].speed_locked = true;  // prevent planner from blending across jog boundary
}

bool TRAJECTORY_HasJogMoves(void)
{
    for (uint32_t i = 0u; i < traj_count; i++) {
        if (traj_queue[(traj_tail + i) % TRAJ_QUEUE_SIZE].is_jog) return true;
    }
    return false;
}

void TRAJECTORY_CancelJog(void)
{
    // Remove jog-flagged moves from the HEAD (newest) backward toward tail,
    // stopping at the first non-jog move so G-code segments are preserved.
    while (traj_count > 0u) {
        uint32_t newest = (traj_head == 0u) ? (TRAJ_QUEUE_SIZE - 1u) : (traj_head - 1u);
        if (!traj_queue[newest].is_jog) break;
        traj_head = newest;
        traj_count--;
    }
}

// ─── Profile evaluation ───────────────────────────────────────────────────────
//
// Used by the interpolator each tick to get the current velocity.
// t is the elapsed time since move start [seconds].

float TRAJECTORY_VelocityAt(const SCurveMove *m, float t)
{
    // Clamp to valid range
    if (t <= 0.0f) return m->v[0];
    if (t >= m->t[7]) return m->v[6] + m->a[6] * (m->t[7] - m->t[6]);

    // Find phase (linear scan — 7 phases, negligible overhead)
    int ph = 0;
    for (int p = 6; p >= 0; p--) {
        if (t >= m->t[p]) { ph = p; break; }
    }
    float dt = t - m->t[ph];
    return m->v[ph] + m->a[ph] * dt + 0.5f * m->jk[ph] * dt * dt;
}

float TRAJECTORY_PositionAt(const SCurveMove *m, float t)
{
    if (t <= 0.0f) return 0.0f;

    float s = 0.0f;
    for (int ph = 0; ph < 7; ph++) {
        float t_start = m->t[ph];
        float t_end   = m->t[ph + 1];
        if (t <= t_start) break;

        float dt = fminf(t, t_end) - t_start;
        s += m->v[ph] * dt
           + 0.5f * m->a[ph] * dt * dt
           + (1.0f / 6.0f) * m->jk[ph] * dt * dt * dt;

        if (t <= t_end) break;
    }
    return fclampf(s, 0.0f, m->millimeters);
}
