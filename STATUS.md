# STATUS — scurve_motion branch
**Date**: March 3, 2026  
**Branch**: `scurve_motion` (based on `lookahead`)  
**Goal**: Replace entire GRBL AVR motion engine with a professional 32-bit architecture:
- Fixed 100kHz servo tick (TMR4 period-match ISR, replaces OC1 variable-rate ISR)
- DDS (Direct Digital Synthesis) step generation — 32-bit accumulator per axis
- S-curve (jerk-limited) velocity profiles computed in main loop on FPU
- G-code protocol layer (gcode_parser.c, settings.c) kept completely intact

---

## Files Deleted This Session

Removed from the branch via `git rm`. Do NOT restore them.

| Deleted | Reason |
|---------|--------|
| `srcs/motion/stepper.c` | OC1 variable-period ISR — replaced by fixed 100kHz TMR4 ISR |
| `srcs/motion/segment_buffer.c` | GRBL 1ms-slice segment prep — entire approach replaced |
| `srcs/motion/motion.c` | GRBL 3-pass planner — replaced by new trajectory planner |
| `srcs/motion/kinematics.c` | Trapezoid profile + AVR junction math — replaced by S-curve solver |

## Files Kept (do not modify tonight)

| Kept | Why |
|------|-----|
| `srcs/motion/homing.c` | Logic is sound — will call new trajectory API once written |
| `srcs/motion/spindle.c` | Unrelated to motion architecture |
| `srcs/motion/motion_utils.c` | Limit/safety checking, unchanged |
| `srcs/motion/tmc5160.c` | SPI driver, unchanged |
| `srcs/gcode/gcode_parser.c` | KEPT ENTIRELY — do not touch |
| `srcs/settings/settings.c` | KEPT — jerk settings added later |
| `srcs/utils/utils.c` | GPIO abstraction — unchanged |
| `srcs/utils/uart_utils.c` | Unchanged |
| `srcs/app.c` | Shell kept — motion calls gutted in Step 4 below |

---

## TONIGHT'S TASKS — Do In Order

---

### STEP 1 — MCC Hardware Reconfiguration (MPLAB X)

Open the project in MPLAB X, open MCC (MPLAB Code Configurator).

#### 1a. Reconfigure TMR4
- Current config: prescaler **1:64**, PR4 varies per GRBL segment
- New config: prescaler **1:1**, PR4 = **499** → `50,000,000 / 500 = 100,000 Hz`
- Enable **TMR4 Period Match interrupt** — this triggers the new servo tick ISR
- Set interrupt priority **IPL6** (highest in the application)

#### 1b. Remove OC1
- Disable/remove Output Compare 1 module in MCC
- OC1 was the GRBL step-rate controller — completely replaced by the DDS ISR
- Step GPIOs are now driven directly by LATSET/LATCLR inside the ISR (same atomic writes as before)

#### 1c. Remove TMR5
- TMR5 was a one-shot used to clear step pin ~3µs after OC1 fired
- At 100kHz fixed tick: step pin is HIGH for exactly 1 tick = **10µs**
- 10µs satisfies DRV8825 (min 1µs) and TMC5160 (min 100ns) — no separate pulse timer needed

#### 1d. Regenerate
- Click Generate in MCC
- `plib_tmr4.c` updated with new frequency
- `plib_ocmp1.c` removed
- `plib_tmr5.c` removed
- **Verify**: open `plib_tmr4.c` and confirm `TMR4_FrequencyGet()` returns `100000`
- **IMPORTANT**: Check that UART3 TX buffer is still 1024 bytes after regeneration (MCC sometimes reverts it to 256 — the $$ command needs 1024)

---

### STEP 2 — Create New Header Files

#### 2a. Create `incs/motion/trajectory.h`

```c
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "data_structures.h"
#include <stdint.h>
#include <stdbool.h>

// Lookahead queue depth — 64 moves (was 16 in GRBL)
#define TRAJ_QUEUE_SIZE     64

// DDS fixed-point scale (use top half of 32-bit range for signed accumulator)
#define DDS_SCALE           0x80000000UL

// S-curve move — one per G-code linear segment.
// Stores the 7-phase jerk-limited profile as a math object.
// NOT a list of pre-computed step intervals (that was the old segment buffer approach).
typedef struct {
    float t[8];              // cumulative phase end times (s). t[0]=0 always, t[7]=total duration.
    float v[8];              // velocity at each phase boundary (mm/s)
    float a[8];              // acceleration at each phase boundary (mm/s^2)
    float j;                 // jerk magnitude (mm/s^3). 0 = constant-velocity stub.
    float a_max;             // peak acceleration actually reached (mm/s^2)
    float millimeters;       // total move distance (mm)
    float unit_vec[NUM_AXIS];// normalised direction per axis. Sign = direction.
    float nominal_speed;     // clamped cruise speed (mm/s)
    float acceleration;      // combined axis-limited acceleration (mm/s^2)
    float entry_speed_sqr;   // planner: entry speed squared (for junction blending)
    float max_entry_speed_sqr;
    bool  speed_locked;      // planner: skip this block in reverse pass
} SCurveMove;

// Public API
void  TRAJECTORY_Initialize(void);
void  TRAJECTORY_Reset(void);

// Add a move to the lookahead queue. Returns false if queue full.
bool  TRAJECTORY_AddMove(CoordinatePoint start, CoordinatePoint end,
                         float feedrate_mm_min, float entry_v, float exit_v);

// Junction blending pass — call after adding each move.
void  TRAJECTORY_Recalculate(void);

// Pop the next ready move for the interpolator. Returns false if queue empty.
bool  TRAJECTORY_GetNextMove(SCurveMove *out);

// S-curve evaluation (used by interpolator for per-tick velocity)
float TRAJECTORY_VelocityAt(const SCurveMove *move, float t);
float TRAJECTORY_PositionAt(const SCurveMove *move, float t);

#endif // TRAJECTORY_H
```

#### 2b. Create `incs/motion/interpolator.h`

```c
#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#include "data_structures.h"
#include "motion/trajectory.h"
#include <stdint.h>
#include <stdbool.h>

// Must match TMR4 PR4 configuration: 50MHz / 500 = 100,000 Hz
#define INTERPOLATOR_TICK_RATE_HZ   100000UL

void INTERPOLATOR_Initialize(APP_DATA *appData);
void INTERPOLATOR_LoadMove(const SCurveMove *move);
void INTERPOLATOR_Stop(void);
void INTERPOLATOR_EnableAllAxes(void);
void INTERPOLATOR_DisableAllAxes(void);
bool INTERPOLATOR_IsActive(void);
bool INTERPOLATOR_MoveComplete(void);   // poll from main loop; auto-clears on read

// Registered with TMR4_CallbackRegister — do NOT call directly
void INTERPOLATOR_Tick(uint32_t status, uintptr_t context);

#endif // INTERPOLATOR_H
```

---

### STEP 3 — Create `srcs/motion/interpolator.c`

The 100kHz fixed-rate servo tick. Pure integer in ISR — no float.

```c
#include "motion/interpolator.h"
#include "utils/utils.h"
#include "app.h"
#include "peripheral/tmr/plib_tmr4.h"
#include <math.h>
#include <string.h>

// ISR-visible state — all volatile
static volatile int32_t  dds_acc[NUM_AXIS];
static volatile int32_t  dds_inc[NUM_AXIS];      // fixed-point: fraction of DDS_SCALE per tick
static volatile int8_t   dds_dir[NUM_AXIS];      // +1 or -1
static volatile uint32_t ticks_remaining;
static volatile bool     interp_active  = false;
static volatile bool     move_complete  = false;

static APP_DATA *g_appData = NULL;

void INTERPOLATOR_Initialize(APP_DATA *appData) {
    g_appData = appData;
    memset((void*)dds_acc, 0, sizeof(dds_acc));
    memset((void*)dds_inc, 0, sizeof(dds_inc));
    for (int i = 0; i < NUM_AXIS; i++) dds_dir[i] = 1;
    ticks_remaining = 0;
    interp_active   = false;
    move_complete   = false;
    TMR4_CallbackRegister(INTERPOLATOR_Tick, (uintptr_t)NULL);
}

void INTERPOLATOR_LoadMove(const SCurveMove *move) {
    float total_time = move->t[7];
    if (total_time < 1e-9f) return;

    uint32_t total_ticks = (uint32_t)(total_time * (float)INTERPOLATOR_TICK_RATE_HZ + 0.5f);
    if (total_ticks == 0) return;

    // Stop ISR during setup
    interp_active = false;

    float v = move->v[1];   // mm/s — constant velocity stub (v[1] = first phase speed)

    for (int axis = 0; axis < NUM_AXIS; axis++) {
        float axis_steps_per_sec = v
                                   * fabsf(move->unit_vec[axis])
                                   * (*(g_axis_settings[axis].steps_per_mm));
        // DDS increment = (steps_per_sec / tick_rate) * DDS_SCALE
        float inc_f = (axis_steps_per_sec / (float)INTERPOLATOR_TICK_RATE_HZ)
                      * (float)DDS_SCALE;
        dds_inc[axis] = (int32_t)(inc_f + 0.5f);
        dds_acc[axis] = 0;
        dds_dir[axis] = (move->unit_vec[axis] >= 0.0f) ? 1 : -1;

        if (dds_dir[axis] > 0) { AXIS_DirSet(axis); }
        else                   { AXIS_DirClear(axis); }
    }

    ticks_remaining = total_ticks;
    move_complete   = false;
    interp_active   = true;

    if (!TMR4_IsStarted()) TMR4_Start();
}

// THE 100kHz ISR — budget 10µs = 2000 CPU cycles at 200MHz
// Actual: ~200 cycles worst case
void INTERPOLATOR_Tick(uint32_t status, uintptr_t context) {
    if (!interp_active) return;

    // 1. Clear step pins from last tick (pulse lasted 10µs — always long enough)
    for (int axis = 0; axis < NUM_AXIS; axis++) {
        AXIS_StepClear(axis);
    }

    // 2. DDS accumulate → emit step on overflow
    for (int axis = 0; axis < NUM_AXIS; axis++) {
        if (dds_inc[axis] == 0) continue;
        dds_acc[axis] += dds_inc[axis];
        if ((uint32_t)dds_acc[axis] >= (uint32_t)DDS_SCALE) {
            dds_acc[axis] -= (int32_t)DDS_SCALE;
            AXIS_StepSet(axis);
            if (dds_dir[axis] > 0) { AXIS_IncrementSteps(axis); }
            else                   { AXIS_DecrementSteps(axis); }
        }
    }

    // 3. Countdown
    if (ticks_remaining > 0) ticks_remaining--;
    if (ticks_remaining == 0) {
        interp_active = false;
        move_complete = true;   // ring main loop
        TMR4_Stop();
        for (int axis = 0; axis < NUM_AXIS; axis++) AXIS_StepClear(axis);
    }
}

bool INTERPOLATOR_MoveComplete(void) {
    if (move_complete) {
        move_complete = false;
        return true;
    }
    return false;
}

bool INTERPOLATOR_IsActive(void) { return interp_active; }

void INTERPOLATOR_Stop(void) {
    interp_active = false;
    ticks_remaining = 0;
    TMR4_Stop();
    for (int axis = 0; axis < NUM_AXIS; axis++) AXIS_StepClear(axis);
}

void INTERPOLATOR_EnableAllAxes(void)  { enable_all_set(); }
void INTERPOLATOR_DisableAllAxes(void) { enable_all_clear(); }
```

---

### STEP 4 — Create `srcs/motion/trajectory.c` (constant-velocity stub tonight)

Full S-curve math comes next session. Tonight's stub validates that DDS output is correct.

```c
#include "motion/trajectory.h"
#include "utils/utils.h"
#include <math.h>
#include <string.h>

static SCurveMove traj_queue[TRAJ_QUEUE_SIZE];
static uint32_t   traj_head  = 0;
static uint32_t   traj_tail  = 0;
static uint32_t   traj_count = 0;

void TRAJECTORY_Initialize(void) {
    traj_head = traj_tail = traj_count = 0;
    memset(traj_queue, 0, sizeof(traj_queue));
}

void TRAJECTORY_Reset(void) { TRAJECTORY_Initialize(); }

bool TRAJECTORY_GetNextMove(SCurveMove *out) {
    if (traj_count == 0) return false;
    *out = traj_queue[traj_tail];
    traj_tail = (traj_tail + 1) % TRAJ_QUEUE_SIZE;
    traj_count--;
    return true;
}

// STUB: constant velocity, no acceleration ramp.
// Purpose: validate DDS step output on hardware before adding profile math.
bool TRAJECTORY_AddMove(CoordinatePoint start, CoordinatePoint end,
                        float feedrate_mm_min, float entry_v, float exit_v) {
    if (traj_count >= TRAJ_QUEUE_SIZE) return false;

    SCurveMove *m = &traj_queue[traj_head];
    memset(m, 0, sizeof(SCurveMove));

    // Geometry
    float dist_sq = 0.0f;
    for (int i = 0; i < NUM_AXIS; i++) {
        float d = end.coordinate[i] - start.coordinate[i];
        m->unit_vec[i] = d;
        dist_sq += d * d;
    }
    float dist = sqrtf(dist_sq);
    if (dist < 1e-6f) return false;
    m->millimeters = dist;
    for (int i = 0; i < NUM_AXIS; i++) m->unit_vec[i] /= dist;

    // Clamp feedrate to per-axis limits
    float v_mm_s = feedrate_mm_min / 60.0f;
    for (int i = 0; i < NUM_AXIS; i++) {
        float uv = fabsf(m->unit_vec[i]);
        if (uv > 1e-6f) {
            float limit = *(g_axis_settings[i].max_rate) / 60.0f;
            if (v_mm_s * uv > limit) v_mm_s = limit / uv;
        }
    }

    float duration = dist / v_mm_s;

    // Constant velocity: fill all phases with same speed
    for (int ph = 0; ph < 8; ph++) {
        m->t[ph] = (ph == 0) ? 0.0f : duration;
        m->v[ph] = v_mm_s;
        m->a[ph] = 0.0f;
    }
    m->j             = 0.0f;
    m->a_max         = 0.0f;
    m->nominal_speed = v_mm_s;
    m->acceleration  = *(g_axis_settings[AXIS_X].acceleration);

    traj_head = (traj_head + 1) % TRAJ_QUEUE_SIZE;
    traj_count++;
    return true;
}

// No junction blending in stub
void  TRAJECTORY_Recalculate(void) { }

float TRAJECTORY_VelocityAt(const SCurveMove *m, float t) { return m->v[1]; }
float TRAJECTORY_PositionAt(const SCurveMove *m, float t) { return m->v[1] * t; }
```

---

### STEP 5 — Create new `srcs/motion/kinematics.c`

Rewrite from scratch — WCS coordinate math ONLY. No velocity, no trapezoid, no planner state.

Keep ONLY these functions from the old file:
- `KINEMATICS_Initialize()` — remove `g_timer_freq`, remove `pl_previous_unit_vec[]`, remove `pl_previous_nominal_speed`
- `KINEMATICS_WorkToMachine()` / `KINEMATICS_MachineToWork()`
- `KINEMATICS_WorkToMachineWithWCS()` / `KINEMATICS_MachineToWorkWithWCS()`
- `KINEMATICS_SetWorkOffset()` / `KINEMATICS_GetActiveWCSOffset()`
- `KINEMATICS_SetWorkCoordinates()` / `KINEMATICS_GetCurrentPosition()`
- `KINEMATICS_SetAxisMachinePosition()`
- `KINEMATICS_GetWorkCoordinates()`

**Delete entirely:**
- `g_timer_freq`, `pl_previous_unit_vec[]`, `pl_previous_nominal_speed` (statics)
- `limit_acceleration_by_axis()` → moves to `trajectory.c`
- `limit_rate_by_axis()` → moves to `trajectory.c`
- `convert_to_unit_vector()` → moves to `trajectory.c`
- `KINEMATICS_LinearMove()` → replaced by `TRAJECTORY_AddMove()`
- `KINEMATICS_RecalculateTrapezoid()` → deleted
- `KINEMATICS_ArcMove()` → arc generation handled in trajectory layer
- `KINEMATICS_PlanArc()` → same
- `KINEMATICS_ResetPlannerState()` → not needed
- `KINEMATICS_CalculateJunctionSpeed()` → moves to `trajectory.c`
- `KINEMATICS_LinearMoveSimple()` → replaced by `TRAJECTORY_AddMove()`
- `KINEMATICS_HomingMove()` → replaced by `TRAJECTORY_AddMove()`
- `KINEMATICS_ResetAccumulators()` → stub/delete
- `static float step_accumulator[NUM_AXIS]` → deleted

---

### STEP 6 — Update `app.c`

Replace all deleted function calls. Search for each of these and fix:

| Search for | Replace with |
|------------|--------------|
| `STEPPER_LoadSegment` | `INTERPOLATOR_LoadMove` |
| `STEPPER_DisableAll()` | `INTERPOLATOR_DisableAllAxes()` |
| `STEPPER_EnableAll()` | `INTERPOLATOR_EnableAllAxes()` |
| `STEPPER_IsEnabled()` | `INTERPOLATOR_IsActive()` |
| `MOTION_Tasks(` | remove call |
| `SEGMENT_PrepBuffer(` | remove call |
| `KINEMATICS_LinearMove(` | `TRAJECTORY_AddMove(` |
| `KINEMATICS_HomingMove(` | `TRAJECTORY_AddMove(` |
| `motionActive` flag | `INTERPOLATOR_IsActive()` |
| `motionSegmentCompleted` flag | `INTERPOLATOR_MoveComplete()` |

New `APP_IDLE` loop structure:
```c
case APP_IDLE:
    // G-code parsing and event dispatch
    GCODE_Tasks(&appData, &appData.gcodeCommandQueue);

    // Feed moves to interpolator when it becomes free
    if (!INTERPOLATOR_IsActive()) {
        SCurveMove next;
        if (TRAJECTORY_GetNextMove(&next)) {
            INTERPOLATOR_LoadMove(&next);
        }
    }

    // Check deferred OK (flow control unchanged)
    if (INTERPOLATOR_MoveComplete()) {
        GCODE_CheckDeferredOk(&appData, &appData.gcodeCommandQueue);
    }
    break;
```

Add includes at top of `app.c`:
```c
#include "motion/interpolator.h"
#include "motion/trajectory.h"
```

In `APP_Initialize()`:
```c
INTERPOLATOR_Initialize(&appData);
TRAJECTORY_Initialize();
```

---

### STEP 7 — Update `data_structures.h`

Remove these fields from `APP_DATA`:

```c
// DELETE all of these:
MotionSegment motionQueue[MAX_MOTION_SEGMENTS];
uint32_t      motionQueueHead, motionQueueTail, motionQueueCount;
StepperBlock  stepperBlocks[MAX_MOTION_SEGMENTS];
StepSegment   segmentBuffer[SEGMENT_BUFFER_SIZE];
volatile uint8_t segmentBufferTail;
uint8_t       segmentBufferHead, segmentNextHead;
uint8_t       prep_st_block_index;
float         prep_mm_remaining, prep_current_speed;
uint8_t       prep_ramp_type;
MotionSegment *prep_pl_block;
float         prep_steps_remaining, prep_step_per_mm;
float         prep_req_mm_increment, prep_dt_remainder;
float         prep_accelerate_until, prep_decelerate_after;
float         prep_maximum_speed, prep_mm_complete, prep_exit_speed;
bool          motionActive;
bool          motionSegmentCompleted;
E_AXIS        dominantAxis;
MotionSegment *currentSegment;
int32_t       bresenham_error[NUM_AXIS];
```

Also delete the `StepSegment` and `StepperBlock` struct definitions from the file — they are no longer used.

The trajectory queue lives inside `trajectory.c` as a static (no need to put it in `APP_DATA`).

---

### STEP 8 — Build and Test

```powershell
# From repo root
make clean
make 2>&1 | Select-String "error:" | Select-Object -First 30
```

Fix all errors. Then:

```powershell
# Full build
make
```

Flash `bins/CNC_V3.hex`. Connect UGS at 115200. Send:

```gcode
G21 G90
G1 X10 F1000
```

**Expected behaviour tonight:**
- Motor moves 10mm at constant 1000mm/min — NO ramp (stub)
- Step pin on oscilloscope: uniform pulse stream, constant frequency (~3,333 Hz for 200 steps/mm at 1000mm/min)
- UGS: motion completes, "ok" returned, `?` shows updated position

If this works — DDS architecture is validated. S-curve profile math goes in next session.

---

## Next Session (After Hardware Validation)

1. Replace constant-velocity stub in `TRAJECTORY_AddMove` with real 7-phase S-curve solver
2. Update `INTERPOLATOR_LoadMove` / `INTERPOLATOR_Tick` to re-evaluate `v(t)` per tick from current phase
3. Add jerk settings `$110`–`$113` to `settings.c` (mm/s³ per axis)
4. Add lookahead junction blending in `TRAJECTORY_Recalculate()`
5. Restore arc support via `TRAJECTORY_AddMove` chord generation
6. Restore homing using `TRAJECTORY_AddMove`

---

## Architecture Reference

```
[UGS / G-code sender via UART]
           ↓
[gcode_parser.c]          KEPT UNCHANGED — do not touch
           ↓  GCODE_Event queue
[app.c + trajectory.c]    S-curve profile math, lookahead queue (64 moves)
           ↓  SCurveMove
[interpolator.c]          100kHz fixed ISR, DDS 32-bit accumulator per axis
           ↓  LATSET / LATCLR
[DRV8825 / TMC5160 drivers → stepper motors]
```

**Key numbers:**
- TMR4: 50MHz / 500 = **100,000 Hz fixed**
- ISR budget: 10µs = **2000 CPU cycles** at 200MHz
- ISR actual cost: ~200 cycles worst case → **10% of budget used**
- Max step rate: 50,000 steps/sec per axis (DDS Nyquist at 100kHz tick)
- Lookahead queue: 64 moves (was 16 in GRBL)
- All acceleration math: main loop FPU, zero float in ISR
