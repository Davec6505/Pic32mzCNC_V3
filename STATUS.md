# Pic32mzCNC_V3 - Development Status Tracker
**PINOUT**: `Enable pin needs to be controlled by setting direction bit.

**Branch**: `scurve_motion`
**Last Updated**: March 8, 2026

---

## ✅ BUG FIX: Arc-to-arc streaming stall — ok released at point of consumption (March 8, 2026)

**Symptom**: Machine stopped mid-file at `MPos:30.031,50.013,0.000` (always same spot) during arc→arc transitions. First arc ran fine; second arc never started. UGS showed `<Idle>` prematurely.

**Root cause — architectural**: The deferred-ok system was polling `commands_consumed` delta at the **top** of `GCODE_Tasks()` (every main loop iteration). During arc generation, a single gcode queue slot (the arc command) spawns hundreds of trajectory segments with no further queue removals — `commands_consumed` is frozen for the entire arc duration. The periodic poll always saw `freed = 0` → no oks released → UGS send window exhausted → gcode queue drained → machine idled. This was an arc-to-arc issue specifically because arc 2 sits at the queue head blocked (arc 2 is not consumed until arc 1 finishes), so the queue never dropped below HIGH_WATER either.

**Root cause — structural**: Deferred-ok release was not tied to the event that causes it (command consumption). It was inferred indirectly via a counter diff, guaranteed to lag by at least one iteration and broken entirely when `commands_consumed` stopped advancing.

**Fix**: Moved `GCODE_CheckDeferredOk()` to fire immediately after each `GCODE_ConsumeEvent()` in `srcs/app.c`. At the exact moment a gcode slot is freed, `commands_consumed` has just incremented → `freed = 1` → ok released immediately. The ok is now causally tied to its command slot, not to a polling interval.

Also kept the free-space fallback in `GCODE_CheckDeferredOk()` as belt-and-braces: if `freed == 0` AND `q->count < HIGH_WATER`, release `HIGH_WATER - q->count` oks. This handles any remaining edge cases.

**Validated**: Full run of `tests/07_complex_long_run_fast.gcode` completed to `MPos:0.000,0.000,0.000` — all sections including dense arc sequences, arc→arc transitions, G4 dwells, and high-speed linear sweeps.

**Files changed**:
- `srcs/app.c:~451` — Added `GCODE_CheckDeferredOk()` call immediately after `GCODE_ConsumeEvent()` in the successful-process branch
- `srcs/app.c:~465` — Added `GCODE_CheckDeferredOk()` call immediately after `GCODE_ConsumeEvent()` in the PROGRAM_END branch
- `srcs/gcode/gcode_parser.c` — `GCODE_CheckDeferredOk()`: added free-space fallback (`if freed==0 && q->count < HIGH_WATER`)

---

## ✅ FEATURE: GRBL-compliant $H ok timing — deferred until homing completes (March 8, 2026)

**Problem**: `$H` (homing) sent its `ok` response immediately when the command arrived — before the homing cycle ran. GRBL v1.1 spec requires the ok only after homing succeeds, or `ALARM:9` on failure. Pre-existing bug, unaffected by but discovered during the arc streaming audit.

**Fix**:
- `$H` QUERY_CHARS handler: `send_ok = false` + `s_homing_pending = true` + `okPendingCount++` (defer the ok into the standard pending pool)
- `GCODE_CheckDeferredOk()`: new homing gate inserted after the dwell gate. While `HOMING_IsActive()` all ok releases are suppressed. When homing finishes:
  - **Success** (`HOMING_STATE_IDLE`): clear sentinel, fall through to release exactly 1 ok
  - **Failure** (`HOMING_STATE_ALARM`): discard the ok slot, print `ALARM:9\r\n`, return — no ok sent

**Note on `HOMING_IsActive()`**: returns false for IDLE, ALARM, and COMPLETE. COMPLETE is a purely transient state resolved within the same `HOMING_Tasks()` call frame; it is never visible to `GCODE_CheckDeferredOk` after `HOMING_Tasks()` returns.

**Files changed**:
- `srcs/gcode/gcode_parser.c` — Added `static bool s_homing_pending = false;`
- `srcs/gcode/gcode_parser.c:~1253` — `$H` handler: suppress immediate ok, arm sentinel
- `srcs/gcode/gcode_parser.c:~GCODE_CheckDeferredOk` — Added homing gate block

---

## ✅ BUG FIX: LOCATE phase freezes — event-driven backoff redesign (March 8, 2026)

**Symptom**: After hardware `$H` test, Z would drive to the limit switch (SEEK working), then freeze in `<Run>` state indefinitely. Backoff never started; pulloff never started.

**Root cause**: `HOMING_StartLocate()` pre-queued both the backoff move AND the slow re-approach simultaneously using fixed distances at the moment the limit switch fired. The re-approach `MOTION_HomingMove()` call used the pre-stop machine position as its start — and the step counter had not yet settled — so `TRAJECTORY_AddMove()` returned `false` (zero-distance guard: `if (dist < 1e-6f) return false`). That caused `g_homing.motion_active = false`. On the very next iteration, `HOMING_Tasks()` saw `!motion_active && motionQueueCount == 0` and immediately fired Alarm 9, halting everything.

**Fix**: Complete redesign of the LOCATE phase to event-driven backoff:

1. **SEEK rising edge** → Stop interpolator, reset trajectory, call `HOMING_StartLocate()` which now only queues a long open-ended backoff (10× `$27` pull-off distance). Sets `locate_backing_off = true`.
2. **LOCATE falling edge** (switch opens during backoff) → Stop the backoff, call `HOMING_StartLocateReapproach()` which queues: (a) a fixed `$27` extra clearance move, then (b) a slow re-approach (`3× pull_off`). Sets `locate_reapproaching = true`.
3. **LOCATE rising edge with `locate_reapproaching`** → That is the precision home point → zero machine position, go to PULLOFF.
4. **`HOMING_Tasks()` alarm guard** → Only alarms if the re-approach runs to completion without hitting the switch, or if the backoff runs out without the switch opening.

**Files changed**:
- `incs/motion/homing.h` — `HomingControl`: added `locate_reapproaching` bool field; added `HOMING_StartLocateReapproach()` declaration
- `srcs/motion/homing.c` — `HOMING_StartLocate()`: rewritten to only start long backoff (no pre-queued re-approach); added new `HOMING_StartLocateReapproach()` function; `HOMING_Tasks()` LOCATE alarm guard updated
- `srcs/app.c` — rising-edge handler: `LOCATE && locate_reapproaching` triggers PULLOFF; `LOCATE && locate_backing_off` ignored (switch still settling); falling-edge handler: stops backoff and calls `HOMING_StartLocateReapproach()`

---

## ✅ BUG FIX: Homing stuck at limit switch — INTERPOLATOR_Stop() + TRAJECTORY_Reset() (March 7, 2026)

**Symptom**: `$H` would drive Z (or the first axis) to the limit switch, then freeze. No backoff, no slow locate, no pulloff — motor sat on the switch indefinitely.

**Root cause**: In `srcs/app.c`, the rising-edge ISR callback (triggered when a limit switch fired during SEEK) was calling `TMR4_Stop()` and clearing the old segment buffer fields (`appData.motionQueueHead/Tail/Count = 0`, `appData.currentSegment = NULL`). The old fields belong to the pre-S-curve architecture and have no effect on the S-curve engine. Meanwhile, `interp_active` inside `interpolator.c` was left `true`. When `HOMING_StartLocate()` called `MOTION_HomingMove()`, that function calls `TRAJECTORY_AddMove()` (queues the backoff) then checks `if (!INTERPOLATOR_IsActive())` before kicking the DDS — it sees `true` (stale), so the interpolator is never restarted and the move sits in the queue forever.

**Fix**:
- `srcs/app.c` (SEEK→LOCATE rising-edge handler, LOCATE→PULLOFF rising-edge handler): Replaced
  ```c
  TMR4_Stop();
  appData.motionQueueHead  = 0;
  appData.motionQueueTail  = 0;
  appData.motionQueueCount = 0;
  appData.currentSegment   = NULL;
  ```
  with:
  ```c
  INTERPOLATOR_Stop();    // clears interp_active AND stops TMR4
  TRAJECTORY_Reset();     // flushes the S-curve trajectory queue
  ```
- `srcs/app.c` (includes): Added `#include "motion/interpolator.h"` and `#include "motion/trajectory.h"` (were missing — would have caused `-Werror` implicit-declaration build failure)

**Files changed**:
- `srcs/app.c:22-24` — Added `#include "motion/interpolator.h"` and `#include "motion/trajectory.h"`
- `srcs/app.c:~569-570` — SEEK→LOCATE block: `INTERPOLATOR_Stop(); TRAJECTORY_Reset();`
- `srcs/app.c:~580-581` — LOCATE→PULLOFF block: `INTERPOLATOR_Stop(); TRAJECTORY_Reset();`

---

## ✅ BUG FIX: $3 direction invert ignored by interpolator (March 7, 2026)

**Symptom**: Setting `$3=4` (invert Z direction) had no visible effect — Z still moved in the same direction.

**Root cause**: `srcs/motion/stepper.c` is excluded from the build (`EXCLUDED_FILES` in `srcs/Makefile`). The S-curve replacement `interpolator.c` set direction GPIO pins via raw `AXIS_DirSet/DirClear` in `INTERPOLATOR_LoadMove()` without consulting `SETTINGS_GetCurrent()->step_direction_invert`. The `STEPPER_ReloadSettings()` stub in `motion_bridge.c` only updated `steps_per_mm`, not the direction mask.

**Fix**:
- `srcs/motion/interpolator.c`: Added `#include "motion/motion_utils.h"` and `#include "settings/settings.h"`. In `INTERPOLATOR_LoadMove()`, replaced bare `AXIS_DirSet/DirClear` with `MOTION_UTILS_SetDirection(axis, forward, dir_inv)` where `dir_inv = SETTINGS_GetCurrent()->step_direction_invert`. `dds_dir[]` (position accounting) is unchanged — only the GPIO pin is inverted.

**Files changed**:
- `srcs/motion/interpolator.c:~10-11` — Added `#include "motion/motion_utils.h"` and `#include "settings/settings.h"`
- `srcs/motion/interpolator.c` — `INTERPOLATOR_LoadMove()` direction block: uses `MOTION_UTILS_SetDirection()` with settings mask

---

## ✅ BUG FIX: $H (homing) returned ok immediately, no motion (March 7, 2026)

**Symptom**: Sending `$H` got an `ok` response immediately; machine stayed at 0,0,0 in `<Idle>` state; no motor motion at all.

**Root cause**: In `srcs/motion/motion_bridge.c`, `GCODE_EVENT_HOMING` was lumped into the fallthrough case with `GCODE_EVENT_SPINDLE_ON`, `GCODE_EVENT_SPINDLE_OFF`, `GCODE_EVENT_COOLANT_ON`, `GCODE_EVENT_COOLANT_OFF`, and `GCODE_EVENT_PROGRAM_END`. That block just drained the trajectory queue and returned `true`. `HOMING_Start()` was never called, `g_homing.state` stayed `HOMING_STATE_IDLE`, and `HOMING_Tasks()` did nothing.

**Fix**:
- `srcs/motion/motion_bridge.c`: Added `#include "motion/homing.h"`. Removed `GCODE_EVENT_HOMING` from the shared fallthrough. Added dedicated `case GCODE_EVENT_HOMING:` that waits for the trajectory to drain, then calls `HOMING_Start(appData, event->data.homing.axes_mask)`. Sends `error:8` if homing is disabled or already running.

**Files changed**:
- `srcs/motion/motion_bridge.c:~10` — Added `#include "motion/homing.h"`
- `srcs/motion/motion_bridge.c` — `GCODE_EVENT_HOMING` given its own case calling `HOMING_Start()`

---

## ✅ IMPLEMENTED: Software Jog ($J= + 0x85 cancel) — Phase 5a

**Files**:
- `incs/gcode/gcode_parser.h` — Added `GCODE_EVENT_JOG` to `GCODE_EventType` enum; added `bool is_jog` field to `linearMove` union struct
- `incs/motion/trajectory.h` — Added `bool is_jog` field to `SCurveMove`; added `TRAJECTORY_TagLastAsJog()`, `TRAJECTORY_HasJogMoves()`, `TRAJECTORY_CancelJog()` to public API
- `incs/motion/motion.h` — Added `MOTION_JogCancel()` and `MOTION_IsJogging()` declarations
- `srcs/motion/trajectory.c` — Implemented `TRAJECTORY_TagLastAsJog()`, `TRAJECTORY_HasJogMoves()`, `TRAJECTORY_CancelJog()`
- `srcs/gcode/gcode_parser.c` — Added `0x85` to `is_control_char()`; `case 0x85:` in CONTROL_CHAR handler calls `MOTION_JogCancel()`; `?` status now reports `<Jog|...>` when `MOTION_IsJogging()`; `$J=` handler in QUERY_CHARS parses and submits jog directly to `MOTION_ProcessGcodeEvent()` (bypasses command queue)
- `srcs/motion/motion_bridge.c` — Added `s_jog_executing` flag; `GCODE_EVENT_JOG` case in `MOTION_ProcessGcodeEvent()`; `MOTION_JogCancel()` flushes jog trajectory entries and stops interpolator if executing jog; `MOTION_IsJogging()` returns true while jog queued or executing; `MOTION_Tasks()` tracks `s_jog_executing` per-move

**Architecture**:
- `$J=G91X1F500` → parsed in QUERY_CHARS → `GCODE_EVENT_JOG` submitted directly to motion bridge → `ok` returned on acceptance
- Jog moves tagged `is_jog=true` + `speed_locked=true` in trajectory queue — planner does not blend across jog boundaries (entry_v=0, exit_v=0)
- Consecutive jog commands chain via `s_planned_position` for smooth multi-step jog
- `0x85` cancels only jog-tagged moves from queue (newest→oldest), leaves G-code moves intact; stops interpolator if jog was executing; no alarm raised
- Status `?` reports `<Jog|MPos:...>` during jog execution (GRBL v1.1 compliant)

**Supported `$J=` modals**: G90/G91 (absolute/relative), G20/G21 (units); at least one axis word and F required; any other word → `error:4`

**Error codes**:
- `error:2` — machine in alarm state (jog rejected)
- `error:4` — bad/missing parameters (no F, no axis, unknown modal)
- `error:8` — trajectory queue full at submission time

**Test sequence (UGS)**:
```
$J=G91X1F500      → jog 1 mm +X, ok
$J=G91X-1F500     → jog 1 mm -X, ok
$J=G90X0Y0F1000   → jog to work origin, ok
0x85              → cancel in-progress jog (no response)
```

**Build**: zero warnings/errors — `bins/CNC_V3.hex`

---

## ✅ IMPLEMENTED: G81/G83 Canned Drilling Cycles — Phase 4

**Files**:
- `incs/gcode/gcode_parser.h` — Added `GCODE_EVENT_CANNED_DRILL`, `GCODE_EVENT_CANNED_PECK`, `GCODE_EVENT_CANNED_CANCEL` to `GCODE_EventType` enum; added `cannedDrill` struct to `GCODE_Event` union (`x y z r q feedrate l g98`)
- `srcs/gcode/gcode_parser.c` — Added `static bool s_canned_g98` modal flag; G98/G99 parsing (immediate modal, no event); G80 → `GCODE_EVENT_CANNED_CANCEL`; G81 → `GCODE_EVENT_CANNED_DRILL`; G83 → `GCODE_EVENT_CANNED_PECK`; G33 stub → `error:2` (blocked, no spindle encoder); all parse `X Y Z R Q F L` with unit scaling
- `srcs/motion/motion_bridge.c` — Added `CannedPhase` enum, 12 static state variables, `cc_add_move_work()` helper, `cc_rapid_fr()` helper; `GCODE_EVENT_CANNED_CANCEL` immediate consume; `GCODE_EVENT_CANNED_DRILL/PECK` 6-phase state machine (RAPID_XY → RAPID_R → FEED_TO → [PECK_LIFT → PECK_PLUNGE →] RETRACT); canned cycle state reset in `STEPPER_Initialize()` on soft reset

**Architecture** — dwell pattern (not arc pattern):
- Event stays in queue (`return false`) while cycle executes across many iterations
- Each phase: drain check → issue one move → advance phase → return false
- No `APP_DATA` or `app.c` changes required

**G81 phase sequence per hole**:
`RAPID_XY → RAPID_R → FEED_TO(Z) → RETRACT(G98/G99)` × L holes

**G83 phase sequence per hole**:
`RAPID_XY → RAPID_R → [FEED_TO(peck) → PECK_LIFT → PECK_PLUNGE] × pecks → FEED_TO(Z) → RETRACT` × L holes

**Behaviour**:
- `G98` / `G99` — modal, sets retract mode (initial Z / R-plane); persists across lines
- `G80` — cancels armed cycle immediately (no drain)
- `G81 X Y Z R F` — simple drill; L-word optional (default 1)
- `G83 X Y Z R Q F` — peck drill; Q = peck increment (positive mm); 0.5mm rapid-plunge clearance above previous depth
- `G90` — X/Y are absolute hole positions; Z/R absolute
- `G91` — X/Y are per-hole increments applied from current position; Z/R relative to current Z; L>1 drills L holes spaced by X/Y increment
- `G33` — replies `error:2` (rigid tapping deferred to Phase 5, needs spindle encoder)
- Soft reset (`Ctrl+X`) cancels any in-progress canned cycle via `STEPPER_Initialize()`
- Build: zero warnings/errors — `bins/CNC_V3.hex`

---

## 🔲 PLANNED: Manual Jog Control — Phase 5

### Phase 5a — Software Jog (`$J=` + `0x85` cancel) ← IMPLEMENT NEXT

**Goal**: Enable UGS jog buttons, jog-wheel in UGS, and any PC-side pendant without hardware changes.

**Scope**:
- Parse `$J=G91 X1 F500` / `$J=G90 X10 Y10 F1000` in the `$`-command handler
- Valid modals inside `$J=`: G90/G91 (position mode), G20/G21 (units), axis words + F only — anything else → `error:4`
- Emit `GCODE_EVENT_JOG` (new event type) which reuses `linearMove` union struct but carries an `is_jog` flag
- Motion bridge: jog move bypasses look-ahead planner (no junction planning across jog boundaries), goes direct to `TRAJECTORY_AddMove()`; `is_jog` flag stored on segment so the queue drains cleanly without waiting for preceding G-code moves
- Real-time cancel byte `0x85` — flush jog-only segments from the trajectory queue, leave any non-jog segments intact, return to IDLE without raising ALARM (distinct from feed hold `!` and soft reset `Ctrl+X`)
- Status report: reports `<Jog|MPos:...>` while jog segment executing (GRBL v1.1 compliant)

**Architecture notes**:
- `0x85` must be caught in the real-time character handler before line buffering, same priority as `?`, `!`, `~`, `0x18`
- Motion queue flush: iterate queue tail→head, discard segments with `is_jog == true`, stop at first non-jog segment; call `STEPPER_StopMotion()` if queue becomes empty
- `$J=` is NOT added to the G-code command queue — parsed and emitted directly, no `ok` response (GRBL v1.1 behaviour: jog commands DO get `ok` on acceptance, cancel gets no response)
- Unit scaling: respect current modal units (G20/G21) and distance mode (G90/G91)

**Files to modify**:
- `incs/gcode/gcode_parser.h` — add `GCODE_EVENT_JOG`; add `bool is_jog` to `linearMove` struct (or reuse with flag)
- `srcs/gcode/gcode_parser.c` — `$J=` branch in query handler; `0x85` in real-time char handler
- `srcs/motion/motion_bridge.c` — `GCODE_EVENT_JOG` case; `0x85` flush logic
- `incs/data_structures.h` / `MotionSegment` — add `bool is_jog` field to segment

**Test sequence (UGS)**:
```
$J=G91X1F500      → jog 1mm +X, ok
$J=G91X-1F500     → jog 1mm -X, ok
$J=G90X0Y0F1000   → jog to origin, ok
0x85              → cancel in-progress jog (no response)
```

---

### ~~Phase 5b — Hardware Pendant (PS2 Serial)~~ — **DROPPED**

> Jogging is handled entirely by UGS on the PC side via `$J=` commands over UART. No firmware pendant support required. If hardware pendant is needed in the future it can use the existing `$J=` + `0x85` infrastructure via any UART-connected device.

---

## ✅ IMPLEMENTED: G54–G59 Multiple Work Coordinate Systems — Phase 3

**Files**:
- `incs/gcode/gcode_parser.h` — Added `uint8_t wcs_number` field to `workOffset` union struct (0–5 = G54–G59; sentinel 255 = use current active WCS, i.e. P0)
- `incs/motion/kinematics.h` — Declared `KINEMATICS_SetActiveWCS(uint8_t)` and `KINEMATICS_GetActiveWCS(void)`
- `srcs/motion/kinematics.c` — Added `static uint8_t s_active_wcs`; `KINEMATICS_Initialize()` now resets it to 0; new `KINEMATICS_SetActiveWCS()` reads the target slot from NVM via `SETTINGS_GetWorkCoordinateSystem()` and reloads `g_wcs.offset`; new `KINEMATICS_GetActiveWCS()` returns current slot
- `srcs/gcode/gcode_parser.c` — `G10` block completely rewritten: previously only handled L20 P0/P1 inline; now emits `GCODE_EVENT_SET_WORK_OFFSET` for both L2 and L20, all P0–P6 values, with NaN defaults for unspecified axes
- `srcs/motion/motion_bridge.c` — Split the old mega-case block (which incorrectly ran probe setup code for ALL modal events including G54–G59/G10) into three separate handlers:
  - `GCODE_EVENT_SET_WCS` — drains motion, updates `appData->activeWCS`, calls `KINEMATICS_SetActiveWCS()`, invalidates planned position
  - `GCODE_EVENT_SET_WORK_OFFSET` — drains motion, resolves P0 sentinel → current active WCS, applies L2 (direct offset) or L20 (offset = MachinePos − DesiredWorkPos) math, saves to NVM via `SETTINGS_SetWorkCoordinateSystem()`, conditionally reloads kinematics cache if the active WCS was modified
  - Remaining modal events (SPINDLE/COOLANT/SET_ABS/REL/TOOL/PROGRAM_END/HOMING) — drain + return true, no probe setup code
  - Probe (PROBE_TOWARD/PROBE_AWAY) — unchanged probe setup code

**Behaviour**:
- `G54`–`G59` — switches active WCS; all subsequent moves use the new origin
- `G10 L2 Pn X Y Z` — directly sets the offset of WCS n (P1=G54 … P6=G59); persisted to NVM flash
- `G10 L20 Pn X Y Z` — sets offset so that current machine position = specified work position; persisted to NVM flash
- `P0` in G10 — applies to the currently active WCS (sentinel 255 resolved at event-process time)
- WCS offset reload in kinematics is instantaneous; next trajectory move uses the new offset
- `$#` → `[G54:x,y,z]` … `[G59:x,y,z]` report correct stored offsets (existing settings handler)
- Build: zero warnings/errors — `bins/CNC_V3.hex`

---

## ✅ IMPLEMENTED: G43 / G43.1 / G49 Tool Length Offset (TLO) — Phase 2

**Files**:
- `incs/gcode/gcode_parser.h` — Added `GCODE_EVENT_TLO_SET`, `GCODE_EVENT_TLO_CANCEL` to `GCODE_EventType` enum; added `struct { float value; bool dynamic; } tlo` to `GCODE_Event` union
- `incs/motion/kinematics.h` — Declared `KINEMATICS_SetTLO()`, `KINEMATICS_ClearTLO()`, `KINEMATICS_GetTLO(bool*)`
- `srcs/motion/kinematics.c` — Added `s_tlo_value`/`s_tlo_active` statics; implemented TLO API; applied TLO to Z in all four coordinate transforms (`WorkToMachine`, `MachineToWork`, `WorkToMachineWithWCS`, `MachineToWorkWithWCS`)
- `srcs/gcode/gcode_parser.c` — Parsed G43/G43.1/G49 in `parse_command_to_event()`; updated `$#` to report live TLO via `KINEMATICS_GetTLO()`; added G43/G49 to `$G` modal state report
- `srcs/motion/motion_bridge.c` — Added `GCODE_EVENT_TLO_SET` and `GCODE_EVENT_TLO_CANCEL` cases to `MOTION_ProcessGcodeEvent()`; G43 (stored) persists to flash via `SETTINGS_SetToolLengthOffset()`; G43.1 (dynamic) applies live without saving

**Behaviour**:
- `G43`     — activates stored TLO from settings (persisted to NVM flash via `SETTINGS_SetToolLengthOffset()`)
- `G43.1 Zn` — activates inline dynamic TLO = n mm (not persisted; lost at power-cycle)
- `G49`     — cancels active TLO (Z reverts to raw WCS)
- TLO applied to Z axis only (GRBL v1.1 standard)
- TLO persists across soft reset (`Ctrl+X`); cancelled only by explicit `G49`
- `$#` → `[TLO:x.xxx]` now shows the live active value (G43.1 dynamic value shown correctly)
- `$G` → shows `G43` or `G49` in modal state
- Build: zero warnings/errors — `bins/CNC_V3.hex`

---

## ✅ FIXED: Deferred-ok release stalls mid-file when gcode queue throughput is steady — March 5, 2026

**Commit**: `df9d197`
**Files**:
- `incs/data_structures.h:140` — Added `uint32_t commands_consumed` to `GCODE_CommandQueue`
- `srcs/gcode/gcode_parser.c` — `GCODE_CheckDeferredOk()` rewritten to use `commands_consumed` delta
- `srcs/gcode/gcode_parser.c:634,643` — `GCODE_GetNextEvent()` discard paths increment `commands_consumed`
- `srcs/gcode/gcode_parser.c:665` — `GCODE_ConsumeEvent()` increments `commands_consumed`
- `srcs/gcode/gcode_parser.c:198` — `GCODE_SoftReset()` resets `commands_consumed = 0`

**Symptom**: `07_complex_long_run_fast.gcode` (199 lines) stopped at `MPos:30.013,50.013` — mid file,
mid-section. Machine went `<Idle>` without returning to origin. Reproducible.

**Root cause**: `GCODE_CheckDeferredOk` tracked `q->count` (instantaneous queue depth) and released
deferred OKs only when `curr < prev_count` (queue shrank vs previous call). When the planner was
consuming commands at roughly the same rate UGS was sending new ones, `q->count` stayed flat — the
net delta was zero — so `okPendingCount` never drained. UGS's in-flight window exhausted and it
stopped sending. Machine received no new commands and idled mid-file.

**Fix**: Replaced the fragile `prev_count` depth-delta approach with a monotonically-increasing
`commands_consumed` counter in `GCODE_CommandQueue`. Every command removed from the queue (via
`GCODE_ConsumeEvent` or the two silent-discard paths in `GCODE_GetNextEvent`) increments this counter.
`CheckDeferredOk` computes `delta = curr_consumed - prev_consumed` — correctly counting actual
removals regardless of simultaneous arrivals — and releases exactly `delta` deferred OKs. Falls back
to flushing all remaining OKs when `q->count == 0` with no new consumption detected that call.

**Verified**: Full 199-line run completed to `MPos:0.000,0.000,0.000` ✅

---

## ✅ FIXED: Arc fills trajectory queue to high-water → UGS credit stall → machine stops mid-file — March 4, 2026

**Commit**: `f5a3c2e`
**File**: `srcs/motion/motion_bridge.c` → `MOTION_ProcessGcodeEvent`, GCODE_EVENT_ARC_MOVE handler

**Symptom**: `07_complex_long_run_fast.gcode` always stopped at `MPos:30.038,10.050` — end of Section 6
arc 5 (CW east→south, G2 X30 Y10 I-20 J0). Sections 7–11 never executed. Machine went `<Idle>` without
returning to origin.

**Root cause**: `$12 = 0.500mm` was used as **direct segment length** (mm per segment), not as chord
deviation. A R=20 quarter-arc generated `ceil(31.42/0.5) = 63 segments` — exactly filling the 64-slot
trajectory queue to its high-water mark of 63 (highWater = TRAJ_QUEUE_SIZE − 1). Once the queue hit
high-water, `SendOrDeferOk` deferred every subsequent `ok`, exhausting UGS's 1024-byte credit window.
UGS stopped sending after only arc 7 was buffered (arcs 8 and sections 7–11 were never transmitted).
When the gcode queue drained (arcs 5, 6, 7 consumed), the machine idled at arc 5's endpoint with nothing
more to execute.

**Fix**: [srcs/motion/motion_bridge.c](srcs/motion/motion_bridge.c) — switched from direct-length to
GRBL-compatible **chord-deviation** formula:
```
half_angle = acos(1 - tol / radius)
seg_len    = 2 * radius * sin(half_angle)
n_seg      = ceil(arc_length / seg_len)
```
With $12 = 0.500mm chord tolerance: R=20 quarter-arc → 4 segments (seg_len ≈ 8.94mm). The trajectory
queue never approaches high-water, so `ok` responses flow without deferral and UGS streams the full file.

**Segment count comparison** (with $12=0.500mm):
| Arc | Old (seg-length) | New (chord-dev) |
|----|-----------------|-----------------|
| R=10 quarter | 16 segs | 3 segs |
| R=20 quarter | 63 segs ← stall | 4 segs ✓ |
| R=30 semicircle | 189 segs | 9 segs ✓ |

---

## ✅ FIXED: Deferred-ok hysteresis regression blocked final arc submission in chained arc section — March 7, 2026

**Files**:
- `srcs/gcode/gcode_parser.c` — `GCODE_CheckDeferredOk()` now releases deferred `ok` responses by actual queue-depth hysteresis (`HIGH_WATER=48`, `LOW_WATER=16`) and only flushes the final pending responses after `motionQueueCount == 0` and `arcGenState == ARC_GEN_IDLE`
- `README.md` — Flow Control section updated to document the restored high-water / low-water behavior and motion-complete final flush

**Symptom**: `tests/07_complex_long_run_fast.gcode` reproducibly stalled at `MPos:30.031,50.013` after `G2 X30 Y50 I20 J0`. The next programmed line, `G2 X50 Y30 I0 J-20`, never reached the firmware because the sender stopped waiting for one missing deferred `ok`.

**Root cause**: The comments and constants still described queue-depth hysteresis flow control, but `GCODE_CheckDeferredOk()` had been changed to a `commands_consumed` edge-based release path. In steady streaming that can miss one release opportunity and strand a deferred `ok`, leaving the sender blocked mid-file.

**Fix**: Restored queue-depth hysteresis: defer at `HIGH_WATER`, release a burst when the queue falls to `LOW_WATER`, and flush the final deferred acknowledgements only after physical motion and arc generation are both complete.

---

## ✅ FIXED: Flow control UART buffer overflow causing long-file stop — March 4, 2026

**Symptom**: `07_complex_long_run_fast.gcode` stopped mid-run at wrong position `(30,10)` instead of
returning to origin `(0,0)`. Final idle at `FS:5000` level confirms it reached section 11 but
didn't complete.

**Root cause — two compounding bugs**:

*Bug 1:* `motionQueueCount = TRAJECTORY_QueueCount()` — when the last trajectory segment is popped
into the interpolator, `QueueCount()` drops to 0 and ok fires immediately while the move is still
physically executing. The streamer receives ok before the physical move finishes.

*Bug 2:* `startupPrefillDone` flag never became `true`. Pre-fill exits only when
`motionQueueCount >= highWater = 63`. Arcs however cap trajectory at `TRAJ_QUEUE_SIZE-2 = 62`
ia the MOTION_Arc() backoff guard. So the count never reached 63. The system stayed in pre-fill
forever, sending ALL oks immediately. With sequential streaming (~100 commands, ~1500 bytes),
this exceeded the 512-byte UART receive ring buffer and dropped incoming characters, corrupting
gcode commands mid-file and causing wrong positions or early stop.

**Fixes** (`srcs/motion/motion_bridge.c` + `srcs/gcode/gcode_parser.c`):

1. Count the active interpolator move: `motionQueueCount = TRAJECTORY_QueueCount() + (INTERPOLATOR_IsActive() ? 1u : 0u)`
   - Long arcs now hold count at `62+1=63 == highWater` → ok deferred until arc nearly complete
   - Linear moves hold count at `1+1=2 < 63` → ok still immediate (pipelining preserved)

2. `startupPrefillDone = true` from the start — removes the pre-fill phase entirely.
   Pre-fill was designed for UGS-style pipelined senders; for a sequential streamer it caused
   all oks to be immediate, defeating flow control.

**Result**: Clean build `BUILD COMPLETE (bins/CNC_V3.hex)`. Board needs reflash and re-test.

---


**Symptom**: `tests/04_arc_test.gcode` — the final arc (`G2 X20 Y10 I10 J0 F500`, Test 5) traces
into deeply negative X (~−20mm) at Z=5 instead of the expected CW semicircle at Z=0.
The Z=5 give-away proved `G0 Z5` (which follows the arc in the file) executed *before* the arc.

**Root cause (move-ordering race)**:

1. `GCODE_EVENT_ARC_MOVE` is processed → `arcGenState = ARC_GEN_ACTIVE`, `s_planned_position =
   (20,10,0)`, **zero trajectory segments added yet** (MOTION_Arc() is incremental, one per loop).
2. Immediately after, `GCODE_EVENT_LINEAR_MOVE` (G0 Z5) is processed.  Trajectory queue is empty,
   interpolator is idle → falls back to step counter at (0,10,0) → queues **G0Z5 first**.
3. `MOTION_Arc()` then *appends* arc segments behind G0Z5 in the FIFO.
4. Execution order: Z-raise → return-to-origin → arc segments executing from wrong physical position
   → bizarre negative-X path.

**Fix (two-layer guard)**:

*Layer 1 — `srcs/app.c` (already present in current code)*:
```c
bool should_process = (appData.arcGenState != ARC_GEN_ACTIVE);
```
Blocks all G-code event processing while arc is generating segments.

*Layer 2 — `srcs/motion/motion_bridge.c` (added this session, defence-in-depth)*:
```c
case GCODE_EVENT_LINEAR_MOVE: {
    if (appData->arcGenState == ARC_GEN_ACTIVE) return false;  // defer until arc done
    ...
```
When `ProcessGcodeEvent` returns false the event stays in the G-code queue and is retried next loop.

**Also fixed**: `stepper.c` had been re-added to the build (EXCLUDED_FILES entry was dropped).
Re-excluded in `srcs/Makefile` line 134.

**Result**: Clean build `BUILD COMPLETE (bins/CNC_V3.hex)`. Board needs reflash to validate.

---

## ✅ fix: stepper.c duplicate-symbol linker error excluded from build — March 2026

**Problem**: After `make clean && make`, linker reported multiple definitions of `STEPPER_Initialize`,
`STEPPER_LoadSegment`, `STEPPER_StopMotion`, `g_feed_hold_pending`, etc.
`srcs/motion/stepper.c` (old OC1/Bresenham engine, dead code) was being compiled alongside
`srcs/motion/motion_bridge.c` (the S-curve bridge that replaced it), producing duplicate symbols.

**Fix**: Added `stepper.c` to `EXCLUDED_FILES` in `srcs/Makefile` line 135:
```makefile
EXCLUDED_FILES := $(SRC_DIR)/gcode_test_app.c $(SRC_DIR)/gcode_test_framework.c $(SRC_DIR)/gcode_parser_dma.c \
                  $(SRC_DIR)/motion/stepper.c
```

**Result**: Clean build — `BUILD COMPLETE (bins/CNC_V3.hex)`.

---

## ✅ RESOLVED: Arc4 wrong-start bug (March 4, 2026)

**Symptom**: Running `tests/04_arc_test.gcode` — arcs 1–3 trace correctly but arc4
(`G2 X20 Y10 I10 J0 F500`, a CW semicircle, start=(0,10), center=(10,10), end=(20,10))
traces deeply into negative-X (~−20mm) and returns to (0,0) rather than (20,10).
The observed path is consistent with center=(**−10**, 10), i.e. start.X=−20 instead of 0.

**Current debug instrumentation** (`srcs/motion/motion_bridge.c`):
- Per-arc compact print (fires once per `GCODE_EVENT_ARC_MOVE` consumed):
  `A<N>:P/S st=(x,y) c=(cx,cy) e=(ex,ey) r=R n=<segments>`
  where P=planned-position used, S=step-counter used.
- First-segment print: `[SEG1] n=N f=(fx,fy) t=(tx,ty)`
- Last-segment print: `DONE:(x,y,z)`

**Two test runs confirmed**:
- Only arcs 1 and 2 debug prints reach the serial terminal — arcs 3 and 4's `A3:`/`A4:` lines
  are silently dropped because the UART TX buffer (1024 bytes) is saturated by the time arc3
  generates. The `ok` flood from UGS pipelining fills the buffer before arc setup can print.
- arc4's actual `start` is unknown — can't read it from the log yet.

**Hypothesis (unconfirmed)**:
- `s_planned_position` after arc3 should be (0, 10, 0). After the G4P0.5 dwell (no-op),
  with arc3 fully executed and interpolator idle, arc4's guard falls to
  `KINEMATICS_GetCurrentPosition()`. If the step counter reads correctly (0,10,0) the geometry
  is still correct. The −20mm X implies center≈(−10,10), meaning `start.X ≈ −20` at arc4 setup.
- Possible cause: the `[APP] Event type=` print (now removed) was flooding the TX buffer,
  causing UART_Printf to block/drop in unexpected ways. The compact `A<N>:` format should
  survive the buffer. **Next step: flash and read the A3/A4 lines.**

**Resolution**: Arc4 confirmed fixed by the multi-move position accumulation fix (`src=plan`).
Isolated test `arc4_isolated.gcode` confirmed: `[ARC_DONE] end=(20.000,10.000,0.000)`, `MPos:20.025,10.000`. 
Full `04_arc_test.gcode` (all 4 arcs) ran clean on COM11 without errors.

**Next steps**: Test homing, then live machine validation.

**Files modified for debug** (to be reverted/cleaned when bug is fixed):
- `srcs/motion/motion_bridge.c` — compact arc debug prints added
- `srcs/app.c` — removed `arcJustFinished` stale init, removed `[APP] Event type=` flood print

**Branch**: `scurve_motion`
**Feature**: LitePlacer G38.x Probe + TMC5160 SPI Driver
**Started**: February 10, 2026
**Last Updated**: March 3, 2026 (multi-move position accumulation fix)

---

## ✅ fix: multi-move position accumulation (diagonal overshoot) — March 3, 2026

**Problem**: When G-code commands were pipelined faster than execution (UGS streaming),
`TRAJECTORY_AddMove(start, end)` used `KINEMATICS_GetCurrentPosition()` (the live step counter)
as `start`. While move 1 was mid-execute the step counter read a mid-move position (e.g. X=5.45mm
instead of X=20mm), so move 2's `unit_vec` and `millimeters` were computed from the wrong start.
The interpolator then fired the WRONG step counts in the WRONG axis directions from the CORRECT
physical position, causing X to overshoot to 35–40mm instead of staying at 20mm.

**Root cause of first attempted fix**: Added `s_planned_position` tracking but guarded on
`TRAJECTORY_QueueCount() > 0`. After the first move was dequeued to the interpolator the queue
count dropped to 0, so the second command re-anchored from the step counter — the exact wrong
behaviour we were trying to avoid.

**Fix** (`srcs/motion/motion_bridge.c`):
- Changed guard condition from `TRAJECTORY_QueueCount() > 0` to
  `TRAJECTORY_QueueCount() > 0 || INTERPOLATOR_IsActive()` in both `GCODE_EVENT_LINEAR_MOVE`
  and `GCODE_EVENT_ARC_MOVE` handlers.
- `s_planned_position` is now used as `start` whenever ANY motion is in-flight (queued OR
  executing), and anchors to the step counter only when the machine is truly idle.
- Added `MOTION_SyncPlannedPosition()` in `incs/motion/motion.h` — call after soft-reset or
  alarm clear so the next command re-anchors from the correct machine position.

**Files changed**:
- `srcs/motion/motion_bridge.c` — `s_planned_position_valid` guard condition (both LINEAR_MOVE and ARC_MOVE)
- `incs/motion/motion.h` — added `MOTION_SyncPlannedPosition()` declaration

---

## ✅ fix: F-word modality in G1/G2/G3 — March 3, 2026

**Problem**: `F` (feedrate) was not properly modal for G1/G2/G3 commands.
- `GCODE_EVENT_SET_FEEDRATE` (standalone `F200` line) was silently discarded — handler returned `true` without storing value.
- G2/G3 arc moves used `event->data.arcMove.feedrate` directly; if omitted they got feedrate=0 and gave 0 mm/min motion.

**Fixes** (`srcs/motion/motion.c`):
- `case GCODE_EVENT_SET_FEEDRATE:` — now writes `appData->modalFeedrate = event->data.setFeedrate.feedrate` so standalone `F200` correctly sets the modal feed.
- Arc path (~line 510): when arc feedrate == 0, falls back to `appData->modalFeedrate` (default 600 if unset); when arc feedrate > 0, updates `appData->modalFeedrate` so subsequent G1/G2/G3 inherit it.

---

## ✅ fix: segment_buffer.c — GRBL-exact do-while rewrite — March 3, 2026

**Problem**: Rectangle motion showed "slow then burst" behaviour — first ~0.4mm crept at ~0.1mm/poll
then remaining ~19.6mm executed in one poll interval. Root cause: fixed 1ms time slice + forced
minimum-1-step with mm snap caused positional drift that discharged as a burst when cruise speed
was reached.

**Root Cause (found from actual GRBL source)**:
- GRBL's `st_prep_buffer()` extends `dt_max` by `DT_SEGMENT` increments until
  `mm_remaining ≤ minimum_mm` (guarantees ≥1 step naturally — no forcing).
- At 2000 mm/s² from rest, first 1ms covers 0.001mm = 0.08 steps.
  GRBL extends to 7ms to get 1 real step at correct timing.
- `dt_remainder` carries fractional step time forward so cumulative timing is exact.
- `step_interval = ceil(timer_freq × (dt + dt_remainder) / n_steps_actual)` — NOT `freq / (speed × spm)`.

**Changes**:
- `srcs/motion/segment_buffer.c` — **complete rewrite of `SEGMENT_PrepBuffer`**:
  - GRBL-exact do-while inner loop with `dt_max` extension for slow speeds
  - New statics: `prep_dt_remainder`, `prep_steps_remaining_f`
  - Step count: GRBL ceil-difference `ceil(spm×mm_before) - ceil(spm×mm_after)`
  - `step_interval = ceil(TMR4_freq × (dt + dt_remainder) / n_steps_actual)`
  - `dt_remainder = (n_steps_remaining_f - step_dist_remaining) × inv_rate` (carried to next seg)
  - Ramp thresholds: `accel_until_mm = millimeters - accelerate_until`, `decel_after_mm = millimeters - decelerate_after`
  - New block init: `prep_current_speed = initial_speed`, ramp = ACCEL (or CRUISE if entry ≥ nominal)
  - Block consumed when `mm_remaining ≤ 0.0f`
- `srcs/motion/segment_buffer.c:242` — `SEGMENT_Initialize()` simplified: removes stale
  APP_DATA prep fields, resets `prep_dt_remainder` and `prep_steps_remaining_f` statics

**Result**: Compiled clean (`-Werror -Wall`, no warnings). Ready for hardware test.

---



## ✅ fix: stepper.c complete rewrite — March 2, 2026 (session 2)

**Problem**: Previous GPT session left stepper.c in a broken state — Taylor ISR code referencing
deleted struct fields (30+ compile errors), stray code inserted before `#include` statements,
missing closing braces.

**Fix**: Deleted stepper.c and recreated from scratch (~350 lines, clean compile).

**Changes**:
- `srcs/motion/stepper.c` — **fully rewritten from scratch**
  - All Taylor series ISR code deleted (`accel_count`, `rest`, `jerk_count`, `jerk_steps`, `initial_rate`, `nominal_rate`, `final_rate`, `step_interval` on MotionSegment — all gone)
  - `TIMER_4_ISR` is now pure GRBL: Bresenham + step count against `current_segment->n_step`, no acceleration math
  - On segment complete: ISR advances `segmentBufferTail`, loads next StepSegment inline (including updating PR4), or stops TMR4 if buffer empty
  - `STEPPER_LoadSegment()` pops from `segmentBuffer` ring buffer, initialises Bresenham from `StepperBlock`, sets constant PR4
  - `_init_bresenham_from_block()` helper isolates GPIO direction writes (main-loop safe) from ISR Bresenham init
  - All support functions retained: `EnableAll`, `DisableAll`, `StopMotion`, `PauseMotion`, `FinalizeHold`, `ResumeMotion`, `SetDirection`, `GetPosition`, `GetPositionPointer`
- `srcs/motion/segment_buffer.c:18` — Removed broken `extern const uint32_t g_timer_freq` (was declared `static` in kinematics.c, not linkable); replaced with inline `TMR4_FrequencyGet()` call at line 142

**Build result**: ✅ `bins/CNC_V3.hex` — no errors, 1 expected bootloader warning

---

## 🚀 refactor: GRBL segment buffer architecture - remove Taylor profiling — March 2, 2026

**Problem**: Rectangle G-code exhibited junction stuttering (stutter→rocket→stop pattern at corners)
due to Taylor series acceleration profiling with large accel_count values producing tiny velocity deltas.

**Root cause**: Firmware attempted to do acceleration math in ISR using Pramod Ranade/Austin formulas.
After fetching actual GRBL source code, discovered **GRBL has NO acceleration math in ISR at all**.

**GRBL Architecture** (verified from GitHub source):
- Main loop `st_prep_buffer()` breaks each planner block into many small constant-velocity segments
- Each segment: N steps at fixed PR4 value (step_interval)
- ISR executes pure Bresenham with constant rate per segment
- When segment completes, ISR loads next segment (different PR4 value)
- All velocity changes are pre-computed as discrete segments

**Implementation** (March 2, 2026):

1. **Data Structures** (`incs/data_structures.h`):
   - Added `StepperBlock` structure (Bresenham data per planner block)
   - Added `StepSegment` structure (small constant-velocity chunks)
   - Added segment ring buffer (16 entries) to APP_DATA
   - Modified `MotionSegment` to remove Taylor fields:
     - REMOVED: `accel_count`, `accel_count_decel`, `rest`, `jerk_count`, `jerk_steps`, `jerk_steps_log2`
     - REMOVED: `initial_rate`, `nominal_rate`, `final_rate`, `step_interval`, `error[]`
     - KEPT: `initial_speed`, `nominal_speed`, `final_speed` (as floats in mm/s)
     - KEPT: Planner fields (trapezoid boundaries, acceleration, millimeters, unit_vec)

2. **Segment Generator** (`srcs/motion/segment_buffer.c` - NEW):
   - Created `SEGMENT_PrepBuffer()` - breaks planner blocks into segments
   - Uses physics (distance = v*t + 0.5*a*t²) for trapezoid discretization
   - Converts velocity (mm/s) to PR4 value (timer ticks)
   - Fills segment ring buffer continuously from main loop
   - Each segment: n_step count + step_interval (PR4 value)

3. **ISR Simplification** (`srcs/motion/stepper.c`):
   - REMOVED ALL Taylor profiling code (lines 550-600)
   - ISR now: Execute step → Increment counter → Load segment when done
   - PR4 is CONSTANT for each segment (no acceleration math)
   - Velocity changes happen by loading next segment with different PR4

4. **Motion Module** (`srcs/motion/motion.c`):
   - Added call to `SEGMENT_PrepBuffer()` at top of MOTION_Tasks
   - Continuously generates segments from planner blocks

5. **Kinematics Module** (`srcs/motion/kinematics.c` - PENDING CLEANUP):
   - TODO: Remove all Taylor/Austin calculation code
   - TODO: Keep only trapezoid profile calculations
   - TODO: Store speeds as floats (mm/s), remove rate (ticks) calculations

**Benefits**:
- Smooth motion at all speeds (no Taylor approximation errors)
- ISR stays simple and fast (no division, no acceleration math)
- True GRBL architecture adapted for PIC32's dynamic TMR4/PR4 timer
- Eliminates junction stuttering at corners

**Status**: IN PROGRESS - Data structures complete, ISR simplified, kinematics cleanup pending

---

## 🐛 fix: homing segments run at crawl speed — March 2, 2026

**Root causes** (two independent issues):

1. **SETTINGS_VERSION mismatch forces default seek rate** — lookahead branch bumped
   `SETTINGS_VERSION` to 3 (jerk + TMC fields), so NVM saved by master (v2) fails the version
   check and firmware falls back to compile-time defaults: `homing_seek_rate = 500 mm/min` and
   `homing_feed_rate = 100 mm/min`. Updated defaults to `2000` / `500` mm/min.

2. **`KINEMATICS_LinearMoveSimple` wrong for homing** — all three homing phases (seek, locate,
   pulloff) called `LinearMoveSimple`, which is an arc-chord helper. It passes `feedrate` (mm/min)
   as `entry_velocity` / `exit_velocity` (expected mm/s), producing a 60× unit error. This was
   accidentally harmless for nominal speed (the floor clamp to `nominal_rate` catches it) but
   caused the GRBL lookahead planner to corrupt the segment if any stale G-code event in the
   queue triggered `MOTION_ProcessGcodeEvent` → `MOTION_PlannerRecalculate` during homing.

**Fix** — `srcs/motion/homing.c` — `HOMING_StartSeek/Locate/Pulloff`:
- Replace all three `KINEMATICS_LinearMoveSimple(...)` calls with `KINEMATICS_LinearMove(..., 0.0f, 0.0f)`.
- Set `segment->speed_locked = true` immediately after each call so `MOTION_PlannerRecalculate`
  skips these segments in its reverse/forward/trapezoid passes.
- On homing complete (all axes done), call `KINEMATICS_ResetPlannerState()` so the first
  post-homing G-code move gets a clean junction (not the homing unit vector).

**Fix** — `srcs/settings/settings.c` — default `CNC_Settings`:
- `homing_seek_rate`: 500 → 2000 mm/min
- `homing_feed_rate`: 100 → 500 mm/min

---

## 🐛 fix: KINEMATICS_RecalculateTrapezoid unit mismatch — high feedrate frozen (F8000) — March 2, 2026

**Root cause**: `KINEMATICS_RecalculateTrapezoid()` stored `initial_rate`, `nominal_rate`, and
`final_rate` in **steps/sec** (rate units), but the ISR and `STEPPER_LoadSegment` treat these
fields as **timer-tick periods**. After the lookahead planner recalculated any segment's trapezoid,
the ISR's `nominal_rate` floor clamped `step_interval` to e.g. 10 666 ticks (≈ 73 Hz/step ≈
55 mm/min) instead of the correct 73 ticks (≈ 10 666 Hz/step = 8 000 mm/min) — making the motor
appear frozen at anything above ~F500.

**Fix applied** — `srcs/motion/kinematics.c` — `KINEMATICS_RecalculateTrapezoid()`:
- Convert entry/nominal/exit step rates to timer-tick periods (`g_timer_freq / rate`) before
  storing to `seg->initial_rate`, `seg->nominal_rate`, `seg->final_rate`.
- Apply same `MAX_START_RATIO=4` and 7-tick hardware-minimum clamps as `KINEMATICS_LinearMove`.
- Recompute `seg->accel_count` (Austin n_entry) and `seg->accel_count_decel` (n_exit) from the
  corrected tick periods so the Taylor ramp starts at the right index.
- Set `seg->step_interval = seg->initial_rate` so `STEPPER_LoadSegment` loads the correct period.

---

## ✅ feat: GRBL-exact lookahead planner (reverse+forward pass + junction speed) — March 3, 2026

Complete replacement of the custom look-ahead system with an exact port of GRBL v1.1's
`plan_buffer_line()` / `planner_recalculate()` two-pass algorithm.

### `incs/data_structures.h`
- `MotionSegment` look-ahead section (was `entry_speed_mms`, `exit_speed_mms`, `speed_locked`):
  - Replaced with GRBL-exact fields: `unit_vec[NUM_AXIS]`, `entry_speed_sqr`,
    `max_entry_speed_sqr`, `max_junction_speed_sqr`, `millimeters`, `speed_locked`

### `srcs/motion/kinematics.c`
- Added `pl_previous_unit_vec[NUM_AXIS]` and `pl_previous_nominal_speed` static planner state.
- Added `convert_to_unit_vector()` — normalises a vector in-place, returns magnitude (mm).
  Equivalent to GRBL's `convert_delta_vector_to_unit_vector()`.
- Added `limit_acceleration_by_axis()` — axis-limited combined acceleration.
  Equivalent to GRBL's `limit_value_by_axis_maximum(settings.acceleration, unit_vec)`.
- Added `limit_rate_by_axis()` — axis-limited combined feed rate (mm/s).
  Equivalent to GRBL's `limit_value_by_axis_maximum(settings.max_rate, unit_vec)`.
- Added `KINEMATICS_ResetPlannerState()` — zeroes planner static state; called at init and soft reset.
- `KINEMATICS_Initialize()` — added `KINEMATICS_ResetPlannerState()` call at end.
- `KINEMATICS_LinearMove()` — replaced `limiting_axis` approach with GRBL-exact unit-vector +
  axis-limited accel/rate; writes `unit_vec`, `millimeters`, `max_junction_speed_sqr`,
  `max_entry_speed_sqr`, `entry_speed_sqr` (conservative), and updates `pl_previous_*` state.
  Junction speed uses GRBL centripetal approximation (dot-product, no trig).
- Added `KINEMATICS_RecalculateTrapezoid(seg, entry_mms, exit_mms)` — exact port of GRBL's
  `calculate_trapezoid_for_block()`. Writes `initial_rate`, `nominal_rate`, `final_rate`,
  `accelerate_until`, `decelerate_after`, `step_interval` from planned entry/exit speeds.

### `srcs/motion/motion.c`
- **Removed** `MOTION_RecomputeExit()` static function (~55 lines) — superseded by planner.
- `MOTION_ProcessGcodeEvent()` G0/G1 handler:
  - **Removed** the entire junction-calc + `KINEMATICS_CalculateJunctionSpeed()` block (~70 lines).
  - **Removed** the manual backward pass (`while (i != tail)`) block (~40 lines).
  - Replaced with: `KINEMATICS_LinearMove(start, end, feedrate, segment, 0, 0)`
    + enqueue + `MOTION_PlannerRecalculate(appData)`.
- `MOTION_ProcessGcodeEvent()` arc handler:
  - **Removed** the `MOTION_RecomputeExit` arc-entry patch block.
  - GRBL junction calculation in `KINEMATICS_LinearMove` handles G1→arc entry automatically.
- Added `MOTION_PlannerRecalculate(APP_DATA*)` — static function implementing GRBL's
  `planner_recalculate()` three-pass algorithm:
  1. **Reverse pass**: newest→oldest, propagate decel constraints backward.
  2. **Forward pass**: oldest→newest, propagate accel constraints forward.
  3. **Trapezoid pass**: calls `KINEMATICS_RecalculateTrapezoid` on every plannable segment.

### `incs/motion/kinematics.h`
- Added `KINEMATICS_RecalculateTrapezoid(MotionSegment*, float entry_mms, float exit_mms)` declaration.
- Added `KINEMATICS_ResetPlannerState(void)` declaration.

---

## ✅ Optimisation: Cache TMR4 frequency at init — March 2, 2026

- `srcs/motion/kinematics.c:13` — Removed dead `TIMER_TICKS_PER_SECOND_DYNAMIC()` macro (no longer used).
- `srcs/motion/kinematics.c:16` — Added `static float g_timer_freq` file-scope variable.
- `srcs/motion/kinematics.c:20` — `KINEMATICS_Initialize()` — Added `g_timer_freq = (float)TMR4_FrequencyGet()` to cache the value once at startup. TMR4 prescaler is fixed by MCC configuration and never changes at runtime.
- `srcs/motion/kinematics.c:225` — `KINEMATICS_LinearMove()` — Removed pointless `const float TIMER_FREQ = g_timer_freq` local alias; all four use-sites now reference `g_timer_freq` directly. Eliminates an unnecessary register copy on every call.

---

## ✅ Optimisation: Merge mm→steps and dominant-axis loops — March 2, 2026

- `srcs/motion/kinematics.c:162` — `KINEMATICS_LinearMove()` — Merged two sequential `for (E_AXIS …)` loops into one. The accumulation (`step_accumulator` → `delta[axis]`) and dominant-axis search (`abs(delta[axis]) > max_delta`) now execute in a single pass over `NUM_AXIS`. Halves loop overhead and improves cache locality — `delta[axis]` is read by the dominant-axis check immediately after being written, while it is still hot in cache. No behavioural change.

---

## ✅ Fix: Remove [CHECK] debug flood in motion.c — March 1, 2026

- `srcs/motion/motion.c:68` — `MOTION_Tasks()` — Removed the throttled `[CHECK] steps_completed/steps_remaining` debug print. Even throttled at 5000 iterations, at 200MHz the main loop fires it roughly once per step (~800 prints per 10mm move at F100), saturating UART during motion. Serial test confirmed motion works correctly (800/800 steps complete). The `[SEGMENT] Complete` print at segment end is retained.

---

## ✅ Fix: Zero-step early-exit in KINEMATICS_LinearMove — March 1, 2026

- `srcs/motion/kinematics.c:197` — `KINEMATICS_LinearMove()` — Added early-exit guard immediately after `max_delta` is computed: when all axes produce 0 steps, set `steps_remaining = 0` and return without running the Taylor/trapezoid physics (avoids wasted computation and spurious `[KIN]` debug lines for bare `G0` / feedrate-only `G1 Fxxx` commands). Caller in `motion.c` already has a `steps_remaining == 0` check that discards the segment and updates position correctly.

---

## ✅ Fix: Taylor series MIN_STEP_HZ clamp + consistent n derivation — March 1, 2026

- `srcs/motion/kinematics.c` — Replaced previous `safe_start*3` block and `n_entry=0` approach:
  - Added `MIN_STEP_HZ = 200.0f`: physics `c₀` clamped so motor never steps slower than 200 Hz — prevents stall on first step at low acceleration / high microstepping settings
  - `n_entry` / `n_exit` now derived **from the actual `initial_rate`/`final_rate` stored** via `v_eff = TIMER_FREQ/(rate·spm)`, `n = 2·v_eff²·spm/a − 1` — eliminates the (rate, n) inconsistency that caused frozen/creeping acceleration
  - When `c₀` is not clamped the formula gives `n=0` exactly (true from-rest start); when clamped it gives `n>0`, consistently reflecting the mid-ramp start speed
  - Same symmetric fix applied to decel exit side (`n_exit`, `final_rate`)

---

## ✅ Feature: S-curve jerk control — March 1, 2026

**What was built**: In-ISR S-curve acceleration shaping using a linear ramp applied to the
existing Taylor series. The Taylor recurrence still computes the exact constant-acceleration
step delta each ISR call; a new jerk counter gates how much of that delta is applied.

**Effect**: Acceleration builds from zero to A_max over `jerk_steps` dominant-axis steps
(S-curve ramp-in), then A_max is maintained for the rest of the accel phase. Same S-curve
applies at the start of the decel phase (ramp-in from cruise to full decel). Result: no
instantaneous jump from zero to A_max at accel/decel phase boundaries — mechanically smooth.

**Formula**: `jerk_steps = round_down_pow2( min( A_max * spm / jerk_setting , accel_steps/2 ) )`
- Low jerk setting → more steps in S-curve ramp → very smooth, slightly slower ramp-up
- High jerk setting → fewer steps → approaches pure trapezoidal (legacy behaviour)
- `jerk_steps` forced power-of-2 so ISR division is a single arithmetic right-shift (zero cost)

**Files changed**:
- `incs/settings/settings.h:32` — Added `float jerk[4]` field after `max_travel[4]`, bumped `SETTINGS_VERSION` (DRV8825: 2→3, TMC5160: 3→4)
- `srcs/settings/settings.c:65` — Added `.jerk = {500.0f, 500.0f, 200.0f, 500.0f}` to `default_settings`
- `srcs/settings/settings.c:285` — `SETTINGS_ProcessParameter()` — added cases 140-143 (set)
- `srcs/settings/settings.c:400` — `SETTINGS_GetParameter()` — added cases 140-143 (get)
- `srcs/settings/settings.c:512` — `SETTINGS_PrintAll()` — added `$140`-`$143` output lines
- `incs/data_structures.h:98` — `MotionSegment` struct: added `jerk_steps`, `jerk_steps_log2`, `jerk_count` fields
- `srcs/motion/kinematics.c:300` — `KINEMATICS_LinearMove()` — compute and store `jerk_steps` / `jerk_steps_log2` / `jerk_count` from `settings->jerk[cfg_axis]`
- `srcs/motion/stepper.c:565` — `TIMER_4_ISR` — reset `jerk_count = 0` at decel phase entry
- `srcs/motion/stepper.c:578` — `TIMER_4_ISR` — accel phase: apply S-curve scaling via bit-shift
- `srcs/motion/stepper.c:600` — `TIMER_4_ISR` — decel phase: apply S-curve scaling via bit-shift

**ISR cost**: 1 comparison + 1 multiply + 1 bit-shift per step during jerk ramp only. Zero cost during cruise and after jerk ramp completes (condition `jerk_count >= jerk_steps` exits early).

**New GRBL parameters**:
- `$140` — X jerk (default 500.0)
- `$141` — Y jerk (default 500.0)
- `$142` — Z jerk (default 200.0)
- `$143` — A jerk (default 500.0)

**Build**: clean, no warnings.

---

## ✅ Bug Fix: G0 rapids running at modal feedrate instead of max_rate — March 1, 2026

**Root cause**: `GCODE_EVENT_LINEAR_MOVE` had no `isRapid` flag. Both G0 and G1 produced
the same event. In `motion.c`, when `feedrate == 0` (no F word on G0), the code substituted
`modalFeedrate` (e.g. F500) instead of the per-axis `max_rate` settings. All G0 moves crawled
at the last programmed feed rate, adding up to a perceived "30-second pause" after any G-code
program containing rapids.

**Fix**:
- `incs/gcode/gcode_parser.h:61` — Added `bool isRapid` to `linearMove` event struct
- `srcs/gcode/gcode_parser.c:479` — Set `ev->data.linearMove.isRapid = (gnum == 0)` in parser
- `srcs/motion/motion.c:337` — When `isRapid`, compute vector feedrate from per-axis `max_rate`
  using GRBL inverse-time approach: `feedrate = total_dist / max(delta[axis] / max_rate[axis])`.
  G0 does **not** update `modalFeedrate` (G0 speed is not persistent).

**Effect**: G0 rapids now run at up to 5000 mm/min (X/Y default max_rate) instead of F500.
The "30-second" two-square test should now complete in ~15 seconds (10s G1 motion + fast G0s).

---

## ✅ Bug Fix: INT3R wrong PPS value — E-Stop ISR never fired — March 1, 2026

**Root cause**: `INT3R = 3` in `srcs/config/default/peripheral/gpio/plib_gpio.c` mapped
the INT3 external interrupt input to the wrong peripheral pin via PPS (Peripheral Pin Select).
RF4 (E-Stop pin) requires `INT3R = 2`. With value 3, INT3 was connected to a different
pin entirely so the ISR never fired regardless of button state.

**Fix**: Changed by user in `srcs/config/default/peripheral/gpio/plib_gpio.c:103`:
```c
INT3R = 2;    /* INT3 → RPF4 (RF4) for E-Stop — must be 2, not 3 */
```

**Key lesson**: When an ISR never fires, **check PPS mapping first** (`INT3R`, `U3RXR` etc.)
before looking at edge polarity, priority, or code logic. A wrong PPS value silently
misroutes the interrupt source to a different pin.

**Wrong analysis (reverted)**: `INTCONSET→INTCONCLR` change in `plib_evic.c` was
incorrect and has been reverted — `INTCONSET = _INTCON_INT3EP_MASK` is correct for
falling-edge on this device.

---

## ⚠️ Post-Mortem: Homing broken by unnecessary code changes — March 1, 2026 (REVERTED)

**What happened**: User reported "Y won't home after Z" after a firmware flash. Rather than
first verifying whether the issue was a settings/hardware problem, changes were made directly
to production homing code based on analysis of a conversation summary that described
*earlier-session* bugs (some of which had already been fixed in code). Specifically:

- `HOMING_NextAxis()` in `srcs/motion/homing.c` was rewritten (adding `return true`, `break`,
  `UTILS_HomingSetCurrentAxis`, `UTILS_HomingLimitReset`) — these changes introduced unexpected
  timing behaviour and broke the working state machine
- `HOMING_IsLimitActiveNow()` was added to `homing.c` + `homing.h` — the function already
  existed as `HOMING_LimitTriggered()` and the stale summary had mislabelled the issue
- `STATUS.md` was updated prematurely to record these as completed fixes

**Root cause of the bad session**: The conversation summary described three "candidate bugs" from
earlier analysis. At least one (the $22 mask issue) had already been fixed in committed code.
Changes were applied without first reading the current file contents to confirm the bugs still
existed, violating the workspace rule to always read before editing.

**Resolution**: All changes reverted with `git restore`. The Y homing issue was resolved by
the user correcting flash settings (steps/mm, rates). No code change was needed.

**Lesson**: When homing reports a symptom after a settings change, **check settings first**
(`$$`), build with `DEBUG_MOTION` to observe state transitions, before touching working code.

---

## ✅ Bug Fix: LinearMoveSimple unit mismatch — homing crept at ~387 mm/min regardless of $24/$25 — March 1, 2026

**Root cause**: `KINEMATICS_LinearMoveSimple()` compared `sqrtf(arc_accel * chord_mm)` (mm/s)
directly against `feedrate` (mm/min) in a `fminf()`. The sqrt result (~387 mm/s for typical
accel/distance) is always much smaller than the feedrate (e.g. 20000 mm/min), so `fminf` always
selected the sqrt value and passed it to `KINEMATICS_LinearMove` which then treated it as mm/min.
Result: homing ran at ~387 mm/min no matter what $24/$25 were set to.

**Fix**: `srcs/motion/kinematics.c:536` — `KINEMATICS_LinearMoveSimple()` — Multiply v_peak by
`60.0f` to convert mm/s → mm/min before the `fminf` comparison:
```c
float v_peak_mm_min = sqrtf(arc_accel * chord_mm) * 60.0f;
float arc_cruise = fminf(feedrate, v_peak_mm_min);
```
Homing now runs at the full $25 seek rate and $24 feed rate as commanded.

**Commit**: `5db7318`

---

## ✅ Bug Fix: rate_delta ceiling division — motor never reached cruise speed — March 1, 2026

**Root cause**: `KINEMATICS_LinearMove()` computed `rate_delta` using floor (integer) division:
```c
rate_delta = (initial_rate - nominal_rate) / accel_steps;   // floor → too small
```
Because floor rounds down, `rate_delta × accel_steps < (initial_rate - nominal_rate)`.  
The ISR's snap-to-nominal condition (`step_interval <= nominal_rate + rate_delta`) was never
triggered within `accel_steps` iterations, so the motor exited the accel phase still above
`nominal_rate` — permanently stuck at a lower-than-commanded cruise speed.

**Example with defaults (F2000, steps/mm=156, accel=500)**:
- `initial_rate=601`, `nominal_rate=150`, `accel_steps=173`
- `rate_delta = 451/173 = 2` (floor)
- After 173 steps: interval = `601 − 2×173 = 255` ticks → ~19 mm/s (not 33.3 mm/s)

**Fix**: Ceiling division ensures `rate_delta × accel_steps ≥ range`, so the snap fires before accel ends:
```c
uint32_t accel_range = segment_buffer->initial_rate - segment_buffer->nominal_rate;
segment_buffer->rate_delta = (accel_range + accel_steps - 1) / accel_steps;  // ceil
```
Same fix applied to `decel_rate_delta` and the copy in `MOTION_RecomputeExit()`.

**Files changed**:
- `srcs/motion/kinematics.c` — `KINEMATICS_LinearMove()`: ceiling for `rate_delta` and `decel_rate_delta`
- `srcs/motion/motion.c` — `MOTION_RecomputeExit()`: ceiling for `decel_rate_delta`

---

## ✅ Bug Fix: MOTION_Arc stop/go jitter — March 2, 2026

**Root cause**: `MOTION_Arc()` generated exactly one arc segment per main-loop call. The ISR consumed segments in microseconds; the main loop was too slow to refill → queue drained to 0 between arc segments → motor stopped and restarted on every segment.

**Fix**: Changed to a `while(arcGenState == ARC_GEN_ACTIVE && motionQueueCount < MAX_MOTION_SEGMENTS)` loop that fills the entire 16-slot queue in one call.

- `srcs/motion/motion.c:162` — `MOTION_Arc()`: one-segment per call → fill-queue loop

---



**Root cause**: SETTINGS_VERSION was bumped from 2 → 3 when TMC5160 fields were added to `CNC_Settings`. Since all axes are currently `DRIVER_DRV8825`, `HAS_TMC5160_AXIS` is NOT defined, so those fields do not exist in the struct — the binary layout is identical to version 2. However the hard-coded `#define SETTINGS_VERSION 3` caused the version check in `SETTINGS_LoadFromFlash` to fail, so the firmware silently fell back to defaults (`steps_per_mm=156, max_rate Z=2000`) instead of loading calibrated flash values. Symptom: correct acc/dec shape but wrong absolute speed.

**Fix**: Made `SETTINGS_VERSION` conditional on `HAS_TMC5160_AXIS`:
- `incs/settings/settings.h:88` — `#define SETTINGS_VERSION 2` when DRV8825-only (struct unchanged); `#define SETTINGS_VERSION 3` when `HAS_TMC5160_AXIS` (TMC fields present → struct differs from v2)

**Rule going forward**: Only bump `SETTINGS_VERSION` when the `CNC_Settings` struct binary layout actually changes (i.e. when unconditional fields are added/reordered, not when `#ifdef`-guarded fields are added).

---
## ✅ Documentation: UML + README Updated for lookahead Architecture — March 1, 2026

Updated all architecture documentation to reflect current `lookahead` branch implementation. Removed all references to the deleted Priority Phase System and old TMR2/OC2/OC3/OC4 per-axis ISR architecture.

- `docs/plantuml/02_segment_clock.puml` — **Complete rewrite**: Now documents period-based TMR4/OC1/TMR5 single-ISR architecture, M14K shadow register swap, switch/Harmony macro GPIO pattern, direct-SFR Timer/OC, Bresenham inside ISR, velocity profiling inside ISR, pulse-width TMR5 callback, race window note
- `docs/plantuml/01_system_overview.puml` — **Complete rewrite**: Removed TMR2 free-running, OC2/OC3/OC4 per-axis ISRs, Priority Phase enum. Added `TIMER_4_ISR` single-ISR with shadow regs, look-ahead planner in kinematics, arc incremental generator
- `docs/plantuml/03_arc_linear_interpolation.puml` — **Targeted fix**: Replaced `[Priority Phase\nSystem]` note with `[MOTION_Tasks Segment Queue]`. Replaced `[Hardware Timers OC1/OC2/OC3/OC4]` with `[TIMER_4_ISR TMR4/TMR5]`
- `README.md:9` — Branch updated: `tmc5160` → `lookahead`
- `README.md:11` — Last build date updated: February 28, 2026 → March 1, 2026
- `README.md:19` — Feature table: Added "Look-ahead backward-pass planner (junction velocity) ✅ Complete"
- `README.md:112` — Call hierarchy: "motion.c — motion queue, phase system" → "motion.c — segment queue, look-ahead planner"
- `README.md:128–143` — **Deleted** "### Priority Phase System" section (5-phase enum no longer exists in codebase)
- `README.md:128` — **Added** "### ISR Architecture" section: M14K shadow regs, switch/Harmony macro rationale, GPIO vs Timer/OC split, 8-step ISR sequence, TMR5 callback
- `README.md` — **Added** "### Look-Ahead Planner" section: backward-pass junction velocity, pre-computed per-segment integer deltas

---

## ✅ Look-Ahead Single-Step Retroactive Planner — February 28, 2026

Implements single-step retroactive velocity planning across `data_structures.h`, `kinematics.c`, and `motion.c`.

**Core change**: when segment N arrives, the planner retroactively patches segment N-1's exit profile to the computed junction speed instead of always decelerating to safe-start (8.33 mm/s).

- `incs/data_structures.h` — `MotionSegment`: Added `decel_rate_delta` (separate accel/decel deltas), `entry_speed_mms`, `exit_speed_mms`, `speed_locked`
- `srcs/motion/kinematics.c` — `KINEMATICS_LinearMove()`: Initialise all new fields; compute `decel_rate_delta = (final_rate - nominal_rate) / decel_steps` independently from `rate_delta`
- `srcs/motion/motion.c` — Decel phase: use `seg->decel_rate_delta` instead of `abs(seg->rate_delta)` (correct for asymmetric entry/exit speeds)
- `srcs/motion/motion.c` — Added `MOTION_RecomputeExit()`: patches `final_rate`, `decelerate_after`, `decel_rate_delta` from a new exit speed
- `srcs/motion/motion.c` — `MOTION_ProcessGcodeEvent()` G1 handler: hoisted `junction_speed`; after queuing N, calls `MOTION_RecomputeExit(N-1, junction_speed)` — retroactive patch guarded by `motionQueueTail` and `speed_locked`
- `srcs/motion/motion.c` — `MOTION_Arc()`: sets `segment->speed_locked = true` on every arc segment
- `srcs/motion/motion.c` — `GCODE_EVENT_ARC_MOVE` handler: computes `arc_cruise = min(feedrate, sqrt(accel * mm_per_arc_segment))` and patches preceding G1 exit to `arc_cruise` for smooth G1→arc entry

**Effect**:
- Consecutive same-direction G1 moves glide through without decelerating
- Corner G1 moves decelerate to the correct angle-computed junction speed
- G1→arc: preceding G1 decelerates to arc cruise speed not to near-stop
- Arc segments remain speed-locked, immune to further patching

---

## ✅ Arc Cruise Speed Fix — February 28, 2026

- `srcs/motion/kinematics.c:500` — `KINEMATICS_LinearMoveSimple()` — Replaced hardcoded `8.33 mm/s` entry/exit velocity with computed triangle-peak cruise speed: `arc_cruise = min(feedrate, sqrt(min_accel_xy * chord_mm))`. Passes `arc_cruise` as feedrate AND entry/exit so `nominal=initial=final` — trapezoidal profiler is fully inactive for arc segments. Consecutive arc segments run back-to-back at constant speed with no inter-segment deceleration. Fixes circles running at ~11 mm/min instead of commanded feedrate.

---

## ✅ Velocity Profiling Restored — February 28, 2026

Restores the GRBL-style trapezoidal acceleration that was disabled in commit `07939e8`.
All three fixes applied simultaneously to avoid race conditions.

- `srcs/motion/kinematics.c:258` — `KINEMATICS_LinearMove()` — Fix 1: `min_steps_per_sec` now computed as GRBL safe-start `sqrtf(2*a/steps_per_mm)` clamped to `[1, cruise]`. Replaces incorrect `= steps_per_sec` that forced `rate_delta=0`.
- `srcs/motion/kinematics.c:299` — `KINEMATICS_LinearMove()` — Fix 2: `step_interval = initial_rate` (safe-start speed). Removes `⚠️ TEMPORARY` that hardwired cruise from step 1.
- `srcs/motion/stepper.c:291` — `STEPPER_SetStepRate()` — Fix 3: Removed `TMR4_PeriodSet` / `OCMP1_CompareValueSet` / `OCMP1_CompareSecondaryValueSet` from main loop. ISR already writes these registers every step from `step_interval` — sole hardware writer is now the ISR, eliminating the race condition that caused X/Y noise in prior attempts.

**Tag before changes**: `v1.1h-20260228-pre-accel`

---

## ✅ Full GRBL Feed Hold / Resume (Graceful Drain) — February 28, 2026

Full GRBL v1.1 compliant `!` (feed hold) / `~` (cycle start) with **two-flag graceful drain** —
in-flight segments complete naturally before the hardware is stopped.

### Two-Flag Architecture (defined in `stepper.c`, declared in `stepper.h`)

| Flag | Meaning | GRBL Status | Hardware |
|------|---------|-------------|----------|
| `g_feed_hold_pending` | `!` received, queue draining | `Hold:1` | TMR4/OC1 running |
| `g_feed_hold_active` | queue empty, fully parked | `Hold:0` | TMR4/OC1 stopped |

### Changed files

- `srcs/motion/stepper.c:413` — `STEPPER_PauseMotion()` — if queue non-empty sets `g_feed_hold_pending` only (drain); if queue already 0 goes straight to `g_feed_hold_active` (immediate stop)
- `srcs/motion/stepper.c:447` — `STEPPER_FinalizeHold()` — NEW: clears pending, sets active, stops TMR4/OC1/TMR5; called by `MOTION_Tasks` when `motionQueueCount` hits 0
- `srcs/motion/stepper.c:461` — `STEPPER_ResumeMotion()` — cancels `g_feed_hold_pending` (motion continues) OR clears `g_feed_hold_active` and restarts TMR4/OC1
- `incs/motion/stepper.h:30` — Added `extern volatile bool g_feed_hold_pending` and `void STEPPER_FinalizeHold(void)`
- `srcs/motion/motion.c:300` — `MOTION_Tasks()` — at queue-drain (`motionQueueCount == 0`) calls `STEPPER_FinalizeHold()` if pending, else normal `TMR4_Stop()`
- `srcs/gcode/gcode_parser.c:876` — status `?` returns `Hold:1` when pending, `Hold:0` when active
- `srcs/gcode/gcode_parser.c:915` — `~` calls `STEPPER_ResumeMotion()` when either flag set
- `srcs/gcode/gcode_parser.c:174` — soft reset clears both flags
- `srcs/app.c:326` — `MOTION_Tasks` gated on `!g_feed_hold_active` only (runs during drain)
- `srcs/app.c:344` — arc generation gated on `!g_feed_hold_pending` (inside the active gate)
- `srcs/app.c:434` — event processing gated on `!g_feed_hold_active && !g_feed_hold_pending`

**Soft reset (`Ctrl+X`)**: clears `g_feed_hold_active` without resuming motion (calls `STEPPER_StopMotion` separately)

**Files changed**:
- `srcs/motion/stepper.c:67` — Added `volatile bool g_feed_hold_active = false;`
- `incs/motion/stepper.h:30` — Added `extern volatile bool g_feed_hold_active;`
- `srcs/motion/stepper.c:407` — `STEPPER_PauseMotion()` sets flag before stopping hardware
- `srcs/motion/stepper.c:426` — `STEPPER_ResumeMotion()` clears flag + handles NULL-segment resume
- `srcs/gcode/gcode_parser.c:69` — Removed `static bool feedHoldActive`; replaced all 5 refs with `g_feed_hold_active`
- `srcs/app.c:322` — `if (!g_feed_hold_active)` gates `MOTION_Tasks` + arc generation
- `srcs/app.c:432` — `if(appData.state != APP_ALARM && !g_feed_hold_active)` gates event processing

Build result: ✅ **BUILD COMPLETE** (`bins/CNC_V3.hex`)

---

## 🔧 Linker Fix: `g_estop_pending` moved to stepper.c — February 28, 2026

After `make clean`, the linker reported `undefined reference to 'g_estop_pending'` from
`gcode_parser.c:161`. Root cause: `g_estop_pending` was defined in `app.c` but the linker
(with `--gc-sections` + `-fdata-sections`) failed to resolve it across the `app.o` → `gcode_parser.o`
dependency. Fixed by moving the definition to `stepper.c` to match the established pattern
used by `g_hard_limit_alarm` and `g_suppress_hard_limits` (both defined in `stepper.c`,
declared in `stepper.h`, which is directly included by all consumers).

- `srcs/motion/stepper.c:63` — Added `volatile bool g_estop_pending = false;`
- `incs/motion/stepper.h:26` — Added `extern volatile bool g_estop_pending;`
- `srcs/app.c:59` — Removed definition (replaced with comment referencing stepper.c)
- `incs/app.h:22` — Removed `extern volatile bool g_estop_pending;` (now in stepper.h)

Build result: ✅ **BUILD COMPLETE** (`bins/CNC_V3.hex`)

---

## ✅ E-Stop Hardware Interrupt — February 27, 2026

E-Stop button on **RF4** (Change Notice Port F, IPL7) — highest priority ISR, beats OC1 at IPL5.

**MCC Configuration**:
- RF4 = digital input, internal pullup enabled, CN-F interrupt, IPL7SRS
- SPI2 = IPL1 (lowest — only used at startup/idle, never during motion)
- Interrupt vector: `CHANGE_NOTICE_F_Handler` → `CHANGE_NOTICE_F_InterruptHandler()` → `ESTOP_Callback()`

**Changes**:
- `incs/data_structures.h:191` — `alarmCode` comment updated; added `volatile bool eStopTriggered`
- `srcs/app.c:57` — Added `ESTOP_Callback()` static function: disables steppers, stops TMR4, flushes motion queue, sets `alarmCode=10`, transitions to `APP_ALARM`
- `srcs/app.c` (APP_CONFIG) — Registered `GPIO_PinInterruptCallbackRegister(ESTOP_PIN, ESTOP_Callback, NULL)` + `ESTOP_InterruptEnable()`
- `srcs/app.c` (APP_ALARM) — Reports `ALARM:10\r\n` once on E-Stop; recovery via Ctrl+X soft reset from host

**Recovery sequence**: Button released → host sends `Ctrl+X` → soft reset → `APP_IDLE`



## 🔧 UART3 TX Buffer Restored — February 26, 2026

MCC regeneration (during SPI2/TMC5160 addition) reverted `UART3_WRITE_BUFFER_SIZE` from 1024 back
to 256. This caused UGS to disconnect immediately after sending `$$` — the settings response
(~400-500 bytes) overflowed the 256-byte TX ring buffer.

- `srcs/config/default/peripheral/uart/plib_uart3.c:60` — `UART3_WRITE_BUFFER_SIZE` restored to `1024U`
- `srcs/config/default/peripheral/uart/plib_uart3.c:61` — `UART3_WRITE_BUFFER_SIZE_9BIT` restored to `1024U >> 1`

> ⚠️ After **any** MCC regeneration always verify `UART3_WRITE_BUFFER_SIZE = 1024U` in this file.

---

## 🔧 Build System Flattened — February 26, 2026

Removed `Debug`/`Release` subdirectory split from the build system. Single output path for all
builds — no hardware JTAG debugger is available so the distinction was meaningless.

**Output change**:
- Before: `bins/Release/CNC_V3.hex`, `objs/Release/`, `other/Release/`, `libs/Release/`
- After:  `bins/CNC_V3.hex`, `objs/`, `other/`, `libs/`

**Deleted directories**: `bins/Release`, `bins/Debug`, `objs/Release`, `objs/Debug`,
`other/Release`, `other/Debug`, `libs/Release`, `libs/Debug`

**Files modified**:
- `Makefile` — Removed `BUILD_CONFIG ?= Release`, all `$(BUILD_CONFIG)` references,
  `clean_all` target; `build` target now does `cd bins && xc32-bin2hex`
- `srcs/Makefile` — Removed `BUILD_CONFIG ?= Default`; `BIN_DIR/OBJ_DIR/OUT_DIR/LIB_DIR`
  now flat; replaced `ifeq Debug/Release/error` block with single `OPT_FLAGS := -g -O$(OPT_LEVEL)`;
  `build_dir` no longer creates `bins/Debug` and `bins/Release`; map file is `CS23.map`

**Serial debug output** still controlled by `DEBUG_FLAGS="DEBUG_MOTION DEBUG_GCODE"` — same
compile-time zero-overhead mechanism, just no longer tied to a build config name.

**Build verified**: `bins/CNC_V3.hex` (262637 bytes), only pre-existing bootloader pragma warning.

---

## 🔧 TMC5160 Runtime Settings via $= — February 26, 2026

Added `$200–$253` parameter range for TMC5160 runtime motor tuning. Parameters persist in NVM
flash and take effect immediately (re-configures the driver via SPI without rebooting).
All code is guarded `#ifdef HAS_TMC5160_AXIS` — DRV8825-only builds compile identically to before.

**Parameter Map** (values are applied per-axis: 0=X, 1=Y, 2=Z, 3=A):

| Param  | Axis | Description                                               | Default |
|--------|------|-----------------------------------------------------------|---------|
| $200–$203 | X–A | Chopper mode: 1=StealthChop 2=SpreadCycle 3=Mixed 4=CoolStep | 1 |
| $210–$213 | X–A | Run current 0–31 (linear % of Vref)                      | 20      |
| $220–$223 | X–A | Hold current 0–31                                         | 10      |
| $230–$233 | X–A | Microstep resolution (0=256 … 8=full-step)                | 4 (16µ) |
| $240–$243 | X–A | TPWMTHRS — StealthChop→SpreadCycle crossover (Mixed mode) | 500     |
| $250–$253 | X–A | TCOOLTHRS — CoolStep lower velocity threshold             | 0       |

**Files changed:**

- `incs/settings/settings.h:7` — Added `#include "common.h"` so `HAS_TMC5160_AXIS` is visible
- `incs/settings/settings.h:58–65` — Added `#ifdef HAS_TMC5160_AXIS` block with 6 arrays inside `CNC_Settings`
- `incs/settings/settings.h:SETTINGS_VERSION` — Bumped 2 → 3 (struct changed; old flash data is invalid and triggers defaults)
- `srcs/settings/settings.c:default_settings` — Added `#ifdef HAS_TMC5160_AXIS` block with sensible defaults matching `g_default_cfg` in `tmc5160.c`
- `srcs/settings/settings.c:SETTINGS_SetValue()` — Added `case 200–253` inside `#ifdef HAS_TMC5160_AXIS`
- `srcs/settings/settings.c:SETTINGS_GetValue()` — Added `case 200–253` inside `#ifdef HAS_TMC5160_AXIS`
- `srcs/settings/settings.c:SETTINGS_PrintAll()` — Added 24 `sprintf` lines for `$200–$253` inside `#ifdef HAS_TMC5160_AXIS`
- `incs/motion/tmc5160.h:24` — Added `#include "settings/settings.h"` for `CNC_Settings` type
- `incs/motion/tmc5160.h:TMC5160_ApplySettings()` — New public API function declaration
- `srcs/motion/tmc5160.c:TMC5160_ApplySettings()` — Implementation: validates axis is TMC5160, builds `TMC5160_AxisConfig` from settings arrays, calls `TMC5160_ConfigAxis()`
- `srcs/gcode/gcode_parser.c:includes` — Added `#ifdef HAS_TMC5160_AXIS` guard around `#include "motion/tmc5160.h"`
- `srcs/gcode/gcode_parser.c:$n=v handler` — Added hook: when param in [200,253], calls `TMC5160_ApplySettings((E_AXIS)(param % 10), s)` immediately after flash save

**Build**: ✅ Clean Release — `bins/Release/CS23.hex` (316 KB, February 26, 2026)

---

## 🔧 Per-Axis Mixed Driver System — February 26, 2026

### Correction: enable wrappers must delegate to enable_all_set/clear
- `srcs/utils/utils.c:enable_x/y/z/a_set/clear` - Fixed: per-axis DRV8825 enable wrappers now call `enable_all_set()` / `enable_all_clear()` (the user-defined single-pin wrapper) instead of calling `EnXYZA_Set/Clear` directly. This respects the existing `enable_all_*` abstraction layer and keeps all future EN pin changes in one place.
- `incs/common.h:DRIVER_DRV8825 comment` - Updated comment to note shared `EnXYZA` pin (no per-axis EN on this PCB)
- `.github/copilot-instructions.md` - Added **🔴 CRITICAL: PCB Hardware Constraints** section at top documenting the single shared `EnXYZA` (RE6) enable pin, the absence of `EnX/EnY/EnZ/EnA` macros, and the per-axis driver define system — so this is never accidentally broken again

Replaced the global `STEPPER_DRIVER_TMC5160` / `STEPPER_DRIVER_LEGACY` compile switch with a
fully configurable per-axis driver assignment system supporting mixed TMC5160 + DRV8825 on the
same board (e.g. 2+2 or 3+1). All selection is compile-time; zero runtime overhead.

- `incs/common.h:17-60` - **Replaced** global driver `#define` with:
  - `DRIVER_TMC5160 1` / `DRIVER_DRV8825 2` — driver type tokens
  - `AXIS_X_DRIVER` / `AXIS_Y_DRIVER` / `AXIS_Z_DRIVER` / `AXIS_A_DRIVER` — per-axis assignment (edit to match wiring)
  - `HAS_TMC5160_AXIS` — auto-derived; defined if ≥1 axis is TMC5160; gates all SPI code
  - `HAS_DRV8825_AXIS` — auto-derived; defined if ≥1 axis is DRV8825; gates per-axis enable arrays
  - `TMC5160_AXIS_MASK` — compile-time bitmask (bit N = axis N is TMC5160); used by mixed-driver enable inlines
- `incs/motion/tmc5160.h:22,119` - Changed `#ifdef STEPPER_DRIVER_TMC5160` guard → `#ifdef HAS_TMC5160_AXIS`
- `srcs/motion/tmc5160.c:17` - Changed file-level guard → `#ifdef HAS_TMC5160_AXIS`
- `srcs/motion/tmc5160.c:TMC5160_Initialize()` - Replaced 4-axis loop with per-axis `#if (AXIS_x_DRIVER == DRIVER_TMC5160)` blocks; CS deassert and ConfigAxis only called for assigned TMC5160 axes
- `srcs/motion/tmc5160.c:TMC5160_Tasks()` - Replaced 4-axis loop with per-axis `#if` blocks using `POLL_TMC_AXIS()` local macro; DRV8825 axes never polled over SPI
- `srcs/motion/tmc5160.c:268` - Updated closing `#endif` comment
- `incs/utils/utils.h:27-31` - Changed enable extern guard `#ifndef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_DRV8825_AXIS`
- `incs/utils/utils.h:STEPPERS_Enable/Disable` - **Replaced** binary TMC5160/legacy block with mixed-driver implementation: TMC5160 block (`#ifdef HAS_TMC5160_AXIS`) + DRV8825 per-axis `#if` blocks (`#ifdef HAS_DRV8825_AXIS`)
- `incs/utils/utils.h:AXIS_EnableSet/Clear` - **Replaced** with three-way `#if defined(HAS_TMC5160_AXIS) && defined(HAS_DRV8825_AXIS)` / `elif TMC5160` / `else DRV8825`; mixed path uses `TMC5160_AXIS_MASK` bitmask for single-compare runtime dispatch (compiler dead-strips one branch for pure configs)
- `srcs/utils/utils.c:enable wrappers` - Changed guard `#ifndef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_DRV8825_AXIS`
- `srcs/utils/utils.c:axis_enable_set[] arrays` - Changed guard `#ifndef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_DRV8825_AXIS`; full 4-entry arrays always compiled when DRV8825 present (TMC5160 entries unreachable via bitmask)
- `srcs/app.c:APP_CONFIG` - Changed `#ifdef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_TMC5160_AXIS`
- `srcs/app.c:APP_IDLE` - Changed `#ifdef STEPPER_DRIVER_TMC5160` → `#ifdef HAS_TMC5160_AXIS`

**Example configurations** (edit `AXIS_x_DRIVER` lines in `incs/common.h`):
```
X+Y=TMC5160, Z+A=DRV8825:  AXIS_X=TMC5160 AXIS_Y=TMC5160 AXIS_Z=DRV8825 AXIS_A=DRV8825
X+Y+Z=TMC5160, A=DRV8825:  AXIS_X=TMC5160 AXIS_Y=TMC5160 AXIS_Z=TMC5160 AXIS_A=DRV8825
All TMC5160:                all four = DRIVER_TMC5160
All DRV8825:                all four = DRIVER_DRV8825  (previous STEPPER_DRIVER_LEGACY behaviour)
```

---

## 🔧 Infrastructure Changes - February 23, 2026

### TMC5160 SPI Driver - Infrastructure Setup

- **MCC**: SPI2 peripheral added and configured (SPI Master mode)
- `srcs/config/config/default/peripheral/spi/spi_master/plib_spi2_master.c` - MCC generated SPI2 PLIB (source)
- `incs/config/config/default/peripheral/spi/spi_master/plib_spi2_master.h` - MCC generated SPI2 PLIB (header, moved to incs)
- `incs/config/config/default/peripheral/spi/spi_master/plib_spi_master_common.h` - MCC common SPI types (header, moved to incs)

### Build System Fixes

- `Makefile:124` - `build_dir` target now forwards `DRY_RUN=$(DRY_RUN)` to inner make
- `srcs/Makefile:380` - `SRC_DIRS` wildcard extended from 4 to 6 levels deep (covers `spi/spi_master/` nesting)

---

## ✅ LitePlacer GRBL Probe Implementation — February 10, 2026

G38.2/3/4/5 probe commands. Planning: [LITEPLACER_GRBL_IMPLEMENTATION.md](docs/readme/LITEPLACER_GRBL_IMPLEMENTATION.md)

---

## 🔧 Code Changes - February 10, 2026

### Phase 1: Probe Event Types (Initial Implementation)

#### File: `incs/gcode/gcode_parser.h`

**Line 35** - `GCODE_EventType enum`  
- **Added**: `GCODE_EVENT_PROBE_TOWARD` - Event type for G38.2, G38.3 probe toward commands
- **Added**: `GCODE_EVENT_PROBE_AWAY` - Event type for G38.4, G38.5 probe away commands

**Line 100** - `GCODE_Event data union`  
- **Added**: `probe` struct member with fields:
  - `float x, y, z, a` - Target coordinates for probe move
  - `float feedrate` - Probe feedrate
  - `bool alarm_on_fail` - true for G38.2/G38.4, false for G38.3/G38.5
  - `bool probe_toward` - true = toward (G38.2/G38.3), false = away (G38.4/G38.5)

### Phase 2: Parser Implementation ✅ COMPLETE

#### File: `srcs/gcode/gcode_parser.c`

**Line 418** - `parse_command_to_event()` function  
- **Added**: G38.x probe command parsing block (57 lines)
- Parses G38.2, G38.3, G38.4, G38.5 with decimal subcode detection
- Determines probe direction: toward (G38.2/G38.3) or away (G38.4/G38.5)
- Sets alarm behavior: true for G38.2/G38.4, false for G38.3/G38.5
- Parses XYZAF parameters with unit scaling (mm/inches)
- Returns `GCODE_EVENT_PROBE_TOWARD` or `GCODE_EVENT_PROBE_AWAY`
- Debug logging for probe parameters

---

## 📝 Pending Implementation

### Phase 3: Probe State Machine (80% COMPLETE ✅)
- [x] `incs/data_structures.h:145` - Add `ProbeState` enum (IDLE, MOVING, TRIGGERED, FAILED)
- [x] `incs/data_structures.h:210` - Add probe state fields to APP_DATA (probeState, probeSuccess, probeAlarmOnFail, probePosition)
- [x] `srcs/motion/motion.c:812` - `MOTION_ProcessGcodeEvent()` - Add GCODE_EVENT_PROBE_TOWARD/AWAY handling (68 lines)
  - Builds start/end coordinates from probe event data
  - Calls `KINEMATICS_LinearMove()` with probe_speed (8.33 mm/s)
  - Initializes `appData->probeState = PROBE_STATE_MOVING`
- [x] `srcs/app.c:278` - `APP_Tasks()` - Add probe trigger monitoring (70 lines)
  - Checks `LIMIT_GetMax(AXIS_Z)` during PROBE_STATE_MOVING
  - Applies $6 probe invert setting
  - On trigger: stops motion, saves position to probePosition, sets PROBE_STATE_TRIGGERED
  - On completion without trigger: sets PROBE_STATE_FAILED
- [x] `srcs/app.c:290` - Add probe result reporting for PROBE_STATE_TRIGGERED
  - Sends `[PRB:x,y,z,a:1]` and "ok"
- [x] `srcs/app.c:302` - Add probe failure handling for PROBE_STATE_FAILED
  - If probeAlarmOnFail: triggers `ALARM:5`, enters APP_ALARM state
  - Else: sends `[PRB:x,y,z,a:0]` and "ok"

### Phase 4: Hardware Configuration (100% COMPLETE ✅)
- [x] `incs/utils/utils.h:155` - Add `PROBE_Get()` inline function
  - Uses Z-max limit switch as probe input (GRBL standard)
  - Applies $6 probe invert setting (NO/NC switch configuration)
  - Returns true when probe contact made
- [x] `srcs/app.c:283` - Updated probe trigger detection to use `PROBE_Get()`
  - Replaced manual `LIMIT_GetMax(AXIS_Z)` + invert logic
  - Now uses hardware abstraction layer for cleaner code

### Phase 5: Testing (PENDING)
- [ ] Test G38.2 probe with trigger detection
- [ ] Test G38.3 probe without alarm on failure
- [ ] Verify `[PRB:...]` response format
- [ ] Test position preservation at trigger point

---

## 🏷️ Git History

### Commits

**Tag**: `v1.1h-20260210-pre-probe` - Stable build before LitePlacer probe implementation

**Commit**: `c37eca6` - "Add LitePlacer GRBL probe implementation plan and initial event types"
- Added comprehensive implementation document
- Added GCODE_EVENT_PROBE_TOWARD and GCODE_EVENT_PROBE_AWAY event types
- Added probe data structure to GCODE_Event union

---

## 📊 Implementation Progress

| Phase | Status | Description |
|-------|--------|-------------|
| Phase 1: Event Types | ✅ Complete | `GCODE_EVENT_PROBE_TOWARD/AWAY` added to parser.h |
| Phase 2: Parser | ✅ Complete | G38.2/3/4/5 parsing in gcode_parser.c |
| Phase 3: State Machine | ✅ Complete | `MOTION_ProcessGcodeEvent`, probe monitoring in APP_Tasks |
| Phase 4: Hardware Config | ✅ Complete | `PROBE_Get()` inline, Z-max as probe input, $6 invert |
| Phase 5: Testing | ⬜ Pending | Flash to dev rig and verify [PRB:...] response |

**Total Progress**: 80% (4/5 phases complete — Phase 5 hardware test pending)

---

## 📚 Reference Documents

- [LITEPLACER_GRBL_IMPLEMENTATION.md](docs/readme/LITEPLACER_GRBL_IMPLEMENTATION.md) - Complete implementation plan
- [SETTINGS_REFERENCE.md](docs/readme/SETTINGS_REFERENCE.md) - GRBL settings reference
- [ARCHITECTURE.md](docs/readme/ARCHITECTURE.md) - System architecture overview

---

**Note**: This file tracks all code changes. Planning documents are separate in `docs/readme/`. No other change tracking files should be created.
