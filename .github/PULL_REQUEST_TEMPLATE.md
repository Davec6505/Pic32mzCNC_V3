## Description

<!-- What does this PR do? Why is it needed? Link the relevant issue if one exists. -->

Closes #

---

## Type of Change

- [ ] Bug fix (non-breaking change that fixes an issue)
- [ ] New feature (non-breaking change that adds functionality)
- [ ] Breaking change (fix or feature that changes existing behaviour)
- [ ] Documentation / comment update only
- [ ] Settings struct change (requires `SETTINGS_VERSION` bump)
- [ ] ISR / timing change (requires extended hardware stress test)

---

## Hardware Test Results

<!-- PRs that touch motion code MUST include real hardware test results. -->
<!-- Run all tests in UGS pipelined streaming mode unless noted otherwise. -->

| Test file | Result | Notes |
|-----------|--------|-------|
| `tests/02_rectangle_path.gcode` | ✅ / ❌ / ⏭️ N/A | |
| `tests/03_circle_20segments.gcode` | ✅ / ❌ / ⏭️ N/A | |
| `tests/05_three_arcs_simple.gcode` | ✅ / ❌ / ⏭️ N/A | |
| `tests/08_arc_cw_ccw_stress.gcode` | ✅ / ❌ / ⏭️ N/A | |
| `tests/09_concentric_semicircles.gcode` | ✅ / ❌ / ⏭️ N/A | |
| `tests/07_complex_long_run_fast.gcode` | ✅ / ❌ / ⏭️ N/A | |

Return-to-origin error (mm):

Motor driver used during testing:

---

## Checklist

- [ ] Tested on **real hardware** (not simulation only)
- [ ] All applicable test G-code files **PASS** (return to origin ≤ 0.05 mm)
- [ ] `STATUS.md` updated with file, line, function, and description of every change
- [ ] `README.md` updated if ISR architecture, call hierarchy, or timing changed
- [ ] `docs/CNC_FIRMWARE_BOOK.md` updated if algorithm or architecture changed
- [ ] `SETTINGS_VERSION` bumped if `CNC_Settings` struct fields were added/removed/reordered
- [ ] `docs/readme/SETTINGS_REFERENCE.md` updated if new `$` parameters were added
- [ ] No raw `UART3_Write` / `printf` debug calls left in production paths
- [ ] No new compiler warnings introduced (`make` output is clean)
- [ ] Branch is rebased on latest `scurve_motion` (no merge commits from upstream)

---

## Notes for Reviewers

<!-- Anything the reviewer should pay special attention to — tricky logic, timing assumptions, 
     register sequences, etc. -->
