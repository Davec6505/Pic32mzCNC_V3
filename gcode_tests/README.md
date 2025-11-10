# G-code Test Files

Test files for CNC controller motion validation.

## Test Files

### 01_simple_square.gcode
- **Purpose:** Basic linear motion test
- **Features:** 
  - Two 20mm x 20mm squares
  - Tests G0 (rapid), G1 (linear), F (feedrate)
  - Validates position tracking and absolute positioning
- **Expected Motion:** Two squares side-by-side, 30mm apart
- **Duration:** ~10 seconds at F500

### 02_simple_arc.gcode
- **Purpose:** Basic arc interpolation test
- **Features:**
  - Quarter circles (G2 and G3)
  - Semicircle
  - Tests I/J offset parameters
- **Expected Motion:** Three arc segments in sequence
- **Duration:** ~8 seconds at F300

### 03_rounded_rectangle.gcode
- **Purpose:** Continuous motion with line-to-arc transitions
- **Features:**
  - 40mm x 20mm rectangle
  - 5mm radius corners
  - Tests tangential transitions between G1 and G2
- **Expected Motion:** Smooth rounded rectangle, no jerking at corners
- **Duration:** ~15 seconds at F400

### 04_tangential_arcs.gcode
- **Purpose:** Complex arc-to-arc transitions
- **Features:**
  - S-curve (two tangential arcs)
  - Figure-8 pattern (continuous circular arcs)
  - Tests smooth velocity transitions
- **Expected Motion:** Smooth curves, no stops between arcs
- **Duration:** ~20 seconds at F350

## Testing Procedure

1. **Connect via UGS** (Universal G-code Sender)
2. **Home the machine:** Send `$H` command
3. **Set work origin:** Send `G92 X0 Y0 Z0` at desired start position
4. **Load test file** in UGS
5. **Run test** and observe motion
6. **Verify position:** Send `?` to check final position matches expected

## Expected Behavior

- **No alarms** during execution
- **Smooth motion** - no jerking or stops mid-arc
- **Accurate positioning** - returns to origin within 0.1mm
- **LED indicators:**
  - LED2 rapid blink during motion (step pulses)
  - LED2 slow heartbeat when idle

## Safety Notes

- Start with **low feedrates** (F300-F500) for initial testing
- Verify work area is clear and machine can complete full motion
- Keep emergency stop ready ($X or Ctrl+X for soft reset)
- Monitor position via `?` status queries during motion

## Settings to Verify

Check these settings before running tests:
```
$100-$103  (steps_per_mm)  - Set correctly for your machine
$110-$113  (max_rate)      - Conservative values for testing
$120-$123  (acceleration)  - Start low, increase after validation
$12        (arc_tolerance) - Default 0.1mm is good for testing
```

## Troubleshooting

**Machine doesn't move:**
- Check `$22` (homing_enable) if using $H
- Verify motor enable pins are active
- Check status with `?` for alarm states

**Arcs are faceted/segmented:**
- Adjust `$12` (mm_per_arc_segment) - smaller = smoother
- Check feedrate isn't too high for segment generation

**Position drift:**
- Verify steps_per_mm settings ($100-$103)
- Check for mechanical binding or missed steps
- Monitor stepper temperature
