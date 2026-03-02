# CNC Controller Settings & Commands Reference

**Firmware:** Pic32mzCNC_V3  
**GRBL Version:** v1.1 Compatible  
**Last Updated:** February 26, 2026

---

## Quick Reference

### System Commands

| Command | Description | Example |
|---------|-------------|---------|
| `$$` | View all settings | `$$` |
| `$X=value` | Set parameter X to value | `$100=40.0` |
| `$#` | View G-code parameters (work offsets) | `$#` |
| `$G` | View G-code parser state | `$G` |
| `$I` | View build info | `$I` |
| `$N` | View startup blocks | `$N` |
| `$H` | Run homing cycle | `$H` |
| `$RST=$` | Restore default settings | `$RST=$` |
| `$SAVE` | Explicitly save current settings to flash | `$SAVE` |
| `?` | Status report | `?` |
| `!` | Feed hold (pause) | `!` |
| `~` | Cycle start/resume | `~` |
| `Ctrl+X` | Soft reset | (0x18) |

---

## Settings Parameters

### Step Configuration ($0-$5)

| Parameter | Description | Default | Units | Range |
|-----------|-------------|---------|-------|-------|
| `$0` | Step pulse time | `10` | µs | 1-255 |
| `$1` | Step idle delay | `25` | ms | 0-255 |
| `$2` | Step pulse invert mask | `0` | mask | 0-255 |
| `$3` | Step direction invert mask | `0` | mask | 0-255 |
| `$4` | Step enable invert | `0` | bool | 0-1 |
| `$5` | Limit pins invert | `0` | bool | 0-1 |

**Notes:**
- `$0`: Pulse width sent to stepper drivers (typically 3-10µs)
- `$2`: Bit mask to invert step pulses (bit 0=X, 1=Y, 2=Z, 3=A)
- `$3`: Bit mask to invert direction signals
- `$4`: Invert enable pin logic (0=active low, 1=active high)
- `$5`: Invert limit switch inputs

---

### Arc Configuration ($12-$13)

| Parameter | Description | Default | Units | Range |
|-----------|-------------|---------|-------|-------|
| `$12` | Arc segment length | `0.5` | mm | 0.001-10.0 |
| `$13` | Arc radius tolerance | `0.002` | mm | 0.0001-1.0 |

**Notes:**
- `$12`: Smaller values = smoother arcs but slower execution; larger values = faster but more faceted
- `$13`: Maximum allowed difference between start and end radius of an arc. If exceeded, controller triggers an alarm. GRBL v1.1 default is 0.002mm (2 microns). Increase slightly if CAM software produces arcs with minor radius mismatch.

---

### Spindle Configuration ($30-$31)

| Parameter | Description | Default | Units | Range |
|-----------|-------------|---------|-------|-------|
| `$30` | Spindle max RPM | `24000` | RPM | 0-50000 |
| `$31` | Spindle min RPM | `0` | RPM | 0-50000 |

**Notes:**
- Used for S-value PWM scaling
- S-word range maps to min/max RPM

---

### Homing Configuration ($22-$27)

| Parameter | Description | Default | Units | Range |
|-----------|-------------|---------|-------|-------|
| `$22` | Homing enable | `1` | bool | 0-1 |
| `$23` | Homing direction invert mask | `0` | mask | 0-255 |
| `$24` | Homing feed rate | `100.0` | mm/min | 1-10000 |
| `$25` | Homing seek rate | `500.0` | mm/min | 1-10000 |
| `$26` | Homing debounce | `25` | ms | 0-1000 |
| `$27` | Homing pull-off | `2.0` | mm | 0-100 |

**Notes:**
- `$22`: Enable/disable $H homing cycle
- `$23`: Bit mask for homing direction per axis
- `$24`: Slow precise approach speed
- `$25`: Fast initial search speed
- `$26`: Limit switch debounce time
- `$27`: Retract distance after homing

---

### Steps Per mm ($100-$103)

| Parameter | Description | Default | Units | Formula |
|-----------|-------------|---------|-------|---------|
| `$100` | X-axis steps/mm | `40.0` | steps/mm | (motor_steps × microsteps) / mm_per_rev |
| `$101` | Y-axis steps/mm | `40.0` | steps/mm | Same as above |
| `$102` | Z-axis steps/mm | `40.0` | steps/mm | Same as above |
| `$103` | A-axis steps/mm | `40.0` | steps/mm | Same as above |

**Calculation Examples:**

**Belt Drive (GT2, 20 tooth pulley):**
```
mm_per_rev = pulley_teeth × belt_pitch
           = 20 teeth × 2mm = 40mm

steps_per_mm = (200 steps/rev × 8 microsteps) / 40mm
             = 1600 / 40
             = 40 steps/mm
```

**Lead Screw (5mm pitch):**
```
steps_per_mm = (200 steps/rev × 8 microsteps) / 5mm
             = 1600 / 5
             = 320 steps/mm
```

**Common Microstepping Values:**
- 1× (full step): 200 steps/rev
- 2× (half step): 400 steps/rev
- 4×: 800 steps/rev
- 8×: 1600 steps/rev
- 16×: 3200 steps/rev
- 32×: 6400 steps/rev
- 256×: 51200 steps/rev

---

### Max Rates ($110-$113)

| Parameter | Description | Default | Units | Range |
|-----------|-------------|---------|-------|-------|
| `$110` | X-axis max rate | `5000.0` | mm/min | 1-50000 |
| `$111` | Y-axis max rate | `5000.0` | mm/min | 1-50000 |
| `$112` | Z-axis max rate | `2000.0` | mm/min | 1-50000 |
| `$113` | A-axis max rate | `5000.0` | mm/min | 1-50000 |

**Notes:**
- Maximum velocity for rapids (G0) and maximum feedrate
- Limited by stepper motor torque and mechanical constraints
- Z-axis typically slower due to vertical load

---

### Acceleration ($120-$123)

| Parameter | Description | Default | Units | Range |
|-----------|-------------|---------|-------|-------|
| `$120` | X-axis acceleration | `500.0` | mm/sec² | 1-10000 |
| `$121` | Y-axis acceleration | `500.0` | mm/sec² | 1-10000 |
| `$122` | Z-axis acceleration | `200.0` | mm/sec² | 1-10000 |
| `$123` | A-axis acceleration | `500.0` | mm/sec² | 1-10000 |

**Notes:**
- Controls speed ramping (trapezoidal velocity profile)
- Higher values = faster acceleration but more risk of stalling
- Z-axis typically lower due to vertical load
- Units are mm per second squared

---

### Max Travel ($130-$132)

| Parameter | Description | Default | Units | Range |
|-----------|-------------|---------|-------|-------|
| `$130` | X-axis max travel | `300.0` | mm | 0-10000 |
| `$131` | Y-axis max travel | `300.0` | mm | 0-10000 |
| `$132` | Z-axis max travel | `100.0` | mm | 0-10000 |

**Notes:**
- Defines work envelope size
- Used for soft limit checking
- Measured from homed position (machine zero)

---

### TMC5160 Driver Settings ($200-$253)

> **Note:** These settings are only active when the firmware is built with `HAS_TMC5160_AXIS` defined (i.e. at least one axis uses a TMC5160 driver). Settings take effect immediately via SPI — no reboot required.

#### Chopper Mode ($200-$203)

| Parameter | Axis | Description | Default |
|-----------|------|-------------|---------|
| `$200` | X | Chopper mode | `1` (StealthChop) |
| `$201` | Y | Chopper mode | `1` (StealthChop) |
| `$202` | Z | Chopper mode | `1` (StealthChop) |
| `$203` | A | Chopper mode | `1` (StealthChop) |

| Value | Mode | Description |
|-------|------|-------------|
| `1` | StealthChop | Silent, low-speed operation |
| `2` | SpreadCycle | High-performance, higher noise |
| `3` | Mixed | StealthChop below `$24x` threshold, SpreadCycle above |
| `4` | CoolStep | Adaptive current scaling |

#### Run Current ($210-$213)

| Parameter | Axis | Description | Default | Range |
|-----------|------|-------------|---------|-------|
| `$210` | X | Run current (0–31 scale, ~0–100% of Vref) | `20` (~63%) | 0-31 |
| `$211` | Y | Run current | `20` | 0-31 |
| `$212` | Z | Run current | `20` | 0-31 |
| `$213` | A | Run current | `20` | 0-31 |

#### Hold Current ($220-$223)

| Parameter | Axis | Description | Default | Range |
|-----------|------|-------------|---------|-------|
| `$220` | X | Hold current (motor idle) | `10` | 0-31 |
| `$221` | Y | Hold current | `10` | 0-31 |
| `$222` | Z | Hold current | `10` | 0-31 |
| `$223` | A | Hold current | `10` | 0-31 |

#### Microstep Resolution ($230-$233)

| Parameter | Axis | Description | Default | Range |
|-----------|------|-------------|---------|-------|
| `$230` | X | MRES register value | `4` (16µ) | 0-8 |
| `$231` | Y | MRES register value | `4` (16µ) | 0-8 |
| `$232` | Z | MRES register value | `4` (16µ) | 0-8 |
| `$233` | A | MRES register value | `4` (16µ) | 0-8 |

| Value | Microsteps |
|-------|------------|
| `0` | 256 |
| `1` | 128 |
| `2` | 64 |
| `3` | 32 |
| `4` | 16 (default) |
| `5` | 8 |
| `6` | 4 |
| `7` | 2 |
| `8` | 1 (full step) |

> **Important:** When changing `$230-$233`, also update `$100-$103` (steps/mm) to match the new microstep count.

#### StealthChop→SpreadCycle Threshold ($240-$243)

| Parameter | Axis | Description | Default |
|-----------|------|-------------|---------|
| `$240` | X | TPWMTHRS — velocity threshold for mode crossover (Mixed mode only) | `500` |
| `$241` | Y | TPWMTHRS | `500` |
| `$242` | Z | TPWMTHRS | `500` |
| `$243` | A | TPWMTHRS | `500` |

**Note:** Only active when chopper mode is `3` (Mixed). Higher value = switches to SpreadCycle earlier.

#### CoolStep Lower Threshold ($250-$253)

| Parameter | Axis | Description | Default |
|-----------|------|-------------|---------|
| `$250` | X | TCOOLTHRS — lower velocity threshold for CoolStep | `0` (disabled) |
| `$251` | Y | TCOOLTHRS | `0` |
| `$252` | Z | TCOOLTHRS | `0` |
| `$253` | A | TCOOLTHRS | `0` |

---

## Usage Examples

### View Current Settings
```gcode
$$
```
Output:
```
$0=10
$1=25
$2=0
$3=0
...
$132=100.000
ok
```

### Change Settings
```gcode
$100=80.0          # Set X steps/mm to 80
$110=10000         # Set X max rate to 10000 mm/min
$120=1000          # Set X acceleration to 1000 mm/sec²
```

### Save and Restore
```gcode
$RST=$             # Restore factory defaults
```

**Note:** Settings are automatically saved to NVM flash when changed.

---

## Real-Time Commands

These commands execute immediately without waiting for motion buffer:

| Character | Action | Description |
|-----------|--------|-------------|
| `?` | Status Query | Machine state, position, feedrate |
| `!` | Feed Hold | Graceful drain: current segment completes (Hold:1), then hardware stops (Hold:0) |
| `~` | Cycle Start | Resume from feed hold |
| `Ctrl+X` (0x18) | Soft Reset | Emergency stop, reset controller |

**Status Report Format:**
```
<Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|FS:0,0>
```
- State: Idle, Run, Hold, Alarm, Home
- MPos: Machine position (absolute from home)
- WPos: Work position (with offsets applied)
- FS: Current feedrate, spindle speed

---

## System Information Commands

### Build Info (`$I`)
```
[VER: 1.1h.20251102:]
[OPT: VHM,35,1024,4]
ok
```

### G-code Parser State (`$G`)
Shows active modal commands (G90/G91, G54-G59, etc.)

### Work Coordinate Offsets (`$#`)
Shows all work coordinate systems (G54-G59) and G92 offset

---

## Settings Storage

**Location:** NVM Flash at `0xBD1F0000` (64KB before bootloader)  
**Size:** 16KB page  
**Persistence:** Settings survive power cycles  
**Validation:** CRC32 checksum on every read/write  

**Safety:**
- Settings area is 64KB before MikroE bootloader
- Bootloader region (0xBD1F4000) is protected
- Invalid checksum triggers default settings restore

---

## Troubleshooting

### Settings Won't Save
- Check NVM flash is not write-protected
- Verify CRC32 validation passes
- Try `$RST=$` to restore defaults first

### Motion Doesn't Match Expected Speed
- Verify `$100-$103` steps/mm calculated correctly
- Check `$110-$113` max rates aren't too low
- Ensure microstepping matches driver configuration

### Homing Fails
- Check `$22` homing enable = 1
- Verify limit switches connected and working
- Adjust `$25` seek rate if too fast
- Increase `$26` debounce if switches are noisy
- Check `$23` direction mask if homing wrong way

---

## Default Settings Summary

| Group | Parameters | Purpose |
|-------|------------|---------|
| Step Config | $0-$5 | Pulse timing and inversion |
| Arc | $12-$13 | Arc interpolation resolution and radius tolerance |
| Spindle | $30-$31 | RPM range |
| Homing | $22-$27 | Homing cycle behavior |
| Steps/mm | $100-$103 | Motion calibration |
| Max Rates | $110-$113 | Speed limits |
| Acceleration | $120-$123 | Motion dynamics |
| Max Travel | $130-$132 | Soft limits |
| TMC5160 Mode | $200-$203 | Chopper mode per axis |
| TMC5160 Run I | $210-$213 | Run current per axis |
| TMC5160 Hold I | $220-$223 | Hold current per axis |
| TMC5160 MRES | $230-$233 | Microstep resolution per axis |
| TMC5160 TPWM | $240-$243 | StealthChop crossover threshold per axis |
| TMC5160 TCOOL | $250-$253 | CoolStep lower threshold per axis |

**Total:** 31 core parameters + 24 TMC5160 parameters (when `HAS_TMC5160_AXIS` enabled)

---

## See Also

- `README.md` - General project documentation
- `docs/` - Additional technical documentation
- GRBL v1.1 Documentation: https://github.com/gnea/grbl/wiki

---

**Firmware Build:** `1.1h.20251102`  
**Hardware:** PIC32MZ2048EFH100 @ 200MHz
