# LitePlacer GRBL Implementation Plan

**Date**: February 10, 2026  
**Status**: Design Phase  
**Target Firmware**: Pic32mzCNC_V3 GRBL v1.1 Compatible

---

## 1. LitePlacer Requirements Analysis

### Current TinyG Implementation

From `TinyGControl.cs` analysis, LitePlacer uses these probe workflows:

#### **A. Normal Probing (G28.4) - For Part Placement**
```csharp
// TinyG sequence:
1. Write_m("{\"zsn\":0}") - Disable Z min switch
2. Write_m("{\"zsx\":1}") - Enable Z max as probe input
3. Write_m("{\"gc\":\"G28.4 Z0\"}") - Probe down to Z=0
4. EnableZswitches() - Restore normal configuration
5. Backoff by specified distance
```
**Behavior**: Resets Z position to 0 when probe triggers

#### **B. Calibration Probing (G38.2) - For Height Measurement**
```csharp
// TinyG sequence:
1. Write_m("{\"zsn\":0}") - Disable Z min switch
2. Write_m("{\"zsx\":1}") - Enable Z max as probe input  
3. Write_m("{\"gc\":\"G38.2 Z-100 F300\"}") - Probe toward workpiece
4. EnableZswitches() - Restore normal configuration
5. Read Cnc.CurrentZ for actual trigger height
```
**Behavior**: Preserves exact Z position where probe triggered

### Key Differences
- **G28.4 (TinyG)**: Homing variant, resets position to 0 after trigger
- **G38.2 (Standard GRBL)**: Probe command, preserves trigger position, reports via `[PRB:x,y,z,a:1]`

---

## 2. GRBL v1.1 Standard Probe Commands

### G38.2 - Probe Toward Workpiece (Alarm on No Contact)
```gcode
G38.2 Z-100 F300
```
- Moves Z axis down 100mm (from current position)
- Stops when probe input triggers
- If no trigger detected before Z-100 reached: **ALARM 5** (Probe fail)
- Reports trigger position: `[PRB:x.xxx,y.yyy,z.zzz,a.aaa:1]`

### G38.3 - Probe Toward Workpiece (No Alarm)
```gcode
G38.3 Z-100 F300
```
- Same as G38.2 but no alarm if probe doesn't trigger
- Reports: `[PRB:x.xxx,y.yyy,z.zzz,a.aaa:0]` (`:0` = no contact)

### G38.4 - Probe Away From Workpiece (Alarm on No Contact)
```gcode
G38.4 Z10 F300
```
- Moves away from probe until contact is lost
- Used for precise edge detection

### G38.5 - Probe Away From Workpiece (No Alarm)
```gcode
G38.5 Z10 F300
```
- Same as G38.4 but no alarm on failure

---

## 3. Implementation Strategy for Pic32mzCNC_V3

### Option A: Pure GRBL Compliance (Recommended)

**Advantages**:
- Standard protocol, works with all GRBL senders (UGS, CNCjs, bCNC)
- Clean firmware implementation
- No custom commands needed

**What LitePlacer Needs to Change**:
- Remove TinyG-specific switch configuration commands (`{\"zsn\":0}`, `{\"zsx\":1}`)
- Use GRBL settings to configure probe input permanently
- Use G38.2/G38.3 for probing operations
- Parse `[PRB:x,y,z,a:1]` response for position

**GRBL Settings for Probe**:
```gcode
$6=0     # Probe pin invert (0=active low, 1=active high)
$5=0     # Limit pins invert (configure Z-max as probe input)
```

### Option B: Hybrid Approach (More Complex)

**Add custom commands for switch configuration**:
- `$ZSN=<value>` - Configure Z min switch (0=disabled, 1-3=enabled with mode)
- `$ZSX=<value>` - Configure Z max switch (0=disabled, 1=probe, 2=limit)

**Advantages**:
- Minimal LitePlacer application changes
- Dynamic switch reconfiguration

**Disadvantages**:
- Non-standard GRBL protocol
- Breaks compatibility with standard senders
- More firmware complexity

---

## 4. Recommended Implementation (Option A)

### Firmware Changes Required

#### **4.1 Add Probe Event Types** (`incs/gcode/gcode_parser.h`)
```c
typedef enum {
    // ... existing events ...
    GCODE_EVENT_PROBE_TOWARD,         // G38.2, G38.3
    GCODE_EVENT_PROBE_AWAY,           // G38.4, G38.5
} GCODE_EventType;

typedef struct {
    CoordinatePoint target;           // Target position
    float feedrate;                   // Probe feedrate
    bool alarm_on_fail;               // true for G38.2/G38.4, false for G38.3/G38.5
    bool probe_toward;                // true = toward (G38.2/G38.3), false = away (G38.4/G38.5)
} ProbeData;

typedef union {
    // ... existing data ...
    ProbeData probe;
} GCODE_EventData;
```

#### **4.2 Parse G38.x Commands** (`srcs/gcode/gcode_parser.c`)
```c
// In parse_command_to_event():
if (strncmp(token, "G38.", 4) == 0) {
    char subcode = token[4];  // '2', '3', '4', or '5'
    
    event->type = (subcode == '2' || subcode == '3') ? 
                  GCODE_EVENT_PROBE_TOWARD : GCODE_EVENT_PROBE_AWAY;
    
    event->data.probe.alarm_on_fail = (subcode == '2' || subcode == '4');
    event->data.probe.probe_toward = (subcode == '2' || subcode == '3');
    
    // Parse XYZAF parameters
    // ... extract target position and feedrate ...
    
    return true;
}
```

#### **4.3 Implement Probe State Machine** (`srcs/app.c`)
```c
typedef enum {
    PROBE_STATE_IDLE,
    PROBE_STATE_MOVING,
    PROBE_STATE_TRIGGERED,
    PROBE_STATE_COMPLETE,
    PROBE_STATE_FAILED
} ProbeState;

// In APP_Tasks():
case GCODE_EVENT_PROBE_TOWARD:
    appData.probeState = PROBE_STATE_MOVING;
    appData.probeAlarmOnFail = event.data.probe.alarm_on_fail;
    
    // Generate motion segment
    MotionSegment segment;
    KINEMATICS_LinearMove(currentPos, targetPos, feedrate, &segment);
    
    // Add to motion queue
    // ... queue management ...
    break;

// Monitor probe input during motion
if (appData.probeState == PROBE_STATE_MOVING) {
    // Check probe pin (Z-max limit switch configured as probe)
    if (LIMIT_GetMax(AXIS_Z)) {  // Probe triggered!
        // Stop motion immediately
        STEPPER_DisableAll();
        appData.motionQueueCount = 0;
        
        // Save trigger position
        appData.probePosition.x = appData.currentX;
        appData.probePosition.y = appData.currentY;
        appData.probePosition.z = appData.currentZ;
        appData.probePosition.a = appData.currentA;
        
        appData.probeState = PROBE_STATE_TRIGGERED;
        appData.probeSuccess = true;
    }
}

// Check for probe completion
if (appData.probeState == PROBE_STATE_TRIGGERED) {
    // Send probe result
    UART_Printf("[PRB:%.3f,%.3f,%.3f,%.3f:1]\r\n",
                appData.probePosition.x,
                appData.probePosition.y,
                appData.probePosition.z,
                appData.probePosition.a);
    
    UART_SendOK();
    appData.probeState = PROBE_STATE_IDLE;
}

// Check for probe failure (target reached without trigger)
if (appData.probeState == PROBE_STATE_MOVING && 
    appData.motionQueueCount == 0 &&
    !appData.probeSuccess) {
    
    if (appData.probeAlarmOnFail) {
        // Trigger ALARM 5 (Probe fail)
        UART_Printf("ALARM:5\r\n");
        appData.state = APP_ALARM;
    } else {
        // G38.3/G38.5 - No alarm, just report failure
        UART_Printf("[PRB:%.3f,%.3f,%.3f,%.3f:0]\r\n",
                    appData.currentX, appData.currentY,
                    appData.currentZ, appData.currentA);
        UART_SendOK();
    }
    appData.probeState = PROBE_STATE_IDLE;
}
```

#### **4.4 Hardware Configuration**
```c
// Configure Z-max limit switch as probe input
// In utils.c GPIO initialization:

// Z-max pin becomes probe input (shared hardware)
// When $6 probe invert setting is applied, this affects Z-max reading
static inline bool PROBE_Get(void) {
    bool pin_state = LIMIT_GetMax(AXIS_Z);  // Read Z-max pin
    uint8_t probe_invert = *(g_grbl_settings.probe_invert);
    return (pin_state ^ probe_invert);  // Apply inversion if needed
}
```

---

## 5. LitePlacer Application Changes (grbl.c)

### Create `GRBLControl.cs` (New File)

**Purpose**: GRBL firmware communication layer (parallel to TinyGControl.cs and SKR3Control.cs)

#### **Key Methods to Implement**:

```csharp
public class GRBLControl
{
    // Constructor
    public GRBLControl(FormMain MainF, CNC cnc, SerialComm com)
    {
        MainForm = MainF;
        Cnc = cnc;
        Com = com;
    }

    // === Probing Methods ===
    
    // Standard probe for calibration (preserves trigger position)
    public bool Nozzle_ProbeDown_Calibration()
    {
        MainForm.DisplayText("Calibration probe (G38.2), GRBL");
        
        // No need to disable switches - GRBL uses settings
        // G38.2: Probe toward, alarm on no contact
        // Z-100: Maximum probe distance
        // F300: Probe feedrate (5mm/s)
        if (!Write_m("G38.2 Z-100 F300", RegularMoveTimeout))
        {
            MainForm.DisplayText("*** Calibration probe failed", KnownColor.DarkRed);
            return false;
        }
        
        // Parse probe result: [PRB:x,y,z,a:1]
        string response = GetResponse_m();
        if (response.StartsWith("[PRB:"))
        {
            // Extract Z position from probe report
            double probeZ = ParseProbeZ(response);
            Cnc.SetCurrentZ(probeZ);
            MainForm.DisplayText("Probe triggered at Z=" + probeZ.ToString("0.000"));
            return true;
        }
        
        return false;
    }
    
    // Normal probe with backoff (for part placement)
    public bool Nozzle_ProbeDown(double backoff)
    {
        MainForm.DisplayText("Probing, GRBL");
        
        // G38.3: Probe toward, NO alarm on no contact (safer for production)
        if (!Write_m("G38.3 Z-100 F300", RegularMoveTimeout))
        {
            return false;
        }
        
        // Parse probe result
        string response = GetResponse_m();
        if (response.StartsWith("[PRB:") && response.Contains(":1]"))
        {
            // Probe triggered successfully
            double probeZ = ParseProbeZ(response);
            
            // Backoff to safe height
            if (!CNC_Z_m(probeZ + backoff))
            {
                return false;
            }
            return true;
        }
        
        // Probe didn't trigger - handle gracefully
        MainForm.DisplayText("Probe did not trigger", KnownColor.Orange);
        return false;
    }
    
    // === Position Setting ===
    
    public void SetPosition(string X, string Y, string Z, string A)
    {
        // GRBL uses G92 (same as SKR3)
        string cmd = "G92";
        if (X != "") cmd += " X" + X;
        if (Y != "") cmd += " Y" + Y;
        if (Z != "") cmd += " Z" + Z;
        if (A != "") cmd += " A" + A;
        
        Write_m(cmd);
        
        // Update internal position tracking
        if (X != "") { double.TryParse(X, out double x); Cnc.SetCurrentX(x); }
        if (Y != "") { double.TryParse(Y, out double y); Cnc.SetCurrentY(y); }
        if (Z != "") { double.TryParse(Z, out double z); Cnc.SetCurrentZ(z); }
        if (A != "") { double.TryParse(A, out double a); Cnc.SetCurrentA(a); }
    }
    
    // === Communication Helpers ===
    
    public bool Write_m(string command, int timeout = 1000)
    {
        Com.Write(command + "\n");
        return WaitForOk(timeout);
    }
    
    public string GetResponse_m()
    {
        // Wait for response line
        while (!Cnc.LineAvailable)
        {
            Thread.Sleep(1);
        }
        return Cnc.ReadLine();
    }
    
    private bool WaitForOk(int timeout)
    {
        DateTime start = DateTime.Now;
        while ((DateTime.Now - start).TotalMilliseconds < timeout)
        {
            if (Cnc.LineAvailable)
            {
                string line = Cnc.ReadLine();
                if (line == "ok") return true;
                if (line.StartsWith("error:") || line.StartsWith("ALARM:"))
                {
                    MainForm.DisplayText("GRBL Error: " + line, KnownColor.DarkRed);
                    return false;
                }
            }
            Thread.Sleep(1);
        }
        MainForm.DisplayText("GRBL timeout", KnownColor.DarkRed);
        return false;
    }
    
    private double ParseProbeZ(string probeReport)
    {
        // Parse: [PRB:10.000,20.000,43.567,0.000:1]
        // Extract Z value (3rd coordinate)
        string[] parts = probeReport.Split(':');
        if (parts.Length < 2) return 0;
        
        string coords = parts[1];  // "10.000,20.000,43.567,0.000"
        string[] values = coords.Split(',');
        if (values.Length >= 3)
        {
            double.TryParse(values[2], out double z);
            return z;
        }
        return 0;
    }
    
    // === Line Reception Handler ===
    
    public void LineReceived(string line)
    {
        // Handle status reports: <Idle|MPos:x,y,z,a|...>
        if (line.StartsWith("<") && line.EndsWith(">"))
        {
            ParseStatusReport(line);
            return;
        }
        
        // Handle probe reports
        if (line.StartsWith("[PRB:"))
        {
            // Store probe result
            return;
        }
        
        // Handle ok/error responses
        if (line == "ok" || line.StartsWith("error:") || line.StartsWith("ALARM:"))
        {
            // Trigger ReadyEvent
            return;
        }
    }
    
    private void ParseStatusReport(string status)
    {
        // Parse: <Idle|MPos:10.000,20.000,30.000,0.000|...>
        if (status.Contains("MPos:"))
        {
            int start = status.IndexOf("MPos:") + 5;
            int end = status.IndexOf('|', start);
            if (end < 0) end = status.IndexOf('>', start);
            
            string coords = status.Substring(start, end - start);
            string[] values = coords.Split(',');
            
            if (values.Length >= 4)
            {
                double.TryParse(values[0], out double x);
                double.TryParse(values[1], out double y);
                double.TryParse(values[2], out double z);
                double.TryParse(values[3], out double a);
                
                Cnc.SetCurrentX(x);
                Cnc.SetCurrentY(y);
                Cnc.SetCurrentZ(z);
                Cnc.SetCurrentA(a);
            }
        }
    }
}
```

### Modify `CNC.cs` to Support GRBL

**Add GRBL board type** (if not already present):
```csharp
public enum ControlBoardType
{
    unknown,
    TinyG,
    SKR3,
    GRBL  // <-- Add this
}
```

**Add GRBL instance**:
```csharp
public class CNC
{
    private TinyGclass TinyG;
    public SKR3class SKR3;
    public GRBLclass GRBL;  // <-- Add this
    
    public CNC(FormMain MainF)
    {
        // ...
        TinyG = new TinyGclass(MainForm, this, Com);
        SKR3 = new SKR3class(MainForm, this, Com);
        GRBL = new GRBLclass(MainForm, this, Com);  // <-- Add this
    }
}
```

**Update probe method routing**:
```csharp
public bool ProbeDown_Calibration()
{
    if (ErrorState)
    {
        MainForm.DisplayText("*** Cnc.ProbeDown_Calibration(), board in error state.", KnownColor.DarkRed);
        return false;
    }

    switch (MainForm.Setting.Controlboard)
    {
        case FormMain.ControlBoardType.GRBL:
            return GRBL.Nozzle_ProbeDown_Calibration();
            
        case FormMain.ControlBoardType.SKR3:
            // SKR3 uses grblHAL which is GRBL-compatible
            // Can share the GRBL implementation
            return GRBL.Nozzle_ProbeDown_Calibration();
            
        case FormMain.ControlBoardType.TinyG:
            return TinyG.Nozzle_ProbeDown_Calibration();
            
        default:
            MainForm.DisplayText("*** Unknown board type", KnownColor.DarkRed);
            return false;
    }
}
```

---

## 6. Testing Plan

### Phase 1: Firmware Implementation
1. ✅ Add probe event types to parser
2. ✅ Implement G38.2/G38.3/G38.4/G38.5 parsing
3. ✅ Add probe state machine to APP_Tasks
4. ✅ Configure Z-max as probe input
5. ✅ Test probe trigger detection with LED indicator
6. ✅ Test probe report format: `[PRB:x,y,z,a:1]`

### Phase 2: LitePlacer Integration
1. ✅ Create GRBLControl.cs
2. ✅ Implement Nozzle_ProbeDown_Calibration()
3. ✅ Implement probe report parsing
4. ✅ Update CNC.cs routing
5. ✅ Test with manual probe commands via terminal

### Phase 3: Validation
1. ✅ Calibration workflow: Probe triggers, Z position preserved
2. ✅ Normal probing: Probe triggers, backoff distance applied
3. ✅ Probe failure handling: No alarm for G38.3, alarm for G38.2
4. ✅ Position accuracy: Verify reported Z matches actual trigger height

---

## 7. Configuration Guide for Users

### Firmware Settings
```gcode
$6=0     # Probe pin invert (0=active low, 1=active high)
$5=0     # Limit pins invert mask
$22=1    # Homing enable
$27=2.0  # Homing pull-off distance (mm)
```

### Hardware Setup
- **Probe Input**: Connect Z-axis max limit switch to probe input pin
- **Active Low**: Switch closes to GND when triggered (set $6=0)
- **Active High**: Switch closes to +5V when triggered (set $6=1)

### LitePlacer Settings
- **Board Type**: Select "GRBL" in LitePlacer settings
- **COM Port**: Same as TinyG/SKR3
- **Baud Rate**: 115200 (standard GRBL)

---

## 8. Summary

### Firmware Changes
- **4 new G-code commands**: G38.2, G38.3, G38.4, G38.5
- **Probe state machine**: Monitor probe input during motion
- **Probe reporting**: `[PRB:x,y,z,a:1]` format (GRBL standard)
- **Alarm handling**: ALARM:5 for probe failure (G38.2/G38.4)

### LitePlacer Changes
- **New file**: GRBLControl.cs (parallel to TinyGControl.cs)
- **Probe methods**: Nozzle_ProbeDown(), Nozzle_ProbeDown_Calibration()
- **Response parsing**: Extract Z from `[PRB:...]` messages
- **CNC.cs routing**: Add GRBL board type and method routing

### Benefits
- ✅ Standard GRBL protocol (no custom commands)
- ✅ Works with any GRBL sender (UGS, CNCjs, bCNC)
- ✅ Clean firmware implementation
- ✅ Minimal LitePlacer application changes
- ✅ Maintains compatibility with TinyG workflow

---

**Next Steps**: Begin firmware implementation starting with Phase 1 (probe event types and parser).
