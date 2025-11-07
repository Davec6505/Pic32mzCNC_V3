# Test script for arc motion (G2/G3) verification
# Tests circular interpolation with various arc configurations

param(
    [string]$Port = "COM4",
    [int]$BaudRate = 115200,
    [int]$DelayMs = 100
)

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "Arc Motion Test Suite (G2/G3)" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

# Open serial port
$serialPort = New-Object System.IO.Ports.SerialPort
$serialPort.PortName = $Port
$serialPort.BaudRate = $BaudRate
$serialPort.DataBits = 8
$serialPort.Parity = [System.IO.Ports.Parity]::None
$serialPort.StopBits = [System.IO.Ports.StopBits]::One
$serialPort.Handshake = [System.IO.Ports.Handshake]::None
$serialPort.ReadTimeout = 1000
$serialPort.WriteTimeout = 1000

try {
    $serialPort.Open()
    Write-Host "✓ Connected to $Port @ $BaudRate baud`n" -ForegroundColor Green
    Start-Sleep -Milliseconds 500
    
    # Clear any startup messages
    while ($serialPort.BytesToRead -gt 0) {
        $null = $serialPort.ReadExisting()
    }
}
catch {
    Write-Host "✗ Failed to open $Port : $_" -ForegroundColor Red
    exit 1
}

function Send-GCode {
    param([string]$Command, [string]$Description = "")
    
    Write-Host "→ " -NoNewline -ForegroundColor Yellow
    Write-Host $Command -NoNewline -ForegroundColor White
    if ($Description) {
        Write-Host " ($Description)" -NoNewline -ForegroundColor Gray
    }
    
    $serialPort.WriteLine($Command)
    Start-Sleep -Milliseconds $DelayMs
    
    # Read response
    $response = ""
    $timeout = 0
    while ($serialPort.BytesToRead -eq 0 -and $timeout -lt 20) {
        Start-Sleep -Milliseconds 50
        $timeout++
    }
    
    if ($serialPort.BytesToRead -gt 0) {
        $response = $serialPort.ReadExisting()
        if ($response -match "ok") {
            Write-Host " ✓" -ForegroundColor Green
        } elseif ($response -match "error") {
            Write-Host " ✗ ERROR" -ForegroundColor Red
            Write-Host "   Response: $response" -ForegroundColor Red
        } else {
            Write-Host ""
            Write-Host "   $response" -ForegroundColor Cyan
        }
    } else {
        Write-Host " (no response)" -ForegroundColor Yellow
    }
    
    return $response
}

function Get-MachineStatus {
    $serialPort.WriteLine("?")
    Start-Sleep -Milliseconds 50
    
    if ($serialPort.BytesToRead -gt 0) {
        $statusResponse = $serialPort.ReadExisting()
        return $statusResponse
    }
    return ""
}

function Monitor-Motion {
    param([string]$TestName, [int]$DurationSeconds = 5)
    
    Write-Host "`n  Monitoring motion..." -ForegroundColor Cyan
    $startTime = Get-Date
    $lastPos = ""
    $updateCount = 0
    
    while (((Get-Date) - $startTime).TotalSeconds -lt $DurationSeconds) {
        $status = Get-MachineStatus
        
        if ($status -match "<(\w+)\|MPos:([\d\.\-]+),([\d\.\-]+),([\d\.\-]+)") {
            $state = $matches[1]
            $x = [math]::Round([double]$matches[2], 3)
            $y = [math]::Round([double]$matches[3], 3)
            $z = [math]::Round([double]$matches[4], 3)
            $currentPos = "$x,$y,$z"
            
            if ($currentPos -ne $lastPos) {
                $stateColor = switch ($state) {
                    "Run"  { "Green" }
                    "Idle" { "White" }
                    "Hold" { "Yellow" }
                    default { "Gray" }
                }
                
                Write-Host "  [$state] " -NoNewline -ForegroundColor $stateColor
                Write-Host "X:$x Y:$y Z:$z" -ForegroundColor White
                
                $lastPos = $currentPos
                $updateCount++
            }
            
            # Exit if machine is idle and we've seen at least one position update
            if ($state -eq "Idle" -and $updateCount -gt 0) {
                break
            }
        }
        
        Start-Sleep -Milliseconds 200
    }
    
    Write-Host "  Position updates: $updateCount`n" -ForegroundColor Gray
}

# Test Suite
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "TEST 1: Setup - Reset and Zero" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan

Send-GCode "G17" "Select XY plane"
Send-GCode "G90" "Absolute positioning"
Send-GCode "G21" "Metric units"
Send-GCode "G92 X0 Y0 Z0" "Zero all axes"
Send-GCode "?" "Check position"

Start-Sleep -Milliseconds 500

Write-Host "`n══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "TEST 2: 90° CW Arc (G2) - Quarter Circle" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "Arc: (0,0) → (10,10) center at (10,0), radius=10mm" -ForegroundColor Gray

Send-GCode "G1 F1000" "Set feedrate 1000 mm/min"
Send-GCode "G2 X10 Y10 I10 J0" "90° clockwise arc"
Monitor-Motion "90° CW Arc" 10

Write-Host "`n══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "TEST 3: 90° CCW Arc (G3) - Return to Origin" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "Arc: (10,10) → (0,0) center at (0,10), radius=10mm" -ForegroundColor Gray

Send-GCode "G3 X0 Y0 I-10 J0" "90° counter-clockwise arc"
Monitor-Motion "90° CCW Arc" 10

Write-Host "`n══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "TEST 4: Full Circle CW (G2) - 360°" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "Arc: (0,0) → (0,0) center at (5,0), radius=5mm" -ForegroundColor Gray

Send-GCode "G2 X0 Y0 I5 J0" "Full circle clockwise"
Monitor-Motion "Full Circle CW" 15

Write-Host "`n══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "TEST 5: Semi-Circle CW (G2) - 180°" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "Arc: (0,0) → (20,0) center at (10,0), radius=10mm" -ForegroundColor Gray

Send-GCode "G2 X20 Y0 I10 J0" "180° clockwise arc"
Monitor-Motion "Semi-Circle CW" 10

Write-Host "`n══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "TEST 6: Semi-Circle CCW (G3) - Return" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "Arc: (20,0) → (0,0) center at (10,0), radius=10mm" -ForegroundColor Gray

Send-GCode "G3 X0 Y0 I-10 J0" "180° counter-clockwise arc"
Monitor-Motion "Semi-Circle CCW" 10

Write-Host "`n══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "TEST 7: Helical Arc - CW with Z Motion" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "Arc: (0,0,0) → (10,10,5) center at (10,0,0), radius=10mm" -ForegroundColor Gray

Send-GCode "G92 Z0" "Reset Z"
Send-GCode "G2 X10 Y10 Z5 I10 J0" "90° arc with Z ramp"
Monitor-Motion "Helical Arc" 12

Write-Host "`n══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "TEST 8: Small Arc - Precision Test" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "Arc: (10,10,5) → (12,12,5) center at (12,10,5), radius=2mm" -ForegroundColor Gray

Send-GCode "G2 X12 Y12 I2 J0" "Small 90° arc, r=2mm"
Monitor-Motion "Small Arc" 8

Write-Host "`n══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "TEST 9: Fast Arc - Speed Test" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "Arc: (12,12,5) → (0,0,0) with F3000" -ForegroundColor Gray

Send-GCode "G1 F3000" "Set feedrate 3000 mm/min"
Send-GCode "G3 X0 Y0 Z0 I-12 J-12" "Fast arc to origin"
Monitor-Motion "Fast Arc" 8

# Final status check
Write-Host "`n══════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "FINAL STATUS CHECK" -ForegroundColor Cyan
Write-Host "══════════════════════════════════════════════════════════" -ForegroundColor Cyan

$finalStatus = Send-GCode "?" "Final position"

if ($finalStatus -match "MPos:([\d\.\-]+),([\d\.\-]+),([\d\.\-]+)") {
    $x = [math]::Round([double]$matches[1], 3)
    $y = [math]::Round([double]$matches[2], 3)
    $z = [math]::Round([double]$matches[3], 3)
    
    Write-Host "`nFinal Position:" -ForegroundColor Cyan
    Write-Host "  X: $x mm" -ForegroundColor White
    Write-Host "  Y: $y mm" -ForegroundColor White
    Write-Host "  Z: $z mm" -ForegroundColor White
    
    # Check if returned to origin
    $tolerance = 0.1
    if ([math]::Abs($x) -lt $tolerance -and [math]::Abs($y) -lt $tolerance -and [math]::Abs($z) -lt $tolerance) {
        Write-Host "`n✓ Successfully returned to origin!" -ForegroundColor Green
    } else {
        Write-Host "`n✗ Position error detected!" -ForegroundColor Yellow
        Write-Host "  Expected: (0.0, 0.0, 0.0)" -ForegroundColor Gray
        Write-Host "  Actual:   ($x, $y, $z)" -ForegroundColor Gray
    }
}

# Cleanup
$serialPort.Close()
Write-Host "`n✓ Test complete - Serial port closed`n" -ForegroundColor Green
