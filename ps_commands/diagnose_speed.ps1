# Diagnostic script to analyze motion speed issue
# Sends a simple move and monitors position change rate

param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200
)

Write-Host "=== Motion Speed Diagnostic ===" -ForegroundColor Cyan
Write-Host "Port: $Port @ $BaudRate baud" -ForegroundColor Gray
Write-Host ""

# Open serial port
try {
    $serial = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, 'None', 8, 'One'
    $serial.Open()
    Start-Sleep -Milliseconds 500
    
    # Flush any startup messages
    while ($serial.BytesToRead -gt 0) {
        $null = $serial.ReadExisting()
    }
    
    Write-Host "Connected to $Port" -ForegroundColor Green
    Write-Host ""
} catch {
    Write-Host "ERROR: Cannot open $Port - $_" -ForegroundColor Red
    exit 1
}

# Function to send command and wait for response
function Send-GCode {
    param([string]$cmd)
    
    Write-Host ">>> $cmd" -ForegroundColor Yellow
    $serial.WriteLine($cmd)
    
    # Wait for "ok" response
    $timeout = [DateTime]::Now.AddSeconds(5)
    
    while ([DateTime]::Now -lt $timeout) {
        if ($serial.BytesToRead -gt 0) {
            $line = $serial.ReadLine().Trim()
            Write-Host $line -ForegroundColor Gray
            
            if ($line -eq "ok") {
                return $true
            }
            if ($line.StartsWith("error")) {
                Write-Host "ERROR: $line" -ForegroundColor Red
                return $false
            }
        }
        Start-Sleep -Milliseconds 10
    }
    
    return $false
}

# Setup
Send-GCode "G21"  # Metric
Send-GCode "G90"  # Absolute
Send-GCode "G92 X0 Y0 Z0"  # Zero current position

Write-Host ""
Write-Host "=== Starting 20mm X-axis move at F8000 ===" -ForegroundColor Cyan
Write-Host ""

# Record start time
$startTime = Get-Date

# Send move command
Send-GCode "G1 X20 F8000"

# Monitor position updates
$lastX = 0
$sampleCount = 0
$positionSamples = @()

for ($i = 0; $i -lt 30; $i++) {
    # Flush buffer
    while ($serial.BytesToRead -gt 0) {
        $null = $serial.ReadExisting()
    }
    
    # Query position
    $serial.WriteLine("?")
    Start-Sleep -Milliseconds 100
    
    # Read response
    $response = ""
    $readTimeout = [DateTime]::Now.AddMilliseconds(200)
    while ([DateTime]::Now -lt $readTimeout -and $serial.BytesToRead -gt 0) {
        $response += $serial.ReadExisting()
        Start-Sleep -Milliseconds 10
    }
    
    # Parse position
    if ($response -match "WPos:([\d\.-]+),([\d\.-]+)") {
        $currentX = [decimal]$matches[1]
        $currentY = [decimal]$matches[2]
        $elapsed = (Get-Date) - $startTime
        
        # Calculate speed if we have movement
        if ($i -gt 0 -and $currentX -ne $lastX) {
            $deltaX = $currentX - $lastX
            $speed = ($deltaX / 0.1) * 60  # mm/min (0.1s between samples)
            $positionSamples += [PSCustomObject]@{
                Time = $elapsed.TotalSeconds
                X = $currentX
                DeltaX = $deltaX
                Speed = $speed
            }
            Write-Host ("[{0:F1}s] X={1:F3} mm  ΔX={2:F3} mm  Speed={3:F0} mm/min" -f $elapsed.TotalSeconds, $currentX, $deltaX, $speed) -ForegroundColor Cyan
        } else {
            Write-Host ("[{0:F1}s] X={1:F3} mm" -f $elapsed.TotalSeconds, $currentX) -ForegroundColor Gray
        }
        
        $lastX = $currentX
        
        # Exit if reached target
        if ($currentX -ge 19.9) {
            Write-Host ""
            Write-Host "✓ Target reached!" -ForegroundColor Green
            break
        }
    }
    
    Start-Sleep -Milliseconds 100
}

$endTime = Get-Date
$totalTime = ($endTime - $startTime).TotalSeconds

Write-Host ""
Write-Host "=== Motion Analysis ===" -ForegroundColor Cyan
Write-Host ("Total time: {0:F2} seconds" -f $totalTime) -ForegroundColor White
Write-Host ("Final position: X={0:F3} mm" -f $lastX) -ForegroundColor White

if ($positionSamples.Count -gt 0) {
    $avgSpeed = ($positionSamples | Measure-Object -Property Speed -Average).Average
    $maxSpeed = ($positionSamples | Measure-Object -Property Speed -Maximum).Maximum
    $minSpeed = ($positionSamples | Measure-Object -Property Speed -Minimum).Minimum
    
    Write-Host ("Average speed: {0:F0} mm/min ({1:F1}% of F8000)" -f $avgSpeed, ($avgSpeed/8000*100)) -ForegroundColor Yellow
    Write-Host ("Max speed: {0:F0} mm/min" -f $maxSpeed) -ForegroundColor White
    Write-Host ("Min speed: {0:F0} mm/min" -f $minSpeed) -ForegroundColor White
}

Write-Host ""
Write-Host "Expected at F8000: 20mm should take ~0.15 seconds" -ForegroundColor Gray
if ($totalTime -gt 1.0) {
    Write-Host ("⚠️ SLOW: Actual time {0:F1}x slower than expected!" -f ($totalTime / 0.15)) -ForegroundColor Red
}

# Close port
$serial.Close()
Write-Host ""
Write-Host "Diagnostic complete" -ForegroundColor Cyan
