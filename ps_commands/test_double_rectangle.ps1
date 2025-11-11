# Test Double Rectangle - Verify G92 coordinate system behavior
# This script sends a corrected 2-rectangle test file via serial

param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200
)

Write-Host "=== Double Rectangle G92 Test ===" -ForegroundColor Cyan
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
    
    if ($cmd.Trim() -eq "" -or $cmd.StartsWith(";")) {
        return  # Skip empty lines and comments
    }
    
    Write-Host ">>> $cmd" -ForegroundColor Yellow
    $serial.WriteLine($cmd)
    
    # Wait for "ok" response
    $timeout = [DateTime]::Now.AddSeconds(10)
    $response = ""
    
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
    
    Write-Host "TIMEOUT waiting for response" -ForegroundColor Red
    return $false
}

# Test G-code (CORRECTED - only ONE G92 at the start)
$gcode = @"
G21
G90
G17
G92 X0 Y0 Z0
G1 F500

G1 X5 Y0
G1 X5 Y5
G1 X0 Y5
G1 X0 Y0

G1 X5 Y0
G1 X5 Y5
G1 X0 Y5
G1 X0 Y0
"@ -split "`n"

Write-Host "=== Sending G-code commands ===" -ForegroundColor Cyan
Write-Host ""

foreach ($line in $gcode) {
    $line = $line.Trim()
    if ($line -ne "") {
        if (-not (Send-GCode $line)) {
            Write-Host "Command failed, aborting test" -ForegroundColor Red
            break
        }
        Start-Sleep -Milliseconds 100  # Small delay between commands
    }
}

Write-Host ""
Write-Host "=== Waiting for motion to complete ===" -ForegroundColor Cyan

# Poll status until Idle and at origin (0,0)
$maxAttempts = 200
$attempts = 0
$success = $false

while ($attempts -lt $maxAttempts) {
    # Flush any pending data
    while ($serial.BytesToRead -gt 0) {
        $null = $serial.ReadExisting()
    }
    
    # Send status query
    $serial.WriteLine("?")
    Start-Sleep -Milliseconds 100
    
    # Read response
    $response = ""
    $timeout = [DateTime]::Now.AddMilliseconds(500)
    while ([DateTime]::Now -lt $timeout -and $serial.BytesToRead -gt 0) {
        $response += $serial.ReadExisting()
        Start-Sleep -Milliseconds 10
    }
    
    $attempts++
    
    # Parse status line
    if ($response -match "<(Idle|Run)[^>]*MPos:([\d\.-]+),([\d\.-]+),([\d\.-]+)[^>]*WPos:([\d\.-]+),([\d\.-]+),([\d\.-]+)") {
        $state = $matches[1]
        $mposX = [decimal]$matches[2]
        $mposY = [decimal]$matches[3]
        $wposX = [decimal]$matches[5]
        $wposY = [decimal]$matches[6]
        
        Write-Host "[$attempts] $state | WPos: ($wposX, $wposY)" -ForegroundColor Gray
        
        # Check if idle and at origin
        if ($state -eq "Idle" -and [Math]::Abs($wposX) -lt 0.1 -and [Math]::Abs($wposY) -lt 0.1) {
            Write-Host ""
            Write-Host "Final Position Check:" -ForegroundColor Cyan
            Write-Host "  Machine: ($mposX, $mposY)" -ForegroundColor White
            Write-Host "  Work:    ($wposX, $wposY)" -ForegroundColor White
            Write-Host "  ✓ SUCCESS: Motion complete, at origin (0, 0)" -ForegroundColor Green
            $success = $true
            break
        }
    }
    
    Start-Sleep -Milliseconds 500
}

if (-not $success) {
    Write-Host ""
    Write-Host "  ✗ FAIL: Motion did not complete or return to origin after $maxAttempts attempts" -ForegroundColor Red
}

# Close port
$serial.Close()
Write-Host ""
Write-Host "Test complete" -ForegroundColor Cyan
