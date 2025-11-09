# Test Motion System with Debug Output
# Sends G-code commands and captures UART responses

param(
    [string]$Port = "COM4",
    [int]$BaudRate = 115200
)

Write-Host "=== Motion Debug Test ===" -ForegroundColor Cyan
Write-Host "Port: $Port @ $BaudRate baud" -ForegroundColor Gray
Write-Host ""

# Open serial port
$serialPort = New-Object System.IO.Ports.SerialPort
$serialPort.PortName = $Port
$serialPort.BaudRate = $BaudRate
$serialPort.Parity = [System.IO.Ports.Parity]::None
$serialPort.DataBits = 8
$serialPort.StopBits = [System.IO.Ports.StopBits]::One
$serialPort.Handshake = [System.IO.Ports.Handshake]::None
$serialPort.ReadTimeout = 1000
$serialPort.WriteTimeout = 1000

try {
    $serialPort.Open()
    Write-Host "✓ Serial port opened" -ForegroundColor Green
    
    # Wait for banner
    Start-Sleep -Milliseconds 500
    
    # Function to send command and read responses
    function Send-GCode {
        param([string]$cmd, [int]$waitMs = 200)
        
        Write-Host "`n>>> $cmd" -ForegroundColor Yellow
        $serialPort.WriteLine($cmd)
        
        Start-Sleep -Milliseconds $waitMs
        
        # Read all available data
        $responses = @()
        $attempts = 0
        while ($serialPort.BytesToRead -gt 0 -and $attempts -lt 20) {
            try {
                $line = $serialPort.ReadLine()
                if ($line) {
                    $responses += $line
                    Write-Host $line -ForegroundColor Gray
                }
            } catch {
                break
            }
            $attempts++
        }
        
        return $responses
    }
    
    # Clear any existing data
    $serialPort.DiscardInBuffer()
    $serialPort.DiscardOutBuffer()
    
    Write-Host "`n=== Test Sequence ===" -ForegroundColor Cyan
    
    # 1. Reset work coordinates
    Write-Host "`n1. Setting work origin (G92 X0 Y0 Z0)" -ForegroundColor Cyan
    $resp = Send-GCode "G92 X0 Y0 Z0" 300
    
    # 2. Query status
    Write-Host "`n2. Initial status query" -ForegroundColor Cyan
    $resp = Send-GCode "?" 200
    
    # 3. Send motion command with debug output expected
    Write-Host "`n3. Sending G1 X50 Y50 F1000" -ForegroundColor Cyan
    Write-Host "   Expected debug output:" -ForegroundColor Gray
    Write-Host "   - [KINEMATICS] dx=... dy=... steps: X=... Y=..." -ForegroundColor DarkGray
    Write-Host "   - [SEGMENT] Loaded: steps=... rate=..." -ForegroundColor DarkGray
    Write-Host "   - [MOTION] Rate update: ..." -ForegroundColor DarkGray
    $resp = Send-GCode "G1 X50 Y50 F100" 500
    
    # 4. Poll status while moving
    Write-Host "`n4. Polling status during motion (10 queries)" -ForegroundColor Cyan
    for ($i = 0; $i -lt 10; $i++) {
        $resp = Send-GCode "?" 100
        Start-Sleep -Milliseconds 50
    }
    

       # 3. Send motion command with debug output expected
    Write-Host "`n3. Sending G1 X5 Y5 F1000" -ForegroundColor Cyan
    Write-Host "   Expected debug output:" -ForegroundColor Gray
    Write-Host "   - [KINEMATICS] dx=... dy=... steps: X=... Y=..." -ForegroundColor DarkGray
    Write-Host "   - [SEGMENT] Loaded: steps=... rate=..." -ForegroundColor DarkGray
    Write-Host "   - [MOTION] Rate update: ..." -ForegroundColor DarkGray
    $resp = Send-GCode "G1 X5 Y5 F100" 500
    
    # 4. Poll status while moving
    Write-Host "`n4. Polling status during motion (10 queries)" -ForegroundColor Cyan
    for ($i = 0; $i -lt 10; $i++) {
        $resp = Send-GCode "?" 100
        Start-Sleep -Milliseconds 50
    }
   

    # 5. Final status
    Write-Host "`n5. Final position check" -ForegroundColor Cyan
    Start-Sleep -Milliseconds 500
    $resp = Send-GCode "?" 200
    
    Write-Host "`n=== Test Complete ===" -ForegroundColor Green
    Write-Host "Check output above for:" -ForegroundColor Yellow
    Write-Host "  - Debug messages from KINEMATICS, SEGMENT, MOTION modules" -ForegroundColor Gray
    Write-Host "  - Position should reach 5.000,5.000 if working correctly" -ForegroundColor Gray
    Write-Host "  - If stuck at 0.025,0.025 → motion stops after 1 step" -ForegroundColor Gray
    
} catch {
    Write-Host "ERROR: $_" -ForegroundColor Red
} finally {
    if ($serialPort.IsOpen) {
        $serialPort.Close()
        Write-Host "`n✓ Serial port closed" -ForegroundColor Green
    }
}
