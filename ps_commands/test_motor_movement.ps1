# Test Motor Movement with Serial Output Capture
# Sends G-code commands and displays responses

param(
    [string]$Port = "COM4",
    [int]$BaudRate = 115200
)

Write-Host "=== Motor Movement Test ===" -ForegroundColor Cyan
Write-Host "Port: $Port, Baud: $BaudRate" -ForegroundColor Yellow

# Open serial port
$port = New-Object System.IO.Ports.SerialPort $Port,$BaudRate,None,8,one
$port.ReadTimeout = 1000
$port.WriteTimeout = 1000
$port.Open()

function Send-GCode {
    param([string]$cmd)
    
    Write-Host "`n>> $cmd" -ForegroundColor Green
    $port.WriteLine($cmd)
    Start-Sleep -Milliseconds 100
    
    # Read response
    $timeout = 0
    while ($port.BytesToRead -eq 0 -and $timeout -lt 20) {
        Start-Sleep -Milliseconds 50
        $timeout++
    }
    
    while ($port.BytesToRead -gt 0) {
        $response = $port.ReadLine()
        Write-Host "<< $response" -ForegroundColor White
        Start-Sleep -Milliseconds 10
    }
}

try {
    Write-Host "`n=== Initial Status ===" -ForegroundColor Cyan
    Send-GCode "?"
    
    Write-Host "`n=== Set Origin ===" -ForegroundColor Cyan
    Send-GCode "G92 X0 Y0 Z0"
    Send-GCode "?"
    
    Write-Host "`n=== Test X Axis - Move 10mm at F100 ===" -ForegroundColor Cyan
    Send-GCode "G1 X10 F100"
    
    # Monitor position during move
    for ($i = 0; $i -lt 10; $i++) {
        Start-Sleep -Milliseconds 500
        Send-GCode "?"
    }
    
    Write-Host "`n=== Return to Origin ===" -ForegroundColor Cyan
    Send-GCode "G1 X0 F100"
    
    for ($i = 0; $i -lt 10; $i++) {
        Start-Sleep -Milliseconds 500
        Send-GCode "?"
    }
    
    Write-Host "`n=== Test Faster Speed F500 ===" -ForegroundColor Cyan
    Send-GCode "G1 X20 F500"
    
    for ($i = 0; $i -lt 5; $i++) {
        Start-Sleep -Milliseconds 300
        Send-GCode "?"
    }
    
    Send-GCode "G1 X0 F500"
    Start-Sleep -Milliseconds 1000
    Send-GCode "?"
    
    Write-Host "`n=== Check Settings ===" -ForegroundColor Cyan
    Send-GCode "`$`$"
    Start-Sleep -Milliseconds 500
    
    # Read all settings output
    while ($port.BytesToRead -gt 0) {
        $response = $port.ReadLine()
        Write-Host "<< $response" -ForegroundColor White
        Start-Sleep -Milliseconds 10
    }
    
    Write-Host "`n=== Test Complete ===" -ForegroundColor Cyan
    
} finally {
    $port.Close()
    Write-Host "`nPort closed." -ForegroundColor Yellow
}

Write-Host "`n=== What to Check ===" -ForegroundColor Yellow
Write-Host "1. Does MPos value change during moves? (Should go 0 -> 10 -> 0 -> 20 -> 0)" -ForegroundColor White
Write-Host "2. Does state show 'Run' during motion?" -ForegroundColor White
Write-Host "3. Check `$4 value (should be 1 for 8266 driver)" -ForegroundColor White
Write-Host "4. Check `$100 value (steps/mm, should be 40)" -ForegroundColor White
