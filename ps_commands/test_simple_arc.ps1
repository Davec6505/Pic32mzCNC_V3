# Simple Arc Test - Single semicircle
# Tests G2 (clockwise arc) with correct I/J parameters
# Expected: 180° arc from (0,0) to (10,0) with 5mm radius

param(
    [string]$ComPort = "COM3",
    [int]$BaudRate = 115200
)

$port = New-Object System.IO.Ports.SerialPort $ComPort,$BaudRate,None,8,one
$port.Open()
Start-Sleep -Milliseconds 500

Write-Host "=== Connected to $ComPort ===" -ForegroundColor Green

function Send-GCode {
    param([string]$cmd)
    Write-Host ">>> $cmd" -ForegroundColor Cyan
    $port.WriteLine($cmd)
    Start-Sleep -Milliseconds 150
    
    # Read response
    while ($port.BytesToRead -gt 0) {
        $response = $port.ReadLine()
        Write-Host $response -ForegroundColor Green
    }
}

Write-Host "`n=== Setup Commands ===" -ForegroundColor Yellow
Send-GCode "G21"      # mm mode
Send-GCode "G90"      # absolute positioning
Send-GCode "G17"      # XY plane
Send-GCode "G92 X0 Y0 Z0 A0"  # Set current as origin

Write-Host "`n=== Positioning ===" -ForegroundColor Yellow
Send-GCode "G0 Z5"    # Lift Z
Send-GCode "G0 X0 Y0" # Go to start
Send-GCode "G0 Z0"    # Lower Z

Write-Host "`n=== Arc Command (Corrected) ===" -ForegroundColor Yellow
Send-GCode "G2 X10 Y0 I5 J0 F1000"  # 180° CW arc, 5mm radius

Write-Host "`n=== Status Check ===" -ForegroundColor Yellow
Start-Sleep -Milliseconds 2000  # Wait for motion to complete
$port.WriteLine("?")
Start-Sleep -Milliseconds 100

while ($port.BytesToRead -gt 0) {
    $response = $port.ReadLine()
    Write-Host $response -ForegroundColor Cyan
}

Write-Host "`n=== Expected Final Position: (10.000, 0.000, 0.000) ===" -ForegroundColor Yellow

$port.Close()
Write-Host "`nTest complete!" -ForegroundColor Green
