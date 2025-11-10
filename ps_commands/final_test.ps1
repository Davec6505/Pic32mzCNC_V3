# Final calibration test
param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200
)

Write-Host "=== Final Calibration Test ===" -ForegroundColor Cyan

try {
    $serialPort = New-Object System.IO.Ports.SerialPort
    $serialPort.PortName = $Port
    $serialPort.BaudRate = $BaudRate
    $serialPort.Open()
    Start-Sleep -Milliseconds 500
    
    while ($serialPort.BytesToRead -gt 0) { $null = $serialPort.ReadLine() }
    
    Write-Host "Resetting position..." -ForegroundColor Yellow
    $serialPort.WriteLine('G92 X0')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { $null = $serialPort.ReadLine() }
    
    Write-Host "Moving X10mm at F500..." -ForegroundColor Yellow
    $serialPort.WriteLine('G1 X10 F500')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Start-Sleep -Milliseconds 3000
    
    Write-Host "`nFinal position:" -ForegroundColor Yellow
    $serialPort.WriteLine('?')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Cyan }
    
    Write-Host "`n=== MEASURE ACTUAL DISTANCE ===" -ForegroundColor Green
    Write-Host "Should be exactly 10mm now!" -ForegroundColor White
    
} finally {
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
    }
}
