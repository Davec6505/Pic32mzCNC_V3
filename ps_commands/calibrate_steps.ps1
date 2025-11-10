# Update X-axis steps per mm to correct calibration
param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200
)

Write-Host "=== Calibrate X-Axis Steps Per MM ===" -ForegroundColor Cyan
Write-Host "Commanded: 10mm, Actual: 2mm" -ForegroundColor Gray
Write-Host "Corrected steps/mm = (10/2) × 40 = 200" -ForegroundColor Gray
Write-Host ""

try {
    $serialPort = New-Object System.IO.Ports.SerialPort
    $serialPort.PortName = $Port
    $serialPort.BaudRate = $BaudRate
    $serialPort.Open()
    Start-Sleep -Milliseconds 500
    
    # Clear buffer
    while ($serialPort.BytesToRead -gt 0) { $null = $serialPort.ReadLine() }
    
    Write-Host "Setting X steps/mm to 200 ($100=200)..." -ForegroundColor Yellow
    $serialPort.WriteLine('$100=200')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`nResetting position to zero..." -ForegroundColor Yellow
    $serialPort.WriteLine('G92 X0')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`nTesting calibrated motion (G1 X10 F500)..." -ForegroundColor Yellow
    $serialPort.WriteLine('G1 X10 F500')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Start-Sleep -Milliseconds 2000  # Wait for motion to complete
    
    Write-Host "`nFinal position check..." -ForegroundColor Yellow
    $serialPort.WriteLine('?')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Cyan }
    
    Write-Host "`n=== Calibration Complete ===" -ForegroundColor Green
    Write-Host "Measure the actual distance again - should now be 10mm!" -ForegroundColor White
    Write-Host "If still not exact, fine-tune $100 using this formula:" -ForegroundColor White
    Write-Host "  New $100 = (Commanded / Actual) × 200" -ForegroundColor Gray
    
} finally {
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
    }
}
