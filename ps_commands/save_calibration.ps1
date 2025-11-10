# Save calibrated settings to flash memory
param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200
)

Write-Host "=== Save Calibrated Settings to Flash ===" -ForegroundColor Cyan

try {
    $serialPort = New-Object System.IO.Ports.SerialPort
    $serialPort.PortName = $Port
    $serialPort.BaudRate = $BaudRate
    $serialPort.Open()
    Start-Sleep -Milliseconds 500
    
    # Clear buffer
    while ($serialPort.BytesToRead -gt 0) { $null = $serialPort.ReadLine() }
    
    Write-Host "`n1. Setting X steps/mm to 200..." -ForegroundColor Yellow
    $serialPort.WriteLine('$100=200')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`n2. Setting Y steps/mm to 200..." -ForegroundColor Yellow
    $serialPort.WriteLine('$101=200')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`n3. Setting Z steps/mm to 200..." -ForegroundColor Yellow
    $serialPort.WriteLine('$102=200')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`n4. Verifying settings before save..." -ForegroundColor Yellow
    $serialPort.WriteLine('$$')
    Start-Sleep -Milliseconds 1000
    $settings = @()
    while ($serialPort.BytesToRead -gt 0) { 
        $line = $serialPort.ReadLine()
        $settings += $line
        if ($line -match '^\$10[0-2]=') {
            Write-Host $line -ForegroundColor Cyan
        }
    }
    
    Write-Host "`n5. Triggering soft reset to save to flash..." -ForegroundColor Yellow
    Write-Host "   (Settings auto-save on soft reset)" -ForegroundColor Gray
    $serialPort.Write([char]0x18)
    Start-Sleep -Milliseconds 1000
    
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`n6. Verifying settings persisted after reset..." -ForegroundColor Yellow
    Start-Sleep -Milliseconds 500
    $serialPort.WriteLine('$100')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Cyan }
    
    Write-Host "`n=== Settings Saved! ===" -ForegroundColor Green
    Write-Host "Test motion: G92 X0, then G1 X10 F500" -ForegroundColor White
    Write-Host "Should now move exactly 10mm!" -ForegroundColor White
    
} finally {
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
    }
}
