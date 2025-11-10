# Fix X-axis direction and test motion settings
param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200
)

Write-Host "=== Fix X-Axis Settings ===" -ForegroundColor Cyan

try {
    $serialPort = New-Object System.IO.Ports.SerialPort
    $serialPort.PortName = $Port
    $serialPort.BaudRate = $BaudRate
    $serialPort.Open()
    Start-Sleep -Milliseconds 500
    
    # Clear buffer
    while ($serialPort.BytesToRead -gt 0) { $null = $serialPort.ReadLine() }
    
    Write-Host "`n1. Inverting X direction ($3=1)..." -ForegroundColor Yellow
    $serialPort.WriteLine('$3=1')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`n2. Setting acceleration to 100 mm/s² ($120=100)..." -ForegroundColor Yellow
    $serialPort.WriteLine('$120=100')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`n3. Resetting position to zero..." -ForegroundColor Yellow
    $serialPort.WriteLine('G92 X0')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`n4. Testing forward motion (G1 X10 F500)..." -ForegroundColor Yellow
    $serialPort.WriteLine('G1 X10 F500')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Write-Host "`n5. Checking position during motion..." -ForegroundColor Yellow
    for ($i = 0; $i -lt 5; $i++) {
        $serialPort.WriteLine('?')
        Start-Sleep -Milliseconds 300
        while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Cyan }
    }
    
    Write-Host "`n=== Settings Applied ===" -ForegroundColor Green
    Write-Host "X direction now inverted (positive commands should move forward)" -ForegroundColor White
    Write-Host "Test: Measure actual distance traveled vs commanded distance" -ForegroundColor White
    Write-Host "If distance is wrong, adjust $100 (steps per mm)" -ForegroundColor White
    
} finally {
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
    }
}
