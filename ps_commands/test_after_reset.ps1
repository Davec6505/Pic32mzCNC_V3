# Force settings reload with soft reset
param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200
)

Write-Host "=== Force Settings Reload ===" -ForegroundColor Cyan

try {
    $serialPort = New-Object System.IO.Ports.SerialPort
    $serialPort.PortName = $Port
    $serialPort.BaudRate = $BaudRate
    $serialPort.Open()
    Start-Sleep -Milliseconds 500
    
    # Clear buffer
    while ($serialPort.BytesToRead -gt 0) { $null = $serialPort.ReadLine() }
    
    Write-Host "Sending soft reset (Ctrl+X)..." -ForegroundColor Yellow
    $serialPort.Write([char]0x18)
    Start-Sleep -Milliseconds 1000
    
    Write-Host "Startup messages:" -ForegroundColor Gray
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Start-Sleep -Milliseconds 500
    
    Write-Host "`nVerifying $100 setting..." -ForegroundColor Yellow
    $serialPort.WriteLine('$100')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Cyan }
    
    Write-Host "`nReset position and test..." -ForegroundColor Yellow
    $serialPort.WriteLine('G92 X0')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { $null = $serialPort.ReadLine() }
    
    $serialPort.WriteLine('G1 X10 F500')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Green }
    
    Start-Sleep -Milliseconds 3000
    
    $serialPort.WriteLine('?')
    Start-Sleep -Milliseconds 200
    while ($serialPort.BytesToRead -gt 0) { Write-Host $serialPort.ReadLine() -ForegroundColor Cyan }
    
    Write-Host "`nMeasure actual distance now!" -ForegroundColor Yellow
    
} finally {
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
    }
}
