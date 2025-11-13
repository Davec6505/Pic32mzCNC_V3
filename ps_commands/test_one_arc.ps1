# Test single valid arc with debug output

$port = new-Object System.IO.Ports.SerialPort COM7,115200,None,8,one
$port.DtrEnable = $true
$port.RtsEnable = $true
$port.Open()

Start-Sleep -Milliseconds 500

Write-Host "`n=== Starting Single Arc Test ===" -ForegroundColor Cyan

# Send setup commands
Write-Host "`nSending setup commands..." -ForegroundColor Green
$port.WriteLine("G21")
Start-Sleep -Milliseconds 100
$port.WriteLine("G90")
Start-Sleep -Milliseconds 100
$port.WriteLine("G17")
Start-Sleep -Milliseconds 100

# Rapid move to start
Write-Host "Moving to start..." -ForegroundColor Green
$port.WriteLine("G0X0Y0Z0")
Start-Sleep -Milliseconds 500

# Single semicircle arc
Write-Host "`nSending Arc: G2X10Y0I5J0F1000 (Semicircle CW)" -ForegroundColor Yellow
$port.WriteLine("G2X10Y0I5J0F1000")
Start-Sleep -Milliseconds 3000

Write-Host "`n=== Arc sent, monitoring responses for 10 seconds ===" -ForegroundColor Cyan

# Read responses
$endTime = (Get-Date).AddSeconds(10)
while ((Get-Date) -lt $endTime) {
    if ($port.BytesToRead -gt 0) {
        $line = $port.ReadLine()
        Write-Host $line
    }
    Start-Sleep -Milliseconds 50
}

$port.Close()
Write-Host "`n=== Test Complete ===" -ForegroundColor Green
