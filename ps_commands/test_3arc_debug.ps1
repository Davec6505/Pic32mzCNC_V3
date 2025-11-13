# Test 3-arc sequence with debug output
# This script sends the exact sequence that was stalling UGS

$port = new-Object System.IO.Ports.SerialPort COM7,115200,None,8,one
$port.DtrEnable = $true
$port.RtsEnable = $true
$port.Open()

Start-Sleep -Milliseconds 500

Write-Host "`n=== FLASH DEBUG FIRMWARE FIRST, THEN PRESS ANY KEY ===" -ForegroundColor Yellow
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

Write-Host "`n=== Starting 3-Arc Debug Test ===" -ForegroundColor Cyan

# Send setup commands
Write-Host "`nSending setup commands..." -ForegroundColor Green
$port.WriteLine("G21")
Start-Sleep -Milliseconds 100
$port.WriteLine("G90")
Start-Sleep -Milliseconds 100
$port.WriteLine("G17")
Start-Sleep -Milliseconds 100
$port.WriteLine("G94")
Start-Sleep -Milliseconds 100
$port.WriteLine("M3S1000")
Start-Sleep -Milliseconds 200

# Rapid moves to start position
Write-Host "Moving to start position..." -ForegroundColor Green
$port.WriteLine("G0Z5F1500")
Start-Sleep -Milliseconds 100
$port.WriteLine("G0X0Y0")
Start-Sleep -Milliseconds 100
$port.WriteLine("G0Z0")
Start-Sleep -Milliseconds 200

# First arc - G2 (CW semicircle)
Write-Host "`nSending Arc 1: G2X10Y0I5J0F1000" -ForegroundColor Yellow
$port.WriteLine("G2X10Y0I5J0F1000")
Start-Sleep -Milliseconds 500

# Dwell
$port.WriteLine("G4P0.5")
Start-Sleep -Milliseconds 600

# Second arc - G3 (CCW quarter circle)
Write-Host "Sending Arc 2: G3X10Y10I0J5F1000" -ForegroundColor Yellow
$port.WriteLine("G3X10Y10I0J5F1000")
Start-Sleep -Milliseconds 500

# Dwell
$port.WriteLine("G4P0.5")
Start-Sleep -Milliseconds 600

# Third arc - G2 (CW semicircle back)
Write-Host "Sending Arc 3: G2X0Y10I-5J0F1000" -ForegroundColor Yellow
$port.WriteLine("G2X0Y10I-5J0F1000")
Start-Sleep -Milliseconds 500

Write-Host "`n=== All 3 arcs sent ===" -ForegroundColor Cyan
Write-Host "Monitoring responses for 10 seconds..." -ForegroundColor Green

# Read responses
$timeout = [datetime]::Now.AddSeconds(10)
while ([datetime]::Now -lt $timeout) {
    if ($port.BytesToRead -gt 0) {
        $response = $port.ReadExisting()
        Write-Host $response -NoNewline -ForegroundColor White
    }
    Start-Sleep -Milliseconds 50
}

Write-Host "`n`n=== Test Complete ===" -ForegroundColor Cyan
$port.Close()
