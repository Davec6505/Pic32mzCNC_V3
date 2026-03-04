param([string]$ComPort = "COM11", [int]$BaudRate = 115200)

$port = New-Object System.IO.Ports.SerialPort $ComPort, $BaudRate, None, 8, One
$port.ReadTimeout = 3000
$port.Open()
Start-Sleep -Milliseconds 300

# Soft reset
$port.Write([byte[]](0x18), 0, 1)
Start-Sleep -Milliseconds 600
$banner = $port.ReadExisting()
Write-Host "--- Reset ---" -ForegroundColor Magenta
Write-Host $banner

function Send-Line {
    param([string]$cmd, [int]$ms = 400)
    Write-Host ">>> $cmd" -ForegroundColor Yellow
    $port.WriteLine($cmd)
    Start-Sleep -Milliseconds $ms
    $r = $port.ReadExisting()
    if ($r.Trim()) { Write-Host $r.Trim() -ForegroundColor Cyan }
}

Write-Host "=== Arc4 Isolated Test ===" -ForegroundColor Green

# Setup
Send-Line "G21"  300
Send-Line "G90"  300
Send-Line "G17"  300

# Move to arc start (0, 10)
Send-Line "G0 X0 Y0 Z0"  2000
Send-Line "G0 X0 Y10"     2000

# Query position before arc
Send-Line "?" 400

# G92 to zero here so we can clearly read final position
Send-Line "G92 X0 Y10"   300

Send-Line "?" 400

# THE ARC: CW semicircle, start=(0,10), center=(10,10), end=(20,10), R=10mm
Send-Line "G2 X20 Y10 I10 J0 F200"  10000

# Position after arc - should be X=20 Y=10
Send-Line "?" 600
Send-Line "G0 X0 Y0"  2000
Send-Line "?" 400

Write-Host "=== Done ===" -ForegroundColor Green
$port.Close()
