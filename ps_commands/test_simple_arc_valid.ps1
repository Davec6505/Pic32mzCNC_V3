# Test valid arc geometry with debug output
# Uses 02_simple_arc.gcode test file

Write-Host ""
Write-Host "=== FLASH DEBUG FIRMWARE FIRST, THEN PRESS ANY KEY ===" -ForegroundColor Cyan
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")

Write-Host ""
Write-Host "=== Starting Simple Arc Test ===" -ForegroundColor Green

# Auto-detect COM port
$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if ($availablePorts.Count -eq 0) {
    Write-Host "ERROR: No COM ports found!" -ForegroundColor Red
    exit 1
}
$comPort = $availablePorts[0]
Write-Host "Using port: $comPort" -ForegroundColor Cyan

# Open serial port
$port = New-Object System.IO.Ports.SerialPort
$port.PortName = $comPort
$port.BaudRate = 115200
$port.DataBits = 8
$port.Parity = "None"
$port.StopBits = 1
$port.ReadTimeout = 1000

try {
    $port.Open()
    Start-Sleep -Milliseconds 500
    
    # Clear any startup messages
    try { $port.ReadExisting() | Out-Null } catch {}
    
    Write-Host ""
    Write-Host "Sending setup commands..." -ForegroundColor Yellow
    $port.WriteLine("G21")
    Start-Sleep -Milliseconds 100
    $port.WriteLine("G90")
    Start-Sleep -Milliseconds 100
    $port.WriteLine("G17")
    Start-Sleep -Milliseconds 100
    
    Write-Host "Moving to start position..." -ForegroundColor Yellow
    $port.WriteLine("G0X0Y0Z0")
    Start-Sleep -Milliseconds 500
    
    Write-Host ""
    Write-Host "Sending Arc 1: G2X10Y10I10J0 (Quarter circle CW)" -ForegroundColor Cyan
    $port.WriteLine("G2X10Y10I10J0F500")
    Start-Sleep -Milliseconds 2000
    
    Write-Host "Sending Arc 2: G2X60Y0I10J0 (Semicircle from X40Y0)" -ForegroundColor Cyan
    $port.WriteLine("G0X40Y0")
    Start-Sleep -Milliseconds 500
    $port.WriteLine("G2X60Y0I10J0F500")
    Start-Sleep -Milliseconds 3000
    
    Write-Host ""
    Write-Host "=== All arcs sent ===" -ForegroundColor Green
    Write-Host "Monitoring responses for 10 seconds..." -ForegroundColor Yellow
    
    # Read responses with timeout
    $endTime = (Get-Date).AddSeconds(10)
    while ((Get-Date) -lt $endTime) {
        try {
            $line = $port.ReadLine()
            Write-Host $line
        }
        catch {
            Start-Sleep -Milliseconds 50
        }
    }
    
} finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}

Write-Host ""
Write-Host "=== Test Complete ===" -ForegroundColor Green
