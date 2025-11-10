# Simple serial test - one command at a time with delays
param(
    [string]$ComPort = "COM4",
    [int]$BaudRate = 115200
)

$port = new-Object System.IO.Ports.SerialPort $ComPort,$BaudRate,None,8,one
$port.Open()
Start-Sleep -Milliseconds 500

Write-Host "=== Connected to $ComPort ===" -ForegroundColor Green

# Test 1: Status
Write-Host "`n[TEST] Sending: ?" -ForegroundColor Yellow
$port.WriteLine("?")
Start-Sleep -Milliseconds 200
$response = $port.ReadExisting()
Write-Host "Response: $response" -ForegroundColor Cyan

# Test 2: Build Info
Write-Host "`n[TEST] Sending: `$I" -ForegroundColor Yellow
$port.WriteLine("`$I")
Start-Sleep -Milliseconds 200
$response = $port.ReadExisting()
Write-Host "Response: $response" -ForegroundColor Cyan

# Test 3: Settings
Write-Host "`n[TEST] Sending: `$`$" -ForegroundColor Yellow
$port.WriteLine("`$`$")
Start-Sleep -Milliseconds 500  # Settings is longer
$response = $port.ReadExisting()
Write-Host "Response: $response" -ForegroundColor Cyan

# Test 4: Parser State
Write-Host "`n[TEST] Sending: `$G" -ForegroundColor Yellow
$port.WriteLine("`$G")
Start-Sleep -Milliseconds 200
$response = $port.ReadExisting()
Write-Host "Response: $response" -ForegroundColor Cyan

# Test 5: Help
Write-Host "`n[TEST] Sending: `$" -ForegroundColor Yellow
$port.WriteLine("`$")
Start-Sleep -Milliseconds 200
$response = $port.ReadExisting()
Write-Host "Response: $response" -ForegroundColor Cyan

$port.Close()
Write-Host "`n=== Tests Complete ===" -ForegroundColor Green
