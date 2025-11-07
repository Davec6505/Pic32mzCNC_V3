# Test UGS command sequence with configurable line endings
# Simulates UGS rapid command pipelining behavior

param(
    [string]$ComPort = "COM4",
    [int]$BaudRate = 115200,
    [switch]$DropCR  # If set, only send LF (\n), otherwise send CRLF (\r\n)
)

# Determine line ending based on flag
if ($DropCR) {
    $lineEnding = "`n"
    Write-Host "Using LF only (\\n)" -ForegroundColor Yellow
} else {
    $lineEnding = "`r`n"
    Write-Host "Using CRLF (\\r\\n)" -ForegroundColor Yellow
}

# Open serial port
$port = New-Object System.IO.Ports.SerialPort $ComPort, $BaudRate, None, 8, One
$port.ReadTimeout = 2000
$port.WriteTimeout = 1000

try {
    $port.Open()
    Write-Host "Connected to $ComPort at $BaudRate baud" -ForegroundColor Green
    
    # Give firmware time to send startup banner
    Start-Sleep -Milliseconds 500
    
    # Read and display startup banner
    try {
        $banner = $port.ReadExisting()
        if ($banner) {
            Write-Host "`n=== STARTUP BANNER ===" -ForegroundColor Cyan
            Write-Host $banner -ForegroundColor Gray
        }
    } catch {
        # No banner or timeout - continue
    }
    
    # Wait a bit before sending commands
    Start-Sleep -Milliseconds 200
    
    # TEST 1: Send single status query
    Write-Host "`n=== TEST 1: Status Query (?) ===" -ForegroundColor Cyan
    $port.Write("?$lineEnding")
    Start-Sleep -Milliseconds 100
    $response = $port.ReadExisting()
    Write-Host "Response: $response" -ForegroundColor Gray
    
    Start-Sleep -Milliseconds 500
    
    # TEST 2: UGS rapid sequence - $I then $$
    Write-Host "`n=== TEST 2: Rapid Sequence `$I + `$`$ ===" -ForegroundColor Cyan
    Write-Host "Sending: `$I$lineEnding" -ForegroundColor Yellow -NoNewline
    $port.Write("`$I$lineEnding")
    
    # Immediate send (no wait) - simulates UGS pipelining
    Write-Host "`$`$$lineEnding" -ForegroundColor Yellow
    $port.Write("`$`$$lineEnding")
    
    # Wait for responses
    Start-Sleep -Milliseconds 1500
    $response = $port.ReadExisting()
    Write-Host "`nResponse:" -ForegroundColor Cyan
    Write-Host $response -ForegroundColor Gray
    
    Start-Sleep -Milliseconds 500
    
    # TEST 3: Send $G
    Write-Host "`n=== TEST 3: Parser State (`$G) ===" -ForegroundColor Cyan
    $port.Write("`$G$lineEnding")
    Start-Sleep -Milliseconds 200
    $response = $port.ReadExisting()
    Write-Host "Response: $response" -ForegroundColor Gray
    
    Start-Sleep -Milliseconds 500
    
    # TEST 4: Full UGS connection sequence (all at once)
    Write-Host "`n=== TEST 4: Full UGS Sequence (All Commands) ===" -ForegroundColor Cyan
    Write-Host "Sending rapid fire: ? + `$I + `$`$ + `$G" -ForegroundColor Yellow
    
    $port.Write("?$lineEnding")
    $port.Write("`$I$lineEnding")
    $port.Write("`$`$$lineEnding")
    $port.Write("`$G$lineEnding")
    
    # Wait for all responses
    Start-Sleep -Milliseconds 2000
    $response = $port.ReadExisting()
    Write-Host "`nResponse:" -ForegroundColor Cyan
    Write-Host $response -ForegroundColor Gray
    
    Write-Host "`n=== TESTS COMPLETE ===" -ForegroundColor Green
    
} catch {
    Write-Host "Error: $_" -ForegroundColor Red
} finally {
    if ($port.IsOpen) {
        $port.Close()
        Write-Host "`nPort closed." -ForegroundColor Yellow
    }
}

Write-Host "`nUsage Examples:" -ForegroundColor Cyan
Write-Host "  .\test_ugs_sequence.ps1                  # Use CRLF (default)" -ForegroundColor Gray
Write-Host "  .\test_ugs_sequence.ps1 -DropCR          # Use LF only" -ForegroundColor Gray
Write-Host "  .\test_ugs_sequence.ps1 -ComPort COM5    # Different port" -ForegroundColor Gray
