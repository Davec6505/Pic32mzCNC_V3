# Test script to mimic UGS - sends commands and polls status every 300ms
# Tests dwell implementation with continuous status polling

$portName = "COM3"
$baudRate = 115200

try {
    $port = New-Object System.IO.Ports.SerialPort $portName, $baudRate, None, 8, One
    $port.Open()
    Start-Sleep -Milliseconds 500
    
    Write-Host "Connected to $portName"
    
    # Function to send command and wait for ok
    function Send-Command {
        param([string]$cmd)
        Write-Host ">>> $cmd" -ForegroundColor Cyan
        $port.WriteLine($cmd)
        Start-Sleep -Milliseconds 100
        while ($port.BytesToRead -gt 0) {
            $response = $port.ReadLine()
            Write-Host $response -ForegroundColor Green
        }
    }
    
    # Function to poll status (mimics UGS)
    function Poll-Status {
        $port.Write("?")
        Start-Sleep -Milliseconds 50
        while ($port.BytesToRead -gt 0) {
            $response = $port.ReadLine()
            if ($response -match "<.*>") {
                Write-Host $response -ForegroundColor Yellow
            }
        }
    }
    
    # Send initial setup commands
    Send-Command "G21"
    Send-Command "G90"
    Send-Command "G17"
    
    # Send simple test: move, dwell, move
    Write-Host "`n=== Test: G0 → G4 P0.5 → G0 ===" -ForegroundColor Magenta
    Send-Command "G0 X10 Y0 F1000"
    
    # Poll status continuously while waiting for motion
    Write-Host "Polling status during motion and dwell..." -ForegroundColor Magenta
    for ($i = 0; $i -lt 20; $i++) {
        Poll-Status
        Start-Sleep -Milliseconds 300  # Poll every 300ms like UGS
    }
    
    Send-Command "G4 P0.5"
    
    # Continue polling during dwell
    Write-Host "Polling during 0.5 second dwell..." -ForegroundColor Magenta
    for ($i = 0; $i -lt 5; $i++) {
        Poll-Status
        Start-Sleep -Milliseconds 300
    }
    
    Send-Command "G0 X0 Y10 F1000"
    
    # Final polling
    for ($i = 0; $i -lt 10; $i++) {
        Poll-Status
        Start-Sleep -Milliseconds 300
    }
    
    Write-Host "`nTest complete!" -ForegroundColor Green
    
} catch {
    Write-Host "Error: $_" -ForegroundColor Red
} finally {
    if ($port -and $port.IsOpen) {
        $port.Close()
    }
}
