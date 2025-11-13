# Simple dwell test - sends commands and polls status
# Goal: See if status polling continues during G4 dwell

$portName = "COM7"
$baudRate = 115200

try {
    $port = New-Object System.IO.Ports.SerialPort $portName, $baudRate, None, 8, One
    $port.ReadTimeout = 1000
    $port.Open()
    Start-Sleep -Milliseconds 500
    
    Write-Host "=== Dwell Test Started ===" -ForegroundColor Cyan
    
    # Clear any startup messages
    Start-Sleep -Milliseconds 500
    while ($port.BytesToRead -gt 0) {
        $port.ReadExisting() | Out-Null
    }
    
    # Send setup
    Write-Host "`nSending setup commands..." -ForegroundColor Yellow
    $port.WriteLine("G21")
    Start-Sleep -Milliseconds 200
    $port.WriteLine("G90")
    Start-Sleep -Milliseconds 200
    
    # Clear responses
    while ($port.BytesToRead -gt 0) {
        Write-Host $port.ReadLine()
    }
    
    Write-Host "`n=== Test Sequence ===" -ForegroundColor Cyan
    Write-Host "1. Send G0 X10 (should move)"
    Write-Host "2. Poll status 5 times"
    Write-Host "3. Send G4 P1.0 (1 second dwell)"
    Write-Host "4. Poll status continuously during dwell"
    Write-Host "5. Send G0 X0 after dwell"
    Write-Host ""
    
    # Step 1: First move
    Write-Host "[CMD] G0 X10 F1000" -ForegroundColor Green
    $port.WriteLine("G0 X10 F1000")
    Start-Sleep -Milliseconds 100
    $response = $port.ReadLine()
    Write-Host "      $response"
    
    # Step 2: Poll a few times
    Write-Host "`n[POLL] Status during/after first move:" -ForegroundColor Magenta
    for ($i = 1; $i -le 5; $i++) {
        $port.Write("?")
        Start-Sleep -Milliseconds 100
        try {
            $status = $port.ReadLine()
            Write-Host "  $i. $status" -ForegroundColor Yellow
        } catch {
            Write-Host "  $i. (no response)" -ForegroundColor Red
        }
        Start-Sleep -Milliseconds 200
    }
    
    # Step 3: Send dwell
    Write-Host "`n[CMD] G4 P1.0 (1 second dwell)" -ForegroundColor Green
    $port.WriteLine("G4 P1.0")
    Start-Sleep -Milliseconds 100
    $response = $port.ReadLine()
    Write-Host "      $response"
    
    # Step 4: Poll during dwell - THIS IS THE CRITICAL TEST
    Write-Host "`n[POLL] Status DURING 1.0 second dwell (should see continuous responses):" -ForegroundColor Magenta
    $startTime = Get-Date
    $pollCount = 0
    $responseCount = 0
    
    while (((Get-Date) - $startTime).TotalSeconds -lt 1.5) {
        $pollCount++
        $port.Write("?")
        Start-Sleep -Milliseconds 50
        
        try {
            $status = $port.ReadLine()
            $responseCount++
            $elapsed = [math]::Round(((Get-Date) - $startTime).TotalMilliseconds, 0)
            Write-Host "  $pollCount. [$elapsed ms] $status" -ForegroundColor Yellow
        } catch {
            # Timeout - no response
        }
        
        Start-Sleep -Milliseconds 250
    }
    
    Write-Host "`n=== Results ===" -ForegroundColor Cyan
    Write-Host "Status polls sent: $pollCount"
    Write-Host "Status responses received: $responseCount"
    
    if ($responseCount -eq 0) {
        Write-Host "FAILED: No status responses during dwell!" -ForegroundColor Red
    } elseif ($responseCount -lt 3) {
        Write-Host "PARTIAL: Some responses but not continuous" -ForegroundColor Yellow
    } else {
        Write-Host "SUCCESS: Status polling working during dwell!" -ForegroundColor Green
    }
    
    # Step 5: Send final move
    Write-Host "`n[CMD] G0 X0 F1000" -ForegroundColor Green
    $port.WriteLine("G0 X0 F1000")
    Start-Sleep -Milliseconds 100
    $response = $port.ReadLine()
    Write-Host "      $response"
    
    # Final status poll
    Write-Host "`n[POLL] Final status:" -ForegroundColor Magenta
    for ($i = 1; $i -le 3; $i++) {
        $port.Write("?")
        Start-Sleep -Milliseconds 100
        try {
            $status = $port.ReadLine()
            Write-Host "  $status" -ForegroundColor Yellow
        } catch {}
        Start-Sleep -Milliseconds 200
    }
    
    Write-Host "`n=== Test Complete ===" -ForegroundColor Cyan
    
} catch {
    Write-Host "Error: $_" -ForegroundColor Red
} finally {
    if ($port -and $port.IsOpen) {
        $port.Close()
    }
}
