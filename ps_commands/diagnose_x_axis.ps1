# Diagnose X-axis Step Pulse Issue
# Purpose: Check GRBL settings and verify stepper enable/step configuration

param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200
)

Write-Host "=== X-Axis Motion Diagnostic Script ===" -ForegroundColor Cyan
Write-Host "Port: $Port, Baud: $BaudRate" -ForegroundColor Gray
Write-Host ""

# Function to send command and read response
function Send-GcodeCommand {
    param(
        [System.IO.Ports.SerialPort]$serial,
        [string]$command,
        [int]$timeout = 2000
    )
    
    Write-Host ">>> $command" -ForegroundColor Yellow
    $serial.WriteLine($command)
    Start-Sleep -Milliseconds 100
    
    $responses = @()
    $startTime = Get-Date
    
    while (((Get-Date) - $startTime).TotalMilliseconds -lt $timeout) {
        if ($serial.BytesToRead -gt 0) {
            $line = $serial.ReadLine().Trim()
            if ($line -ne "") {
                Write-Host "<<< $line" -ForegroundColor Green
                $responses += $line
                
                # Stop reading if we get "ok" or "error"
                if ($line -match "^ok" -or $line -match "^error") {
                    break
                }
            }
        }
        Start-Sleep -Milliseconds 50
    }
    
    return $responses
}

try {
    # Open serial port
    Write-Host "Opening serial port $Port..." -ForegroundColor Cyan
    $serialPort = New-Object System.IO.Ports.SerialPort
    $serialPort.PortName = $Port
    $serialPort.BaudRate = $BaudRate
    $serialPort.Open()
    
    Start-Sleep -Milliseconds 500
    
    # Clear any startup messages
    while ($serialPort.BytesToRead -gt 0) {
        $null = $serialPort.ReadLine()
    }
    
    Write-Host "`n=== STEP 1: Check Critical Settings ===" -ForegroundColor Cyan
    Write-Host "Looking for:" -ForegroundColor Gray
    Write-Host "  `$2 = Step pulse invert mask (should be 0 for normal)" -ForegroundColor Gray
    Write-Host "  `$4 = Enable invert mask (depends on driver type)" -ForegroundColor Gray
    Write-Host "  `$5 = Limit pins invert mask" -ForegroundColor Gray
    Write-Host ""
    
    # Request all settings
    $settingsResponse = Send-GcodeCommand -serial $serialPort -command '$$' -timeout 5000
    
    # Parse critical settings
    $step2 = $null
    $step4 = $null
    $step5 = $null
    
    foreach ($line in $settingsResponse) {
        if ($line -match '^\$2=(\d+)') { $step2 = $matches[1] }
        if ($line -match '^\$4=(\d+)') { $step4 = $matches[1] }
        if ($line -match '^\$5=(\d+)') { $step5 = $matches[1] }
    }
    
    Write-Host "`n--- Critical Settings Analysis ---" -ForegroundColor Cyan
    
    if ($null -ne $step2) {
        Write-Host "`$2 (Step Invert) = $step2" -ForegroundColor $(if ($step2 -eq 0) { "Green" } else { "Yellow" })
        if ($step2 -ne 0) {
            Write-Host "  WARNING: Step pulses are inverted! Bit mask: 0x$('{0:X}' -f [int]$step2)" -ForegroundColor Yellow
            Write-Host "  This means step pulses are ACTIVE LOW instead of ACTIVE HIGH" -ForegroundColor Yellow
        }
    }
    
    if ($null -ne $step4) {
        Write-Host "`$4 (Enable Invert) = $step4" -ForegroundColor $(if ($step4 -eq 0) { "Green" } else { "Cyan" })
        if ($step4 -eq 0) {
            Write-Host "  Enable is ACTIVE LOW (pin LOW = enabled)" -ForegroundColor Gray
        } else {
            Write-Host "  Enable is ACTIVE HIGH (pin HIGH = enabled), Bit mask: 0x$('{0:X}' -f [int]$step4)" -ForegroundColor Cyan
        }
    }
    
    if ($null -ne $step5) {
        Write-Host "`$5 (Limit Invert) = $step5" -ForegroundColor Green
    }
    
    Write-Host "`n=== STEP 2: Check Current Status ===" -ForegroundColor Cyan
    $status = Send-GcodeCommand -serial $serialPort -command '?' -timeout 1000
    
    Write-Host "`n=== STEP 3: Test Simple X-Axis Motion ===" -ForegroundColor Cyan
    Write-Host "Sending: G1 X10 F100 (move 10mm in X at 100mm/min)" -ForegroundColor Gray
    
    $moveResponse = Send-GcodeCommand -serial $serialPort -command 'G1 X10 F100' -timeout 2000
    
    Start-Sleep -Milliseconds 500
    
    Write-Host "`n=== STEP 4: Check Status During/After Motion ===" -ForegroundColor Cyan
    for ($i = 0; $i -lt 3; $i++) {
        $status = Send-GcodeCommand -serial $serialPort -command '?' -timeout 1000
        Start-Sleep -Milliseconds 500
    }
    
    Write-Host "`n=== DIAGNOSTIC SUMMARY ===" -ForegroundColor Cyan
    Write-Host "1. Check the settings above" -ForegroundColor White
    Write-Host "   - If `$2 != 0: Step pulses are inverted (ISR doesn't handle this!)" -ForegroundColor White
    Write-Host "   - If `$4 != 0: Enable pins are active-high (check driver)" -ForegroundColor White
    Write-Host ""
    Write-Host "2. Check status responses" -ForegroundColor White
    Write-Host "   - Position should change from 0.000 to 10.000" -ForegroundColor White
    Write-Host "   - If position stays 0.000: Motors not moving" -ForegroundColor White
    Write-Host ""
    Write-Host "3. Hardware checks" -ForegroundColor White
    Write-Host "   - Verify stepper driver enable pin is at correct voltage" -ForegroundColor White
    Write-Host "   - Verify step pulses appear on RD4 (scope/logic analyzer)" -ForegroundColor White
    Write-Host "   - Check if LED1 toggles rapidly (confirms ISR firing)" -ForegroundColor White
    Write-Host ""
    
} catch {
    Write-Host "ERROR: $_" -ForegroundColor Red
} finally {
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
        Write-Host "`nSerial port closed." -ForegroundColor Gray
    }
}

Write-Host "`nDiagnostic complete!" -ForegroundColor Cyan
