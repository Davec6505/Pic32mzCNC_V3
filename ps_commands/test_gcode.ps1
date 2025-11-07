# PowerShell G-code Test Script for PIC32MZ CNC
# Usage: .\test_gcode.ps1 -Port COM3 -BaudRate 115200

param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200,
    [switch]$Help
)

if ($Help) {
    Write-Host "PowerShell G-code Test Script"
    Write-Host "Usage: .\test_gcode.ps1 -Port COM3 -BaudRate 115200"
    Write-Host ""
    Write-Host "Parameters:"
    Write-Host "  -Port       Serial port name (default: COM3)"
    Write-Host "  -BaudRate   Baud rate (default: 115200)"
    Write-Host "  -Help       Show this help message"
    exit
}

# Test commands
$testCommands = @(
    "",  # Empty line (should get banner after reset)
    "G92 X0 Y0 Z0",  # Set work coordinates to zero
    "?",  # Status query
    "G1 X10 Y5 F100",  # Linear move
    "?",  # Status query
    "G1 X0 Y0 F100",  # Return to origin
    "?"  # Status query
)

Write-Host "=== PIC32MZ CNC G-code Test Script ===" -ForegroundColor Cyan
Write-Host "Port: $Port @ $BaudRate baud" -ForegroundColor Yellow
Write-Host ""

try {
    # Open serial port
    $serialPort = New-Object System.IO.Ports.SerialPort
    $serialPort.PortName = $Port
    $serialPort.BaudRate = $BaudRate
    $serialPort.DataBits = 8
    $serialPort.Parity = [System.IO.Ports.Parity]::None
    $serialPort.StopBits = [System.IO.Ports.StopBits]::One
    $serialPort.ReadTimeout = 2000
    $serialPort.WriteTimeout = 1000
    $serialPort.NewLine = "`r`n"
    
    Write-Host "Opening port $Port..." -ForegroundColor Green
    $serialPort.Open()
    
    if ($serialPort.IsOpen) {
        Write-Host "Port opened successfully!" -ForegroundColor Green
        Write-Host ""
        
        # Wait for startup banner
        Start-Sleep -Milliseconds 500
        
        # Read any startup messages
        while ($serialPort.BytesToRead -gt 0) {
            $line = $serialPort.ReadLine()
            Write-Host "<<< $line" -ForegroundColor Gray
        }
        
        Write-Host ""
        Write-Host "=== Starting Test Sequence ===" -ForegroundColor Cyan
        Write-Host ""
        
        # Send test commands
        foreach ($cmd in $testCommands) {
            if ($cmd -eq "") {
                continue
            }
            
            Write-Host ">>> $cmd" -ForegroundColor Yellow
            $serialPort.WriteLine($cmd)
            
            # Wait for response
            Start-Sleep -Milliseconds 200
            
            # Read responses (may be multiple lines)
            $responseLines = @()
            $startTime = Get-Date
            $timeout = 3  # 3 second timeout
            
            while (((Get-Date) - $startTime).TotalSeconds -lt $timeout) {
                if ($serialPort.BytesToRead -gt 0) {
                    try {
                        $line = $serialPort.ReadLine()
                        $responseLines += $line
                        Write-Host "<<< $line" -ForegroundColor Cyan
                        
                        # If we got "OK" or status response, command is done
                        if ($line -match "^OK" -or $line -match "^<.*>") {
                            Start-Sleep -Milliseconds 100  # Small delay for any trailing output
                            if ($serialPort.BytesToRead -eq 0) {
                                break
                            }
                        }
                    }
                    catch {
                        # Timeout reading line
                        break
                    }
                }
                else {
                    Start-Sleep -Milliseconds 50
                }
            }
            
            # Check for "OK" response
            if ($responseLines -contains "OK") {
                Write-Host "[PASS] Command acknowledged" -ForegroundColor Green
            }
            elseif ($responseLines | Where-Object { $_ -match "^<.*>" }) {
                Write-Host "[INFO] Status response received" -ForegroundColor Blue
            }
            else {
                Write-Host "[WARN] No OK response" -ForegroundColor Magenta
            }
            
            Write-Host ""
            Start-Sleep -Milliseconds 500
        }
        
        Write-Host "=== Test Complete ===" -ForegroundColor Cyan
        
    }
    else {
        Write-Host "Failed to open port $Port" -ForegroundColor Red
        exit 1
    }
}
catch {
    Write-Host "Error: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
finally {
    if ($serialPort -and $serialPort.IsOpen) {
        Write-Host ""
        Write-Host "Closing port..." -ForegroundColor Yellow
        $serialPort.Close()
        $serialPort.Dispose()
        Write-Host "Port closed." -ForegroundColor Green
    }
}
