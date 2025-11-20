# Test Z-axis direction control
# This script sends commands to verify Z direction pin toggling

param(
    [string]$Port = "COM3",
    [int]$BaudRate = 115200
)

Write-Host "=== Z-Axis Direction Test ===" -ForegroundColor Cyan
Write-Host "Port: $Port, Baud: $BaudRate`n"

# Open serial port
$serialPort = New-Object System.IO.Ports.SerialPort
$serialPort.PortName = $Port
$serialPort.BaudRate = $BaudRate
$serialPort.DataBits = 8
$serialPort.Parity = 'None'
$serialPort.StopBits = 1
$serialPort.Handshake = 'None'
$serialPort.ReadTimeout = 1000
$serialPort.WriteTimeout = 1000

try {
    $serialPort.Open()
    Write-Host "✓ Serial port opened successfully`n" -ForegroundColor Green
    
    Start-Sleep -Milliseconds 500
    
    # Function to send command and read response
    function Send-Command {
        param([string]$cmd)
        Write-Host "→ $cmd" -ForegroundColor Yellow
        $serialPort.WriteLine($cmd)
        Start-Sleep -Milliseconds 100
        
        $response = ""
        $timeout = 10
        while ($timeout -gt 0) {
            try {
                $line = $serialPort.ReadLine()
                $response += $line + "`n"
                Write-Host "← $line" -ForegroundColor Gray
                if ($line -match "ok|error") { break }
            } catch {
                break
            }
            $timeout--
        }
        Start-Sleep -Milliseconds 50
        return $response
    }
    
    Write-Host "`n=== Test 1: Check Current Settings ===" -ForegroundColor Cyan
    Send-Command "$$" | Out-Null
    
    Write-Host "`n=== Test 2: Move Z Up (should be positive direction) ===" -ForegroundColor Cyan
    Send-Command "`$3=5"
    Send-Command "0x18"  # Soft reset
    Start-Sleep -Milliseconds 500
    Send-Command "G91"   # Relative positioning
    Send-Command "G0Z10F500"  # Move Z up 10mm
    Start-Sleep -Milliseconds 2000
    Send-Command "?"
    
    Write-Host "`n=== Test 3: Change direction and move Z Up again ===" -ForegroundColor Cyan
    Send-Command "`$3=1"
    Send-Command "0x18"  # Soft reset
    Start-Sleep -Milliseconds 500
    Send-Command "G91"   # Relative positioning
    Send-Command "G0Z10F500"  # Move Z up 10mm (should go opposite direction)
    Start-Sleep -Milliseconds 2000
    Send-Command "?"
    
    Write-Host "`n=== Test 4: Restore original and move Z Down ===" -ForegroundColor Cyan
    Send-Command "`$3=5"
    Send-Command "0x18"  # Soft reset
    Start-Sleep -Milliseconds 500
    Send-Command "G91"   # Relative positioning
    Send-Command "G0Z-10F500"  # Move Z down 10mm
    Start-Sleep -Milliseconds 2000
    Send-Command "?"
    
    Write-Host "`n=== Test Complete ===" -ForegroundColor Green
    Write-Host "Observations:" -ForegroundColor Cyan
    Write-Host "- Test 2 ($$3=5): Z should move one direction"
    Write-Host "- Test 3 ($$3=1): Z should move OPPOSITE direction"
    Write-Host "- Test 4 ($$3=5, negative Z): Z should move opposite of Test 2"
    Write-Host "`nIf Z always moves the same direction, the problem is likely:"
    Write-Host "  1. Hardware: DirZ pin not connected to driver"
    Write-Host "  2. Driver: Direction input not working"
    Write-Host "  3. Wiring: DirZ signal not reaching stepper driver"
    
} catch {
    Write-Host "Error: $_" -ForegroundColor Red
} finally {
    if ($serialPort.IsOpen) {
        $serialPort.Close()
        Write-Host "`n✓ Serial port closed" -ForegroundColor Green
    }
}
