# Test script for acceleration/deceleration profiling
# Visual demonstration of trapezoidal velocity profiling

param(
    [string]$PortName = "COM4",
    [int]$BaudRate = 115200
)

Write-Host "`n=====================================" -ForegroundColor Cyan
Write-Host "  ACCELERATION/DECELERATION TEST" -ForegroundColor Cyan
Write-Host "=====================================" -ForegroundColor Cyan

# Open serial port
$port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, None, 8, one
$port.ReadTimeout = 2000
$port.WriteTimeout = 2000

try {
    $port.Open()
    Write-Host "`n[OK] Connected to $PortName" -ForegroundColor Green
    Start-Sleep -Milliseconds 500
    
    # Function to send command and wait for response
    function Send-GCode {
        param([string]$cmd, [int]$waitMs = 200)
        
        Write-Host "`n>>> $cmd" -ForegroundColor Yellow
        $port.WriteLine($cmd)
        Start-Sleep -Milliseconds $waitMs
        
        $response = ""
        while($port.BytesToRead -gt 0) {
            $response += $port.ReadExisting()
        }
        
        if($response) {
            Write-Host $response -NoNewline -ForegroundColor Gray
        }
    }
    
    # Function to watch status during motion
    function Watch-Motion {
        param([int]$durationMs = 3000, [int]$pollInterval = 200)
        
        $endTime = (Get-Date).AddMilliseconds($durationMs)
        $motionSeen = $false
        
        Write-Host "`n[WATCHING MOTION...]" -ForegroundColor Magenta
        
        while((Get-Date) -lt $endTime) {
            # Send status query
            $port.WriteLine("?")
            Start-Sleep -Milliseconds $pollInterval
            
            # Read response
            $status = ""
            while($port.BytesToRead -gt 0) {
                $status = $port.ReadExisting()
            }
            
            if($status -match "<(\w+)\|MPos:([\d.]+),([\d.]+),([\d.]+)\|WPos:([\d.]+),([\d.]+),([\d.]+)\|FS:([\d.]+),(\d+)>") {
                $state = $matches[1]
                $mx = [math]::Round([double]$matches[2], 2)
                $my = [math]::Round([double]$matches[3], 2)
                $mz = [math]::Round([double]$matches[4], 2)
                $feedrate = [int]$matches[8]
                
                # Color code based on state
                if($state -eq "Run") {
                    Write-Host "  [$state] X:$mx Y:$my Z:$mz F:$feedrate" -ForegroundColor Green
                    $motionSeen = $true
                } elseif($state -eq "Idle" -and $motionSeen) {
                    Write-Host "  [$state] X:$mx Y:$my Z:$mz F:$feedrate" -ForegroundColor Cyan
                    break  # Motion complete
                } else {
                    Write-Host "  [$state] X:$mx Y:$my Z:$mz F:$feedrate" -ForegroundColor DarkGray
                }
            }
        }
    }
    
    Write-Host "`n" + "="*60 -ForegroundColor Cyan
    Write-Host "TEST 1: LONG MOVE - Full Trapezoid (Accel/Cruise/Decel)" -ForegroundColor Cyan
    Write-Host "="*60 -ForegroundColor Cyan
    Write-Host "Command: G1 X100 F1000" -ForegroundColor White
    Write-Host "Expected: Smooth acceleration, cruise at 1000mm/min, smooth deceleration" -ForegroundColor White
    
    Send-GCode "G92 X0 Y0 Z0"  # Set origin
    Send-GCode "G1 X100 F1000"  # Long move with trapezoid
    Watch-Motion -durationMs 8000 -pollInterval 150
    
    Write-Host "`n" + "="*60 -ForegroundColor Cyan
    Write-Host "TEST 2: SHORT MOVE - Triangle Profile (No Cruise)" -ForegroundColor Cyan
    Write-Host "="*60 -ForegroundColor Cyan
    Write-Host "Command: G1 X5 F1000" -ForegroundColor White
    Write-Host "Expected: Accelerate then immediately decelerate (triangle profile)" -ForegroundColor White
    
    Send-GCode "G92 X0"  # Reset X
    Send-GCode "G1 X5 F1000"  # Short move - triangle profile
    Watch-Motion -durationMs 2000 -pollInterval 100
    
    Write-Host "`n" + "="*60 -ForegroundColor Cyan
    Write-Host "TEST 3: SLOW MOVE - Low Feedrate" -ForegroundColor Cyan
    Write-Host "="*60 -ForegroundColor Cyan
    Write-Host "Command: G1 X20 F100" -ForegroundColor White
    Write-Host "Expected: Gentle acceleration, slow cruise, gentle deceleration" -ForegroundColor White
    
    Send-GCode "G92 X0"
    Send-GCode "G1 X20 F100"
    Watch-Motion -durationMs 15000 -pollInterval 200
    
    Write-Host "`n" + "="*60 -ForegroundColor Cyan
    Write-Host "TEST 4: FAST MOVE - High Feedrate" -ForegroundColor Cyan
    Write-Host "="*60 -ForegroundColor Cyan
    Write-Host "Command: G1 X50 F3000" -ForegroundColor White
    Write-Host "Expected: Rapid acceleration, fast cruise, rapid deceleration" -ForegroundColor White
    
    Send-GCode "G92 X0"
    Send-GCode "G1 X50 F3000"
    Watch-Motion -durationMs 4000 -pollInterval 100
    
    Write-Host "`n" + "="*60 -ForegroundColor Cyan
    Write-Host "TEST 5: DIAGONAL MOVE - Multi-Axis Coordination" -ForegroundColor Cyan
    Write-Host "="*60 -ForegroundColor Cyan
    Write-Host "Command: G1 X30 Y30 F1000" -ForegroundColor White
    Write-Host "Expected: Coordinated XY motion with smooth velocity profile" -ForegroundColor White
    
    Send-GCode "G92 X0 Y0"
    Send-GCode "G1 X30 Y30 F1000"
    Watch-Motion -durationMs 5000 -pollInterval 150
    
    Write-Host "`n" + "="*60 -ForegroundColor Cyan
    Write-Host "TEST 6: RETURN TO ORIGIN - Negative Direction" -ForegroundColor Cyan
    Write-Host "="*60 -ForegroundColor Cyan
    Write-Host "Command: G1 X0 Y0 F1000" -ForegroundColor White
    Write-Host "Expected: Same acceleration profile in reverse direction" -ForegroundColor White
    
    Send-GCode "G1 X0 Y0 F1000"
    Watch-Motion -durationMs 5000 -pollInterval 150
    
    Write-Host "`n" + "="*60 -ForegroundColor Green
    Write-Host "ALL TESTS COMPLETE!" -ForegroundColor Green
    Write-Host "="*60 -ForegroundColor Green
    
    Write-Host "`nObservations:" -ForegroundColor Cyan
    Write-Host "  - Long moves should show Run state for extended period" -ForegroundColor White
    Write-Host "  - Short moves transition quickly from Run to Idle" -ForegroundColor White
    Write-Host "  - Position should update smoothly during motion" -ForegroundColor White
    Write-Host "  - Feedrate should match commanded value during cruise" -ForegroundColor White
    
    Write-Host "`nNote: If you see jerky motion or position errors," -ForegroundColor Yellow
    Write-Host "      the acceleration/deceleration calculations may need tuning." -ForegroundColor Yellow
    
} catch {
    Write-Host "`n[ERROR] $($_.Exception.Message)" -ForegroundColor Red
} finally {
    if($port.IsOpen) {
        $port.Close()
        Write-Host "`n[OK] Port closed" -ForegroundColor Green
    }
}

Write-Host "`nPress any key to exit..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
