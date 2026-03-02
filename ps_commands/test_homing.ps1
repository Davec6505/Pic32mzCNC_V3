# Homing Test Script - Monitors $H cycle with status queries
# Usage: .\test_homing.ps1 -Port COM3

param(
    [string]$Port    = "COM3",
    [int]$BaudRate   = 115200,
    [int]$PollMs     = 300     # Status query interval in milliseconds
)

Write-Host "=== Homing Monitor ===" -ForegroundColor Cyan
Write-Host "Port: $Port @ $BaudRate baud | Poll: ${PollMs}ms" -ForegroundColor Yellow
Write-Host ""

try {
    $sp = New-Object System.IO.Ports.SerialPort
    $sp.PortName   = $Port
    $sp.BaudRate   = $BaudRate
    $sp.DataBits   = 8
    $sp.Parity     = [System.IO.Ports.Parity]::None
    $sp.StopBits   = [System.IO.Ports.StopBits]::One
    $sp.ReadTimeout  = 200
    $sp.WriteTimeout = 2000
    $sp.Open()
    Write-Host "Port opened." -ForegroundColor Green
} catch {
    Write-Host "ERROR: Cannot open $Port - $_" -ForegroundColor Red
    exit 1
}

function Send-Line([string]$line) {
    $sp.Write($line + "`r`n")
}

function Send-Raw([string]$raw) {
    $sp.Write($raw)
}

function Read-Available {
    $data = ""
    try {
        Start-Sleep -Milliseconds 80
        while ($sp.BytesToRead -gt 0) {
            $data += $sp.ReadExisting()
        }
    } catch { }
    return $data.Trim()
}

# ── Soft-reset first ──────────────────────────────────────────────────────────
Write-Host "[*] Sending Ctrl+X (soft reset)..." -ForegroundColor Gray
$sp.Write([char]0x18)
Start-Sleep -Milliseconds 1200
$banner = Read-Available
if ($banner) { Write-Host "  Banner: $banner" -ForegroundColor DarkGray }

# ── Send $22=7 to ensure all 3 axes enabled, then $H ─────────────────────────
Write-Host "[*] Setting `$22=7 (enable X+Y+Z homing)..." -ForegroundColor Gray
Send-Line "`$22=7"
Start-Sleep -Milliseconds 400
$r = Read-Available
if ($r) { Write-Host "  > $r" -ForegroundColor DarkGray }

Write-Host "[*] Sending `$H..." -ForegroundColor Yellow
Send-Line "`$H"
Start-Sleep -Milliseconds 200

# ── Monitoring loop ───────────────────────────────────────────────────────────
$start    = Get-Date
$lastState = ""
$iteration = 0

Write-Host ""
Write-Host "===  Live Status  ===" -ForegroundColor Cyan
Write-Host "(Press CTRL+C to stop)" -ForegroundColor DarkGray
Write-Host ""

try {
    while ($true) {
        $iteration++
        $elapsed = [int]((Get-Date) - $start).TotalSeconds

        # Send status query
        $sp.Write("?")
        Start-Sleep -Milliseconds $PollMs

        # Read all available data
        $raw = ""
        try {
            while ($sp.BytesToRead -gt 0) {
                $raw += $sp.ReadExisting()
            }
        } catch { }

        if ($raw.Trim()) {
            $lines = $raw -split "`r?`n" | Where-Object { $_.Trim() -ne "" }
            foreach ($line in $lines) {
                $ts = "[{0,4}s]" -f $elapsed

                # Colour-code by content
                if ($line -match "<Idle") {
                    if ($lastState -ne "Idle") {
                        Write-Host "$ts $line" -ForegroundColor Green
                        $lastState = "Idle"
                    } else {
                        Write-Host "$ts $line" -ForegroundColor DarkGreen
                    }
                } elseif ($line -match "<Home") {
                    Write-Host "$ts $line" -ForegroundColor Cyan
                    $lastState = "Home"
                } elseif ($line -match "<Run") {
                    Write-Host "$ts $line" -ForegroundColor Yellow
                    $lastState = "Run"
                } elseif ($line -match "ALARM") {
                    Write-Host "$ts $line" -ForegroundColor Red
                    Write-Host ""
                    Write-Host ">>> ALARM detected - homing failed! <<<" -ForegroundColor Red
                    break
                } elseif ($line -match "^ok") {
                    Write-Host "$ts $line" -ForegroundColor DarkGray
                } elseif ($line -match "error") {
                    Write-Host "$ts $line" -ForegroundColor Magenta
                } else {
                    Write-Host "$ts $line" -ForegroundColor Gray
                }
            }
        }

        # Stop if Idle after homing started (Home or Run seen)
        if ($lastState -eq "Idle" -and $iteration -gt 5) {
            Write-Host ""
            Write-Host ">>> Machine returned to Idle after ${elapsed}s <<<" -ForegroundColor Green
            break
        }

        # Timeout after 120 seconds
        if ($elapsed -gt 120) {
            Write-Host ""
            Write-Host ">>> Timeout (120s) - homing did not complete <<<" -ForegroundColor Red
            break
        }
    }
} catch [System.Management.Automation.PipelineStoppedException] {
    Write-Host "`nStopped by user." -ForegroundColor Yellow
} finally {
    if ($sp -and $sp.IsOpen) { $sp.Close() }
    Write-Host "Port closed." -ForegroundColor DarkGray
}
