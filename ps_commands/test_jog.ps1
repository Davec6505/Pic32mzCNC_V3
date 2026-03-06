# Software Jog Test Script — Phase 5a
# Tests $J= command, 0x85 cancel, and <Jog|...> status reporting
# Usage: .\test_jog.ps1 -Port COM3

param(
    [string]$Port     = "COM3",
    [int]$BaudRate    = 115200,
    [float]$JogDist   = 1.0,      # mm per jog step (default 1mm)
    [float]$JogFeed   = 500.0,    # mm/min feedrate for jog steps
    [int]$PollMs      = 150       # Status query interval during jog
)

# ── Colour helpers ─────────────────────────────────────────────────────────────
function Write-Pass  ([string]$msg) { Write-Host "  [PASS] $msg" -ForegroundColor Green }
function Write-Fail  ([string]$msg) { Write-Host "  [FAIL] $msg" -ForegroundColor Red }
function Write-Info  ([string]$msg) { Write-Host "  [INFO] $msg" -ForegroundColor Cyan }
function Write-Step  ([string]$msg) { Write-Host "`n--- $msg ---" -ForegroundColor Yellow }

Write-Host "=== Software Jog Test (Phase 5a) ===" -ForegroundColor Cyan
Write-Host "Port: $Port @ $BaudRate baud | Dist: ${JogDist}mm | Feed: ${JogFeed}mm/min" -ForegroundColor DarkGray
Write-Host ""

# ── Open port ──────────────────────────────────────────────────────────────────
try {
    $sp = New-Object System.IO.Ports.SerialPort
    $sp.PortName    = $Port
    $sp.BaudRate    = $BaudRate
    $sp.DataBits    = 8
    $sp.Parity      = [System.IO.Ports.Parity]::None
    $sp.StopBits    = [System.IO.Ports.StopBits]::One
    $sp.ReadTimeout  = 300
    $sp.WriteTimeout = 2000
    $sp.Open()
    Write-Host "Port opened." -ForegroundColor Green
} catch {
    Write-Host "ERROR: Cannot open $Port — $_" -ForegroundColor Red
    exit 1
}

# ── Helpers ────────────────────────────────────────────────────────────────────
function Send-Line([string]$line) {
    Write-Host "  TX: $line" -ForegroundColor DarkGray
    $sp.Write($line + "`r`n")
}

function Send-Byte([byte]$b) {
    $buf = [byte[]]@($b)
    $sp.Write($buf, 0, 1)
}

function Read-Available([int]$waitMs = 120) {
    Start-Sleep -Milliseconds $waitMs
    $data = ""
    try { while ($sp.BytesToRead -gt 0) { $data += $sp.ReadExisting() } } catch {}
    return $data.Trim()
}

function Wait-Idle([int]$timeoutSec = 10) {
    $deadline = (Get-Date).AddSeconds($timeoutSec)
    while ((Get-Date) -lt $deadline) {
        $sp.Write("?")
        Start-Sleep -Milliseconds $PollMs
        $r = ""
        try { while ($sp.BytesToRead -gt 0) { $r += $sp.ReadExisting() } } catch {}
        $r = $r.Trim()
        if ($r) { Write-Host "  >> $r" -ForegroundColor DarkGray }
        if ($r -match "<Idle") { return $true }
    }
    return $false
}

# ── Soft reset ─────────────────────────────────────────────────────────────────
Write-Step "1  Soft reset and wait for Idle"
Send-Byte 0x18
Start-Sleep -Milliseconds 1200
$banner = Read-Available 300
if ($banner) { Write-Info "Banner: $banner" }

$sp.Write("?")
$state = Read-Available 250
Write-Info "Initial state: $state"
if ($state -match "<Alarm") {
    Write-Info "Machine in ALARM — sending `$X to clear..."
    Send-Line "`$X"
    $r = Read-Available 300
    Write-Info "After `$X: $r"
}

# ── Test 1: $J= with bad params (expect error) ─────────────────────────────────
Write-Step "2  Error handling — bad/missing parameters"

Send-Line "`$J=G91X1"          # missing F → error:4
$r = Read-Available
if ($r -match "error:4") { Write-Pass "Missing F → error:4 ($r)" }
else                      { Write-Fail "Expected error:4, got: $r" }

Send-Line "`$J=G91F500"        # no axis word → error:4
$r = Read-Available
if ($r -match "error:4") { Write-Pass "No axis → error:4 ($r)" }
else                      { Write-Fail "Expected error:4, got: $r" }

Send-Line "`$J=G91G17X1F500"   # unknown modal (G17 not allowed in $J) → error:4
$r = Read-Available
if ($r -match "error:4") { Write-Pass "Unknown modal → error:4 ($r)" }
else                      { Write-Fail "Expected error:4, got: $r" }

# ── Test 2: Single jog +X incremental ──────────────────────────────────────────
Write-Step "3  Single incremental jog +X ${JogDist}mm"

Send-Line "`$J=G91X${JogDist}F${JogFeed}"
$r = Read-Available 100
if ($r -match "^ok") { Write-Pass "Got ok" }
else                  { Write-Fail "Expected ok, got: $r" }

# Poll status — should see Jog at least once
$sawJog  = $false
$sawIdle = $false
$deadline = (Get-Date).AddSeconds(8)
while ((Get-Date) -lt $deadline -and -not $sawIdle) {
    $sp.Write("?")
    Start-Sleep -Milliseconds $PollMs
    $raw = ""
    try { while ($sp.BytesToRead -gt 0) { $raw += $sp.ReadExisting() } } catch {}
    $raw = $raw.Trim()
    if ($raw) { Write-Host "  >> $raw" -ForegroundColor DarkGray }
    if ($raw -match "<Jog")  { $sawJog  = $true }
    if ($raw -match "<Idle") { $sawIdle = $true }
}
if ($sawJog)  { Write-Pass "Status reported <Jog|...> during jog" }
else          { Write-Fail "Never saw <Jog|...> state during jog" }
if ($sawIdle) { Write-Pass "Returned to <Idle> after jog" }
else          { Write-Fail "Machine did not return to Idle within 8s" }

# ── Test 3: Jog -X back to origin area ────────────────────────────────────────
Write-Step "4  Single incremental jog -X ${JogDist}mm"

Send-Line "`$J=G91X-${JogDist}F${JogFeed}"
$r = Read-Available 100
if ($r -match "^ok") { Write-Pass "Got ok" }
else                  { Write-Fail "Expected ok, got: $r" }
if (-not (Wait-Idle 8)) { Write-Fail "Machine did not return to Idle within 8s" }
else                     { Write-Pass "Returned to Idle" }

# ── Test 4: Multi-axis jog ─────────────────────────────────────────────────────
Write-Step "5  Multi-axis incremental jog X+${JogDist} Y+${JogDist}"

Send-Line "`$J=G91X${JogDist}Y${JogDist}F${JogFeed}"
$r = Read-Available 100
if ($r -match "^ok") { Write-Pass "Got ok" }
else                  { Write-Fail "Expected ok, got: $r" }
if (-not (Wait-Idle 8)) { Write-Fail "Timeout" }
else                     { Write-Pass "Returned to Idle" }

# Return
Send-Line "`$J=G91X-${JogDist}Y-${JogDist}F${JogFeed}"
Read-Available 100 | Out-Null
Wait-Idle 8 | Out-Null

# ── Test 5: 0x85 cancel of in-progress jog ────────────────────────────────────
Write-Step "6  0x85 jog cancel — start long jog, cancel mid-move"

# Long jog so we have time to cancel
Send-Line "`$J=G91X50F${JogFeed}"
$r = Read-Available 100
if ($r -match "^ok") { Write-Pass "Long jog accepted (ok)" }
else                  { Write-Fail "Expected ok, got: $r" }

Start-Sleep -Milliseconds 300   # let it start moving

# Send 0x85 cancel byte
Write-Host "  TX: [0x85] Jog Cancel" -ForegroundColor DarkGray
Send-Byte 0x85

# 0x85 must NOT produce any response; machine must stop and return Idle
$cancelResponse = Read-Available 300
if ($cancelResponse -match "^ok|error") {
    Write-Fail "0x85 generated an unexpected response: $cancelResponse"
} else {
    Write-Pass "0x85 produced no response (GRBL v1.1 compliant)"
}

# Confirm Idle
$idleAfterCancel = Wait-Idle 5
if ($idleAfterCancel) { Write-Pass "Machine returned to Idle after cancel" }
else                   { Write-Fail "Machine did not go Idle after 0x85 cancel" }

# Return to near-zero (machine may have moved some distance before cancel)
Write-Info "Jogging back toward X0 Y0 (absolute)..."
Send-Line "`$J=G90X0Y0F${JogFeed}"
$r = Read-Available 100
Write-Host "  >> $r" -ForegroundColor DarkGray
Wait-Idle 15 | Out-Null

# ── Test 6: Consecutive rapid jog commands (chaining) ─────────────────────────
Write-Step "7  Consecutive jog commands — chaining test"

$d = $JogDist / 2.0
for ($i = 0; $i -lt 4; $i++) {
    Send-Line "`$J=G91X${d}F${JogFeed}"
    Start-Sleep -Milliseconds 60    # send before previous jog finishes
}
$r = Read-Available 60
Write-Info "Last reply: $r"
$idle = Wait-Idle 12
if ($idle) { Write-Pass "Chained jog sequence completed cleanly" }
else        { Write-Fail "Chain jog did not complete within 12s" }

# Return
Send-Line "`$J=G90X0Y0F${JogFeed}"
Read-Available 100 | Out-Null
Wait-Idle 12 | Out-Null

# ── Test 7: Query during jog confirms <Jog|...> state ─────────────────────────
Write-Step "8  Status query reports <Jog|...> machine state"

Send-Line "`$J=G91X20F${JogFeed}"
Read-Available 60 | Out-Null   # consume ok

$jogStatusSeen = $false
for ($i = 0; $i -lt 15; $i++) {
    $sp.Write("?")
    Start-Sleep -Milliseconds $PollMs
    $raw = ""
    try { while ($sp.BytesToRead -gt 0) { $raw += $sp.ReadExisting() } } catch {}
    if ($raw -match "<Jog") { $jogStatusSeen = $true; write-host "  >> $raw" -ForegroundColor DarkGray; break }
    if ($raw -match "<Idle") { break }
}
if ($jogStatusSeen) { Write-Pass "Status correctly reports <Jog|...> during jog" }
else                 { Write-Fail "Never observed <Jog|...> in status query" }

Wait-Idle 15 | Out-Null

# ── Summary ────────────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "=== Jog Test Complete ===" -ForegroundColor Cyan
Write-Host "If all [PASS] — Phase 5a (Software Jog) validated on hardware." -ForegroundColor Green
Write-Host ""

$sp.Close()
