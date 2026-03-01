param(
    [string]$Port    = "COM11",
    [int]   $Baud    = 115200,
    [string]$Command = "G1X50F600"
)

# ─────────────────────────────────────────────────────────────────────────────
# debug_accel.ps1  — capture [STEPPER_Load] + position polls for one move
# Usage:  .\ps_commands\debug_accel.ps1 -Port COM11
# ─────────────────────────────────────────────────────────────────────────────

function Try-Port($p) {
    try {
        $s = New-Object System.IO.Ports.SerialPort $p, 115200
        $s.Open(); $s.Close(); return $true
    } catch { return $false }
}

# Auto-detect if default port is closed
if (-not (Try-Port $Port)) {
    foreach ($alt in "COM11","COM12","COM3","COM4","COM5") {
        if ((Try-Port $alt)) { $Port = $alt; break }
    }
}

Write-Host "Connecting to $Port …" -ForegroundColor Cyan

$sp = New-Object System.IO.Ports.SerialPort
$sp.PortName  = $Port
$sp.BaudRate  = $Baud
$sp.ReadTimeout  = 200
$sp.WriteTimeout = 2000
$sp.NewLine   = "`r`n"

try { $sp.Open() } catch {
    Write-Host "ERROR: cannot open $Port — close VS Code Serial Monitor first, or specify -Port COMx" -ForegroundColor Red
    exit 1
}

$collected = [System.Collections.Generic.List[string]]::new()

function Read-All {
    $buf = ""
    try {
        while ($true) { $buf += $sp.ReadExisting(); Start-Sleep -Milliseconds 50 }
    } catch {}
    return $buf
}

function Send($line) {
    $sp.WriteLine($line)
    Start-Sleep -Milliseconds 100
    $r = Read-All
    if ($r.Trim()) { foreach($l in ($r -split "`n")) { if($l.Trim()) {
        $collected.Add($l.TrimEnd())
        Write-Host "  $($l.TrimEnd())" -ForegroundColor Gray
    }}}
}

Write-Host "`n── Handshake ──" -ForegroundColor Yellow
Send "?"
Start-Sleep -Milliseconds 300

Write-Host "`n── Reset position ──" -ForegroundColor Yellow
Send "`$X"           # clear alarm
Send "G92 X0 Y0 Z0"  # zero work coords
Send "?"

Write-Host "`n── Sending: $Command ──" -ForegroundColor Yellow
$sp.WriteLine($Command)   # don't wait — start polling immediately

Write-Host "`n── Polling position every 250 ms ──" -ForegroundColor Yellow
$polls = 0
$deadline = (Get-Date).AddSeconds(12)
while ((Get-Date) -lt $deadline) {
    $sp.Write("?")
    Start-Sleep -Milliseconds 250
    $r = $sp.ReadExisting()
    if ($r.Trim()) {
        foreach($l in ($r -split "\n")) {
            $l = $l.TrimEnd()
            if ($l) {
                $collected.Add($l)
                $color = if ($l -match "STEPPER_Load|initial_rate|accel_count") { "Green" }
                         elseif ($l -match "Run|Idle") { "Cyan" }
                         else { "White" }
                Write-Host "  $l" -ForegroundColor $color
            }
        }
    }
    $polls++
    if ($polls -gt 60) { break }
}

$sp.Close()

Write-Host "`n═══ SUMMARY OF STEPPER_Load LINES ═══" -ForegroundColor Yellow
$collected | Where-Object { $_ -match "STEPPER_Load|initial_rate" } | ForEach-Object { Write-Host $_ -ForegroundColor Green }

Write-Host "`nDone." -ForegroundColor Cyan
