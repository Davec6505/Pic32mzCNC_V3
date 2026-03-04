param(
    [string]$ComPort = "COM11",
    [int]$BaudRate = 115200,
    [string]$FilePath = "tests\04_arc_test.gcode",
    [int]$OkTimeout = 15000
)

$port = New-Object System.IO.Ports.SerialPort $ComPort, $BaudRate, None, 8, One
$port.ReadTimeout = 1000
$port.Open()
Start-Sleep -Milliseconds 300

# Soft reset
$port.Write([byte[]](0x18), 0, 1)
Start-Sleep -Milliseconds 600
Write-Host "--- Reset: $($port.ReadExisting().Trim()) ---" -ForegroundColor Magenta

function WaitForOk {
    param([int]$ms = $OkTimeout)
    $deadline = (Get-Date).AddMilliseconds($ms)
    $buf = ""
    while ((Get-Date) -lt $deadline) {
        try {
            $line = $port.ReadLine().Trim()
            if ($line) { Write-Host "    $line" -ForegroundColor Cyan }
            $buf += $line
            if ($line -eq "ok") { return $true }
            if ($line -match "^error") { Write-Host "    [ERROR] $line" -ForegroundColor Red; return $false }
        } catch { Start-Sleep -Milliseconds 20 }
    }
    Write-Host "    [TIMEOUT]" -ForegroundColor Red
    return $false
}

$lines = Get-Content $FilePath | Where-Object { $_ -notmatch '^\s*;' -and $_.Trim() -ne '' }
$n = 0
Write-Host "=== Streaming: $FilePath ===" -ForegroundColor Green

foreach ($line in $lines) {
    $cmd = ($line -split ';')[0].Trim()
    if (-not $cmd) { continue }
    $n++
    Write-Host "[$n] >>> $cmd" -ForegroundColor Yellow
    $port.WriteLine($cmd)
    $ok = WaitForOk
    if (-not $ok) { break }
}

# Final position
Write-Host ">>> ?" -ForegroundColor Yellow
$port.WriteLine("?")
Start-Sleep -Milliseconds 500
Write-Host $port.ReadExisting().Trim() -ForegroundColor Green

$port.Close()
Write-Host "=== Done ($n lines) ===" -ForegroundColor Green
