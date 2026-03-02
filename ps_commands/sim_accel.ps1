param(
    [double]$spm  = 100.0,    # steps/mm  (change to match $101 setting)
    [double]$a    = 500.0,    # mm/s² acceleration
    [double]$feed = 10.0,     # mm/s  (600mm/min = 10)
    [double]$MINHZ = 200.0    # minimum step frequency floor
)

$F = 781250.0  # TMR4 post-prescaler Hz

$c0raw = $F * [Math]::Sqrt(2.0 / ($a * $spm))
$c0    = [Math]::Min($c0raw, $F / $MINHZ)
$nom   = [int]($F / ($feed * $spm))
$ir    = [int]$c0
if ($ir -lt $nom) { $ir = $nom }

$veff  = $F / ($ir * $spm)
$n     = [int](2.0 * $veff * $veff * $spm / $a) - 1
if ($n -lt 0) { $n = 0 }

Write-Host ""
Write-Host "=== Taylor AccDec Simulation ==="
Write-Host ("  spm={0}  a={1}mm/s2  feed={2}mm/s ({3}mm/min)  MIN_HZ={4}" -f $spm,$a,$feed,($feed*60),$MINHZ)
Write-Host ("  c0_raw={0,6:F0} ticks  ({1,6:F1} Hz)" -f $c0raw,($F/$c0raw))
Write-Host ("  c0_clamped={0,3:F0} ticks  ({1,6:F1} Hz)  <== initial step rate" -f $c0,($F/$c0))
Write-Host ("  nominal   ={0,3} ticks  ({1,6:F1} Hz)  <== cruise step rate" -f $nom,($F/$nom))
Write-Host ("  n_entry   ={0}" -f $n)
Write-Host ""
Write-Host ("{0,4}  {1,8}  {2,8}  {3,8}  {4,8}  {5}" -f "Step","ac_count","interval","Hz","mm/s","note")
Write-Host ("-" * 60)

$iv   = $ir
$rest = 0
$ac   = $n

for ($s = 1; $s -le 100; $s++) {
    $hz   = $F / $iv
    $sp   = $hz / $spm
    $note = if ($iv -le $nom) { "<-- CRUISE" } else { "" }
    Write-Host ("{0,4}  {1,8}  {2,8}  {3,8:F1}  {4,8:F3}  {5}" -f $s,$ac,$iv,$hz,$sp,$note)
    if ($iv -le $nom) { break }

    $ac++
    $den   = 4 * $ac + 1
    $num   = 2 * $iv + $rest
    $d     = [int]($num / $den)
    $rest  = $num % $den
    $iv    = $iv - $d
    if ($iv -lt $nom) { $iv = $nom }
}
Write-Host ""
