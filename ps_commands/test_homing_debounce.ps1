param(
    [string]$Compiler = "gcc"
)

$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
$utils = Get-Content (Join-Path $root "srcs\utils\utils.c") -Raw
$settings = Get-Content (Join-Path $root "srcs\settings\settings.c") -Raw
$settingsHeader = Get-Content (Join-Path $root "incs\settings\settings.h") -Raw
$homing = Get-Content (Join-Path $root "srcs\motion\homing.c") -Raw
$parser = Get-Content (Join-Path $root "srcs\gcode\gcode_parser.c") -Raw
$harness = Get-Content (Join-Path $root "tests\homing_debounce_test.c") -Raw

function Extract-Source([string]$source, [string]$pattern) {
    $match = [regex]::Match($source, $pattern)
    if (!$match.Success) {
        throw "Production source extraction failed for: $pattern"
    }
    return $match.Value
}

$production = Extract-Source $settingsHeader '(?ms)^typedef struct \{.*?^\} CNC_Settings;'
$production += "`nCNC_Settings* SETTINGS_GetCurrent(void);`n"
$production += "`n" + (Extract-Source $settings '(?ms)^bool SETTINGS_SetValue\(.*?^\}')
$production += "`n" + (Extract-Source $settings '(?ms)^float SETTINGS_GetValue\(.*?^\}')
$production += "`n" + (Extract-Source $utils '(?ms)^// ===== HOMING LIMIT STATE CHANGE TRACKER =====.*\z')
$production += "`n" + (Extract-Source $homing '(?ms)^bool HOMING_IsLimitActiveNow\(.*?^\}')
$production += "`n" + (Extract-Source $homing '(?ms)^bool HOMING_LimitTriggered\(.*?^\}')
$valueParsing = Extract-Source $parser '(?s)char\* value_end;.*?(?=\s+CNC_Settings\* s = SETTINGS_GetCurrent\(\);)'
$production += "`nstatic bool parse_value(long param, char* pend, float* result) {`n"
$production += $valueParsing + "`n*result = val; return valid_value;`n}`n"
$harness = $harness.Replace("/* PRODUCTION_SOURCE */", $production)

$temp = Join-Path ([IO.Path]::GetTempPath()) ("cnc-debounce-" + [guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $temp | Out-Null
try {
    $source = Join-Path $temp "homing_debounce_test.c"
    $binary = Join-Path $temp "homing_debounce_test.exe"
    Set-Content $source $harness -Encoding ASCII
    & $Compiler -std=c99 -O1 -ffast-math -Wall -Wextra -Werror $source -o $binary
    if ($LASTEXITCODE -ne 0) {
        throw "Homing debounce regression compilation failed ($LASTEXITCODE)."
    }
    & $binary
    if ($LASTEXITCODE -ne 0) {
        throw "Homing debounce regressions failed ($LASTEXITCODE)."
    }
} finally {
    Remove-Item -LiteralPath $temp -Recurse -Force
}
