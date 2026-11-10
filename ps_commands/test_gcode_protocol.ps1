param(
    [string]$Compiler = "gcc"
)

$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
$parser = Get-Content (Join-Path $root "srcs\gcode\gcode_parser.c") -Raw
$header = Get-Content (Join-Path $root "incs\gcode\gcode_parser.h") -Raw
$harness = Get-Content (Join-Path $root "tests\gcode_protocol_test.c") -Raw

function Extract-Source([string]$source, [string]$pattern) {
    $match = [regex]::Match($source, $pattern)
    if (!$match.Success) {
        throw "Production source extraction failed for: $pattern"
    }
    return $match.Value
}

$types = Extract-Source $header '(?ms)^typedef enum \{\r?\n    GCODE_STATE_IDLE,.*?^\} GCODE_Data;'
$harness = $harness.Replace("/* PRODUCTION_TYPES */", $types)
$production = Extract-Source $parser '(?ms)^static uint8_t txBuffer.*?(?=^/\*[^\r\n]*\r?\n/\* Forward Declarations)'
$production += "`n" + (Extract-Source $parser '(?ms)^static inline bool is_control_char\(.*?(?=^GCODE_CommandQueue\* Extract_)')
$production += "`n" + (Extract-Source $parser '(?ms)^void GCODE_SoftReset\(.*?^\}')
$production += "`n" + (Extract-Source $parser '(?ms)^void GCODE_USART_Initialize\(.*?^\}')
$production += "`n" + (Extract-Source $parser '(?ms)^void GCODE_CheckDeferredOk\(.*?^\}')
$production += "`n" + (Extract-Source $parser '(?ms)^static void SendOrDeferOk\(.*?^\}')
$production += "`n" + (Extract-Source $parser '(?ms)^static void service_realtime_byte\(.*?^\}')
$production += "`n" + (Extract-Source $parser '(?ms)^void GCODE_Tasks\(.*?^\}')
$harness = $harness.Replace("/* PRODUCTION_SOURCE */", $production)

$temp = Join-Path ([IO.Path]::GetTempPath()) ("cnc-protocol-" + [guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $temp | Out-Null
try {
    $source = Join-Path $temp "gcode_protocol_test.c"
    $binary = Join-Path $temp "gcode_protocol_test.exe"
    Set-Content $source $harness -Encoding ASCII
    & $Compiler -std=c99 -O1 -Wall -Wextra -Werror $source -o $binary
    if ($LASTEXITCODE -ne 0) {
        throw "G-code protocol regression compilation failed ($LASTEXITCODE)."
    }
    & $binary
    if ($LASTEXITCODE -ne 0) {
        throw "G-code protocol regressions failed ($LASTEXITCODE)."
    }
} finally {
    Remove-Item -LiteralPath $temp -Recurse -Force
}
