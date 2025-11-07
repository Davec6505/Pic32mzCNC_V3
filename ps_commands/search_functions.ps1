param(
    [string]$filePath = ".",
    [string]$suffixes = "*.c,*.h",
    [string]$functionName = ""
)

$suffixArray = $suffixes -split ','
$pattern = "^[a-zA-Z_][a-zA-Z0-9_ \*]*\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*\("

$results = @()

foreach ($suffix in $suffixArray) {
    Get-ChildItem -Path $filePath -Recurse -Include $suffix | ForEach-Object {
        $file = $_.FullName
        $lines = Get-Content $file
        for ($i = 0; $i -lt $lines.Count; $i++) {
            if ($lines[$i] -match $pattern) {
                $func = $matches[1]
                if ($functionName -eq "" -or $func -match $functionName) {
                    $results += [PSCustomObject]@{
                        File = $file
                        LineNumber = $i + 1
                        Function = $func
                    }
                }
            }
        }
    }
}

# Output results
$results | Format-Table File, LineNumber, Function

# Open in VS Code if only one result and user wants to navigate
if ($results.Count -gt 0) {
    $file = $results[0].File
    $line = $results[0].LineNumber
    Write-Host "Opening $file at line $line in VS Code..."
    code -g "${file}:${line}"
}