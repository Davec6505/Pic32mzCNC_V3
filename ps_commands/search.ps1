param(
    [Parameter(Mandatory=$true)]
    [string]$wordToFind,

    [string]$filePath = "../",                # Default: current directory
    [string]$outputPath = "",               # Default: no output file
    [string]$searchType = "simple",         # Default: simple search
    [switch]$caseSensitive = $false,        # Default: case-insensitive
    [ValidateSet("UTF8", "Unicode", "Ascii", "Default", "BigEndianUnicode")]
    [string]$encoding = "UTF8",             # Default: UTF8
    [string]$suffixes = "*.c,*.h"           # Default: C and header files
)

# Convert suffixes to array
$suffixArray = $suffixes -split ','

# Prepare results container
$results = @()

# Loop through each suffix
foreach ($suffix in $suffixArray) {
    $files = Get-ChildItem -Path $filePath -Recurse -Filter $suffix.Trim()

    foreach ($file in $files) {
        $content = Get-Content -Path $file.FullName -Encoding $encoding

        for ($i = 0; $i -lt $content.Count; $i++) {
            $line = $content[$i]
            $lineNumber = $i + 1
                    
            $match = $false

            if ($searchType -eq "regex") {
                $options = if ($caseSensitive) { 'None' } else { 'IgnoreCase' }
                if ($line -match [regex]::new($wordToFind, $options)) {
                    $match = $true
                }
            } else {
                $comparison = if ($caseSensitive) { 'Ordinal' } else { 'OrdinalIgnoreCase' }
                if ($line.IndexOf($wordToFind, 0, [System.StringComparison]::$comparison) -ge 0) {
                    $match = $true
                }
            }

            if ($match) {
              $results += "$($file.Name):${lineNumber}: $line"
            }
        }
    }
}


# Output results
if ($results.Count -gt 0) {
    if ($outputPath) {
        $results | Set-Content -Path $outputPath -Encoding $encoding
        Write-Host "Matches written to $outputPath"
    } else {
        $results | ForEach-Object {
            if ($_ -match "^(.*?):(\d+):\s?(.*)$") {
                $file = $matches[1].PadRight(5)        # Filename column (adjust width as needed)
                $lineNumber = $matches[2].PadRight(5)   # Line number column, right-aligned
                 # Line number column, right-aligned
                $line = $matches[3].Trim() # Code line, left-aligned
                Write-Host "$file" -ForegroundColor Cyan -NoNewline
                Write-Host " $lineNumber " -ForegroundColor Green -NoNewline
                Write-Host "$line" -ForegroundColor Yellow
            } else {
                Write-Host $_  # fallback if format is unexpected
            }
        }
    }
} else {
    Write-Host "No matches found."
}
