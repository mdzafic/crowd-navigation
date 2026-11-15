param([string]$Label = "EXP-001", [int]$Repeats = 15)
$exe = "$PSScriptRoot\..\build\crowd_bench.exe"
$csv = "$PSScriptRoot\..\results\bench.csv"
New-Item -ItemType Directory -Force "$PSScriptRoot\..\results" | Out-Null
& $exe $Repeats --csv $csv --label $Label