# ==============================================================================
# Reorganization PowerShell Script for DEMDProject
# ==============================================================================

$baseDir = Split-Path -Parent $PSScriptRoot
if (-not $baseDir) { $baseDir = Get-Location }

# Create directories
$dirs = @("src", "models", "docs", "scripts")
foreach ($d in $dirs) {
    $target = Join-Path $baseDir $d
    if (-not (Test-Path $target)) {
        New-Item -ItemType Directory -Path $target | Out-Null
    }
}

# 1. Relocate academic PDF report
$pdfSource = Join-Path $baseDir "Electrical_machines_report (3).pdf"
$pdfDest = Join-Path $baseDir "docs\Electrical_machines_report.pdf"
if (Test-Path $pdfSource) {
    Move-Item -Path $pdfSource -Destination $pdfDest -Force
    Write-Host "PDF report moved to docs/Electrical_machines_report.pdf" -ForegroundColor Green
}

# 2. Extract and relocate Simulink model
$zipSource = Join-Path $baseDir "Simulink_final.slx.zip"
$modelsDir = Join-Path $baseDir "models"
if (Test-Path $zipSource) {
    Expand-Archive -Path $zipSource -DestinationPath $modelsDir -Force
    Move-Item -Path $zipSource -Destination (Join-Path $modelsDir "Simulink_final.slx.zip") -Force
    Write-Host "Simulink model extracted into models/" -ForegroundColor Green
}

# 3. Clean up obsolete files
$txtSource = Join-Path $baseDir "Report_final.m.txt"
if (Test-Path $txtSource) {
    Remove-Item $txtSource -Force
    Write-Host "File Report_final.m.txt removed (superseded by src/Report_final.m)" -ForegroundColor Yellow
}

$testTxt = Join-Path $baseDir "src\test.txt"
if (Test-Path $testTxt) {
    Remove-Item $testTxt -Force
}

Write-Host "Repository reorganization completed successfully!" -ForegroundColor Cyan
