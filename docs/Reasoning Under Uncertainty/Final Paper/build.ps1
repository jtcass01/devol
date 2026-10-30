<#
Build the document with MiKTeX. No Perl, so no latexmk: this runs the classic
four-pass sequence directly (pdflatex, bibtex, pdflatex, pdflatex), which is
all a document with citations and cross-references needs.

    .\build.ps1            build, then report warnings
    .\build.ps1 -Clean     delete aux files first
    .\build.ps1 -Quiet     suppress the warning report

The root .tex is found automatically: the one file in this directory that
contains \documentclass. That way renaming the template for your class does
not mean editing this script.
#>
[CmdletBinding()]
param(
    [switch]$Clean,
    [switch]$Quiet
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$bin = "$env:LOCALAPPDATA\Programs\MiKTeX\miktex\bin\x64"
if (-not (Test-Path (Join-Path $bin "pdflatex.exe"))) {
    throw "pdflatex not found under $bin. Is MiKTeX installed?"
}
$env:PATH = "$bin;$env:PATH"

$roots = @(Get-ChildItem -Filter *.tex | Where-Object {
    Select-String -Path $_.FullName -Pattern '\\documentclass' -Quiet
})
if ($roots.Count -ne 1) {
    throw "expected exactly one .tex containing \documentclass here, found $($roots.Count)"
}
$name = [System.IO.Path]::GetFileNameWithoutExtension($roots[0].Name)
Write-Host "building $($roots[0].Name)" -ForegroundColor DarkGray

$aux = @("*.aux", "*.bbl", "*.blg", "*.log", "*.out", "*.toc", "*.lof", "*.lot")
if ($Clean) {
    Get-ChildItem $aux -ErrorAction SilentlyContinue | Remove-Item -Force
    Write-Host "cleaned aux files" -ForegroundColor DarkGray
}

# TeX tools write routine notices to stderr, and PowerShell 5.1 turns any
# stderr from a native command into a terminating NativeCommandError under
# ErrorActionPreference=Stop - even when the exe exits 0. So relax the
# preference around each pass and judge success on whether the PDF was
# written, not on the exit code. Do NOT add a 2>&1 redirect here; that is what
# manufactures the ErrorRecord in the first place.
function Invoke-Pass([string]$label, [scriptblock]$cmd) {
    Write-Host "==> $label" -ForegroundColor Cyan
    $prev = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try { & $cmd | Out-Null } finally { $ErrorActionPreference = $prev }
}

Invoke-Pass "pdflatex (1/3)" { pdflatex -interaction=nonstopmode "$name.tex" }
Invoke-Pass "bibtex"         { bibtex $name }
Invoke-Pass "pdflatex (2/3)" { pdflatex -interaction=nonstopmode "$name.tex" }
Invoke-Pass "pdflatex (3/3)" { pdflatex -interaction=nonstopmode "$name.tex" }

if (-not (Test-Path "$name.pdf")) { throw "build failed: $name.pdf was not written" }

$pdf = Get-Item "$name.pdf"
Write-Host ("`n{0}  {1} KB  {2}" -f $pdf.Name, [math]::Round($pdf.Length / 1KB), $pdf.LastWriteTime) -ForegroundColor Green

if ($Quiet) { return }

# Undefined references and citations mean the cross-reference passes have not
# settled; overfull boxes mean something is sticking past the margin. Both are
# worth seeing without opening the log.
$patterns = 'LaTeX Warning|Overfull|Underfull|Citation .* undefined|Reference .* undefined'
$warnings = @(Get-Content "$name.log" | Select-String -Pattern $patterns)

if ($warnings.Count -eq 0) {
    Write-Host "no warnings" -ForegroundColor Green
} else {
    Write-Host ("{0} warning(s):" -f $warnings.Count) -ForegroundColor Yellow
    $warnings | ForEach-Object { Write-Host "  $($_.Line.Trim())" -ForegroundColor DarkYellow }
}
