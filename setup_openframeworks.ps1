# setup_openframeworks.ps1
#
# Downloads openFrameworks 0.12.0 (VS release) and installs it to a
# user-local directory. Designed to be run once; idempotent guard refuses
# to overwrite an existing install.
#
# Usage (PowerShell, normal user, no admin needed):
#   .\setup_openframeworks.ps1
#
# Optional overrides:
#   .\setup_openframeworks.ps1 -InstallRoot D:\of -Force
#
# What this script does:
#   1. Sanity-checks the install path (refuses if it already exists, unless
#      -Force is passed).
#   2. Downloads of_v0.12.0_vs_release.zip from openframeworks.cc (~500 MB).
#   3. Extracts to a temp dir, then moves the inner folder to InstallRoot.
#   4. Verifies projectGenerator\projectGenerator.exe is present.
#   5. Prints the next-step command to generate the welding-trainer project.
#
# What this script does NOT do:
#   * Modify PATH, system env vars, or any registry keys.
#   * Touch the welding/ repo. The repo stays bit-identical.
#   * Generate the VS solution for the welding project — that is step 3 of
#     SETUP_OF.md and is run manually so the addon picks are reviewable.

[CmdletBinding()]
param(
    [string] $InstallRoot = (Join-Path $env:USERPROFILE 'dev\openFrameworks'),
    [string] $Version     = "0.12.1",
    [switch] $Force
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

# As of 0.12.x the project moved release hosting to GitHub releases. The
# legacy openframeworks.cc/versions/ URL pattern returns 404 for 0.12.0+.
$zipUrl = "https://github.com/openframeworks/openFrameworks/releases/download/$Version/of_v${Version}_vs_64_release.zip"
$tmpZip   = Join-Path $env:TEMP "of_v${Version}_vs.zip"
$tmpDir   = Join-Path $env:TEMP "of_unzip_$(Get-Random)"

# --- Pre-flight ------------------------------------------------------------

Write-Host "=== openFrameworks $Version installer ===" -ForegroundColor Cyan
Write-Host "Target: $InstallRoot"
Write-Host "Source: $zipUrl"
Write-Host ""

if (Test-Path $InstallRoot) {
    if (-not $Force) {
        Write-Host "ERROR: $InstallRoot already exists." -ForegroundColor Red
        Write-Host "Pass -Force to overwrite, or pick a different -InstallRoot."
        exit 1
    }
    Write-Host "Removing existing $InstallRoot (--Force was set)" -ForegroundColor Yellow
    Remove-Item -Recurse -Force $InstallRoot
}

$parent = Split-Path $InstallRoot -Parent
if (-not (Test-Path $parent)) {
    New-Item -ItemType Directory -Force -Path $parent | Out-Null
}

# --- Download --------------------------------------------------------------

Write-Host "[1/4] Downloading openFrameworks $Version (~500 MB)..."
$swStart = Get-Date
# IWR is slow on large files because of its default progress display. Disabling
# the progress preference cuts download time by roughly 5x on Windows 11.
$prev = $ProgressPreference
$ProgressPreference = "SilentlyContinue"
try {
    Invoke-WebRequest -Uri $zipUrl -OutFile $tmpZip -UseBasicParsing
} finally {
    $ProgressPreference = $prev
}
$dlElapsed = (Get-Date) - $swStart
Write-Host ("       done in {0:N1}s ({1:N0} MB)" -f $dlElapsed.TotalSeconds, ((Get-Item $tmpZip).Length / 1MB))

# --- Extract ---------------------------------------------------------------

Write-Host "[2/4] Extracting..."
New-Item -ItemType Directory -Force -Path $tmpDir | Out-Null
Expand-Archive -Path $tmpZip -DestinationPath $tmpDir -Force

# The zip top-level is "of_v0.12.0_vs_release" — pick whichever single dir
# came out and rename to the target.
$inner = Get-ChildItem -Directory $tmpDir | Select-Object -First 1
if ($null -eq $inner) {
    Write-Host "ERROR: archive extracted nothing." -ForegroundColor Red
    exit 1
}
Write-Host "[3/4] Moving $($inner.Name) -> $InstallRoot"
Move-Item -Path $inner.FullName -Destination $InstallRoot

# --- Verify ---------------------------------------------------------------

$pg = Join-Path $InstallRoot "projectGenerator\projectGenerator.exe"
if (-not (Test-Path $pg)) {
    Write-Host "WARNING: $pg not found." -ForegroundColor Yellow
    Write-Host "         The zip layout may have changed. Check $InstallRoot manually."
} else {
    Write-Host "[4/4] projectGenerator.exe present"
}

# --- Cleanup --------------------------------------------------------------

Remove-Item -Force $tmpZip   -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force $tmpDir -ErrorAction SilentlyContinue

# --- Next steps -----------------------------------------------------------

Write-Host ""
Write-Host "=== Install complete ===" -ForegroundColor Green
Write-Host ""
Write-Host "Next, follow SETUP_OF.md to generate the welding-trainer VS project."
Write-Host "Quick-start command (run from this directory):"
Write-Host ""
Write-Host "  & '$pg' -o'$InstallRoot' -a'ofxGui' '$PSScriptRoot\of-app'" -ForegroundColor Cyan
Write-Host ""
