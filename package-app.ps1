#Requires -Version 5.1
<#
.SYNOPSIS
    Package the openFrameworks "welding trainer" into a self-contained zip that
    another Windows user can unzip and run — no build, no openFrameworks, no
    Visual Studio required.

.DESCRIPTION
    of-app.exe links the whole openFrameworks stack statically, so the only
    things a runnable bundle needs are the exe itself, its bin\data\ assets
    (loaded relative to the working directory at runtime), and the Microsoft
    Visual C++ 2015-2022 x64 redistributable on the target machine.

    This script stages those, drops in a README.txt with run instructions, and
    zips it to dist\. It rebuilds the exe first by default (so the bundle is
    never stale and always reflects the current --source=serial default a bare
    double-click resolves to — see main.cpp parseSourceSpec). Pass
    -NoBuild to skip the rebuild and package the existing of-app.exe as-is.

.PARAMETER NoBuild
    Skip the rebuild and package the existing of-app\bin\of-app.exe as-is. By
    default the script runs build-and-run.ps1 -NoRun first to produce a fresh
    Release/x64 exe.

.PARAMETER OutDir
    Directory to write the zip into. Default: dist\ under the repo root.

.EXAMPLE
    .\package-app.ps1
    Rebuild Release/x64, then zip of-app.exe + data\ into dist\. The bundled app
    defaults to serial (hardware) mode when double-clicked.

.EXAMPLE
    .\package-app.ps1 -NoBuild
    Package the existing of-app.exe without rebuilding.
#>

[CmdletBinding()]
param(
    [switch]$NoBuild,
    [string]$OutDir
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$RepoRoot = $PSScriptRoot
$BinDir   = Join-Path $RepoRoot 'of-app\bin'
$AppExe   = Join-Path $BinDir   'of-app.exe'
$DataDir  = Join-Path $BinDir   'data'
if (-not $OutDir) { $OutDir = Join-Path $RepoRoot 'dist' }

function Info { param([string]$m) Write-Host "  $m" -ForegroundColor Gray }
function Ok   { param([string]$m) Write-Host "  $m" -ForegroundColor Green }
function Die  { param([string]$m) Write-Host "`nERROR: $m" -ForegroundColor Red; exit 1 }

# --- Rebuild (default; skip with -NoBuild) ---------------------------------
if (-not $NoBuild) {
    Info 'Rebuilding of-app (Release/x64) ...'
    & (Join-Path $RepoRoot 'build-and-run.ps1') -NoRun
    if ($LASTEXITCODE -ne 0) { Die "build-and-run.ps1 failed (exit $LASTEXITCODE)." }
}

# --- Sanity checks ---------------------------------------------------------
if (-not (Test-Path -LiteralPath $AppExe))  { Die "Not found: $AppExe`n    Build it first:  .\build-and-run.ps1 -NoRun   (or pass -Build)." }
if (-not (Test-Path -LiteralPath $DataDir)) { Die "Not found: $DataDir`n    The bin\data\ asset folder is required at runtime." }

$exeAge = (Get-Date) - (Get-Item -LiteralPath $AppExe).LastWriteTime
if ($NoBuild -and $exeAge.TotalHours -gt 24) {
    Write-Host ("  WARNING: of-app.exe is {0:N1} h old and -NoBuild is set. Drop -NoBuild to rebuild if source changed." -f $exeAge.TotalHours) -ForegroundColor Yellow
}

# --- Stage into a temp folder ---------------------------------------------
$stamp     = Get-Date -Format 'yyyy-MM-dd'
$pkgName   = "welding-trainer_$stamp"
$stageRoot = Join-Path ([System.IO.Path]::GetTempPath()) $pkgName
if (Test-Path -LiteralPath $stageRoot) { Remove-Item -LiteralPath $stageRoot -Recurse -Force }
New-Item -ItemType Directory -Path $stageRoot -Force | Out-Null

Info "Staging exe + data ..."
Copy-Item -LiteralPath $AppExe -Destination $stageRoot
Copy-Item -LiteralPath $DataDir -Destination $stageRoot -Recurse

# --- README ----------------------------------------------------------------
$readme = @'
Welding Trainer — Haptic Pantograph (standalone build)
======================================================

A haptics demo: trace weld paths and watch the bead, arc, and sparks render in
real time. This is a self-contained Windows build — no install needed beyond the
one runtime library noted below.

WHAT'S IN HERE
--------------
  of-app.exe   The application.
  data\        Fonts, weld-path definitions, and sounds it loads at startup.
               Keep of-app.exe and data\ together in the same folder.

REQUIREMENTS
------------
  - Windows 10 or 11 (64-bit), standard edition. ("N" editions need the
    Media Feature Pack.)
  - Microsoft Visual C++ 2015-2022 Redistributable (x64). Most machines have
    it; if of-app.exe fails to start with a "VCRUNTIME140.dll missing" error,
    install it:
        winget install Microsoft.VCRedist.2015+.x64
    or download "Visual C++ Redistributable for Visual Studio 2015-2022 (x64)"
    from microsoft.com.

HOW TO RUN
----------
  Double-click of-app.exe.

  This build defaults to SERIAL (hardware) mode: it opens the first connected
  Arduino/serial device and drives the physical rig. If no device is attached
  the HUD reads "SERIAL: no device" and there is no input.

  No hardware? Run the mouse demo instead, from a terminal opened in this folder:
      .\of-app.exe --source=mouse                    (trace with the mouse)
      .\of-app.exe --mode=linkage --source=mouse     (four-bar linkage view)

  Multiple serial adapters plugged in? Pin the port explicitly:
      .\of-app.exe --source=serial:COM6

CONTROLS
--------
  Drive the torch with the rig handle (serial) or the mouse (--source=mouse).
  Spacebar is the emergency stop. (The on-screen HUD shows the current state.)
'@

Set-Content -LiteralPath (Join-Path $stageRoot 'README.txt') -Value $readme -Encoding ASCII

# --- Zip -------------------------------------------------------------------
if (-not (Test-Path -LiteralPath $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }
$zipPath = Join-Path $OutDir "$pkgName.zip"
if (Test-Path -LiteralPath $zipPath) { Remove-Item -LiteralPath $zipPath -Force }

Info "Compressing ..."
Compress-Archive -Path (Join-Path $stageRoot '*') -DestinationPath $zipPath -CompressionLevel Optimal

Remove-Item -LiteralPath $stageRoot -Recurse -Force

$sizeMB = [math]::Round((Get-Item -LiteralPath $zipPath).Length / 1MB, 1)
Ok "Packaged -> $zipPath  ($sizeMB MB)"
Info "Send that zip; the recipient unzips and double-clicks of-app.exe."
