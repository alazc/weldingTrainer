#Requires -Version 5.1
<#
.SYNOPSIS
    Build and run the openFrameworks "welding trainer" app on Windows with
    MSVC 2022 Build Tools.

.DESCRIPTION
    One entry point for the two phases of working with this repo:

      BUILD  Build of-app\of-app.sln via MSBuild (Release/x64 by default),
             producing of-app\bin\of-app.exe. The .sln/.vcxproj are
             hand-authored; this script NEVER calls projectGenerator.

      RUN    Launch of-app\bin\of-app.exe with --mode / --source, with the
             process working directory set to of-app\bin (so openFrameworks
             finds bin\data\). Skipped with -NoRun.

    Toolchain paths are hardcoded to the known BuildTools install, with a
    vswhere fallback so the script is portable across machines. All paths are
    resolved relative to the script location, so it runs from any cwd. The
    script is non-interactive (no pauses / Read-Host).

.PARAMETER Mode
    App mode passed as --mode=<value>. trainer | linkage. Default: trainer.

.PARAMETER Source
    Input source passed as --source=<value>. mouse | serial | serial:COMn |
    replay:<path.csv>. Default: mouse.

.PARAMETER Config
    MSBuild configuration. Release | Debug. Default: Release.

.PARAMETER NoRun
    Build only; do not launch the GUI app.

.PARAMETER Clean
    Remove build outputs (of-app\obj, of-app\bin\*.exe) before building.

.PARAMETER Help
    Show this help and exit.

.EXAMPLE
    .\build-and-run.ps1
    Build the trainer (Release/x64) and launch it with --source=mouse.

.EXAMPLE
    .\build-and-run.ps1 -Mode linkage
    Build and launch in four-bar linkage mode.

.EXAMPLE
    .\build-and-run.ps1 -Source replay:of-app\bin\session.csv
    Build and launch the trainer replaying a previously recorded session
    (sessions are written to of-app\bin\session.csv by the app).

.EXAMPLE
    .\build-and-run.ps1 -NoRun
    Build the GUI app but do not launch it.

.NOTES
    Repo standard build:
        MSBuild of-app\of-app.sln /p:Configuration=Release /p:Platform=x64 /m
#>

[CmdletBinding()]
param(
    [ValidateSet('trainer', 'linkage')]
    [string]$Mode = 'trainer',

    [string]$Source = 'mouse',

    [ValidateSet('Release', 'Debug')]
    [string]$Config = 'Release',

    [switch]$NoRun,
    [switch]$Clean,
    [switch]$Help
)

if ($Help) {
    Get-Help -Detailed $PSCommandPath
    return
}

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# ---------------------------------------------------------------------------
# Paths (all absolute, anchored to the script location).
# ---------------------------------------------------------------------------
$RepoRoot   = $PSScriptRoot
$OfAppDir   = Join-Path $RepoRoot 'of-app'
$Solution   = Join-Path $OfAppDir 'of-app.sln'
$BinDir     = Join-Path $OfAppDir 'bin'
$AppExe     = Join-Path $BinDir   'of-app.exe'
$ObjDir     = Join-Path $OfAppDir 'obj'
$OfRoot     = if ($env:OF_ROOT) { $env:OF_ROOT } else { Join-Path $env:USERPROFILE 'dev\openFrameworks' }

# Hardcoded BuildTools location, with vswhere fallback below.
$MsBuildHard = 'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe'

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
function Write-Banner {
    param([string]$Text, [ConsoleColor]$Color = 'Cyan')
    $bar = '=' * 70
    Write-Host ''
    Write-Host $bar -ForegroundColor $Color
    Write-Host ("  {0}" -f $Text) -ForegroundColor $Color
    Write-Host $bar -ForegroundColor $Color
}

function Write-Info  { param([string]$m) Write-Host "  $m" -ForegroundColor Gray }
function Write-Ok    { param([string]$m) Write-Host "  $m" -ForegroundColor Green }
function Die {
    param([string]$m)
    Write-Host ''
    Write-Host "ERROR: $m" -ForegroundColor Red
    exit 1
}

# Resolve MSBuild.exe: hardcoded path, else vswhere.
function Resolve-MsBuild {
    if (Test-Path -LiteralPath $MsBuildHard) { return $MsBuildHard }

    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (Test-Path -LiteralPath $vswhere) {
        # -products * so Build Tools (not just IDE editions) are considered.
        $found = & $vswhere -products * -latest -requires Microsoft.Component.MSBuild `
                    -find 'MSBuild\**\Bin\MSBuild.exe' 2>$null | Select-Object -First 1
        if ($found -and (Test-Path -LiteralPath $found)) { return $found }
    }
    Die "Could not locate MSBuild.exe. Checked:`n    $MsBuildHard`n    and vswhere. Install MSVC 2022 Build Tools (or full VS)."
}

# ---------------------------------------------------------------------------
# Pre-flight: openFrameworks root must exist for the GUI build.
# ---------------------------------------------------------------------------
if (-not (Test-Path -LiteralPath $OfRoot)) {
    Die "openFrameworks root not found: $OfRoot`n    The of-app vcxproj imports its props. Install oF there (see SETUP_OF.md)."
}

# ---------------------------------------------------------------------------
# CLEAN
# ---------------------------------------------------------------------------
if ($Clean) {
    Write-Banner 'CLEAN' Magenta
    if (Test-Path -LiteralPath $ObjDir) {
        Write-Info "Removing $ObjDir"
        Remove-Item -LiteralPath $ObjDir -Recurse -Force -ErrorAction SilentlyContinue
    }
    if (Test-Path -LiteralPath $AppExe) {
        Write-Info "Removing $AppExe"
        Remove-Item -LiteralPath $AppExe -Force -ErrorAction SilentlyContinue
    }
    Write-Ok 'Clean complete.'
}

# ---------------------------------------------------------------------------
# BUILD  GUI app via MSBuild
# ---------------------------------------------------------------------------
Write-Banner ("BUILD — of-app ({0}/x64)" -f $Config) Cyan

if (-not (Test-Path -LiteralPath $Solution)) {
    Die "Solution not found: $Solution"
}
$msbuild = Resolve-MsBuild
Write-Info "MSBuild: $msbuild"

$msbuildArgs = @(
    $Solution,
    "/p:Configuration=$Config",
    '/p:Platform=x64',
    "/p:OF_ROOT=$OfRoot",
    '/m'
)
Write-Host ("  > `"{0}`" {1}" -f $msbuild, ($msbuildArgs -join ' ')) -ForegroundColor DarkGray

& $msbuild @msbuildArgs
if ($LASTEXITCODE -ne 0) {
    Die "MSBuild failed (exit $LASTEXITCODE)."
}
if (-not (Test-Path -LiteralPath $AppExe)) {
    Die "Build reported success but $AppExe was not produced."
}
Write-Ok "Build OK — $AppExe"

# ---------------------------------------------------------------------------
# RUN  (skip if -NoRun)
# ---------------------------------------------------------------------------
if (-not $NoRun) {
    Write-Banner ("RUN — {0} mode, source={1}" -f $Mode, $Source) Green

    if (-not (Test-Path -LiteralPath $AppExe)) {
        Die "App not found: $AppExe (build it first; do not pass -NoRun)."
    }

    $appArgs = @("--mode=$Mode", "--source=$Source")
    Write-Info "Launching with working dir: $BinDir"
    Write-Host ("  > `"{0}`" {1}" -f $AppExe, ($appArgs -join ' ')) -ForegroundColor DarkGray

    # WorkingDirectory must be bin\ so openFrameworks loads bin\data\ assets.
    Start-Process -FilePath $AppExe -ArgumentList $appArgs -WorkingDirectory $BinDir
    Write-Ok 'App launched.'
} else {
    Write-Info '-NoRun set: not launching the GUI.'
}

exit 0
