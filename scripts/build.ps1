<#
.SYNOPSIS
  Build halo_vr.dll from every .cpp under src/ against the UEVR plugin SDK.

.DESCRIPTION
  One translation unit, MSVC x64, no other dependencies. The UEVR SDK headers are consumed from a
  praydog/UEVR checkout (they are not vendored here; UEVR's license is all-rights-reserved).

.PARAMETER SdkPath
  Path to a praydog/UEVR checkout (the folder containing include\uevr). Defaults to $env:UEVR_SDK.

.PARAMETER Deploy
  Also copy the built DLL into %APPDATA%\UnrealVRMod\HaloCampaignEvolved\plugins\.
#>
[CmdletBinding()]
param(
    [string]$SdkPath = $env:UEVR_SDK,
    [switch]$Deploy
)

$ErrorActionPreference = 'Stop'

$repo   = Split-Path -Parent $PSScriptRoot
$srcDir = Join-Path $repo 'src'
$outDir = Join-Path $repo 'build'
$dll    = Join-Path $outDir 'halo_vr.dll'

# Every .cpp under src\ is compiled - add a file, it builds, no script edit needed.
# (UEVR's own plugin targets glob the same way; see cmake.toml in a praydog/UEVR checkout.)
$srcFiles = @(Get-ChildItem $srcDir -Filter *.cpp -Recurse -File -ErrorAction SilentlyContinue | Sort-Object FullName)

if (-not $SdkPath) { throw 'Set -SdkPath or $env:UEVR_SDK to a praydog/UEVR checkout (see COMPILING.md).' }
$incDir = Join-Path $SdkPath 'include'
if (-not (Test-Path (Join-Path $incDir 'uevr\API.hpp'))) { throw "UEVR SDK headers not found under: $incDir" }
if ($srcFiles.Count -eq 0) { throw "No .cpp files found under: $srcDir" }
if (-not (Test-Path $outDir)) { New-Item -ItemType Directory -Path $outDir | Out-Null }

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vsPath  = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { throw 'No Visual Studio C++ toolchain found (install VS Build Tools with the C++ workload).' }
$vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'

# x64 toolchain: the DLL is loaded into an x64 game. /MT so no CRT redistributable is needed.
# Flatten to a single string first: a nested array here would stringify as "System.Object[]".
$srcArgs = ($srcFiles | ForEach-Object { "`"$($_.FullName)`"" }) -join ' '

$clArgs = @(
    '/nologo', '/LD', '/MT', '/O2', '/EHsc', '/std:c++20',
    "/I`"$incDir`"",
    "/I`"$srcDir`"",
    $srcArgs,
    "/Fe:`"$dll`"",
    "/Fo:`"$outDir/`"",
    '/link', 'user32.lib'
) -join ' '

Write-Host ("Compiling halo_vr ({0} source file{1})..." -f $srcFiles.Count, $(if($srcFiles.Count -eq 1){''}else{'s'})) -ForegroundColor Cyan
$srcFiles | ForEach-Object { Write-Host "  $($_.FullName.Replace("$repo\",''))" -ForegroundColor DarkGray }
$out = cmd /c "`"$vcvars`" >nul 2>&1 && cl $clArgs 2>&1"
$rc  = $LASTEXITCODE
$out | Where-Object { $_ -match 'error|warning C|Plugin\.cpp' } | ForEach-Object { Write-Host "  $_" }

if ($rc -ne 0 -or -not (Test-Path $dll)) {
    Write-Host "BUILD FAILED (cl exit $rc)" -ForegroundColor Red
    exit 1
}
Write-Host ("BUILD OK  ({0:N0} bytes)  -> {1}" -f (Get-Item $dll).Length, $dll) -ForegroundColor Green

if ($Deploy) {
    $live = Join-Path $env:APPDATA 'UnrealVRMod\HaloCampaignEvolved\plugins\halo_vr.dll'
    $liveDir = Split-Path -Parent $live
    if (-not (Test-Path $liveDir)) { New-Item -ItemType Directory -Path $liveDir -Force | Out-Null }
    try {
        Copy-Item -Path $dll -Destination $live -Force
        Write-Host "DEPLOYED -> $live" -ForegroundColor Green
    } catch {
        Write-Host 'DEPLOY FAILED - the DLL is locked. Close the game first.' -ForegroundColor Red
        exit 2
    }
}
