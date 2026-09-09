<#
.SYNOPSIS
  Compile and run the wrist-roll cancellation geometry checks. No game, no headset, ~5 seconds.

.DESCRIPTION
  aimrollfix rotates the aim direction back about the handle axis by the twist the wrist is
  carrying, so that rolling the controller stops sliding the aim sideways. Whether it does that is
  a question about pure geometry, and a headset is the wrong instrument for it: in there, "the
  construction is wrong", "the calibration is stale" and "my hand moved" all look the same.

  tests\AimRollGeometryTest.cpp includes the REAL src\Math.hpp and exercises the shipping
  functions - quat_up, rotate_about_axis, wrist_twist_upright - rather than a copy that would pass
  forever while the plugin rotted. It asserts the invariance directly, and carries a paired control
  arm (the UNcorrected direction must still swing) so a test that silently stopped measuring
  anything would fail rather than pass.

  RUN IT AFTER TOUCHING any of those three functions.

  The test deliberately lives in tests\ and not src\test\: both build scripts compile every .cpp
  under src\ recursively, so a file with a main() in there links into halo_vr.dll and breaks the
  contributor build and CI.

.PARAMETER KeepArtifacts
  Leave the compiled exe and object file in place instead of removing them.

.EXAMPLE
  scripts\Verify-AimRollGeometry.ps1
#>
[CmdletBinding()]
param(
    [switch]$KeepArtifacts
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$testSrc  = Join-Path $repoRoot 'tests\AimRollGeometryTest.cpp'
$mathHdr  = Join-Path $repoRoot 'src\Math.hpp'

if (-not (Test-Path $testSrc)) { throw "Test source not found: $testSrc" }
if (-not (Test-Path $mathHdr)) { throw "src\Math.hpp not found: $mathHdr" }

# Same compiler discovery as scripts\build.ps1, so this cannot pass under a toolset the plugin is
# not built with.
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { throw "vswhere.exe not found - install Visual Studio Build Tools." }
$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { throw "No Visual Studio installation with the C++ toolset was found." }
$vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'
if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found: $vcvars" }

$outDir = Join-Path ([System.IO.Path]::GetTempPath()) ('halo_aimroll_test_' + [System.Guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Path $outDir -Force | Out-Null
$exe = Join-Path $outDir 'AimRollGeometryTest.exe'
$obj = Join-Path $outDir 'AimRollGeometryTest.obj'

Write-Host "[aimroll] compiling tests\AimRollGeometryTest.cpp against src\Math.hpp"

# /W4 /WX: this file is small, self-contained and pure maths, so it has no excuse for a warning -
# and a warning here usually means a real numeric mistake (truncation, signed/unsigned, shadowing).
$clArgs = @(
    '/nologo', '/std:c++17', '/EHsc', '/O2', '/W4', '/WX',
    "`"$testSrc`"",
    # Name the object file outright. Passing a DIRECTORY here needs a trailing backslash, and a
    # backslash immediately before the closing quote is read as an escape by the C runtime's
    # command-line parser - it swallows the quote and the rest of the line becomes one garbage path.
    "/Fo:`"$obj`"",
    "/Fe:`"$exe`""
) -join ' '

$build = cmd /c "`"$vcvars`" >nul 2>&1 && cl $clArgs 2>&1"
$buildFailed = ($LASTEXITCODE -ne 0)
$build | Where-Object { $_ -match 'error|warning' } | ForEach-Object { Write-Host "  $_" }

if ($buildFailed -or -not (Test-Path $exe)) {
    if (-not $KeepArtifacts) { Remove-Item $outDir -Recurse -Force -ErrorAction SilentlyContinue }
    throw "Compile FAILED. If Math.hpp changed shape, fix the test rather than deleting the check."
}

Write-Host "[aimroll] running"
Write-Host ""
& $exe
$rc = $LASTEXITCODE
Write-Host ""

if (-not $KeepArtifacts) {
    Remove-Item $outDir -Recurse -Force -ErrorAction SilentlyContinue
} else {
    Write-Host "[aimroll] artifacts kept in $outDir"
}

if ($rc -ne 0) {
    Write-Host "[aimroll] FAILED - the roll construction does not hold. Do not ship aimrollfix on." -ForegroundColor Red
    exit 1
}

Write-Host "[aimroll] OK - roll cancellation verified against the shipping maths." -ForegroundColor Green
exit 0
