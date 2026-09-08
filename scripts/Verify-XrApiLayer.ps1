<#
.SYNOPSIS
  Build the OpenXR API layer and run it against a fake loader. No game, no headset, ~5 seconds.

.DESCRIPTION
  The layer sits between two parties this repo does not own: the OpenXR loader above it and the
  runtime below it. Testing it in a headset needs the game, an injection and a human, and it is the
  wrong instrument anyway -- when the quad does not appear, "the negotiation struct is wrong" and
  "the pose maths is wrong" look identical from inside a headset.

  So apilayer\test\LayerSelfTest.cpp IS a loader: it negotiates, walks the layer through
  xrCreateApiLayerInstance with a stub chain underneath, submits frames, and asserts on WHAT THE
  STUB RUNTIME RECEIVED rather than on what the layer says it did.

  RUN THIS AFTER ANY CHANGE UNDER apilayer\. It is the only automated reader that code has, and the
  code runs inside every OpenXR application on a user's machine once the layer is registered.

  TWO RUNS, NOT ONE. The process-name gate is decided once per process and cached, so both sides of
  it cannot be exercised in a single run:
    * ACTIVE (HALOVR_LAYER_FORCE=1) -- the layer must intercept and function.
    * GATED  (no force variable, host is not the game) -- the layer must be INVISIBLE: xrEndFrame
      unwrapped, the bridge API refused. That half matters more than the active half, because it is
      the half that runs in other people's games.

.PARAMETER KeepArtifacts
  Leave the built DLL and test exe behind for inspection instead of using a temp directory.
#>
[CmdletBinding()]
param(
    [switch]$KeepArtifacts
)

$ErrorActionPreference = 'Stop'

$repo    = Split-Path -Parent $PSScriptRoot
$testSrc = Join-Path $repo 'apilayer\test\LayerSelfTest.cpp'
$srcDir  = Join-Path $repo 'src'

if (-not (Test-Path $testSrc)) { throw "Self-test source not found: $testSrc" }

$work = if ($KeepArtifacts) { Join-Path $repo 'build\apilayer-selftest' }
        else { Join-Path ([System.IO.Path]::GetTempPath()) ("halovr-layertest-" + [guid]::NewGuid().ToString('N').Substring(0,8)) }
if (-not (Test-Path $work)) { New-Item -ItemType Directory -Path $work -Force | Out-Null }

try {
    # 1. The layer itself, into the same folder as the test exe so LoadLibraryW finds it by name.
    & (Join-Path $PSScriptRoot 'build-apilayer.ps1') -OutDir $work
    if ($LASTEXITCODE -ne 0) { throw 'The layer failed to build; the self-test cannot run.' }

    # 2. The harness.
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    $vsPath  = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $vsPath) { throw 'No Visual Studio C++ toolchain found.' }
    $vcvars  = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'
    $exe     = Join-Path $work 'LayerSelfTest.exe'

    # /W4 /WX on the harness too: a test that warns is a test nobody trusts.
    $clArgs = @(
        '/nologo', '/MT', '/O2', '/EHsc', '/std:c++20', '/W4', '/WX',
        "/I`"$srcDir`"", "/I`"$srcDir\thirdparty`"",
        "`"$testSrc`"",
        "/Fe:`"$exe`"", "/Fo:`"$work/`"",
        '/link', 'user32.lib'
    ) -join ' '

    Write-Host 'Compiling the self-test harness...' -ForegroundColor Cyan
    $out = cmd /c "`"$vcvars`" >nul 2>&1 && cl $clArgs 2>&1"
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $exe)) {
        $out | Where-Object { $_ -match 'error|warning' } | ForEach-Object { Write-Host "  $_" }
        throw 'The self-test harness failed to build.'
    }

    $fail = 0

    # 3a. ACTIVE. The force variable is scoped to this process tree, so it cannot leak anywhere.
    Write-Host ''
    $env:HALOVR_LAYER_FORCE = '1'
    Remove-Item Env:\HALOVR_LAYER_DISABLE -ErrorAction SilentlyContinue
    & $exe 'active'
    if ($LASTEXITCODE -ne 0) { $fail++ ; Write-Host 'ACTIVE MODE FAILED' -ForegroundColor Red }

    # 3b. GATED. No force variable, and the host executable is LayerSelfTest.exe -- exactly the
    # situation of every OpenXR application on the machine that is not this game.
    Write-Host ''
    Remove-Item Env:\HALOVR_LAYER_FORCE -ErrorAction SilentlyContinue
    & $exe 'gated'
    if ($LASTEXITCODE -ne 0) { $fail++ ; Write-Host 'GATED MODE FAILED' -ForegroundColor Red }

    # 3c. The manifest's kill switch must beat the force variable, or a player who sets
    # HALOVR_LAYER_DISABLE has no way to turn the layer off on the developer activation route.
    Write-Host ''
    $env:HALOVR_LAYER_FORCE   = '1'
    $env:HALOVR_LAYER_DISABLE = '1'
    & $exe 'gated'
    if ($LASTEXITCODE -ne 0) { $fail++ ; Write-Host 'DISABLE-OVERRIDE MODE FAILED' -ForegroundColor Red }
    Remove-Item Env:\HALOVR_LAYER_FORCE   -ErrorAction SilentlyContinue
    Remove-Item Env:\HALOVR_LAYER_DISABLE -ErrorAction SilentlyContinue

    Write-Host ''
    if ($fail -gt 0) {
        Write-Host "XR API LAYER SELF-TEST FAILED ($fail of 3 modes)" -ForegroundColor Red
        exit 1
    }
    Write-Host 'XR API LAYER SELF-TEST PASSED (active + gated + disable-override)' -ForegroundColor Green
}
finally {
    if (-not $KeepArtifacts -and (Test-Path $work)) {
        Remove-Item -Recurse -Force $work -ErrorAction SilentlyContinue
    }
}
