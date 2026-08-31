<#
.SYNOPSIS
  Build XrApiLayer_HALOVR_reticule.dll -- the OpenXR API layer that carries the compositor reticule
  to players.

.DESCRIPTION
  A SECOND, SEPARATE DLL. It is not part of halo_vr.dll and must never be built into it: an API
  layer is loaded by the OpenXR loader, has its own DllMain and its own required export
  (xrNegotiateLoaderApiLayerInterface), and lives in a process where halo_vr.dll may not even be
  present. scripts\build.ps1 globs src\**\*.cpp, which is exactly why the layer's source lives in
  apilayer\src\ and not under src\ -- see the note in build.ps1.

  No UEVR SDK is needed here. The layer depends on nothing but the Windows SDK and the vendored
  OpenXR headers in src\thirdparty\openxr, which is what makes it buildable by anyone who forks
  this repo without also fetching a UEVR checkout.

.PARAMETER OutDir
  Where to put the DLL. Defaults to build\ next to the plugin's own output; package.ps1 stages it
  from there into apilayer\ inside the release zip.

.PARAMETER Deploy
  Also copy the DLL and its manifest into %APPDATA%\UnrealVRMod\HaloCampaignEvolved\apilayer\,
  which is where a shipped install has them.

  NOTE this does NOT register the layer. Registration is a registry write that affects every OpenXR
  application for the user, so it is a separate, explicit, reversible step --
  profile\apilayer\Register-XrApiLayer.ps1. Deploying the files is safe on its own; nothing loads
  them until they are either registered or activated per-process with XR_API_LAYER_PATH.
#>
[CmdletBinding()]
param(
    [string]$OutDir,
    [switch]$Deploy
)

$ErrorActionPreference = 'Stop'

$repo      = Split-Path -Parent $PSScriptRoot
$layerSrc  = Join-Path $repo 'apilayer\src'
$pluginSrc = Join-Path $repo 'src'
if (-not $OutDir) { $OutDir = Join-Path $repo 'build' }

$dllName  = 'XrApiLayer_HALOVR_reticule.dll'
$dll      = Join-Path $OutDir $dllName
$manifest = Join-Path $repo 'profile\apilayer\XrApiLayer_HALOVR_reticule.json'

$srcFiles = @(Get-ChildItem $layerSrc -Filter *.cpp -File -ErrorAction SilentlyContinue | Sort-Object FullName)
if ($srcFiles.Count -eq 0) { throw "No .cpp files found under: $layerSrc" }
if (-not (Test-Path (Join-Path $pluginSrc 'XrLayerAbi.h'))) {
    throw "src\XrLayerAbi.h not found. That header is the CONTRACT the layer and the plugin share; there is exactly one copy and both sides compile against it."
}
if (-not (Test-Path (Join-Path $pluginSrc 'thirdparty\openxr\loader_interfaces.h'))) {
    throw "src\thirdparty\openxr\loader_interfaces.h not found. It is the loader<->layer negotiation ABI; see the README in that folder."
}
if (-not (Test-Path $manifest)) { throw "Layer manifest not found: $manifest" }
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Path $OutDir -Force | Out-Null }

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vsPath  = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { throw 'No Visual Studio C++ toolchain found (install VS Build Tools with the C++ workload).' }
$vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'

$srcArgs = ($srcFiles | ForEach-Object { "`"$($_.FullName)`"" }) -join ' '

# /MT for the same reason the plugin uses it: this DLL is loaded into arbitrary processes by the
# OpenXR loader and must not need a CRT redistributable that the host may not have.
# /W4 /WX because this code runs inside other people's applications. A warning here is not a style
# question -- it is the only automated reader this file has.
$objDir = Join-Path $OutDir 'apilayer-obj'
if (-not (Test-Path $objDir)) { New-Item -ItemType Directory -Path $objDir -Force | Out-Null }

$clArgs = @(
    '/nologo', '/LD', '/MT', '/O2', '/EHsc', '/std:c++20', '/W4', '/WX',
    "/I`"$pluginSrc`"",
    "/I`"$pluginSrc\thirdparty`"",
    $srcArgs,
    "/Fe:`"$dll`"",
    "/Fo:`"$objDir/`"",
    '/link', 'user32.lib'
) -join ' '

Write-Host ("Compiling {0} ({1} source file{2})..." -f $dllName, $srcFiles.Count, $(if($srcFiles.Count -eq 1){''}else{'s'})) -ForegroundColor Cyan
$out = cmd /c "`"$vcvars`" >nul 2>&1 && cl $clArgs 2>&1"
$rc  = $LASTEXITCODE
$out | Where-Object { $_ -match 'error|warning' } | ForEach-Object { Write-Host "  $_" }

if ($rc -ne 0 -or -not (Test-Path $dll)) {
    Write-Host "LAYER BUILD FAILED (cl exit $rc)" -ForegroundColor Red
    exit 1
}

# THE EXPORT IS THE WHOLE POINT. The OpenXR loader looks for exactly one symbol; a DLL that builds
# but does not export it is loaded, rejected, and reported only as a line in a loader log nobody
# reads. Assert it here, where the failure is a build failure.
$exports = cmd /c "`"$vcvars`" >nul 2>&1 && dumpbin /exports `"$dll`" 2>&1"
foreach ($sym in 'xrNegotiateLoaderApiLayerInterface', 'halovr_layer_get_api') {
    if (-not ($exports | Select-String -SimpleMatch $sym -Quiet)) {
        Write-Host "LAYER BUILD FAILED - the DLL does not export $sym" -ForegroundColor Red
        exit 1
    }
}

Write-Host ("LAYER OK  ({0:N0} bytes)  -> {1}" -f (Get-Item $dll).Length, $dll) -ForegroundColor Green
Write-Host '  exports xrNegotiateLoaderApiLayerInterface + halovr_layer_get_api' -ForegroundColor DarkGray

if ($Deploy) {
    $live = Join-Path $env:APPDATA 'UnrealVRMod\HaloCampaignEvolved\apilayer'
    if (-not (Test-Path $live)) { New-Item -ItemType Directory -Path $live -Force | Out-Null }
    try {
        Copy-Item -Path $dll      -Destination $live -Force
        Copy-Item -Path $manifest -Destination $live -Force
        Write-Host "DEPLOYED -> $live" -ForegroundColor Green
        Write-Host '  NOT registered. Files on disk load nothing on their own - run' -ForegroundColor DarkGray
        Write-Host '  profile\apilayer\Register-XrApiLayer.ps1 to make the loader pick them up.' -ForegroundColor DarkGray
    } catch {
        Write-Host 'DEPLOY FAILED - the DLL is locked. Close every OpenXR application first.' -ForegroundColor Red
        exit 2
    }
}
