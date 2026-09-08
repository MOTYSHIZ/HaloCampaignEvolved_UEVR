<#
.SYNOPSIS
  Register the Halo VR reticule OpenXR API layer for the CURRENT USER. Reversible with
  Unregister-XrApiLayer.ps1.

.DESCRIPTION
  WHAT THIS DOES, PLAINLY: it writes ONE value under HKEY_CURRENT_USER telling the OpenXR loader
  where this folder's XrApiLayer_HALOVR_reticule.json is. Nothing is copied, nothing is installed,
  no service is created, no file outside this folder is touched, and no administrator rights are
  needed.

  WHY IT IS NEEDED AT ALL. The mod draws the aim reticule as an OpenXR composition layer, which is
  submitted to your headset AFTER the game's post-processing - that is what makes it stay bright in
  sunlight and not blow out in shade, instead of being dimmed and tonemapped along with the scene.
  Reaching that stage of the pipeline requires an OpenXR "API layer", and the OpenXR loader only
  loads layers it has been told about. Steam launches the game itself, so there is no opportunity to
  set an environment variable for one run - which leaves this registry value as the way to say it.

  WHAT ELSE IS AFFECTED. An implicit API layer is loaded into EVERY OpenXR application you run, not
  only this game. That is unavoidable and it is why the layer is written to notice immediately that
  it is not in Halo and do nothing at all - no interception, no work, no files. If you would still
  rather it were not present in your other titles, do not run this: the mod works without it and
  falls back to the in-game reticule, and you can also switch it off at any time by setting the
  environment variable HALOVR_LAYER_DISABLE=1.

  TO UNDO: run Unregister-XrApiLayer.ps1 from this same folder. It removes the value it added and
  nothing else.

  IF YOU RUN THE GAME AS ADMINISTRATOR this will not take effect. The OpenXR loader deliberately
  ignores per-user layer registrations in elevated processes, so that a normal-privilege program
  cannot inject code into an elevated one. Run the game normally, or do not use this feature.

.PARAMETER ManifestPath
  The layer manifest to register. Defaults to the one beside this script, which is where a normal
  install puts it.

.PARAMETER WhatIfOnly
  Print what would be written and change nothing.
#>
[CmdletBinding()]
param(
    [string]$ManifestPath,
    [switch]$WhatIfOnly
)

$ErrorActionPreference = 'Stop'

# The registry location is fixed by the OpenXR specification: SOFTWARE\Khronos\OpenXR\<major>\...
# The <major> is the API major version, which is 1 for every OpenXR release to date.
$RegPath = 'HKCU:\SOFTWARE\Khronos\OpenXR\1\ApiLayers\Implicit'

if (-not $ManifestPath) {
    $ManifestPath = Join-Path $PSScriptRoot 'XrApiLayer_HALOVR_reticule.json'
}
$ManifestPath = [System.IO.Path]::GetFullPath($ManifestPath)

Write-Host 'Halo VR - OpenXR reticule layer registration' -ForegroundColor Cyan
Write-Host '--------------------------------------------'

# ---- checks before touching anything -------------------------------------------------------
if (-not (Test-Path -LiteralPath $ManifestPath)) {
    Write-Host "NOT REGISTERED: no manifest at" -ForegroundColor Red
    Write-Host "  $ManifestPath"
    Write-Host 'Run this script from the apilayer folder inside your UEVR profile for this game.'
    exit 1
}

# The manifest names its DLL with a relative path, so the loader looks for it beside the manifest.
# A registration pointing at a manifest whose DLL is missing produces a warning in a loader log
# nobody reads, and the reticule simply never appears - so check it here, where it can be said out
# loud.
$dll = Join-Path (Split-Path -Parent $ManifestPath) 'XrApiLayer_HALOVR_reticule.dll'
if (-not (Test-Path -LiteralPath $dll)) {
    Write-Host 'NOT REGISTERED: the manifest is here but its DLL is not.' -ForegroundColor Red
    Write-Host "  expected: $dll"
    Write-Host 'Re-extract the release zip; both files belong in this folder, side by side.'
    exit 1
}

if ($WhatIfOnly) {
    Write-Host 'WHAT-IF - nothing will be written.' -ForegroundColor Yellow
    Write-Host "  key:   $RegPath"
    Write-Host "  name:  $ManifestPath"
    Write-Host '  value: 0  (DWORD; 0 means enabled, any other value means the loader skips it)'
    exit 0
}

# ---- the write -------------------------------------------------------------------------------
if (-not (Test-Path -LiteralPath $RegPath)) {
    New-Item -Path $RegPath -Force | Out-Null
}

# IDEMPOTENT, and it has to be: players re-run installers. Registering the same path twice is a
# single value either way, and re-running after an update is exactly how a stale path gets corrected.
$existing = Get-ItemProperty -LiteralPath $RegPath -ErrorAction SilentlyContinue

# Clear out any OLD registration of this same layer from a DIFFERENT folder. Without this, a player
# who moves or reinstalls their profile ends up with two entries - the stale one pointing at a
# manifest that no longer exists, or worse at an older DLL that the plugin will then refuse on ABI
# grounds while the correct one sits unused.
$stale = @()
if ($existing) {
    $stale = @($existing.PSObject.Properties |
        Where-Object { $_.Name -like '*XrApiLayer_HALOVR_reticule.json' -and $_.Name -ne $ManifestPath } |
        ForEach-Object { $_.Name })
}
foreach ($old in $stale) {
    Remove-ItemProperty -LiteralPath $RegPath -Name $old -ErrorAction SilentlyContinue
    Write-Host "  removed a previous registration: $old" -ForegroundColor DarkYellow
}

New-ItemProperty -LiteralPath $RegPath -Name $ManifestPath -Value 0 -PropertyType DWord -Force | Out-Null

# ---- read it back ----------------------------------------------------------------------------
# Never report success from the fact that a write did not throw. Read the value the loader will
# read, and report THAT.
$check = Get-ItemProperty -LiteralPath $RegPath -Name $ManifestPath -ErrorAction SilentlyContinue
if ($null -eq $check -or $check.$ManifestPath -ne 0) {
    Write-Host 'REGISTRATION FAILED - the value did not read back as expected.' -ForegroundColor Red
    exit 1
}

Write-Host 'REGISTERED' -ForegroundColor Green
Write-Host "  $ManifestPath"
Write-Host ''
Write-Host 'What happens now:'
Write-Host '  * Start the game and inject as usual. Nothing else changes.'
Write-Host '  * The layer writes halo_vr_layer.log in this folder when it loads into the game. If'
Write-Host '    that file never appears, the loader is not picking the layer up - see'
Write-Host '    docs\TROUBLESHOOTING.md.'
Write-Host '  * To undo: Unregister-XrApiLayer.ps1, in this folder.'
Write-Host '  * To disable without unregistering: set the environment variable HALOVR_LAYER_DISABLE=1'
