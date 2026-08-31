<#
.SYNOPSIS
  Remove the Halo VR reticule OpenXR API layer registration for the current user.

.DESCRIPTION
  The exact inverse of Register-XrApiLayer.ps1. It deletes the registry values that name THIS
  layer's manifest and nothing else - other vendors' layers under the same key (Virtual Desktop,
  Meta, SteamVR, OpenXR Toolkit and so on) are left completely alone.

  It is safe to run when nothing is registered: it says so and exits 0. It does not delete any
  files - the layer DLL and manifest stay where they are, simply unused, so re-registering later is
  one command and no re-download.

  IF YOU ARE HERE BECAUSE ANOTHER VR GAME MISBEHAVES: run this, and the layer stops being loaded
  anywhere. You can also leave it registered and set the environment variable HALOVR_LAYER_DISABLE=1,
  which the layer honours on every activation route. Either way the Halo mod keeps working; it falls
  back to the in-game reticule.

.PARAMETER All
  Remove EVERY registration of this layer, from any folder, rather than only the one beside this
  script. Use this if you have moved or reinstalled your profile and a stale entry was left behind.
#>
[CmdletBinding()]
param(
    [switch]$All
)

$ErrorActionPreference = 'Stop'

$RegPath = 'HKCU:\SOFTWARE\Khronos\OpenXR\1\ApiLayers\Implicit'
$Manifest = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'XrApiLayer_HALOVR_reticule.json'))

Write-Host 'Halo VR - OpenXR reticule layer removal' -ForegroundColor Cyan
Write-Host '---------------------------------------'

if (-not (Test-Path -LiteralPath $RegPath)) {
    Write-Host 'Nothing to do - no OpenXR implicit-layer registrations exist for this user.'
    exit 0
}

$props = Get-ItemProperty -LiteralPath $RegPath -ErrorAction SilentlyContinue
if (-not $props) {
    Write-Host 'Nothing to do - the key is empty.'
    exit 0
}

# Match on OUR manifest filename only. This is the line that keeps the script from being destructive:
# the key is shared with every other OpenXR layer the user has installed, and a wildcard sweep here
# would silently disable their headset streamer.
$targets = @($props.PSObject.Properties |
    Where-Object { $_.Name -like '*XrApiLayer_HALOVR_reticule.json' } |
    ForEach-Object { $_.Name })

if (-not $All) {
    $targets = @($targets | Where-Object { $_ -eq $Manifest })
}

if ($targets.Count -eq 0) {
    Write-Host 'Nothing to do - this layer is not registered.'
    if (-not $All) {
        Write-Host 'If you moved your profile, an entry may still point at the old folder.'
        Write-Host 'Re-run with -All to remove every registration of this layer.'
    }
    exit 0
}

foreach ($t in $targets) {
    Remove-ItemProperty -LiteralPath $RegPath -Name $t -ErrorAction SilentlyContinue
    # Read back rather than trusting the removal. A value that is still there after a delete that
    # did not throw is exactly the sort of thing that turns into "I uninstalled it and it is still
    # happening".
    $still = Get-ItemProperty -LiteralPath $RegPath -Name $t -ErrorAction SilentlyContinue
    if ($null -ne $still) {
        Write-Host "FAILED to remove: $t" -ForegroundColor Red
        exit 1
    }
    Write-Host "  removed  $t" -ForegroundColor Green
}

Write-Host ''
Write-Host 'UNREGISTERED. The layer is no longer loaded into any application.' -ForegroundColor Green
Write-Host 'The files are still in this folder; Register-XrApiLayer.ps1 turns it back on.'
