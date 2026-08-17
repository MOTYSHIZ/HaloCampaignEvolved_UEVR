<#
.SYNOPSIS
  Build the plugin and assemble the release zip.

.DESCRIPTION
  Produces build\HaloCampaignEvolved.zip whose root is the PROFILE CONTENTS (config.txt at the zip
  root, the DLL under plugins\). That exact layout is what makes UEVR's "Import Config" a one-click
  install into %APPDATA%\UnrealVRMod\HaloCampaignEvolved\. The zip is named after the game's exe
  basename on purpose - UEVR keys profiles on it.

  Prints the zip's SHA-256 for the release notes: an injected DLL is exactly the kind of file
  antivirus flags, and a published hash lets users verify what they downloaded.
#>
[CmdletBinding()]
param(
    [string]$SdkPath = $env:UEVR_SDK
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot

& (Join-Path $PSScriptRoot 'build.ps1') -SdkPath $SdkPath
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$stage = Join-Path $repo 'build\stage'
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Path (Join-Path $stage 'plugins') -Force | Out-Null

Copy-Item (Join-Path $repo 'profile\*') $stage -Recurse -Force
Remove-Item (Join-Path $stage 'plugins\.gitkeep') -ErrorAction SilentlyContinue
Copy-Item (Join-Path $repo 'build\halo_vr.dll') (Join-Path $stage 'plugins\') -Force

# SHIPPING MANIFEST. The copy above takes profile\ wholesale, so a file that exists on the author's
# disk but was never committed is simply absent in a CI checkout -- the zip builds fine and ships
# broken. The plugin degrades quietly in exactly that case (a missing cutscene_hint.png just turns
# the hint off), so nothing downstream would report it either. Assert the payload instead: CI fails
# here, before a release exists, rather than a user finding out.
$required = @('config.txt', 'halo_vr.cfg', 'halo_vr_dev.cfg', 'halo_vr_user_reference.txt',
              'cvars_data.txt', 'user_script.txt',
              'reticle_ring.png', 'cutscene_hint.png',
              'scripts\halo_vr_settings.lua',
              'plugins\halo_vr.dll', 'plugins\CutsceneDetectionPlugin.dll')
$missing = @($required | Where-Object { -not (Test-Path (Join-Path $stage $_)) })
if ($missing.Count -gt 0) {
    throw ("Release payload incomplete -- missing: {0}`n" -f ($missing -join ', ')) +
          "If the file exists locally, it is probably UNTRACKED: commit it, or CI ships without it."
}

# The inverse checks. USER-OWNED files must never ship: the zip OVERWRITES whatever it contains
# on upgrade (UEVR's Import Config merges file-by-file), so shipping one would clobber every
# player's kept settings -- their absence from the zip is exactly what makes settings survive
# updates and "delete halo_vr_user.cfg" mean "back to shipped defaults".
$forbidden = @('halo_vr_user.cfg', 'halo_vr_calib.cfg', 'halo_vr_calib_left.cfg') |
    Where-Object { Test-Path (Join-Path $stage $_) }
if ($forbidden.Count -gt 0) {
    throw ("Release payload contains user-owned files -- must not ship: {0}" -f ($forbidden -join ', '))
}

# Both shipped catalogs must be INERT -- every line commented. The dev catalog is parsed after
# halo_vr_user.cfg, so one uncommented key would silently override every player's settings; the
# reference catalog is the menu players copy lines from verbatim, so it must document every key
# in its inert form.
foreach ($catalog in 'halo_vr_dev.cfg', 'halo_vr_user_reference.txt') {
    if (Select-String -Path (Join-Path $stage $catalog) -Pattern '^[A-Za-z0-9_]+=' -Quiet) {
        throw "$catalog in the payload has UNCOMMENTED keys -- shipped catalogs must be inert."
    }
}

# And the shipped halo_vr.cfg may carry ONLY the calibration data. Settings defaults live in the
# compiled plugin (Config.hpp), so any other active key here would be a shipped override nobody
# decided on -- and one an update would silently clobber.
$calibKeys = @('grip','gripyaw','griproll','calibver','offx','offy','offz','aimoffyaw','aimoffpitch',
               'aimcalibver','dirgrip','dirgripyaw','dirgriproll','diroffx','diroffy','diroffz',
               'pivauto','pivx','pivy','pivz','calibrelative')
$cfgActive = @(Select-String -Path (Join-Path $stage 'halo_vr.cfg') -Pattern '^([A-Za-z0-9_]+)=' |
    ForEach-Object { $_.Matches[0].Groups[1].Value.ToLower() })
$stray = @($cfgActive | Where-Object { $calibKeys -notcontains $_ })
if ($stray.Count -gt 0) {
    throw ("halo_vr.cfg in the payload has non-calibration keys (settings belong in code + the catalogs): {0}" -f ($stray -join ', '))
}

$zip = Join-Path $repo 'build\HaloCampaignEvolved.zip'
if (Test-Path $zip) { Remove-Item -Force $zip }
Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $zip

$hash = (Get-FileHash -Algorithm SHA256 $zip).Hash
Write-Host ("PACKAGED  {0}  ({1:N0} bytes)" -f $zip, (Get-Item $zip).Length) -ForegroundColor Green
Write-Host ("SHA-256   {0}" -f $hash) -ForegroundColor Green
