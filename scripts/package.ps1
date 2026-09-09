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

# THE SECOND DLL. The OpenXR API layer is what carries the compositor reticule on a player's
# machine - the plugin's other attachment needs UEVRBackend.pdb, which only exists in a UEVR
# checkout. It is built separately because it IS separate: its own DllMain, its own loader-facing
# export, loaded by the OpenXR loader rather than by UEVR. It needs no UEVR SDK, so it builds even
# where the plugin could not.
& (Join-Path $PSScriptRoot 'build-apilayer.ps1')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$stage = Join-Path $repo 'build\stage'
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Path (Join-Path $stage 'plugins') -Force | Out-Null

Copy-Item (Join-Path $repo 'profile\*') $stage -Recurse -Force
Remove-Item (Join-Path $stage 'plugins\.gitkeep') -ErrorAction SilentlyContinue
Copy-Item (Join-Path $repo 'build\halo_vr.dll') (Join-Path $stage 'plugins\') -Force

# The layer's manifest and register/unregister scripts came across with profile\ above; only its
# DLL is a build output. The two must land SIDE BY SIDE - the manifest names the DLL by a relative
# path, so the loader looks for it in the manifest's own folder and a split pair registers cleanly
# and then loads nothing.
Copy-Item (Join-Path $repo 'build\XrApiLayer_HALOVR_reticule.dll') (Join-Path $stage 'apilayer\') -Force

# SHIPPING MANIFEST. The copy above takes profile\ wholesale, so a file that exists on the author's
# disk but was never committed is simply absent in a CI checkout -- the zip builds fine and ships
# broken. The plugin degrades quietly in exactly that case (a missing cutscene_hint.png just turns
# the hint off), so nothing downstream would report it either. Assert the payload instead: CI fails
# here, before a release exists, rather than a user finding out.
$required = @('config.txt', 'halo_vr.cfg', 'halo_vr_dev.cfg', 'halo_vr_user_reference.txt',
              'cvars_data.txt', 'user_script.txt',
              'reticle_ring.png', 'cutscene_hint.png',
              'scripts\halo_vr_settings.lua',
              'plugins\halo_vr.dll',
              # The OpenXR API layer ships as a matched SET, and every member is load-bearing:
              # the DLL is the code, the manifest is the only thing that tells the loader the DLL
              # exists, and the two scripts are the only way a player can turn it on or off. Ship
              # three of the four and the feature is either uninstallable or unremovable.
              'apilayer\XrApiLayer_HALOVR_reticule.dll',
              'apilayer\XrApiLayer_HALOVR_reticule.json',
              'apilayer\Register-XrApiLayer.ps1',
              'apilayer\Unregister-XrApiLayer.ps1')
$missing = @($required | Where-Object { -not (Test-Path (Join-Path $stage $_)) })
if ($missing.Count -gt 0) {
    throw ("Release payload incomplete -- missing: {0}`n" -f ($missing -join ', ')) +
          "If the file exists locally, it is probably UNTRACKED: commit it, or CI ships without it."
}

# The inverse checks. USER-OWNED files must never ship: the zip OVERWRITES whatever it contains
# on upgrade (UEVR's Import Config merges file-by-file), so shipping one would clobber every
# player's kept settings -- their absence from the zip is exactly what makes settings survive
# updates and "delete halo_vr_user.cfg" mean "back to shipped defaults".
# halo_vr_weapons.cfg joined this list with PR #7: the plugin REWRITES it in full on every
# per-weapon capture, so shipping one would replace a player's captured weapon deltas wholesale.
# It matters more since the per-weapon calibration gained a SHIPPED baseline in halo_vr.cfg: this
# file is now the OVERRIDE tier over that baseline, parsed last and winning weapon by weapon. Ship
# one and every player inherits the author's captures permanently -- and because those outrank the
# baseline, no future release could correct them.
#
# apilayer\halo_vr_layer.log joined the list because the layer WRITES IT into its own folder at
# runtime, which means an author who has ever run the game has one sitting in profile\apilayer\
# waiting to be swept up by the wholesale copy above. Shipping it is harmless in itself and
# thoroughly confusing: every player would receive a log of somebody else's session, in the exact
# file the troubleshooting guide tells them to read to find out whether the layer loaded.
$forbidden = @('halo_vr_user.cfg', 'halo_vr_calib.cfg', 'halo_vr_calib_left.cfg',
                'halo_vr_weapons.cfg', 'apilayer\halo_vr_layer.log',
                # The third-party cutscene-detection plugin was RETIRED 2026-09-08: our own fix
                # (the movie as an OpenXR quad, cutscenemono=6) replaced it and it is gone from the
                # repo. The profile is copied wholesale, so an author who still has it in a live
                # %APPDATA% profile could drag a stray copy back into profile\plugins\ and ship it
                # unnoticed. Fail here instead. Also of note: its licensing was never cleared for
                # redistribution (docs\GameRecon\Elliotttate-Study.md), so shipping it was a risk
                # regardless of the feature.
                'plugins\CutsceneDetectionPlugin.dll') |
    Where-Object { Test-Path (Join-Path $stage $_) }
if ($forbidden.Count -gt 0) {
    throw ("Release payload contains files that must not ship: {0}" -f ($forbidden -join ', '))
}

# The layer manifest must keep a RELATIVE library_path. An absolute one would be the author's own
# disk baked into every player's install: the loader would look for the DLL at a path that does not
# exist on their machine, the layer would never load, and the only symptom is a reticule that is not
# there. Relative means "beside this manifest", which is true for everyone.
$layerManifest = Join-Path $stage 'apilayer\XrApiLayer_HALOVR_reticule.json'
$libPath = (Get-Content -Raw $layerManifest | ConvertFrom-Json).api_layer.library_path
if ($libPath -notmatch '^\.[\\/]' ) {
    throw "The API layer manifest's library_path must be relative to the manifest (got: $libPath)."
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
# wpnfix / wpnfixver joined this list with the shipped PER-WEAPON calibration: they are fitted
# measurements exactly like grip and offx, not settings, so halo_vr.cfg is their home and the
# compiled Config.hpp defaults are not (a struct default cannot express "the assault rifle needs
# 2 degrees"). wpnfixver is the frame stamp and is as load-bearing as calibver -- shipping the
# values without it would have every one of them silently ignored.
$calibKeys = @('grip','gripyaw','griproll','calibver','offx','offy','offz','aimoffyaw','aimoffpitch',
               'aimcalibver','dirgrip','dirgripyaw','dirgriproll','diroffx','diroffy','diroffz',
               'pivauto','pivx','pivy','pivz','calibrelative','wpnfix','wpnfixver','wpnscope','wpnoff')
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
