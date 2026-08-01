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

$zip = Join-Path $repo 'build\HaloCampaignEvolved.zip'
if (Test-Path $zip) { Remove-Item -Force $zip }
Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $zip

$hash = (Get-FileHash -Algorithm SHA256 $zip).Hash
Write-Host ("PACKAGED  {0}  ({1:N0} bytes)" -f $zip, (Get-Item $zip).Length) -ForegroundColor Green
Write-Host ("SHA-256   {0}" -f $hash) -ForegroundColor Green
