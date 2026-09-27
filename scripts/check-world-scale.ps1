<#
.SYNOPSIS
  Fail if any source reads or writes UEVR's VR_WorldScale directly, instead of through core/WorldScale.

.DESCRIPTION
  The world scale is the PLAYER'S: UEVR's slider, not a constant of this mod. A real metre of head or
  hand movement is drawn as 100 x VR_WorldScale UE centimetres, so every length or speed that crosses
  between the room (real metres) and the world (UE cm) has to use it. Two failure shapes, both shipped:

    1. A bare x100 / x0.01 between room and world. Exact at world scale 1.0 and wrong by the scale
       everywhere else, with no error anywhere. Roomscale, every holster and HUD marker, and the arms'
       rig scale all did this; at the profile's 1.312 the view slid 31% of every roomscale step and
       markers sat 24% short of the hand.
    2. A private reader of VR_WorldScale with its own guard. Five existed, with three different
       guards, and three fell back to 1.0 on the 0.01 the plugin's own cutscene mono collapse writes.

  So there is ONE reader, src/core/WorldScale.{hpp,cpp}: uevr_world_scale() / uevr_cm_per_metre() on
  the game thread, the _cached forms on any other thread. This check enforces the second rule exactly:
  a quoted "VR_WorldScale" anywhere else must carry a marker within 6 lines saying why the RAW value
  is the point, e.g. the collapse saving the player's value before overwriting it:

      // WORLDSCALE-RAW: saves the player's value so the collapse can restore it

  The first rule cannot be linted without false positives, so -Audit lists candidates for a human:
  bare x100 / x0.01 conversions on lines that also touch room-space quantities. Run it before a
  release; it never fails the build.

.PARAMETER Audit
  Also print candidate bare room<->world conversions for review. Exit code is unaffected.

.EXAMPLE
  scripts\check-world-scale.ps1
  scripts\check-world-scale.ps1 -Audit
#>
[CmdletBinding()]
param(
    [switch]$Audit
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$src  = Join-Path $repo 'src'
$reader = 'core\WorldScale.cpp'   # the one file allowed to read it bare

$files = @(Get-ChildItem $src -Recurse -File -Include *.cpp, *.hpp, *.inl, *.h |
           Where-Object { $_.FullName -notmatch '\\thirdparty\\' } | Sort-Object FullName)

$violations = New-Object System.Collections.Generic.List[string]
foreach ($f in $files) {
    $rel = $f.FullName.Substring($src.Length + 1)
    if ($rel -ieq $reader) { continue }
    $lines = [System.IO.File]::ReadAllLines($f.FullName)
    for ($i = 0; $i -lt $lines.Length; $i++) {
        $line = $lines[$i]
        if ($line.IndexOf('"VR_WorldScale"') -lt 0) { continue }
        if ($line.TrimStart().StartsWith('//')) { continue }   # a comment quoting the name is not a read
        $lo = [Math]::Max(0, $i - 6); $hi = [Math]::Min($lines.Length - 1, $i + 6)
        $marked = $false
        for ($k = $lo; $k -le $hi; $k++) { if ($lines[$k].IndexOf('WORLDSCALE-RAW:') -ge 0) { $marked = $true; break } }
        if (-not $marked) { $violations.Add(('  src\{0}:{1}: {2}' -f $rel, ($i + 1), $line.Trim())) }
    }
}

if ($Audit) {
    # Heuristic, for a human: a bare 100 / 0.01 on a line that also names a room-space quantity.
    # Expect noise (log formatting prints metres as cm); what matters is a hit that CONVERTS.
    $conv = '\*\s*100\.0f|/\s*100\.0f|\*\s*0\.01f|100\.0f\s*\*|0\.01f\s*\*'
    $room = 'hmd|room|pose|standing|head|grip|hand|\bso\.|\bhp\.'
    Write-Host 'AUDIT -- candidate bare room<->world conversions (review each; noise expected):'
    $n = 0
    foreach ($f in $files) {
        $rel = $f.FullName.Substring($src.Length + 1)
        $lines = [System.IO.File]::ReadAllLines($f.FullName)
        for ($i = 0; $i -lt $lines.Length; $i++) {
            $t = $lines[$i]
            if ($t.TrimStart().StartsWith('//')) { continue }
            if ($t -match $conv -and $t -match $room -and $t -notmatch 'log_info|hlog|printf|snprintf|WORLDSCALE-RAW') {
                Write-Host ('  src\{0}:{1}: {2}' -f $rel, ($i + 1), $t.Trim()); $n++
            }
        }
    }
    Write-Host ('AUDIT: {0} candidate line(s).' -f $n)
}

if ($violations.Count -gt 0) {
    Write-Host 'WORLD SCALE CHECK FAILED -- VR_WorldScale used outside core/WorldScale:' -ForegroundColor Red
    $violations | ForEach-Object { Write-Host $_ }
    Write-Host ''
    Write-Host 'Read the scale through src/core/WorldScale.hpp: uevr_world_scale() / uevr_cm_per_metre() on the'
    Write-Host 'game thread, uevr_world_scale_cached() / uevr_cm_per_metre_cached() on any other thread.'
    Write-Host 'If the RAW value is genuinely the point, say why within 6 lines:  // WORLDSCALE-RAW: <reason>'
    exit 1
}
Write-Host ('World scale check: OK ({0} source files; every VR_WorldScale use goes through core/WorldScale or is marked).' -f $files.Count)
exit 0
