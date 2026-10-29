#requires -Version 5.1
[CmdletBinding()]
param([string] $PrusaSlicer, [string] $PlannerExecutable)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectDirectory = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$wrapper = Join-Path $projectDirectory 'tools\Invoke-CdprSlice.ps1'
if (-not $PrusaSlicer) { $PrusaSlicer = Join-Path $projectDirectory 'build\tools\PrusaSlicer-2.9.6\PrusaSlicer-2.9.6\prusa-slicer-console.exe' }
if (-not $PlannerExecutable) { $PlannerExecutable = Join-Path $projectDirectory 'build\offline\bin\Debug\cdpr-slicer-plan.exe' }
$testDirectory = Join-Path $projectDirectory ('build\slice-tests-' + [Guid]::NewGuid().ToString('N'))
[IO.Directory]::CreateDirectory($testDirectory) | Out-Null
$checks = [Collections.Generic.List[string]]::new()
function Check([bool] $Condition, [string] $Name) {
    if (-not $Condition) { throw "FAIL: $Name" }
    $checks.Add($Name)
    Write-Host "PASS: $Name"
}
function Reject([scriptblock] $Action, [string] $Pattern, [string] $Name) {
    $message = ''
    try { & $Action | Out-Null } catch { $message = $_.Exception.Message }
    Check ($message -match $Pattern) $Name
}
$settings = @{
    PlannerConfig = Join-Path $projectDirectory 'config\synthetic-planner.json'
    PrusaSlicer = $PrusaSlicer
    PlannerExecutable = $PlannerExecutable
}
$cube = Join-Path $projectDirectory 'tests\fixtures\cube-10mm.stl'
Reject { & $wrapper @settings -Model $cube -RunName '..\escape' } 'RunName' 'Reject path traversal in output names'
Reject { & $wrapper @settings -Model (Join-Path $projectDirectory 'tests\fixtures\modal.gcode') } 'STL models only' 'Reject non-STL model input'
Reject { & $wrapper @settings -Model (Join-Path $testDirectory 'missing.stl') } 'does not exist' 'Reject missing input'
Reject { & $wrapper @settings -Model $cube -CenterXYMm @(0, 0, 0) } 'two finite' 'Reject invalid placement coordinate count'
Reject { & $wrapper @settings -Model $cube -CenterXYMm @([double]::NaN, 0) } 'two finite' 'Reject nonfinite placement coordinates'
Reject { & $wrapper @settings -Model (Join-Path $projectDirectory 'tests\fixtures\cube-500mm.stl') } '(?s)no G-code.*outside of the print volume' 'Report engine print-volume rejection when exit status is zero'

# A copied entry point in a generated project tests the execution guard without
# changing the reviewed profile or existing project files.
$guardProject = Join-Path $testDirectory 'guard-project'
[IO.Directory]::CreateDirectory((Join-Path $guardProject 'tools')) | Out-Null
[IO.Directory]::CreateDirectory((Join-Path $guardProject 'config')) | Out-Null
$guardWrapper = Join-Path $guardProject 'tools\Invoke-CdprSlice.ps1'
[IO.File]::Copy($wrapper, $guardWrapper, $false)
$guardProfile = Join-Path $guardProject 'config\prusa-cdpr.ini'
$profileText = [IO.File]::ReadAllText((Join-Path $projectDirectory 'config\prusa-cdpr.ini'))
[IO.File]::WriteAllText($guardProfile, $profileText.Replace('post_process =', 'post_process = must-never-run'))
Reject { & $guardWrapper @settings -Model $cube } 'post_process' 'Reject post-processing scripts before engine startup'
Check (-not (Test-Path -LiteralPath (Join-Path $guardProject 'build'))) 'Rejected profile creates no working files'
[IO.File]::WriteAllText($guardProfile, ($profileText + "`npost_process = duplicate`n"))
Reject { & $guardWrapper @settings -Model $cube } 'post_process' 'Reject duplicate profile control keys'
[IO.File]::WriteAllText($guardProfile, $profileText.Replace('binary_gcode = 0', 'binary_gcode = 1'))
Reject { & $guardWrapper @settings -Model $cube } 'binary_gcode' 'Reject binary G-code producer settings'
Reject { & $wrapper @settings -Model $cube -SlicingProfile $guardProfile } 'binary_gcode' 'Apply producer guards to an explicit custom profile'
[IO.File]::WriteAllText($guardProfile, $profileText.Replace('start_gcode = G21', 'start_gcode = G28\nG21'))
$rejectedName = 'rejected-cube-' + [Guid]::NewGuid().ToString('N')
Reject { & $guardWrapper @settings -Model $cube -RunName $rejectedName } 'Unsupported G command' 'Unsupported generated commands propagate as importer errors'
Check (-not (Test-Path -LiteralPath (Join-Path $projectDirectory ('out\' + $rejectedName + '.json')))) 'Rejected generated G-code publishes no plan'

# Spaces, apostrophes and Unicode must survive the process boundary literally.
$unicodeName = 'cube ''model'' ' + [char]0x6A21 + [char]0x578B + '.stl'
$modelCopy = Join-Path $testDirectory $unicodeName
[IO.File]::Copy($cube, $modelCopy, $false)
$runName = 'cube-acceptance-' + [Guid]::NewGuid().ToString('N')
$result = & $wrapper @settings -Model $modelCopy -RunName $runName
$receipt = [IO.File]::ReadAllText($result.Receipt) | ConvertFrom-Json
Check ($receipt.model.path -eq $modelCopy -and $receipt.engine.version -eq '2.9.6') 'Real STL slicing accepts literal Unicode and spaced paths'
foreach ($kind in @('gcode', 'json', 'html')) {
    $entry = $receipt.outputs.$kind
    Check ((Get-FileHash -LiteralPath $entry.path -Algorithm SHA256).Hash.ToLowerInvariant() -eq $entry.sha256) "Receipt binds exact $kind output bytes"
}
Check ($receipt.model.sha256 -eq (Get-FileHash -LiteralPath $cube -Algorithm SHA256).Hash.ToLowerInvariant()) 'Receipt binds the original cube mesh'
$job = [IO.File]::ReadAllText($result.Job) | ConvertFrom-Json
Check ($job.stage -eq 'timed' -and -not $job.validation.hardware_ready -and -not $receipt.hardware_access) 'Full pipeline produces an explicitly offline timed artifact'
Check ($job.source.sha256 -eq $receipt.outputs.gcode.sha256 -and $job.source.profile_sha256 -eq $receipt.planner.configuration_sha256) 'Plan source and profile hashes match the receipt'
Check (@($job.validation.checks | Where-Object { $_.status -eq 'passed' }).Count -eq 8) 'All eight implemented planning checks pass'
$moves = @($job.operations | Where-Object { $null -ne $_.motion })
$deposits = @($moves | Where-Object { @($_.motion.extrusion_spans | Where-Object { $_.purpose -eq 'deposit' }).Count -gt 0 })
$heights = @($deposits | ForEach-Object { [Math]::Round($_.motion.nozzle.end.position_m[2], 9) } | Sort-Object -Unique)
Check ($heights.Count -eq 40 -and [Math]::Abs($heights[0] - 0.00025) -lt 1e-10 -and [Math]::Abs($heights[-1] - 0.01) -lt 1e-10) 'Cube has 40 layers from 0.25 mm through 10 mm'
$xValues = @($deposits | ForEach-Object { $_.motion.nozzle.start.position_m[0]; $_.motion.nozzle.end.position_m[0] })
$yValues = @($deposits | ForEach-Object { $_.motion.nozzle.start.position_m[1]; $_.motion.nozzle.end.position_m[1] })
foreach ($axis in @($xValues, $yValues)) {
    $extent = $axis | Measure-Object -Minimum -Maximum
    Check ($extent.Minimum -ge 0.045 -and $extent.Minimum -lt 0.0453 -and $extent.Maximum -le 0.055 -and $extent.Maximum -gt 0.0547) 'Deposited centerlines preserve the centered 10 mm cube footprint'
}
Check (@($moves | Where-Object { $_.motion.delta_filament_mm -lt 0 }).Count -gt 0 -and $deposits.Count -gt 100) 'Real path retains retractions and material deposition'
$sample = $deposits[0].motion
Check ([Math]::Abs($sample.end_effector.end.position_m[0] - $sample.nozzle.end.position_m[0] - 0.1) -lt 1e-10 -and
    [Math]::Abs($sample.end_effector.end.position_m[2] - $sample.nozzle.end.position_m[2] + 0.05) -lt 1e-10) 'Cube uses the registered bed and nozzle transforms'
$before = (Get-FileHash -LiteralPath $result.Receipt -Algorithm SHA256).Hash
Reject { & $wrapper @settings -Model $cube -RunName $runName } 'already exists' 'Repeated run name refuses to overwrite outputs'
Check ((Get-FileHash -LiteralPath $result.Receipt -Algorithm SHA256).Hash -eq $before) 'Existing receipt remains byte identical'

[IO.File]::WriteAllText((Join-Path $testDirectory 'results.json'), ([ordered]@{
    powershell = $PSVersionTable.PSVersion.ToString()
    checks = $checks
    artifacts = $result
    motion_count = $moves.Count
    layer_count = $heights.Count
    slicing_s = $receipt.duration_s.slicing
    planning_s = $receipt.duration_s.planning_and_preview
} | ConvertTo-Json -Depth 8))
Write-Host "$($checks.Count) workflow checks passed. Evidence: $testDirectory"
