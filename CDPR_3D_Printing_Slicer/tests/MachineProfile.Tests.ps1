#requires -Version 5.1
[CmdletBinding()]
param()
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectDirectory = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$repositoryDirectory = [IO.Path]::GetFullPath((Join-Path $projectDirectory '..'))
$prepare = Join-Path $projectDirectory 'tools\New-CdprMachineProfile.ps1'
$testDirectory = Join-Path $projectDirectory ('build\machine-profile-tests-' + [Guid]::NewGuid().ToString('N'))
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
$rootConfig = Join-Path $repositoryDirectory 'RobotConfig.json'
$rootHash = (Get-FileHash -LiteralPath $rootConfig -Algorithm SHA256).Hash
$profileName = 'test-' + [Guid]::NewGuid().ToString('N')
$result = & $prepare -RobotConfig $rootConfig -Name $profileName
$planner = Get-Content -LiteralPath $result.PlannerConfig -Raw | ConvertFrom-Json
$calibration = Get-Content -LiteralPath (Join-Path $result.Directory 'calibration.local.json') -Raw | ConvertFrom-Json
$limits = Get-Content -LiteralPath (Join-Path $result.Directory 'limits.local.json') -Raw | ConvertFrom-Json
$report = Get-Content -LiteralPath $result.Report -Raw | ConvertFrom-Json
Check ($planner.robot_model.configuration_path -eq $rootConfig -and $planner.robot_model.expected_robot_config_sha256 -eq $rootHash.ToLowerInvariant()) 'Candidate binds the exact selected configuration bytes in place'
Check ($null -eq $planner.robot_model.rail_offsets_m -and $null -eq $planner.robot_model.starting_rail_offsets_m) 'No zero or recovered rail state is invented'
Check ($null -eq $planner.import.initial_state.nozzle_pose_in_bed -and $null -eq $planner.import.initial_state.filament_position_mm) 'Configured home and synthetic datums are not used as current state'
Check ($calibration.frames.status -eq 'unverified' -and $null -eq $calibration.frames.world_from_bed -and $null -eq $calibration.frames.end_effector_from_nozzle) 'Missing registration stays unverified and empty'
Check ($calibration.provenance.calibration_id -eq '' -and $calibration.provenance.method -eq '') 'No calibration identity or method is fabricated'
Check ($null -eq $limits.limits.maximum_cable_speed_m_per_s -and $null -eq $limits.limits.maximum_path_acceleration_m_per_s2) 'Operating limits remain unknown'
$output = [IO.Path]::GetFullPath((Join-Path $result.Directory $planner.output_directory))
Check ($output -eq (Join-Path $projectDirectory 'out')) 'Generated output path resolves to the existing guarded out directory'
Check (-not $planner.hardware_access -and -not $report.hardware_access -and -not $report.ready_for_planning -and $report.status -eq 'incomplete_candidate') 'Package explicitly remains an incomplete offline candidate'
Check ($report.configured_geometry.frame_attachments_m.Count -eq 8 -and $report.required_inputs.Count -eq 6) 'Source observations and six missing measurement groups are recorded'
$before = (Get-FileHash -LiteralPath $result.PlannerConfig -Algorithm SHA256).Hash
Reject { & $prepare -RobotConfig $rootConfig -Name $profileName } 'already exists' 'Existing candidate directory cannot be overwritten'
Check ((Get-FileHash -LiteralPath $result.PlannerConfig -Algorithm SHA256).Hash -eq $before) 'Existing candidate remains byte identical'
Reject { & $prepare -RobotConfig $rootConfig -Name '..\escape' } 'profile name' 'Reject output name traversal'
Reject { & $prepare -RobotConfig $rootConfig -Name 'CON' } 'profile name' 'Reject reserved device names'
$badConfig = Join-Path $testDirectory 'bad.json'
[IO.File]::WriteAllText($badConfig, '{"ModelName":"bad","cableMotorNum":7}')
Reject { & $prepare -RobotConfig $badConfig -Name ('bad-' + [Guid]::NewGuid().ToString('N')) } 'Missing RobotConfig field' 'Malformed geometry input creates no package'

$alternate = Join-Path $repositoryDirectory 'bin\RobotConfig_YES.json'
$alternateResult = & $prepare -RobotConfig $alternate -Name ('test-yes-' + [Guid]::NewGuid().ToString('N'))
$alternateReport = Get-Content -LiteralPath $alternateResult.Report -Raw | ConvertFrom-Json
Check ($alternateReport.selected_configuration.model_name -eq 'YES' -and $alternateReport.selected_configuration.sha256 -ne $report.selected_configuration.sha256) 'Alternate robot selection records a distinct model identity'

$plannerExecutable = Join-Path $projectDirectory 'build\offline\bin\Debug\cdpr-slicer-plan.exe'
$rejectedName = 'machine-incomplete-' + [Guid]::NewGuid().ToString('N')
$nativeErrorPreference = $ErrorActionPreference
try {
    $ErrorActionPreference = 'Continue'
    $errorOutput = & $plannerExecutable --input (Join-Path $projectDirectory 'tests\fixtures\modal.gcode') --config $result.PlannerConfig --stage timed --output ($rejectedName + '.json') 2>&1
    $nativeExitCode = $LASTEXITCODE
} finally { $ErrorActionPreference = $nativeErrorPreference }
Check ($nativeExitCode -ne 0) 'Existing native planner rejects the incomplete package'
Check (($errorOutput | Out-String) -match 'explicit starting nozzle pose') 'Native rejection identifies the missing measurement'
Check (-not (Test-Path -LiteralPath (Join-Path $projectDirectory ('out\' + $rejectedName + '.json')))) 'Incomplete package publishes no planning artifact'
Check ((Get-FileHash -LiteralPath $rootConfig -Algorithm SHA256).Hash -eq $rootHash) 'Original RobotConfig stays byte identical'
[IO.File]::WriteAllText((Join-Path $testDirectory 'results.json'), ([ordered]@{
    powershell = $PSVersionTable.PSVersion.ToString(); checks = $checks; candidate = $result;
    native_rejection = ($errorOutput | Out-String).Trim(); source_sha256 = $rootHash
} | ConvertTo-Json -Depth 8))
Write-Host "$($checks.Count) machine profile checks passed. Evidence: $testDirectory"
