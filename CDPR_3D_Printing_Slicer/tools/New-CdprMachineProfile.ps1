#requires -Version 5.1
<#
.SYNOPSIS
Prepare an incomplete, model-pinned offline machine profile for measurement.
.EXAMPLE
& .\tools\New-CdprMachineProfile.ps1 -RobotConfig ..\RobotConfig.json -Name yes-candidate
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string] $RobotConfig,
    [Parameter(Mandatory = $true)][string] $Name
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectDirectory = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))

function Local-Path([string] $Path) {
    $full = [IO.Path]::GetFullPath($ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Path))
    if ($full -notmatch '^[A-Za-z]:\\' -or $full.Substring(2).Contains(':') -or $full.Length -ge 240) {
        throw 'Use a conventional local drive path shorter than 240 characters.'
    }
    $cursor = $full
    while ($cursor) {
        if (Test-Path -LiteralPath $cursor) {
            if (([IO.File]::GetAttributes($cursor) -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
                throw "Reparse points are not supported: $cursor"
            }
        }
        $cursor = [IO.Path]::GetDirectoryName($cursor)
    }
    return $full
}
function New-Text([string] $Path, [string] $Text) {
    $bytes = [Text.UTF8Encoding]::new($false).GetBytes($Text)
    $stream = [IO.File]::Open($Path, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::Read)
    try { $stream.Write($bytes, 0, $bytes.Length) } finally { $stream.Dispose() }
}
function New-Json([string] $Path, $Value) { New-Text $Path ($Value | ConvertTo-Json -Depth 16) }

if ($Name -notmatch '^[A-Za-z0-9][A-Za-z0-9_-]{0,49}$' -or
    $Name -match '^(?i:CON|PRN|AUX|NUL|COM[0-9]|LPT[0-9])$') {
    throw 'Name must be a simple non-reserved profile name of at most 50 characters.'
}
$modelPath = Local-Path $RobotConfig
if (-not [IO.File]::Exists($modelPath) -or (Get-Item -LiteralPath $modelPath).Length -gt 4MB) {
    throw 'Select a regular RobotConfig JSON file of at most 4 MiB.'
}
$sourceBytes = [IO.File]::ReadAllBytes($modelPath)
$sha = [Security.Cryptography.SHA256]::Create()
try { $sourceHash = ([BitConverter]::ToString($sha.ComputeHash($sourceBytes))).Replace('-', '').ToLowerInvariant() } finally { $sha.Dispose() }
$source = [Text.UTF8Encoding]::new($false, $true).GetString($sourceBytes) | ConvertFrom-Json
foreach ($field in @('ModelName','cableMotorNum','railMotorNum','frameAttachments','endEffectorAttachments','pulleyZdir','pulleyRadius','endEffectorPosLimit')) {
    if ($source.PSObject.Properties.Name -notcontains $field) { throw "Missing RobotConfig field: $field" }
}
if ($source.cableMotorNum -ne 8 -or $source.railMotorNum -ne 4 -or
    @($source.frameAttachments).Count -ne 8 -or @($source.endEffectorAttachments).Count -ne 8 -or @($source.pulleyZdir).Count -ne 8) {
    throw 'The existing offline planner supports eight cables and four fixed rails.'
}
# Values below are configuration observations, not a replacement model decoder.
# Actual model/manifest validation remains in the existing strict C++ loader.
$bundle = Local-Path (Join-Path $projectDirectory ('config\machines\' + $Name))
if (Test-Path -LiteralPath $bundle) { throw "Profile directory already exists; nothing was overwritten: $bundle" }
$planner = [IO.File]::ReadAllText((Join-Path $projectDirectory 'config\planner.example.json')) | ConvertFrom-Json
$planner.robot_model.configuration_path = $modelPath
$planner.robot_model.expected_robot_config_sha256 = $sourceHash
$planner.robot_model.calibration_manifest_path = 'calibration.local.json'
$planner.robot_model.motion_limits_manifest_path = 'limits.local.json'
$planner.output_directory = '../../../out'
$calibration = [ordered]@{
    schema_version = 1
    frames = $planner.frames
    provenance = [ordered]@{ calibration_id = ''; method = '' }
}
$limits = [ordered]@{ schema_version = 1; geometry_contract = 'cu_brick_fixed_rails_v1'; limits = $planner.planning.limits }
$requirements = @(
    [ordered]@{ group = 'Rail state'; fields = @('robot_model.rail_offsets_m','robot_model.starting_rail_offsets_m'); evidence = 'Two explicit four-element arrays in metres; confirm rail order and datum for this setup.' },
    [ordered]@{ group = 'Print registration'; fields = @('frames.world_from_bed','frames.end_effector_from_nozzle','frames.status','provenance.calibration_id','provenance.method'); evidence = 'Measured translations in metres and unit XYZW quaternions, with calibration identity and method.' },
    [ordered]@{ group = 'Initial nozzle and filament'; fields = @('import.initial_state.nozzle_pose_in_bed','import.initial_state.logical_e_mm','import.initial_state.filament_position_mm'); evidence = 'Explicit nozzle pose in the bed frame and separate logical/commanded filament datums. Configured home is not a measurement of the starting pose.' },
    [ordered]@{ group = 'Motion limits'; fields = @('limits.maximum_path_acceleration_m_per_s2','limits.minimum_cable_length_m','limits.maximum_cable_length_m','limits.maximum_cable_speed_m_per_s','limits.maximum_cable_acceleration_m_per_s2','limits.minimum_motor_count','limits.maximum_motor_count'); evidence = 'Reviewed machine limits. Cable arrays have eight entries; motor-count arrays contain signed integers. Controller XYZ/speed bounds may only be tightened.' },
    [ordered]@{ group = 'Print head'; fields = @('limits.maximum_filament_rate_mm_per_s','limits.maximum_volumetric_flow_mm3_per_s','planning.filament_diameter_mm','planning.temperature_wait_policy'); evidence = 'Reviewed filament/flow parameters, filament diameter, and hotend/bed wait tolerances and timeouts.' },
    [ordered]@{ group = 'Offline sampling'; fields = @('planning.sample_period_s','planning.path_tolerance_m'); evidence = 'Explicit sampling cadence in seconds and spatial validation interval in metres; these do not establish physical feasibility.' }
)
$report = [ordered]@{
    schema_version = 1
    status = 'incomplete_candidate'
    hardware_access = $false
    ready_for_planning = $false
    created_utc = [DateTime]::UtcNow.ToString('o')
    selected_configuration = [ordered]@{ path = $modelPath; sha256 = $sourceHash; model_name = $source.ModelName }
    configured_geometry = [ordered]@{ cable_count = $source.cableMotorNum; rail_count = $source.railMotorNum; pulley_radius_m = $source.pulleyRadius;
        frame_attachments_m = $source.frameAttachments; end_effector_attachments_m = $source.endEffectorAttachments; pulley_z_direction = $source.pulleyZdir }
    configured_controller_bounds_raw = $source.endEffectorPosLimit
    bounds_interpretation = 'Existing controller pairs x/y as X min/max, z/roll as Y min/max, and pitch/yaw as Z min/max. These are configuration observations, not calibrated print workspace.'
    required_inputs = $requirements
    files = @('planner.local.json','calibration.local.json','limits.local.json','setup.local.json','MEASUREMENTS.md')
}
$checklist = @"
# Machine profile candidate

Status: **incomplete; real-geometry planning is blocked by missing inputs**.
Hardware access: false.

Selected configuration: $modelPath

Model name: $($source.ModelName)

Exact source SHA-256: $sourceHash

This package references the existing configuration in place. The configuration
was read only. Its device-enable flags and network addresses are never applied.
No Robot is constructed, no recovery state is restored, and no driver is loaded.
This source selection is a candidate, not confirmation of the active machine.

## Fill the measured inputs

1. Confirm the active configuration, anchor/end-effector attachment geometry,
   four-rail order/datum and the configured legacy XYZ bound pairing.
2. In planner.local.json, supply the current and starting rail offsets, initial
   nozzle pose in the bed frame, logical E and commanded filament datums.
3. In calibration.local.json, enter world_from_bed and end_effector_from_nozzle.
   Each transform maps child coordinates into its parent. Use translation_m and
   rotation_xyzw (x, y, z, w). Record calibration_id and method; set status to
   verified only when the supplied registration has been verified.
4. In limits.local.json, enter the reviewed acceleration, eight-cable length/
   speed/acceleration/count limits, and filament/flow limits. Null workspace and
   path speed inherit the selected controller bounds; explicit bounds may only
   tighten them. A home pose or controller speed setting cannot fill the other
   missing limits.
5. In planner.local.json, enter the sampling cadence, spatial interval, filament
   diameter and temperature wait policies. Keep fixed rails and orientation.
   The reviewed Prusa profile establishes G21/G90/M82 before motion; modal
   defaults can remain unspecified when using that producer.

Do not copy synthetic example transforms or operating limits into this package.
The placeholders deliberately remain null or unverified; the existing strict
planner rejects the incomplete package. Existing model hashes detect a changed
source configuration, but do not certify calibration or physical readiness.

## Offline validation after completion

Use the existing Invoke-CdprSlice.ps1 entry point with this planner.local.json
as PlannerConfig, first on the included cube. Inspect transformed placement,
geometry, timings and the recorded checks. The current planner still leaves
hardware_ready false; no part of this package authorizes physical execution.

The complete machine-profile guide is in docs/machine-profile.md at the slicer
project root. setup.local.json records all missing input groups for review.
"@
[IO.Directory]::CreateDirectory($bundle) | Out-Null
New-Json (Join-Path $bundle 'planner.local.json') $planner
New-Json (Join-Path $bundle 'calibration.local.json') $calibration
New-Json (Join-Path $bundle 'limits.local.json') $limits
New-Json (Join-Path $bundle 'setup.local.json') $report
New-Text (Join-Path $bundle 'MEASUREMENTS.md') $checklist
Write-Host "Prepared incomplete $($source.ModelName) candidate: $bundle"
Write-Host 'Missing rail state, print registration, initial pose/datums and operating parameters remain explicit.'
[pscustomobject]@{ Directory = $bundle; PlannerConfig = Join-Path $bundle 'planner.local.json'; Checklist = Join-Path $bundle 'MEASUREMENTS.md'; Report = Join-Path $bundle 'setup.local.json' }
