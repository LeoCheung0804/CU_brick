#requires -Version 5.1
<#
.SYNOPSIS
Slice an STL with the reviewed PrusaSlicer profile, then export an offline CDPR preview.
.EXAMPLE
& .\tools\Invoke-CdprSlice.ps1 -Model .\tests\fixtures\cube-10mm.stl -PlannerConfig .\config\synthetic-planner.json
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string] $Model,
    [Parameter(Mandatory = $true)][string] $PlannerConfig,
    [string] $PrusaSlicer,
    [string] $PlannerExecutable,
    [string] $SlicingProfile,
    [double[]] $CenterXYMm = @(50, 50),
    [string] $RunName = ('slice-' + [Guid]::NewGuid().ToString('N')),
    [switch] $OpenPreview,
    [ValidateSet('geometrically_checked', 'timed')][string] $PlanningStage = 'timed'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$PlanningStage = $PlanningStage.ToLowerInvariant()
$projectDirectory = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))

function Get-LocalPath([string] $Path, [switch] $File) {
    $full = [IO.Path]::GetFullPath($ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Path))
    if ($full -notmatch '^[A-Za-z]:\\' -or $full.Substring(2).Contains(':') -or $full.Length -ge 240) {
        throw 'Use a conventional local drive path shorter than 240 characters.'
    }
    $cursor = $full
    while ($cursor) {
        if ([IO.File]::Exists($cursor) -or [IO.Directory]::Exists($cursor)) {
            if (([IO.File]::GetAttributes($cursor) -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
                throw "Reparse points are not supported: $cursor"
            }
        }
        $cursor = [IO.Path]::GetDirectoryName($cursor)
    }
    if ($File -and -not [IO.File]::Exists($full)) { throw "Required file does not exist: $full" }
    return $full
}

function Write-NewText([string] $Path, [string] $Text) {
    $bytes = [Text.UTF8Encoding]::new($false).GetBytes($Text)
    $stream = [IO.File]::Open($Path, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::Read)
    try { $stream.Write($bytes, 0, $bytes.Length) } finally { $stream.Dispose() }
}

# ProcessStartInfo on Windows PowerShell takes one command line. Quote each
# argument using the Windows argv rules; never pass file paths through a shell.
function ConvertTo-NativeArgument([string] $Value) {
    $escaped = [regex]::Replace($Value, '(\\*)"', '$1$1\"')
    $escaped = [regex]::Replace($escaped, '(\\+)$', '$1$1')
    return '"' + $escaped + '"'
}

function Invoke-LocalTool([string] $Executable, [string[]] $Arguments, [string] $LogName) {
    $start = [Diagnostics.ProcessStartInfo]::new()
    $start.FileName = $Executable
    $start.Arguments = ($Arguments | ForEach-Object { ConvertTo-NativeArgument $_ }) -join ' '
    $start.WorkingDirectory = $runDirectory
    $start.UseShellExecute = $false
    $start.CreateNoWindow = $true
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    foreach ($name in @('APPDATA', 'LOCALAPPDATA', 'TEMP', 'TMP')) {
        $start.EnvironmentVariables[$name] = $privateDataDirectory
    }
    $process = [Diagnostics.Process]::new()
    $process.StartInfo = $start
    $watch = [Diagnostics.Stopwatch]::StartNew()
    try {
        if (-not $process.Start()) { throw "Cannot start $Executable" }
        $stdout = $process.StandardOutput.ReadToEndAsync()
        $stderr = $process.StandardError.ReadToEndAsync()
        if (-not $process.WaitForExit(300000)) {
            $process.Kill()
            $process.WaitForExit()
            throw "Offline tool exceeded five minutes; retained working files: $runDirectory"
        }
        $output = $stdout.GetAwaiter().GetResult()
        $errors = $stderr.GetAwaiter().GetResult()
        Write-NewText (Join-Path $runDirectory ($LogName + '.stdout.log')) $output
        Write-NewText (Join-Path $runDirectory ($LogName + '.stderr.log')) $errors
        if ($process.ExitCode -ne 0) {
            throw "$LogName failed (exit $($process.ExitCode)). $($errors.Trim()) See $runDirectory"
        }
        return [pscustomobject]@{ Output = $output; ErrorOutput = $errors; DurationSeconds = $watch.Elapsed.TotalSeconds }
    } finally { $process.Dispose() }
}

$projectDirectory = Get-LocalPath $projectDirectory
$modelPath = Get-LocalPath $Model -File
$plannerConfigPath = Get-LocalPath $PlannerConfig -File
if ($CenterXYMm.Count -ne 2 -or @($CenterXYMm | Where-Object { [double]::IsNaN($_) -or [double]::IsInfinity($_) }).Count -gt 0) {
    throw 'CenterXYMm requires two finite millimetre coordinates.'
}
$centerArgument = ($CenterXYMm | ForEach-Object { $_.ToString('G17', [Globalization.CultureInfo]::InvariantCulture) }) -join ','
if ([IO.Path]::GetExtension($modelPath) -ine '.stl') {
    throw 'This milestone accepts STL models only. 3MF project settings and scripts are not imported.'
}
if ((Get-Item -LiteralPath $modelPath).Length -gt 64MB) { throw 'STL input exceeds 64 MiB.' }
if ($RunName -notmatch '^[A-Za-z0-9][A-Za-z0-9_-]{0,89}$' -or
    $RunName -match '^(?i:CON|PRN|AUX|NUL|COM[0-9]|LPT[0-9])$') {
    throw 'RunName must be a simple, non-reserved name of at most 90 characters.'
}
if (-not $PrusaSlicer) {
    $PrusaSlicer = Join-Path $projectDirectory 'build\tools\PrusaSlicer-2.9.6\PrusaSlicer-2.9.6\prusa-slicer-console.exe'
}
if (-not $PlannerExecutable) {
    $PlannerExecutable = Join-Path $projectDirectory 'build\offline\bin\Debug\cdpr-slicer-plan.exe'
}
$slicerPath = Get-LocalPath $PrusaSlicer -File
$plannerPath = Get-LocalPath $PlannerExecutable -File
if ([IO.Path]::GetExtension($slicerPath) -ine '.exe' -or [IO.Path]::GetExtension($plannerPath) -ine '.exe') {
    throw 'Supply the console and planner .exe files directly.'
}
if (-not $SlicingProfile) { $SlicingProfile = Join-Path $projectDirectory 'config\prusa-cdpr.ini' }
$slicingProfilePath = Get-LocalPath $SlicingProfile -File
$slicingProfileText = [IO.File]::ReadAllText($slicingProfilePath)
# Prevent post-processing execution, substitutions and binary/arc output before
# starting the external engine. The strict importer checks every emitted command.
foreach ($setting in @('post_process\s*=\s*', 'gcode_substitutions\s*=\s*', 'binary_gcode\s*=\s*0',
    'arc_fitting\s*=\s*disabled', 'use_volumetric_e\s*=\s*0', 'use_firmware_retraction\s*=\s*0')) {
    $key = ($setting -split '\\s')[0]
    $entries = @($slicingProfileText -split '\r?\n' | Where-Object { $_ -match ('^\s*' + $key + '\s*=') })
    if ($entries.Count -ne 1 -or $entries[0] -notmatch ('^\s*' + $setting + '\s*$')) {
        throw "Incompatible or duplicated slicing profile setting: $key"
    }
}
$outputDirectory = Get-LocalPath (Join-Path $projectDirectory 'out')
$workRoot = Get-LocalPath (Join-Path $projectDirectory 'build\slice')
foreach ($suffix in @('.gcode', '.json', '.html', '.receipt.json')) {
    if (Test-Path -LiteralPath (Join-Path $outputDirectory ($RunName + $suffix))) {
        throw "An output already exists for $RunName. Choose a new RunName; nothing was overwritten."
    }
}
[IO.Directory]::CreateDirectory($workRoot) | Out-Null
$runDirectory = Get-LocalPath (Join-Path $workRoot ([Guid]::NewGuid().ToString('N')))
[IO.Directory]::CreateDirectory($runDirectory) | Out-Null
$privateDataDirectory = Join-Path $runDirectory 'private-data'
[IO.Directory]::CreateDirectory($privateDataDirectory) | Out-Null
$modelSnapshot = Join-Path $runDirectory 'model.stl'
$profileSnapshot = Join-Path $runDirectory 'slicer.ini'
$gcodeSnapshot = Join-Path $runDirectory ($RunName + '.gcode')
[IO.File]::Copy($modelPath, $modelSnapshot, $false)
Write-NewText $profileSnapshot $slicingProfileText
$help = Invoke-LocalTool $slicerPath @('--help') 'version'
if ($help.Output -notmatch '^PrusaSlicer-2\.9\.6\s') {
    throw 'This profile is validated with PrusaSlicer 2.9.6 only. Supply its prusa-slicer-console.exe.'
}
Write-Host 'Slicing STL with PrusaSlicer 2.9.6...'
$sliced = Invoke-LocalTool $slicerPath @('--datadir', $privateDataDirectory, '--config-compatibility', 'disable', '--load', $profileSnapshot,
    '--center', $centerArgument, '--export-gcode', '--output', $gcodeSnapshot, $modelSnapshot) 'slicer'
if (-not [IO.File]::Exists($gcodeSnapshot)) {
    throw "PrusaSlicer produced no G-code. $($sliced.Output.Trim()) $($sliced.ErrorOutput.Trim()) See $runDirectory"
}
$gcodeSnapshot = Get-LocalPath $gcodeSnapshot -File
if ((Get-Item -LiteralPath $gcodeSnapshot).Length -gt 64MB) { throw 'Generated G-code exceeds 64 MiB.' }
$plannerConfigHash = (Get-FileHash -LiteralPath $plannerConfigPath -Algorithm SHA256).Hash.ToLowerInvariant()
Write-Host "Planning the CDPR path ($PlanningStage)..."
$planned = Invoke-LocalTool $plannerPath @('--input', $gcodeSnapshot, '--config', $plannerConfigPath,
    '--stage', $PlanningStage, '--output', ($RunName + '.json'), '--preview-output', ($RunName + '.html')) 'planner'
Write-Host $planned.Output.Trim()
if ((Get-FileHash -LiteralPath $plannerConfigPath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $plannerConfigHash) {
    throw "Planner configuration changed during the run. Inspect retained outputs for $RunName."
}
# The planner publishes JSON/HTML first, using its existing guarded output path.
# Publication is intentionally separate; failure retains each completed artifact.
$outputDirectory = Get-LocalPath $outputDirectory
$gcodeOutput = Join-Path $outputDirectory ($RunName + '.gcode')
$jobOutput = Get-LocalPath (Join-Path $outputDirectory ($RunName + '.json')) -File
$previewOutput = Get-LocalPath (Join-Path $outputDirectory ($RunName + '.html')) -File
[IO.File]::Copy($gcodeSnapshot, $gcodeOutput, $false)
$receiptOutput = Join-Path $outputDirectory ($RunName + '.receipt.json')
$receipt = [ordered]@{
    schema_version = 1
    mode = 'offline_stl_to_cdpr'
    hardware_access = $false
    created_utc = [DateTime]::UtcNow.ToString('o')
    engine = [ordered]@{ name = 'PrusaSlicer'; version = '2.9.6'; executable = $slicerPath;
        sha256 = (Get-FileHash -LiteralPath $slicerPath -Algorithm SHA256).Hash.ToLowerInvariant() }
    model = [ordered]@{ path = $modelPath; sha256 = (Get-FileHash -LiteralPath $modelSnapshot -Algorithm SHA256).Hash.ToLowerInvariant(); units = 'mm' }
    slicing_profile = [ordered]@{ path = $slicingProfilePath; sha256 = (Get-FileHash -LiteralPath $profileSnapshot -Algorithm SHA256).Hash.ToLowerInvariant() }
    planner = [ordered]@{ executable = $plannerPath; sha256 = (Get-FileHash -LiteralPath $plannerPath -Algorithm SHA256).Hash.ToLowerInvariant(); stage = $PlanningStage;
        configuration = $plannerConfigPath; configuration_sha256 = $plannerConfigHash }
    placement = [ordered]@{ center_xy_mm = $CenterXYMm; ensure_on_bed = $true }
    outputs = [ordered]@{}
    duration_s = [ordered]@{ slicing = $sliced.DurationSeconds; planning_and_preview = $planned.DurationSeconds }
    working_directory = $runDirectory
}
foreach ($artifact in @($gcodeOutput, $jobOutput, $previewOutput)) {
    $receipt.outputs[[IO.Path]::GetExtension($artifact).Substring(1)] = [ordered]@{
        path = $artifact; sha256 = (Get-FileHash -LiteralPath $artifact -Algorithm SHA256).Hash.ToLowerInvariant()
    }
}
Write-NewText $receiptOutput ($receipt | ConvertTo-Json -Depth 8)
Write-Host "Preview: $previewOutput"
Write-Host "Receipt: $receiptOutput"
if ($OpenPreview) { Invoke-Item -LiteralPath $previewOutput }
[pscustomobject]@{ GCode = $gcodeOutput; Job = $jobOutput; Preview = $previewOutput; Receipt = $receiptOutput }
