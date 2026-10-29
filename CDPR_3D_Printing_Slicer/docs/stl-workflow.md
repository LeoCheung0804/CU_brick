# STL to CDPR preview

The Windows entry point now runs the complete offline workflow:

```text
STL (millimetres) -> PrusaSlicer 2.9.6 -> reviewed text G-code
                 -> existing CDPR importer/geometry/timing -> JSON + 3D HTML
```

PrusaSlicer runs as a separate process. Its source, dependencies and compiler
requirements do not enter the C++14 planner build. The adapter adds no controller
or print-head interface. All new code and generated files stay in this project.

## Run the included cube

PrusaSlicer 2.9.6 portable and the Debug planner are already available in this
checkout. From Windows PowerShell:

```powershell
$project = 'C:\Users\leoch\Documents\Dev\CU_brick\CDPR_3D_Printing_Slicer'
Set-Location -LiteralPath $project

& .\tools\Invoke-CdprSlice.ps1 `
    -Model .\tests\fixtures\cube-10mm.stl `
    -PlannerConfig .\config\synthetic-planner.json `
    -OpenPreview
```

For another STL, replace `-Model` with its path. Relative paths resolve against
the PowerShell location. STL coordinates are interpreted as millimetres.
The reviewed profile has a 100 x 100 mm bed; the model is centered at (50, 50) mm
and placed on the bed without scaling. Layer height is 0.25 mm, with two walls,
15% rectilinear infill and three top/bottom solid layers. Supports are disabled.

For an explicit alternative export recipe, pass `-SlicingProfile <ini>` and
`-CenterXYMm <x>,<y>`. The placement coordinates must be two finite values in mm;
the profile still passes the existing producer execution guards. The receipt
records both the chosen profile and center. The [500 mm cube example](large-cube-example.md)
uses a separate 600 mm demo bed, centered at (0,0), and a coarse hollow recipe.
The default profile is unchanged. A zero-exit engine run that creates no G-code
is an error; its stdout/stderr and retained working path are reported.

`synthetic-planner.json`, `synthetic-calibration.json`, `synthetic-limits.json`
and the robot-model fixture describe mathematical demo data. Explicitly choose
a separately verified planner configuration for real machine geometry. Slicing
temperatures, feedrates and retractions in `prusa-cdpr.ini` are demo parameters,
not calibrated material/hotend settings or robot operating limits.

The command returns paths and creates four new files in `out/`:

- `<run>.gcode`: unfiltered text output from PrusaSlicer.
- `<run>.json`: the existing timed planning artifact.
- `<run>.html`: the standalone 3D preview.
- `<run>.receipt.json`: exact SHA-256 digests of the STL snapshot, slicing
  profile, engine executable, planner/configuration and each output; it also
  records placement, elapsed times and the retained working directory.

The run name defaults to a new GUID. An optional `-RunName cube-demo-001` must
be unused; existing outputs are never replaced. The command retains working
STL/profile snapshots and separate engine/planner logs in `build/slice/<GUID>/`.
JSON/HTML publication, G-code copying and receipt publication are separate:
an error retains completed files, names the working directory where applicable,
and never rolls back or overwrites a previous output. An incomplete set without
a receipt must be inspected before reuse; retry with a new name.

## Inspect the print

1. Choose **Inspect layer -> 40 / 40** to see the complete cube, or any earlier
   height to inspect the print up to that layer. Selection pauses playback and
   moves the displayed robot to that layer's final motion.
2. Choose **Nozzle / print bed**, then **Fit path** to examine the deposited
   centerlines. **Fit robot** shows the entire cable frame.
3. Clear **Show travel moves** to isolate deposited material and the retained
   retraction/recovery/prime markers. Travel moves use dashed gray lines.
4. Use **Reset**, then **Play** for timed playback. Playback restores the full
   path view. Temperature waits require explicit **Continue**, as before.

Layer heights come from horizontal depositing geometry, not G-code comments.
They are display groups, not a new layer contract in the job schema. Nonplanar
paths remain inspectable by original operation. The rendered material consists
of toolpath centerlines; bead width, deposited volume, deformation and physical
extrusion feedback are not simulated.

## Engine and profile

Only PrusaSlicer **2.9.6** is accepted by this adapter. Its official portable
Windows archive is available on the
[release page](https://github.com/prusa3d/PrusaSlicer/releases/tag/version_2.9.6).
The archive downloaded for this checkout is `PrusaSlicer-2.9.6.zip`, 106,598,059
bytes, SHA-256:

```text
5aaf22e42f95accecfa122d23a835911f289ecc2ff606db3e83d637ddcc0a209
```

The checksum was compared with the official GitHub release asset digest.
The portable files remain under ignored `build/tools/PrusaSlicer-2.9.6/`;
no global installation was performed.
PrusaSlicer is an external AGPL-3.0 application; see its
[source and license](https://github.com/prusa3d/PrusaSlicer).

For a fresh checkout, extract the official archive into
`build/tools/PrusaSlicer-2.9.6/`, retaining its original inner directory. The
default executable is:

```text
build/tools/PrusaSlicer-2.9.6/PrusaSlicer-2.9.6/prusa-slicer-console.exe
```

Alternatively pass `-PrusaSlicer 'C:\path\prusa-slicer-console.exe'` and, for a
different existing planner build, `-PlannerExecutable 'C:\path\cdpr-slicer-plan.exe'`.
The script never downloads or installs tools itself.

Each invocation supplies an isolated `--datadir`, child-only AppData/temp paths,
an explicit snapshotted profile and `--config-compatibility disable`. It uses
Windows argument quoting without a command shell. The adapter checks critical
profile settings before starting the engine, including disabled post-processing,
substitutions, binary G-code, arc fitting, volumetric extrusion and firmware retraction.
Model inputs are restricted to STL, avoiding inherited 3MF project settings.

The profile emits linear, single-tool, absolute filament-length G-code and
explicit unit/position/E modes. It defines startup/ending temperature and barrier
events and omits homing, leveling, motor shutdown and firmware acceleration
commands. The existing strict importer still validates every command; an
unsupported command aborts the run. No cleanup filter silently drops commands.

## Validation

After building the standalone planner, run the external-engine workflow checks:

```powershell
& .\tests\SliceWorkflow.Tests.ps1
```

These require the same portable engine and built planner. They cover the real
cube pipeline, dimensional/layer/E/frame checks, provenance, Windows paths,
output collisions, producer execution guards and rejection propagation.
The optional `tests/PreviewWorkflow.Tests.cjs <cube.html>` uses an existing
Playwright installation and Edge for desktop/mobile UI checks; no npm package
or browser is installed by the workflow. Evidence and measured limits are in
[validation.md](validation.md).

Large models still inherit the planner's sample budget, the 20,000-motion HTML
limit and the 64 MiB artifact/preview limits. No input is simplified silently.
Planning remains rest-to-rest at each original segment; time estimates are for
that conservative model. Continuous motion, actual machine calibration and
physical feasibility remain the next integration work.

For the user-selected AB1 configuration, [machine-profile.md](machine-profile.md)
describes the new measurement templates. That incomplete package must be filled
and reviewed before it can replace the runnable synthetic profile above.
