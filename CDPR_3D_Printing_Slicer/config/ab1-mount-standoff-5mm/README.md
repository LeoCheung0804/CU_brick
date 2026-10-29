# Proposed AB1 cable fixings: 5 mm outward standoffs

This offline design candidate moves the free-cable fixing centers 5 mm outward
from the EE's aluminum Y faces. It addresses the capsule/member contacts in
the [confirmed rail-face reference](../ab1-frame-5004x5004x2305/README.md).
The original hole locations remain on the rails; a bracket or eyelet would
connect each hole to its proposed cable center. The bracket, fasteners, local
cable connection and their strength have not been modeled or verified.

The proposal is explicitly marked `synthetic: true` and named
`PROPOSED-AB1-5mm-outward-mounts-20261029` in the preview and Python output.
It is not a measurement or an approved hardware modification.

## Fixing centers relative to the EE center

| Cable | X (mm) | Y (mm) | Z (mm) |
| --- | ---: | ---: | ---: |
| C1 | -100 | -205 | +270 |
| C2 | +100 | -205 | +270 |
| C3 | +100 | +205 | +270 |
| C4 | -100 | +205 | +270 |
| C5 | -180 | -205 | -270 |
| C6 | +180 | -205 | -270 |
| C7 | +180 | +205 | -270 |
| C8 | -180 | +205 | -270 |

Only Y changes from the reference, from +/-200 to +/-205 mm. The 400 x 400 x
580 mm EE body, centered CoG, approximate 70 kg mass and nozzle at
[0,0,-390] mm remain. Outer anchor spacing remains 5004 x 5004 x 2305 mm.
Routing remains lower frame to upper EE, upper frame to lower EE, with zero
rail offsets. The aligned bed origin is [2.502,2.502,-2.205] m, 100 mm above
the lower anchors. Live registration remains unverified.

## Shared configuration

`robot.json` SHA-256 is
`8be5ed5105f1a92263f591b0d9323759da664444e26c346579d6dc1efcb20c0c`.
`planner.json` and `physical.json` pin that digest. `calibration.json` retains
the bed/nozzle transforms, and `limits.json` retains the offline geometric
enclosure with operating limits unknown.

`pyCDPR/config/ab1-mount-standoff-5mm.json` references these same files;
`pyCDPR/AB1.py` selects it by default. Python uses straight cables, while the
C++ assessment uses pulley-aware free spans. The confirmed reference and
earlier output files remain available. Active root/bin controller configurations
retain their previous bytes.

## Sampled result

Open `out/ab1-cube-500mm-standoff-5mm-10mm-mechanics-20261029.html`.
The 500 mm cube retains 10 mm layers, 50 deposition heights, a 12 mm demo
nozzle/bead width, 303 motions and the same bed-frame and world toolpaths.
It is a hollow single-perimeter example, with no infill or caps.

All 1,406 sampled poses have zero detected contacts and at least **4 mm**
clearance in the supplied model. The 5 mm fixing-center distance minus the
1 mm cable radius gives that minimum. All twelve aluminum member boxes,
cable radius, zero extra clearance and zero attachment exclusion are unchanged;
the same checks and planner binary found 16 mount contacts at every pose in
the reference layout. No collision checks were disabled.

All samples retain wrench rank 6 and a gravity-balancing allocation under
mathematical 0..1700 N bounds. The largest returned allocation is 1027.31 N,
about 60.4% of the supplied breaking strength. These allocations are not
measured forces, required capacities or approved loads. Pretension and allowed
working tensions remain unknown, and zero-tension cables occur.

Sampled clearance excludes the unmodeled standoff hardware. Continuous clearance,
tolerances, sag, full bed/head/frame hardware, the growing print and dynamics
remain unverified. Hardware readiness remains false.

## Generate another offline run

```powershell
$ErrorActionPreference = 'Stop'
$project = 'C:\Users\leoch\Documents\Dev\CU_brick\CDPR_3D_Printing_Slicer'
$slicer = Join-Path $project 'build\offline\bin\Release\cdpr-slicer-plan.exe'
$planner = Join-Path $project 'config\ab1-mount-standoff-5mm\planner.json'
$physics = Join-Path $project 'config\ab1-mount-standoff-5mm\physical.json'
$runName = 'ab1-standoff5mm-' + [Guid]::NewGuid().ToString('N')
$run = & (Join-Path $project 'tools\Invoke-CdprSlice.ps1') `
    -Model (Join-Path $project 'tests\fixtures\cube-500mm.stl') `
    -PlannerConfig $planner -PlannerExecutable $slicer `
    -SlicingProfile (Join-Path $project 'config\cube-500mm-10mm-layers.ini') `
    -PlanningStage geometrically_checked -CenterXYMm 0,0 -RunName $runName
& $slicer --assess-job $run.Job --config $planner --physics $physics `
    --output "${runName}-report.json" --preview-output "${runName}-mechanics.html"
if ($LASTEXITCODE -ne 0) { throw 'Assessment failed; read the diagnostic.' }
Invoke-Item -LiteralPath (Join-Path $project "out\${runName}-mechanics.html")
```

To run the matching Python calculation explicitly:

```powershell
$pythonProject = 'C:\Users\leoch\Documents\Dev\pyCDPR'
& (Join-Path $pythonProject '.venv\Scripts\python.exe') -B `
    (Join-Path $pythonProject 'AB1.py') `
    --config (Join-Path $pythonProject 'config\ab1-mount-standoff-5mm.json')
```

Sixteen Python tests and eight artifact/browser groups passed. Evidence and
scope are recorded in the [validation record](../../docs/validation.md).
