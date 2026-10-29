# AB1: 5004 x 5004 x 2305 mm outer anchor spacing

This is the confirmed rail-face reference configuration for the C++ slicer
and pyCDPR. The later [5 mm standoff proposal](../ab1-mount-standoff-5mm/README.md)
is separate; this reference preserves the holes on the aluminum faces.
The user corrected the outer attachment-point separations to
**5004 mm in X, 5004 mm in Y and 2305 mm in Z**. These are anchor-coordinate
differences; detailed outer-frame members and pulley hardware are not CAD models.

The existing coordinate convention is retained: the upper anchor plane is
world Z=0, the lower plane is Z=-2.305 m, and the first XY corner is [0,0].

| Lower-frame / upper-frame cable | X (m) | Y (m) | Lower Z (m) | Upper Z (m) |
| --- | --- | --- | --- | --- |
| C1 / C5 | 0 | 0 | -2.305 | 0 |
| C2 / C6 | 5.004 | 0 | -2.305 | 0 |
| C3 / C7 | 5.004 | 5.004 | -2.305 | 0 |
| C4 / C8 | 0 | 5.004 | -2.305 | 0 |

The EE remains 400 x 400 x 580 mm. Its confirmed rail-face holes remain
upper [X=+/-100,Y=+/-200,Z=+270] mm and lower
[X=+/-180,Y=+/-200,Z=-270] mm. CoG/origin [0,0,0], approximately 70 kg mass,
nozzle [0,0,-390] mm, 2 mm cable diameter, 45 mm pulley radius and zero rails
are retained. Routing remains lower frame to upper EE, upper frame to lower EE.

## Bed and offline bounds

The 500 mm cube is recentered at X=Y=2.502 m. The aligned bed origin is
[2.502,2.502,-2.205] m, keeping the user's 100 mm height above the lower
anchors. Initial EE position is [2.502,2.502,-1.815] m; first and final layer
EE heights are -1.805 m and -1.315 m. The nozzle remains 100 mm below the EE
body bottom. These are explicit offline placements; live registration is unverified.

The old geometry's legacy X bound ended at 2 m, excluding the new center.
This separate profile sets the loader-required legacy XYZ bounds and the
manifest envelope to the anchor box: [0,0,-2.305]..[5.004,5.004,0] m. It is
an enclosure for geometric screening; it does not establish a usable working
workspace. Motion, tension and hardware limits require their own validation.
Active root/bin controller files retain their previous bytes.

## Shared configuration and outputs

`robot.json` SHA-256 is
`d374a603cacf41b84009e5f82524e8e0fd369563d5cbd5724ccd68e7d516a83f`.
`planner.json` and `physical.json` pin that digest. `calibration.json` records
the bed/nozzle transforms, and `limits.json` contains the geometric enclosure
with other operating limits unknown. `pyCDPR/config/ab1-frame-5004x5004x2305.json`
references these same files; select it explicitly with `AB1.py --config`.
The Python default now selects the separate standoff proposal.
Python uses straight cables; C++ includes pulley tangents/wrap.

Reference preview:
`out/ab1-cube-500mm-frame-5004-2305-10mm-mechanics-20261029.html`.
The STL is still 500 mm square and tall, with 10 mm first/subsequent layers,
50 deposition heights and 303 motions. The user-approved demo nozzle/bead
width remains 12 mm. The hollow shell has one perimeter, no infill or caps.
Earlier smaller-frame configurations and outputs remain snapshots.

## Generate another run

```powershell
$ErrorActionPreference = 'Stop'
$project = 'C:\Users\leoch\Documents\Dev\CU_brick\CDPR_3D_Printing_Slicer'
$slicer = Join-Path $project 'build\offline\bin\Release\cdpr-slicer-plan.exe'
$planner = Join-Path $project 'config\ab1-frame-5004x5004x2305\planner.json'
$physics = Join-Path $project 'config\ab1-frame-5004x5004x2305\physical.json'
$runName = 'ab1-frame5004-' + [Guid]::NewGuid().ToString('N')
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

To run the matching offline Python calculation:

```powershell
$pythonProject = 'C:\Users\leoch\Documents\Dev\pyCDPR'
& (Join-Path $pythonProject '.venv\Scripts\python.exe') -B `
    (Join-Path $pythonProject 'AB1.py') `
    --config (Join-Path $pythonProject 'config\ab1-frame-5004x5004x2305.json')
```

## Assessment limits

All 1,406 sampled poses have rank 6 and a static gravity-balancing allocation
under mathematical 0..1700 N bounds. The largest returned allocation is about
979.12 N (57.6% of the supplied breaking strength). These are feasible
allocations, not measured forces, minimum required capacities or approved loads.
Pretension and allowed working tensions remain unknown; zero-tension cables occur.

All samples retain 16 cable/member mount contacts and `potential_collision`.
The four retained smallest-clearance contacts are at the lower mounts, about
-1 mm. No attachment exclusion or invented mounting clearance is applied.
Sampling does not certify continuous clearance. Full bed/head/frame hardware,
sag, growing print, dynamics and physical execution remain outside the model.
Hardware readiness remains false.
