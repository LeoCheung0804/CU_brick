# AB1: attachment holes on the aluminum rail faces

This is the preceding smaller-frame offline profile. The current
[5004 x 5004 x 2305 mm anchor profile](../ab1-frame-5004x5004x2305/README.md)
supersedes its outer-frame geometry and placement; its EE rail-face holes remain.
The user confirmed these hole centers from the annotated sketch. Coordinates
are in millimetres relative to the centered EE origin; each cable pulls from
the specified point on an outer Y face of a 40 x 40 mm aluminum member.

| Cable / config index | X (mm) | Y (mm) | Z (mm) | Member |
| --- | --- | --- | --- | --- |
| C1 / 0 | -100 | -200 | +270 | Top front rail |
| C2 / 1 | +100 | -200 | +270 | Top front rail |
| C3 / 2 | +100 | +200 | +270 | Top back rail |
| C4 / 3 | -100 | +200 | +270 | Top back rail |
| C5 / 4 | -180 | -200 | -270 | Lower corner face |
| C6 / 5 | +180 | -200 | -270 | Lower corner face |
| C7 / 6 | +180 | +200 | -270 | Lower corner face |
| C8 / 7 | -180 | +200 | -270 | Lower corner face |

The upper holes are 100 mm along X from each end of their rail. Their Z is
the center of the top 40 mm section, 20 mm below the outer top. The lower
holes are at the lower corner-face centers. All eight points lie on the
existing member envelopes; none floats in the open top of the frame.

The outer EE remains 400 x 400 x 580 mm, with CoG/origin [0,0,0] and
approximately 70 kg moving mass. The nozzle stays [0,0,-390] mm, **100 mm
below the outer frame bottom**, independently of the new hole heights.
Bed origin remains [0.925,0.925,-1.70] m; initial EE world position remains
[0.925,0.925,-1.31] m. Current/starting rails are zero. Routing remains lower
frame to upper EE (C1-C4), upper frame to lower EE (C5-C8).

## Shared files

- `robot.json` supplies geometry, with SHA-256
  `b9202872947cba9e7920be78fd6d23807786696705a06dd7175aa213a7580dbb`.
- `planner.json` and `physical.json` pin that digest. The physical model
  retains twelve 4040 member envelopes and the 2 mm cable diameter.
- `calibration.json` retains the explicit offline bed/nozzle registration.
  Its software `verified` status does not establish measured live registration.
- `limits.json` retains unknown motion operating limits as null. Minimum
  pretension and maximum working tension also remain null in `physical.json`.
- `pyCDPR/config/ab1-rail-face-mounts.json` references these same files;
  `pyCDPR/AB1.py --config <reference>` reads them and verifies the digest.
  Its current default uses the corrected outer-frame profile linked above.

Root/bin controller configurations remain separate. These files are an
offline planning/static-analysis profile, not a full execution configuration.
Previous configurations and generated outputs remain historical snapshots.

## C++ slicer

The current preview is `out/ab1-cube-500mm-10mm-layers-mechanics-20261029.html`.
Both first and subsequent layers are **10 mm**, giving 50 deposition heights
from 10 to 500 mm. `config/cube-500mm-10mm-layers.ini` uses the user-approved
12 mm demo nozzle and bead width. The shell remains one perimeter with no
infill or caps. Its centerlines span +/-244 mm in X/Y, allowing for half of
the 12 mm nominal bead width within the 500 mm cube. Material, temperature,
feed and filament settings remain demonstration values.

Select XZ/front and zoom to inspect the hole heights, or orbit the 3D view.
To reslice the STL and generate another assessment without replacing outputs:

```powershell
$project = 'C:\Users\leoch\Documents\Dev\CU_brick\CDPR_3D_Printing_Slicer'
$ErrorActionPreference = 'Stop'
$slicer = Join-Path $project 'build\offline\bin\Release\cdpr-slicer-plan.exe'
$planner = Join-Path $project 'config\ab1-rail-face-mounts\planner.json'
$physics = Join-Path $project 'config\ab1-rail-face-mounts\physical.json'
$runName = 'ab1-10mm-' + [Guid]::NewGuid().ToString('N')
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

The new 50-layer job has 303 motions. The preceding 0.8 mm, 625-layer preview
`out/ab1-cube-500mm-rail-face-mounts-20261029.html` remains a snapshot.
Replanning another G-code file requires this planner profile as well;
an old geometry-bound job cannot be assessed with a different robot digest.

## pyCDPR

```powershell
$pythonProject = 'C:\Users\leoch\Documents\Dev\pyCDPR'
& (Join-Path $pythonProject '.venv\Scripts\python.exe') -B `
    (Join-Path $pythonProject 'AB1.py') `
    --config (Join-Path $pythonProject 'config\ab1-rail-face-mounts.json')
```

The output identifies the geometry digest, exact attachment coordinates,
initial pose, rank and a static allocation. `--pose X Y Z YAW PITCH ROLL`
uses world metres and ZYX Euler radians. Keep the repositories as sibling
folders or adjust the two paths in the Python reference configuration.

Python uses its existing straight-cable backend. C++ includes pulley
tangents/wrap. Shared inputs do not imply identical cable lengths or forces.

## Assessment results and limits

All 1,406 sampled poses have wrench rank 6 and a gravity-balancing allocation
under the mathematical bounds. The largest returned C++ allocation is about
1,368.20 N, or 80.5% of the supplied 1,700 N **breaking strength**. It is one
feasible allocation, not a required capacity, measured force or approved load.
Zero-tension cables occur. Pretension and allowed working limits remain unknown.

Each pose has 16 counted capsule/member contacts: one member at each upper
mount and three adjacent envelopes at each lower mount. No additional contact
is counted at these sampled poses. The report retains four smallest-clearance
contacts per pose, all at upper mounts. Its status remains `potential_collision`
because attachment exclusion is zero and the cable endcaps touch their rails;
the minimum signed clearance is -1 mm. No mounting clearance is invented or
warning suppressed. Compared with the previous profile, the four crossing
cable pairs are no longer reported in this sampled run.

Continuous clearance, fasteners/grooves, full head/bed/chassis, sag, growing
print and dynamics remain outside this model. Hardware readiness remains
false. See [the mechanics guide](../../docs/mechanics.md).
