# 500 mm cube offline example

`tests/fixtures/cube-500mm.stl` is a closed, outward-oriented cube with vertices
from (0,0,0) to (500,500,500). Interpret STL coordinates as millimetres. The file
has 12 triangles, 8 unique vertices, 18 mesh edges each shared by two faces,
and signed enclosed volume 125,000,000 mm³. Its three dimensions are all 50 cm.

The default 100 x 100 x 100 mm export volume cannot contain it. A real test with
that profile returned `All objects are outside of the print volume.` from
PrusaSlicer, with exit status zero and no G-code. The wrapper now detects this
case and reports the engine's stdout/stderr instead of a missing-file error.

## Run the large demo

The separate project-local demo profiles created for this checkout are:

- `config/cube-500mm-demo.local.ini`: 600 x 600 mm bed spanning -300..300 mm,
  600 mm maximum height, 1.2 mm nozzle and extrusion width, 0.8 mm layers,
  one perimeter, zero infill, and zero top/bottom solid layers.
- `config/cube-500mm-demo.local.json`: the existing synthetic robot/registration
  and synthetic limits, with 10 s temporal sampling and 50 mm spatial intervals
  for this coarse visualization. These settings are not physical certification.
- `config/cube-500mm-bottom-centre.local.json` and its dedicated calibration
  manifest: the same recipe/model/limits, translated to start at the workspace
  bottom centre. This is the current placement used in the command below.

The STL describes a complete cube. This specific print recipe draws hollow
walls without top/bottom caps, keeping the demonstration within the existing
planning/preview budgets. It is not a full-density print or calibrated print-head
profile. No geometry is scaled down and no G-code commands are silently removed.

From the slicer project directory:

```powershell
& .\tools\Invoke-CdprSlice.ps1 `
    -Model .\tests\fixtures\cube-500mm.stl `
    -PlannerConfig .\config\cube-500mm-bottom-centre.local.json `
    -SlicingProfile .\config\cube-500mm-demo.local.ini `
    -CenterXYMm 0,0 `
    -OpenPreview
```

The existing wrapper defaults remain unchanged: without those optional parameters,
it uses `prusa-cdpr.ini` and center (50,50) mm. Every custom profile still passes
the same producer execution guards and the strict native importer. The receipt
records the selected profile digest and actual placement coordinates.

## Bottom-centre placement

The updated preview is
`out/cube-500mm-bottom-centre-2f8ab389f5404a51a8bbd0f38a168691.html`, with matching
G-code, timed JSON and receipt files. The original example is retained.

The synthetic workspace limits refer to the **end-effector origin** and range
from -0.5 to +0.5 m on each axis. Its X/Y midpoint is (0,0). The updated initial
EE pose is therefore **(0,0,-0.5) m**. The existing EE-to-nozzle translation is
(0,0,+0.05) m; preserving it puts the world print-bed origin at **(0,0,-0.45) m**.
Only this demo's world-from-bed translation changed, from (0.1,0.2,0) to
(0,0,-0.45). Global synthetic calibration, machine geometry and limits are unchanged.

The first deposited layer is 0.8 mm above that initial level: EE world
Z=-0.4992 m and nozzle world Z=-0.4492 m. The final layer is EE Z approximately
0 and nozzle Z=+0.05 m. Every EE endpoint stays in the configured XYZ bounds,
and the full deposited path is centered in X/Y. Nozzle coordinates in the bed
frame, cube dimensions, all extrusion spans and 625 heights are unchanged.
The frame selector still distinguishes nozzle/bed coordinates from EE/world
coordinates; workspace constraints apply to the latter.

Six desktop/mobile browser groups verified initial/first/final poses, reset,
layout and clean console/network behavior. Full path comparison and receipt
checks are retained in the validation record. This remains a synthetic offline
example, not an AB1 registration or physical feasibility result.

## Completed run — 2026-10-28

Artifact prefix:
`out/cube-500mm-demo-3caa4dfca9a742fe88dd1c18b0f61c93`.

The real engine/planner run produced `.gcode`, `.json`, `.html` and `.receipt.json`:

- 625 deposition heights, from 0.8 through 500 mm.
- 3,753 motions and 3,764 total operations.
- 56,275 timing knots; 8 implemented checks passed and 9 remain unevaluated.
- Depositing centerlines span -249.4..249.4 mm in X and Y; their 0.6 mm edge
  inset matches the specified 1.2 mm extrusion width.
- Slicing: 0.446 s; Debug planning plus preview: 48.478 s on this workstation.
- HTML preview about 22.1 MB; timed JSON about 24.5 MB.

The conservative rest-to-rest model totals about 449,927 seconds (5.21 days),
excluding unknown heater wait durations. This is an offline model result, not a
validated physical print-time estimate. Use layer/operation selection to inspect
the whole object rather than waiting for real-time playback to finish.

In the preview, choose **625 / 625**, **Nozzle / print bed**, then **Fit path**.
Clear **Show travel moves** to inspect material paths. **Fit robot** restores the
whole synthetic frame. The midpoint deposition height is 250.4 mm (layer 313).

The generated STL dimensions, closed mesh/volume, profile/output/source digests,
layer count and transformed plan were checked. All 28 workflow checks passed
under Windows PowerShell 5.1. Five Edge desktop/mobile acceptance groups passed,
with zero page errors/warnings or external page requests. Evidence is recorded
in [validation.md](validation.md).

This run uses synthetic geometry/registration and performs no hardware access.
The measured AB1 profile still requires its real rail state, print registration,
initial pose/datums and reviewed print limits before real-geometry planning.
