# AB1: upper EE attachments inset 100 mm

Historical profile: the confirmed aluminum-face geometry in
[ab1-rail-face-mounts](../ab1-rail-face-mounts/README.md) supersedes this layout.
The points below lie in the open top rather than on the aluminum members.

This profile applies the user's requested horizontal inset to the four upper
EE attachment points. The inset is **100 mm from each X and Y outer edge**,
measured in the horizontal plane. The 400 x 400 x 580 mm body does not shrink.

| Cables / config indices | EE-local X/Y | EE-local Z | Routing |
| --- | --- | --- | --- |
| C1-C4 / 0-3 | +/-0.10 m | +0.29 m | Lower frame anchors to upper EE |
| C5-C8 / 4-7 | +/-0.20 m | -0.29 m | Upper frame anchors to lower EE |

The order is (-X,-Y), (+X,-Y), (+X,+Y), (-X,+Y) in each group. CoG/origin
remain at [0,0,0], nozzle tip at [0,0,-0.39] m, and bed origin at
[0.925,0.925,-1.70] m. Current/starting rails remain zero. The initial EE
world pose is [0.925,0.925,-1.31] m. The existing 500 mm cube path is retained.

## Shared configuration

- `robot.json` is the single geometry source for the offline C++ slicer and
  Python analysis. It contains the native loader's geometry/scale/envelope
  fields. Its SHA-256 is
  `0428b3f4d9307d75bb8efbd690a7a59f1de68c2f7e0944f39248725d12415589`.
- `planner.json` pins that geometry digest and references local calibration
  and limits files. `physical.json` is pinned to the same digest.
- `calibration.json` retains the explicit offline bed/nozzle registration.
  Its software `verified` status does not establish measured live registration.
- `limits.json` retains unknown operating limits as null.
- In the sibling `pyCDPR` repository, `config/ab1-top-inset-100mm.json` references
  this planner and physical model. `AB1.py` reads those files and checks the
  geometry digest. It does not keep another copy of the attachment coordinates.

This is an offline planning/analysis profile. Active root/bin robot-control
configurations are separate and were not replaced by this update.

## C++ slicer

From any PowerShell directory, using the current checkout path:

```powershell
$project = 'C:\Users\leoch\Documents\Dev\CU_brick\CDPR_3D_Printing_Slicer'
$slicer = Join-Path $project 'build\offline\bin\Release\cdpr-slicer-plan.exe'
$planner = Join-Path $project 'config\ab1-top-inset-100mm\planner.json'
$physics = Join-Path $project 'config\ab1-top-inset-100mm\physical.json'
$job = Join-Path $project 'out\ab1-cube-500mm-top-inset-100mm-20261029.json'
$runName = 'ab1-inset-' + [Guid]::NewGuid().ToString('N')
& $slicer --assess-job $job --config $planner --physics $physics `
    --output "${runName}.json" --preview-output "${runName}.html"
if ($LASTEXITCODE -ne 0) { throw 'Assessment failed; read the diagnostic.' }
Invoke-Item -LiteralPath (Join-Path $project "out\${runName}.html")
```

The saved preview is `out/ab1-cube-500mm-top-inset-100mm-20261029.html`
relative to the slicer project. Select layer 1 and XZ/front to inspect the inset.
All previous previews remain snapshots of their earlier geometry.

## pyCDPR

```powershell
$pythonProject = 'C:\Users\leoch\Documents\Dev\pyCDPR'
& (Join-Path $pythonProject '.venv\Scripts\python.exe') -B `
    (Join-Path $pythonProject 'AB1.py') `
    --config (Join-Path $pythonProject 'config\ab1-top-inset-100mm.json')
```

This reports the shared geometry digest, attachment coordinates, registered
initial pose and a static allocation. `--pose X Y Z YAW PITCH ROLL` accepts
world metres and Python's ZYX Euler radians. The Python `SpatialKinematic`
backend uses straight cables; C++ also includes pulley tangents/wrap. Matching
configuration does not imply identical cable lengths or tension allocations.

The approximate moving mass remains 70 kg, with user-specified centered CoG.
Minimum pretension and maximum allowed working tension are unknown. Both
analyses therefore use mathematical nonnegativity and the supplied 1,700 N
breaking-strength ceiling, while retaining an unknown operating-limits status.

## Assessment scope

The C++ run retains 3,753 motions / 625 layers and assesses 17,506 poses.
Static gravity balance is found at all samples. Sampled wrench ranks include
5 and 6; the initial center pose remains rank 5. Potential contacts remain
flagged at all samples, including cable-pair and cable/body contacts.

The 12 original 4040 member envelopes remain. The new upper attachment mounts
or crossmembers have not been supplied, so no additional members are invented.
The model still omits full bed/head/chassis geometry, cable sag, growing print
and actuator dynamics. Working limits remain unevaluated and hardware readiness
remains false. See [the mechanics guide](../../docs/mechanics.md).
