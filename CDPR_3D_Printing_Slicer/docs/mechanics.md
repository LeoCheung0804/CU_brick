# Sampled collision and static cable-load assessment

The standalone planner can attach a separate mechanics assessment to a
model-bound job and export an interactive 3D view. This adds sampled geometric
contact screening and gravity-load equilibrium calculations. It does not change
the job's stage, recorded validation checks or `hardware_ready: false`.

## Current AB1 proposal: cable centers 5 mm outside the rail faces

Open `out/ab1-cube-500mm-standoff-5mm-10mm-mechanics-20261029.html`.
The [shared proposal and commands](../config/ab1-mount-standoff-5mm/README.md)
move only the EE-local Y coordinate of every free-cable fixing center from
+/-0.200 to +/-0.205 m. Upper X/Z remain +/-0.100/+0.270 m and lower X/Z
remain +/-0.180/-0.270 m. The confirmed holes stay on the rails; a standoff
bracket or eyelet would connect each hole to the proposed cable center.
Its solid geometry, fasteners, local cable connection and strength are not
modeled or verified. The proposal is explicitly marked synthetic and uses
model ID `PROPOSED-AB1-5mm-outward-mounts-20261029`.

The original rail-face model counted the 1 mm cable-radius endcaps touching
their supporting aluminum as collisions. Moving a hole along the same face
would retain this contact. With the proposed 5 mm outward cable centers, the
same collision checker finds **zero contacts at all 1,406 sampled poses**, with
minimum clearance **4 mm**. All twelve EE member boxes, 1 mm cable radius,
zero extra clearance and zero attachment exclusion are unchanged.

Frame anchors, cable routing, body, CoG, nozzle, bed, zero rails and the complete
10 mm-layer toolpath are unchanged. All poses retain rank 6 and a static
gravity-balancing allocation under mathematical 0..1700 N bounds. The largest
returned allocation is 1027.31 N. Pretension and allowed working tensions remain
unknown, so the tension warning stays. Breaking strength is not a working limit.

Both the native planner and `pyCDPR/AB1.py` use the new shared files, pinned to
robot SHA-256 `8be5ed5105f1a92263f591b0d9323759da664444e26c346579d6dc1efcb20c0c`.
Python still uses straight cables and C++ uses pulley-aware geometry.
The sampled result excludes the proposed mounting hardware and does not
establish continuous clearance or physical load capacity. Hardware readiness
remains false. The reference profile and its output below are preserved.

## Reference AB1 profile: holes on the aluminum rail faces

The reference preview is
`out/ab1-cube-500mm-frame-5004-2305-10mm-mechanics-20261029.html`.
Outer anchors now span X/Y 0..5.004 m and Z -2.305..0 m, matching the user's
5004 x 5004 x 2305 mm separation. The bed is recentered at
[2.502,2.502,-2.205] m, 100 mm above the lower anchors; initial EE position is
[2.502,2.502,-1.815] m. The anchor bounding box is used only as an offline
geometric enclosure; approved working workspace and live registration remain unknown.
The 500 mm cube
uses 10 mm first/subsequent layers and a user-approved 12 mm demo nozzle/bead,
giving 50 layers and 303 motions. The user confirmed hole
centers on the outer Y faces of the 40 mm aluminum rails. C1-C4 are
[X=+/-0.10, Y=+/-0.20, Z=+0.27] m, 100 mm along the top rails from each end.
C5-C8 are [X=+/-0.18, Y=+/-0.20, Z=-0.27] m on the lower corner faces. Body dimensions,
cable order/routing, centered CoG and nozzle Z=-0.39 m are retained.
The nozzle remains 100 mm below the outer body bottom;
its position is independent of the changed hole heights.

The versioned profile in `config/ab1-frame-5004x5004x2305/` supplies the C++ planner
and the sibling Python configuration. `pyCDPR/config/ab1-frame-5004x5004x2305.json`
references those same files, and `pyCDPR/AB1.py` validates their geometry digest.
Both programs use source SHA-256
`d374a603cacf41b84009e5f82524e8e0fd369563d5cbd5724ccd68e7d516a83f`.
The Python backend uses straight cables; the C++ assessment includes pulleys.

All 1,406 sampled poses have rank 6 and a gravity-balancing allocation.
The largest returned allocation is 979.12 N, about 57.6% of breaking strength;
this is not an approved working load. Each pose retains 16 capsule/member
contacts at the mounts (four upper single-member contacts and twelve lower
corner contacts). No additional contact is counted, including the previous
four crossing cable pairs. The four retained smallest-clearance contacts are
lower mount contacts at about -1 mm. Zero attachment exclusion preserves the
`potential_collision` warning. Working-tension/pretension limits remain unknown,
and hardware readiness remains false. This sampled model does not certify
continuous clearance or detailed mounting hardware.

See the [shared profile and commands](../config/ab1-frame-5004x5004x2305/README.md).
The preceding smaller-frame, 10 mm-layer example remains in
`out/ab1-cube-500mm-10mm-layers-mechanics-20261029.html`.
The previous 0.8 mm / 625-layer rail-face example remains in
`out/ab1-cube-500mm-rail-face-mounts-20261029.html` with its original report.
The preceding [upper-inset profile](../config/ab1-top-inset-100mm/README.md)
placed the upper points in the open top; it is retained as a historical snapshot.
The following section documents the earlier all-corners example.

## Previous AB1 example: all eight corners

Open `out/ab1-cube-500mm-corners-review-20261029.html`. It contains the original
500 mm cube toolpath, registered to the user-corrected AB1 slicer geometry, with
625 deposition heights. This is a new **geometrically_checked, untimed** job;
the older timed cube previews use synthetic geometry and remain separate.

The assessment uses the following supplied values and explicit assumptions:

| Input | Value and provenance |
| --- | --- |
| Robot geometry | `config/ab1-corners-20261029-robot.local.json`: frame/pulley/scale/envelope fields from the selected AB1 configuration, with corrected EE corners |
| Slicer model SHA-256 | `d915eae72736e15a60ea69606e145a841c69e82c13f32a3123191aede7af588c` |
| Moving mass | Approximately 70 kg, supplied by the user; treated as the total moving assembly mass for this example |
| EE frame | User-specified outer dimensions 0.40 x 0.40 x 0.58 m, built from 4040 aluminium |
| Component model | Twelve rectangular 40 mm member envelopes; a hollow frame, not a solid EE box or detailed extrusion CAD |
| EE origin and CoG | User specified the geometric centre, with CoG `[0,0,0]` relative to that origin |
| EE attachment points | All eight corners: X/Y=+/-0.20 m, Z=+/-0.29 m |
| Cable routing | User confirmed lower frame to upper EE corners (C1-C4), upper frame to lower EE corners (C5-C8); existing XY corner order retained |
| Cable diameter | User-specified 2 mm; capsule radius 0.001 m |
| Current and starting rails | User's zero-offset answer represented as `[0,0,0,0]` m for both |
| Nozzle | User corrected it to 0.10 m below the frame bottom; -0.29-0.10 gives nozzle-local `[0,0,-0.39]` m |
| Bed | User specified 0.10 m above the lowest frame attachment; config bottom Z=-1.80 gives bed world Z=-1.70 m |
| Bed registration | Aligned axes, centred in the frame at `[0.925,0.925,-1.70]` m; an offline placement assumption |
| Bed contact model | Zero-thickness 500 x 500 mm print-footprint plane, X/Y `[0.675,1.175]` m at Z=-1.70 m; full bed extent and thickness were not supplied |
| Other obstacles | Explicitly empty, following the user's answer |
| Contact allowance | Zero additional clearance and zero attachment exclusion; these are intersection-only screening settings |
| Applied load | Gravity `[0,0,-9.80665]` m/s^2; no additional external force or torque assumed |
| Cable breaking strength | User-supplied 170 daN, converted to 1,700 N per cable |
| Working limits / pretension | Unknown; both remain null |

Initial nozzle and filament datums are explicit offline scenario values, not a
live robot position or encoder reading. The calibration manifest's existing
`verified` field means the supplied registration is accepted by the software
contract; it does not establish measured registration accuracy. The physical
model's `synthetic: false` identifies user-specified AB1 data, not independently
verified measurements. Head, nozzle-mount and other unprovided components are
not invented as collision boxes.

The original root/bin controller configurations retain SHA-256
`3b39535a98935345005c4b7955706db0918b72b06faf0da4c7868cac2ee59f5e`.
The corrected geometry is isolated in the slicer profile. This local model
contains the fields used by the offline geometry loader, not a complete robot
execution configuration. Each attachment is relative to the centred EE origin:

| UI cable / config index | X (m) | Y (m) | Z (m) |
| --- | --- | --- | --- |
| C1 / 0 | -0.20 | -0.20 | +0.29 |
| C2 / 1 | +0.20 | -0.20 | +0.29 |
| C3 / 2 | +0.20 | +0.20 | +0.29 |
| C4 / 3 | -0.20 | +0.20 | +0.29 |
| C5 / 4 | -0.20 | -0.20 | -0.29 |
| C6 / 5 | +0.20 | -0.20 | -0.29 |
| C7 / 6 | +0.20 | +0.20 | -0.29 |
| C8 / 7 | -0.20 | +0.20 | -0.29 |

### Findings

The corrected nozzle lies **100 mm below the frame bottom**. The initial EE
origin/CoG is `[0.925,0.925,-1.31]` m, the frame bottom is Z=-1.60 m, and the
nozzle tip is on the bed reference at Z=-1.70 m. Minimum body/bed separation
across the path is 100 mm. All nozzle/bed toolpath coordinates are unchanged;
the EE moves 190 mm higher than in the preceding example.

The confirmed routing produces four intersecting cable pairs: **C1/C5,
C2/C6, C3/C7 and C4/C8**. They appear at all 17,506 samples. Each sample also
includes 24 capsule/member contacts at the intentional corner connections
(eight cables, three adjacent members per corner), for 28 contacts total.
Attachment exclusions remain zero; no unmeasured mounting clearance is invented.
The four contacts with smallest signed clearances are retained, exposing the
cable crossings instead of hiding them behind endpoint contacts. The first
issue is operation 5 at progress 0. Click **Inspect first issue** to see it.

The previous 90 mm frame-below-nozzle finding belongs to the retained older
`ab1-cube-500mm-mechanics-*` artifacts. The corrected geometry above supersedes
that placement while retaining the user-confirmed cable routing.

Gravity can be balanced at every sample under the mathematical bounds used.
The largest returned allocation is approximately **589.97 N** on one cable.
These are individual feasible allocations, not unique forces, minimum required
capacities, measured tensions or motor commands. Allocations can contain
zero-tension cables. That does not measure actual slack or
prove that a positive-pretension allocation is impossible. With no supplied
minimum pretension or allowed working tension, the report correctly says
`limits_not_evaluated`. A force below 1,700 N breaking strength does not establish
an acceptable working load.

The corrected symmetric arrangement has sampled wrench rank **5/6** throughout
the job. It balances the specified gravity load but cannot independently support
all six force/torque directions under this model. This limitation is displayed
alongside the unknown working limits.

## Using the view

- **Inspect first issue** pauses at the first reported contact or unbalanceable
  load sample. Reported cables, members and obstacles are highlighted red.
- **CoG / Nozzle labels** mark the centred load point and tip. Select layer 1
  and **XZ · front** to inspect the frame and the 100 mm nozzle extension clearly.
- **Play geometry / Pause / Reset** replay the untimed path. Each motion uses
  an illustrative 0.3 seconds at 1x; this is not actuator timing or print duration.
  Temperature waits still pause for manual **Continue** without heater feedback.
- **Inspect layer**, the operation slider and segment slider inspect the path.
  In this assessed view, **World / robot** draws the nozzle toolpath in world
  coordinates; the pose readout remains the EE origin. Bed/nozzle coordinates
  show the same geometry in the bed frame.
- Cable lengths remain approximate interpolated shared-kinematics values in
  metres. Forces are in newtons at the **nearest assessment sample**, not
  interpolated force predictions. Rank, sampled clearance and percentage of
  supplied breaking strength are shown separately.
- **Show modeled obstacles** includes the bed reference footprint. The generic
  grid and configured XYZ envelope are display references, not additional
  assessed obstacles. **Scope and assumptions** contains the exact input model.

Plain previews without an assessment retain their existing behavior: timed
jobs follow their recorded quintic timing; untimed jobs offer manual scrubbing.

## Repeat the assessment

The five user-specific local inputs are:

- `config/ab1-corners-20261029-robot.local.json`
- `config/ab1-corners-20261029-planner.local.json`
- `config/ab1-corners-20261029-calibration.local.json`
- `config/ab1-corners-20261029-limits.local.json`
- `config/ab1-corners-20261029-physical.local.json`

The existing geometric job can be assessed again without editing or executing
it. Use a fresh output name because publication never overwrites a file:

~~~powershell
$project = 'C:\Users\leoch\Documents\Dev\CU_brick\CDPR_3D_Printing_Slicer'
$slicer = Join-Path $project 'build\offline\bin\Release\cdpr-slicer-plan.exe'
$job = Join-Path $project 'out\ab1-cube-500mm-corners-20261029.json'
$planner = Join-Path $project 'config\ab1-corners-20261029-planner.local.json'
$physics = Join-Path $project 'config\ab1-corners-20261029-physical.local.json'
$runName = 'ab1-mechanics-' + [Guid]::NewGuid().ToString('N')

& $slicer --assess-job $job --config $planner --physics $physics `
    --output "${runName}.json" --preview-output "${runName}.html"
if ($LASTEXITCODE -ne 0) { throw 'Assessment failed; read the diagnostic above.' }
Invoke-Item -LiteralPath (Join-Path $project "out\${runName}.html")
~~~

Exit 0 means the assessment was completed and published, including when it
reports a potential collision or infeasible static load. Read the statuses;
successfully creating a report does not clear its findings. Rejected inputs
produce no partial assessment. JSON and optional HTML are separate publications;
an already published JSON is retained if HTML publication subsequently fails.

The assessment mode resolves `--assess-job`, `--config` and `--physics` against
the process working directory. Model and manifest paths inside the planner
configuration resolve against that configuration's directory. Output names
are simple `.json`/`.html` leaf names under this checkout's `out/`.

To regenerate the AB1 geometry from the existing engine output, use this same
planner configuration and a fresh JSON output name:

~~~powershell
$gcode = Join-Path $project 'out\cube-500mm-demo-3caa4dfca9a742fe88dd1c18b0f61c93.gcode'
$geometryName = 'ab1-geometry-' + [Guid]::NewGuid().ToString('N')
& $slicer --input $gcode --config $planner --stage geometrically_checked `
    --output "${geometryName}.json"
if ($LASTEXITCODE -ne 0) { throw 'Geometry planning failed.' }
~~~

Actual cable speed/acceleration limits, path acceleration, cadence and thermal
wait settings remain unknown in this profile. Do not substitute synthetic
timing limits to label the AB1 job timed.

## Physical-model input contract

All distances use metres; mass uses kilograms; force uses newtons; torque uses
newton metres. JSON requires the exact named fields below. Unknown collision
or statics data is represented by null, not guessed numeric defaults. Unknown
keys, duplicate keys, invalid ranges and nonfinite numbers are rejected.

| Object | Fields |
| --- | --- |
| Root | `schema_version`=1, nonempty `model_id` and `provenance`, explicit boolean `synthetic`, exact `robot_config_sha256`, `sampling`, `collision`, `statics` |
| `sampling` | `maximum_step_m` in (0,1], `maximum_samples` in 1..20000, `maximum_samples_per_motion` in 2..2001 and no larger than the total budget |
| `collision` | Null, or `cable_radius_m` in (0,0.1], `clearance_m` in [0,1], `attachment_exclusion_m` in [0,0.5], `ee_boxes_local_m`, `obstacles_world`, and optional `bed_reference_world` |
| `ee_boxes_local_m` | Null for unknown, or 1..32 named boxes in EE coordinates |
| `obstacles_world` | Null for unknown, or 0..64 named world boxes; `[]` explicitly declares no such obstacles |
| Named box | Unique `id`, `minimum_m` and `maximum_m` three-vectors; strictly positive dimensions |
| `bed_reference_world` | Optional null or named positive-area XY plane with equal min/max Z; a footprint reference, not bed thickness |
| `statics` | Null, or `mass_kg`, `cog_ee_m`, `gravity_m_per_s2`, `external_force_n`, `external_torque_nm`, `minimum_tension_n`, `maximum_tension_n`, `breaking_strength_n` |
| Tension fields | Null when unknown, one common scalar, or eight numbers in RobotConfig cable order 0..7; nonnegative minimum, strictly larger positive upper bound; working upper cannot exceed a supplied breaking strength |

Gravity, external force and external torque are world-frame vectors. The CoG
is EE-local. External force acts at the EE origin; any additional moment about
that origin must be supplied in `external_torque_nm`. Unknown minimum tension
uses only mathematical nonnegativity. Unknown working maximum uses the supplied
breaking strength as an ultimate ceiling if available; otherwise it is
unbounded. Either missing working bound leaves the operating-limits check
unevaluated. The parser also limits numeric magnitude, identity lengths, JSON
nesting and bytes; see `MechanicsAssessment.cpp` for exact numeric envelopes.

## Calculation and scope

Samples include both ends of each translating motion with spacing no greater
than `maximum_step_m`. Stationary E-only moves need one pose. Orientation must
be fixed within each motion. Budgets are checked before calculation; intervals
between poses are not continuously certified. The report is limited to 24 MiB,
and the existing standalone HTML limit remains 64 MiB.
If indentation alone exceeds the report cap, the writer uses compact JSON
before checking that same cap; samples and numeric values are preserved.

The adapter exposes the pulley tangent outlet and EE attachment already used
by its guarded intermediate geometry. Cable lengths still come exclusively
from the existing `cu_brick::ComputeCableLengths` source referenced in place.
Collision checks use finite-radius free-span capsules for cable/cable and
cable/box distances. Optional EE component boxes support cable/body checks in
local coordinates. Each transformed component's conservative world AABB is
screened against declared world obstacles and the optional bed plane. Rotated
component bounds can over-report contact; contact is labelled potential.
Optional attachment exclusion trims both ends of each cable span and is
explicitly excluded from coverage. AB1 currently uses no trimming.

For each pose the cable wrench matrix has columns
`[u_i; (attachment_i - EE_origin) cross u_i]`, where `u_i` points along the
straight free cable toward its pulley tangent. The solver seeks bounded
`T` satisfying `W*T = -[external force + m*g; external torque + (R*CoG) cross m*g]`.
It tries SVD projection and active-bound projection, with independent-column
vertex enumeration as a fallback. It accepts an allocation only after checking
its bounds and both residual norms (force <=1e-6 N, torque <=1e-6 Nm).
The relative SVD/QR rank threshold is 1e-10. A rank below six indicates missing
wrench authority even when this particular gravity load balances.

This calculation does not model inertia, cable elasticity, friction, sag,
actuator response, actual pretension, growing deposited material, full robot
chassis, unprovided head geometry, moving obstacles, thermal behavior or
feedback. Neither this assessment nor a wrench rank of six establishes a
complete feasible workspace or safe execution.

The sealed report binds the canonical job, model snapshot, embedded physical
model and exact physical-input bytes by SHA-256. Preview export rejects stale
or changed reports and models. These digests detect inconsistency; they are
not signatures or hardware authorization. Reports store the four contacts
with smallest signed clearances per pose plus the full count, allocations/residuals and sampled
coverage. The job remains immutable and `simulated` remains unsupported.

## Development and remaining inputs

No new dependency was added: the numerical code uses the repository's existing
Eigen and JSON headers. `MechanicsTests.cpp` verifies force/moment balance,
bounded and impossible loads, collision cases, missing data, budgets and report
binding. CLI/browser acceptance evidence is recorded in [validation](validation.md).
The architecture and source boundaries are in [architecture](architecture.md).

Next physical inputs are detailed mounting/pulley routing and bed geometry
to evaluate the remaining cable contacts, minimum pretension and
manufacturer-approved allowed working tensions, plus
the existing motion/feedback and print-head requirements. Hardware integration
and operation still require their separate approval.

Numerical background: [Eigen 3.3 linear algebra and residual checking](https://libeigen.gitlab.io/eigen/docs-3.3/group__TutorialLinearAlgebra.html).
