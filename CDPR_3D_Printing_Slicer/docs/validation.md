# Offline validation and 3D preview record

## Corner attachments, centred CoG and nozzle correction — 2026-10-29

The user corrected the EE attachments to all eight corners of the 400 x 400 x
580 mm frame, specified its origin/CoG at the centre, and placed the nozzle
100 mm below the frame bottom. They explicitly confirmed retaining the saved
routing: lower frame anchors to upper EE corners and upper frame anchors to
lower EE corners. XY corner order and cable numbering are preserved.

The new slicer model has attachment coordinates X/Y=+/-0.20 m, Z=+/-0.29 m,
CoG `[0,0,0]`, nozzle `[0,0,-0.39]` m. The original 500 mm cube/nozzle path stays
at its existing world location; every EE endpoint moves 0.19 m upward. Initial
EE/CoG world position is `[0.925,0.925,-1.31]` m; the frame bottom is Z=-1.60 m,
100 mm above the bed reference at Z=-1.70 m. Final-layer EE Z is -0.81 m.

Five new ignored inputs use prefix `config/ab1-corners-20261029-`:
`robot.local.json`, `planner.local.json`, `calibration.local.json`,
`limits.local.json`, `physical.local.json`. The robot file contains only the
native offline loader's geometry/scale/envelope fields plus provenance. Its
SHA-256 is `d915eae72736e15a60ea69606e145a841c69e82c13f32a3123191aede7af588c`.
Root/bin controller configurations retain their prior bytes; this correction
targets the slicer model, not the hardware application's configuration.

| Artifact | Purpose |
| --- | --- |
| `out/ab1-cube-500mm-corners-20261029.json` | Corrected, untimed geometric job with 3,753 motions / 625 layers |
| `out/ab1-cube-500mm-corners-review-20261029.json` | Full 17,506-sample assessment of the corrected geometry |
| `out/ab1-cube-500mm-corners-review-20261029.html` | Current preview with labelled CoG/nozzle, corner attachments and cable-crossing highlights |

The confirmed routing intersects C1/C5, C2/C6, C3/C7 and C4/C8 at every sampled
pose. Each pose reports 28 contacts: four cable pairs plus 24 intentional
capsule/member endpoint contacts. No attachment regions were silently trimmed.
The contact cap now retains the four smallest signed clearances rather than
the first four encountered, so the crossing pairs remain visible. A native
regression checks that multiple mounting contacts cannot mask closer crossings.
The UI displays C1-C8 names, total contact count and the sampled rank warning.

Gravity equilibrium is feasible at all samples under nonnegativity and the
ultimate 1,700 N ceiling; peak allocated force is 589.9668021464136 N. Working
limits/pretension remain unknown. Wrench rank is 5/6 throughout: a gravity-load
allocation does not establish full force/torque authority. Hardware readiness
remains false and no robot/print-head connection was made.

The larger set of detailed contact records exceeded the 24 MiB limit when
pretty-printed. The writer now falls back to compact JSON before applying the
same byte cap, retaining every sample and numeric value. The final report is
15,732,893 bytes, SHA-256
`9bd141c78ea76cc291d31aa0bd29a94a01e69d6c11530dbbabe84ca65a55089a`.
The initial rejected export published no partial artifact.

Verification:

- Debug and Release builds / both CTest entries passed, covering 18 groups:
  `build/offline/build-corners-final-debug.log`,
  `build/offline/ctest-corners-final-debug.log`,
  `build/offline/build-corners-final-release.log`,
  `build/offline/ctest-corners-final-release.log`.
- Nine artifact/browser groups passed: exact corner/model hashes and preserved
  controller/old-job bytes; all motion transforms; full bounded assessment;
  visible CoG/nozzle labels and rank; issue seeking with all four pairs;
  first-layer 3D/XZ geometry; last layer/play/pause; reset/mobile layout;
  zero page warnings/errors/external requests.
- Evidence: `build/offline/qa/ab1-corners-1793247011664/results.json`.
  Screenshots `corners-first-layer-3d.png`, `corners-first-layer-xz.png` and
  `corners-mobile.png` were visually inspected. Desktop was 1440 x 1040;
  mobile was 390 x 844 with no horizontal overflow. Existing Playwright/Edge
  was used because the Browser plugin was unavailable.

Source changes are limited to `src/visualization/PreviewTemplate.html`
(CoG/nozzle labels and contact/rank display),
`src/planning/MechanicsAssessment.cpp` (bounded contact selection/serialization)
and `tests/MechanicsTests.cpp` (contact-cap regression), plus project README and
documentation. Preparation/QA scripts stay under project `build/offline/qa/`.
Older reports remain snapshots of their earlier input geometry; the current
usage command and exact attachment table are in [mechanics](mechanics.md).

## AB1 sampled mechanics and geometry replay — 2026-10-29

The user approved offline collision/tension work and supplied approximate mass,
EE dimensions/CoG, cable diameter/breaking strength, zero rail offsets and
bed/nozzle heights. The implementation and all generated data stay inside the
slicer project. It reuses the original shared cable solver in place and adds no
dependency, controller connection or hardware execution. See
[mechanics](mechanics.md) for assumptions, input contract and commands.

### Artifacts and findings

| Artifact | Purpose |
| --- | --- |
| `out/ab1-cube-500mm-mechanics-20261029.json` | Untimed AB1 geometrically checked cube: 3,764 operations, 3,753 motions, 625 layers |
| `out/ab1-cube-500mm-mechanics-report-20261029.json` | Separate model/physical-input-bound sampled assessment |
| `out/ab1-cube-500mm-mechanics-view-20261029.html` | Standalone 3D geometry replay with members, bed footprint, contacts and static force readouts |

The assessment checks 17,506 poses at no more than 0.1 m spatial spacing. It
reports `potential_collision` at 3,142 poses, involving the EE members and the
zero-thickness bed reference footprint. The first issue is operation 5,
progress 0, EE world `[0.925,0.925,-1.5]` m; the last sampled contact is operation
679 at EE Z=-1.4104 m. No free-cable contact was found in these samples.

The supplied nozzle offset is EE-local Z=-0.20 m, while the centred 580 mm frame
extends to Z=-0.29 m. The model therefore places members **90 mm below the tip**,
crossing the bed reference at the initial pose. The input dimensions and nozzle
position were retained. This is a placement/measurement issue to resolve,
not a cleared print plan.

One static gravity-load allocation exists at every sample under nonnegativity
and the supplied 1,700 N breaking-strength ceiling. Peak returned allocation is
373.0541579782985 N. Every allocation includes a zero-tension cable; this neither
measures actual slack nor proves that positive pretension is impossible.
Minimum sampled wrench rank is six, maximum recorded force residual is
1.37e-12 N and maximum moment residual is 4.54e-13 Nm. Minimum pretension and
maximum allowed working tension remain null, so tension status is correctly
`limits_not_evaluated`. Breaking strength is not a working-load rating.

The job remains byte-for-byte unchanged and untimed. Original planning checks
are not promoted; report and view retain `hardware_ready=false`. Geometry replay
uses explicitly illustrative 0.3 seconds per motion and pauses for manual
Continue at the two temperature waits. No actuator timing or thermal feedback
is inferred. Growing print, full bed/chassis/head CAD, sag and dynamics remain
outside this assessment.

Exact SHA-256 values:

- Job bytes / report job binding:
  `bdf6b0074ee7691b64d3f4860784c468b8de5097b72b7b2452b045a50b7921e8`.
- Report file bytes:
  `0be0cd38b0f93da20ab37a2087f2b16906e728522ed1b4584d634fd23c68c3d0`.
- Physical source bytes:
  `ff6214fbde64d3fdd4876524164a953f33ce2d28e85e7b488346662e69099efd`.
- Root and bin AB1 configurations both remain:
  `3b39535a98935345005c4b7955706db0918b72b06faf0da4c7868cac2ee59f5e`.

### Verification

| Verification | Result and evidence |
| --- | --- |
| Final MSVC Debug build + CTest | PASS, both entries / 18 groups; `build/offline/build-mechanics-final-debug.log`, `ctest-mechanics-final-debug.log` |
| Final MSVC Release build + CTest | PASS, both entries / 18 groups; `build/offline/build-mechanics-bed-release.log`, `ctest-mechanics-bed-release.log` |
| Mechanics native tests | Five groups: missing inputs/binding/immutability; independently recomputed force and moment balance; impossible/deficient loads; cable/body/obstacle/bed contacts; budgets/malformed data/stale reports/inert HTML |
| CLI acceptance | Seven groups PASS: separate report/immutable job, no overwrite, cross-robot rejection, intentional collision and impossible load, budget/no partial output, traversal rejection, duplicate physical keys; `build/offline/qa/mechanics-cli-summary.json` |
| Full AB1 artifact checks | Four groups PASS: exact input bindings, all 17,506 expected sample locations, allocation bounds/residuals and unknown-limit status, preserved bed/frame finding; `build/offline/qa/ab1-mechanics-verification.json` |
| Edge desktop/mobile acceptance | Eleven groups PASS: physical data/readouts, unknown working limits, first issue, geometry Play/Pause, final layer/immutable timing, wait boundaries/reduced motion, 390 px mobile layout, intentional negative case, positive control, legacy timed/untimed controls, zero page errors/warnings/external requests |

The Browser plugin was unavailable; existing Playwright and installed Edge were
used with an isolated project-local profile. Browser evidence:
`build/offline/qa/mechanics-browser-1793205781449/results.json`, with screenshots
`ab1-first-bed-issue.png`, `ab1-last-layer.png`, `ab1-mobile.png`. All three were
visually checked, and the mobile page has no horizontal overflow. Logs are in
`build/offline/qa/mechanics-browser-checks.log` and
`build/offline/qa/mechanics-cli-checks-final.log`.

Source changes were confined to the following project-relative files:

| File | Change |
| --- | --- |
| `include/cdpr_slicer/Mechanics.h` | Free-span, assessment, validation and preview interfaces |
| `src/cdpr/KinematicsAdapter.cpp` | Expose existing guarded pulley intermediates while retaining shared length computation |
| `src/planning/MechanicsAssessment.cpp` | Strict physical-input decoder, sampled contacts, bounded statics, report binding/validation |
| `src/cli/PlanMain.cpp` | Separate `--assess-job` mode using existing guarded read/new-file publication |
| `src/visualization/PreviewWriter.cpp` | Bind the optional assessment to its exact job and model |
| `src/visualization/PreviewTemplate.html` | Component/obstacle rendering, contact/force readouts, issue seeking, explicit geometry replay |
| `tests/MechanicsTests.cpp` | Five numerical/contract regression groups |
| `CMakeLists.txt` | Add the source and Windows mechanics CTest target, with existing dependencies |
| `README.md`, `docs/architecture.md`, `docs/mechanics.md`, `docs/validation.md` | Current behavior, assumptions, usage and evidence |

The four `config/ab1-mechanics-20261029-*.local.json` files are ignored local
scenario inputs. Build/QA logs, scripts and artifacts remain under project
`build/` and `out/`; earlier examples were retained. Existing root/bin robot
configuration hashes, `.vscode/settings.json`, root README,
`docs/WORKFLOW_ASCII.md` and `bin/RobotConfig_YES.json` match their pre-phase
baselines. No root controller/kinematics source, tracked index or commit changed.
The following sections record earlier phases and their then-current limits.

## 500 mm cube bottom-centre placement — 2026-10-28

The user requested moving the example's start to the workspace bottom and middle.
The synthetic XYZ envelope is [-0.5,+0.5] m on each axis and applies to the EE
origin. New dedicated local profiles preserve the existing 50 mm positive-Z
EE-to-nozzle offset and set world_from_bed translation to (0,0,-0.45) m. Thus the
initial EE is (0,0,-0.5) m, exactly the bottom-centre of that configured envelope.
The first print layer is EE Z=-0.4992 m; the nozzle's bed plane is world Z=-0.45 m.
The original source profiles and artifacts were retained unchanged.

New configuration files:
`config/cube-500mm-bottom-centre.local.json` and
`config/cube-500mm-bottom-centre-calibration.local.json`.
They reuse the existing synthetic model, limits, slicing recipe and explicit
zero rails; their provenance labels the registration as an offline mathematical
example only. No real AB1 calibration was asserted or controller setting changed.

A fresh engine/planner run produced artifact prefix
`out/cube-500mm-bottom-centre-2f8ab389f5404a51a8bbd0f38a168691`, with G-code,
timed JSON, standalone HTML and exact-byte receipt. All 3,753 motions / 3,764
operations and 625 layer heights were retained. Eight implemented checks passed;
nine remain unevaluated and hardware readiness stays false.

`build/offline/qa/cube-bottom-centre-check.cjs` checked all motions against the
previous run: nozzle/bed geometry, operation kinds, signed E and extrusion spans
are unchanged; each EE start/end equals its previous world position minus
(0.1,0.2,0.45) m. All EE endpoints lie in the unchanged XYZ envelope. EE bounds
are X/Y +/-0.2494 m and Z -0.5..approximately 0 m; nozzle world Z spans
-0.45..+0.05 m. Exact receipt hashes passed. Structured evidence:
`build/offline/qa/cube-bottom-centre-results.json`.

Six Edge desktop/mobile acceptance groups passed: initial bottom-centre EE,
first layer at -0.4992 m, last layer at approximately 0 m, Reset restoring the
bottom start, mobile fit, and zero page warnings/errors/external requests.
Evidence and screenshots:
`build/offline/qa/cube-bottom-centre-browser-1793177546957/`.
Log: `build/offline/qa/cube-bottom-centre-browser.log`.
The completed placement screenshot was visually checked. Browser plugin remains
unavailable; existing Playwright/Edge ran with an isolated project-local profile
through the established browser escalation path.

Only local demo profiles and slicer documentation changed; no source, global
example calibration, measured robot configuration or renderer was modified.
No rebuild or unrelated test rerun was needed. This validates commanded geometry,
not body/cable collisions, dynamics or physical execution. No hardware was used.

## 500 mm cube example — 2026-10-28

The user requested a 50 cm cube STL and a code run. A new
`tests/fixtures/cube-500mm.stl` was generated from the existing closed cube mesh
with a uniform 50x coordinate scale. It is 500 x 500 x 500 mm, with 12 triangles,
8 unique vertices, 18 two-face mesh edges, consistent outward orientation and
signed enclosed volume 125,000,000 mm³. Mesh SHA-256:
`f9149c78ca83b7d2ebca9996b52e00a8cd4a45fd91f669a78acd334f6d10d606`.

The unchanged default profile correctly rejected the model as outside its
100 mm print volume. PrusaSlicer 2.9.6 reported this on stderr but returned zero
and produced no G-code. `tools/Invoke-CdprSlice.ps1` now checks for the missing
output explicitly and includes both engine output streams in its diagnostic.
The same wrapper now accepts optional `-SlicingProfile` and `-CenterXYMm` inputs,
retaining producer execution guards, finite placement validation, new-file-only
publication, exact profile provenance and actual center coordinates in receipts.
Defaults remain the existing profile and (50,50) mm center.

A dedicated synthetic demo used `config/cube-500mm-demo.local.ini` and
`config/cube-500mm-demo.local.json`: 600 mm print volume centered at zero,
1.2 mm nozzle/width, 0.8 mm layers, one perimeter and no infill or top/bottom
caps. The existing synthetic model/registration/limits were reused; temporal
sampling was explicitly 10 s and spatial interval 50 mm for this coarse preview.
These are visualization-only settings, not AB1 calibration or print-head data.
The full-sized STL was not reduced or truncated. The local profile files are
ignored development data; their exact digests are recorded in the run receipt.

The actual engine and Debug planner completed successfully:
`out/cube-500mm-demo-3caa4dfca9a742fe88dd1c18b0f61c93` with `.gcode`, `.json`,
`.html` and `.receipt.json` outputs. It contains 3,753 motions / 3,764 operations,
625 heights from 0.8 to 500 mm and 56,275 timing knots. Depositing centerline XY
bounds are +/-249.4 mm, accounting for the 1.2 mm extrusion width. Eight checks
passed, nine remain unevaluated, and hardware readiness remains false. Slicing
took 0.446 s and planning/preview 48.478 s. The conservative model's total
449,926.854 s excludes unknown temperature waits and is not a validated physical
print-time estimate. Geometry/output checks and hashes are retained in
`build/offline/qa/cube-500mm-check.cjs` and `cube-500mm-results.json`.

`tests/SliceWorkflow.Tests.ps1` passed **28 checks** on Windows PowerShell
5.1.26100.9711. The four added checks cover malformed/nonfinite placement,
zero-exit print-volume rejection and producer guards on an explicit profile.
The existing 10 mm end-to-end path still passed after the wrapper extension.
Final log: `build/offline/qa/cube-500mm-workflow-tests-final.log`.
Structured evidence:
`build/slice-tests-60d8eec614284a1283b7ee4de7c91404/results.json`.

Browser plugin not available: existing Playwright and Edge were used, with a
project-local isolated browser profile via the previously required browser
escalation path. Five large-cube acceptance groups passed: correct 625-layer
page/model identity, final Z=500 mm with travel hidden and material retained,
mid-height Z=250.4 mm, desktop/mobile fit, and no page warnings/errors/external
requests. Load was 273 ms on this workstation. The final cube screenshot was
visually checked. Evidence and screenshots:
`build/offline/qa/cube-500mm-browser-1793173852972/`.
Log: `build/offline/qa/cube-500mm-browser.log`.

Only the slicer wrapper, its workflow tests, the new STL and slicer documentation
were changed; local demo/QA files stay under the slicer project. Native C++ and
renderer source were unchanged, so neither was rebuilt. Default profiles and
both measured AB1 RobotConfig files retain their earlier digests. No controller
application was launched, no calibration state was inferred and no hardware was
connected or operated. Reproduction and limits are in
[large-cube-example.md](large-cube-example.md).

## Measured AB1 configuration import — 2026-10-23

The user supplied fetched commit
`f1ab67c63d3a4ceb63c4c3b3a4a7540ed6752bab` and explicitly requested merging its
updated configuration into the code and using its information. That commit
changes the root `RobotConfig.json` as well as unrelated logs, CSVs and recovery
state. Only its configuration blob was applied: both root `RobotConfig.json`
and the previously selected `bin/RobotConfig.json` now contain that exact blob.
The other commit files were not imported. No branch merge/commit or staging was
performed; HEAD remains `f9fb380cadd2310066ef73ce7c4f9e1d385b0a4c`.

Source Git blob: `65448337252e6a333d0acd3e627ac2f82a8b64d3`.
Both applied files have SHA-256:
`3b39535a98935345005c4b7955706db0918b72b06faf0da4c7868cac2ee59f5e`.
Git's worktree restore could not acquire an index lock in the read-only Git
metadata sandbox. Reading the blob and writing its exact bytes into the two
authorized workspace configuration files completed the file-only import without
changing the index. Original bytes are retained under
`build/config-merge-f1ab67c-c94e2f67849142e78fead6c8b521daea/`, together with
`import-receipt.json` recording the source revision and file scope.

The measured AB1 frame now spans X/Y 0..1.85 m and Z -1.8..0 m, replacing the
previous bin configuration's 2.115 m span / -1.78 m lower anchors. The imported
configuration also supplies the common cable scale 815027.0614, rail scale
-7388.5256197, 45 mm pulley radius, 0.2 m/s path bound and pickup preset
(1.05, 1.05, -0.5) m. The existing legacy decoder/kinematics are reused in place.

A new model-pinned package at `config/machines/ab1-f1ab67c-20261023/` references
the updated bin configuration. Its setup report records the source commit and
configuration path in that commit. The prior `ab1-20261023/` package is retained
as a historical snapshot, whose old model digest correctly becomes stale.
Current/starting rail state, print-bed/nozzle registration, initial nozzle/E
state, print limits and policies absent from the supplied file remain unfilled.
No camera/pickup/home preset was treated as print registration or a measured
starting pose. No real-machine motion plan or physical readiness claim was made.

`tests/MachineProfile.Tests.ps1` was updated to use the separate YES config for
the alternate-model test, because root and bin now intentionally match. All
**19 machine-profile checks passed** on Windows PowerShell 5.1.26100.9711.
Log: `build/offline/qa/machine-profile-f1ab67c.log`.
Structured evidence:
`build/machine-profile-tests-b977a93fe6254e4c98470a2968dba440/results.json`.
The existing native CTest program passed all **13 offline groups** (1/1, 1.58 s).
Log: `build/offline/ctest-config-f1ab67c.log`.
No native source was changed or rebuilt, and the renderer was unchanged.

Imported device settings include isOnline=true/useCableMotor=true. The robot
control application was not launched. The source's malformed RPiIP string
`19268.101.30` and broad XYZ bounds were preserved exactly and flagged in the
machine-profile guide for review; no address or mechanical limit was guessed.
Offline preparation/planning do not consume device-enable flags or addresses.
Pre-existing `.vscode/settings.json`, root README and workflow edits were
preserved, as were the YES configuration and recovery/log/CSV files.

## AB1 measurement-template preparation — 2026-10-23

After requesting continuation, the user explicitly selected
`bin/RobotConfig.json` (AB1) and asked to prepare templates for missing print-bed,
nozzle and rail measurements. The new tool is preparation only. It does not
claim the actual machine is calibrated or generate a real-geometry print job.

| File | Change |
| --- | --- |
| `tools/New-CdprMachineProfile.ps1` (new) | Explicit local configuration selection, source digest, new incomplete manifest bundle, raw geometry observations and measurement checklist |
| `tests/MachineProfile.Tests.ps1` (new) | Profile identity, unknown-input preservation, publication/path/name guards, model selection and native incomplete-profile rejection |
| `.gitignore` | Keep generated local machine packages under ignored `config/machines/` |
| `docs/machine-profile.md` (new) | AB1 source selection, actual saved geometry/bounds, template conventions and completion workflow |
| `README.md`, `docs/architecture.md`, `docs/stl-workflow.md`, `docs/validation.md` | Record the preparation command, scope and evidence |

The prepared user package is `config/machines/ab1-20261023/`, containing
`planner.local.json`, `calibration.local.json`, `limits.local.json`,
`setup.local.json` and `MEASUREMENTS.md`. The source remains in place and is pinned
to SHA-256 `56eb4790762ec2fe7674522f07306391949991691bb1a4848b466c7027146400`.
Current/starting rail arrays, initial nozzle pose/E datums, print registration
and operating parameters stay null; calibration status remains unverified.
No saved home position, recovery/log state, synthetic limit or identity
registration was substituted for a missing measurement.

`tests/MachineProfile.Tests.ps1` passed **19 checks** on Windows PowerShell
5.1.26100.9711. Checks covered the exact model binding, all missing measurement
groups, absent calibration provenance, output path, explicitly incomplete status,
source observations, no-overwrite/byte preservation, traversal/device-name and
malformed-input rejection, distinct AB1 selection, native planner rejection with
the expected missing-starting-pose diagnostic, no job publication, and unchanged
original configuration bytes. Tests use the existing built offline CLI; they
do not slice a model or load a hardware driver.

Final log: `build/offline/qa/machine-profile-tests-final.log`.
Structured evidence:
`build/machine-profile-tests-bf503008aec34808988a8ab704f6fc72/results.json`.
The initial test attempt hit Windows PowerShell's terminating handling of native
stderr; the expected-error test was corrected to capture the diagnostic and
exit code explicitly, then all 19 checks passed.

AB1's saved nominal anchor envelope is X/Y 0..2.115 m and Z -1.78..0 m.
The existing legacy XYZ bound pairs decode to X -0.1..2, Y -0.02..13.8 and
Z -3.75..0.3 m; the configured path-speed bound is 0.2 m/s. These recorded bounds
extend beyond the anchor envelope and require review/tightening as appropriate
for a measured print workspace. They are not edited or treated as a physical
feasibility result. See `docs/machine-profile.md` for the exact pairing.

All three existing RobotConfig files and the pre-existing root README/workflow
document retained their earlier SHA-256 digests. No native source, dependency,
renderer, controller or root build/configuration file was changed. A rebuild,
external-engine rerun and browser rerun were unnecessary for this preparation
change; the previous STL-stage results below remain the last evidence for those
unchanged components. No physical measurement, connection or operation occurred.

## STL workflow update — 2026-10-22

The user approved the proposed real-STL -> external slicing engine -> existing
CDPR planner -> 3D preview milestone. This phase adds the PowerShell adapter,
reviewed PrusaSlicer export profile, complete synthetic examples and layer/travel
inspection. It does not modify the robot-control application or implement a
hardware host. No physical connection or operation was performed.

| File | Change |
| --- | --- |
| `tools/Invoke-CdprSlice.ps1` (new) | Direct external process, input/profile snapshots, private data/temp, command guards, existing timed CLI, new outputs and receipt |
| `config/prusa-cdpr.ini` (new) | PrusaSlicer 2.9.6 linear single-tool compatibility profile |
| `config/synthetic-planner.json` (new) | Complete explicit offline example using the existing synthetic model |
| `config/synthetic-calibration.json`, `config/synthetic-limits.json` (new) | Synthetic registration and operating-limit fixtures, never machine calibration |
| `tests/fixtures/cube-10mm.stl` (new) | Closed 12-triangle 10 mm cube in millimetres |
| `tests/SliceWorkflow.Tests.ps1` (new) | Real external-engine pipeline and rejection/provenance/geometry checks |
| `tests/PreviewWorkflow.Tests.cjs` (new) | Existing Playwright/Edge acceptance checks for the real cube |
| `src/visualization/PreviewTemplate.html` | Geometry-derived layer selector, cumulative build inspection, travel toggle, deposited-path fitting |
| `docs/stl-workflow.md` (new), `README.md`, `docs/architecture.md`, `docs/gcode-dialect.md`, `docs/validation.md` | Usage, limits, interface and validation record |

The official PrusaSlicer 2.9.6 portable Windows ZIP was downloaded into
`build/tools/` and checked against the official GitHub release asset SHA-256:
`5aaf22e42f95accecfa122d23a835911f289ecc2ff606db3e83d637ddcc0a209`.
The archive is 106,598,059 bytes; its console executable digest is
`7fd50b52d1cc3da87dbcb71e45e764412dd45688bea028f351333e86d9769704`.
The release/source/license links are in [stl-workflow.md](stl-workflow.md).
No global dependency installation or C++ dependency/build-link change occurred.
The workflow script itself performs no downloading.

The serial Debug rebuild passed with MSVC/C++14 and no compiler warnings.
CTest 1/1 passed all **13 offline groups** in 1.49 s. Logs are
`build/offline/build-stl.log` and `build/offline/ctest-stl.log`.

`tests/SliceWorkflow.Tests.ps1` passed **24 checks** under Windows PowerShell
5.1.26100.9711 using the real engine and planner. It checked output-name
traversal, missing/non-STL files, profile post-processing/duplicate/binary
guards, unsupported generated G28 propagation with no plan publication,
literal spaces/apostrophes/Unicode in an input path, all three output digests,
the mesh/configuration/source hashes, offline readiness, eight implemented
checks, 40 layer heights from 0.25 through 10 mm, the centered 10 mm footprint,
retained retraction/deposition, frame transforms and no-overwrite behavior.
Final log: `build/offline/qa/stl-workflow-final.log`.
Structured evidence:
`build/slice-tests-c9faff3f624840218645b22c8955e1c4/results.json`.

The final real cube contains **1,356 motions / 1,369 operations**, 40 deposition
heights and 1,027 depositing spans. PrusaSlicer took 0.327 s; the existing Debug
planner plus preview took 28.302 s on this workstation. These are observations
for this fixture, not a large-model performance claim.

Final artifact prefix: `out/cube-acceptance-de6f256d09144413b172d98d9b0347bf`.
Its `.gcode`, `.json`, `.html` and `.receipt.json` files are retained. Exact
output digests recorded in the receipt are:

| Output | SHA-256 |
| --- | --- |
| G-code | `4748ee2c7a8e6ada6eaf358a932e06b30e90e403cd0865b7ac9749796e696294` |
| Timed JSON | `52a913276244fb5a574fa98963d50c1e4086700dd981cadf2219ae22709990a4` |
| 3D HTML | `6baff06ee8f3b467173099d82404e971bf261a7783f3ccc39fd681cfbf572a3f` |

Browser plugin not available: regular existing Playwright 1.62.1 and installed
Edge 155.0.4283.62 were used with a separate project-local browser profile.
Restricted-process browser startup failed; the local browser check then ran
through the approved escalation path. No npm dependency/browser was installed.
The final cube passed **seven browser groups** at 1440 x 1040 and 390 x 844:
page/model/layer identity, progressive layer selection with correct nozzle Z,
travel hiding while retaining deposits, both temperature waits and playback
pause, redraw observations, mobile layout, and zero console warnings/errors or
external page requests. The 237 ms load and 16.6 ms median / 17.7 ms p95 redraw
intervals are fixture observations including animation-frame pacing.
Screenshots of the cube, top view and mobile robot layout were visually checked.
Evidence and screenshots:
`build/offline/qa/stl-browser-1792647628009/`.
Console log: `build/offline/qa/stl-browser-verified.log`.

Limitations: STL only, fixed 100 x 100 mm export bed centered at (50,50),
millimetres, the exact reviewed engine/profile, horizontal deposition-height
display groups, and the existing sample/artifact/20,000-motion preview limits.
The receipt is provenance, not a signature or execution authorization. The
cube uses synthetic geometry, calibration and limits. Timing remains rest-to-rest
at every original segment; nine existing checks remain unevaluated and
`hardware_ready` remains false. No dynamic simulation, continuous corner
blending, physical bead model, controller integration or hardware operation
is added by this phase.

The original root `README.md` and `docs/WORKFLOW_ASCII.md` retained their prior
digests recorded below. Existing robot/controller/build/configuration files
outside this project were not modified. Existing demo outputs were retained.

## 3D preview update — 2026-10-22

The user requested a 3D CDPR simulation in place of the projection. The approved
change implements offline commanded kinematic playback. It adds no physical
dynamics, controller changes, device connection or dependency installation.
All edits, generated artifacts, QA scripts, logs, screenshots and the isolated
browser profile stay inside this project.

The existing user demo was exported to new files:
`out/demo-3d-1792645883867-timed.json` and
`out/demo-3d-1792645883867-timed.html`. The new JSON is byte-for-byte identical
to `out/demo-5ffcaa73bd5c442a842364bc8ab97b49.json`; the plan ID and 8 passed /
9 unevaluated checks are unchanged. Neither original demo file was replaced.

| File | 3D update |
| --- | --- |
| `CMakeLists.txt` | Bundle local HTML, split C++ string literals for MSVC, track template changes |
| `include/cdpr_slicer/Planner.h` | Model-bound preview overload; preserve path-only overload |
| `src/cli/PlanMain.cpp` | Pass the already selected model to advanced-stage preview export |
| `src/visualization/PreviewWriter.cpp` | Validate model identity/registration; export geometry and bounded shared-solver samples |
| `src/visualization/PreviewTemplate.html` (new) | Perspective renderer, robot/cables/EE/nozzle, camera, timing, waits, controls and readouts |
| `src/visualization/PreviewTemplate.h.in` (new) | Generated C++ embedding template |
| `tests/OfflineTests.cpp` | Model identity/seal rejection, cable endpoint equality, unchanged artifact data and marker escaping regressions |
| `README.md`, `docs/architecture.md`, `docs/validation.md` | Usage, implementation contract and execution evidence |

Final build/CTest commands are the same serial Debug commands recorded in the
S2-C section below. MSVC initially rejected a single large literal (C2026);
splitting the bundled template into adjacent line literals resolved it.
The final build has no warnings/errors. All **13 Windows offline groups pass**,
including the new model-bound 3D preview group (CTest 1/1, 1.47 s).
Logs: `build/offline/build-3d.log`, `build/offline/ctest-3d.log` and
`build/offline/Testing/Temporary/LastTest.log`.

Browser flow: local 3D preview -> scrub/play/pause -> temperature-wait continuation
-> orbit/pan/zoom and coordinate changes -> render updated robot/path/readouts.
Browser plugin not available; the already installed Playwright 1.62.1 and Edge
155.0.4283.62 were used with a separate project-local browser profile. No browser
package was installed. Browser startup used the previously reviewed sandbox
escalation because Edge cannot start reliably in the sandbox.

| Required browser check | Result |
| --- | --- |
| Identity | PASS: expected local HTML URL and CDPR 3D simulator title |
| Nonblank | PASS: rendered frame, eight cable rows, EE/nozzle, 17 checks, meaningful controls |
| Framework overlay | PASS: none |
| Console/page errors | PASS: zero warnings, errors or exceptions |
| Screenshot evidence | PASS: desktop, mobile and closeup screenshots visually inspected |
| Interaction proof | PASS: all nine acceptance groups below |

1. Initial perspective view, 8 cable endpoints/readouts, correct stage/checks.
2. Exact quintic midpoint: EE [0.105,0.2,-0.0499] m, nozzle [0.005,0,0.0001] m,
   commanded filament 0.5 mm, and changing sampled cable lengths.
3. Play advances motion; Pause retains operation and fraction without drift.
4. Both temperature waits pause until explicit Continue; epochs, dwell, end and
   Reset preserve their expected order.
5. Orbit, wheel zoom, Shift-pan, keyboard orbit, fit controls, workspace toggle
   and every projection update the canvas.
6. Reduced-motion preference change pauses playback; pages do not autoplay.
7. 390x844 layout has no horizontal overflow; emulated two-finger pinch changes
   the camera. Projected label positions confirm all eight anchors fit in both
   desktop and mobile canvases after Fit/Reset.
8. Geometric stage supports manual scrubbing with Play disabled; imported stage
   displays no invented robot geometry or cable data.
9. Page requests contain only the three local preview navigations; no external
   requests or runtime errors.

Desktop viewport: 1440x1040. Mobile viewport: 390x844. The first camera fit clipped
frame corners; fitting projected corners with aspect/margins fixed that issue.
The final browser run passed after rebuilding, rerunning all 13 test groups and
exporting fresh files. Touch was emulated; no real phone/device was tested.

Evidence under `build/offline/qa/`: `generate-3d.cjs`,
`preview3d-summary-final.json`, `generate-3d-final.log`,
`preview3d-acceptance.cjs`, `preview3d-browser-results.json`,
`preview3d-browser.log`, `preview3d-desktop.png`, `preview3d-desktop-full.png`,
`preview3d-mobile.png`, `preview3d-mobile-full.png`, `preview3d-closeup.png`.
Initial diagnostic exports and screenshots were retained as generated QA data.

Limits: the scene uses the existing synthetic model and calibration. Frame/body
envelopes are schematic and the grid is a coordinate reference. Cable curves,
pulley contact geometry, sag, tension, collisions and actuator dynamics are not
rendered or simulated. Length readouts interpolate exported samples and are
labelled approximate. No simulated-stage artifact or hardware readiness claim
is made. No non-Windows build, other browser engine, real touch device, maximum
resource-size rendering benchmark or physical execution was tested.

The existing controller/root configuration/build files remain unchanged; root
README and docs/WORKFLOW_ASCII.md hashes still match the S2-C baseline below.

## S2-C baseline (prior to the 3D update)

S2-C was authorized by the user's instruction to proceed and completed on
2026-10-21. The standalone Windows Debug build, all 12 offline test groups,
33 CLI checks, an additional output-junction rejection check, and desktop/mobile
preview checks passed. This is synthetic offline software evidence. No physical
dynamics simulation or robot/print-head connection or operation was performed.

All work in this stage stays inside CDPR_3D_Printing_Slicer. The preexisting
repository README change and untracked docs/WORKFLOW_ASCII.md are preserved.
No existing controller, root configuration, recovery or build file is modified.

## Evidence and limits

| Check | Current evidence |
| --- | --- |
| Manual source/contract review | Performed; supplemented by the bounded checks below |
| JSON syntax | Configuration, job schema and synthetic model parse in read-only checks |
| Schema references | All 65 local definition references resolve |
| Source/target inventory | Six S2-B implementation files present inside the project |
| Static text inventory | 24 project files checked; 10 CMake source paths and 23 public function definitions found; shared inputs exist |
| Build/link compatibility | MSVC 19.42, x64 Debug, C++14: passed; final build has no warnings |
| Offline test suite | One CTest entry, all 12 Windows groups passed; 1.14 s final run |
| Generated HTML and visual layout | Installed Edge with Playwright: desktop/mobile passed after wrapping fix |
| Geometry/trajectory results | All four CLI stages accepted synthetic data; missing/invalid inputs rejected |
| Hardware/physical safety | Not evaluated; no physical interface accessed |

The CLI and path-based model loader target Windows. Nine test groups are pure
math/import/IO/preview-source checks; three further groups use Windows generated
model/manifest files. The numerical source uses C++14 and existing Eigen.
Non-Windows file-model loading returns an unsupported diagnostic, and those
three Windows test groups are not compiled there. Non-Windows behavior was not
built or run in this stage.

## S2-B scope delivered in source

The loader reads explicit geometry, rail state and optional calibration/limits
manifests without constructing Robot. It reproduces the effective legacy scale
selection and numeric conversions, binds exact source digests, and checks a
decoded snapshot consistency digest before planning.

Rigid transforms preserve the original nozzle path and derive world-frame EE
poses using the actual Rx*Ry*Rz convention. Domain/range checks wrap the original
ComputeCableLengths implementation. Spatial checks inspect pulley geometry,
length/count limits and the configured XYZ envelope. They do not solve tension,
collision or robot singularity.

The planner preserves source operations, original endpoints and signed E, uses
a stop at every segment, applies analytic quintic path/filament timing limits,
and slows the profile when sampled cable finite differences exceed caps.
Cable speed/acceleration stay not_evaluated because continuous extrema are
uncertified. Collision, tension, singularity, feedback, thermal protection,
extrusion synchronization and stop interlocks remain unevaluated.

The artifact reader/writer now supports imported, transformed,
geometrically_checked and timed. It checks transform consistency, explicit
quintic/event interpolation, knots, endpoint rest states and relative-time epoch
order. simulated remains unsupported. hardware_ready is always false.

The preview writer returns standalone HTML with projections, frame selection,
source-operation selection and recorded check status. Data is escaped before
embedding and displayed as text. Generated imported/timed previews were rendered
and interacted with in S2-C; page logs contained no warnings or errors.

CLI stages are explicit; default import does not open robot-model files.
Missing timing inputs can still permit a labelled geometric artifact when
registration, model identity, spatial sampling and XYZ envelope are known.
Failed/indeterminate planning returns no usable partial job. Every output is a
new file under this checkout's out directory.

## Compatibility findings preserved

The existing Robot loader calls model.value("cableMotorScale",509295). In the
repository's nlohmann implementation the integer default fixes the result type,
so a fractional common scale is truncated before storage in double. Individual
scales are used only when useIndividualCableScale is true, and then converted
through float. endEffVelLmt and rough XYZ bounds also use float storage.

The offline snapshot deliberately matches those effective values. No fix to
existing Robot.cpp is authorized or applied. Any later change to those
controller semantics needs separate review and invalidates associated plans.

Current/starting rail offsets must be explicit. No controlFreq, recovery-file,
zero-rail, identity-transform, motion-limit or hardware-capability default is
used as proof of feasibility.

## Executed offline tests

Every group below reported PASS in `build/offline/Testing/Temporary/LastTest.log`.

| Group | Acceptance/rejection evidence |
| --- | --- |
| Modal fixture | Units, G90/G91/M82/M83, G92, E recovery, inert events, operation/source order |
| Unit modes/extrusion | Modal F across units, E-only rates/prime/recovery and adjacent E words |
| Rejection cases | Unsupported/malformed commands, unknown state, no partial import |
| Source envelopes | XOR numbering/checksum, BOM/CRLF/UTF-8, control bytes and bounds |
| Digests/identity | SHA-256 known answers, state-sensitive import ID and stale source digest |
| Job round trip/tampering | Strict keys/types/enums/payloads/indices/spans/readiness and nonfinite data |
| Configuration/fixture integrity | Unknown values retained; strict import options; synthetic geometry |
| Coordinate conventions | Noncommuting frame/nozzle offsets, legacy XYZ Euler order and gimbal-lock rejection |
| Preview escaping | HTML/script delimiters remain inert JSON; status notice is present |
| Model provenance/legacy scales (Windows) | Snapshot mutations, explicit rail state/manifests, common integer and individual/path-speed float behavior |
| Planning/artifact round trip (Windows) | E/endpoints/source order, analytic path/filament caps, wait epochs, strict timed serialization |
| Geometry/planning rejection (Windows) | Outside envelope, missing cadence/cable limits, unmanifested overrides, sample exhaustion, degenerate pulleys and tighter cable retiming |

Tests confirmed the modal import fixture's six movements and 15 total operations.
Its G92 X0 at physical X10 mm makes the later absolute X50
command end at physical X60 mm. Final accumulated commanded filament is 2.5 mm.
The separate CLI assertions confirmed these values and the original source SHA-256.

The model fixture now includes explicit synthetic scales and an XYZ envelope.
Generated test manifests use synthetic provenance and known test transforms.
They must never be used as real robot calibration. Tests also construct minimal
legacy-shaped input solely to exercise loader conversions; no Robot is created.

## S2-B file-by-file checklist

All paths here are relative to CDPR_3D_Printing_Slicer/.

| Action ID | New file | Responsibility |
| --- | --- | --- |
| N12 | `src/geometry/CoordinateTransforms.cpp` | Registered transforms, legacy Euler conversion and transformed artifact |
| N13 | `src/cdpr/RobotModelLoader.cpp` | Explicit read-only geometry/manifests, numeric compatibility, source/snapshot identity |
| N14 | `src/cdpr/KinematicsAdapter.cpp` | Domain/length/count guards around shared existing solver |
| N15 | `src/planning/WorkspaceValidator.cpp` | Spatial checks, report and geometric artifact |
| N16 | `src/planning/TrajectoryPlanner.cpp` | Quintic timing, E rates, sampled cable slowdown and epochs |
| N19 | `src/visualization/PreviewWriter.cpp` | Bounded standalone interactive HTML writer |

Existing slicer files updated for this stage:

| File | Change |
| --- | --- |
| `CMakeLists.txt` | Existing Eigen geometry headers, planning library, shared math dependency and test-data path declarations |
| `include/cdpr_slicer/Planner.h` | Model/provenance binding, strict configuration helpers, stage APIs and resource bounds |
| `include/cdpr_slicer/Toolpath.h` | Explicit interpolation string in timing objects |
| `src/io/PrintJobIO.cpp` | Advanced stage IO, strict frame/limit/options decoding and timing consistency |
| `src/cli/PlanMain.cpp` | Explicit planning stages, selected model/manifests and optional HTML export |
| `schemas/print-job.schema.json` | Registered advanced stages and explicit motion/event timing contract |
| `config/planner.example.json` | Explicit sample resource caps; unknown machine data remains null |
| `tests/OfflineTests.cpp` | Five added groups and generated-file regression source |
| `tests/fixtures/robot-model.json` | Explicit synthetic scales/envelope for model loading |
| `README.md` | Current source status, CLI and remaining approval boundary |
| `docs/architecture.md` | Manifest, identity, transformation, sampling and timing contracts |
| `docs/gcode-dialect.md` | Status clarification; restricted importer semantics retained |
| `docs/validation.md` | This evidence/checklist/next-stage record |

The original eight S2-A additions remain: parser, IO, CLI, test source, modal/
rejected/model fixtures and this validation document. .gitignore, Execution.h,
modal.gcode and rejected.gcode remain unchanged in S2-B. G-code parser
implementation remains unchanged. No dependencies are installed or copied.

## S2-C environment and commands

The repository/project/build ancestors were checked as ordinary local directories
before generation. All new source changes, profiles, manifests, artifacts,
browser profile, QA scripts, screenshots and logs are inside this slicer project.
No dependency was installed, downloaded or copied. Build and out directories and
local profiles remain ignored by Git.

Toolchain: CMake/CTest 3.31.0; Visual Studio 2022 Community; MSVC 19.42.34433.0;
Windows SDK 10.0.22621.0; x64 Debug; explicit C++14. Existing Eigen 3.3.7,
nlohmann headers and the original CableKinematics.cpp were used in place.

Executed from the repository root, all final commands exited zero:

~~~powershell
cmake -S .\CDPR_3D_Printing_Slicer -B .\CDPR_3D_Printing_Slicer\build\offline -G "Visual Studio 17 2022" -A x64 -DBUILD_TESTING=ON -DCDPR_BUILD_HARDWARE_HOST=OFF -DCMAKE_CXX_STANDARD=14
cmake --build .\CDPR_3D_Printing_Slicer\build\offline --config Debug --parallel 1 -- /nodeReuse:false
ctest --test-dir .\CDPR_3D_Printing_Slicer\build\offline -C Debug --output-on-failure
~~~

The first parallel MSBuild run stalled after Checking Build System and was
interrupted. Serial MSBuild with node reuse disabled completed. The first serial
build reported C4456 for a shadowed local `end`; renaming that local removed the
warning. This finding did not require any existing controller change.

Retained compiler/test evidence:

- `build/offline/CMakeFiles/CMakeConfigureLog.yaml`
- `build/offline/build-first.log` (interrupted parallel run)
- `build/offline/build-serial.log` (original warning)
- `build/offline/build-final.log` (successful final build)
- `build/offline/ctest-final.log` and `Testing/Temporary/LastTest.log`
- `build/offline/test-data/case-*.json` generated with CREATE_NEW and retained.

## CLI acceptance: 33 checks passed, plus junction rejection

The disposable driver is `build/offline/qa/cli-acceptance.cjs`. Full invocations,
expected/actual exit codes and diagnostics are in `qa/cli-results.json` and
`qa/cli.log`; `qa/cli-summary.json` records selected paths. QA uses the already
bundled Node runtime. Generated profiles/manifests use the prefix `s2c-38148` and
explicit SYNTHETIC-ONLY provenance. No RobotConfig or real calibration is used.

| Check | Observed result |
| --- | --- |
| Help, import, transformed, geometric, timed, repeated timed | Exit 0; all four artifact stages also passed --inspect-job |
| Modal values | 6 motions, 15 operations, final X=0.06 m, accumulated filament=2.5 mm |
| Original data | Source hash, nozzle endpoints, signed E and operation/source order preserved |
| Import isolation | Succeeds with deliberately missing model path; no model load required |
| Wait epochs | Two temperature waits create later epochs 1 and 2; inert data only |
| Unknown initial state and unsupported G28 | Exit 2; no output |
| Missing cadence, rails, calibration, limits manifest or cable-speed cap | Exit 2; no output |
| Changed transform/limit, stale model hash, sample exhaustion, outside workspace | Exit 2; no output |
| Geometry without cadence | Accepted as geometrically_checked; no timing claim |
| Outside output path, traversal, reserved name, dot/space/ADS filename, duplicate flags | Exit 3; no output |
| Existing JSON | Exit 3; existing file SHA-256 unchanged |
| Existing HTML | Exit 3; existing HTML SHA-256 unchanged; newly published JSON retained and reported |
| Tampered hardware_ready | Strict reader rejected with exit 2 |
| Repeated timing | Exact serialized bytes and content ID identical; saved-geometric timing equivalence also passed in offline suite |

All generated artifacts retain hardware_ready=false. The timed artifact records
8 passed structural/sampled checks and 9 unevaluated checks. Spatial validation
used 29 points; timing diagnostics report 366 temporal knots. These counts do
not establish continuous feasibility.

The separate `qa/output-reparse.ps1` temporarily held only this run's verified
generated output folder inside the project, installed a junction to an empty
project-local disposable directory, and invoked the CLI. It rejected the
junction with exit 3; the target stayed empty. The script removed only the
verified junction, restored the original folder and verified all nine original
output hashes. Evidence: `qa/reparse-results.json`, `qa/reparse-cli.log`.
The original wrapper inherited the expected child exit 3 despite this passing
result; the saved script now explicitly returns zero on successful assertions.

Junction creation and Edge startup were denied/failed inside the execution
sandbox. The same bounded, authorized checks succeeded through reviewed sandbox
escalation. A denied preliminary junction attempt restored the output folder
in its finally block before retrying.

## Browser acceptance and layout fix

Flow: generated preview loads -> switch projection/frame -> select motion or
inert event -> geometry, source operation, timing and check status render.

Browser plugin not available. The installed Playwright package 1.62.1 drove
installed Edge 155.0.4283.62 in headless mode with a separate project-local
profile. The CLI skill wrapper would fetch another package, so the existing
Playwright API was used with no installation. The local file URLs are
`out/s2c-38148-timed-fixed.html` and `out/s2c-38148-imported-fixed.html`.

| Required check | Result |
| --- | --- |
| Page identity | PASS: expected file URL and CDPR toolpath preview title |
| Nonblank and no framework overlay | PASS: summary, controls, canvas, legend and 17 recorded checks |
| Console health | PASS: zero page warnings, console errors or page exceptions |
| Network | PASS: page requests were only the two local HTML navigations |
| Projection/frame controls | PASS: XY/XZ/YZ changed canvas; available EE frame selected |
| Operation selection | PASS: retract/E-only, mixed recover/deposit, wait, dwell, fan and end-job records; physical source lines and timing epochs retained |
| Import-only state | PASS: world/EE option disabled; only two checks passed |
| Desktop | PASS: 1440x1000, full-page screenshot visually inspected |
| Mobile | PASS: 390x844, full-page screenshot visually inspected; scroll width 390 px |

Before the fix, the long job ID made a 390 px viewport scroll to 515 px.
`#summary { overflow-wrap:anywhere }` fixed the overflow. The final build and
all 12 test groups were rerun, and new preview exports matched the original
JSON bytes exactly. The same browser interactions then passed on both sizes.

Screenshots and checks are retained under `build/offline/qa/`:
`first-mobile.png` (failure), `final-desktop.png`, `final-mobile.png`,
`final-desktop-viewport.png`, `final-mobile-viewport.png`, `final-desktop-dom.txt`,
`final-browser-results.json`, `browser-final.log`.
The driver is `preview-acceptance.cjs`; `refresh-previews.cjs` exported the fixed
HTML without replacing original artifacts. Test scripts/results are disposable
generated acceptance evidence and are not added to production targets.

## S2-C file-by-file record and remaining boundary

| File/path | Approved S2-C action completed |
| --- | --- |
| `src/io/PrintJobIO.cpp` | Rename shadowed local; no validation behavior change |
| `src/visualization/PreviewWriter.cpp` | Wrap summary job IDs to fit narrow screens |
| `README.md` | Record build/test/preview status and generated synthetic data |
| `docs/architecture.md` | Update status and approval boundary |
| `docs/gcode-dialect.md` | Link executed validation |
| `docs/validation.md` | Evidence, findings, commands, paths and remaining limits |
| `build/offline/**` | Standalone build, tests, QA scripts/logs/screenshots and isolated browser profile |
| `config/s2c-38148-*.local.json` | Synthetic import/timing profiles, calibration/limits and negative cases |
| `out/s2c-38148-*` | New JSON/HTML artifacts; no prior artifact replaced |
| Other slicer source/config/schema/fixtures | Preserved |
| Existing repository/controller/build/configuration files | Preserved; no modification approved or made |

Root README's preexisting change and docs/WORKFLOW_ASCII.md were preserved;
their SHA-256 values remained BA83EE5F4EC29F6906C15D892241386B5D1518AA902FF70B214C3881E6AEB3C5
and 0D9A26C24C338321575740AD167022E29518F1400B3F7FA5FC6909F1DE95C07E.
Git still reports no tracked change outside the project except that preexisting
README edit. Generated data is ignored; all 24 project source/document files
remain untracked pending any separate version-control instruction.
The root diff whitespace check reports trailing whitespace in the preexisting
README edit at line 77; it was preserved. The slicer text checks passed.

Limits: only Windows x64 Debug and the two stated viewport sizes in Edge were
tested. No independent JSON Schema validator was installed; the strict C++
reader/semantic invariants and round-trip tests provided interchange checks.
Actual existing repository ancestors were not replaced with junctions for a
negative test; their rejection code was inspected and ordinary ancestors were
verified before execution. Output-directory junction rejection was exercised.
Concurrent directory substitution races and other filesystems were not tested.

Continuous cable extrema, collision, tension, singularity, physical feedback,
thermal protection, extrusion synchronization and stop interlocks remain
unevaluated. No vendor library/Robot session or device driver was used. New
dependencies, controller/print-head implementation, dynamics simulation and
every physical connection or operation still require separate explicit approval.

## Primary references

The SHA-256 source follows
[NIST FIPS 180-4](https://csrc.nist.gov/pubs/fips/180-4/upd1/final).
The abc/56-byte/million-a expected digests are published in
[NIST NSRL test data](https://www.nist.gov/itl/ai/ai-standards-and-guidelines-group/nsrl-test-data).
Known-answer tests passed in S2-C; no cryptographic certification is claimed.

Handle publication refuses replacement as documented by
[Microsoft FILE_RENAME_INFO](https://learn.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-file_rename_info)
and uses
[SetFileInformationByHandle](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-setfileinformationbyhandle).
Mathematical design references and explicit engineering limits are recorded in
the architecture document.
