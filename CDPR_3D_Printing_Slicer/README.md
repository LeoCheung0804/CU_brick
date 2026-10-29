# CDPR 3D Printing Slicer

A separate offline import and planning project inside the existing CU_brick
repository. All slicer files and future generated outputs stay in this folder.

## Current scope: STL slicing, offline planning and sampled mechanics

The [mechanics guide](docs/mechanics.md) now covers sampled collision screening
and quasi-static cable-load allocation, with a new AB1 500 mm cube view at
`out/ab1-cube-500mm-standoff-5mm-10mm-mechanics-20261029.html`.
Outer anchor spacing is now the user-corrected 5004 x 5004 x 2305 mm.
The current cube uses
10 mm first/subsequent layers (50 layers total), with a user-approved 12 mm
demo nozzle and bead width. It uses the user's approximate
70 kg, 400 x 400 x 580 mm 4040 frame, 2 mm cables and stated bed/nozzle heights.
The latest [shared AB1 proposal](config/ab1-mount-standoff-5mm/README.md) moves
the free-cable fixing centers 5 mm outward from the aluminum Y faces:
upper [X=+/-0.10, Y=+/-0.205, Z=+0.27] m and lower
[X=+/-0.18, Y=+/-0.205, Z=-0.27] m. The reference rail holes remain at
Y=+/-0.20 m. The proposed bracket/eyelet geometry and strength are unverified;
the profile is explicitly marked synthetic. The CoG is
centred and the nozzle extends 100 mm below the bottom (local Z=-0.39 m).
The Python profile and `pyCDPR/AB1.py` read the same geometry and physical
parameters. Saved routing is retained. All 1,406 samples have wrench rank 6
and zero detected contacts, with minimum modeled clearance of 4 mm.
The member boxes and collision settings are unchanged, with zero attachment
exclusion. The confirmed rail-face reference and its 16 mount contacts per
pose remain available separately. The largest returned allocation is about
1027 N. The corrected frame has
100 mm initial bed clearance. Working-tension and pretension limits remain unknown;
1,700 N is the supplied breaking strength, not an allowed working load.

This AB1 job is geometry-only. Its **Play geometry** control uses illustrative
timing and pauses at temperature waits. The assessment is a separate report;
the original job, recorded planning checks and hardware readiness do not change.
The proposed mounting layout passes 16 Python tests and eight artifact/browser groups.
The preceding 10 mm recipe's 28 STL workflow checks, Debug/Release native tests and
mechanics release's
CLI/browser checks are retained in [validation](docs/validation.md).
No new dependency or hardware driver was added.

`tools/Invoke-CdprSlice.ps1` now runs PrusaSlicer 2.9.6 as a separate process,
imports its unfiltered text G-code with the reviewed compatibility profile,
and exports a timed CDPR job, standalone 3D preview and provenance receipt.
The included 10 mm cube produces 40 layers. See the
[STL workflow guide](docs/stl-workflow.md) for the command and profile limits.

A [500 mm cube example](docs/large-cube-example.md) now includes the closed STL
and a completed 625-layer offline run using a separate coarse hollow-print
recipe. The wrapper accepts an explicit export profile and placement center;
its original 100 mm profile defaults remain unchanged.
The preceding example places the EE at the synthetic workspace's bottom centre,
starting at world (0,0,-0.5) m while preserving the existing nozzle offset.

The next setup phase now targets the user-selected **AB1** configuration at
`bin/RobotConfig.json`. `tools/New-CdprMachineProfile.ps1` prepares a model-pinned,
incomplete profile and measurement checklist. The current AB1 package is in
`config/machines/ab1-f1ab67c-20261023/`, pinned to the user-supplied measured
configuration from commit `f1ab67c`. See the [machine-profile guide](docs/machine-profile.md).
That template remains incomplete. The later mechanics example uses separately
labelled user-specified zero rails and offline placement assumptions; it does
not supply measured live state or missing operating limits.

Source now covers restricted G-code import, coordinate transforms, explicit
model/manifest loading, guarded reuse of the existing cable kinematics, sampled
workspace validation, segment timing, JSON interchange and an interactive HTML
preview writer with an interactive 3D CDPR scene. The Windows CLI exposes those stages. Offline test source is
included and passes on Windows with MSVC 19.42 and C++14.

The standalone Debug build and all 13 offline test groups pass. The 3D update
also passed nine browser acceptance groups covering playback, wait epochs,
camera controls, coordinate frames, reduced motion and mobile/touch behavior.
The preceding S2-C baseline passed 33 CLI checks and output-junction rejection.
Results and retained evidence are in [validation](docs/validation.md).
No dynamics simulation or hardware operation was performed.

The STL update also passes 24 external-engine workflow checks on Windows
PowerShell 5.1 and seven desktop/mobile browser checks. The official portable
engine is isolated under ignored `build/tools/`; the C++ planner has no new
library dependency.

The timed planner stops at every original G-code motion segment. Continuous
corner blending, controller execution, print-head drivers and hardware
synchronization remain future work. A planning artifact always has
`validation.hardware_ready: false`.

## Module boundaries

| File/module | Responsibility |
| --- | --- |
| `include/cdpr_slicer/Toolpath.h` | Units, operations, frames, timing and validation data |
| `include/cdpr_slicer/Planner.h` | Offline import/planning interfaces and explicit inputs |
| `include/cdpr_slicer/Execution.h` | Abstract execution ports; no device implementation |
| `include/cdpr_slicer/Mechanics.h` | Checked cable spans and separate model-bound mechanics/preview APIs |
| `src/gcode/GCodeParser.cpp` | Restricted modal G-code import and signed extrusion |
| `src/geometry/CoordinateTransforms.cpp` | Bed/nozzle to world/EE transforms; legacy XYZ Euler conversion |
| `src/cdpr/RobotModelLoader.cpp` | Read-only geometry/manifest loading, identity and snapshot consistency |
| `src/cdpr/KinematicsAdapter.cpp` | Domain/range guards around existing ComputeCableLengths |
| `src/planning/WorkspaceValidator.cpp` | Spatial sampling, XYZ envelope, cable length/count checks |
| `src/planning/TrajectoryPlanner.cpp` | Quintic segment timing, extrusion rates and sampled cable retiming |
| `src/planning/MechanicsAssessment.cpp` | Physical input validation, sampled capsule/component contact screening and bounded static equilibrium |
| `src/io/PrintJobIO.cpp` | Strict interchange/config decoding and structural validation |
| `src/visualization/PreviewWriter.cpp` | Model-bound export, shared cable samples and escaped standalone HTML |
| `src/visualization/PreviewTemplate.html` | Local 3D renderer, camera, playback, readouts and recorded checks |
| `src/visualization/PreviewTemplate.h.in` | Bundle the renderer into the C++ executable during configuration |
| `src/cli/PlanMain.cpp` | Windows file CLI and new-file output inside project out/ |
| `tools/Invoke-CdprSlice.ps1` | External STL slicer process, isolated working data, strict planner invocation and exact-byte receipt |
| `tools/New-CdprMachineProfile.ps1` | Model-pinned incomplete machine-profile templates and measurement checklist |
| `config/prusa-cdpr.ini` | Pinned engine's linear single-tool G-code export profile |
| `config/synthetic-*.json` | Complete, explicitly synthetic planner/frame/limit fixtures |
| `tests/OfflineTests.cpp` | Import, geometry, provenance, planning and preview regression source |
| `tests/MechanicsTests.cpp` | Static force/moment balance, contact cases, binding and missing-data/budget regressions |
| `tests/SliceWorkflow.Tests.ps1`, `tests/PreviewWorkflow.Tests.cjs` | Real STL pipeline and layer/travel/playback acceptance checks |
| `tests/fixtures/` | Synthetic input, never real calibration or a hardware configuration |

This mechanics phase leaves existing controller files, root build files,
configuration and recovery files unchanged. No Robot instance, controller, COM, ADS, Teknic or print-head
driver is linked. Existing JSON/Eigen headers and CableKinematics.cpp are
referenced in place. The approved STL phase downloaded only the official,
checksum-verified portable PrusaSlicer; it is not part of the planner build.

## CLI contract

The Windows CLI was built and exercised with synthetic data:

~~~text
cdpr-slicer-plan --input <gcode> --config <planner.json> [--stage <stage>]
                 [--output <name.json>] [--preview-output <name.html>]
cdpr-slicer-plan --inspect-job <job.json>
cdpr-slicer-plan --assess-job <job.json> --config <planner.json>
                 --physics <physical-model.json> --output <assessment.json>
                 [--preview-output <name.html>]
~~~

| Stage | Result and prerequisites |
| --- | --- |
| `imported` (default) | Modal geometry/events only; no model files opened |
| `transformed` | Complete model identity, calibration and fixed orientation/rails |
| `geometrically_checked` | Transformed path plus explicit spatial interval and XYZ envelope |
| `timed` | Geometry checks plus explicit cadence, applicable path/filament limits, eight cable speed/acceleration limits, and wait policies |

Only explicit regular local-disk files are accepted. Paths supplied to
`--config`/`--inspect-job` resolve against the process working directory.
The assessment mode's `--assess-job` and `--physics` do too.
Input/model/manifest/output paths in a configuration resolve against that
configuration's directory. The model API itself requires normalized absolute
local paths; its file-loading implementation is Windows-only.

Output is restricted to this checkout's `out/` directory. Filenames permit
letters, digits, hyphens and underscores with `.json` or `.html` as appropriate.
Existing files are never replaced. Output ancestors and output directory
cannot be reparse points. JSON and optional HTML are published separately; if
HTML publication fails after JSON succeeds, the created JSON is retained and
its path is reported. Synthetic S2-C artifacts are retained in `out/`.

## Configuration and model identity

`config/planner.example.json` remains intentionally incomplete. Initial nozzle
pose, filament datum, rails, transforms, limits, sampling and thermal policy
are not inferred from zero, identity transforms or the old control interval.
G92 establishes logical origins only; it cannot establish an unknown physical
starting pose or filament datum.

Planning reads an explicit robot/synthetic geometry file, an explicit current
and starting four-rail state, and separate calibration/limits manifests. S2-B
supports eight cables/four fixed rails and unit +/-Z pulley axes. It matches the
controller's scale-selection flag, common-scale integer truncation,
individual-scale/path-speed float conversion and legacy XYZ bound ordering.

Null configuration frame transforms may inherit a verified manifest registration
with matching frame names. Null limit entries inherit bound model/manifest
values; supplied values must match them. A manifest may tighten controller XYZ
or speed bounds. Geometry and limits cannot be silently overridden after
identity is computed. Model hashes and an internal snapshot digest detect
stale/changed data; they are not signatures or permission to move hardware.

Calibration manifest format, limit format, mathematical assumptions, resource
limits and unresolved checks are specified in [architecture](docs/architecture.md).
S2-C created explicitly synthetic local profiles and manifests under `config/`.
They are ignored development data, never machine calibration or operating limits.

## Using the 3D preview

Export with `--preview-output` as before. Advanced stages embed the selected
model's eight frame anchors, eight EE attachments, fixed rail offsets and XYZ
envelope. Import-only previews show the 3D path without inventing a robot model.
Existing HTML files are snapshots; export a new filename to use the new renderer.

- **Play / Pause / Reset:** follow timed segments with their original quintic
  progression. A temperature wait pauses; **Continue** advances into the next
  epoch without claiming heater feedback. Plain geometry-only previews offer
  manual scrubbing; an attached mechanics assessment also enables explicitly
  illustrative geometry replay.
- **Operation / segment sliders:** inspect an original operation and its local
  time fraction. Source commands, timing and E accounting remain available.
- **Camera:** drag to orbit, wheel/pinch to zoom, Shift-drag to pan. A focused
  canvas also supports arrow keys, +/- zoom and 0 to reset. Playback starts only
  on explicit user action and pauses when the page is hidden or reduced motion
  is enabled during playback.
- **Fit robot / Fit path:** inspect the whole frame or focus on deposited
  centerlines (the original path if there is no horizontal deposition).
  Reset view restores the initial camera orientation.
- **View / Coordinates:** choose perspective or XY/XZ/YZ and bed/nozzle or
  world/EE coordinates. The robot and its anchors are transformed consistently.
- **Inspect layer / Show travel moves:** pause at a selected deposition height,
  show the build up to that height and optionally hide dashed travel spans.
  Layers derive from horizontal depositing geometry, not comments. Playback,
  Reset and manual operation selection restore all layers.

Cable readouts interpolate bounded samples calculated through the existing
C++ cable solver. The drawn cables are straight schematic spans from frame
anchors to EE attachments. Frame/body envelopes and the bed grid are reference
geometry; no chassis/bed mesh is inferred. Pulley wrap, sag, tension, collision,
heating and physical execution are not simulated by a plain preview. An
assessment adds the declared EE components/obstacles, contact markers and
nearest-sample static load readouts; see [the scope and assumptions](docs/mechanics.md).
No plan stage, content ID,
recorded validation flag or hardware-ready flag changes when exporting the view.

## Timing and validation limits

Motion uses `quintic_progress_v1` with zero velocity/acceleration at both
endpoints, synchronized commanded E and retained source operations/spans.
Events use `event_v1`. Waits end their relative-time epoch; later times start at
zero in a new epoch. Temperature targets remain inert data.

Cartesian peak speed/acceleration and filament-rate bounds determine an initial
duration. Finite differences of shared cable lengths then guide bounded
retiming. Cable speed/acceleration checks stay `not_evaluated` because sampled
estimates do not certify continuous extrema. Collision, tension, robot
singularity, physical feedback, thermal protection and hardware synchronization
also remain unevaluated in the original planning artifact. Separate mechanics
reports record their narrower sampled contact/static-load results; unknown
working tensions remain unevaluated there too. Sampled geometry and rough XYZ bounds are not a
physical safety certificate.

The reader/writer supports imported, transformed, geometrically checked and
timed artifacts. `simulated` remains unsupported. Imported schema-v1 files stay
compatible; S2-B gives previously unimplemented timing an explicit interpolation
field. Every accepted artifact retains `hardware_ready: false`.

## Build and next approval

CMake declares `cdpr_slicer::import`, `cdpr_slicer::planning` and the Windows
`cdpr-slicer-plan` target. Planning links the original mathematical source,
which was compiled in place as a dependency of the standalone build. No vendor
library is linked. `CDPR_BUILD_HARDWARE_HOST=ON` fails explicitly.

`BUILD_TESTING` defaults OFF. When enabled during approved configuration, it
declares two CTest programs on Windows and a project-local build/test-data directory.
Both entries passed: 13 original groups plus five mechanics groups. The mathematical source
uses C++14; the path-based model tests and CLI target Windows.

See the [validation record and file checklist](docs/validation.md) for concrete
S2-C/STL commands, generated-file scope and remaining limits. Further dependency installation,
controller integration, physical simulation and every physical connection or
operation require separate approval.
