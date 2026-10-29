# Architecture and contract version 1

Status: S2-C, the 3D kinematic preview, sampled mechanics and the external STL workflow are validated on Windows/MSVC with C++14. Build,
offline tests, synthetic CLI stages and browser previews passed; see
[validation](validation.md) for evidence and limits. Controller and print-head
sections below remain future integration contracts. No physical simulation or
robot/print-head connection was performed. The offline mechanics addition
screens sampled contacts and solves static loads under explicit inputs;
see [mechanics](mechanics.md) for its narrower scope and the current AB1 findings.

## Boundaries

~~~text
STL millimetres -> separate PrusaSlicer 2.9.6 process + explicit export profile
        |
Reviewed text G-code
        |
offline import -> normalized nozzle operations
        |          signed E, thermal/fan data, dwells/barriers
        v
registered bed/nozzle transforms -> world/EE poses
        |
guarded existing ComputeCableLengths -> spatial length/count/envelope checks
        |
quintic segment timing + sampled cable retiming -> JSON / standalone preview
        |
separately approved future controller host
        |                          |
existing Robot/CableController   print-head port
        |                          |
existing motor interfaces       extrusion / thermal / fan controller
~~~

All new source/configuration/test/documentation stays in this project. Existing
JSON/Eigen headers and the existing CableKinematics.cpp are referenced in place.
No existing kinematics solver is copied. The adapter mirrors intermediate
domain checks because the original solver does not expose them; all returned
cable lengths come from the shared function.

The offline targets never construct Robot or link COM, ADS, Teknic or print-head
drivers. Windows disk-file APIs serve explicit model/input/output files only.
The current robot application and its build remain independent.

The STL entry point is `tools/Invoke-CdprSlice.ps1`. It snapshots the mesh and
reviewed INI, uses a private data/temp directory and direct Windows argument
quoting, and invokes the existing CLI at the timed stage. No slicing library
is linked into the C++14 application. It accepts STL only; 3MF projects and
their inherited profiles are outside this phase. It checks the engine version
and producer execution controls before slicing. Strict import is the final
command compatibility boundary: the adapter never strips generated commands.
The separate receipt binds exact mesh/profile/engine/planner/configuration
and output bytes. It supplements the existing job/model identities without
changing the interchange schema or validation claims. See
[stl-workflow.md](stl-workflow.md) for placement, limits and publication behavior.

Layer inspection groups horizontal depositing segments by bed-frame Z, rounded
to nanometres for display grouping. Selecting a group pauses and positions the
robot at that group's last motion; only the build up to that height is drawn.
Travel visibility changes rendering alone. Layer selection does not change the
job, time interpolation, extrusion accounting, cable samples or readiness flags.

The machine-profile preparation tool references an explicitly selected existing
RobotConfig in place, pins its raw-byte hash and writes incomplete planner,
calibration and limit manifests plus a measurement checklist under
`config/machines/<name>/`. It records raw configuration observations without
duplicating the native geometry decoder or kinematics. Measurements stay null
and registration unverified. The strict loader/planner remain the validation
boundary; preparation never marks a real-machine job ready. The selected target
is AB1 at `bin/RobotConfig.json`, updated from user-supplied commit `f1ab67c`.
The root and bin AB1 configuration copies were explicitly updated in the
configuration-import phase; offline code still never constructs Robot or uses
their online/device flags. See [machine-profile.md](machine-profile.md).

## Units, frames and rotations

| Quantity | Representation |
| --- | --- |
| Cartesian position, cable length, rail offset | meters |
| Internal orientation | unit quaternion object x,y,z,w |
| Legacy Robot pose | XYZ meters, roll/pitch/yaw radians |
| Filament position/delta/rate | mm / mm / mm per second |
| Time | seconds relative to an execution epoch |
| Temperature, fan | degrees Celsius, fraction [0,1] |
| Motor count | checked signed int32, truncating length times effective scale |

Quaternion inputs must be finite, components within [-1,1], and squared norm
within 1e-10 of one. Only roundoff in an already valid unit input is normalized.
No arbitrary quaternion is silently repaired.

For bed_from_nozzle supplied by the importer:

~~~text
world_from_end_effector =
    world_from_bed * bed_from_nozzle * inverse(end_effector_from_nozzle)
~~~

A transform maps child-frame column vectors into its parent. Original nozzle
geometry stays in the named bed frame; derived EE geometry is in the world
frame. Every exported derived endpoint is recomputed/checked against these
transforms by the artifact validator.

The legacy rotation is `Rx(roll) * Ry(pitch) * Rz(yaw)`. Conversion uses
pitch=asin(R02), roll=atan2(-R12,R22), yaw=atan2(-R01,R00), checks reconstruction,
and rejects cos(pitch) magnitude <=1e-8. Fixed orientation means the nozzle offset
is constant along each straight path and EE paths are also straight. Fixed rails
are explicit input. S2-B supports eight cables, four rails and unit +/-Z pulley
axes, matching the inspected installation/shared geometry convention.

Imported bed/nozzle frame names must match the calibration names. The current
G-code importer names them print_bed/nozzle_tip. Unknown transforms remain null,
not identity. A calibration status of verified records supplied provenance;
it is not independent metrology or permission to arm hardware.

## Model loading and controller compatibility

LoadRobotModel reads normalized absolute local file paths on Windows, max 4 MiB
per file. CLI configuration-relative paths are normalized before this call.
Only regular disk files are accepted; device/UNC/network/alternate-stream paths,
reserved port names and final-file reparse points are rejected. No recovery
reader, motor-counter calibration or robot connection is invoked.

The legacy loader selects these geometry fields: cableMotorNum, railMotorNum,
pulleyRadius, frameAttachments, endEffectorAttachments, pulleyZdir,
endEffectorPosLimit, cableMotorScale/useIndividualCableScale and, when enabled,
cableMotorScaleIndividual. Other legacy fields are not executed or applied.
Even isOnline=true does not construct or connect a Robot.

Effective cable scales follow the inspected Robot loader: use the common scale
when useIndividualCableScale is absent/false; when true, require all eight
individual entries and reproduce its float-to-double conversion. Do not choose
individual entries merely because the array exists. The common scale is also
read by Robot via model.value("cableMotorScale",509295): nlohmann's integer
default type truncates a fractional configured value before assignment to
double. The offline loader reproduces that effective value after checking the
integer conversion range, rather than changing the existing controller.

The six legacy bound values are XYZ min/max pairs in x,y,z,roll,pitch,yaw key
order, including the controller's float conversion. They are rough Cartesian
bounds, not six independent pose limits and not a tension/collision workspace.
An explicitly supplied endEffVelLmt is a path-speed bound, with the controller's
float storage reproduced; no absent value or
controlFreq default is adopted as a printing limit or cadence.

Current and starting four-rail offsets must be supplied explicitly. Shared
kinematics applies its existing rail correction to the first four cables.
No rail value is inferred from recovery files or default-initialized storage.

The synthetic geometry fixture uses its own explicit wrapper, counts, axes,
scales, workspace and rails. Its request rails must match the fixture. It is
not a RobotConfig and contains no ports/addresses. The path-based loader is
Windows-only in this stage; the numerical APIs use C++14 and bundled Eigen.

## Manifest contracts

The reusable S2-C profiles and manifests describe the synthetic fixture. The
later local AB1 mechanics scenario separately records user-specified geometry,
zero rails and offline bed/nozzle placement assumptions. It leaves missing
operating limits null and does not establish measured registration or live
robot state; see [mechanics](mechanics.md).

A calibration manifest has exactly schema_version=1, frames, provenance.
frames has the same keys as the planner's frames object and must contain both
explicit transforms with status=verified. provenance has exactly calibration_id
and method, both nonempty strings of at most 256 UTF-8 bytes.

Incomplete template, intentionally rejected until measured transforms and
reviewed provenance are supplied:

~~~json
{
  "schema_version": 1,
  "frames": {
    "world_frame_id": "cdpr_world",
    "bed_frame_id": "print_bed",
    "end_effector_frame_id": "end_effector",
    "nozzle_frame_id": "nozzle_tip",
    "world_from_bed": null,
    "end_effector_from_nozzle": null,
    "status": "unverified"
  },
  "provenance": {
    "calibration_id": "REPLACE_WITH_REVIEWED_ID",
    "method": "REPLACE_WITH_CALIBRATION_PROVENANCE"
  }
}
~~~

Each transform object has translation_m [x,y,z] and rotation_xyzw {x,y,z,w}.
No pickup/ground offset, camera output or default quaternion establishes nozzle
registration.

A motion-limits manifest has exactly:

~~~json
{
  "schema_version": 1,
  "geometry_contract": "cu_brick_fixed_rails_v1",
  "limits": {
    "workspace": null,
    "maximum_path_speed_m_per_s": null,
    "maximum_path_acceleration_m_per_s2": null,
    "minimum_cable_length_m": null,
    "maximum_cable_length_m": null,
    "maximum_cable_speed_m_per_s": null,
    "maximum_cable_acceleration_m_per_s2": null,
    "minimum_motor_count": null,
    "maximum_motor_count": null,
    "maximum_filament_rate_mm_per_s": null,
    "maximum_volumetric_flow_mm3_per_s": null
  }
}
~~~

These nulls intentionally leave timing/feasibility unknown. Workspace, when
supplied, is {minimum_m:[x,y,z], maximum_m:[x,y,z]}. Cable/count arrays have
exactly eight entries. Positive speeds/accelerations/flow and ordered bounds
are required. Count limits require int32 integer literals. A manifest may
tighten controller XYZ/speed bounds, never widen them; missing workspace/speed
entries retain explicit controller bounds where available.

A missing calibration or limits manifest permits only a provisional model
snapshot. BindPlanningOptions requires both manifests before a transformed
artifact is produced. A present but incomplete limits manifest can support a
labelled geometric artifact while timing remains indeterminate.

Null configuration limit entries inherit model/manifest values. Non-null
entries must agree exactly with the bound values. Both null configuration
transforms with unverified status inherit manifest transforms only when frame
names agree; supplied transforms must match the manifest. No silent override
changes registered geometry or mechanical limits.

## Identity and stale data

Identity fields are lowercase SHA-256 digests of exact robot configuration,
calibration manifest and motion-limits manifest bytes. Those source files cover
geometry, scale selection, registration provenance and configured limits.

rail_state_sha256 hashes ASCII `cdpr-rail-state-v1` plus LF, then eight
IEEE-754 binary64 values in big-endian byte order: current r0..r3, then starting
s0..s3. Negative zero is canonicalized to positive zero; nonfinite values and
non-binary64 platforms are rejected.

The model also contains a snapshot_sha256 digest over canonical decoded
geometry, effective scales, rails, frames, limits, source identities and
provenance-presence flags. Validation/binding rejects a snapshot changed after
loading. This catches stale application data, not a malicious caller capable
of forging arbitrary process memory. No digest is a signature or hardware lease.

Source SHA-256 covers exact G-code bytes including BOM/line endings. The import
ID includes its canonical initial-state info trace. Transformed/geometric/timed
IDs hash the serialized candidate with job_id temporarily set to content-v1,
then use transform-/geometry-/plan- plus the digest. Equal output content has
equal identity across a saved geometric stage. Controller identity/session
checks remain a separate future boundary.

## Artifact and geometry checks

The strict decoder requires defined fields, rejects unknown/duplicate keys and
invalid types/enums/nonfinite values, checks all unit quaternions, contiguous IDs,
physical source order, payload-kind agreement, G92/E continuity and span
partitions. Logical E may jump after G92; accumulated commanded filament cannot.
Commanded filament is not measured delivery. Retraction debt begins at zero for
this input's history, without asserting a physical priming state.

Imported jobs have syntax/modal checks passed. Transformed jobs additionally
pass transform/orientation consistency. Geometrically checked jobs add sampled
geometry and XYZ envelope checks; configured motor bounds may be passed at
sample points when both bounds are known. Timed jobs also pass internal timing
structure. All other checks remain not_evaluated.

Domain guards reject nonfinite/degenerate pulley planes, radial norms <=1e-12,
invalid tangent/acos domains, nonpositive lengths and count overflow before
using results from the existing solver. Cable bounds and optional motor bounds
are checked at the chosen spatial samples. The adapter does not clamp invalid
geometry into a plausible cable command.

The configured path_tolerance_m is the maximum spatial sampling step in this
linear prototype. It is not a certified nonlinear chord-error tolerance.
Axis-aligned XYZ envelopes contain each fixed-orientation straight EE segment
when its endpoints are inside. Cable length/count/domain checks are sampled.
Collision, structural singularity and cable tension are not inferred from them.

ValidateWorkspace returns a report; CreateGeometricPlan also returns a staged
job. Reopening a geometric job for timing recomputes geometry against the
selected snapshot instead of trusting the recorded passed flags. Deserializing
an artifact alone checks consistency, not source authenticity or actual workspace
feasibility.

## Quintic timing and extrusion

Every original motion uses quintic_progress_v1. For u=t/T:

~~~text
s = 10u^3 - 15u^4 + 6u^5
ds/dt = 30u^2(1-u)^2 / T
peak(ds/dt) = 1.875 / T
peak(abs(d2s/dt2)) = (10 sqrt(3) / 3) / T^2
~~~

Interpolate the original straight position by s. Filament offset is delta_E*s;
filament rate is delta_E*ds/dt. Ordered extrusion purposes remain tied to their
original progress spans, including mixed recovery/deposition within one line.
No E or endpoint is dropped. Motion starts/ends with zero speed and acceleration.

Initial duration respects requested F, applicable explicit path speed/
acceleration, filament rate and optional positive volume-flow cap. A configured
volume-flow cap requires filament diameter for positive E. Positive/negative E
retain sign. Extruder acceleration and actual delivered flow remain unverified.

Profiles include both endpoints and at least five samples. Interval count meets
the explicit sample_period_s and spatial step using the 1.875 progress-rate
bound. Neither value is inferred from the existing 50 ms generator/playback
behavior.

Cable lengths at temporal knots feed secant speed estimates and finite-
difference acceleration estimates, including start/stop. If sampled caps are
exceeded, duration is multiplied by 1.05 times max(speed ratio, sqrt(acceleration
ratio)) and resampled, at most 12 attempts. Cable speed/acceleration status stays
not_evaluated: these estimates do not certify continuous extrema. Timing is an
offline planning model, not a validated vendor interpolation mode.

Each segment stops; there is no continuous corner blending/lookahead. This can
affect print quality and throughput. The old trajectory generators/RunCableTraj
are not used as an executor for this format, and their endpoint/cadence/queue
limitations remain unchanged.

Events use event_v1 with no motion profile. Dwell duration is preserved; other
events are instantaneous in relative plan time. Equal timestamps retain ID
order. Heat-only versus heat-or-cool mode is preserved; timed waits require
explicit positive tolerance and timeout <=86400 s. A wait ends its epoch.
The next operation starts at time zero in the next epoch only after a future
executor confirms the wait. No heater action occurs during planning.

## Resource and export bounds

G-code/JSON limits from S2-A remain: 64 MiB input/output, 4096-byte source lines,
1,000,000 operations including end_job, 10,000 total diagnostics, JSON depth 32,
indices <=2^53-1 and finite numeric values. String lengths use UTF-8 byte bounds.

S2-B caps each spatial pass and final temporal output at 200,000 samples and
each motion at 50,000 samples including endpoints; options may tighten those
bounds. One operation duration is <=86400 s; one epoch is <=604800 s. These
resource bounds are not machine capability limits. Exhaustion rejects the
requested result; a partial job is never usable.

The reader/writer supports imported/transformed/geometrically_checked/timed.
simulated remains unsupported. S2-B specifies interpolation on previously
unimplemented timing objects; valid imported schema-v1 jobs remain compatible.
The schema is structural; C++ semantic checks add cross-field invariants.
hardware_ready is always false.

Preview generation returns standalone HTML for at most 20,000 motion operations,
with no external scripts/assets/network access. It escapes embedded data and
uses textContent for source/diagnostics. The renderer lives in PreviewTemplate.html;
CMake bundles it into a generated header, splitting literals to satisfy MSVC.
Changes to the template automatically trigger reconfiguration before rebuilding.

CreatePreviewHtml(job) produces a 3D path view without robot geometry.
CreatePreviewHtml(job,model) additionally validates the snapshot seal, all four
identity digests and exact frame registration before binding robot geometry.
The CLI uses the model-bound overload for advanced stages. Print-job JSON and
the simulated-stage contract remain unchanged; this is visualization data only.

The inert robotdata block contains world frame anchors with current fixed-rail
Z offsets applied to the first four, local EE attachment points, pulley radius,
rail offsets, model digest, optional XYZ envelope and per-motion display samples.
Each motion gets min(33,floor(40000/motion_count)) samples, including endpoints;
with at most 20,000 motions the total is bounded by 40,000. The existing checked
C++ solver supplies all cable lengths. A rejected sample prevents preview export;
display samples do not promote a validation check or establish continuous safety.

The browser performs rigid transforms and interpolates these samples for labelled
approximate length readouts. It does not duplicate pulley inverse kinematics.
It draws the anchor envelope, EE attachment envelope, eight straight schematic
cable spans, nozzle offset, reference grid, optional XYZ envelope and selected
toolpath. Perspective and XY/XZ/YZ views share bed/world frame selection.
Camera fitting checks projected corners against viewport margins.

Timed playback advances local segment time using requestAnimationFrame and the
artifact's quintic progress law. Filament uses that same progress, including
stationary E-only moves. Instantaneous events preserve operation order; a
temperature wait pauses until an explicit Continue advances to the next epoch.
No heater completion is inferred. Plain untimed previews have manual scrubbing
only. An attached mechanics assessment enables geometry replay at an explicitly
illustrative 0.3 seconds per motion, preserving wait pauses without claiming
actuator timing. It does not write timing into the job.
Play/pause/reset, segment/operation seeking, speed, orbit/pan/zoom, keyboard
camera access and touch pinch are local visualization controls. No autoplay;
page hiding and a changed reduced-motion preference pause playback.

Output remains bounded to 64 MiB and data markers are substituted in reverse
template-offset order, so marker text inside user data cannot alter substitution.
The scene is commanded kinematic playback, not dynamics or evidence of execution.
A plain preview does not assess tension or contact. A mechanics-bound preview
adds declared component/obstacle geometry, contact markers and nearest-sample
static allocations. Pulley wrap, cable sag and physical execution remain absent.

## Separate mechanics assessment

`Mechanics.h` exposes checked free-span endpoints, `AssessMechanics`, report
validation and a model/report-bound preview overload. The existing guarded
kinematics adapter supplies its pulley-tangent intermediates; all cable lengths
still come from the shared `ComputeCableLengths` implementation referenced in
place. No root controller or original kinematics source is changed.

`MechanicsAssessment.cpp` validates an explicit physical JSON model pinned to
the robot-configuration digest. It screens finite-radius free cables against
each other, EE component boxes and declared world obstacles/optional bed plane.
Conservative transformed component AABBs screen body/environment contact.
It also finds one bounded static equilibrium allocation using cable directions,
attachment moments and a supplied moving mass/CoG/load. Working limits remain
unevaluated when either pretension or allowed maximum is unknown. A supplied
breaking strength is an ultimate ceiling, never a working-load rating.

Samples have a preflight budget of 20,000 total and 2,001 per motion. The
24 MiB report seals job/model/physical-input digests, stores sampled coverage
and individual results, and leaves the planning artifact immutable. The CLI
mode `--assess-job ... --config ... --physics ... --output ...` publishes it
separately, with optional standalone HTML. Exit zero means a completed report;
negative findings remain in its statuses. `simulated` remains unsupported.
The [mechanics guide](mechanics.md) defines the input contract, equations,
limitations, AB1 placement assumptions and reproduction commands.

CLI outputs are new JSON/optional HTML under this checkout's out directory,
using held ancestor handles, reparse rejection and handle-based rename with
replacement disabled. Outputs are separate publications: a later HTML failure
retains an already published JSON and reports its path. A crash may leave
unconsumed pending temporary files; none is implicitly accepted as a job.

## Future controller and print-head boundary

Execution.h still declares ports only. Capabilities default false; stationary,
brake and inhibit feedback remain unknown until observed. Submission acceptance
does not prove physical arrival. Future host integration must own a single
Robot session and use Robot/CableController/motor interfaces rather than bypass
them. No root controller modification is part of S2-B.

Queue and arm must be separate. Job/session/epoch/chunk identity, bounded queues,
duplicate/replay rejection, measured skew/clock behavior, following error and
feedback age need actual hardware capabilities and reviewed numerical limits.
A cooperative lease does not exclude an unrelated vendor tool or old executable.

Pause/cancel/fault stop must coordinate motion and extrusion, invalidate pending
epochs, retain uncertain resources for shutdown retries, and require
reinitialization after an uncertain stop. Independent extrusion inhibit,
watchdogs, thermal protection and emergency stopping remain hardware
prerequisites. A software stop or file flag does not implement those protections.

S2-C build/test/application/browser checks were explicitly approved and completed.
Dependencies, controller changes, print-head implementation and every physical
connection or operation require their own later approval.

## Primary references

Composition and quaternion conventions follow the
[Eigen geometry tutorial](https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialGeometry.html);
code uses the repository's existing Eigen 3.3.7 headers. The quintic boundary
conditions follow the point-to-point timing discussion in
[Modern Robotics, sections 9.1–9.2](https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-1-and-9-2-point-to-point-trajectories-part-2-of-2/).
The coefficients, analytic extrema and sampling/retiming policy above are the
explicit engineering choices for this offline prototype. Hash and Windows
publication references are recorded in the validation notes.
