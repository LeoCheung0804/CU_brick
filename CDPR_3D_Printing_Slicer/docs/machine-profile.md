# AB1 machine profile preparation

The selected offline target is `bin/RobotConfig.json`, model **AB1**. On
2026-10-23 the user supplied measured configuration revision
`f1ab67c63d3a4ceb63c4c3b3a4a7540ed6752bab`. Its `RobotConfig.json` was imported
into both the root configuration and the selected bin configuration. The two
files now contain the same AB1 configuration. `bin/RobotConfig_YES.json` remains
the separate YES configuration.

The current package is `config/machines/ab1-f1ab67c-20261023/`:

| File | Fill or review |
| --- | --- |
| `planner.local.json` | Four current and four starting rail offsets, initial nozzle pose/filament datums, sampling and print-head policy |
| `calibration.local.json` | Bed-to-world and nozzle-to-EE registration, calibration identity/method and registration status |
| `limits.local.json` | Reviewed path/cable/motor-count and filament/flow operating limits |
| `setup.local.json` | Read-only record of the selected configuration hash, nominal geometry and six groups of missing inputs |
| `MEASUREMENTS.md` | Package-specific completion checklist |

The package references the original AB1 configuration in place and pins its
exact SHA-256:

```text
3b39535a98935345005c4b7955706db0918b72b06faf0da4c7868cac2ee59f5e
```

The requested import changes only the root and bin AB1 configuration copies
outside the slicer project. Logs, demo CSVs and recovery state from the commit
were not imported. The original measurement package `ab1-20261023/` is retained
as a historical snapshot; its old model digest will reject the updated source.
Use the new package above. Device flags, ports and addresses are never used by
the preparation tool. It does not construct Robot, load a hardware driver,
restore robot position, slice a model or execute motion.

## Configured geometry and bounds

AB1 has eight cables, four rails and a 45 mm pulley radius. Its nominal frame
anchor envelope spans X/Y = 0 to 1.85 m and Z = -1.8 to 0 m, before applying
the explicitly supplied rail offsets. These are saved geometry values, not a
measurement of the current robot state.

The existing controller uses the unusual pairs in `endEffectorPosLimit` as:

| Axis | JSON keys | Configured interval, m |
| --- | --- | --- |
| X | x / y | -0.1 to 2 |
| Y | z / roll | -0.02 to 13.8 |
| Z | pitch / yaw | -3.75 to 0.3 |

The configured path-speed bound is 0.2 m/s. In particular, the Y/Z bounds
extend beyond the nominal anchor envelope; review a tighter print workspace
against the actual setup. These broad saved bounds alone do not establish
cable tension, collision clearance, reachability or a printable workspace.
The slicer manifest can tighten these bounds but cannot widen them. The tool
does not automatically correct or reinterpret the existing configuration.

The configured home pose is a preset, not proof of the current physical pose.
The preparation tool does not turn it into an initial nozzle pose or infer
rail positions from it, recovery files, logs or default zeros.

Other imported settings include a common cable scale of 815027.0614 counts/m,
rail scale -7388.5256197 counts/m and pickup preset (1.05, 1.05, -0.5) m.
The native loader retains the controller's existing integer conversion of the
common cable scale. The imported configuration has isOnline=true and
useCableMotor=true; no control application was launched during this import.
The saved RPiIP value `19268.101.30` is malformed and was preserved from the
requested revision for review, rather than replaced with a guessed address.
Neither that address nor the device flags are consumed by offline planning.

## Measurement conventions

`rail_offsets_m` and `starting_rail_offsets_m` each need four numbers in the
same rail order/datum used by the existing kinematics. They are separate inputs
even if a reviewed setup has equal values. No value is prefilled.

Each calibration transform uses this object shape. The nulls below indicate
missing measurements; replace all of them with the reviewed values:

```json
{
  "translation_m": [null, null, null],
  "rotation_xyzw": {"x": null, "y": null, "z": null, "w": null}
}
```

- `world_from_bed` maps bed coordinates into CDPR world coordinates.
- `end_effector_from_nozzle` maps nozzle coordinates into EE coordinates.
- Translations are metres; rotations are unit quaternions in X/Y/Z/W order.
  Do not enter Euler angles or assume that the nozzle coincides with the EE.
- Record the calibration identifier and method. Set the manifest's frames
  status to `verified` only when the supplied registration is verified.
  The planner's null frame overrides then inherit that manifest registration.

The initial nozzle pose is in the bed frame and uses:

```json
{
  "position_m": [null, null, null],
  "orientation_xyzw": {"x": null, "y": null, "z": null, "w": null}
}
```

Also provide separate logical E and commanded filament positions in millimetres.
G92 E0 establishes a logical coordinate; it does not measure the physical
filament state. For the reviewed PrusaSlicer producer, G21/G90/M82 establish
units and modes, so those mode defaults may stay unspecified.

Cable length/speed/acceleration arrays have eight entries, in cable order, in
m, m/s and m/s². Motor-count bounds have eight signed integer entries. Path
speed/acceleration use m/s and m/s². Filament speed is mm/s; volume flow is
mm³/s; filament diameter is mm. The existing planner uses only supplied or
verified-manifest limits; synthetic example limits do not fill missing values.
Sampling cadence is seconds and the spatial validation interval is metres.
Hotend and bed wait policies need reviewed positive tolerances in °C and
timeouts in seconds. Prusa export temperatures/speeds are separate demo slicing
parameters and should be reviewed for the intended print head and material.

## Prepare another package

From the slicer project directory:

```powershell
& .\tools\New-CdprMachineProfile.ps1 `
    -RobotConfig ..\bin\RobotConfig.json `
    -Name ab1-next-measurement
```

The command requires an explicit source and unused name. It creates a new
directory below `config/machines/`, writes UTF-8 JSON with null measurements,
and refuses to overwrite an existing package. Generated machine packages are
ignored by Git. This is preparation only; the setup report never claims a
filled or verified profile is ready for execution.

## After completing the measurements

First review the completed profile and registration. Then validate a small
cube through the existing offline command:

```powershell
& .\tools\Invoke-CdprSlice.ps1 `
    -Model .\tests\fixtures\cube-10mm.stl `
    -PlannerConfig .\config\machines\ab1-f1ab67c-20261023\planner.local.json `
    -OpenPreview
```

The prepared package currently rejects planning because measurements are
missing. After filling them, the native strict loader still checks the model
hash, rail state, registration, manifests and transformed toolpath. The report
in `setup.local.json` remains the original preparation record; it is not a
live readiness check. The planner/receipt are the validation evidence for an
actual offline run. Hardware readiness remains false and physical execution
is a later integration phase.

Run the preparation checks with `tests/MachineProfile.Tests.ps1`. They require
Windows PowerShell 5.1+ and the existing built planner for the incomplete-profile
rejection check. They do not require the external slicing engine or hardware.
