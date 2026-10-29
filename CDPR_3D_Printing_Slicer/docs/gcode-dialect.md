# Restricted FDM G-code import contract

Dialect ID: `marlin_restricted_v1`. Importer source was added in S2-A, used by S2-B,
and built and tested with synthetic data in S2-C. See [validation](validation.md).
The importer is linear, single-tool and fixed-orientation.

The STL workflow now has a tested producer: PrusaSlicer 2.9.6 with
`config/prusa-cdpr.ini`. The external adapter passes the original emitted G-code
to this importer without a command-removal filter. Profile-controlled homing,
leveling, motor-control, firmware acceleration, arcs and binary output are
excluded at export. Unsupported generated commands still reject the complete
job. See [the workflow guide](stl-workflow.md); other engines, profiles and engine
versions do not inherit this compatibility claim.

## Lexical and input rules

- Accept plain ASCII/UTF-8 text with LF or CRLF; an initial UTF-8 BOM is permitted.
- Reject embedded NULs, invalid encoding, overlong lines and configured input limits.
- Accept case-insensitive command/parameter letters and optional whitespace
  between a letter and number. Do not require spaces between words.
- Accept signed decimal numbers and decimal points. Exponent notation has no
  representation: `E/e` always starts an axis/parameter word. Thus `X1e-3` means
  `X1 E-3`, not 0.001. Producers must export decimal axis words.
- Consume each numeric token completely. Reject NaN, Infinity, missing values,
  duplicate parameter letters and unrecognized parameters on a supported command.
- Accept semicolon comments and non-nested parenthesized comments. Reject
  unterminated or nested parentheses. Comments never execute code.
- Permit one G/M/T command per physical line in v1. A parameter-only motion
  line is allowed only with an established G0/G1 mode. Multiple command words
  on one line are rejected rather than guessed or silently dropped.
- Optional N line numbers must increase by one once present. A checksum, when
  present, is the decimal RepRap XOR of ASCII bytes before `*`, excluding line
  endings. Checksummed lines require N; the checksum suffix is validated before
  comments are stripped. Partial/duplicate sequence input is rejected. An
  unnumbered preamble may precede the first N line. After numbering starts,
  executable lines must be numbered and checksum presence must be consistent.
  Blank/comment-only unnumbered lines remain allowed.
  The checksum suffix contains only its decimal byte value and whitespace;
  trailing annotations after the checksum are rejected.
- Track original 1-based line ranges. Any error rejects the complete import;
  no truncated program or partial successful job is returned.
- Binary G-code, 3MF archives and model files are not parser inputs in v1.

## Modal state

Begin with unknown units, XYZ mode, E mode, position and feedrate unless they
are supplied explicitly in the import request. An explicit profile initial
state is recorded; it does not establish a physical robot position.

An explicit G20/G21, G90/G91 or M82/M83 can establish its corresponding state.
Missing axes retain the established logical value. Relative motion from an
unknown position is rejected. A known physical starting nozzle pose is required
to validate the first move, even if its destination names all three axes.

For every nonzero move, including XYZ-only travel, logical E and a commanded
filament datum must also be known. The latter must come from the explicit
initial profile; G92 E establishes only logical E. No-op moves with known pose
and an unchanged E delta are omitted with a source-linked info diagnostic.

The restricted Marlin behavior is deliberate: G90 sets absolute positioning
for XYZ and E; G91 sets relative positioning for XYZ and E. M82/M83 override E
until the next G90/G91. Changing modes never itself moves an axis or resets E.
F values remain modal, including an F-only line. G0 and G1 share the feedrate in
this restricted profile. A future producer needing different semantics requires
a distinct dialect ID and acceptance fixtures.

G20/G21 change the interpretation of subsequent numeric length/feed words;
already normalized coordinates, origin offsets and feedrates are not rescaled.
E is filament length, not volume. The profile does not enable volumetric mode.

## Commands

| Input | Normalized behavior |
| --- | --- |
| G0/G1 with X/Y/Z/E/F | Linear geometry plus signed E; classify by geometry and extrusion spans, not G number |
| G20 / G21 | Inch / millimeter input units; canonical geometry m and filament mm |
| G90 / G91 | Set XYZ and E mode as described above |
| M82 / M83 | Override absolute / relative E mode |
| G92 X/Y/Z/E | Change logical origins without moving; retain physical geometry/E continuity |
| G4 P / G4 S | P milliseconds or S seconds; require exactly one nonnegative argument |
| M104 S / M109 S | Hotend set target / heat-up wait, tool 0 only |
| M109 R | Hotend wait for heating or cooling; see wait policy below |
| M140 S / M190 S or R | Bed set/wait event; head capability remains unevaluated |
| M106 S / M107 | S in [0,255] maps to fan fraction; M107 is fraction 0; fan channel 0 |
| M400 | Synchronization barrier; it never proves completion of an external CDPR |
| T0 | Select the only supported tool; record in import trace without physical action |
| M117 text / M73 supported progress parameters | Allowlisted metadata only; retain an info diagnostic, no physical operation |
| Blank/comment-only lines | No physical operation; optional layer annotations are informational |
| End of file | Append one synthetic end_job after preserving all prior operations |

M104/M109 may explicitly name T0. M106 may explicitly name P0 and must include S.
M107 may explicitly name P0. Missing targets, multiple S/R/P alternatives or
other heater/tool/fan channels are rejected. M73 permits P (0..100 percent) and
R (nonnegative minutes) only. M117's remainder is inert text, never a macro.

S waits use wait_mode `heat_only`; R waits use `heat_or_cool`. Both require an
explicitly configured positive tolerance and timeout before executable timing
can be planned. A heat-only wait already above its target does not wait for
cooling; a heat-or-cool wait requires the target window. Missing wait policy or
head capability leaves checks unevaluated in imported planning results, not
implicitly successful. Temperature set operations have a null wait_mode.

A zero-duration G4 becomes an explicit barrier. An F-only line updates modal
state without a movement. A no-op XYZ/E command can be omitted only with an
info diagnostic preserving its source line. G92 state changes and T0 selections
are likewise retained in import diagnostics, not sent to robot calibration.

## Extrusion and feedrate

Track logical E independently of signed physical filament position. G92 E0
changes only the logical origin. Compute the move's delta according to the
current E mode; carry it through transforms and retiming unchanged.

Retraction is a negative delta. Later positive movement first recovers known
retraction debt; any remainder is deposition on a moving path or prime on an
E-only move. Split purposes into interpolation spans if one original linear
move includes both recovery and deposition/prime. Do not invent or remove E.
Debt begins at zero for this imported command history; no physical priming state
is inferred. Nonzero increments lost to floating-point range/precision are
rejected when detected rather than treated as a completed movement.

When an F word occurs, normalize it once to `F_mm_per_min = F * unit_mm`, where
`unit_mm` is 1 or 25.4 at that word. Nonzero XYZ movement uses
`F_mm_per_min / 60000` m/s; E-only movement uses `F_mm_per_min / 60` mm/s.
Store that normalized modal quantity. Later unit switches do not rescale it;
each new F word replaces it using the units active at that word.
Each move's feedrate must be defined, finite and strictly positive.

Filament diameter is needed only for volumetric/flow validation, not for
preserving the slicer's commanded E. Missing diameter/flow limits leave those
checks unevaluated. Speed changes retime both geometry and extrusion.

## Explicitly unsupported

Recognize and reject G2/G3 arcs in v1, G5 splines, firmware retraction G10/G11,
G28 homing, G29/probing, workspace changes, motor-enable/disable commands,
M200 volumetric E, acceleration/jerk/pressure-advance commands, feed/flow
overrides, multi-tool changes, pause macros, firmware settings and unknown codes.
No unsupported command is forwarded to Robot or to a printer.

G2/G3 can be added later only with plane/center conventions and chord-tolerance
tests. Homing, heater safety and motor shutdown require controller procedures,
not text substitutions. Imported start/end sequences are fully checked; a
conventional desktop-printer preamble is not automatically CDPR-compatible.

## Review sources

- [Marlin G0/G1](https://marlinfw.org/docs/gcode/G000-G001.html)
- [Marlin M82](https://marlinfw.org/docs/gcode/M082.html)
- [Marlin M83](https://marlinfw.org/docs/gcode/M083.html)
- [Marlin G92](https://marlinfw.org/docs/gcode/G092.html)
- [Marlin M109](https://marlinfw.org/docs/gcode/M109.html)
- [Marlin M190](https://marlinfw.org/docs/gcode/M190.html)

These are dialect references, not instructions to execute G-code. This profile
deliberately supports a restricted subset; binary/model import and firmware
features remain unsupported. Any discovered mismatch must update this contract
and its acceptance fixtures before support is claimed.
