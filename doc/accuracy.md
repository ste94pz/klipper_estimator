# Estimate scope and accuracy

`klipper_estimator` reproduces the time needed for motion commands found in a G-code file. It is most useful as a machine-specific lower bound and as a way to compare slicing choices.

## What affects an estimate

Use printer limits that match the machine which will run the file. The
preferred source is Moonraker because Klipper has already resolved
configuration defaults and included files. A dumped, versioned JSON5 snapshot
is useful for repeatable or offline estimates: it records the Klipper version,
retrieval time, effective limits, resolved settings, source mode, and a stable
fingerprint.

When Moonraker is unavailable, the estimator can resolve a Klipper
configuration tree within an explicitly declared root. This mode follows
Klipper's include ordering and `SAVE_CONFIG` precedence and resolves only the
defaults used by currently supported estimator features. Missing, cyclic, or
out-of-root includes fail instead of producing a partial estimate. Sections
whose behavior is not modeled are named in snapshot warnings; no limits are
invented for them. An equivalent set of effective supported settings produces
the same fingerprint as a configuration-default Moonraker snapshot when the
Klipper-version provenance is also equivalent.

Configuration-default and runtime-snapshot estimates answer different
questions. The default uses values resolved from `configfile.settings` and is
stable across transient `SET_VELOCITY_LIMIT` or G-code-state changes. An
explicit runtime snapshot applies the current `toolhead` limits and
`gcode_move` coordinate modes. Cached runtime state is rejected because it can
no longer be assumed current. A configuration-default cache fallback is marked
`degraded` in machine-readable output; failure to obtain Moonraker data without
a usable cache does not fall back to generic limits.

G-code may change velocity, acceleration, minimum cruise ratio, coordinate
mode, or extrusion mode during a print. The estimator applies Klipper-compatible
lookahead and junction propagation, queue-flush boundaries, coordinate origins,
feed and extrusion overrides, saved G-code state, and G-code offsets.
Unsupported state-changing commands and macros can reduce accuracy even when
the initial printer limits are correct; recognized unsupported state changes
are returned as structured diagnostics instead of being silently assigned zero
effect.

For `cartesian`, `corexy`, `corexz`, `hybrid_corexy`, and `hybrid_corexz`, the
snapshot also carries XYZ travel ranges and the configured Z velocity and
acceleration limits. The planner applies the same component-based Z scaling as
Klipper, including diagonal moves, and diagnoses endpoints outside the
configured range. It assumes the axes have already been homed before the print.
For `delta`, `polar`, `deltesian`, and `rotary_delta`, the snapshot carries the
configured machine geometry and slow-zone parameters. Reachability is checked
separately from duration limits, and the planner mirrors the backend-specific Z,
radial, angular, arm-ratio, and tapered-envelope behavior from the pinned
Klipper reference. Invalid geometry is rejected while loading the configuration.
Generic Cartesian expressions, dual-carriage active state, and other unsupported
kinematics are not approximated: they degrade snapshot accuracy and emit a
structured unsupported-kinematics diagnostic.

Each configured extruder has independent motion and extrusion limits. The
planner tracks tool activation and per-tool E positions, applies Klipper's
extrude-only rules to retractions and moves without X/Y motion (including Z+E),
and diagnoses excessive extrusion distance or cross section. Firmware
retraction is expanded into signed E moves, so per-tool filament statistics do
not turn retractions into positive output. Runtime snapshots initialize the
active tool from Moonraker; later `ACTIVATE_EXTRUDER` commands are applied from
the file. Pressure advance and input shaping remain outside nominal duration
because they do not change the toolhead move timing in the pinned reference.

## What is not generally predictable from the file

Klipper macros may inspect live printer state and execute different commands on each run. Heating, homing, probing, filament changes, user pauses, recovery, and network or host delays may also add wall-clock time that is not represented by normal movement commands.

The reported minimal time should therefore not be interpreted as a guaranteed completion time. Use `ESTIMATOR_ADD_TIME` for known fixed overhead as described in the main README.

## Getting repeatable results

1. Generate the estimate with the configuration of the target printer.
2. Keep extrusion mode explicit in slicer start G-code when possible.
3. Keep dynamic velocity-limit commands in the exported file rather than only in an external macro.
4. Add measured constant macro overhead with `ESTIMATOR_ADD_TIME`.
5. Compare estimates with completed, unpaused prints and investigate errors that scale with move count or geometry separately from constant startup overhead.

The configuration fields `initial_coordinate_mode` and
`initial_extrusion_mode` select the state before the first file command. Their
compatibility defaults are `absolute` and `relative`, respectively; Klipper's
native startup extrusion mode is `absolute`. Choose `absolute` explicitly when
the file is analyzed without an invisible start macro that issues `M83`.

When reporting an accuracy problem, include the G-code, dumped estimator
snapshot and its fingerprint, estimator version, actual print duration, and
whether the print contained pauses or runtime overrides. Remove credentials
and private macro contents before sharing files.
