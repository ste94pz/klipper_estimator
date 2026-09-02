# Klipper print time estimator

This repository is a maintained fork of the now-archived
[Annex Engineering project](https://github.com/Annex-Engineering/klipper_estimator).
The original estimator was created by Lasse Dalegaard and improved by many
contributors. Their work made this continuation possible, and is gratefully
acknowledged.

`klipper_estimator` is a tool for determining the time a print will take using
the Klipper firmware. Currently it provides the following modes:

  * `estimate` mode outputs detailed statistics about a print job
  * `post-process` mode can be used as a Slicer post-processing script, updating
    the gcode output file with corrected time estimates.
  * `dump-moves` mode dumps planning data for every move in a file

The estimator mirrors Klipper's motion planner and the supported kinematics
described below. Small numeric differences may remain due to rounding modes. If
the timing is far off (for example, more than a minute over a 12-hour print),
this is considered a bug.

## Getting `klipper_estimator`

Pre-built binaries are available for the latest release on the GitHub Releases
page. If you wish to build the tool yourself or poke around the source, see the
[Building section](#Building).

Binaries are provided for Windows, Linux, Mac OS X, and Raspberry Pi targets.
On Linux and Mac OS X, ensure that you give the downloaded file executable
permissions. This can be done in the terminal as follows:
```
$ chmod +x klipper_estimator
```
Change the filename (last parameter) to match the downloaded file.

For Arch Linux, an AUR package
[`klipper_estimator`](https://aur.archlinux.org/packages/klipper-estimator) is
available, courtesy of Wilhelm Schuster. Thanks!

## Usage

Basic usage info can be found by running `klipper_estimator` with no arguments.

### Configuration

In order to provide accurate times, `klipper_estimator` needs printer settings
including maximum velocity, acceleration, etc. It can take these either from a
config file (`--config_file`) or grab them directly from Moonraker
(`--config_moonraker_url` and, if authentication is required,
`--config_moonraker_api_key`). It can also resolve a Klipper configuration tree
offline with `--config_klipper_root` and `--config_klipper_file`.

Moonraker configuration is stored as a versioned snapshot. A snapshot records
its source, retrieval time, Klipper version, resolved `configfile.settings`,
every configured extruder, the queried `toolhead` and `gcode_move` objects, and
a stable SHA-256 fingerprint. The fingerprint covers the effective limits and
resolved settings but not the retrieval time, so equivalent snapshots remain
comparable.

The default `--config_moonraker_mode configuration-default` estimates with
resolved configuration defaults while retaining runtime objects as provenance.
Use `--config_moonraker_mode runtime-snapshot` explicitly to apply the current
runtime velocity limits and G-code coordinate modes. Runtime snapshots are not
loaded from the Moonraker cache because those overrides would be stale.

For an offline printer configuration, declare both the directory that confines
all configuration access and the root file relative to that directory:

```
$ ./klipper_estimator \
    --config_klipper_root /home/pi/printer_data/config \
    --config_klipper_file printer.cfg \
    dump-config > config.json
```

Offline loading follows Klipper's linear include order, including sorted `*`,
`?`, and character-class glob matches, and applies a valid generated
`SAVE_CONFIG` block after regular configuration. A saved option does not
override the same option in the root file or an include, matching Klipper.
Missing or cyclic includes, malformed values, corrupted generated blocks, and
paths or symlinks escaping the declared root are rejected. Wildcard includes
are allowed to match no files, as in Klipper.

The resulting snapshot has source `offline_configuration` and selection
`configuration_default`. It resolves the Klipper defaults currently consumed
by the estimator for `[printer]`, every contiguous `[extruderN]`,
`[stepper_x]`/`[stepper_y]`/`[stepper_z]`, `[firmware_retraction]`, and
`[gcode_arcs]`. Other sections are listed in the snapshot warnings and are not
assigned guessed values. A live Moonraker snapshot remains preferable when
available because Klipper itself then provides all resolved settings and its
version.

To experiment with settings, use `dump-config` together with
`--config_moonraker_url` to export a complete snapshot. The result can be
imported again without losing its provenance:

To dump a config, use e.g.:
```
$ ./klipper_estimator --config_moonraker_url http://192.168.0.21 dump-config > config.json
```

The config file format is JSON5 and thus allows normal JSON with some
extensions (see https://json5.org/). Legacy flat `PrinterLimits` JSON5 files
remain supported. A legacy Moonraker cache is migrated explicitly and marked
as degraded because it contains no Klipper version or resolved-settings
provenance.

After generating a config, one can use this in other commands like so:
```
$ ./klipper_estimator --config_file config.json estimate ...
```

If Moonraker is unavailable, `--config_moonraker_ignore_error` uses
`--config_moonraker_cache_file` only when a compatible configuration-default
snapshot exists. With no usable cache the command fails instead of silently
falling back to generic limits. Running without any configuration source still
uses the built-in limits for compatibility, but human output prints a warning
and JSON output reports `configuration.accuracy` as `degraded` together with
the reason. Estimate JSON always includes the configuration source, fingerprint,
retrieval time, Klipper version, and warnings.

### Quirks

Be aware of the following "quirks" when using `klipper_estimator` compared to Klipper itself:

#### Relative extrusion by default

`klipper_estimator` assumes _relative_ extrusion and _absolute_ movement by
default. This is different from Klipper, which assumes _absolute_ extrusion as
well. This difference exists because `klipper_estimator` can't see inside
macros. Most users use relative extrusion, and put the M83 command in their
print start macro, making it invisible to `klipper_estimator`.

The initial modes are explicit JSON5 configuration fields. Existing
configurations keep the compatibility defaults shown here:

```json5
{
  initial_coordinate_mode: "absolute",
  initial_extrusion_mode: "relative",
}
```

Set `initial_extrusion_mode` to `"absolute"` to use Klipper's startup default,
or keep the mode explicit in exported G-code with `M82`/`M83`. G-code may switch
XYZ independently with `G90`/`G91`.

If you wish to use _absolute_ extrusion, you must ensure that an `M82` command
is inserted in your slicer start gcode. E.g.:

```
PRINT_START
M82
```

#### G-code coordinate state

The estimator models Klipper's physical position separately from its G-code
origin. It supports `G90`, `G91`, `G92`, `M82`, `M83`, `M220`, `M221`, named
`SAVE_GCODE_STATE`/`RESTORE_GCODE_STATE`, and `SET_GCODE_OFFSET`. The `MOVE` and
`MOVE_SPEED` parameters on restore and offset commands generate planned moves
with the same coordinate-state behavior as Klipper.

Known state-changing commands that are parsed but not modeled produce a
structured diagnostic. Human output prints the diagnostic before the estimate;
JSON output includes it in the top-level `diagnostics` array. Post-processing
and `dump-moves` report the same condition on stderr. Such an estimate remains
a lower bound rather than silently assuming that the command had no effect.

#### Motion planning and velocity limits

The lookahead planner follows Klipper's minimum-cruise-ratio algorithm from the
pinned reference documented under [Development checks](#development-checks).
`SET_VELOCITY_LIMIT` supports `VELOCITY`, `ACCEL`,
`SQUARE_CORNER_VELOCITY`, and `MINIMUM_CRUISE_RATIO`. The former
`ACCEL_TO_DECEL` parameter and `max_accel_to_decel` configuration field remain
available only as compatibility adapters for files and configurations produced
for older Klipper versions; prefer `MINIMUM_CRUISE_RATIO` in new inputs.

Commands that make Klipper wait for or flush queued motion form lookahead
boundaries in the estimate. This includes `M400`, `G4`, homing, temperature
waits, and the estimator's existing indeterminate wait operations. Motion on
the two sides of a boundary therefore starts or ends at rest as appropriate.

#### Kinematics

The estimator applies Klipper's configured axis ranges, `max_z_velocity`, and
`max_z_accel` for `cartesian`, `corexy`, `corexz`, `hybrid_corexy`, and
`hybrid_corexz`. Z limits are scaled by the Z component of a diagonal move in
the same way as Klipper. Because a print file is estimated for a prepared
printer, configured axes are treated as homed; a move outside their range
produces a structured `move_outside_kinematic_bounds` diagnostic indicating
that Klipper would reject the file.

Delta, polar, deltesian, and rotary-delta configurations are supported from
Moonraker or an offline Klipper configuration. The estimator imports their
machine geometry, rejects unreachable endpoints, applies Z limits, and mirrors
Klipper's radial, angular, arm-ratio, and tapered slow zones. Invalid geometry
fails configuration loading instead of producing an estimate.

Other unsupported backends, `generic_cartesian`, and dual-carriage
configurations degrade snapshot accuracy, name the unsupported backend in their
warnings, and produce an `unsupported_kinematics` planner diagnostic. Generic
Cartesian carriage expressions and active dual-carriage state must be modeled
before those configurations can be supported safely.

#### Motion transforms

Saved `[bed_mesh NAME]` profiles are imported from Moonraker snapshots and
offline Klipper configuration trees. `BED_MESH_PROFILE LOAD=...`,
`BED_MESH_CLEAR`, and `BED_MESH_OFFSET` update the active transform while the
file is processed. The planner applies Klipper's mesh interpolation, fade, and
trajectory splitting before kinematic checks. A runtime snapshot also imports
the currently active mesh, including an adaptive mesh exposed only through the
`bed_mesh` status object.

Skew profiles and `SET_SKEW`/`SKEW_PROFILE LOAD=...` are supported. Klipper's
transform registration order is preserved: skew correction changes X/Y first,
then bed mesh looks up and splits that corrected trajectory. Profile save or
remove operations remain explicit unsupported-state diagnostics because they
mutate session configuration rather than merely selecting a known profile.

Klipper does not expose the active skew factors in its status object, only a
profile name that may be stale after `SET_SKEW`. Runtime snapshots containing
`skew_correction` are therefore marked degraded even when a named profile can
be used as a best available initial value. Active unsupported transforms such
as `bed_tilt`, `z_thermal_adjust`, or runtime object exclusion likewise degrade
snapshot accuracy and produce `unsupported_motion_transform` diagnostics.
Configuration-default and offline snapshots start with bed mesh and skew
inactive; activation hidden inside a macro cannot be inferred from the print
file.

#### Extruders

Every contiguous `[extruderN]` is modeled with its own filament and nozzle
diameters, extrude-only velocity, acceleration and distance limits, maximum
cross section, and instantaneous corner velocity. `ACTIVATE_EXTRUDER` switches
the active limits and restores that tool's independent physical E position, as
Klipper does. A runtime Moonraker snapshot also uses the currently active
`toolhead.extruder`; configuration-default and offline snapshots start with
`extruder`.

Klipper's extrude-only rules are applied to retractions and to any move without
X/Y motion, including combined Z+E moves. Files that exceed the configured
extrude-only distance or cross section produce structured diagnostics naming
the active tool and the rejected limit. Firmware `G10`/`G11` moves retain their
negative/positive filament direction, and `SET_RETRACTION` resets the firmware
retraction latch in the same way as Klipper.

Estimate JSON includes signed `net_distance`, positive `extruded_distance`, and
negative `retracted_distance` statistics under each sequence's `extruders`
object. Human output reports the same values per tool. Pressure advance and
input shaping do not change nominal planner duration at the pinned Klipper
reference and are therefore not added to it.

### `estimate` mode

Estimation mode is useful for determining statistics about a print, in order to
optimize print times. It gives a high level summary.

Basic usage:
```
$ ./klipper_estimator [config options] estimate ~/3DBenchy.data
Sequences:
 Run 0:
  Total moves: 42876
  Total distance: 73313.01640025008
  Total extrude distance: 3407.877500000097
  Minimal time: 1h29m9.948s (5349.947936969622)
  Average flow: 1.5321468696371368 mm3/s
  Phases:
    Acceleration: 27m4.291s
    Cruise:       35m1.116s
    Deceleration: 27m4.291s
  Moves:
  Layer times:
         0 => 2.536s
         ... [some lines omitted for brevity]
        48 => 4.834s
  Kind times:
   4m23.463s            => FILL
   2.639s               => Other
   18m0.185s            => SOLID-FILL
   28m29.706s           => WALL-INNER
   38m13.706s           => WALL-OUTER
```

The calculations are done based only on the commands found in the file, with no
regards for macro expansions. This means that `print_start` type macros will
count as zero seconds, as well heat up times, homing, etc. Therefore the time
output should be considered a "minimal time", assuming these extra factors take
no time.

See [Estimate scope and accuracy](doc/accuracy.md) for practical guidance on
configuration, repeatability, and interpreting the result.

### `post-process` mode

In `post-process` mode `klipper_estimator` directly modifies the filename passed
in in-place, updating time estimations in the file.

When using `klipper_estimator` in `post-process` mode, simply add a
post-processing script in your slicer like so:
```
/path/to/klipper_estimator --config_moonraker_url http://192.168.0.21 post-process
```
Change the path and config options to fit your situation.

Currently the following slicers are supported:

  * PrusaSlicer
  * SuperSlicer
  * OrcaSlicer
  * ideaMaker
  * Cura
  * Simplify3D

In PrusaSlicer, SuperSlicer, and OrcaSlicer `Post-processing scripts` are set in `Output
Options` under `Print Settings`:

![PrusaSlicer, Orcaslicer, and SuperSlicer Post-processing scripts option](/doc/post_processing_psss.png)

Note that ideaMaker does not have support for post-processing scripts, and thus
cannot automatically run `klipper_estimator` on export.

For Cura, using
[klipper Preprocessor](https://github.com/pedrolamas/klipper-preprocessor) is
recommended. See their git repository for information on how to set up this tool.

In Simplify3D the relevant estimation command must be added under `Scripts` in
the `Additional terminal commands for post processing` field. This field is just called
`Post Processing` in V5.x, and the command should be appended with a [output_filepath].

```
/path/to/klipper_estimator --config_moonraker_url http://192.168.0.21 post-process [output_filepath]
```

### `dump-moves` mode

The `dump-moves` mode is used like `estimate` mode, but instead of providing a
summary, move planning data is dumped for every move.

### Accurately estimating `PRINT_START`/`PRINT_END` macros

Klipper macros can perform arbitrarily complex operations. `klipper_estimator`
has no hope of estimating how long these will take, as the Jinja templates can
access any state of the read printer. However it is often the case that the
amount of print time actually spent within the macro is constant. A prime
example of this is print start macros. The macro may execute homing and heating
commands, but the print timer does not start until the first material is
extruded. This generally happens when the prime line is started.

This gives rise to an offset in print time that we cannot estimate, but the user
can easily measure it after a print is over.

To compensate for this, `klipper_estimator` understands the following gcode
comment(generally syntax followed by some examples):

```
; ESTIMATOR_ADD_TIME <duration, seconds> [description]
; E.g.:
; ESTIMATOR_ADD_TIME 21
; ESTIMATOR_ADD_TIME 21 Print start
```

When `klipper_estimator` encounters a comment with this format, it will add the
requested duration to the total print time. The time will also be tracked as a
"move kind", if the description field is given.

Note that only the upper-case string `ESTIMATOR_ADD_TIME`, on a separate comment
line, will trigger this behaviour. Any whitespace between the `;` and `E`
characters will however be ignored.

The intended usage of this functionality is for print start macros, when
executed by the slicer. E.g. in PrusaSlicer, SuperSlicer, or OrcaSlicer, one might set their
print start gcode like this:

```
; ESTIMATOR_ADD_TIME 20 Prime line
print_start extruder=[first_layer_temperature] bed=[first_layer_bed_temperature]
```

## Development checks

The normal verification does not require a Klipper checkout:

```sh
cargo fmt --all -- --check
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
```

Planner changes must additionally be checked against the pinned Klipper
reference. Set `KLIPPER_PATH` if the checkout is not available as `klipper/`:

```sh
KLIPPER_PATH=/path/to/klipper bash tests/run_klipper_differential.sh
```

The differential fixtures verify Klipper commit
`f0892d82b0f1c1228454f09eb508eddde2250f4b`. The test rejects another revision
instead of silently changing the baseline. Numeric tolerances and fixture
provenance are recorded next to the fixtures and test code.

## Building

`klipper_estimator` is written in Rust. Version 1.58 or newer is required to
compile the tool. Assuming a Rust toolchain is installed, along with git, one
can build `klipper_estimator` by running:

```
$ git clone https://github.com/ste94pz/klipper_estimator.git
$ cd klipper_estimator
$ cargo build --release
// Resulting binary will be at `target/release/klipper_estimator`(.exe on Windows)
```

## Acknowledgements

This project is in no way endorsed by the Klipper project. Please do not direct
any support requests to the Klipper project.

  * [Klipper](https://www.klipper3d.org/) by [Kevin O'Connor](https://www.patreon.com/koconnor)
  * [Moonraker](https://github.com/Arksine/moonraker) by Arksine
