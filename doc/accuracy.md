# Estimate scope and accuracy

`klipper_estimator` reproduces the time needed for motion commands found in a G-code file. It is most useful as a machine-specific lower bound and as a way to compare slicing choices.

## What affects an estimate

Use printer limits that match the machine which will run the file. The preferred source is Moonraker because Klipper has already resolved configuration defaults and included files. A dumped JSON5 configuration is useful for repeatable or offline estimates.

G-code may change velocity, acceleration, coordinate mode, or extrusion mode during a print. Unsupported state-changing commands and macros can reduce accuracy even when the initial printer limits are correct.

## What is not generally predictable from the file

Klipper macros may inspect live printer state and execute different commands on each run. Heating, homing, probing, filament changes, user pauses, recovery, and network or host delays may also add wall-clock time that is not represented by normal movement commands.

The reported minimal time should therefore not be interpreted as a guaranteed completion time. Use `ESTIMATOR_ADD_TIME` for known fixed overhead as described in the main README.

## Getting repeatable results

1. Generate the estimate with the configuration of the target printer.
2. Keep extrusion mode explicit in slicer start G-code when possible.
3. Keep dynamic velocity-limit commands in the exported file rather than only in an external macro.
4. Add measured constant macro overhead with `ESTIMATOR_ADD_TIME`.
5. Compare estimates with completed, unpaused prints and investigate errors that scale with move count or geometry separately from constant startup overhead.

When reporting an accuracy problem, include the G-code, dumped estimator configuration, estimator version, actual print duration, and whether the print contained pauses or runtime overrides. Remove credentials and private macro contents before sharing files.
