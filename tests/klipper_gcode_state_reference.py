#!/usr/bin/env python3
"""Run state commands through the pinned Klipper GCodeMove implementation."""

import importlib.util
import json
import pathlib
import shlex
import subprocess
import sys

sys.dont_write_bytecode = True

import klipper_reference


PINNED_KLIPPER_COMMIT = "f0892d82b0f1c1228454f09eb508eddde2250f4b"


class FakeGcmd:
    def __init__(self, line):
        self.line = line
        tokens = shlex.split(line)
        self.command = tokens[0].upper()
        self.params = {}
        for token in tokens[1:]:
            if "=" in token:
                key, value = token.split("=", 1)
            else:
                key, value = token[0], token[1:]
            self.params[key.upper()] = value

    def get_command_parameters(self):
        return self.params

    def get_float(self, key, default=None, above=None):
        value = self.params.get(key.upper())
        if value is None:
            return default
        value = float(value)
        if above is not None and value <= above:
            raise ValueError(f"{key} must be above {above}")
        return value

    def get_int(self, key, default=None):
        value = self.params.get(key.upper())
        return default if value is None else int(value)

    def get(self, key, default=None):
        return self.params.get(key.upper(), default)

    def get_commandline(self):
        return self.line

    def error(self, message):
        return ValueError(message)


def load_gcode_move(klipper_path):
    path = pathlib.Path(klipper_path) / "klippy" / "extras" / "gcode_move.py"
    spec = importlib.util.spec_from_file_location("pinned_gcode_move", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.GCodeMove


def main():
    klipper_path, fixture_path = sys.argv[1:]
    commit = subprocess.run(
        ["git", "-C", klipper_path, "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if commit != PINNED_KLIPPER_COMMIT:
        raise SystemExit(
            f"Klipper reference mismatch: expected {PINNED_KLIPPER_COMMIT}, found {commit}"
        )
    fixture = json.loads(pathlib.Path(fixture_path).read_text())
    cls = load_gcode_move(klipper_path)
    state = cls.__new__(cls)
    state.absolute_coord = fixture["initial_coordinate_mode"] == "absolute"
    state.allow_absolute_extrude = fixture["initial_extrusion_mode"] == "absolute"
    state.base_position = [0.0] * 4
    state.last_position = [0.0] * 4
    state.homing_position = [0.0] * 4
    state.axis_map = {"X": 0, "Y": 1, "Z": 2, "E": 3}
    state.speed = 100.0
    state.speed_factor = 1.0 / 60.0
    state.extrude_factor = 1.0
    state.saved_states = {}
    moves = []

    def record_move(position, speed):
        moves.append({"end": list(position), "speed": speed})

    state.move_with_transform = record_move
    for line in fixture["commands"]:
        command = line.split(maxsplit=1)[0].upper()
        getattr(state, "cmd_" + command)(FakeGcmd(line))

    toolhead_module = klipper_reference.load_toolhead(pathlib.Path(klipper_path))
    toolhead = klipper_reference.ReferenceToolhead(fixture["limits"])
    queue = toolhead_module.LookAheadQueue()
    start = [0.0, 0.0, 0.0, 0.0]
    for move_data in moves:
        move = toolhead_module.Move(toolhead, start, move_data["end"], move_data["speed"])
        queue.add_move(move)
        start = move_data["end"]
    planned_moves = queue.flush()
    for move_data, planned_move in zip(moves, planned_moves):
        move_data["total_time"] = klipper_reference.phase(planned_move)["total_t"]

    result = {
        "name": fixture["name"],
        "position": state.last_position,
        "base_position": state.base_position,
        "homing_position": state.homing_position,
        "gcode_position": state._get_gcode_position(),
        "speed": state.speed,
        "speed_factor": state.speed_factor,
        "extrude_factor": state.extrude_factor,
        "absolute_coordinate": state.absolute_coord,
        "absolute_extrusion": state.allow_absolute_extrude,
        "moves": moves,
    }
    json.dump(result, sys.stdout, sort_keys=True)


if __name__ == "__main__":
    main()
