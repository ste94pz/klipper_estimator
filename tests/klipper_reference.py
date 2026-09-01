#!/usr/bin/env python3
"""Emit normalized phases using a pinned Klipper toolhead.py.

The imports that initialize a printer are stubbed; Move and LookAheadQueue are
loaded unchanged from Klipper. Reference: klippy/toolhead.py at the commit in
PINNED_KLIPPER_COMMIT below.
"""

import importlib.util
import json
import math
import pathlib
import subprocess
import sys
import types

PINNED_KLIPPER_COMMIT = "f0892d82b0f1c1228454f09eb508eddde2250f4b"


def load_toolhead(klipper_root):
    for module_name in ("mcu", "chelper"):
        sys.modules[module_name] = types.ModuleType(module_name)
    kinematics = types.ModuleType("kinematics")
    kinematics.__path__ = []
    extruder = types.ModuleType("kinematics.extruder")
    kinematics.extruder = extruder
    sys.modules["kinematics"] = kinematics
    sys.modules["kinematics.extruder"] = extruder

    source = klipper_root / "klippy" / "toolhead.py"
    spec = importlib.util.spec_from_file_location("pinned_toolhead", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ExtraAxisWithoutLimits:
    def calc_junction(self, _previous, _current, _axis_index):
        return math.inf


class ReferenceToolhead:
    def __init__(self, limits):
        self.max_velocity = limits["max_velocity"]
        self.max_accel = limits["max_acceleration"]
        self.min_cruise_ratio = limits["minimum_cruise_ratio"]
        self.square_corner_velocity = limits["square_corner_velocity"]
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.0) - 1.0) / self.max_accel
        self.mcr_pseudo_accel = self.max_accel * (1.0 - self.min_cruise_ratio)
        self.extra_axes = [ExtraAxisWithoutLimits()]

    def set_acceleration(self, acceleration):
        self.max_accel = acceleration
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.0) - 1.0) / self.max_accel
        self.mcr_pseudo_accel = self.max_accel * (1.0 - self.min_cruise_ratio)


def phase(move):
    return {
        "distance": move.move_d,
        "start_v": move.start_v,
        "cruise_v": move.cruise_v,
        "end_v": move.end_v,
        "accel_t": move.accel_t,
        "cruise_t": move.cruise_t,
        "decel_t": move.decel_t,
        "total_t": move.accel_t + move.cruise_t + move.decel_t,
    }


def main():
    klipper_root = pathlib.Path(sys.argv[1]).resolve()
    fixture_path = pathlib.Path(sys.argv[2]).resolve()
    commit = subprocess.run(
        ["git", "-C", str(klipper_root), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if commit != PINNED_KLIPPER_COMMIT:
        raise SystemExit(
            "Klipper reference mismatch: expected "
            f"{PINNED_KLIPPER_COMMIT}, found {commit}"
        )

    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    module = load_toolhead(klipper_root)
    toolhead = ReferenceToolhead(fixture["limits"])
    queue = module.LookAheadQueue()
    start = [0.0, 0.0, 0.0, 0.0]
    for item in fixture["moves"]:
        if "accel" in item:
            toolhead.set_acceleration(item["accel"])
        move = module.Move(toolhead, start, item["end"], item["speed"])
        queue.add_move(move)
        start = item["end"]
    moves = queue.flush()
    print(json.dumps({"name": fixture["name"], "moves": [phase(m) for m in moves]}))


if __name__ == "__main__":
    main()
