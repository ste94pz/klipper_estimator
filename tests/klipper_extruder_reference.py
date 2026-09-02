#!/usr/bin/env python3
"""Exercise the pinned Klipper PrinterExtruder move checks unchanged."""

import importlib.util
import json
import math
import os
import pathlib
import re
import subprocess
import sys
import types

PINNED_KLIPPER_COMMIT = "f0892d82b0f1c1228454f09eb508eddde2250f4b"


def load_modules(klipper_root):
    for name in ("mcu", "chelper", "stepper"):
        sys.modules[name] = types.ModuleType(name)
    package = types.ModuleType("kinematics")
    package.__path__ = []
    placeholder = types.ModuleType("kinematics.extruder")
    package.extruder = placeholder
    sys.modules["kinematics"] = package
    sys.modules["kinematics.extruder"] = placeholder

    toolhead_spec = importlib.util.spec_from_file_location(
        "pinned_toolhead", klipper_root / "klippy" / "toolhead.py"
    )
    toolhead = importlib.util.module_from_spec(toolhead_spec)
    toolhead_spec.loader.exec_module(toolhead)

    extruder_spec = importlib.util.spec_from_file_location(
        "pinned_extruder", klipper_root / "klippy" / "kinematics" / "extruder.py"
    )
    extruder = importlib.util.module_from_spec(extruder_spec)
    extruder_spec.loader.exec_module(extruder)
    retraction_spec = importlib.util.spec_from_file_location(
        "pinned_firmware_retraction",
        klipper_root / "klippy" / "extras" / "firmware_retraction.py",
    )
    retraction = importlib.util.module_from_spec(retraction_spec)
    retraction_spec.loader.exec_module(retraction)
    return toolhead, extruder, retraction


class Printer:
    @staticmethod
    def command_error(message):
        return RuntimeError(message)


class Toolhead:
    def __init__(self, config, extruder):
        self.printer = Printer()
        self.max_velocity = config["max_velocity"]
        self.max_accel = config["max_acceleration"]
        self.min_cruise_ratio = config["minimum_cruise_ratio"]
        scv2 = config["square_corner_velocity"] ** 2
        self.junction_deviation = scv2 * (math.sqrt(2.0) - 1.0) / self.max_accel
        self.mcr_pseudo_accel = self.max_accel * (1.0 - self.min_cruise_ratio)
        self.extra_axes = [extruder]


class ScriptRecorder:
    def __init__(self):
        self.scripts = []

    def run_script_from_command(self, script):
        self.scripts.append(script)


class RetractionCommand:
    @staticmethod
    def get_float(name, current, **_kwargs):
        return 1.0 if name == "RETRACT_LENGTH" else current


def make_extruder(module, config):
    instance = module.PrinterExtruder.__new__(module.PrinterExtruder)
    instance.name = "extruder"
    instance.nozzle_diameter = config["nozzle_diameter"]
    instance.filament_area = math.pi * (config["filament_diameter"] * 0.5) ** 2
    instance.max_extrude_ratio = config["max_extrude_cross_section"] / instance.filament_area
    instance.max_e_velocity = config["max_extrude_only_velocity"]
    instance.max_e_accel = config["max_extrude_only_accel"]
    instance.max_e_dist = config["max_extrude_only_distance"]
    instance.instant_corner_v = config["instantaneous_corner_velocity"]
    instance.heater = types.SimpleNamespace(can_extrude=True)
    instance.printer = Printer()
    return instance


def main():
    klipper_root = pathlib.Path(sys.argv[1]).resolve()
    fixture = json.loads(pathlib.Path(sys.argv[2]).read_text(encoding="utf-8"))
    commit = subprocess.run(
        ["git", "-C", str(klipper_root), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if commit != PINNED_KLIPPER_COMMIT and os.environ.get("KLIPPER_ALLOW_UNPINNED") != "1":
        raise SystemExit(f"Klipper reference mismatch: expected {PINNED_KLIPPER_COMMIT}, found {commit}")

    toolhead_module, extruder_module, retraction_module = load_modules(klipper_root)
    extruder = make_extruder(extruder_module, fixture["extruder"])
    toolhead = Toolhead(fixture["printer"], extruder)
    results = []
    for item in fixture["moves"]:
        move = toolhead_module.Move(toolhead, item["start"], item["end"], item["speed"])
        error = None
        try:
            extruder.check_move(move, 3)
        except RuntimeError as exc:
            error = str(exc).splitlines()[0]
        results.append({
            "name": item["name"],
            "max_cruise_v2": move.max_cruise_v2,
            "acceleration": move.accel,
            "error": error,
        })

    first_item, second_item = fixture["junction"]
    first = toolhead_module.Move(toolhead, first_item["start"], first_item["end"], first_item["speed"])
    second = toolhead_module.Move(toolhead, second_item["start"], second_item["end"], second_item["speed"])
    extruder.check_move(first, 3)
    extruder.check_move(second, 3)
    second.calc_junction(first)
    recorder = ScriptRecorder()
    retraction = retraction_module.FirmwareRetraction.__new__(
        retraction_module.FirmwareRetraction
    )
    retraction.retract_length = 2.0
    retraction.retract_speed = 20.0
    retraction.unretract_extra_length = 0.5
    retraction.unretract_speed = 10.0
    retraction.unretract_length = 2.5
    retraction.is_retracted = False
    retraction.gcode = recorder
    retraction.cmd_G10(None)
    retraction.cmd_G11(None)
    retraction.cmd_G10(None)
    retraction.cmd_SET_RETRACTION(RetractionCommand())
    retraction.cmd_G10(None)
    firmware_deltas = [
        float(re.search(r"^G1 E(-?[0-9.]+)", script, re.MULTILINE).group(1))
        for script in recorder.scripts
    ]
    print(json.dumps({
        "name": fixture["name"],
        "moves": results,
        "junction_v2": second.max_start_v2,
        "firmware_deltas": firmware_deltas,
    }))


if __name__ == "__main__":
    main()
