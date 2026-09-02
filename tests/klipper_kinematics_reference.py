#!/usr/bin/env python3
"""Run Cartesian-family check_move methods from the pinned Klipper tree."""

import importlib.util
import json
import math
import pathlib
import os
import subprocess
import sys
import types

sys.dont_write_bytecode = True

import klipper_reference

PINNED_KLIPPER_COMMIT = "f0892d82b0f1c1228454f09eb508eddde2250f4b"
CLASS_NAMES = {
    "cartesian": "CartKinematics",
    "corexy": "CoreXYKinematics",
    "corexz": "CoreXZKinematics",
    "hybrid_corexy": "HybridCoreXYKinematics",
    "hybrid_corexz": "HybridCoreXZKinematics",
}


def load_kinematics(klipper_root, backend):
    sys.modules["stepper"] = types.ModuleType("stepper")
    package = sys.modules["kinematics"]
    package.idex_modes = types.ModuleType("kinematics.idex_modes")
    sys.modules["kinematics.idex_modes"] = package.idex_modes
    source = klipper_root / "klippy" / "kinematics" / f"{backend}.py"
    spec = importlib.util.spec_from_file_location(f"kinematics.{backend}", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    klipper_root = pathlib.Path(sys.argv[1]).resolve()
    fixture_path = pathlib.Path(sys.argv[2]).resolve()
    commit = subprocess.run(
        ["git", "-C", str(klipper_root), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if commit != PINNED_KLIPPER_COMMIT and os.environ.get("KLIPPER_ALLOW_UNPINNED") != "1":
        raise SystemExit(
            f"Klipper reference mismatch: expected {PINNED_KLIPPER_COMMIT}, found {commit}"
        )

    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    toolhead_module = klipper_reference.load_toolhead(klipper_root)
    limits = fixture["limits"]
    modules = {
        backend: load_kinematics(klipper_root, backend) for backend in CLASS_NAMES
    }
    output = []
    for item in fixture["cases"]:
        toolhead = klipper_reference.ReferenceToolhead(limits)
        toolhead.printer = toolhead
        toolhead.command_error = RuntimeError
        move = toolhead_module.Move(toolhead, item["start"], item["end"], item["speed"])
        module = modules[item["backend"]]
        checker = object.__new__(getattr(module, CLASS_NAMES[item["backend"]]))
        checker.limits = list(zip(limits["axis_minimum"], limits["axis_maximum"]))
        checker.max_z_velocity = limits["max_z_velocity"]
        checker.max_z_accel = limits["max_z_accel"]
        rejected = False
        try:
            checker.check_move(move)
        except RuntimeError:
            rejected = True
        output.append(
            {
                "backend": item["backend"],
                "case": item["case"],
                "rejected": rejected,
                "max_velocity": math.sqrt(move.max_cruise_v2),
                "acceleration": move.accel,
            }
        )
    json.dump({"name": fixture["name"], "cases": output}, sys.stdout, sort_keys=True)


if __name__ == "__main__":
    main()
