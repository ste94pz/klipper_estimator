#!/usr/bin/env python3
"""Run non-linear check_move methods from the pinned Klipper tree."""

import importlib.util
import json
import math
import pathlib
import subprocess
import sys
import types

sys.dont_write_bytecode = True

import klipper_reference

PINNED_KLIPPER_COMMIT = "f0892d82b0f1c1228454f09eb508eddde2250f4b"
CLASS_NAMES = {
    "delta": "DeltaKinematics",
    "polar": "PolarKinematics",
    "deltesian": "DeltesianKinematics",
    "rotary_delta": "RotaryDeltaKinematics",
}


def load_kinematics(klipper_root, backend):
    sys.modules["stepper"] = types.ModuleType("stepper")
    sys.modules["mathutil"] = types.ModuleType("mathutil")
    sys.modules["chelper"] = types.ModuleType("chelper")
    source = klipper_root / "klippy" / "kinematics" / f"{backend}.py"
    spec = importlib.util.spec_from_file_location(f"kinematics.{backend}", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def delta_checker(module, config):
    checker = object.__new__(module.DeltaKinematics)
    checker.max_velocity = config["max_velocity"]
    checker.max_accel = config["max_accel"]
    checker.max_z_velocity = config["max_z_velocity"]
    checker.max_z_accel = config["max_z_accel"]
    checker.radius = config["radius"]
    checker.min_z = config["minimum_z"]
    checker.arm_lengths = config["arm_lengths"]
    checker.min_arm_length = min(config["arm_lengths"])
    checker.min_arm2 = checker.min_arm_length**2
    checker.max_z = min(config["position_endstops"])
    abs_endstops = [
        endstop + math.sqrt(arm**2 - checker.radius**2)
        for endstop, arm in zip(config["position_endstops"], config["arm_lengths"])
    ]
    checker.limit_z = min(
        endstop - arm for endstop, arm in zip(abs_endstops, config["arm_lengths"])
    )
    half_step = min(config["step_distances"]) * 0.5

    def ratio_to_xy(ratio):
        return (
            ratio
            * math.sqrt(
                checker.min_arm_length**2 / (ratio**2 + 1.0) - half_step**2
            )
            + half_step
            - checker.radius
        )

    checker.slow_xy2 = ratio_to_xy(module.SLOW_RATIO) ** 2
    checker.very_slow_xy2 = ratio_to_xy(2.0 * module.SLOW_RATIO) ** 2
    checker.max_xy2 = min(
        config["print_radius"],
        checker.min_arm_length - checker.radius,
        ratio_to_xy(4.0 * module.SLOW_RATIO),
    ) ** 2
    checker.need_home = False
    checker.limit_xy2 = -1.0
    checker.home_position = (0.0, 0.0, checker.max_z)
    return checker


def polar_checker(module, config):
    checker = object.__new__(module.PolarKinematics)
    checker.max_velocity = config["max_velocity"]
    checker.max_accel = config["max_accel"]
    checker.max_z_velocity = config["max_z_velocity"]
    checker.max_z_accel = config["max_z_accel"]
    checker.v_rad_max = config["max_angular_velocity"]
    checker.limit_xy2 = config["maximum_radius"] ** 2
    checker.limit_z = (config["minimum_z"], config["maximum_z"])
    return checker


def deltesian_checker(module, config):
    checker = object.__new__(module.DeltesianKinematics)
    checker.max_velocity = config["max_velocity"]
    checker.max_accel = config["max_accel"]
    checker.max_z_velocity = config["max_z_velocity"]
    checker.max_z_accel = config["max_z_accel"]
    checker.arm_x = config["arm_x_lengths"]
    checker.arm2 = [arm**2 for arm in config["arm_lengths"]]
    cosine = math.cos(math.radians(config["minimum_angle"]))
    x_min = math.ceil(
        -min(checker.arm_x[0], cosine * config["arm_lengths"][1] - checker.arm_x[1])
    )
    x_max = math.floor(
        min(checker.arm_x[1], cosine * config["arm_lengths"][0] - checker.arm_x[0])
    )
    if config["print_width"]:
        x_limits = (-config["print_width"] * 0.5, config["print_width"] * 0.5)
    else:
        x_limits = (x_min, x_max)
    checker._abs_endstop = [
        endstop + math.sqrt(arm**2 - arm_x**2)
        for endstop, arm, arm_x in zip(
            config["position_endstops"], config["arm_lengths"], checker.arm_x
        )
    ]
    max_z = min(checker._pillars_z_max(x) for x in x_limits)
    checker.limits = [x_limits, config["y_range"], (config["minimum_z"], max_z)]
    checker.home_z = 0.0
    checker.homed_axis = [True, True, True]
    ratio = config["slow_ratio"]
    checker.slow_x2 = checker.very_slow_x2 = None
    if ratio > 0.0:
        ratio2 = ratio**2
        checker.slow_x2 = min(
            math.sqrt(ratio2 * arm**2 / (ratio2 + 1.0)) - arm_x
            for arm, arm_x in zip(config["arm_lengths"], checker.arm_x)
        ) ** 2
        checker.very_slow_x2 = min(
            math.sqrt(2.0 * ratio2 * arm**2 / (2.0 * ratio2 + 1.0)) - arm_x
            for arm, arm_x in zip(config["arm_lengths"], checker.arm_x)
        ) ** 2
    return checker


def rotary_delta_checker(module, config):
    checker = object.__new__(module.RotaryDeltaKinematics)
    checker.max_z_velocity = config["max_z_velocity"]
    checker.min_z = config["minimum_z"]
    checker.max_z = min(config["position_endstops"])
    checker.max_xy2 = min(
        min(config["shoulder_radius"] + arm for arm in config["upper_arm_lengths"]),
        min(arm - config["shoulder_radius"] for arm in config["lower_arm_lengths"]),
    ) ** 2
    arm_z = []
    for endstop, upper, lower in zip(
        config["position_endstops"],
        config["upper_arm_lengths"],
        config["lower_arm_lengths"],
    ):
        dx = -config["shoulder_radius"]
        dy = endstop - config["shoulder_height"]
        c1 = 0.5 / dy * (dx * dx + dy * dy + upper * upper - lower * lower)
        c2 = dx / dy
        scale = c2 * c2 + 1.0
        scaled_x = c1 * c2 + math.sqrt(scale * upper * upper - c1 * c1)
        scaled_y = c1 * scale - c2 * scaled_x
        angle = math.atan2(scaled_y, scaled_x)
        arm_z.append(config["shoulder_height"] + upper * math.sin(angle))
    checker.limit_z = min(z - arm for z, arm in zip(arm_z, config["lower_arm_lengths"]))
    checker.need_home = False
    checker.limit_xy2 = -1.0
    checker.home_position = (0.0, 0.0, checker.max_z)
    return checker


BUILDERS = {
    "delta": delta_checker,
    "polar": polar_checker,
    "deltesian": deltesian_checker,
    "rotary_delta": rotary_delta_checker,
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
            f"Klipper reference mismatch: expected {PINNED_KLIPPER_COMMIT}, found {commit}"
        )

    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    toolhead_module = klipper_reference.load_toolhead(klipper_root)
    modules = {name: load_kinematics(klipper_root, name) for name in CLASS_NAMES}
    output = []
    for item in fixture["cases"]:
        limits = fixture["limits"]
        toolhead = klipper_reference.ReferenceToolhead(limits)
        toolhead.printer = toolhead
        toolhead.command_error = RuntimeError
        move = toolhead_module.Move(toolhead, item["start"], item["end"], item["speed"])
        backend = item["backend"]
        checker = BUILDERS[backend](modules[backend], fixture["backends"][backend])
        rejected = False
        try:
            checker.check_move(move)
        except RuntimeError:
            rejected = True
        output.append(
            {
                "backend": backend,
                "case": item["case"],
                "rejected": rejected,
                "max_velocity": math.sqrt(move.max_cruise_v2),
                "acceleration": move.accel,
            }
        )
    json.dump({"name": fixture["name"], "cases": output}, sys.stdout, sort_keys=True)


if __name__ == "__main__":
    main()
