#!/usr/bin/env python3
"""Run pinned Klipper bed_mesh path splitting followed by its motion planner."""

import importlib.util
import json
import pathlib
import subprocess
import sys
import types

sys.dont_write_bytecode = True

import klipper_reference

PINNED_KLIPPER_COMMIT = "f0892d82b0f1c1228454f09eb508eddde2250f4b"


def load_bed_mesh(klipper_root):
    package = types.ModuleType("extras")
    package.__path__ = []
    package.probe = types.ModuleType("extras.probe")
    sys.modules["extras"] = package
    sys.modules["extras.probe"] = package.probe
    source = klipper_root / "klippy" / "extras" / "bed_mesh.py"
    spec = importlib.util.spec_from_file_location("extras.bed_mesh", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def skew(position, factors):
    x, y, z, e = position
    return [
        x - y * factors["xy"] - z * (factors["xz"] - factors["xy"] * factors["yz"]),
        y - z * factors["yz"],
        z,
        e,
    ]


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
    bed_mesh = load_bed_mesh(klipper_root)
    toolhead_module = klipper_reference.load_toolhead(klipper_root)
    params = {
        "min_x": fixture["bed_mesh"]["min"][0],
        "max_x": fixture["bed_mesh"]["max"][0],
        "min_y": fixture["bed_mesh"]["min"][1],
        "max_y": fixture["bed_mesh"]["max"][1],
        "x_count": len(fixture["bed_mesh"]["points"][0]),
        "y_count": len(fixture["bed_mesh"]["points"]),
        "mesh_x_pps": fixture["bed_mesh"]["mesh_pps"][0],
        "mesh_y_pps": fixture["bed_mesh"]["mesh_pps"][1],
        "algo": fixture["bed_mesh"]["algorithm"],
        "tension": fixture["bed_mesh"]["tension"],
    }
    mesh = bed_mesh.ZMesh(params, "test")
    mesh.build_mesh(fixture["bed_mesh"]["points"])
    splitter = bed_mesh.MoveSplitter.__new__(bed_mesh.MoveSplitter)
    splitter.split_delta_z = fixture["bed_mesh"]["split_delta_z"]
    splitter.move_check_distance = fixture["bed_mesh"]["move_check_distance"]
    splitter.initialize(mesh, 0.0)

    logical_start = [0.0, 0.0, 0.0, 0.0]
    transformed_start = skew(logical_start, fixture["skew"])
    physical_start = list(transformed_start)
    physical_start[2] += mesh.calc_z(physical_start[0], physical_start[1])
    endpoints = []
    speeds = []
    for item in fixture["moves"]:
        transformed_end = skew(item["end"], fixture["skew"])
        splitter.build_move(transformed_start, transformed_end, 1.0)
        while not splitter.traverse_complete:
            endpoint = splitter.split()
            if endpoint is not None:
                endpoints.append(endpoint)
                speeds.append(item["speed"])
        logical_start = item["end"]
        transformed_start = transformed_end

    toolhead = klipper_reference.ReferenceToolhead(fixture["limits"])
    queue = toolhead_module.LookAheadQueue()
    emitted = []
    start = physical_start
    for endpoint, speed in zip(endpoints, speeds):
        move = toolhead_module.Move(toolhead, start, endpoint, speed)
        if queue.add_move(move):
            emitted.extend(queue.flush(lazy=True))
        start = endpoint
    moves = emitted + queue.flush()
    print(json.dumps({
        "name": fixture["name"],
        "endpoints": endpoints,
        "moves": [klipper_reference.phase(move) for move in moves],
    }))


if __name__ == "__main__":
    main()
