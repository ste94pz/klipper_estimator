#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
klipper_path=${KLIPPER_PATH:-"$repository_root/klipper"}

if [[ ! -f "$klipper_path/klippy/toolhead.py" ]]; then
    echo "Klipper checkout not found at $klipper_path" >&2
    echo "Set KLIPPER_PATH to commit f0892d82b0f1c1228454f09eb508eddde2250f4b" >&2
    exit 1
fi

KLIPPER_PATH="$klipper_path" cargo test \
    --manifest-path "$repository_root/Cargo.toml" \
    -p lib_klipper \
    --test accuracy_baseline \
    pinned_klipper_differential_baseline \
    -- --exact

KLIPPER_PATH="$klipper_path" cargo test \
    --manifest-path "$repository_root/Cargo.toml" \
    -p lib_klipper \
    --test gcode_state_differential \
    command_sequence_matches_pinned_klipper_gcode_move \
    -- --exact

KLIPPER_PATH="$klipper_path" cargo test \
    --manifest-path "$repository_root/Cargo.toml" \
    -p lib_klipper \
    --test kinematics_differential \
    cartesian_family_matches_pinned_klipper_check_move \
    -- --exact

KLIPPER_PATH="$klipper_path" cargo test \
    --manifest-path "$repository_root/Cargo.toml" \
    -p lib_klipper \
    --test kinematics_differential \
    nonlinear_backends_match_pinned_klipper_check_move \
    -- --exact

KLIPPER_PATH="$klipper_path" cargo test \
    --manifest-path "$repository_root/Cargo.toml" \
    -p lib_klipper \
    --test extruder_differential \
    extruder_limits_match_pinned_klipper \
    -- --exact

KLIPPER_PATH="$klipper_path" cargo test \
    --manifest-path "$repository_root/Cargo.toml" \
    -p lib_klipper \
    --test motion_transform_differential \
    bed_mesh_path_and_duration_match_pinned_klipper \
    -- --exact
