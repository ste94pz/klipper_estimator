use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use lib_klipper::gcode::parse_gcode;
use lib_klipper::planner::{Planner, PlanningOperation, PositionMode, PrinterLimits};
use serde::Deserialize;

const PINNED_KLIPPER_COMMIT: &str = "f0892d82b0f1c1228454f09eb508eddde2250f4b";

#[derive(Deserialize)]
struct Fixture {
    name: String,
    initial_coordinate_mode: PositionMode,
    initial_extrusion_mode: PositionMode,
    limits: FixtureLimits,
    commands: Vec<String>,
}

#[derive(Deserialize)]
struct FixtureLimits {
    max_velocity: f64,
    max_acceleration: f64,
    minimum_cruise_ratio: f64,
    square_corner_velocity: f64,
    instant_corner_velocity: f64,
}

#[derive(Deserialize)]
struct ReferenceMove {
    end: [f64; 4],
    speed: f64,
    total_time: f64,
}

#[derive(Deserialize)]
struct Reference {
    name: String,
    position: [f64; 4],
    base_position: [f64; 4],
    homing_position: [f64; 4],
    gcode_position: [f64; 4],
    speed: f64,
    speed_factor: f64,
    extrude_factor: f64,
    absolute_coordinate: bool,
    absolute_extrusion: bool,
    moves: Vec<ReferenceMove>,
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}

fn assert_close(context: &str, actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1.0e-8,
        "{context}: expected {:.12}, got {:.12}",
        expected,
        actual
    );
}

// Differential coverage for Klipper klippy/extras/gcode_move.py, GCodeMove,
// pinned at f0892d82b0f1c1228454f09eb508eddde2250f4b.
#[test]
fn command_sequence_matches_pinned_klipper_gcode_move() {
    let root = repo_root();
    let klipper = std::env::var_os("KLIPPER_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("klipper"));
    if !klipper.join("klippy/extras/gcode_move.py").is_file() {
        eprintln!(
            "skipping G-code state differential: set KLIPPER_PATH to Klipper {PINNED_KLIPPER_COMMIT}"
        );
        return;
    }

    let fixture_path = root.join("tests/fixtures/gcode_state/state_commands.json");
    let fixture: Fixture = serde_json::from_slice(&fs::read(&fixture_path).unwrap()).unwrap();
    let output = Command::new("python3")
        .arg(root.join("tests/klipper_gcode_state_reference.py"))
        .arg(&klipper)
        .arg(&fixture_path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "Klipper G-code state reference failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let reference: Reference = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(reference.name, fixture.name);

    let mut limits = PrinterLimits {
        max_velocity: fixture.limits.max_velocity,
        max_acceleration: fixture.limits.max_acceleration,
        square_corner_velocity: fixture.limits.square_corner_velocity,
        instant_corner_velocity: fixture.limits.instant_corner_velocity,
        initial_coordinate_mode: fixture.initial_coordinate_mode,
        initial_extrusion_mode: fixture.initial_extrusion_mode,
        ..PrinterLimits::default()
    };
    limits.set_minimum_cruise_ratio(fixture.limits.minimum_cruise_ratio);
    let mut planner = Planner::from_limits(limits);
    for line in &fixture.commands {
        planner.process_cmd(&parse_gcode(line).unwrap());
    }
    planner.finalize();
    let moves: Vec<_> = planner
        .iter()
        .filter_map(|operation| match operation {
            PlanningOperation::Move(move_) => Some(move_),
            _ => None,
        })
        .collect();

    for (name, actual, expected) in [
        (
            "position",
            planner.toolhead_state.position.to_array(),
            reference.position,
        ),
        (
            "base position",
            planner.toolhead_state.base_position.to_array(),
            reference.base_position,
        ),
        (
            "homing position",
            planner.toolhead_state.homing_position.to_array(),
            reference.homing_position,
        ),
        (
            "G-code position",
            planner.toolhead_state.gcode_position().to_array(),
            reference.gcode_position,
        ),
    ] {
        for axis in 0..4 {
            assert_close(&format!("{name} axis {axis}"), actual[axis], expected[axis]);
        }
    }
    assert_close("speed", planner.toolhead_state.velocity, reference.speed);
    assert_close(
        "speed factor",
        planner.toolhead_state.speed_factor,
        reference.speed_factor,
    );
    assert_close(
        "extrude factor",
        planner.toolhead_state.extrude_factor,
        reference.extrude_factor,
    );
    assert_eq!(
        planner.toolhead_state.position_modes[0] == PositionMode::Absolute,
        reference.absolute_coordinate
    );
    assert_eq!(
        planner.toolhead_state.position_modes[3] == PositionMode::Absolute,
        reference.absolute_extrusion
    );
    assert_eq!(moves.len(), reference.moves.len());
    for (index, (actual, expected)) in moves.iter().zip(&reference.moves).enumerate() {
        for axis in 0..4 {
            assert_close(
                &format!("move {index} endpoint axis {axis}"),
                actual.end.to_array()[axis],
                expected.end[axis],
            );
        }
        assert_close(
            &format!("move {index} speed"),
            actual.requested_velocity,
            expected.speed,
        );
        assert_close(
            &format!("move {index} planned time"),
            actual.total_time(),
            expected.total_time,
        );
    }
}
