use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use lib_klipper::gcode::parse_gcode;
use lib_klipper::planner::{Planner, PlanningMove, PlanningOperation, PositionMode, PrinterLimits};
use serde::Deserialize;

const ABS_TOLERANCE: f64 = 1.0e-8;
const REL_TOLERANCE: f64 = 1.0e-8;
const PINNED_KLIPPER_COMMIT: &str = "f0892d82b0f1c1228454f09eb508eddde2250f4b";

#[derive(Debug, Deserialize)]
struct Fixture {
    name: String,
    limits: FixtureLimits,
    moves: Vec<FixtureMove>,
    #[serde(default)]
    known_difference_tolerances: BTreeMap<String, f64>,
}

#[derive(Debug, Deserialize)]
struct FixtureLimits {
    max_velocity: f64,
    max_acceleration: f64,
    minimum_cruise_ratio: f64,
    square_corner_velocity: f64,
}

#[derive(Debug, Deserialize)]
struct FixtureMove {
    end: [f64; 4],
    speed: f64,
    accel: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct ReferenceOutput {
    name: String,
    moves: Vec<NormalizedMove>,
}

#[derive(Debug, Deserialize)]
struct NormalizedMove {
    distance: f64,
    start_v: f64,
    cruise_v: f64,
    end_v: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    total_t: f64,
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}

// `Option::is_some_and` would raise the documented Rust 1.58 MSRV.
#[allow(clippy::unnecessary_map_or)]
fn fixture_paths() -> Vec<PathBuf> {
    let mut paths: Vec<_> = fs::read_dir(repo_root().join("tests/fixtures/accuracy"))
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().map_or(false, |ext| ext == "json"))
        .collect();
    paths.sort();
    paths
}

fn load_fixture(path: &Path) -> Fixture {
    serde_json::from_slice(&fs::read(path).unwrap()).unwrap()
}

fn planner_for(fixture: &Fixture) -> Planner {
    let mut limits = PrinterLimits {
        max_velocity: fixture.limits.max_velocity,
        max_acceleration: fixture.limits.max_acceleration,
        square_corner_velocity: fixture.limits.square_corner_velocity,
        ..PrinterLimits::default()
    };
    limits.set_minimum_cruise_ratio(fixture.limits.minimum_cruise_ratio);
    limits.set_square_corner_velocity(fixture.limits.square_corner_velocity);
    Planner::from_limits(limits)
}

fn estimator_moves(fixture: &Fixture) -> Vec<PlanningMove> {
    let mut planner = planner_for(fixture);
    let mut previous_e = 0.0;
    for item in &fixture.moves {
        if let Some(accel) = item.accel {
            planner.process_cmd(&parse_gcode(&format!("M204 S{accel}")).unwrap());
        }
        let relative_e = item.end[3] - previous_e;
        let line = format!(
            "G1 X{} Y{} Z{} E{} F{}",
            item.end[0],
            item.end[1],
            item.end[2],
            relative_e,
            item.speed * 60.0
        );
        planner.process_cmd(&parse_gcode(&line).unwrap());
        previous_e = item.end[3];
    }
    planner.finalize();
    planner
        .iter()
        .filter_map(|operation| match operation {
            PlanningOperation::Move(move_) => Some(move_),
            _ => None,
        })
        .collect()
}

fn normalized(move_: &PlanningMove) -> NormalizedMove {
    NormalizedMove {
        distance: move_.distance,
        start_v: move_.start_v,
        cruise_v: move_.cruise_v,
        end_v: move_.end_v,
        accel_t: move_.accel_time(),
        cruise_t: move_.cruise_time(),
        decel_t: move_.decel_time(),
        total_t: move_.total_time(),
    }
}

fn assert_close_with_tolerance(context: &str, actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{}: expected {:.12}, got {:.12} (tolerance {:.3e})",
        context,
        expected,
        actual,
        tolerance
    );
}

fn assert_close(context: &str, actual: f64, expected: f64) {
    let tolerance = ABS_TOLERANCE.max(REL_TOLERANCE * actual.abs().max(expected.abs()));
    assert_close_with_tolerance(context, actual, expected, tolerance);
}

fn assert_same_move(
    fixture: &str,
    index: usize,
    actual: &NormalizedMove,
    expected: &NormalizedMove,
    known_difference_tolerances: &BTreeMap<String, f64>,
) {
    for (field, actual, expected) in [
        ("distance", actual.distance, expected.distance),
        ("start_v", actual.start_v, expected.start_v),
        ("cruise_v", actual.cruise_v, expected.cruise_v),
        ("end_v", actual.end_v, expected.end_v),
        ("accel_t", actual.accel_t, expected.accel_t),
        ("cruise_t", actual.cruise_t, expected.cruise_t),
        ("decel_t", actual.decel_t, expected.decel_t),
        ("total_t", actual.total_t, expected.total_t),
    ] {
        let default_tolerance = ABS_TOLERANCE.max(REL_TOLERANCE * actual.abs().max(expected.abs()));
        let tolerance = known_difference_tolerances
            .get(field)
            .copied()
            .unwrap_or(default_tolerance);
        assert_close_with_tolerance(
            &format!("{fixture} move {index} {field}"),
            actual,
            expected,
            tolerance,
        );
    }
}

#[test]
fn initial_coordinate_modes_are_an_explicit_compatibility_baseline() {
    let planner = Planner::from_limits(PrinterLimits::default());
    assert_eq!(
        planner.toolhead_state.position_modes[0],
        PositionMode::Absolute
    );
    assert_eq!(
        planner.toolhead_state.position_modes[1],
        PositionMode::Absolute
    );
    assert_eq!(
        planner.toolhead_state.position_modes[2],
        PositionMode::Absolute
    );
    assert_eq!(
        planner.toolhead_state.position_modes[3],
        PositionMode::Relative
    );
}

#[test]
fn individual_move_and_total_time_baseline() {
    let fixture = load_fixture(&repo_root().join("tests/fixtures/accuracy/long_moves.json"));
    let moves = estimator_moves(&fixture);
    assert_eq!(moves.len(), 2);
    assert_close("first move distance", moves[0].distance, 100.0);
    assert_close("continuous junction", moves[0].end_v, 120.0);
    assert_close("next continuous junction", moves[1].start_v, 120.0);
    let total: f64 = moves.iter().map(PlanningMove::total_time).sum();
    assert_close("long move total", total, 1.816_666_666_666_666_7);
}

#[test]
fn short_junction_reversal_and_dynamic_limit_baseline() {
    let corner_fixture =
        load_fixture(&repo_root().join("tests/fixtures/accuracy/short_corner_reversal.json"));
    let corner_moves = estimator_moves(&corner_fixture);
    assert_eq!(corner_moves.len(), 4);
    assert_close("right-angle junction", corner_moves[0].end_v, 5.0);
    assert_close("reversal stop", corner_moves[2].end_v, 0.0);
    assert_close(
        "short-corner total",
        corner_moves.iter().map(PlanningMove::total_time).sum(),
        0.330_347_190_615_825_8,
    );

    let dynamic_fixture =
        load_fixture(&repo_root().join("tests/fixtures/accuracy/dynamic_acceleration.json"));
    let dynamic_moves = estimator_moves(&dynamic_fixture);
    assert_eq!(dynamic_moves.len(), 3);
    assert_close("dynamic-limit junction", dynamic_moves[1].end_v, 5.0);
    assert_close(
        "dynamic-limit cruise",
        dynamic_moves[2].cruise_v,
        77.540_312_096_354_11,
    );
}

#[test]
fn pinned_klipper_differential_baseline() {
    let root = repo_root();
    let klipper = std::env::var_os("KLIPPER_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("klipper"));
    if !klipper.join("klippy/toolhead.py").is_file() {
        eprintln!(
            "skipping differential baseline: set KLIPPER_PATH to Klipper {PINNED_KLIPPER_COMMIT}"
        );
        return;
    }

    for fixture_path in fixture_paths() {
        let fixture = load_fixture(&fixture_path);
        let output = Command::new("python3")
            .arg(root.join("tests/klipper_reference.py"))
            .arg(&klipper)
            .arg(&fixture_path)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "Klipper reference failed for {}:\n{}",
            fixture.name,
            String::from_utf8_lossy(&output.stderr)
        );
        let reference: ReferenceOutput = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(reference.name, fixture.name);
        let actual: Vec<_> = estimator_moves(&fixture).iter().map(normalized).collect();
        assert_eq!(
            actual.len(),
            reference.moves.len(),
            "{} move count",
            fixture.name
        );
        for (index, (actual, expected)) in actual.iter().zip(&reference.moves).enumerate() {
            assert_same_move(
                &fixture.name,
                index,
                actual,
                expected,
                &fixture.known_difference_tolerances,
            );
        }
    }
}
