use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use lib_klipper::gcode::parse_gcode;
use lib_klipper::planner::{
    ExtruderLimits, FirmwareRetractionOptions, Planner, PlannerDiagnosticCode, PlanningOperation,
    PrinterLimits,
};
use serde::Deserialize;

const PINNED_KLIPPER_COMMIT: &str = "f0892d82b0f1c1228454f09eb508eddde2250f4b";

#[derive(Deserialize)]
struct Fixture {
    name: String,
    printer: PrinterFixture,
    extruder: ExtruderLimits,
    moves: Vec<MoveFixture>,
    junction: Vec<MoveFixture>,
}

#[derive(Deserialize)]
struct PrinterFixture {
    max_velocity: f64,
    max_acceleration: f64,
    minimum_cruise_ratio: f64,
    square_corner_velocity: f64,
}

#[derive(Deserialize)]
struct MoveFixture {
    name: Option<String>,
    end: [f64; 4],
    speed: f64,
}

#[derive(Deserialize)]
struct Reference {
    name: String,
    moves: Vec<ReferenceMove>,
    junction_v2: f64,
    firmware_deltas: Vec<f64>,
}

#[derive(Deserialize)]
struct ReferenceMove {
    name: String,
    max_cruise_v2: f64,
    acceleration: f64,
    error: Option<String>,
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}

fn limits(fixture: &Fixture) -> PrinterLimits {
    let mut limits = PrinterLimits {
        max_velocity: fixture.printer.max_velocity,
        max_acceleration: fixture.printer.max_acceleration,
        square_corner_velocity: fixture.printer.square_corner_velocity,
        extruders: BTreeMap::from([("extruder".into(), fixture.extruder.clone())]),
        ..PrinterLimits::default()
    };
    limits.set_minimum_cruise_ratio(fixture.printer.minimum_cruise_ratio);
    limits.set_square_corner_velocity(fixture.printer.square_corner_velocity);
    limits
}

fn assert_close(context: &str, actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1.0e-8,
        "{}: expected {:.12}, got {:.12}",
        context,
        expected,
        actual
    );
}

// Differential coverage for PrinterExtruder.check_move and calc_junction in
// klippy/kinematics/extruder.py at the pinned reference commit.
#[test]
fn extruder_limits_match_pinned_klipper() {
    let root = repo_root();
    let klipper = std::env::var_os("KLIPPER_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("klipper"));
    if !klipper.join("klippy/kinematics/extruder.py").is_file() {
        eprintln!(
            "skipping extruder differential: set KLIPPER_PATH to Klipper {PINNED_KLIPPER_COMMIT}"
        );
        return;
    }
    let fixture_path = root.join("tests/fixtures/extruders/moves.json");
    let fixture: Fixture = serde_json::from_slice(&fs::read(&fixture_path).unwrap()).unwrap();
    let output = Command::new("python3")
        .arg(root.join("tests/klipper_extruder_reference.py"))
        .arg(&klipper)
        .arg(&fixture_path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "Klipper extruder reference failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let reference: Reference = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(reference.name, fixture.name);

    for (item, expected) in fixture.moves.iter().zip(&reference.moves) {
        assert_eq!(item.name.as_deref(), Some(expected.name.as_str()));
        let mut planner = Planner::from_limits(limits(&fixture));
        let line = format!(
            "G1 X{} Y{} Z{} E{} F{}",
            item.end[0],
            item.end[1],
            item.end[2],
            item.end[3],
            item.speed * 60.0
        );
        planner.process_cmd(&parse_gcode(&line).unwrap());
        planner.finalize();
        let move_ = planner
            .iter()
            .find_map(|operation| match operation {
                PlanningOperation::Move(move_) => Some(move_),
                _ => None,
            })
            .unwrap();
        assert_close(
            &format!("{} velocity", expected.name),
            move_.max_cruise_v2,
            expected.max_cruise_v2,
        );
        assert_close(
            &format!("{} acceleration", expected.name),
            move_.acceleration,
            expected.acceleration,
        );
        let has_cross_section_error = planner.diagnostics().iter().any(|diagnostic| {
            diagnostic.code == PlannerDiagnosticCode::MoveExceedsMaximumExtrusion
        });
        assert_eq!(has_cross_section_error, expected.error.is_some());
    }

    let mut planner = Planner::from_limits(limits(&fixture));
    let mut previous_e = 0.0;
    for item in &fixture.junction {
        let line = format!(
            "G1 X{} Y{} Z{} E{} F{}",
            item.end[0],
            item.end[1],
            item.end[2],
            item.end[3] - previous_e,
            item.speed * 60.0
        );
        planner.process_cmd(&parse_gcode(&line).unwrap());
        previous_e = item.end[3];
    }
    planner.finalize();
    let moves: Vec<_> = planner
        .iter()
        .filter_map(|operation| match operation {
            PlanningOperation::Move(move_) => Some(move_),
            _ => None,
        })
        .collect();
    assert_close(
        "extruder junction",
        moves[1].max_start_v2,
        reference.junction_v2,
    );

    let mut retraction_limits = limits(&fixture);
    retraction_limits.firmware_retraction = Some(FirmwareRetractionOptions {
        retract_length: 2.0,
        retract_speed: 20.0,
        unretract_extra_length: 0.5,
        unretract_speed: 10.0,
        lift_z: 0.0,
    });
    let mut planner = Planner::from_limits(retraction_limits);
    for command in [
        "G10",
        "G11",
        "G10",
        "SET_RETRACTION RETRACT_LENGTH=1",
        "G10",
    ] {
        planner.process_cmd(&parse_gcode(command).unwrap());
    }
    planner.finalize();
    let deltas: Vec<_> = planner
        .iter()
        .filter_map(|operation| match operation {
            PlanningOperation::Move(move_) => Some(move_.delta().w),
            _ => None,
        })
        .collect();
    assert_eq!(deltas, reference.firmware_deltas);
}
