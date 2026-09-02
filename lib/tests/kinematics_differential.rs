use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use lib_klipper::gcode::parse_gcode;
use lib_klipper::glam::{DVec3, DVec4};
use lib_klipper::kinematics::{
    CartesianKinematics, CartesianKinematicsKind, DeltaKinematics, DeltesianKinematics, Kinematics,
    PolarKinematics, RotaryDeltaKinematics,
};
use lib_klipper::planner::{Planner, PlannerDiagnosticCode, PlanningOperation, PrinterLimits};
use serde::Deserialize;

const PINNED_KLIPPER_COMMIT: &str = "f0892d82b0f1c1228454f09eb508eddde2250f4b";

#[derive(Deserialize)]
struct Fixture {
    name: String,
    limits: FixtureLimits,
    cases: Vec<FixtureCase>,
}

#[derive(Deserialize)]
struct FixtureLimits {
    max_velocity: f64,
    max_acceleration: f64,
    minimum_cruise_ratio: f64,
    square_corner_velocity: f64,
    #[serde(default)]
    axis_minimum: [f64; 3],
    #[serde(default)]
    axis_maximum: [f64; 3],
    #[serde(default)]
    max_z_velocity: f64,
    #[serde(default)]
    max_z_accel: f64,
}

#[derive(Deserialize)]
struct FixtureCase {
    backend: String,
    case: String,
    start: [f64; 4],
    end: [f64; 4],
    speed: f64,
}

#[derive(Deserialize)]
struct Reference {
    name: String,
    cases: Vec<ReferenceCase>,
}

#[derive(Deserialize)]
struct ReferenceCase {
    backend: String,
    case: String,
    rejected: bool,
    max_velocity: f64,
    acceleration: f64,
}

#[derive(Deserialize)]
struct NonlinearFixture {
    name: String,
    limits: FixtureLimits,
    backends: serde_json::Map<String, serde_json::Value>,
    cases: Vec<FixtureCase>,
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}

fn kind(name: &str) -> CartesianKinematicsKind {
    match name {
        "cartesian" => CartesianKinematicsKind::Cartesian,
        "corexy" => CartesianKinematicsKind::Corexy,
        "corexz" => CartesianKinematicsKind::Corexz,
        "hybrid_corexy" => CartesianKinematicsKind::HybridCorexy,
        "hybrid_corexz" => CartesianKinematicsKind::HybridCorexz,
        _ => panic!("unexpected Cartesian-family backend: {}", name),
    }
}

fn assert_close(context: &str, actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1.0e-8,
        "{context}: expected {:.12}, got {:.12}",
        expected,
        actual
    );
}

// Differential coverage for check_move in Klipper's cartesian.py, corexy.py,
// corexz.py, hybrid_corexy.py, and hybrid_corexz.py at the pinned commit.
#[test]
fn cartesian_family_matches_pinned_klipper_check_move() {
    let root = repo_root();
    let klipper = std::env::var_os("KLIPPER_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("klipper"));
    if !klipper.join("klippy/kinematics/cartesian.py").is_file() {
        eprintln!(
            "skipping kinematics differential: set KLIPPER_PATH to Klipper {PINNED_KLIPPER_COMMIT}"
        );
        return;
    }

    let fixture_path = root.join("tests/fixtures/kinematics/cartesian_family.json");
    let fixture: Fixture = serde_json::from_slice(&fs::read(&fixture_path).unwrap()).unwrap();
    let output = Command::new("python3")
        .arg(root.join("tests/klipper_kinematics_reference.py"))
        .arg(&klipper)
        .arg(&fixture_path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "Klipper kinematics reference failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let reference: Reference = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(reference.name, fixture.name);
    assert_eq!(reference.cases.len(), fixture.cases.len());

    for (item, expected) in fixture.cases.iter().zip(&reference.cases) {
        assert_eq!(expected.backend, item.backend);
        assert_eq!(expected.case, item.case);
        let mut limits = PrinterLimits {
            max_velocity: fixture.limits.max_velocity,
            max_acceleration: fixture.limits.max_acceleration,
            square_corner_velocity: fixture.limits.square_corner_velocity,
            kinematics: Kinematics::CartesianFamily {
                config: CartesianKinematics {
                    kind: kind(&item.backend),
                    axis_minimum: DVec3::from(fixture.limits.axis_minimum),
                    axis_maximum: DVec3::from(fixture.limits.axis_maximum),
                    max_z_velocity: fixture.limits.max_z_velocity,
                    max_z_accel: fixture.limits.max_z_accel,
                },
            },
            ..PrinterLimits::default()
        };
        limits.set_minimum_cruise_ratio(fixture.limits.minimum_cruise_ratio);
        let mut planner = Planner::from_limits(limits);
        planner.toolhead_state.position = DVec4::from(item.start);
        planner.process_cmd(
            &parse_gcode(&format!(
                "G1 X{} Y{} Z{} F{}",
                item.end[0],
                item.end[1],
                item.end[2],
                item.speed * 60.0
            ))
            .unwrap(),
        );
        planner.finalize();
        let move_ = planner
            .iter()
            .find_map(|operation| match operation {
                PlanningOperation::Move(move_) => Some(move_),
                _ => None,
            })
            .unwrap();
        let rejected = planner
            .diagnostics()
            .iter()
            .any(|diagnostic| diagnostic.code == PlannerDiagnosticCode::MoveOutsideKinematicBounds);
        assert_eq!(
            rejected, expected.rejected,
            "{} {}",
            item.backend, item.case
        );
        assert_close(
            &format!("{} {} max velocity", item.backend, item.case),
            move_.max_cruise_v2.sqrt(),
            expected.max_velocity,
        );
        assert_close(
            &format!("{} {} acceleration", item.backend, item.case),
            move_.acceleration,
            expected.acceleration,
        );
    }
}

#[test]
fn unsupported_backend_is_a_structured_diagnostic() {
    let planner = Planner::from_limits(PrinterLimits {
        kinematics: Kinematics::unsupported("delta", "backend is not modeled"),
        ..PrinterLimits::default()
    });
    assert_eq!(planner.diagnostics().len(), 1);
    assert_eq!(
        planner.diagnostics()[0].code,
        PlannerDiagnosticCode::UnsupportedKinematics
    );
    assert!(planner.diagnostics()[0].message.contains("delta"));
}

fn nonlinear_backend(fixture: &NonlinearFixture, backend: &str) -> Kinematics {
    let value = fixture.backends.get(backend).unwrap().clone();
    match backend {
        "delta" => Kinematics::Delta {
            config: serde_json::from_value::<DeltaKinematics>(value).unwrap(),
        },
        "polar" => Kinematics::Polar {
            config: serde_json::from_value::<PolarKinematics>(value).unwrap(),
        },
        "deltesian" => Kinematics::Deltesian {
            config: serde_json::from_value::<DeltesianKinematics>(value).unwrap(),
        },
        "rotary_delta" => Kinematics::RotaryDelta {
            config: serde_json::from_value::<RotaryDeltaKinematics>(value).unwrap(),
        },
        _ => panic!("unexpected non-linear backend: {}", backend),
    }
}

// Differential coverage for check_move in Klipper's delta.py, polar.py,
// deltesian.py, and rotary_delta.py at the pinned commit.
#[test]
fn nonlinear_backends_match_pinned_klipper_check_move() {
    let root = repo_root();
    let klipper = std::env::var_os("KLIPPER_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("klipper"));
    if !klipper.join("klippy/kinematics/delta.py").is_file() {
        eprintln!(
            "skipping kinematics differential: set KLIPPER_PATH to Klipper {PINNED_KLIPPER_COMMIT}"
        );
        return;
    }

    let fixture_path = root.join("tests/fixtures/kinematics/nonlinear.json");
    let fixture: NonlinearFixture =
        serde_json::from_slice(&fs::read(&fixture_path).unwrap()).unwrap();
    let output = Command::new("python3")
        .arg(root.join("tests/klipper_nonlinear_kinematics_reference.py"))
        .arg(&klipper)
        .arg(&fixture_path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "Klipper non-linear kinematics reference failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let reference: Reference = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(reference.name, fixture.name);
    assert_eq!(reference.cases.len(), fixture.cases.len());

    for (item, expected) in fixture.cases.iter().zip(&reference.cases) {
        assert_eq!(expected.backend, item.backend);
        assert_eq!(expected.case, item.case);
        let mut limits = PrinterLimits {
            max_velocity: fixture.limits.max_velocity,
            max_acceleration: fixture.limits.max_acceleration,
            square_corner_velocity: fixture.limits.square_corner_velocity,
            kinematics: nonlinear_backend(&fixture, &item.backend),
            ..PrinterLimits::default()
        };
        limits.set_minimum_cruise_ratio(fixture.limits.minimum_cruise_ratio);
        let mut planner = Planner::from_limits(limits);
        planner.toolhead_state.position = DVec4::from(item.start);
        planner.process_cmd(
            &parse_gcode(&format!(
                "G1 X{} Y{} Z{} F{}",
                item.end[0],
                item.end[1],
                item.end[2],
                item.speed * 60.0
            ))
            .unwrap(),
        );
        planner.finalize();
        let move_ = planner
            .iter()
            .find_map(|operation| match operation {
                PlanningOperation::Move(move_) => Some(move_),
                _ => None,
            })
            .unwrap();
        let rejected = planner
            .diagnostics()
            .iter()
            .any(|diagnostic| diagnostic.code == PlannerDiagnosticCode::MoveOutsideKinematicBounds);
        assert_eq!(
            rejected, expected.rejected,
            "{} {}",
            item.backend, item.case
        );
        assert_close(
            &format!("{} {} max velocity", item.backend, item.case),
            move_.max_cruise_v2.sqrt(),
            expected.max_velocity,
        );
        assert_close(
            &format!("{} {} acceleration", item.backend, item.case),
            move_.acceleration,
            expected.acceleration,
        );
    }
}
