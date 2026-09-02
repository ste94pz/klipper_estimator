use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use lib_klipper::gcode::parse_gcode;
use lib_klipper::motion_transform::{
    BedMeshConfig, BedMeshProfile, MotionTransformConfig, SkewCorrectionConfig, SkewFactors,
};
use lib_klipper::planner::{
    Planner, PlannerDiagnosticCode, PlanningMove, PlanningOperation, PrinterLimits,
};
use serde::Deserialize;

const PINNED_KLIPPER_COMMIT: &str = "f0892d82b0f1c1228454f09eb508eddde2250f4b";

#[derive(Deserialize)]
struct Fixture {
    name: String,
    limits: Limits,
    bed_mesh: Mesh,
    skew: SkewFactors,
    moves: Vec<FixtureMove>,
}

#[derive(Deserialize)]
struct Limits {
    max_velocity: f64,
    max_acceleration: f64,
    minimum_cruise_ratio: f64,
    square_corner_velocity: f64,
}

#[derive(Deserialize)]
struct Mesh {
    min: [f64; 2],
    max: [f64; 2],
    points: Vec<Vec<f64>>,
    mesh_pps: [usize; 2],
    algorithm: String,
    tension: f64,
    split_delta_z: f64,
    move_check_distance: f64,
}

#[derive(Deserialize)]
struct FixtureMove {
    end: [f64; 4],
    speed: f64,
}

#[derive(Deserialize)]
struct Reference {
    name: String,
    endpoints: Vec<[f64; 4]>,
    moves: Vec<ReferenceMove>,
}

#[derive(Deserialize)]
struct ReferenceMove {
    distance: f64,
    start_v: f64,
    cruise_v: f64,
    end_v: f64,
    total_t: f64,
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
        "{}: expected {:.12}, got {:.12}",
        context,
        expected,
        actual
    );
}

fn configured_limits(fixture: &Fixture, active: bool) -> PrinterLimits {
    let profile = BedMeshProfile::from_probed(
        fixture.bed_mesh.min,
        fixture.bed_mesh.max,
        fixture.bed_mesh.points.clone(),
        fixture.bed_mesh.mesh_pps,
        &fixture.bed_mesh.algorithm,
        fixture.bed_mesh.tension,
    )
    .unwrap();
    let mut limits = PrinterLimits {
        max_velocity: fixture.limits.max_velocity,
        max_acceleration: fixture.limits.max_acceleration,
        square_corner_velocity: fixture.limits.square_corner_velocity,
        motion_transforms: MotionTransformConfig {
            bed_mesh: Some(BedMeshConfig {
                profiles: BTreeMap::from([("test".into(), profile)]),
                initial_profile: active.then(|| "test".into()),
                split_delta_z: fixture.bed_mesh.split_delta_z,
                move_check_distance: fixture.bed_mesh.move_check_distance,
                ..BedMeshConfig::default()
            }),
            skew_correction: Some(SkewCorrectionConfig {
                profiles: BTreeMap::from([("test".into(), fixture.skew)]),
                initial_profile: active.then(|| "test".into()),
            }),
            ..MotionTransformConfig::default()
        },
        ..PrinterLimits::default()
    };
    limits.set_minimum_cruise_ratio(fixture.limits.minimum_cruise_ratio);
    limits.set_square_corner_velocity(fixture.limits.square_corner_velocity);
    limits
}

fn run_planner(fixture: &Fixture, active: bool) -> Vec<PlanningMove> {
    let mut planner = Planner::from_limits(configured_limits(fixture, active));
    for item in &fixture.moves {
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

// Differential coverage for extras/bed_mesh.py:ZMesh, BedMesh.move, MoveSplitter,
// extras/skew_correction.py:PrinterSkew, and toolhead.py at the pinned commit.
#[test]
fn bed_mesh_path_and_duration_match_pinned_klipper() {
    let root = repo_root();
    let klipper = std::env::var_os("KLIPPER_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("klipper"));
    if !klipper.join("klippy/extras/bed_mesh.py").is_file() {
        eprintln!("skipping motion-transform differential: set KLIPPER_PATH to Klipper {PINNED_KLIPPER_COMMIT}");
        return;
    }
    for filename in ["bed_mesh.json", "bed_mesh_bicubic.json"] {
        let fixture_path = root.join("tests/fixtures/motion_transforms").join(filename);
        let fixture: Fixture = serde_json::from_slice(&fs::read(&fixture_path).unwrap()).unwrap();
        let output = Command::new("python3")
            .arg(root.join("tests/klipper_motion_transform_reference.py"))
            .arg(&klipper)
            .arg(&fixture_path)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "Klipper motion-transform reference failed:\n{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let reference: Reference = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(reference.name, fixture.name);
        let moves = run_planner(&fixture, true);
        assert_eq!(moves.len(), reference.moves.len());
        assert_eq!(moves.len(), reference.endpoints.len());
        for (index, ((move_, expected), endpoint)) in moves
            .iter()
            .zip(&reference.moves)
            .zip(&reference.endpoints)
            .enumerate()
        {
            for (axis, expected_axis) in endpoint.iter().enumerate() {
                assert_close(
                    &format!("{filename} move {index} endpoint axis {axis}"),
                    move_.end[axis],
                    *expected_axis,
                );
            }
            assert_close(
                &format!("{filename} move {index} distance"),
                move_.distance,
                expected.distance,
            );
            assert_close(
                &format!("{filename} move {index} start"),
                move_.start_v,
                expected.start_v,
            );
            assert_close(
                &format!("{filename} move {index} cruise"),
                move_.cruise_v,
                expected.cruise_v,
            );
            assert_close(
                &format!("{filename} move {index} end"),
                move_.end_v,
                expected.end_v,
            );
            assert_close(
                &format!("{filename} move {index} time"),
                move_.total_time(),
                expected.total_t,
            );
        }
    }
}

#[test]
fn inactive_profiles_leave_path_unchanged_and_commands_change_state() {
    let root = repo_root();
    let fixture: Fixture = serde_json::from_slice(
        &fs::read(root.join("tests/fixtures/motion_transforms/bed_mesh.json")).unwrap(),
    )
    .unwrap();
    let inactive = run_planner(&fixture, false);
    assert_eq!(inactive.len(), fixture.moves.len());
    for (move_, expected) in inactive.iter().zip(&fixture.moves) {
        assert_close("inactive x", move_.end.x, expected.end[0]);
        assert_close("inactive y", move_.end.y, expected.end[1]);
        assert_close("inactive z", move_.end.z, expected.end[2]);
    }

    let mut planner = Planner::from_limits(configured_limits(&fixture, false));
    for command in [
        "BED_MESH_PROFILE LOAD=test",
        "SKEW_PROFILE LOAD=test",
        "G1 X40 Y30 Z0.2 F3600",
        "BED_MESH_CLEAR",
        "SET_SKEW CLEAR=1",
        "G1 X50 Y40 Z0.2 F3600",
        "BED_MESH_PROFILE LOAD=missing",
    ] {
        planner.process_cmd(&parse_gcode(command).unwrap());
    }
    planner.finalize();
    assert!(planner.diagnostics().iter().any(|diagnostic| {
        diagnostic.code == PlannerDiagnosticCode::UnknownMotionTransformProfile
    }));
    let moves: Vec<_> = planner
        .iter()
        .filter_map(|operation| match operation {
            PlanningOperation::Move(move_) => Some(move_),
            _ => None,
        })
        .collect();
    assert!(
        moves.len() > 2,
        "active mesh should split the first logical move"
    );
    let last = moves.last().unwrap();
    assert_close("cleared x", last.end.x, 50.0);
    assert_close("cleared y", last.end.y, 40.0);
    assert_close("cleared z", last.end.z, 0.2);

    let planner = Planner::from_limits(PrinterLimits {
        motion_transforms: MotionTransformConfig {
            unsupported_active: vec!["z_thermal_adjust".into()],
            ..MotionTransformConfig::default()
        },
        ..PrinterLimits::default()
    });
    assert_eq!(
        planner.diagnostics()[0].code,
        PlannerDiagnosticCode::UnsupportedMotionTransform
    );
}
