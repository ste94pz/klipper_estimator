use lib_klipper::gcode::parse_gcode;
use lib_klipper::planner::{
    Planner, PlannerDiagnosticCode, PlanningMove, PlanningOperation, PositionMode, PrinterLimits,
};

fn process(planner: &mut Planner, lines: &[&str]) {
    for line in lines {
        planner.process_cmd(&parse_gcode(line).unwrap());
    }
}

fn moves(planner: &mut Planner) -> Vec<PlanningMove> {
    planner.finalize();
    planner
        .iter()
        .filter_map(|operation| match operation {
            PlanningOperation::Move(move_) => Some(move_),
            _ => None,
        })
        .collect()
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1.0e-9,
        "expected {:.12}, got {:.12}",
        expected,
        actual
    );
}

// Semantics copied from Klipper klippy/extras/gcode_move.py, GCodeMove command handlers,
// reference commit f0892d82b0f1c1228454f09eb508eddde2250f4b.
#[test]
fn coordinate_feed_and_extrusion_overrides_match_klipper_state_transforms() {
    let mut planner = Planner::from_limits(PrinterLimits::default());
    process(
        &mut planner,
        &[
            "G1 X10 E5 F1200",
            "G92 X2 E1",
            "G1 X3 E2",
            "G91",
            "G1 X2",
            "G90",
            "M82",
            "M221 S200",
            "G1 E4",
            "M220 S50",
            "G1 X14",
        ],
    );

    assert_close(planner.toolhead_state.position.x, 22.0);
    assert_close(planner.toolhead_state.position.w, 9.0);
    assert_close(planner.toolhead_state.base_position.x, 8.0);
    assert_close(planner.toolhead_state.base_position.w, 1.0);
    assert_close(planner.toolhead_state.gcode_position().x, 14.0);
    assert_close(planner.toolhead_state.gcode_position().w, 4.0);
    assert_close(planner.toolhead_state.velocity, 10.0);
    assert_close(planner.toolhead_state.speed_factor, 1.0 / 120.0);
    assert_close(planner.toolhead_state.extrude_factor, 2.0);
    assert_eq!(
        planner.toolhead_state.position_modes,
        [PositionMode::Absolute; 4]
    );
}

#[test]
fn saved_state_restore_preserves_current_e_and_optionally_moves_xyz() {
    let mut planner = Planner::from_limits(PrinterLimits {
        initial_extrusion_mode: PositionMode::Absolute,
        ..PrinterLimits::default()
    });
    process(
        &mut planner,
        &[
            "G1 X10 E10 F1200",
            "SAVE_GCODE_STATE NAME=sample",
            "G91",
            "M83",
            "G1 X5 E2 F600",
            "RESTORE_GCODE_STATE NAME=sample MOVE=1 MOVE_SPEED=7",
        ],
    );

    assert_close(planner.toolhead_state.position.x, 10.0);
    assert_close(planner.toolhead_state.position.w, 12.0);
    assert_close(planner.toolhead_state.base_position.w, 2.0);
    assert_close(planner.toolhead_state.gcode_position().w, 10.0);
    assert_close(planner.toolhead_state.velocity, 20.0);
    assert_eq!(
        planner.toolhead_state.position_modes,
        [PositionMode::Absolute; 4]
    );

    let moves = moves(&mut planner);
    let restore_move = moves.last().unwrap();
    assert_close(restore_move.start.x, 15.0);
    assert_close(restore_move.end.x, 10.0);
    assert_close(restore_move.end.w, 12.0);
    assert_close(restore_move.requested_velocity, 7.0);
}

#[test]
fn gcode_offset_preserves_gcode_position_when_move_is_requested() {
    let mut planner = Planner::from_limits(PrinterLimits::default());
    process(
        &mut planner,
        &[
            "G1 X10 F1200",
            "SET_GCODE_OFFSET X=2",
            "G1 X9",
            "SET_GCODE_OFFSET X_ADJUST=1 MOVE=1 MOVE_SPEED=5",
        ],
    );

    assert_close(planner.toolhead_state.position.x, 12.0);
    assert_close(planner.toolhead_state.base_position.x, 3.0);
    assert_close(planner.toolhead_state.homing_position.x, 3.0);
    assert_close(planner.toolhead_state.gcode_position().x, 9.0);
    assert_close(planner.toolhead_state.velocity, 20.0);
    assert_close(moves(&mut planner).last().unwrap().requested_velocity, 5.0);
}

#[test]
fn unsupported_state_changes_and_unknown_restores_are_structured() {
    let mut planner = Planner::from_limits(PrinterLimits::default());
    process(
        &mut planner,
        &[
            "SET_KINEMATIC_POSITION X=10",
            "RESTORE_GCODE_STATE NAME=missing",
        ],
    );

    assert_eq!(planner.diagnostics().len(), 2);
    assert_eq!(
        planner.diagnostics()[0].code,
        PlannerDiagnosticCode::UnsupportedStateCommand
    );
    assert_eq!(planner.diagnostics()[0].command, "SET_KINEMATIC_POSITION");
    assert_eq!(
        planner.diagnostics()[1].code,
        PlannerDiagnosticCode::UnknownSavedGcodeState
    );
}

#[test]
fn initial_modes_are_explicit_and_keep_the_relative_e_compatibility_default() {
    let limits: PrinterLimits = serde_json::from_str("{}").unwrap();
    assert_eq!(limits.initial_coordinate_mode, PositionMode::Absolute);
    assert_eq!(limits.initial_extrusion_mode, PositionMode::Relative);

    let configured: PrinterLimits = serde_json::from_str(
        r#"{"initial_coordinate_mode":"relative","initial_extrusion_mode":"absolute"}"#,
    )
    .unwrap();
    let planner = Planner::from_limits(configured);
    assert_eq!(
        planner.toolhead_state.position_modes,
        [
            PositionMode::Relative,
            PositionMode::Relative,
            PositionMode::Relative,
            PositionMode::Absolute,
        ]
    );
}
