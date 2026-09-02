use std::collections::BTreeMap;

use lib_klipper::firmware_retraction::FirmwareRetractionOptions;
use lib_klipper::gcode::parse_gcode;
use lib_klipper::planner::{
    ExtruderLimits, Planner, PlannerDiagnosticCode, PlanningMove, PlanningOperation, PrinterLimits,
};

fn extruder(
    nozzle: f64,
    max_velocity: f64,
    max_accel: f64,
    max_distance: f64,
    corner_velocity: f64,
) -> ExtruderLimits {
    ExtruderLimits {
        nozzle_diameter: nozzle,
        filament_diameter: 1.75,
        max_extrude_only_velocity: max_velocity,
        max_extrude_only_accel: max_accel,
        max_extrude_only_distance: max_distance,
        instantaneous_corner_velocity: corner_velocity,
        max_extrude_cross_section: 4.0 * nozzle * nozzle,
    }
}

fn planner_with_extruders() -> Planner {
    let limits = PrinterLimits {
        extruders: BTreeMap::from([
            ("extruder".into(), extruder(0.4, 5.0, 20.0, 10.0, 1.0)),
            ("extruder1".into(), extruder(0.6, 12.0, 60.0, 20.0, 2.0)),
        ]),
        ..PrinterLimits::default()
    };
    Planner::from_limits(limits)
}

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

// Semantics copied from klippy/kinematics/extruder.py:cmd_ACTIVATE_EXTRUDER at
// Klipper f0892d82b0f1c1228454f09eb508eddde2250f4b.
#[test]
fn tool_changes_preserve_independent_e_positions_and_move_identity() {
    let mut planner = planner_with_extruders();
    process(
        &mut planner,
        &[
            "M83",
            "G1 E5 F600",
            "ACTIVATE_EXTRUDER EXTRUDER=extruder1",
            "G1 E2 F600",
            "ACTIVATE_EXTRUDER EXTRUDER=extruder",
            "G1 E1 F600",
        ],
    );

    assert_eq!(planner.toolhead_state.active_extruder(), Some("extruder"));
    assert_eq!(planner.toolhead_state.position.w, 6.0);
    let moves = moves(&mut planner);
    let names: Vec<_> = moves
        .iter()
        .map(|move_| planner.move_extruder_name(move_).unwrap())
        .collect();
    assert_eq!(names, ["extruder", "extruder1", "extruder"]);
    assert_eq!(moves[0].end.w, 5.0);
    assert_eq!(moves[1].start.w, 0.0);
    assert_eq!(moves[1].end.w, 2.0);
    assert_eq!(moves[2].start.w, 5.0);
    assert_eq!(moves[2].end.w, 6.0);
}

#[test]
fn active_tool_limits_apply_to_extrude_only_and_combined_z_e_moves() {
    let mut planner = planner_with_extruders();
    process(&mut planner, &["M83", "G1 Z10 E20 F6000"]);
    let primary = moves(&mut planner).remove(0);
    assert_eq!(primary.max_cruise_v2, 6.25); // 5mm/s E divided by E/Z ratio 2.
    assert_eq!(primary.acceleration, 10.0);

    let mut planner = planner_with_extruders();
    process(
        &mut planner,
        &[
            "M83",
            "ACTIVATE_EXTRUDER EXTRUDER=extruder1",
            "G1 Z10 E20 F6000",
        ],
    );
    let secondary = moves(&mut planner).remove(0);
    assert_eq!(secondary.max_cruise_v2, 36.0);
    assert_eq!(secondary.acceleration, 30.0);
}

#[test]
fn each_tool_uses_its_own_instantaneous_corner_velocity() {
    fn junction_for(name: &str) -> f64 {
        let mut limits = planner_with_extruders().toolhead_state.limits.clone();
        limits.initial_extruder = Some(name.into());
        let mut planner = Planner::from_limits(limits);
        process(&mut planner, &["M83", "G1 X10 E1 F6000", "G1 X20 E2 F6000"]);
        moves(&mut planner)[1].max_start_v2
    }

    assert_eq!(junction_for("extruder"), 100.0);
    assert_eq!(junction_for("extruder1"), 400.0);
}

#[test]
fn invalid_extrusion_reports_the_same_classes_klipper_rejects() {
    let mut planner = planner_with_extruders();
    process(
        &mut planner,
        &[
            "M83",
            "G1 E11 F600",
            "G1 X10 E10 F600",
            "ACTIVATE_EXTRUDER EXTRUDER=missing",
        ],
    );
    let codes: Vec<_> = planner
        .diagnostics()
        .iter()
        .map(|diagnostic| diagnostic.code.clone())
        .collect();
    assert_eq!(
        codes,
        [
            PlannerDiagnosticCode::ExtrudeOnlyMoveTooLong,
            PlannerDiagnosticCode::MoveExceedsMaximumExtrusion,
            PlannerDiagnosticCode::UnknownExtruder,
        ]
    );
}

// Semantics copied from klippy/extras/firmware_retraction.py at the pinned
// reference commit: G10 emits negative E, G11 emits positive E, and
// SET_RETRACTION clears the retracted latch.
#[test]
fn firmware_retraction_has_signed_moves_and_resets_its_latch() {
    let mut limits = planner_with_extruders().toolhead_state.limits.clone();
    limits.firmware_retraction = Some(FirmwareRetractionOptions {
        retract_length: 2.0,
        unretract_extra_length: 0.5,
        retract_speed: 20.0,
        unretract_speed: 10.0,
        lift_z: 0.0,
    });
    let mut planner = Planner::from_limits(limits);
    process(
        &mut planner,
        &[
            "G10",
            "G11",
            "G10",
            "SET_RETRACTION RETRACT_LENGTH=1",
            "G10",
        ],
    );
    let deltas: Vec<_> = moves(&mut planner)
        .iter()
        .map(|move_| move_.end.w - move_.start.w)
        .collect();
    assert_eq!(deltas, [-2.0, 2.5, -2.0, -1.0]);
}
