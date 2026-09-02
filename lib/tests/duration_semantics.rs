use lib_klipper::gcode::parse_gcode;
use lib_klipper::planner::{
    CommandContract, ContractState, Delay, DurationContractCategory, Planner,
    PlannerDiagnosticCode, PlanningOperation, PositionMode, PrinterLimits, UnknownDurationCategory,
};

#[test]
fn command_contract_applies_declared_duration_and_resulting_state() {
    let mut limits = PrinterLimits::default();
    limits.command_contracts.insert(
        "PRINT_START".into(),
        CommandContract {
            duration: 12.5,
            category: DurationContractCategory::Macro,
            state: ContractState {
                coordinate_mode: Some(PositionMode::Absolute),
                extrusion_mode: Some(PositionMode::Relative),
                gcode_position: Some([10.0, 20.0, 30.0, 0.0]),
                physical_position: Some([110.0, 120.0, 130.0, 0.0]),
                speed_factor_percent: Some(80.0),
                extrusion_factor_percent: Some(95.0),
                active_extruder: None,
            },
        },
    );
    let mut planner = Planner::from_limits(limits);

    planner.process_cmd(&parse_gcode("PRINT_START BED=60 EXTRUDER=200").unwrap());
    planner.finalize();

    assert_eq!(
        planner.toolhead_state.position.to_array(),
        [110.0, 120.0, 130.0, 0.0]
    );
    assert_eq!(
        planner.toolhead_state.gcode_position().to_array(),
        [10.0, 20.0, 30.0, 0.0]
    );
    assert_eq!(
        planner.toolhead_state.position_modes[3],
        PositionMode::Relative
    );
    assert_eq!(planner.toolhead_state.speed_factor, 0.8 / 60.0);
    assert_eq!(planner.toolhead_state.extrude_factor, 0.95);

    let operations = planner.iter().collect::<Vec<_>>();
    assert!(matches!(
        operations.as_slice(),
        [PlanningOperation::Delay(Delay::Contract {
            duration,
            category: DurationContractCategory::Macro,
            ..
        })] if duration.as_secs_f64() == 12.5
    ));
}

#[test]
fn unknown_waits_and_macros_are_omitted_instead_of_receiving_fake_time() {
    let mut planner = Planner::from_limits(PrinterLimits::default());
    planner.process_cmd(&parse_gcode("M109 S200").unwrap());
    planner.process_cmd(&parse_gcode("PRINT_START").unwrap());
    planner.finalize();

    let operations = planner.iter().collect::<Vec<_>>();
    assert!(operations.iter().any(|operation| matches!(
        operation,
        PlanningOperation::Delay(Delay::Unknown {
            command,
            category: UnknownDurationCategory::TemperatureWait,
        }) if command == "M109"
    )));
    assert!(operations.iter().any(|operation| matches!(
        operation,
        PlanningOperation::Delay(Delay::Unknown {
            command,
            category: UnknownDurationCategory::CommandOrMacro,
        }) if command == "PRINT_START"
    )));
    assert!(planner
        .diagnostics()
        .iter()
        .any(|diagnostic| diagnostic.code == PlannerDiagnosticCode::UnknownDuration));
}
