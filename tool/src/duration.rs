use std::collections::BTreeMap;

use lib_klipper::planner::{
    Delay, DurationContractCategory, Planner, PlanningOperation, UnknownDurationCategory,
};
use serde::Serialize;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct OmittedDurationComponent {
    pub command: String,
    pub category: UnknownDurationCategory,
    pub reason: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DurationEstimate {
    pub motion_time: f64,
    pub deterministic_time: f64,
    pub expected_total_time: f64,
    /// Backward-compatible alias used by existing JSON consumers.
    pub total_time: f64,
    pub duration_components: BTreeMap<String, f64>,
    pub omitted_duration_components: Vec<OmittedDurationComponent>,
    #[serde(skip)]
    needs_lookahead_flush: bool,
}

impl Default for DurationEstimate {
    fn default() -> Self {
        Self {
            motion_time: 0.0,
            deterministic_time: 0.0,
            expected_total_time: 0.0,
            total_time: 0.0,
            duration_components: BTreeMap::new(),
            omitted_duration_components: Vec::new(),
            needs_lookahead_flush: true,
        }
    }
}

impl DurationEstimate {
    pub fn add_operation(&mut self, planner: &Planner, operation: &PlanningOperation) {
        match operation {
            PlanningOperation::Move(move_) => {
                if self.needs_lookahead_flush {
                    self.add_deterministic("lookahead_flush", 0.25);
                    self.needs_lookahead_flush = false;
                }
                let duration = move_.total_time();
                self.motion_time += duration;
                self.add_deterministic("motion", duration);
            }
            PlanningOperation::Delay(Delay::Dwell(duration)) => {
                self.add_deterministic("dwell", duration.as_secs_f64());
            }
            PlanningOperation::Delay(Delay::Contract {
                duration,
                command: _,
                category,
            }) => {
                self.add_deterministic(contract_category(*category), duration.as_secs_f64());
                self.needs_lookahead_flush = true;
            }
            PlanningOperation::Delay(Delay::EstimatorAddition { duration, kind }) => {
                let component = planner.kind_str(kind).map_or_else(
                    || "estimator_addition".to_string(),
                    |kind| format!("estimator_addition:{kind}"),
                );
                self.add_expected(&component, duration.as_secs_f64());
                self.needs_lookahead_flush = true;
            }
            PlanningOperation::Delay(Delay::Unknown { command, category }) => {
                let omitted = OmittedDurationComponent {
                    command: command.clone(),
                    category: *category,
                    reason: "duration is unknown and was not included".into(),
                };
                if !self.omitted_duration_components.contains(&omitted) {
                    self.omitted_duration_components.push(omitted);
                }
                self.needs_lookahead_flush = true;
            }
            PlanningOperation::Fill => {}
        }
    }

    pub fn add_deterministic(&mut self, category: &str, duration: f64) {
        self.deterministic_time += duration;
        self.expected_total_time += duration;
        self.total_time = self.expected_total_time;
        *self
            .duration_components
            .entry(category.to_string())
            .or_default() += duration;
    }

    pub(crate) fn add_expected(&mut self, category: &str, duration: f64) {
        self.expected_total_time += duration;
        self.total_time = self.expected_total_time;
        *self
            .duration_components
            .entry(category.to_string())
            .or_default() += duration;
    }
}

fn contract_category(category: DurationContractCategory) -> &'static str {
    match category {
        DurationContractCategory::Macro => "macro_contract",
        DurationContractCategory::Homing => "homing_model",
        DurationContractCategory::Probing => "probing_model",
        DurationContractCategory::Other => "command_contract",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lib_klipper::gcode::parse_gcode;
    use lib_klipper::planner::{CommandContract, ContractState, PrinterLimits};

    #[test]
    fn separates_motion_dwell_expected_additions_and_omissions() {
        let mut limits = PrinterLimits::default();
        limits.command_contracts.insert(
            "PRINT_START".into(),
            CommandContract {
                duration: 4.0,
                category: DurationContractCategory::Macro,
                state: ContractState::default(),
            },
        );
        let mut planner = Planner::from_limits(limits);
        for line in [
            "PRINT_START",
            "G1 X10 F600",
            "G4 P2000",
            "; ESTIMATOR_ADD_TIME 3 measured",
            "M109 S200",
        ] {
            planner.process_cmd(&parse_gcode(line).unwrap());
        }
        planner.finalize();

        let mut estimate = DurationEstimate::default();
        for operation in planner.iter().collect::<Vec<_>>() {
            estimate.add_operation(&planner, &operation);
        }

        assert!(estimate.motion_time > 0.0);
        assert_eq!(
            estimate.deterministic_time,
            estimate.motion_time + 4.0 + 2.0 + 0.25
        );
        assert_eq!(
            estimate.expected_total_time,
            estimate.deterministic_time + 3.0
        );
        assert_eq!(estimate.total_time, estimate.expected_total_time);
        assert_eq!(estimate.omitted_duration_components.len(), 1);
        assert_eq!(estimate.omitted_duration_components[0].command, "M109");
    }
}
