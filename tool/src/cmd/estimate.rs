use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufReader;

use lib_klipper::gcode::GCodeReader;
use lib_klipper::glam::{DVec2, Vec4Swizzles};
use lib_klipper::planner::{Delay, Planner, PlannerDiagnostic, PlanningMove, PlanningOperation};

use clap::Parser;
use ordered_float::NotNan;
use serde::{ser::SerializeSeq, Serialize, Serializer};

use crate::calibration::{fetch_history_calibration, CalibrationReport};
use crate::config_snapshot::{ConfigSnapshot, ConfigSnapshotSummary, SnapshotAccuracy};
use crate::duration::DurationEstimate;
use crate::Opts;

fn format_time(mut seconds: f64) -> String {
    let mut parts = Vec::new();

    if seconds > 86400.0 {
        parts.push(format!("{}d", (seconds / 86400.0).floor()));
        seconds %= 86400.0;
    }
    if seconds > 3600.0 {
        parts.push(format!("{}h", (seconds / 3600.0).floor()));
        seconds %= 3600.0;
    }
    if seconds > 60.0 {
        parts.push(format!("{}m", (seconds / 60.0).floor()));
        seconds %= 60.0;
    }
    if seconds > 0.0 {
        parts.push(format!("{:.3}s", seconds));
    }

    if parts.is_empty() {
        return "0s".into();
    }

    parts.join("")
}

#[derive(clap::ArgEnum, Debug, Clone, Copy, Eq, PartialEq)]
pub enum OutputFormat {
    Human,
    Json,
}

#[derive(Parser, Debug)]
pub struct EstimateCmd {
    input: String,
    #[clap(arg_enum, long, short, default_value_t = OutputFormat::Human)]
    format: OutputFormat,
    #[clap(long)]
    omit_move_kinds: bool,
    #[clap(long)]
    omit_layer_times: bool,
    /// Calibrate expected time from verified, completed Moonraker history jobs
    #[clap(long = "history_calibration")]
    history_calibration: bool,
    /// Maximum number of recent Moonraker history records to inspect
    #[clap(long = "history_calibration_limit", default_value_t = 50)]
    history_calibration_limit: usize,
    /// Reject calibration observations older than this many days
    #[clap(long = "history_calibration_max_age_days", default_value_t = 90)]
    history_calibration_max_age_days: u64,
    /// Minimum verified observations required to apply calibration
    #[clap(long = "history_calibration_min_samples", default_value_t = 3)]
    history_calibration_min_samples: usize,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize)]
struct EstimationState {
    metadata: EstimationMetadata,
    configuration: Option<ConfigSnapshotSummary>,
    #[serde(skip_serializing_if = "Option::is_none")]
    calibration: Option<CalibrationReport>,
    #[serde(flatten)]
    duration: DurationEstimate,
    sequences: Vec<EstimationSequence>,
    diagnostics: Vec<PlannerDiagnostic>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize)]
#[serde(rename_all = "snake_case")]
enum EstimateAccuracyClass {
    #[default]
    Complete,
    Degraded,
    LowerBound,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize)]
struct EstimationMetadata {
    estimator_version: String,
    klipper_version: Option<String>,
    configuration_fingerprint: String,
    backend: String,
    accuracy_class: EstimateAccuracyClass,
}

impl EstimationMetadata {
    fn from_snapshot(snapshot: &ConfigSnapshot) -> Self {
        Self {
            estimator_version: env!("TOOL_VERSION").into(),
            klipper_version: snapshot.klipper_version.clone(),
            configuration_fingerprint: snapshot.fingerprint.clone(),
            backend: snapshot.limits.kinematics.backend_name().into(),
            accuracy_class: if snapshot.accuracy == SnapshotAccuracy::Complete {
                EstimateAccuracyClass::Complete
            } else {
                EstimateAccuracyClass::Degraded
            },
        }
    }
}

#[derive(Debug, Clone, PartialEq, Default, Serialize)]
struct EstimationSequence {
    #[serde(flatten)]
    duration: DurationEstimate,
    total_distance: f64,
    total_extrude_distance: f64,
    max_flow: Option<f64>,
    max_speed: Option<f64>,
    num_moves: usize,
    total_z_time: f64,
    total_output_time: f64,
    total_travel_time: f64,
    total_extrude_only_time: f64,
    extruders: BTreeMap<String, ExtruderEstimation>,
    phase_times: EstimationPhaseTimes,
    kind_times: BTreeMap<String, f64>,
    #[serde(serialize_with = "serialize_layer_times")]
    layer_times: BTreeMap<NotNan<f64>, f64>,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize)]
struct ExtruderEstimation {
    /// Signed filament movement; retractions remain negative.
    net_distance: f64,
    extruded_distance: f64,
    retracted_distance: f64,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize)]
struct EstimationPhaseTimes {
    acceleration: f64,
    cruise: f64,
    deceleration: f64,
}

fn serialize_layer_times<S: Serializer>(
    lts: &BTreeMap<NotNan<f64>, f64>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    let mut seq = serializer.serialize_seq(Some(lts.len()))?;

    for (z, t) in lts {
        seq.serialize_element(&[z, t])?;
    }

    seq.end()
}

impl EstimationState {
    fn finalize_accuracy_class(&mut self) {
        if !self.diagnostics.is_empty() || !self.duration.omitted_duration_components.is_empty() {
            self.metadata.accuracy_class = EstimateAccuracyClass::LowerBound;
        }
    }

    fn add(&mut self, planner: &Planner, op: &PlanningOperation) {
        self.duration.add_operation(planner, op);
        match op {
            PlanningOperation::Move(m) => self.add_move(planner, m),
            PlanningOperation::Delay(Delay::Dwell(t)) => {
                let t = t.as_secs_f64();
                let seq = self.get_cur_seq();
                seq.duration.add_deterministic("dwell", t);
                let kind = "Dwell";
                if let Some(kt) = seq.kind_times.get_mut(kind) {
                    *kt += t;
                } else {
                    seq.kind_times.insert(kind.to_string(), t);
                }
            }
            PlanningOperation::Delay(Delay::EstimatorAddition { duration, kind }) => {
                // If current sequence has moves or there is no sequence, make a new one
                if self
                    .sequences
                    .last()
                    .map(|s| s.num_moves != 0)
                    .unwrap_or(true)
                {
                    self.sequences.push(EstimationSequence::default());
                }
                let seq = self.sequences.last_mut().unwrap();
                let t = duration.as_secs_f64();
                seq.duration.add_operation(planner, op);
                let kind = planner.kind_str(kind).unwrap_or("Other");
                if let Some(kt) = seq.kind_times.get_mut(kind) {
                    *kt += t;
                } else {
                    seq.kind_times.insert(kind.to_string(), t);
                }
            }
            PlanningOperation::Delay(Delay::Contract { .. }) => {
                let seq = self.get_cur_seq();
                seq.duration.add_operation(planner, op);
            }
            PlanningOperation::Delay(Delay::Unknown { .. }) => {
                if self
                    .sequences
                    .last()
                    .map(|sequence| sequence.num_moves != 0)
                    .unwrap_or(true)
                {
                    self.sequences.push(EstimationSequence::default());
                }
                self.sequences
                    .last_mut()
                    .expect("sequence was just initialized")
                    .duration
                    .add_operation(planner, op);
            }
            _ => {}
        }
    }

    fn get_cur_seq(&mut self) -> &mut EstimationSequence {
        if self.sequences.is_empty() {
            self.sequences.push(EstimationSequence::default());
        }
        self.sequences.last_mut().unwrap()
    }

    fn add_move(&mut self, planner: &Planner, m: &PlanningMove) {
        let seq = self.get_cur_seq();
        seq.duration
            .add_operation(planner, &PlanningOperation::Move(*m));
        seq.total_distance += m.distance;
        let extrusion_delta = m.end.w - m.start.w;
        seq.total_extrude_distance += extrusion_delta;
        if m.is_extrude_move() {
            let extruder = planner.move_extruder_name(m).unwrap_or("unconfigured");
            let stats = seq.extruders.entry(extruder.into()).or_default();
            stats.net_distance += extrusion_delta;
            if extrusion_delta > 0.0 {
                stats.extruded_distance += extrusion_delta;
            } else {
                stats.retracted_distance += extrusion_delta;
            }
        }
        seq.num_moves += 1;
        seq.max_speed = Some(seq.max_speed.unwrap_or(0.0).max(m.cruise_v));

        match (m.is_extrude_move(), m.is_kinematic_move()) {
            (true, true) => {
                seq.total_output_time += m.total_time();
                if let Some(flow_rate) =
                    m.flow_rate(planner.move_filament_radius(m).unwrap_or(1.75 / 2.0))
                {
                    seq.max_flow = Some(seq.max_flow.unwrap_or(0.0).max(flow_rate));
                }
            }
            (true, false) => seq.total_extrude_only_time += m.total_time(),
            (false, true) => seq.total_travel_time += m.total_time(),
            _ => {}
        }

        {
            let pt = &mut seq.phase_times;
            pt.acceleration += m.accel_time();
            pt.cruise += m.cruise_time();
            pt.deceleration += m.decel_time();
        }

        let kind = planner.move_kind_str(m).unwrap_or("Other");
        if let Some(t) = seq.kind_times.get_mut(kind) {
            *t += m.total_time();
        } else {
            seq.kind_times.insert(kind.to_string(), m.total_time());
        }

        if (m.start.z - m.end.z).abs() < f64::EPSILON {
            *seq.layer_times
                .entry(NotNan::new((m.start.z * 1000.0).round() / 1000.0).unwrap())
                .or_insert(0.0) += m.total_time();
        } else {
            seq.total_z_time += m.total_time();
        }
    }
}

impl EstimateCmd {
    pub fn run(&self, opts: &Opts) {
        let src: Box<dyn std::io::Read> = match self.input.as_str() {
            "-" => Box::new(std::io::stdin()),
            filename => Box::new(File::open(filename).expect("opening gcode file failed")),
        };
        let rdr = GCodeReader::new(BufReader::new(src));

        let snapshot = opts.config_snapshot();
        let mut planner = opts.make_planner();
        let mut state = EstimationState {
            metadata: EstimationMetadata::from_snapshot(snapshot),
            configuration: Some(snapshot.summary()),
            ..EstimationState::default()
        };

        for (i, cmd) in rdr.enumerate() {
            let cmd = cmd.expect("gcode read");
            planner.process_cmd(&cmd);

            if i % 1000 == 0 {
                for o in planner.iter().collect::<Vec<_>>() {
                    state.add(&planner, &o);
                }
            }
        }

        planner.finalize();
        for o in planner.iter().collect::<Vec<_>>() {
            state.add(&planner, &o);
        }
        state.diagnostics = planner.diagnostics().to_vec();
        state.finalize_accuracy_class();

        if self.history_calibration {
            let fingerprint = state
                .configuration
                .as_ref()
                .expect("estimate configuration is always present")
                .fingerprint
                .clone();
            let mut calibration = match opts.moonraker_connection() {
                Some((url, api_key)) if self.history_calibration_limit > 0 => {
                    fetch_history_calibration(
                        url,
                        api_key,
                        &fingerprint,
                        self.history_calibration_limit,
                        self.history_calibration_max_age_days
                            .saturating_mul(24 * 60 * 60),
                        self.history_calibration_min_samples.max(1),
                    )
                }
                Some(_) => CalibrationReport::unavailable(
                    "history calibration limit must be greater than zero",
                ),
                None => CalibrationReport::unavailable(
                    "history calibration requires --config_moonraker_url",
                ),
            };
            calibration.apply(&mut state.duration);
            state.calibration = Some(calibration);
        }

        match self.format {
            OutputFormat::Human => {
                println!(
                    "Estimator: {} | Klipper: {} | Backend: {} | Accuracy: {:?}",
                    state.metadata.estimator_version,
                    state
                        .metadata
                        .klipper_version
                        .as_deref()
                        .unwrap_or("unknown"),
                    state.metadata.backend,
                    state.metadata.accuracy_class
                );
                println!();
                if let Some(configuration) = &state.configuration {
                    println!("Configuration: {}", configuration.fingerprint);
                    for warning in &configuration.warnings {
                        println!("  Warning: {warning}");
                    }
                    println!();
                }
                if !state.diagnostics.is_empty() {
                    println!("Diagnostics:");
                    for diagnostic in &state.diagnostics {
                        println!("  {}: {}", diagnostic.command, diagnostic.message);
                    }
                    println!();
                }
                if !state.duration.omitted_duration_components.is_empty() {
                    println!("Omitted duration components:");
                    for omitted in &state.duration.omitted_duration_components {
                        println!(
                            "  {} ({:?}): {}",
                            omitted.command, omitted.category, omitted.reason
                        );
                    }
                    println!();
                }
                if let Some(calibration) = &state.calibration {
                    println!("History calibration: {:?}", calibration.status);
                    if let Some(reason) = &calibration.reason {
                        println!("  Reason: {reason}");
                    }
                    if let Some(model) = &calibration.model {
                        println!("  Samples: {}", model.sample_count);
                        println!("  Median residual: {:.3}s", model.median_residual);
                    }
                    if let Some(prediction) = &calibration.prediction {
                        println!(
                            "  Expected total: {} ({:.3}s)",
                            format_time(prediction.expected_total_time),
                            prediction.expected_total_time
                        );
                        println!(
                            "  {:.0}% interval: {:.3}s .. {:.3}s",
                            prediction.confidence * 100.0,
                            prediction.lower,
                            prediction.upper
                        );
                    }
                    println!();
                }
                println!("Sequences:");

                let cross_section = std::f64::consts::PI * (1.75f64 / 2.0).powf(2.0);
                for (i, seq) in state.sequences.iter().enumerate() {
                    if i > 0 {
                        println!();
                    }
                    println!(" Run {}:", i);
                    println!("  Total moves:                 {}", seq.num_moves);
                    println!("  Total distance:              {:.3}mm", seq.total_distance);
                    println!(
                        "  Total extrude distance:      {:.3}mm",
                        seq.total_extrude_distance
                    );
                    for (name, extruder) in &seq.extruders {
                        println!(
                            "   {name}: net {:.3}mm, extruded {:.3}mm, retracted {:.3}mm",
                            extruder.net_distance,
                            extruder.extruded_distance,
                            extruder.retracted_distance
                        );
                    }
                    println!(
                        "  Motion time:                 {} ({:.3}s)",
                        format_time(seq.duration.motion_time),
                        seq.duration.motion_time
                    );
                    println!(
                        "  Deterministic time:          {} ({:.3}s)",
                        format_time(seq.duration.deterministic_time),
                        seq.duration.deterministic_time
                    );
                    println!(
                        "  Expected total time:         {} ({:.3}s)",
                        format_time(seq.duration.expected_total_time),
                        seq.duration.expected_total_time
                    );
                    println!(
                        "  Total print move time:       {} ({:.3}s)",
                        format_time(seq.total_output_time),
                        seq.total_output_time
                    );
                    println!(
                        "  Total extrude-only time:     {} ({:.3}s)",
                        format_time(seq.total_extrude_only_time),
                        seq.total_extrude_only_time
                    );
                    println!(
                        "  Total travel time:           {} ({:.3}s)",
                        format_time(seq.total_travel_time),
                        seq.total_travel_time
                    );
                    println!(
                        "  Average speed:               {:.3} mm/s",
                        seq.total_distance / seq.duration.total_time
                    );
                    println!(
                        "  Top speed:                   {}",
                        if let Some(max_speed) = seq.max_speed {
                            format!("{:.3} mm/s", max_speed)
                        } else {
                            "-".to_string()
                        }
                    );
                    println!(
                        "  Average flow:                {:.3} mm³/s",
                        seq.total_extrude_distance * cross_section / seq.duration.total_time
                    );
                    println!(
                        "  Maximum flow:                {}",
                        if let Some(max_flow) = seq.max_flow {
                            format!("{:.3} mm³/s", max_flow)
                        } else {
                            "-".to_string()
                        }
                    );
                    println!(
                        "  Average flow (output only):  {:.3} mm³/s",
                        seq.total_extrude_distance * cross_section / seq.total_output_time
                    );
                    println!("  Phases:");
                    println!(
                        "   Acceleration:               {}",
                        format_time(seq.phase_times.acceleration)
                    );
                    println!(
                        "   Cruise:                     {}",
                        format_time(seq.phase_times.cruise)
                    );
                    println!(
                        "   Deceleration:               {}",
                        format_time(seq.phase_times.deceleration)
                    );

                    let mut kind_times = seq.kind_times.iter().collect::<Vec<_>>();
                    if !self.omit_move_kinds && !kind_times.is_empty() {
                        println!("  Move kind distribution:");
                        kind_times.sort_by_key(|(_, t)| {
                            NotNan::new(**t).unwrap_or_else(|_| NotNan::new(0.0).unwrap())
                        });
                        let kind_length = kind_times
                            .iter()
                            .map(|(_, t)| format_time(**t).len())
                            .max()
                            .unwrap_or(0);
                        for (k, t) in kind_times.iter().rev() {
                            println!("   {:kind_length$}     {}", format_time(**t), k);
                        }
                    }

                    let layer_times = seq
                        .layer_times
                        .iter()
                        .map(|(l, t)| (format!("{l:.3}"), format_time(*t)))
                        .collect::<Vec<_>>();
                    if !self.omit_layer_times && !layer_times.is_empty() {
                        println!("  Layer time distribution:");
                        let longest_z = layer_times.iter().map(|(z, _)| z.len()).max().unwrap_or(0);
                        let longest_t = layer_times.iter().map(|(_, t)| t.len()).max().unwrap_or(0);
                        let colon = ": ";
                        let column = longest_z + longest_t + colon.len();
                        let offset = " ".repeat(3);
                        let spacing = " ".repeat(4);

                        let term_width = term_size::dimensions().map(|(w, _)| w).unwrap_or(0);
                        let available_width = term_width.saturating_sub(offset.len());

                        let num_columns =
                            (available_width.saturating_sub(column) / (column + spacing.len()) + 1)
                                .max(1);
                        let chunk_size = layer_times.len() / num_columns
                            + usize::from(layer_times.len() % num_columns != 0);
                        let columnized = layer_times.chunks(chunk_size).collect::<Vec<_>>();
                        for line in 0.. {
                            if columnized
                                .iter()
                                .map(|c| c.len().saturating_sub(line))
                                .max()
                                .unwrap_or(0)
                                == 0
                            {
                                break;
                            }

                            print!("{offset}");
                            for i in 0..num_columns {
                                if let Some((t, l)) =
                                    columnized.get(i).and_then(|col| col.get(line))
                                {
                                    if i > 0 {
                                        print!("{spacing}");
                                    }
                                    print!("{t:>longest_z$}{colon}{l:>longest_t$}");
                                }
                            }
                            println!();
                        }
                    }
                }
            }
            OutputFormat::Json => {
                serde_json::to_writer_pretty(std::io::stdout(), &state)
                    .expect("Serialization error");
            }
        }
    }
}

#[derive(Parser, Debug)]
pub struct DumpMovesCmd {
    input: String,
}

#[derive(Debug)]
struct DumpMovesState {
    move_idx: usize,
    ctime: f64,
    ztime: f64,
}

impl DumpMovesState {
    fn flush(&mut self, planner: &mut Planner) {
        for o in planner.iter().collect::<Vec<_>>() {
            let m = match o.get_move() {
                Some(m) => m,
                None => continue,
            };
            self.move_idx += 1;

            let mut kind = String::new();
            if m.is_extrude_move() {
                kind.push('E');
            }
            if m.is_kinematic_move() {
                kind.push('K');
            }
            println!(
                "N{}[{}] @ {:.8} => {:.8} / z{:.8}:",
                self.move_idx,
                kind,
                self.ctime,
                self.ctime + m.total_time(),
                self.ztime,
            );
            println!(
                "    Path:       {} => {} [{:.3}∠{:.2}]",
                (m.start * 1000.0).round() / 1000.0,
                (m.end * 1000.0).round() / 1000.0,
                m.distance,
                m.rate.xy().angle_between(DVec2::new(1.0, 0.0)) * 180.0 / std::f64::consts::PI,
            );
            println!("    Axes {}", (m.rate * 1000.0).round() / 1000.0);
            println!("    Line width: {:?}", m.line_width(1.75 / 2.0, 0.25),);
            println!("    Flow rate: {:?}", m.flow_rate(1.75 / 2.0));
            println!("    Kind: {}", planner.move_kind_str(&m).unwrap_or("Other"));
            println!("    Acceleration {:.4}", m.acceleration);
            println!("    Delta v2: {:.4}", m.delta_v2);
            println!("    Max start_v2: {:.4}", m.max_start_v2);
            println!("    Max cruise_v2: {:.4}", m.max_cruise_v2);
            println!("    Next junction_v2: {:.4}", m.next_junction_v2);
            println!("    Max MCR start_v2: {:.4}", m.max_mcr_start_v2);
            println!(
                "    Velocity:   {:.3} => {:.3} => {:.3}",
                m.start_v, m.cruise_v, m.end_v
            );
            println!(
                "    Time:       {:.4}+{:.4}+{:.4} = {:.4}",
                m.accel_time(),
                m.cruise_time(),
                m.decel_time(),
                m.total_time(),
            );
            self.ctime += m.total_time();

            println!(
                "    Distances:  {:.3}+{:.3}+{:.3} = {:.3}",
                m.accel_distance(),
                m.cruise_distance(),
                m.decel_distance(),
                m.distance
            );

            println!();

            self.ztime += m.total_time();
        }
    }
}

impl DumpMovesCmd {
    pub fn run(&self, opts: &Opts) {
        let src: Box<dyn std::io::Read> = match self.input.as_str() {
            "-" => Box::new(std::io::stdin()),
            filename => Box::new(File::open(filename).expect("opening gcode file failed")),
        };
        let rdr = GCodeReader::new(BufReader::new(src));

        let mut planner = opts.make_planner();
        let mut state = DumpMovesState {
            move_idx: 0,
            ctime: 0.25,
            ztime: 0.0,
        };

        for (i, cmd) in rdr.enumerate() {
            let cmd = cmd.expect("gcode read");
            planner.process_cmd(&cmd);

            if i % 1000 == 0 {
                state.flush(&mut planner);
            }
        }
        planner.finalize();
        state.flush(&mut planner);
        for diagnostic in planner.diagnostics() {
            eprintln!("{}: {}", diagnostic.command, diagnostic.message);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config_snapshot::ConfigSnapshot;
    use crate::duration::OmittedDurationComponent;
    use lib_klipper::gcode::parse_gcode;
    use lib_klipper::planner::{ExtruderLimits, PrinterLimits, UnknownDurationCategory};

    #[test]
    fn filament_statistics_keep_sign_and_tool_identity() {
        let extruder = ExtruderLimits {
            max_extrude_only_velocity: 100.0,
            max_extrude_only_accel: 1000.0,
            ..ExtruderLimits::default()
        };
        let limits = PrinterLimits {
            extruders: BTreeMap::from([
                ("extruder".into(), extruder.clone()),
                ("extruder1".into(), extruder),
            ]),
            ..PrinterLimits::default()
        };
        let mut planner = Planner::from_limits(limits);
        for line in [
            "M83",
            "G1 E5 F600",
            "G1 E-2 F600",
            "ACTIVATE_EXTRUDER EXTRUDER=extruder1",
            "G1 E3 F600",
        ] {
            planner.process_cmd(&parse_gcode(line).unwrap());
        }
        planner.finalize();
        let mut state = EstimationState::default();
        for operation in planner.iter().collect::<Vec<_>>() {
            state.add(&planner, &operation);
        }

        let stats = &state.sequences[0].extruders;
        assert_eq!(stats["extruder"].net_distance, 3.0);
        assert_eq!(stats["extruder"].extruded_distance, 5.0);
        assert_eq!(stats["extruder"].retracted_distance, -2.0);
        assert_eq!(stats["extruder1"].net_distance, 3.0);
        assert_eq!(stats["extruder1"].extruded_distance, 3.0);
        assert_eq!(stats["extruder1"].retracted_distance, 0.0);
    }

    #[test]
    fn estimate_metadata_exposes_reproducibility_context() {
        let mut snapshot = ConfigSnapshot::built_in_defaults();
        snapshot.klipper_version = Some("v0.13.0-745-gf0892d82b".into());
        snapshot.refresh_fingerprint();

        let metadata = EstimationMetadata::from_snapshot(&snapshot);
        let json = serde_json::to_value(metadata).unwrap();
        assert_eq!(json["estimator_version"], env!("TOOL_VERSION"));
        assert_eq!(json["klipper_version"], "v0.13.0-745-gf0892d82b");
        assert_eq!(json["configuration_fingerprint"], snapshot.fingerprint);
        assert_eq!(json["backend"], "unconfigured");
        assert_eq!(json["accuracy_class"], "degraded");
    }

    #[test]
    fn omitted_duration_makes_the_accuracy_class_a_lower_bound() {
        let mut state = EstimationState::default();
        state
            .duration
            .omitted_duration_components
            .push(OmittedDurationComponent {
                command: "PRINT_START".into(),
                category: UnknownDurationCategory::CommandOrMacro,
                reason: "duration is unknown and was not included".into(),
            });
        state.finalize_accuracy_class();

        assert_eq!(
            state.metadata.accuracy_class,
            EstimateAccuracyClass::LowerBound
        );
    }
}
