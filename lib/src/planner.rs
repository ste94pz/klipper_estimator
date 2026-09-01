use std::collections::{HashMap, VecDeque};
use std::time::Duration;

use crate::arcs::ArcState;
pub use crate::firmware_retraction::FirmwareRetractionOptions;
use crate::firmware_retraction::FirmwareRetractionState;
use crate::gcode::{GCodeCommand, GCodeOperation};

use crate::kind_tracker::{Kind, KindTracker};
use glam::Vec4Swizzles;
use glam::{DVec3 as Vec3, DVec4 as Vec4};
use serde::{Deserialize, Serialize};

#[derive(Debug)]
pub struct Planner {
    operations: OperationSequence,
    pub toolhead_state: ToolheadState,
    pub kind_tracker: KindTracker,
    pub firmware_retraction: Option<FirmwareRetractionState>,
    pub arc_state: ArcState,
    diagnostics: Vec<PlannerDiagnostic>,
}

impl Planner {
    pub fn from_limits(limits: PrinterLimits) -> Planner {
        let firmware_retraction = limits
            .firmware_retraction
            .as_ref()
            .map(|_| FirmwareRetractionState::default());
        Planner {
            operations: OperationSequence::default(),
            toolhead_state: ToolheadState::from_limits(limits),
            kind_tracker: KindTracker::new(),
            firmware_retraction,
            arc_state: ArcState::default(),
            diagnostics: Vec::new(),
        }
    }

    /// Processes a gcode command through the planning engine and appends it to the currently
    /// open move sequence.
    /// Returns the number of planning operations the command resulted in
    pub fn process_cmd(&mut self, cmd: &GCodeCommand) -> usize {
        if Self::is_unsupported_traditional_state_command(cmd) {
            self.diagnostics.push(PlannerDiagnostic {
                code: PlannerDiagnosticCode::UnsupportedStateCommand,
                command: cmd
                    .op
                    .to_string()
                    .split_whitespace()
                    .next()
                    .unwrap_or_default()
                    .into(),
                message: "parsed state-changing command is not modeled; estimate is a lower bound"
                    .into(),
            });
        }
        if let Some(m) = Self::is_dwell(cmd, &mut self.kind_tracker) {
            self.operations.add_delay(m);
        } else if let GCodeOperation::Move { x, y, z, e, f } = &cmd.op {
            if let Some(v) = f {
                self.toolhead_state.set_gcode_speed(*v);
            }

            let move_kind = self.kind_tracker.kind_from_comment(&cmd.comment);

            if x.is_some() || y.is_some() || z.is_some() || e.is_some() {
                let mut m = self.toolhead_state.perform_move([*x, *y, *z, *e]);
                m.kind = move_kind;
                self.operations.add_move(m, &self.toolhead_state);
            } else {
                self.operations.add_fill();
            }
        } else if let GCodeOperation::Traditional {
            letter,
            code,
            params,
        } = &cmd.op
        {
            match (letter, code) {
                ('G', 10) => {
                    let kt = &mut self.kind_tracker;
                    let m = &mut self.toolhead_state;
                    let seq = &mut self.operations;
                    if let Some(fr) = self.firmware_retraction.as_mut() {
                        return fr.retract(kt, m, seq);
                    }
                    self.diagnostics
                        .push(PlannerDiagnostic::unsupported_state_command("g10"));
                }
                ('G', 11) => {
                    let kt = &mut self.kind_tracker;
                    let m = &mut self.toolhead_state;
                    let seq = &mut self.operations;
                    if let Some(fr) = self.firmware_retraction.as_mut() {
                        return fr.unretract(kt, m, seq);
                    }
                    self.diagnostics
                        .push(PlannerDiagnostic::unsupported_state_command("g11"));
                }
                ('G', v @ 2 | v @ 3) => {
                    let move_kind = self.kind_tracker.kind_from_comment(&cmd.comment);
                    let m = &mut self.toolhead_state;
                    let seq = &mut self.operations;
                    return self.arc_state.generate_arc(
                        m,
                        seq,
                        move_kind,
                        params,
                        match v {
                            2 => crate::arcs::ArcDirection::Clockwise,
                            3 => crate::arcs::ArcDirection::CounterClockwise,
                            _ => unreachable!("v can only be 2 or 3"),
                        },
                    );
                }
                ('G', 17) => {
                    self.arc_state.set_plane(crate::arcs::Plane::XY);
                }
                ('G', 18) => {
                    self.arc_state.set_plane(crate::arcs::Plane::XZ);
                }
                ('G', 19) => {
                    self.arc_state.set_plane(crate::arcs::Plane::YZ);
                }
                ('G', 92) => {
                    self.toolhead_state.set_gcode_position([
                        params.get_number::<f64>('X'),
                        params.get_number::<f64>('Y'),
                        params.get_number::<f64>('Z'),
                        params.get_number::<f64>('E'),
                    ]);
                }
                ('G', 90) => self.toolhead_state.position_modes[..3].fill(PositionMode::Absolute),
                ('G', 91) => self.toolhead_state.position_modes[..3].fill(PositionMode::Relative),
                ('M', 82) => self.toolhead_state.position_modes[3] = PositionMode::Absolute,
                ('M', 83) => self.toolhead_state.position_modes[3] = PositionMode::Relative,
                ('M', 220) => {
                    self.toolhead_state
                        .set_speed_factor(params.get_number::<f64>('S').unwrap_or(100.0));
                }
                ('M', 221) => {
                    self.toolhead_state
                        .set_extrude_factor(params.get_number::<f64>('S').unwrap_or(100.0));
                }
                ('M', 204) => {
                    let s = params.get_number::<f64>('S');
                    let p = params.get_number::<f64>('P');
                    let t = params.get_number::<f64>('T');
                    match (s, p, t) {
                        (Some(s), _, _) => self.toolhead_state.limits.set_max_acceleration(s),
                        (_, Some(p), Some(t)) => {
                            self.toolhead_state.limits.set_max_acceleration(p.min(t))
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
            self.operations.add_fill();
        } else if let GCodeOperation::Extended { command, params } = &cmd.op {
            match command.as_str() {
                "set_velocity_limit" => {
                    if let Some(v) = params.get_number::<f64>("velocity") {
                        self.toolhead_state.limits.set_max_velocity(v);
                    }
                    if let Some(v) = params.get_number::<f64>("accel") {
                        self.toolhead_state.limits.set_max_acceleration(v);
                    }
                    if let Some(v) = params.get_number::<f64>("accel_to_decel") {
                        self.toolhead_state.limits.set_max_accel_to_decel(v);
                    }
                    if let Some(v) = params.get_number::<f64>("square_corner_velocity") {
                        self.toolhead_state.limits.set_square_corner_velocity(v);
                    }
                }
                "set_retraction" => {
                    let m = &mut self.toolhead_state;
                    if let Some(fr) = self.firmware_retraction.as_ref() {
                        fr.set_options(m, params);
                    }
                }
                "set_gcode_offset" => {
                    let mut offset = [None; 4];
                    let mut adjustment = [None; 4];
                    for (axis, name) in ["x", "y", "z", "e"].iter().enumerate() {
                        offset[axis] = params.get_number::<f64>(name);
                        adjustment[axis] = params.get_number::<f64>(&format!("{name}_adjust"));
                    }
                    let move_requested = params.get_number::<i64>("move").unwrap_or(0) != 0;
                    let move_speed = params.get_number::<f64>("move_speed");
                    if let Some(m) = self.toolhead_state.set_gcode_offset(
                        offset,
                        adjustment,
                        move_requested,
                        move_speed,
                    ) {
                        self.operations.add_move(m, &self.toolhead_state);
                        return 1;
                    }
                }
                "save_gcode_state" => {
                    let name = params.get_string("name").unwrap_or("default");
                    self.toolhead_state.save_gcode_state(name);
                }
                "restore_gcode_state" => {
                    let name = params.get_string("name").unwrap_or("default");
                    let move_requested = params.get_number::<i64>("move").unwrap_or(0) != 0;
                    let move_speed = params.get_number::<f64>("move_speed");
                    match self
                        .toolhead_state
                        .restore_gcode_state(name, move_requested, move_speed)
                    {
                        Ok(Some(m)) => {
                            self.operations.add_move(m, &self.toolhead_state);
                            return 1;
                        }
                        Ok(None) => {}
                        Err(()) => self
                            .diagnostics
                            .push(PlannerDiagnostic::unknown_saved_state(name)),
                    }
                }
                _ => {}
            }
            if Self::is_unsupported_state_command(command) {
                self.diagnostics
                    .push(PlannerDiagnostic::unsupported_state_command(command));
            }
            self.operations.add_fill();
        } else if let (true, Some(comment)) = (cmd.op.is_nop(), cmd.comment.as_ref()) {
            if let Some(comment) = comment.strip_prefix("TYPE:") {
                // IdeaMaker only gives us `TYPE:`s
                let kind = self.kind_tracker.get_kind(comment);
                self.kind_tracker.set_current(Some(kind));
                self.operations.add_fill();
            } else if let Some(cmd) = comment.trim_start().strip_prefix("ESTIMATOR_ADD_TIME ") {
                if let Some((duration, kind)) = Self::parse_buffer_cmd(&mut self.kind_tracker, cmd)
                {
                    self.operations.add_delay(Delay::Indeterminate(
                        Duration::from_secs_f64(duration),
                        kind,
                    ));
                } else {
                    self.operations.add_fill();
                }
            } else {
                self.operations.add_fill();
            }
        } else {
            self.operations.add_fill();
        }
        1 // Most commands result in a single planning op
    }

    /// Performs final processing on the final sequence, if one is active.
    pub fn finalize(&mut self) {
        self.operations.flush();
    }

    pub fn diagnostics(&self) -> &[PlannerDiagnostic] {
        &self.diagnostics
    }

    fn is_unsupported_state_command(command: &str) -> bool {
        matches!(
            command,
            "activate_extruder"
                | "bed_mesh_clear"
                | "bed_mesh_profile"
                | "restore_dual_carriage_state"
                | "save_dual_carriage_state"
                | "set_dual_carriage"
                | "set_kinematic_position"
                | "set_skew"
                | "skew_profile"
        )
    }

    fn is_unsupported_traditional_state_command(cmd: &GCodeCommand) -> bool {
        matches!(
            cmd.op,
            GCodeOperation::Traditional {
                letter: 'G',
                code: 20 | 28,
                ..
            } | GCodeOperation::Traditional {
                letter: 'M',
                code: 600,
                ..
            }
        )
    }

    fn is_dwell(cmd: &GCodeCommand, kind_tracker: &mut KindTracker) -> Option<Delay> {
        let indef = Duration::from_secs_f64(0.1);
        match &cmd.op {
            GCodeOperation::Traditional {
                letter: 'G',
                code: 4,
                params,
            } => Some(Delay::Pause(Duration::from_secs_f64(
                params.get_number('P').map_or(0.25, |v: f64| v / 1000.0),
            ))),
            GCodeOperation::Traditional {
                letter: 'G',
                code: 28,
                ..
            } => Some(Delay::Indeterminate(
                indef,
                Some(kind_tracker.get_kind("Indeterminate time")),
            )),
            GCodeOperation::Traditional {
                letter: 'M',
                code: 109 | 190,
                ..
            } => Some(Delay::Indeterminate(
                indef,
                Some(kind_tracker.get_kind("Indeterminate time")),
            )),
            GCodeOperation::Extended { command: cmd, .. } if cmd == "temperature_wait" => Some(
                Delay::Indeterminate(indef, Some(kind_tracker.get_kind("Indeterminate time"))),
            ),
            GCodeOperation::Traditional {
                letter: 'M',
                code: 600,
                ..
            } => Some(Delay::Indeterminate(
                indef,
                Some(kind_tracker.get_kind("Indeterminate time")),
            )),
            _ => None,
        }
    }

    fn parse_buffer_cmd(kind_tracker: &mut KindTracker, cmd: &str) -> Option<(f64, Option<Kind>)> {
        let (a, b) = cmd
            .split_once(' ')
            .map_or((cmd, None), |(l, r)| (l, Some(r)));
        let duration = a.parse().ok()?;
        let kind = b.map(|s| kind_tracker.get_kind(s));
        Some((duration, kind))
    }

    pub fn next_operation(&mut self) -> Option<PlanningOperation> {
        self.operations.next_operation()
    }

    pub fn iter(&mut self) -> PlanningOperationIter<'_> {
        PlanningOperationIter { planner: self }
    }

    pub fn move_kind_str<'a>(&'a self, m: &PlanningMove) -> Option<&'a str> {
        m.kind.map(|k| self.kind_tracker.resolve_kind(k))
    }

    pub fn kind_str<'a>(&'a self, kind: &Option<Kind>) -> Option<&'a str> {
        kind.map(|k| self.kind_tracker.resolve_kind(k))
    }
}

#[derive(Debug)]
pub enum Delay {
    Indeterminate(Duration, Option<Kind>),
    Pause(Duration),
}

impl Delay {
    pub fn duration(&self) -> Duration {
        match self {
            Delay::Indeterminate(d, _) => *d,
            Delay::Pause(d) => *d,
        }
    }
}

#[derive(Debug)]
pub enum PlanningOperation {
    Delay(Delay),
    Move(PlanningMove),
    Fill,
}

#[derive(Debug, Clone, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PlannerDiagnosticCode {
    UnknownSavedGcodeState,
    UnsupportedStateCommand,
}

#[derive(Debug, Clone, Eq, PartialEq, Serialize)]
pub struct PlannerDiagnostic {
    pub code: PlannerDiagnosticCode,
    pub command: String,
    pub message: String,
}

impl PlannerDiagnostic {
    fn unknown_saved_state(name: &str) -> Self {
        Self {
            code: PlannerDiagnosticCode::UnknownSavedGcodeState,
            command: "RESTORE_GCODE_STATE".into(),
            message: format!("unknown saved G-code state: {name}"),
        }
    }

    fn unsupported_state_command(command: &str) -> Self {
        Self {
            code: PlannerDiagnosticCode::UnsupportedStateCommand,
            command: command.to_ascii_uppercase(),
            message: "parsed state-changing command is not modeled; estimate is a lower bound"
                .into(),
        }
    }
}

impl PlanningOperation {
    pub fn is_fill(&self) -> bool {
        matches!(self, Self::Fill)
    }

    pub fn is_move(&self) -> bool {
        matches!(self, Self::Move(_))
    }

    pub fn get_move(&self) -> Option<PlanningMove> {
        match self {
            Self::Move(m) => Some(*m),
            _ => None,
        }
    }
}

#[derive(Debug)]
pub struct PlanningOperationIter<'a> {
    planner: &'a mut Planner,
}

impl<'a> Iterator for PlanningOperationIter<'a> {
    type Item = PlanningOperation;

    fn next(&mut self) -> Option<Self::Item> {
        self.planner.next_operation()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PlanningMove {
    pub start: Vec4,
    pub end: Vec4,
    pub distance: f64,
    pub rate: Vec4,
    pub requested_velocity: f64,
    pub acceleration: f64,
    pub junction_deviation: f64,
    pub max_start_v2: f64,
    pub max_cruise_v2: f64,
    pub max_dv2: f64,
    pub max_smoothed_v2: f64,
    pub smoothed_dv2: f64,

    pub kind: Option<Kind>,

    pub start_v: f64,
    pub cruise_v: f64,
    pub end_v: f64,
}

impl PlanningMove {
    /// Create a new `PlanningMove` that travels between the two points `start`
    /// and `end`.
    pub(crate) fn new(start: Vec4, end: Vec4, toolhead_state: &ToolheadState) -> PlanningMove {
        if start.xyz() == end.xyz() {
            Self::new_extrude_move(start, end, toolhead_state)
        } else {
            Self::new_kinematic_move(start, end, toolhead_state)
        }
    }

    fn new_extrude_move(start: Vec4, end: Vec4, toolhead_state: &ToolheadState) -> PlanningMove {
        let dirs = Vec4::new(0.0, 0.0, 0.0, end.w - start.w);
        let move_d = dirs.w.abs();
        let inv_move_d = if move_d > 0.0 { 1.0 / move_d } else { 0.0 };
        PlanningMove {
            start,
            end,
            distance: (start.w - end.w).abs(),
            rate: dirs * inv_move_d,
            requested_velocity: toolhead_state.velocity,
            acceleration: f64::MAX,
            junction_deviation: toolhead_state.limits.junction_deviation,
            max_start_v2: 0.0,
            max_cruise_v2: toolhead_state.velocity * toolhead_state.velocity,
            max_dv2: f64::MAX,
            max_smoothed_v2: 0.0,
            smoothed_dv2: f64::MAX,
            kind: None,

            start_v: 0.0,
            cruise_v: 0.0,
            end_v: 0.0,
        }
    }

    fn new_kinematic_move(start: Vec4, end: Vec4, toolhead_state: &ToolheadState) -> PlanningMove {
        let distance = start.xyz().distance(end.xyz()); // Can't be zero
        let velocity = toolhead_state
            .velocity
            .min(toolhead_state.limits.max_velocity);

        PlanningMove {
            start,
            end,
            distance,
            rate: (end - start) / distance,
            requested_velocity: velocity,
            acceleration: toolhead_state.limits.max_acceleration,
            junction_deviation: toolhead_state.limits.junction_deviation,
            max_start_v2: 0.0,
            max_cruise_v2: velocity * velocity,
            max_dv2: 2.0 * distance * toolhead_state.limits.max_acceleration,
            max_smoothed_v2: 0.0,
            smoothed_dv2: 2.0 * distance * toolhead_state.limits.accel_to_decel,
            kind: None,

            start_v: 0.0,
            cruise_v: 0.0,
            end_v: 0.0,
        }
    }

    fn apply_junction(&mut self, previous_move: &PlanningMove, toolhead_state: &ToolheadState) {
        if !self.is_kinematic_move() || !previous_move.is_kinematic_move() {
            return;
        }

        let mut junction_cos_theta = -self.rate.xyz().dot(previous_move.rate.xyz());
        if junction_cos_theta > 0.999999 {
            // Move was not at an angle, skip all this
            return;
        }
        junction_cos_theta = junction_cos_theta.max(-0.999999);
        let sin_theta_d2 = (0.5 * (1.0 - junction_cos_theta)).sqrt();
        let r = sin_theta_d2 / (1.0 - sin_theta_d2);
        let tan_theta_d2 = sin_theta_d2 / (0.5 * (1.0 + junction_cos_theta)).sqrt();
        let move_centripetal_v2 = 0.5 * self.distance * tan_theta_d2 * self.acceleration;
        let prev_move_centripetal_v2 =
            0.5 * previous_move.distance * tan_theta_d2 * previous_move.acceleration;

        let extruder_v2 = toolhead_state.extruder_junction_speed_v2(self, previous_move);

        self.max_start_v2 = extruder_v2
            .min(r * self.junction_deviation * self.acceleration)
            .min(r * previous_move.junction_deviation * previous_move.acceleration)
            .min(move_centripetal_v2)
            .min(prev_move_centripetal_v2)
            .min(self.max_cruise_v2)
            .min(previous_move.max_cruise_v2)
            .min(previous_move.max_start_v2 + previous_move.max_dv2);
        self.max_smoothed_v2 = self
            .max_start_v2
            .min(previous_move.max_smoothed_v2 + previous_move.smoothed_dv2);
    }

    fn set_junction(&mut self, start_v2: f64, cruise_v2: f64, end_v2: f64) {
        self.start_v = start_v2.sqrt();
        self.cruise_v = cruise_v2.sqrt();
        self.end_v = end_v2.sqrt();
    }

    pub fn is_kinematic_move(&self) -> bool {
        self.start.xyz() != self.end.xyz()
    }

    pub fn is_extrude_move(&self) -> bool {
        (self.end.w - self.start.w).abs() >= f64::EPSILON
    }

    pub fn is_extrude_only_move(&self) -> bool {
        !self.is_kinematic_move() && self.is_extrude_move()
    }

    pub fn is_zero_distance(&self) -> bool {
        self.distance.abs() < f64::EPSILON
    }

    pub fn line_width(&self, filament_radius: f64, layer_height: f64) -> Option<f64> {
        // Only moves that are both extruding and moving have a line width
        if !self.is_kinematic_move() || !self.is_extrude_move() {
            return None;
        }
        Some(self.rate.w * filament_radius * filament_radius * std::f64::consts::PI / layer_height)
    }

    pub fn flow_rate(&self, filament_radius: f64) -> Option<f64> {
        if !self.is_extrude_move() {
            return None;
        }
        Some(
            self.delta().w * filament_radius * filament_radius * std::f64::consts::PI
                / self.total_time(),
        )
    }

    pub fn limit_speed(&mut self, velocity: f64, acceleration: f64) {
        let v2 = velocity * velocity;
        if v2 < self.max_cruise_v2 {
            self.max_cruise_v2 = v2;
        }
        self.acceleration = self.acceleration.min(acceleration);
        self.max_dv2 = 2.0 * self.distance * self.acceleration;
        self.smoothed_dv2 = self.smoothed_dv2.min(self.max_dv2);
    }

    pub fn delta(&self) -> Vec4 {
        self.end - self.start
    }

    pub fn accel_distance(&self) -> f64 {
        (self.cruise_v * self.cruise_v - self.start_v * self.start_v) * 0.5 / self.acceleration
    }

    pub fn accel_time(&self) -> f64 {
        self.accel_distance() / ((self.start_v + self.cruise_v) * 0.5)
    }

    pub fn cruise_distance(&self) -> f64 {
        (self.distance - self.accel_distance() - self.decel_distance()).max(0.0)
    }

    pub fn cruise_time(&self) -> f64 {
        self.cruise_distance() / self.cruise_v
    }

    pub fn decel_distance(&self) -> f64 {
        (self.cruise_v * self.cruise_v - self.end_v * self.end_v) * 0.5 / self.acceleration
    }

    pub fn decel_time(&self) -> f64 {
        self.decel_distance() / ((self.end_v + self.cruise_v) * 0.5)
    }

    pub fn total_time(&self) -> f64 {
        self.accel_time() + self.cruise_time() + self.decel_time()
    }
}

#[derive(Debug)]
enum OperationSequenceOperation {
    Delay(Delay),
    MoveSequence(MoveSequence),
    Fill,
}

impl From<OperationSequenceOperation> for PlanningOperation {
    fn from(oso: OperationSequenceOperation) -> Self {
        match oso {
            OperationSequenceOperation::Delay(d) => PlanningOperation::Delay(d),
            OperationSequenceOperation::Fill => PlanningOperation::Fill,
            OperationSequenceOperation::MoveSequence(_) => {
                panic!("Invalid conversion of move sequence to planning op")
            }
        }
    }
}

#[derive(Debug, Default)]
pub struct OperationSequence {
    ops: VecDeque<OperationSequenceOperation>,
}

impl OperationSequence {
    pub(crate) fn add_delay(&mut self, delay: Delay) {
        self.ops.push_back(OperationSequenceOperation::Delay(delay));
    }

    pub(crate) fn add_move(&mut self, move_cmd: PlanningMove, toolhead_state: &ToolheadState) {
        if let Some(OperationSequenceOperation::MoveSequence(ms)) = self.ops.back_mut() {
            ms.add_move(move_cmd, toolhead_state);
        } else {
            let mut ms = MoveSequence::default();
            ms.add_move(move_cmd, toolhead_state);
            self.ops
                .push_back(OperationSequenceOperation::MoveSequence(ms));
        }
    }

    pub(crate) fn add_fill(&mut self) {
        if let Some(OperationSequenceOperation::MoveSequence(ms)) = self.ops.back_mut() {
            ms.add_fill();
        } else {
            self.ops.push_back(OperationSequenceOperation::Fill);
        }
    }

    pub(crate) fn flush(&mut self) {
        for o in self.ops.iter_mut() {
            if let OperationSequenceOperation::MoveSequence(ms) = o {
                ms.flush();
            }
        }
    }

    fn next_operation(&mut self) -> Option<PlanningOperation> {
        if let Some(OperationSequenceOperation::MoveSequence(ms)) = self.ops.front_mut() {
            let m = ms.next_move();
            if ms.is_empty() {
                self.ops.pop_front();
            }
            m
        } else {
            self.ops.pop_front().map(|o| o.into())
        }
    }
}

#[derive(Debug)]
enum MoveSequenceOperation {
    Move(PlanningMove),
    Fill,
}

impl MoveSequenceOperation {
    fn is_fill(&self) -> bool {
        matches!(self, MoveSequenceOperation::Fill)
    }
}

impl From<MoveSequenceOperation> for PlanningOperation {
    fn from(mso: MoveSequenceOperation) -> Self {
        match mso {
            MoveSequenceOperation::Move(m) => PlanningOperation::Move(m),
            MoveSequenceOperation::Fill => PlanningOperation::Fill,
        }
    }
}

#[derive(Debug, Default)]
pub struct MoveSequence {
    moves: VecDeque<MoveSequenceOperation>,
    flush_count: usize,
}

impl MoveSequence {
    pub(crate) fn add_fill(&mut self) {
        self.moves.push_back(MoveSequenceOperation::Fill);
    }

    pub(crate) fn add_move(&mut self, mut move_cmd: PlanningMove, toolhead_state: &ToolheadState) {
        if move_cmd.distance == 0.0 {
            self.add_fill();
            return;
        }
        if let Some(prev_move) = self.last_move() {
            move_cmd.apply_junction(prev_move, toolhead_state);
        }
        self.moves.push_back(MoveSequenceOperation::Move(move_cmd));
    }

    fn is_empty(&self) -> bool {
        self.moves.is_empty()
    }

    fn last_move(&self) -> Option<&PlanningMove> {
        self.moves.iter().rev().find_map(|o| match o {
            MoveSequenceOperation::Move(m) => Some(m),
            _ => None,
        })
    }

    fn process(&mut self, partial: bool) {
        if self.flush_count == self.moves.len() {
            // If there's nothing to flush, bail quickly
            return;
        }

        let mut delayed: Vec<(&mut PlanningMove, f64, f64)> = Vec::new();

        let mut next_end_v2 = 0.0;
        let mut next_smoothed_v2 = 0.0;
        let mut peak_cruise_v2 = 0.0;

        let mut update_flush_count = partial;
        let skip = if partial { self.flush_count } else { 0 };
        if !partial {
            self.flush_count = self.moves.len();
        }

        for (idx, m) in self.moves.iter_mut().enumerate().skip(skip).rev() {
            if let MoveSequenceOperation::Move(m) = m {
                let reachable_start_v2 = next_end_v2 + m.max_dv2;
                let start_v2 = m.max_start_v2.min(reachable_start_v2);
                let reachable_smoothed_v2 = next_smoothed_v2 + m.smoothed_dv2;
                let smoothed_v2 = m.max_smoothed_v2.min(reachable_smoothed_v2);
                if smoothed_v2 < reachable_smoothed_v2 {
                    if (smoothed_v2 + m.smoothed_dv2 > next_smoothed_v2) || !delayed.is_empty() {
                        if update_flush_count && peak_cruise_v2 != 0.0 {
                            self.flush_count = idx;
                            update_flush_count = false;
                        }

                        peak_cruise_v2 = m
                            .max_cruise_v2
                            .min((smoothed_v2 + reachable_smoothed_v2) * 0.5);

                        if !delayed.is_empty() {
                            if !update_flush_count && idx < self.flush_count {
                                let mut mc_v2 = peak_cruise_v2;
                                for (m, ms_v2, me_v2) in delayed.iter_mut().rev() {
                                    mc_v2 = mc_v2.min(*ms_v2);
                                    m.set_junction(ms_v2.min(mc_v2), mc_v2, me_v2.min(mc_v2));
                                }
                            }
                            delayed.clear();
                        }
                    }

                    if !update_flush_count && idx < self.flush_count {
                        let cruise_v2 = ((start_v2 + reachable_start_v2) * 0.5)
                            .min(m.max_cruise_v2)
                            .min(peak_cruise_v2);
                        m.set_junction(
                            start_v2.min(cruise_v2),
                            cruise_v2,
                            next_end_v2.min(cruise_v2),
                        );
                    }
                } else {
                    delayed.push((m, start_v2, next_end_v2));
                }
                next_end_v2 = start_v2;
                next_smoothed_v2 = smoothed_v2;
            }
        }

        if update_flush_count {
            self.flush_count = 0;
        }

        // Advance while the next operation is a fill
        while self.flush_count < self.moves.len() && self.moves[self.flush_count].is_fill() {
            self.flush_count += 1;
        }
    }

    fn flush(&mut self) {
        self.process(false);
    }

    fn next_move(&mut self) -> Option<PlanningOperation> {
        self.process(true);
        if self.flush_count == 0 {
            return None;
        }
        match self.moves.pop_front() {
            None => None,
            Some(v) => {
                self.flush_count -= 1;
                Some(v.into())
            }
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct PrinterLimits {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_accel_to_decel: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub minimum_cruise_ratio: Option<f64>,
    pub square_corner_velocity: f64,
    #[serde(skip)]
    pub junction_deviation: f64,
    #[serde(skip)]
    pub accel_to_decel: f64,
    pub instant_corner_velocity: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub firmware_retraction: Option<FirmwareRetractionOptions>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mm_per_arc_segment: Option<f64>,
    pub move_checkers: Vec<MoveChecker>,
    pub initial_coordinate_mode: PositionMode,
    pub initial_extrusion_mode: PositionMode,
}

impl Default for PrinterLimits {
    fn default() -> Self {
        PrinterLimits {
            max_velocity: 100.0,
            max_acceleration: 100.0,
            max_accel_to_decel: Some(50.0),
            minimum_cruise_ratio: None,
            square_corner_velocity: 5.0,
            junction_deviation: Self::scv_to_jd(5.0, 100000.0),
            accel_to_decel: 50.0,
            instant_corner_velocity: 1.0,
            move_checkers: vec![],
            firmware_retraction: None,
            mm_per_arc_segment: None,
            initial_coordinate_mode: PositionMode::Absolute,
            // Compatibility default: slicer start macros commonly contain the M83 that the
            // estimator cannot see. Klipper itself starts in absolute extrusion mode.
            initial_extrusion_mode: PositionMode::Relative,
        }
    }
}

impl PrinterLimits {
    pub fn recalculate(&mut self) {
        self.update_junction_deviation();
        self.update_accel_to_decel();
    }

    pub fn set_max_velocity(&mut self, v: f64) {
        self.max_velocity = v;
    }

    pub fn set_max_acceleration(&mut self, v: f64) {
        self.max_acceleration = v;
        self.update_junction_deviation();
        self.update_accel_to_decel();
    }

    pub fn set_max_accel_to_decel(&mut self, v: f64) {
        self.max_accel_to_decel = Some(v);
        self.minimum_cruise_ratio = None;
        self.update_accel_to_decel();
    }

    pub fn set_minimum_cruise_ratio(&mut self, v: f64) {
        self.max_accel_to_decel = None;
        self.minimum_cruise_ratio = Some(v.clamp(0.0, 1.0));
        self.update_accel_to_decel();
    }

    pub fn set_square_corner_velocity(&mut self, scv: f64) {
        self.square_corner_velocity = scv;
        self.update_junction_deviation();
    }

    pub fn set_instant_corner_velocity(&mut self, icv: f64) {
        self.instant_corner_velocity = icv;
    }

    fn scv_to_jd(scv: f64, acceleration: f64) -> f64 {
        let scv2 = scv * scv;
        scv2 * (2.0f64.sqrt() - 1.0) / acceleration
    }

    fn update_junction_deviation(&mut self) {
        self.junction_deviation =
            Self::scv_to_jd(self.square_corner_velocity, self.max_acceleration);
    }

    fn update_accel_to_decel(&mut self) {
        self.accel_to_decel = match (self.minimum_cruise_ratio, self.max_accel_to_decel) {
            (Some(v), _) => self.max_acceleration * (1.0 - v.clamp(0.0, 1.0)),
            (_, Some(v)) => v.min(self.max_acceleration),
            _ => 50.0f64.min(self.max_acceleration),
        }
    }
}

#[derive(Debug, Default, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PositionMode {
    #[default]
    Absolute,
    Relative,
}

#[derive(Debug)]
pub struct ToolheadState {
    /// Physical position submitted to the motion planner (Klipper `last_position`).
    pub position: Vec4,
    /// Origin used to transform absolute G-code coordinates into physical coordinates.
    pub base_position: Vec4,
    pub homing_position: Vec4,
    pub position_modes: [PositionMode; 4],
    pub limits: PrinterLimits,

    pub velocity: f64,
    pub speed_factor: f64,
    pub extrude_factor: f64,
    saved_states: HashMap<String, SavedGcodeState>,
}

#[derive(Debug, Clone)]
struct SavedGcodeState {
    position: Vec4,
    base_position: Vec4,
    homing_position: Vec4,
    position_modes: [PositionMode; 4],
    velocity: f64,
    speed_factor: f64,
    extrude_factor: f64,
}

impl ToolheadState {
    fn from_limits(limits: PrinterLimits) -> Self {
        let coordinate_mode = limits.initial_coordinate_mode;
        let extrusion_mode = limits.initial_extrusion_mode;
        ToolheadState {
            position: Vec4::ZERO,
            base_position: Vec4::ZERO,
            homing_position: Vec4::ZERO,
            position_modes: [
                coordinate_mode,
                coordinate_mode,
                coordinate_mode,
                extrusion_mode,
            ],
            velocity: limits.max_velocity,
            speed_factor: 1.0 / 60.0,
            extrude_factor: 1.0,
            saved_states: HashMap::new(),
            limits,
        }
    }

    pub fn perform_move(&mut self, axes: [Option<f64>; 4]) -> PlanningMove {
        let mut new_pos = self.position;

        for (axis, v) in axes.iter().enumerate() {
            if let Some(v) = v {
                let value = if axis == 3 {
                    *v * self.extrude_factor
                } else {
                    *v
                };
                new_pos.as_mut()[axis] = match self.position_modes[axis] {
                    PositionMode::Relative => new_pos.as_ref()[axis] + value,
                    PositionMode::Absolute => self.base_position.as_ref()[axis] + value,
                };
            }
        }

        self.perform_physical_move(new_pos, None)
    }

    pub fn perform_relative_move(
        &mut self,
        axes: [Option<f64>; 4],
        kind: Option<Kind>,
    ) -> PlanningMove {
        let cur_pos_mode = self.position_modes;
        self.position_modes = [PositionMode::Relative; 4];
        let mut pm = self.perform_move(axes);
        pm.kind = kind;
        self.position_modes = cur_pos_mode;
        pm
    }

    pub fn set_speed(&mut self, v: f64) {
        if v <= 0.0 {
            panic!("Requested toolhead velocity {} <= 0", v);
        }
        self.velocity = v
    }

    pub fn set_gcode_speed(&mut self, feedrate: f64) {
        self.set_speed(feedrate * self.speed_factor);
    }

    pub fn gcode_position(&self) -> Vec4 {
        let mut position = self.position - self.base_position;
        position.w /= self.extrude_factor;
        position
    }

    fn set_gcode_position(&mut self, axes: [Option<f64>; 4]) {
        if axes.iter().all(Option::is_none) {
            self.base_position = self.position;
            return;
        }
        for (axis, value) in axes.iter().enumerate() {
            if let Some(value) = value {
                let value = if axis == 3 {
                    value * self.extrude_factor
                } else {
                    *value
                };
                self.base_position.as_mut()[axis] = self.position.as_ref()[axis] - value;
            }
        }
    }

    fn set_speed_factor(&mut self, percent: f64) {
        if percent <= 0.0 {
            return;
        }
        let gcode_speed = self.velocity / self.speed_factor;
        self.speed_factor = percent / (60.0 * 100.0);
        self.velocity = gcode_speed * self.speed_factor;
    }

    fn set_extrude_factor(&mut self, percent: f64) {
        if percent <= 0.0 {
            return;
        }
        let new_factor = percent / 100.0;
        let e_value = (self.position.w - self.base_position.w) / self.extrude_factor;
        self.base_position.w = self.position.w - e_value * new_factor;
        self.extrude_factor = new_factor;
    }

    fn save_gcode_state(&mut self, name: &str) {
        self.saved_states.insert(
            name.into(),
            SavedGcodeState {
                position: self.position,
                base_position: self.base_position,
                homing_position: self.homing_position,
                position_modes: self.position_modes,
                velocity: self.velocity,
                speed_factor: self.speed_factor,
                extrude_factor: self.extrude_factor,
            },
        );
    }

    fn restore_gcode_state(
        &mut self,
        name: &str,
        move_requested: bool,
        move_speed: Option<f64>,
    ) -> Result<Option<PlanningMove>, ()> {
        let state = self.saved_states.get(name).cloned().ok_or(())?;
        let e_diff = self.position.w - state.position.w;
        self.position_modes = state.position_modes;
        self.base_position = state.base_position;
        self.base_position.w += e_diff;
        self.homing_position = state.homing_position;
        self.velocity = state.velocity;
        self.speed_factor = state.speed_factor;
        self.extrude_factor = state.extrude_factor;

        if !move_requested {
            return Ok(None);
        }
        let target = Vec4::new(
            state.position.x,
            state.position.y,
            state.position.z,
            self.position.w,
        );
        Ok(Some(self.perform_physical_move(target, move_speed)))
    }

    fn set_gcode_offset(
        &mut self,
        offsets: [Option<f64>; 4],
        adjustments: [Option<f64>; 4],
        move_requested: bool,
        move_speed: Option<f64>,
    ) -> Option<PlanningMove> {
        let mut move_delta = Vec4::ZERO;
        for axis in 0..4 {
            let offset = offsets[axis].or_else(|| {
                adjustments[axis].map(|adjustment| self.homing_position.as_ref()[axis] + adjustment)
            });
            if let Some(offset) = offset {
                let delta = offset - self.homing_position.as_ref()[axis];
                move_delta.as_mut()[axis] = delta;
                self.base_position.as_mut()[axis] += delta;
                self.homing_position.as_mut()[axis] = offset;
            }
        }
        if !move_requested {
            return None;
        }
        Some(self.perform_physical_move(self.position + move_delta, move_speed))
    }

    pub(crate) fn perform_physical_move(
        &mut self,
        new_position: Vec4,
        requested_velocity: Option<f64>,
    ) -> PlanningMove {
        let previous_velocity = self.velocity;
        if let Some(velocity) = requested_velocity {
            self.set_speed(velocity);
        }
        let mut planning_move = PlanningMove::new(self.position, new_position, self);
        for checker in &self.limits.move_checkers {
            checker.check(&mut planning_move);
        }
        self.position = new_position;
        self.velocity = previous_velocity;
        planning_move
    }

    fn extruder_junction_speed_v2(&self, cur_move: &PlanningMove, prev_move: &PlanningMove) -> f64 {
        let diff_r = (cur_move.rate.w - prev_move.rate.w).abs();
        if diff_r > 0.0 {
            let v = self.limits.instant_corner_velocity / diff_r;
            v * v
        } else {
            cur_move.max_cruise_v2
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MoveChecker {
    AxisLimiter {
        axis: Vec3,
        max_velocity: f64,
        max_accel: f64,
    },
    ExtruderLimiter {
        max_velocity: f64,
        max_accel: f64,
    },
}

impl MoveChecker {
    pub fn check(&self, move_cmd: &mut PlanningMove) {
        match self {
            Self::AxisLimiter {
                axis,
                max_velocity,
                max_accel,
            } => Self::check_axis(move_cmd, *axis, *max_velocity, *max_accel),
            Self::ExtruderLimiter {
                max_velocity,
                max_accel,
            } => Self::check_extruder(move_cmd, *max_velocity, *max_accel),
        }
    }

    fn check_axis(move_cmd: &mut PlanningMove, axis: Vec3, max_velocity: f64, max_accel: f64) {
        if move_cmd.is_zero_distance() {
            return;
        }
        let ratio = move_cmd.distance / (move_cmd.delta().xyz().dot(axis)).abs();
        move_cmd.limit_speed(max_velocity * ratio, max_accel * ratio);
    }

    fn check_extruder(move_cmd: &mut PlanningMove, max_velocity: f64, max_accel: f64) {
        if !move_cmd.is_extrude_only_move() {
            return;
        }
        let e_rate = move_cmd.rate.w;
        if move_cmd.rate.xy() == glam::DVec2::ZERO || e_rate < 0.0 {
            let inv_extrude_r = 1.0 / e_rate.abs();
            move_cmd.limit_speed(max_velocity * inv_extrude_r, max_accel * inv_extrude_r);
        }
    }
}
