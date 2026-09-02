use crate::planner::PlanningMove;
use glam::{DVec3 as Vec3, Vec4Swizzles};
use serde::{Deserialize, Serialize};

const DELTA_SLOW_RATIO: f64 = 3.0;

/// Cartesian-family backends whose `check_move` behavior is equivalent in
/// Klipper at f0892d82b0f1c1228454f09eb508eddde2250f4b.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CartesianKinematicsKind {
    Cartesian,
    Corexy,
    Corexz,
    HybridCorexy,
    HybridCorexz,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CartesianKinematics {
    pub kind: CartesianKinematicsKind,
    pub axis_minimum: Vec3,
    pub axis_maximum: Vec3,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DeltaKinematics {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub minimum_z: f64,
    pub radius: f64,
    pub print_radius: f64,
    pub arm_lengths: [f64; 3],
    pub tower_angles: [f64; 3],
    pub position_endstops: [f64; 3],
    pub step_distances: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PolarKinematics {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub max_angular_velocity: f64,
    pub maximum_radius: f64,
    pub minimum_z: f64,
    pub maximum_z: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DeltesianKinematics {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub minimum_z: f64,
    pub minimum_angle: f64,
    pub print_width: Option<f64>,
    pub slow_ratio: f64,
    pub arm_x_lengths: [f64; 2],
    pub arm_lengths: [f64; 2],
    pub position_endstops: [f64; 2],
    pub y_range: [f64; 2],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RotaryDeltaKinematics {
    pub max_z_velocity: f64,
    pub minimum_z: f64,
    pub shoulder_radius: f64,
    pub shoulder_height: f64,
    pub upper_arm_lengths: [f64; 3],
    pub lower_arm_lengths: [f64; 3],
    pub tower_angles: [f64; 3],
    pub position_endstops: [f64; 3],
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Kinematics {
    /// Compatibility state for legacy estimator configurations which did not
    /// record a Klipper kinematics backend.
    #[default]
    Unconfigured,
    CartesianFamily {
        config: CartesianKinematics,
    },
    Delta {
        config: DeltaKinematics,
    },
    Polar {
        config: PolarKinematics,
    },
    Deltesian {
        config: DeltesianKinematics,
    },
    RotaryDelta {
        config: RotaryDeltaKinematics,
    },
    Unsupported {
        backend: String,
        reason: String,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum MoveOutOfRange {
    Axis {
        axis: usize,
        position: f64,
        minimum: f64,
        maximum: f64,
    },
    Reachability {
        backend: &'static str,
        position: Vec3,
    },
}

/// Planner-facing interface. Reachability and motion limits are deliberately
/// separate so a geometry error is never disguised as a plausible duration.
pub trait KinematicsChecker {
    fn validate_move(&self, move_cmd: &PlanningMove) -> Result<(), MoveOutOfRange>;
    fn limit_move(&self, move_cmd: &mut PlanningMove);

    fn check_move(&self, move_cmd: &mut PlanningMove) -> Result<(), MoveOutOfRange> {
        self.validate_move(move_cmd)?;
        self.limit_move(move_cmd);
        Ok(())
    }
}

impl Kinematics {
    pub fn backend_name(&self) -> &str {
        match self {
            Self::Unconfigured => "unconfigured",
            Self::CartesianFamily { config } => match config.kind {
                CartesianKinematicsKind::Cartesian => "cartesian",
                CartesianKinematicsKind::Corexy => "corexy",
                CartesianKinematicsKind::Corexz => "corexz",
                CartesianKinematicsKind::HybridCorexy => "hybrid_corexy",
                CartesianKinematicsKind::HybridCorexz => "hybrid_corexz",
            },
            Self::Delta { .. } => "delta",
            Self::Polar { .. } => "polar",
            Self::Deltesian { .. } => "deltesian",
            Self::RotaryDelta { .. } => "rotary_delta",
            Self::Unsupported { backend, .. } => backend,
        }
    }

    pub fn is_unconfigured(&self) -> bool {
        matches!(self, Self::Unconfigured)
    }

    pub fn unsupported(backend: impl Into<String>, reason: impl Into<String>) -> Self {
        Self::Unsupported {
            backend: backend.into(),
            reason: reason.into(),
        }
    }

    pub fn unsupported_details(&self) -> Option<(&str, &str)> {
        match self {
            Self::Unsupported { backend, reason } => Some((backend, reason)),
            _ => None,
        }
    }
}

fn axis_violation(axis: usize, position: f64, minimum: f64, maximum: f64) -> MoveOutOfRange {
    MoveOutOfRange::Axis {
        axis,
        position,
        minimum,
        maximum,
    }
}

impl KinematicsChecker for Kinematics {
    fn validate_move(&self, move_cmd: &PlanningMove) -> Result<(), MoveOutOfRange> {
        match self {
            Self::CartesianFamily { config } => config.validate_move(move_cmd),
            Self::Delta { config } => config.validate_move(move_cmd),
            Self::Polar { config } => config.validate_move(move_cmd),
            Self::Deltesian { config } => config.validate_move(move_cmd),
            Self::RotaryDelta { config } => config.validate_move(move_cmd),
            Self::Unconfigured | Self::Unsupported { .. } => Ok(()),
        }
    }

    fn limit_move(&self, move_cmd: &mut PlanningMove) {
        match self {
            Self::CartesianFamily { config } => config.limit_move(move_cmd),
            Self::Delta { config } => config.limit_move(move_cmd),
            Self::Polar { config } => config.limit_move(move_cmd),
            Self::Deltesian { config } => config.limit_move(move_cmd),
            Self::RotaryDelta { config } => config.limit_move(move_cmd),
            Self::Unconfigured | Self::Unsupported { .. } => {}
        }
    }
}

impl KinematicsChecker for CartesianKinematics {
    fn validate_move(&self, move_cmd: &PlanningMove) -> Result<(), MoveOutOfRange> {
        // Port of _check_endstops in cartesian.py, corexy.py, corexz.py,
        // hybrid_corexy.py, and hybrid_corexz.py at the pinned commit above.
        let delta = move_cmd.delta().xyz();
        let end = move_cmd.end.xyz();
        for axis in 0..3 {
            if delta.as_ref()[axis] != 0.0
                && (end.as_ref()[axis] < self.axis_minimum.as_ref()[axis]
                    || end.as_ref()[axis] > self.axis_maximum.as_ref()[axis])
            {
                return Err(axis_violation(
                    axis,
                    end.as_ref()[axis],
                    self.axis_minimum.as_ref()[axis],
                    self.axis_maximum.as_ref()[axis],
                ));
            }
        }
        Ok(())
    }

    fn limit_move(&self, move_cmd: &mut PlanningMove) {
        let delta = move_cmd.delta().xyz();
        if delta.z != 0.0 {
            let z_ratio = move_cmd.distance / delta.z.abs();
            move_cmd.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio);
        }
    }
}

impl DeltaKinematics {
    fn parameters(&self) -> (f64, f64, f64, f64, f64, f64) {
        let min_arm = self
            .arm_lengths
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let abs_endstops = std::array::from_fn::<_, 3, _>(|i| {
            self.position_endstops[i] + (self.arm_lengths[i].powi(2) - self.radius.powi(2)).sqrt()
        });
        let max_z = self
            .position_endstops
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let limit_z = (0..3)
            .map(|i| abs_endstops[i] - self.arm_lengths[i])
            .fold(f64::INFINITY, f64::min);
        let half_step = self
            .step_distances
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
            * 0.5;
        let ratio_radius = |ratio: f64| {
            ratio * (min_arm.powi(2) / (ratio.powi(2) + 1.0) - half_step.powi(2)).sqrt() + half_step
                - self.radius
        };
        let slow_xy2 = ratio_radius(DELTA_SLOW_RATIO).powi(2);
        let very_slow_xy2 = ratio_radius(2.0 * DELTA_SLOW_RATIO).powi(2);
        let max_xy2 = self
            .print_radius
            .min(min_arm - self.radius)
            .min(ratio_radius(4.0 * DELTA_SLOW_RATIO))
            .powi(2);
        (min_arm, max_z, limit_z, max_xy2, slow_xy2, very_slow_xy2)
    }
}

impl KinematicsChecker for DeltaKinematics {
    // Port of DeltaKinematics.check_move in delta.py at the pinned commit.
    fn validate_move(&self, move_cmd: &PlanningMove) -> Result<(), MoveOutOfRange> {
        let end = move_cmd.end.xyz();
        let (min_arm, max_z, limit_z, max_xy2, _, _) = self.parameters();
        let mut allowed_xy2 = max_xy2;
        if end.z > limit_z {
            let above = end.z - limit_z;
            let radius = self.radius - (min_arm.powi(2) - (min_arm - above).powi(2)).sqrt();
            allowed_xy2 = allowed_xy2.min(radius.powi(2));
        }
        if end.x.powi(2) + end.y.powi(2) > allowed_xy2 || end.z < self.minimum_z || end.z > max_z {
            return Err(MoveOutOfRange::Reachability {
                backend: "delta",
                position: end,
            });
        }
        Ok(())
    }

    fn limit_move(&self, move_cmd: &mut PlanningMove) {
        let delta = move_cmd.delta().xyz();
        if delta.z != 0.0 {
            let ratio = move_cmd.distance / delta.z.abs();
            move_cmd.limit_speed(self.max_z_velocity * ratio, self.max_z_accel * ratio);
        }
        let (_, _, _, _, slow_xy2, very_slow_xy2) = self.parameters();
        let start = move_cmd.start.xyz();
        let end = move_cmd.end.xyz();
        let extreme = (start.x.powi(2) + start.y.powi(2)).max(end.x.powi(2) + end.y.powi(2));
        if extreme > slow_xy2 {
            let scale = if extreme > very_slow_xy2 { 0.25 } else { 0.5 };
            move_cmd.limit_speed(self.max_velocity * scale, self.max_accel * scale);
        }
    }
}

impl KinematicsChecker for PolarKinematics {
    // Port of PolarKinematics.check_move and distance_to_center in polar.py.
    fn validate_move(&self, move_cmd: &PlanningMove) -> Result<(), MoveOutOfRange> {
        let delta = move_cmd.delta().xyz();
        let end = move_cmd.end.xyz();
        if end.x.powi(2) + end.y.powi(2) > self.maximum_radius.powi(2) {
            return Err(MoveOutOfRange::Reachability {
                backend: "polar",
                position: end,
            });
        }
        if delta.z != 0.0 && (end.z < self.minimum_z || end.z > self.maximum_z) {
            return Err(axis_violation(2, end.z, self.minimum_z, self.maximum_z));
        }
        Ok(())
    }

    fn limit_move(&self, move_cmd: &mut PlanningMove) {
        let delta = move_cmd.delta().xyz();
        if delta.z != 0.0 {
            let ratio = move_cmd.distance / delta.z.abs();
            move_cmd.limit_speed(self.max_z_velocity * ratio, self.max_z_accel * ratio);
        }
        if (delta.x != 0.0 || delta.y != 0.0) && self.max_angular_velocity != 0.0 {
            let start = move_cmd.start.xy();
            let end = move_cmd.end.xy();
            let segment = end - start;
            let dot = segment.dot(-start);
            let length2 = segment.length_squared();
            let min_dist = if dot <= 0.0 {
                start.length()
            } else if dot >= length2 {
                end.length()
            } else {
                segment.perp_dot(-start).abs() / length2.sqrt()
            };
            if min_dist != 0.0 {
                let angular = move_cmd.max_cruise_v2.sqrt() / min_dist;
                if self.max_angular_velocity < angular {
                    let scale = self.max_angular_velocity / angular;
                    move_cmd.limit_speed(self.max_velocity * scale, self.max_accel * scale);
                }
            }
        }
    }
}

impl DeltesianKinematics {
    fn x_limits(&self) -> [f64; 2] {
        let cosine = self.minimum_angle.to_radians().cos();
        let x_min = (-self.arm_x_lengths[0])
            .max(-(cosine * self.arm_lengths[1] - self.arm_x_lengths[1]))
            .ceil();
        let x_max = self.arm_x_lengths[1]
            .min(cosine * self.arm_lengths[0] - self.arm_x_lengths[0])
            .floor();
        match self.print_width {
            Some(width) if width != 0.0 => [-width * 0.5, width * 0.5],
            _ => [x_min, x_max],
        }
    }

    fn pillars_z_max(&self, x: f64) -> f64 {
        let abs_endstop = std::array::from_fn::<_, 2, _>(|i| {
            self.position_endstops[i]
                + (self.arm_lengths[i].powi(2) - self.arm_x_lengths[i].powi(2)).sqrt()
        });
        (0..2)
            .map(|i| {
                let horizontal = if i == 0 {
                    self.arm_x_lengths[i] + x
                } else {
                    self.arm_x_lengths[i] - x
                };
                abs_endstop[i] - (self.arm_lengths[i].powi(2) - horizontal.powi(2)).sqrt()
            })
            .fold(f64::INFINITY, f64::min)
    }

    fn max_z(&self) -> f64 {
        let limits = self.x_limits();
        self.pillars_z_max(limits[0])
            .min(self.pillars_z_max(limits[1]))
    }

    fn slow_limits(&self) -> Option<(f64, f64)> {
        if self.slow_ratio == 0.0 {
            return None;
        }
        let sr2 = self.slow_ratio.powi(2);
        let slow_x2 = (0..2)
            .map(|i| {
                (sr2 * self.arm_lengths[i].powi(2) / (sr2 + 1.0)).sqrt() - self.arm_x_lengths[i]
            })
            .fold(f64::INFINITY, f64::min)
            .powi(2);
        let very_slow_x2 = (0..2)
            .map(|i| {
                ((2.0 * sr2 * self.arm_lengths[i].powi(2)) / (2.0 * sr2 + 1.0)).sqrt()
                    - self.arm_x_lengths[i]
            })
            .fold(f64::INFINITY, f64::min)
            .powi(2);
        Some((slow_x2, very_slow_x2))
    }
}

impl KinematicsChecker for DeltesianKinematics {
    // Port of DeltesianKinematics.check_move in deltesian.py.
    fn validate_move(&self, move_cmd: &PlanningMove) -> Result<(), MoveOutOfRange> {
        let delta = move_cmd.delta().xyz();
        let end = move_cmd.end.xyz();
        let max_z = self.max_z();
        let z_max = if end.z > max_z {
            self.pillars_z_max(end.x)
        } else {
            max_z
        };
        let ranges = [self.x_limits(), self.y_range, [self.minimum_z, z_max]];
        for (axis, range) in ranges.iter().enumerate() {
            if delta.as_ref()[axis] != 0.0
                && (end.as_ref()[axis] < range[0] || end.as_ref()[axis] > range[1])
            {
                return Err(axis_violation(axis, end.as_ref()[axis], range[0], range[1]));
            }
        }
        Ok(())
    }

    fn limit_move(&self, move_cmd: &mut PlanningMove) {
        let delta = move_cmd.delta().xyz();
        if delta.z != 0.0 {
            let ratio = move_cmd.distance / delta.z.abs();
            move_cmd.limit_speed(self.max_z_velocity * ratio, self.max_z_accel * ratio);
        }
        if delta.x != 0.0 {
            if let Some((slow_x2, very_slow_x2)) = self.slow_limits() {
                let move_x2 = move_cmd.start.x.powi(2).max(move_cmd.end.x.powi(2));
                if move_x2 > very_slow_x2 {
                    move_cmd.limit_speed(self.max_velocity * 0.25, self.max_accel * 0.25);
                } else if move_x2 > slow_x2 {
                    move_cmd.limit_speed(self.max_velocity * 0.5, self.max_accel * 0.5);
                }
            }
        }
    }
}

impl RotaryDeltaKinematics {
    fn limits(&self) -> (f64, f64, f64) {
        let max_z = self
            .position_endstops
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let max_radius = self
            .upper_arm_lengths
            .iter()
            .copied()
            .map(|arm| self.shoulder_radius + arm)
            .fold(f64::INFINITY, f64::min)
            .min(
                self.lower_arm_lengths
                    .iter()
                    .copied()
                    .map(|arm| arm - self.shoulder_radius)
                    .fold(f64::INFINITY, f64::min),
            );
        let limit_z = (0..3)
            .map(|i| {
                let dx = -self.shoulder_radius;
                let dy = self.position_endstops[i] - self.shoulder_height;
                let upper2 = self.upper_arm_lengths[i].powi(2);
                let lower2 = self.lower_arm_lengths[i].powi(2);
                let c1 = 0.5 / dy * (dx * dx + dy * dy + upper2 - lower2);
                let c2 = dx / dy;
                let scale = c2 * c2 + 1.0;
                let elbow_x = c1 * c2 + (scale * upper2 - c1 * c1).sqrt();
                let elbow_y = (c1 * scale - c2 * elbow_x) / scale;
                self.shoulder_height + elbow_y - self.lower_arm_lengths[i]
            })
            .fold(f64::INFINITY, f64::min);
        (max_z, max_radius.powi(2), limit_z)
    }
}

impl KinematicsChecker for RotaryDeltaKinematics {
    // Port of RotaryDeltaKinematics.check_move in rotary_delta.py.
    fn validate_move(&self, move_cmd: &PlanningMove) -> Result<(), MoveOutOfRange> {
        let end = move_cmd.end.xyz();
        let (max_z, max_xy2, limit_z) = self.limits();
        let allowed_xy2 = if end.z > limit_z {
            max_xy2.min((max_z - end.z).powi(2))
        } else {
            max_xy2
        };
        if end.x.powi(2) + end.y.powi(2) > allowed_xy2 || end.z < self.minimum_z || end.z > max_z {
            return Err(MoveOutOfRange::Reachability {
                backend: "rotary_delta",
                position: end,
            });
        }
        Ok(())
    }

    fn limit_move(&self, move_cmd: &mut PlanningMove) {
        if move_cmd.delta().z != 0.0 {
            // Klipper supplies the move's existing acceleration, so rotary
            // delta constrains only velocity here.
            move_cmd.limit_speed(self.max_z_velocity, move_cmd.acceleration);
        }
    }
}
