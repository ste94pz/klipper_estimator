use crate::planner::PlanningMove;
use glam::{DVec3 as Vec3, Vec4Swizzles};
use serde::{Deserialize, Serialize};

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
    Unsupported {
        backend: String,
        reason: String,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveOutOfRange {
    pub axis: usize,
    pub position: f64,
    pub minimum: f64,
    pub maximum: f64,
}

/// Planner-facing interface. Configuration loading remains in the tool crate;
/// the planner only receives resolved kinematics data.
pub trait KinematicsChecker {
    fn check_move(&self, move_cmd: &mut PlanningMove) -> Result<(), MoveOutOfRange>;
}

impl Kinematics {
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

impl KinematicsChecker for Kinematics {
    fn check_move(&self, move_cmd: &mut PlanningMove) -> Result<(), MoveOutOfRange> {
        let config = match self {
            Self::CartesianFamily { config } => config,
            Self::Unconfigured | Self::Unsupported { .. } => return Ok(()),
        };

        // Port of check_move/_check_endstops in cartesian.py, corexy.py,
        // corexz.py, hybrid_corexy.py, and hybrid_corexz.py at the pinned
        // Klipper reference commit above. Estimation treats configured axes as
        // homed because a print file is planned for an already prepared toolhead.
        let delta = move_cmd.delta().xyz();
        let end = move_cmd.end.xyz();
        for axis in 0..3 {
            if delta.as_ref()[axis] != 0.0
                && (end.as_ref()[axis] < config.axis_minimum.as_ref()[axis]
                    || end.as_ref()[axis] > config.axis_maximum.as_ref()[axis])
            {
                return Err(MoveOutOfRange {
                    axis,
                    position: end.as_ref()[axis],
                    minimum: config.axis_minimum.as_ref()[axis],
                    maximum: config.axis_maximum.as_ref()[axis],
                });
            }
        }

        if delta.z != 0.0 {
            let z_ratio = move_cmd.distance / delta.z.abs();
            move_cmd.limit_speed(
                config.max_z_velocity * z_ratio,
                config.max_z_accel * z_ratio,
            );
        }
        Ok(())
    }
}
