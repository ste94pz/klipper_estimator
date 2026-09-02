use std::collections::BTreeMap;

use glam::DVec4 as Vec4;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MotionTransformConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bed_mesh: Option<BedMeshConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub skew_correction: Option<SkewCorrectionConfig>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub unsupported_active: Vec<String>,
}

impl MotionTransformConfig {
    pub fn is_empty(&self) -> bool {
        self.bed_mesh.is_none()
            && self.skew_correction.is_none()
            && self.unsupported_active.is_empty()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct BedMeshConfig {
    pub profiles: BTreeMap<String, BedMeshProfile>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub initial_profile: Option<String>,
    pub fade_start: f64,
    pub fade_end: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fade_target: Option<f64>,
    pub split_delta_z: f64,
    pub move_check_distance: f64,
}

impl Default for BedMeshConfig {
    fn default() -> Self {
        Self {
            profiles: BTreeMap::new(),
            initial_profile: None,
            fade_start: 1.0,
            fade_end: 0.0,
            fade_target: None,
            split_delta_z: 0.025,
            move_check_distance: 5.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BedMeshProfile {
    pub min: [f64; 2],
    pub max: [f64; 2],
    /// The fully interpolated matrix in increasing Y order, as exposed by Klipper.
    pub matrix: Vec<Vec<f64>>,
}

impl BedMeshProfile {
    #[allow(clippy::needless_range_loop)] // Klipper interpolation is expressed by matrix axes.
    pub fn from_probed(
        min: [f64; 2],
        max: [f64; 2],
        points: Vec<Vec<f64>>,
        mesh_pps: [usize; 2],
        algorithm: &str,
        tension: f64,
    ) -> Option<Self> {
        let probed_width = points.first().map_or(0, Vec::len);
        if probed_width < 2
            || points.len() < 2
            || points.iter().any(|row| row.len() != probed_width)
        {
            return None;
        }
        let x_mult = mesh_pps[0] + 1;
        let y_mult = mesh_pps[1] + 1;
        let width = (probed_width - 1) * x_mult + 1;
        let height = (points.len() - 1) * y_mult + 1;
        let mut matrix = vec![vec![0.0; width]; height];
        for (y, row) in points.iter().enumerate() {
            for (x, value) in row.iter().enumerate() {
                matrix[y * y_mult][x * x_mult] = *value;
            }
        }
        match algorithm {
            "direct" if mesh_pps == [0, 0] => matrix = points,
            "lagrange" => {
                let xs: Vec<_> = (0..probed_width)
                    .map(|index| coordinate(index * x_mult, min[0], max[0], width))
                    .collect();
                let ys: Vec<_> = (0..points.len())
                    .map(|index| coordinate(index * y_mult, min[1], max[1], height))
                    .collect();
                for y in (0..height).step_by(y_mult) {
                    for x in 0..width {
                        if x % x_mult != 0 {
                            let value = coordinate(x, min[0], max[0], width);
                            matrix[y][x] = lagrange(&xs, value, |index| matrix[y][index * x_mult]);
                        }
                    }
                }
                for x in 0..width {
                    for y in 0..height {
                        if y % y_mult != 0 {
                            let value = coordinate(y, min[1], max[1], height);
                            matrix[y][x] = lagrange(&ys, value, |index| matrix[index * y_mult][x]);
                        }
                    }
                }
            }
            "bicubic" if probed_width >= 4 && points.len() >= 4 => {
                for y in (0..height).step_by(y_mult) {
                    for x in 0..width {
                        if x % x_mult != 0 {
                            matrix[y][x] = cardinal_at(&matrix[y], x, x_mult, tension);
                        }
                    }
                }
                for x in 0..width {
                    let column: Vec<_> = matrix.iter().map(|row| row[x]).collect();
                    for y in 0..height {
                        if y % y_mult != 0 {
                            matrix[y][x] = cardinal_at(&column, y, y_mult, tension);
                        }
                    }
                }
            }
            _ => return None,
        }
        let profile = Self { min, max, matrix };
        profile.is_valid().then_some(profile)
    }

    pub fn is_valid(&self) -> bool {
        let width = self.matrix.first().map_or(0, Vec::len);
        width >= 2
            && self.matrix.len() >= 2
            && self.matrix.iter().all(|row| row.len() == width)
            && self.max[0] > self.min[0]
            && self.max[1] > self.min[1]
            && self.matrix.iter().flatten().all(|value| value.is_finite())
    }

    fn calc_z(&self, x: f64, y: f64, offsets: [f64; 2]) -> f64 {
        let width = self.matrix[0].len();
        let height = self.matrix.len();
        let (tx, xi) = linear_index(x + offsets[0], self.min[0], self.max[0], width);
        let (ty, yi) = linear_index(y + offsets[1], self.min[1], self.max[1], height);
        let z0 = lerp(tx, self.matrix[yi][xi], self.matrix[yi][xi + 1]);
        let z1 = lerp(tx, self.matrix[yi + 1][xi], self.matrix[yi + 1][xi + 1]);
        lerp(ty, z0, z1)
    }

    fn average(&self) -> f64 {
        let sum: f64 = self.matrix.iter().flatten().sum();
        // Mirrors bed_mesh.ZMesh.get_z_average() at KLIPPER_REFERENCE.
        (sum / self.matrix.iter().map(Vec::len).sum::<usize>() as f64 * 100.0).round() / 100.0
    }
}

fn coordinate(index: usize, min: f64, max: f64, count: usize) -> f64 {
    min + (max - min) * index as f64 / (count - 1) as f64
}

fn lagrange(points: &[f64], value: f64, sample: impl Fn(usize) -> f64) -> f64 {
    points.iter().enumerate().fold(0.0, |total, (i, point)| {
        let (numerator, denominator) = points
            .iter()
            .enumerate()
            .filter(|(j, _)| *j != i)
            .fold((1.0, 1.0), |(n, d), (_, other)| {
                (n * (value - other), d * (point - other))
            });
        total + sample(i) * numerator / denominator
    })
}

fn cardinal_at(values: &[f64], index: usize, multiplier: usize, tension: f64) -> f64 {
    let segment = (index / multiplier).min(values.len() / multiplier - 1);
    let p1_index = segment * multiplier;
    let p2_index = (p1_index + multiplier).min(values.len() - 1);
    let p0_index = p1_index.saturating_sub(multiplier);
    let p3_index = (p2_index + multiplier).min(values.len() - 1);
    let t = (index - p1_index) as f64 / multiplier as f64;
    let t2 = t * t;
    let t3 = t2 * t;
    let p0 = values[p0_index];
    let p1 = values[p1_index];
    let p2 = values[p2_index];
    let p3 = values[p3_index];
    let m1 = tension * (p2 - p0);
    let m2 = tension * (p3 - p1);
    p1 * (2.0 * t3 - 3.0 * t2 + 1.0)
        + p2 * (-2.0 * t3 + 3.0 * t2)
        + m1 * (t3 - 2.0 * t2 + t)
        + m2 * (t3 - t2)
}

fn linear_index(value: f64, min: f64, max: f64, count: usize) -> (f64, usize) {
    let distance = (max - min) / (count - 1) as f64;
    let raw = ((value - min) / distance).floor();
    let index = raw.max(0.0).min((count - 2) as f64) as usize;
    let coordinate = min + distance * index as f64;
    (((value - coordinate) / distance).clamp(0.0, 1.0), index)
}

fn lerp(t: f64, a: f64, b: f64) -> f64 {
    a + (b - a) * t
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SkewCorrectionConfig {
    pub profiles: BTreeMap<String, SkewFactors>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub initial_profile: Option<String>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct SkewFactors {
    pub xy: f64,
    pub xz: f64,
    pub yz: f64,
}

#[derive(Debug, Clone)]
pub(crate) struct MotionTransformState {
    config: MotionTransformConfig,
    active_mesh: Option<String>,
    mesh_offsets: [f64; 2],
    tool_offset: f64,
    skew: SkewFactors,
}

impl MotionTransformState {
    pub(crate) fn new(config: MotionTransformConfig) -> Self {
        let active_mesh = config
            .bed_mesh
            .as_ref()
            .and_then(|bed_mesh| bed_mesh.initial_profile.clone());
        let skew = config
            .skew_correction
            .as_ref()
            .and_then(|config| {
                config
                    .initial_profile
                    .as_ref()
                    .and_then(|name| config.profiles.get(name))
            })
            .copied()
            .unwrap_or_default();
        Self {
            config,
            active_mesh,
            mesh_offsets: [0.0; 2],
            tool_offset: 0.0,
            skew,
        }
    }

    pub(crate) fn unsupported_active(&self) -> &[String] {
        &self.config.unsupported_active
    }

    pub(crate) fn clear_bed_mesh(&mut self) {
        self.active_mesh = None;
        self.mesh_offsets = [0.0; 2];
        self.tool_offset = 0.0;
    }

    pub(crate) fn load_bed_mesh(&mut self, name: &str) -> bool {
        let found = self
            .config
            .bed_mesh
            .as_ref()
            .is_some_and(|config| config.profiles.contains_key(name));
        if found {
            self.active_mesh = Some(name.into());
            self.mesh_offsets = [0.0; 2];
            self.tool_offset = 0.0;
        }
        found
    }

    pub(crate) fn offset_bed_mesh(&mut self, x: Option<f64>, y: Option<f64>, zfade: Option<f64>) {
        if self.active_mesh.is_none() {
            return;
        }
        if let Some(x) = x {
            self.mesh_offsets[0] = x;
        }
        if let Some(y) = y {
            self.mesh_offsets[1] = y;
        }
        if let Some(zfade) = zfade {
            self.tool_offset = zfade;
        }
    }

    pub(crate) fn load_skew(&mut self, name: &str) -> bool {
        let factors = self
            .config
            .skew_correction
            .as_ref()
            .and_then(|config| config.profiles.get(name))
            .copied();
        if let Some(factors) = factors {
            self.skew = factors;
            true
        } else {
            false
        }
    }

    pub(crate) fn set_skew(&mut self, factors: SkewFactors) {
        self.skew = factors;
    }

    pub(crate) fn skew(&self) -> SkewFactors {
        self.skew
    }

    /// Port of `PrinterSkew.calc_skew`, `BedMesh.move`, and `MoveSplitter` at
    /// Klipper f0892d82b0f1c1228454f09eb508eddde2250f4b. Registration order in
    /// Klipper makes skew wrap bed mesh, so the mesh lookup sees skewed X/Y.
    pub(crate) fn transform_move(&self, start: Vec4, end: Vec4) -> Vec<Vec4> {
        let start = apply_skew(start, self.skew);
        let end = apply_skew(end, self.skew);
        let Some((config, profile)) = self.active_profile() else {
            return vec![end];
        };
        let factor = fade_factor(config, end.z, self.tool_offset);
        let fade_target = fade_target(config, profile);
        if factor == 0.0 {
            let mut target = end;
            target.z += fade_target;
            return vec![target];
        }

        let offset = |position: Vec4| {
            factor * (profile.calc_z(position.x, position.y, self.mesh_offsets) - fade_target)
                + fade_target
        };
        let delta = end - start;
        let distance = delta.truncate().length();
        if distance == 0.0 {
            let mut target = end;
            target.z += offset(end);
            return vec![target];
        }

        let mut result = Vec::new();
        let mut checked = 0.0;
        let mut last_offset = offset(start);
        while checked + config.move_check_distance < distance {
            checked += config.move_check_distance;
            let position = start + delta * (checked / distance);
            let next_offset = offset(position);
            if (next_offset - last_offset).abs() >= config.split_delta_z {
                let mut target = position;
                target.z += next_offset;
                result.push(target);
                last_offset = next_offset;
            }
        }
        let mut target = end;
        target.z += offset(end);
        result.push(target);
        result
    }

    pub(crate) fn transform_position(&self, position: Vec4) -> Vec4 {
        let position = apply_skew(position, self.skew);
        let Some((config, profile)) = self.active_profile() else {
            return position;
        };
        let factor = fade_factor(config, position.z, self.tool_offset);
        let target = fade_target(config, profile);
        let mut position = position;
        position.z +=
            factor * (profile.calc_z(position.x, position.y, self.mesh_offsets) - target) + target;
        position
    }

    pub(crate) fn untransform_position(&self, mut position: Vec4) -> Vec4 {
        if let Some((config, profile)) = self.active_profile() {
            let max_adjustment = profile.calc_z(position.x, position.y, self.mesh_offsets);
            let target = fade_target(config, profile);
            let adjustment = max_adjustment - target;
            let fade_z = position.z + self.tool_offset;
            let factor = if config.fade_end <= config.fade_start {
                1.0
            } else if fade_z.min(fade_z - max_adjustment) >= config.fade_end {
                0.0
            } else if fade_z.max(fade_z - max_adjustment) >= config.fade_start {
                ((config.fade_end + target - fade_z)
                    / (config.fade_end - config.fade_start - adjustment))
                    .clamp(0.0, 1.0)
            } else {
                1.0
            };
            position.z -= factor * adjustment + target;
        }
        unapply_skew(position, self.skew)
    }

    fn active_profile(&self) -> Option<(&BedMeshConfig, &BedMeshProfile)> {
        let config = self.config.bed_mesh.as_ref()?;
        let profile = config.profiles.get(self.active_mesh.as_ref()?)?;
        profile.is_valid().then_some((config, profile))
    }
}

fn apply_skew(mut position: Vec4, factors: SkewFactors) -> Vec4 {
    position.x =
        position.x - position.y * factors.xy - position.z * (factors.xz - factors.xy * factors.yz);
    position.y -= position.z * factors.yz;
    position
}

fn unapply_skew(mut position: Vec4, factors: SkewFactors) -> Vec4 {
    position.x += position.y * factors.xy + position.z * factors.xz;
    position.y += position.z * factors.yz;
    position
}

fn fade_target(config: &BedMeshConfig, profile: &BedMeshProfile) -> f64 {
    if config.fade_end > config.fade_start {
        config.fade_target.unwrap_or_else(|| profile.average())
    } else {
        0.0
    }
}

fn fade_factor(config: &BedMeshConfig, z: f64, tool_offset: f64) -> f64 {
    if config.fade_end <= config.fade_start {
        return 1.0;
    }
    let z = z + tool_offset;
    if z >= config.fade_end {
        0.0
    } else if z >= config.fade_start {
        (config.fade_end - z) / (config.fade_end - config.fade_start)
    } else {
        1.0
    }
}

pub fn calc_skew_factor(ac: f64, bd: f64, ad: f64) -> Option<f64> {
    if ac <= 0.0 || bd <= 0.0 || ad <= 0.0 {
        return None;
    }
    let radicand = 2.0 * ac * ac + 2.0 * bd * bd - 4.0 * ad * ad;
    if radicand < 0.0 {
        return None;
    }
    let side = radicand.sqrt() / 2.0;
    let cosine = (ac * ac - side * side - ad * ad) / (2.0 * side * ad);
    cosine
        .is_finite()
        .then(|| (std::f64::consts::FRAC_PI_2 - cosine.acos()).tan())
}
