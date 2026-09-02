use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use lib_klipper::glam::DVec3;
use lib_klipper::kinematics::{
    CartesianKinematics, CartesianKinematicsKind, DeltaKinematics, DeltesianKinematics, Kinematics,
    PolarKinematics, RotaryDeltaKinematics,
};
use lib_klipper::planner::{
    ExtruderLimits, FirmwareRetractionOptions, MoveChecker, PositionMode, PrinterLimits,
};
use reqwest::StatusCode;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use thiserror::Error;
use url::Url;

pub const CONFIG_SNAPSHOT_SCHEMA_VERSION: u32 = 1;

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Serialize, Deserialize, clap::ArgEnum)]
#[serde(rename_all = "snake_case")]
pub enum SnapshotSelection {
    #[default]
    ConfigurationDefault,
    RuntimeSnapshot,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SnapshotAccuracy {
    Complete,
    Degraded,
}

#[derive(Debug, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SnapshotSourceKind {
    BuiltInDefaults,
    Moonraker,
    MoonrakerCache,
    OfflineConfiguration,
    ConfigurationFile,
    Merged,
}

#[derive(Debug, Clone, Eq, PartialEq, Serialize, Deserialize)]
pub struct SnapshotSource {
    pub kind: SnapshotSourceKind,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub location: Option<String>,
    pub selection: SnapshotSelection,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RuntimeObjects {
    pub toolhead: BTreeMap<String, Value>,
    pub gcode_move: BTreeMap<String, Value>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub extruders: BTreeMap<String, BTreeMap<String, Value>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ExtruderSnapshot {
    pub nozzle_diameter: f64,
    pub filament_diameter: f64,
    pub max_extrude_only_velocity: f64,
    pub max_extrude_only_accel: f64,
    pub max_extrude_only_distance: f64,
    pub instantaneous_corner_velocity: f64,
    pub max_extrude_cross_section: f64,
}

impl Default for ExtruderSnapshot {
    fn default() -> Self {
        let defaults = ExtruderLimits::default();
        Self {
            nozzle_diameter: defaults.nozzle_diameter,
            filament_diameter: defaults.filament_diameter,
            max_extrude_only_velocity: defaults.max_extrude_only_velocity,
            max_extrude_only_accel: defaults.max_extrude_only_accel,
            max_extrude_only_distance: defaults.max_extrude_only_distance,
            instantaneous_corner_velocity: defaults.instantaneous_corner_velocity,
            max_extrude_cross_section: defaults.max_extrude_cross_section,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigSnapshot {
    pub schema_version: u32,
    pub source: SnapshotSource,
    pub retrieved_at_unix_seconds: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub klipper_version: Option<String>,
    pub fingerprint: String,
    pub accuracy: SnapshotAccuracy,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<String>,
    pub limits: PrinterLimits,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub configfile_settings: BTreeMap<String, Value>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub extruders: BTreeMap<String, ExtruderSnapshot>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub runtime: Option<RuntimeObjects>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ConfigSnapshotSummary {
    pub schema_version: u32,
    pub source: SnapshotSource,
    pub retrieved_at_unix_seconds: u64,
    pub klipper_version: Option<String>,
    pub fingerprint: String,
    pub accuracy: SnapshotAccuracy,
    pub warnings: Vec<String>,
}

impl ConfigSnapshot {
    pub fn built_in_defaults() -> Self {
        let mut snapshot = Self {
            schema_version: CONFIG_SNAPSHOT_SCHEMA_VERSION,
            source: SnapshotSource {
                kind: SnapshotSourceKind::BuiltInDefaults,
                location: None,
                selection: SnapshotSelection::ConfigurationDefault,
            },
            retrieved_at_unix_seconds: now_unix_seconds(),
            klipper_version: None,
            fingerprint: String::new(),
            accuracy: SnapshotAccuracy::Degraded,
            warnings: vec![
                "using built-in generic printer limits; estimate is not machine-specific".into(),
            ],
            limits: PrinterLimits::default(),
            configfile_settings: BTreeMap::new(),
            extruders: BTreeMap::new(),
            runtime: None,
        };
        snapshot.refresh_fingerprint();
        snapshot
    }

    pub fn summary(&self) -> ConfigSnapshotSummary {
        ConfigSnapshotSummary {
            schema_version: self.schema_version,
            source: self.source.clone(),
            retrieved_at_unix_seconds: self.retrieved_at_unix_seconds,
            klipper_version: self.klipper_version.clone(),
            fingerprint: self.fingerprint.clone(),
            accuracy: self.accuracy,
            warnings: self.warnings.clone(),
        }
    }

    pub fn validate(&self) -> Result<(), SnapshotError> {
        if self.schema_version != CONFIG_SNAPSHOT_SCHEMA_VERSION {
            return Err(SnapshotError::UnsupportedSchema(self.schema_version));
        }
        let expected = self.calculate_fingerprint();
        if self.fingerprint != expected {
            return Err(SnapshotError::FingerprintMismatch {
                expected,
                actual: self.fingerprint.clone(),
            });
        }
        Ok(())
    }

    pub fn refresh_fingerprint(&mut self) {
        self.fingerprint = self.calculate_fingerprint();
    }

    pub(crate) fn upgrade_legacy_extruders(&mut self) -> Result<(), SnapshotError> {
        if !self.limits.extruders.is_empty() || self.configfile_settings.is_empty() {
            return Ok(());
        }
        let extruders = parse_extruders(&self.configfile_settings)?;
        apply_extruder_limits(&mut self.limits, &extruders);
        self.extruders = extruders;
        if self.source.selection == SnapshotSelection::RuntimeSnapshot {
            if let Some(active) = self
                .runtime
                .as_ref()
                .and_then(|runtime| runtime.toolhead.get("extruder"))
                .and_then(Value::as_str)
            {
                self.limits.initial_extruder = Some(active.into());
            }
        }
        self.warnings
            .push("migrated legacy snapshot to the per-extruder limit model".into());
        self.refresh_fingerprint();
        Ok(())
    }

    fn calculate_fingerprint(&self) -> String {
        #[derive(Serialize)]
        struct FingerprintInput<'a> {
            schema_version: u32,
            selection: SnapshotSelection,
            klipper_version: &'a Option<String>,
            limits: &'a PrinterLimits,
            configfile_settings: &'a BTreeMap<String, Value>,
            runtime: Option<&'a RuntimeObjects>,
        }

        let runtime = (self.source.selection == SnapshotSelection::RuntimeSnapshot)
            .then_some(self.runtime.as_ref())
            .flatten();
        let encoded = serde_json::to_vec(&FingerprintInput {
            schema_version: self.schema_version,
            selection: self.source.selection,
            klipper_version: &self.klipper_version,
            limits: &self.limits,
            configfile_settings: &self.configfile_settings,
            runtime,
        })
        .expect("configuration fingerprint input must serialize");
        format!("sha256:{:x}", Sha256::digest(encoded))
    }
}

#[derive(Error, Debug)]
pub enum SnapshotError {
    #[error("given URL cannot be a base URL")]
    UrlCannotBeBase,
    #[error("invalid URL: {0}")]
    UrlParse(#[from] url::ParseError),
    #[error("Moonraker request failed: {0}")]
    Request(#[from] reqwest::Error),
    #[error("Moonraker did not report required printer object '{0}'")]
    MissingObject(String),
    #[error("Moonraker response is missing required field '{0}'")]
    MissingField(String),
    #[error("Moonraker reports unknown active extruder '{0}'")]
    UnknownActiveExtruder(String),
    #[error("invalid {backend} kinematics geometry: {reason}")]
    InvalidKinematicsGeometry { backend: String, reason: String },
    #[error("unsupported configuration snapshot schema {0}")]
    UnsupportedSchema(u32),
    #[error("configuration snapshot fingerprint mismatch (expected {expected}, found {actual})")]
    FingerprintMismatch { expected: String, actual: String },
    #[error("cached runtime snapshots are stale by definition; reconnect to Moonraker or select configuration-default mode")]
    StaleRuntimeCache,
    #[error("could not read Moonraker cache: {0}")]
    CacheRead(#[source] std::io::Error),
    #[error("could not parse Moonraker cache: {0}")]
    CacheParse(#[source] serde_json::Error),
    #[error("could not write Moonraker cache: {0}")]
    CacheWrite(#[source] std::io::Error),
    #[error(
        "offline configuration path '{0}' must be relative to the declared configuration root"
    )]
    OfflineAbsolutePath(String),
    #[error("could not access offline configuration root '{path}': {source}")]
    OfflineRoot {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("could not read offline configuration file '{path}': {source}")]
    OfflineRead {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("offline configuration path '{path}' resolves outside root '{root}'")]
    OfflineOutsideRoot { path: String, root: String },
    #[error("offline configuration include '{include}' from '{source_file}' does not exist")]
    OfflineMissingInclude {
        include: String,
        source_file: String,
    },
    #[error("offline configuration include '{include}' from '{source_file}' resolves '{first}' and '{second}' to the same file")]
    OfflineAmbiguousInclude {
        include: String,
        source_file: String,
        first: String,
        second: String,
    },
    #[error("recursive offline configuration include of '{0}'")]
    OfflineRecursiveInclude(String),
    #[error("invalid offline configuration at {path}:{line}: {message}")]
    OfflineParse {
        path: String,
        line: usize,
        message: String,
    },
    #[error("offline configuration is missing required option '[{section}] {option}'")]
    OfflineMissingOption { section: String, option: String },
    #[error("invalid offline configuration value '[{section}] {option} = {value}': {message}")]
    OfflineValue {
        section: String,
        option: String,
        value: String,
        message: String,
    },
    #[error("offline extruder sections must be contiguous from '[extruder]'; unexpected '[{0}]'")]
    OfflineExtruderSequence(String),
}

#[derive(Debug, Deserialize)]
struct MoonrakerRoot<T> {
    result: T,
}

#[derive(Debug, Deserialize)]
struct ObjectList {
    objects: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct ObjectQuery {
    status: BTreeMap<String, Value>,
}

#[derive(Debug, Deserialize)]
struct PrinterInfo {
    software_version: Option<String>,
}

pub fn fetch_moonraker_snapshot(
    source_url: &str,
    api_key: Option<&str>,
    selection: SnapshotSelection,
) -> Result<ConfigSnapshot, SnapshotError> {
    let client = reqwest::blocking::Client::new();
    let list_url = endpoint(source_url, &["printer", "objects", "list"])?;
    let info_url = endpoint(source_url, &["printer", "info"])?;
    let query_url = endpoint(source_url, &["printer", "objects", "query"])?;

    let available: BTreeSet<String> = send_get(&client, list_url, api_key)?
        .json::<MoonrakerRoot<ObjectList>>()?
        .result
        .objects
        .into_iter()
        .collect();
    for required in ["configfile", "toolhead", "gcode_move", "extruder"] {
        if !available.contains(required) {
            return Err(SnapshotError::MissingObject(required.into()));
        }
    }

    let objects = requested_objects(&available);

    let request = object_query_request(&client, query_url, api_key, &objects)?;
    let status = client
        .execute(request)?
        .error_for_status()?
        .json::<MoonrakerRoot<ObjectQuery>>()?
        .result
        .status;
    let info = send_get(&client, info_url, api_key)?
        .json::<MoonrakerRoot<PrinterInfo>>()?
        .result;

    snapshot_from_status(source_url, selection, info.software_version, status)
}

fn requested_objects(available: &BTreeSet<String>) -> BTreeMap<String, Option<Vec<String>>> {
    let mut objects = BTreeMap::<String, Option<Vec<String>>>::new();
    objects.insert("configfile".into(), Some(vec!["settings".into()]));
    objects.insert("toolhead".into(), None);
    objects.insert("gcode_move".into(), None);
    for name in available.iter().filter(|name| is_extruder_object(name)) {
        objects.insert(name.clone(), None);
    }
    for optional in ["firmware_retraction", "gcode_arcs"] {
        if available.contains(optional) {
            objects.insert(optional.into(), None);
        }
    }
    objects
}

fn object_query_request(
    client: &reqwest::blocking::Client,
    query_url: Url,
    api_key: Option<&str>,
    objects: &BTreeMap<String, Option<Vec<String>>>,
) -> Result<reqwest::blocking::Request, SnapshotError> {
    let mut request = client.post(query_url).json(&json!({ "objects": objects }));
    if let Some(api_key) = api_key {
        request = request.header("X-Api-Key", api_key);
    }
    Ok(request.build()?)
}

fn send_get(
    client: &reqwest::blocking::Client,
    url: Url,
    api_key: Option<&str>,
) -> Result<reqwest::blocking::Response, reqwest::Error> {
    let mut request = client.get(url);
    if let Some(api_key) = api_key {
        request = request.header("X-Api-Key", api_key);
    }
    request.send()?.error_for_status()
}

fn endpoint(source_url: &str, segments: &[&str]) -> Result<Url, SnapshotError> {
    let mut url = Url::parse(source_url)?;
    url.set_query(None);
    let mut path = url
        .path_segments_mut()
        .map_err(|_| SnapshotError::UrlCannotBeBase)?;
    path.pop_if_empty();
    path.extend(segments);
    drop(path);
    Ok(url)
}

fn snapshot_from_status(
    source_url: &str,
    selection: SnapshotSelection,
    klipper_version: Option<String>,
    mut status: BTreeMap<String, Value>,
) -> Result<ConfigSnapshot, SnapshotError> {
    let configfile = take_object(&mut status, "configfile")?;
    let settings = object_field(&configfile, "settings")?
        .as_object()
        .ok_or_else(|| SnapshotError::MissingField("configfile.settings".into()))?;
    let configfile_settings: BTreeMap<String, Value> = settings.clone().into_iter().collect();

    let toolhead = take_object(&mut status, "toolhead")?;
    let gcode_move = take_object(&mut status, "gcode_move")?;
    let mut extruder_runtime = BTreeMap::new();
    for name in status
        .keys()
        .filter(|name| is_extruder_object(name))
        .cloned()
        .collect::<Vec<_>>()
    {
        extruder_runtime.insert(name.clone(), take_object(&mut status, &name)?);
    }

    let extruders = parse_extruders(&configfile_settings)?;
    let mut limits = limits_from_settings(&configfile_settings, &extruders)?;
    if selection == SnapshotSelection::RuntimeSnapshot {
        apply_runtime_limits(&mut limits, &toolhead, &gcode_move)?;
    }
    limits.recalculate();

    let mut snapshot = ConfigSnapshot {
        schema_version: CONFIG_SNAPSHOT_SCHEMA_VERSION,
        source: SnapshotSource {
            kind: SnapshotSourceKind::Moonraker,
            location: Some(redact_url(source_url)),
            selection,
        },
        retrieved_at_unix_seconds: now_unix_seconds(),
        klipper_version,
        fingerprint: String::new(),
        accuracy: SnapshotAccuracy::Complete,
        warnings: Vec::new(),
        limits,
        configfile_settings,
        extruders,
        runtime: Some(RuntimeObjects {
            toolhead,
            gcode_move,
            extruders: extruder_runtime,
        }),
    };
    apply_kinematics_classification(&mut snapshot);
    snapshot.refresh_fingerprint();
    Ok(snapshot)
}

pub(crate) fn apply_kinematics_classification(snapshot: &mut ConfigSnapshot) {
    if let Some((backend, reason)) = snapshot.limits.kinematics.unsupported_details() {
        snapshot.accuracy = SnapshotAccuracy::Degraded;
        let warning = format!("configured Klipper kinematics '{backend}' is unsupported: {reason}");
        if !snapshot.warnings.contains(&warning) {
            snapshot.warnings.push(warning);
        }
    }
}

fn take_object(
    status: &mut BTreeMap<String, Value>,
    name: &str,
) -> Result<BTreeMap<String, Value>, SnapshotError> {
    status
        .remove(name)
        .and_then(|value| value.as_object().cloned())
        .map(|map| map.into_iter().collect())
        .ok_or_else(|| SnapshotError::MissingObject(name.into()))
}

fn object_field<'a>(
    object: &'a BTreeMap<String, Value>,
    field: &str,
) -> Result<&'a Value, SnapshotError> {
    object
        .get(field)
        .ok_or_else(|| SnapshotError::MissingField(field.into()))
}

fn number(object: &serde_json::Map<String, Value>, path: &str) -> Result<f64, SnapshotError> {
    object
        .get(path.rsplit('.').next().unwrap_or(path))
        .and_then(Value::as_f64)
        .ok_or_else(|| SnapshotError::MissingField(path.into()))
}

fn optional_number(object: &serde_json::Map<String, Value>, field: &str) -> Option<f64> {
    object.get(field).and_then(Value::as_f64)
}

fn settings_object<'a>(
    settings: &'a BTreeMap<String, Value>,
    name: &str,
) -> Result<&'a serde_json::Map<String, Value>, SnapshotError> {
    settings
        .get(name)
        .and_then(Value::as_object)
        .ok_or_else(|| SnapshotError::MissingField(format!("configfile.settings.{name}")))
}

fn inherited_numbers<const N: usize>(
    settings: &BTreeMap<String, Value>,
    sections: [&str; N],
    field: &str,
    defaults: [f64; N],
) -> Result<[f64; N], SnapshotError> {
    let mut values = defaults;
    for (index, section) in sections.iter().enumerate() {
        let object = settings_object(settings, section)?;
        if let Some(value) = optional_number(object, field) {
            values[index] = value;
        } else if index == 0 {
            return Err(SnapshotError::MissingField(format!(
                "configfile.settings.{section}.{field}"
            )));
        } else {
            values[index] = values[0];
        }
    }
    Ok(values)
}

fn default_numbers<const N: usize>(
    settings: &BTreeMap<String, Value>,
    sections: [&str; N],
    field: &str,
    defaults: [f64; N],
) -> Result<[f64; N], SnapshotError> {
    let mut values = defaults;
    for (index, section) in sections.iter().enumerate() {
        if let Some(value) = optional_number(settings_object(settings, section)?, field) {
            values[index] = value;
        }
    }
    Ok(values)
}

fn linear_step_distance(
    object: &serde_json::Map<String, Value>,
    path: &str,
) -> Result<f64, SnapshotError> {
    if let Some(distance) = optional_number(object, "step_distance") {
        return Ok(distance);
    }
    let rotation_distance = number(object, &format!("{path}.rotation_distance"))?;
    let microsteps = number(object, &format!("{path}.microsteps"))?;
    let full_steps = optional_number(object, "full_steps_per_rotation").unwrap_or(200.0);
    let gearing = object
        .get("gear_ratio")
        .and_then(Value::as_array)
        .and_then(|pairs| {
            pairs.iter().try_fold(1.0, |ratio, pair| {
                let pair = pair.as_array()?;
                Some(ratio * pair.first()?.as_f64()? / pair.get(1)?.as_f64()?)
            })
        })
        .unwrap_or(1.0);
    let distance = rotation_distance / (microsteps * full_steps * gearing);
    if distance.is_finite() && distance > 0.0 {
        Ok(distance)
    } else {
        Err(SnapshotError::MissingField(format!("{path}.step_distance")))
    }
}

fn offline_step_distance(
    options: &BTreeMap<String, String>,
    section: &str,
) -> Result<f64, SnapshotError> {
    let rotation = required_float(options, section, "rotation_distance", |value| value > 0.0)?;
    let microsteps = required_float(options, section, "microsteps", |value| value >= 1.0)?;
    let full_steps = optional_float(
        options,
        section,
        "full_steps_per_rotation",
        200.0,
        |value| value >= 1.0 && value % 4.0 == 0.0,
    )?;
    let gearing = options.get("gear_ratio").map_or(Ok(1.0), |value| {
        value.split(',').try_fold(1.0, |ratio, stage| {
            let mut parts = stage.split(':').map(str::trim);
            let first = parts.next().and_then(|part| part.parse::<f64>().ok());
            let second = parts.next().and_then(|part| part.parse::<f64>().ok());
            match (first, second, parts.next()) {
                (Some(first), Some(second), None) if first > 0.0 && second > 0.0 => {
                    Ok(ratio * first / second)
                }
                _ => Err(SnapshotError::OfflineValue {
                    section: section.into(),
                    option: "gear_ratio".into(),
                    value: value.into(),
                    message: "expected one or more positive numerator:denominator pairs".into(),
                }),
            }
        })
    })?;
    Ok(rotation / (microsteps * full_steps * gearing))
}

fn limits_from_settings(
    settings: &BTreeMap<String, Value>,
    extruders: &BTreeMap<String, ExtruderSnapshot>,
) -> Result<PrinterLimits, SnapshotError> {
    let printer = settings
        .get("printer")
        .and_then(Value::as_object)
        .ok_or_else(|| SnapshotError::MissingField("configfile.settings.printer".into()))?;
    let primary_extruder = extruders
        .get("extruder")
        .ok_or_else(|| SnapshotError::MissingField("configfile.settings.extruder".into()))?;

    let mut limits = PrinterLimits::default();
    limits.set_max_velocity(number(printer, "configfile.settings.printer.max_velocity")?);
    limits.set_max_acceleration(number(printer, "configfile.settings.printer.max_accel")?);
    if let Some(value) = optional_number(printer, "minimum_cruise_ratio") {
        limits.set_minimum_cruise_ratio(value);
    } else if let Some(value) = optional_number(printer, "max_accel_to_decel") {
        limits.set_max_accel_to_decel(value);
    }
    limits.set_square_corner_velocity(number(
        printer,
        "configfile.settings.printer.square_corner_velocity",
    )?);
    limits.set_instant_corner_velocity(primary_extruder.instantaneous_corner_velocity);
    apply_extruder_limits(&mut limits, extruders);
    limits.kinematics = kinematics_from_settings(settings, printer)?;

    for (axis, prefix) in [(DVec3::X, "x"), (DVec3::Y, "y"), (DVec3::Z, "z")] {
        let velocity = optional_number(printer, &format!("max_{prefix}_velocity"));
        let accel = optional_number(printer, &format!("max_{prefix}_accel"));
        if let (Some(max_velocity), Some(max_accel)) = (velocity, accel) {
            limits.move_checkers.push(MoveChecker::AxisLimiter {
                axis,
                max_velocity,
                max_accel,
            });
        }
    }

    limits.firmware_retraction = settings
        .get("firmware_retraction")
        .and_then(Value::as_object)
        .map(|object| {
            Ok::<_, SnapshotError>(FirmwareRetractionOptions {
                retract_length: number(object, "firmware_retraction.retract_length")?,
                unretract_extra_length: number(
                    object,
                    "firmware_retraction.unretract_extra_length",
                )?,
                unretract_speed: number(object, "firmware_retraction.unretract_speed")?,
                retract_speed: number(object, "firmware_retraction.retract_speed")?,
                lift_z: optional_number(object, "lift_z").unwrap_or(0.0),
            })
        })
        .transpose()?;
    limits.mm_per_arc_segment = settings
        .get("gcode_arcs")
        .and_then(Value::as_object)
        .and_then(|object| optional_number(object, "resolution"));
    Ok(limits)
}

fn apply_extruder_limits(
    limits: &mut PrinterLimits,
    extruders: &BTreeMap<String, ExtruderSnapshot>,
) {
    limits.extruders = extruders
        .iter()
        .map(|(name, extruder)| {
            (
                name.clone(),
                ExtruderLimits {
                    nozzle_diameter: extruder.nozzle_diameter,
                    filament_diameter: extruder.filament_diameter,
                    max_extrude_only_velocity: extruder.max_extrude_only_velocity,
                    max_extrude_only_accel: extruder.max_extrude_only_accel,
                    max_extrude_only_distance: extruder.max_extrude_only_distance,
                    instantaneous_corner_velocity: extruder.instantaneous_corner_velocity,
                    max_extrude_cross_section: extruder.max_extrude_cross_section,
                },
            )
        })
        .collect();
}

fn kinematics_from_settings(
    settings: &BTreeMap<String, Value>,
    printer: &serde_json::Map<String, Value>,
) -> Result<Kinematics, SnapshotError> {
    let backend = match printer.get("kinematics").and_then(Value::as_str) {
        Some(backend) => backend,
        None => return Ok(Kinematics::Unconfigured),
    };
    if settings.contains_key("dual_carriage") {
        return Ok(Kinematics::unsupported(
            backend,
            "dual-carriage active state is not modeled",
        ));
    }
    let kind = match backend {
        "cartesian" => CartesianKinematicsKind::Cartesian,
        "corexy" => CartesianKinematicsKind::Corexy,
        "corexz" => CartesianKinematicsKind::Corexz,
        "hybrid_corexy" => CartesianKinematicsKind::HybridCorexy,
        "hybrid_corexz" => CartesianKinematicsKind::HybridCorexz,
        "generic_cartesian" => {
            return Ok(Kinematics::unsupported(
                backend,
                "generic Cartesian carriage expressions are not modeled",
            ))
        }
        "delta" => {
            let sections = ["stepper_a", "stepper_b", "stepper_c"];
            let arms = inherited_numbers(settings, sections, "arm_length", [0.0; 3])?;
            let endstops = inherited_numbers(settings, sections, "position_endstop", [0.0; 3])?;
            let angles = default_numbers(settings, sections, "angle", [210.0, 330.0, 90.0])?;
            let mut step_distances = [0.0; 3];
            for (index, section) in sections.iter().enumerate() {
                step_distances[index] = linear_step_distance(
                    settings_object(settings, section)?,
                    &format!("configfile.settings.{section}"),
                )?;
            }
            let radius = number(printer, "configfile.settings.printer.delta_radius")?;
            return validated_kinematics(Kinematics::Delta {
                config: DeltaKinematics {
                    max_velocity: number(printer, "configfile.settings.printer.max_velocity")?,
                    max_accel: number(printer, "configfile.settings.printer.max_accel")?,
                    max_z_velocity: number(printer, "configfile.settings.printer.max_z_velocity")?,
                    max_z_accel: number(printer, "configfile.settings.printer.max_z_accel")?,
                    minimum_z: optional_number(printer, "minimum_z_position").unwrap_or(0.0),
                    radius,
                    print_radius: optional_number(printer, "print_radius").unwrap_or(radius),
                    arm_lengths: arms,
                    tower_angles: angles,
                    position_endstops: endstops,
                    step_distances,
                },
            });
        }
        "polar" => {
            let arm = settings_object(settings, "stepper_arm")?;
            let z = settings_object(settings, "stepper_z")?;
            return validated_kinematics(Kinematics::Polar {
                config: PolarKinematics {
                    max_velocity: number(printer, "configfile.settings.printer.max_velocity")?,
                    max_accel: number(printer, "configfile.settings.printer.max_accel")?,
                    max_z_velocity: number(printer, "configfile.settings.printer.max_z_velocity")?,
                    max_z_accel: number(printer, "configfile.settings.printer.max_z_accel")?,
                    max_angular_velocity: optional_number(printer, "max_angular_velocity")
                        .unwrap_or(0.0),
                    maximum_radius: number(arm, "configfile.settings.stepper_arm.position_max")?,
                    minimum_z: optional_number(z, "position_min").unwrap_or(0.0),
                    maximum_z: number(z, "configfile.settings.stepper_z.position_max")?,
                },
            });
        }
        "deltesian" => {
            let sections = ["stepper_left", "stepper_right"];
            let arm_x = inherited_numbers(settings, sections, "arm_x_length", [0.0; 2])?;
            let arms = inherited_numbers(settings, sections, "arm_length", [0.0; 2])?;
            let endstops = inherited_numbers(settings, sections, "position_endstop", [0.0; 2])?;
            let y = settings_object(settings, "stepper_y")?;
            return validated_kinematics(Kinematics::Deltesian {
                config: DeltesianKinematics {
                    max_velocity: number(printer, "configfile.settings.printer.max_velocity")?,
                    max_accel: number(printer, "configfile.settings.printer.max_accel")?,
                    max_z_velocity: number(printer, "configfile.settings.printer.max_z_velocity")?,
                    max_z_accel: number(printer, "configfile.settings.printer.max_z_accel")?,
                    minimum_z: optional_number(printer, "minimum_z_position").unwrap_or(0.0),
                    minimum_angle: optional_number(printer, "min_angle").unwrap_or(5.0),
                    print_width: optional_number(printer, "print_width"),
                    slow_ratio: optional_number(printer, "slow_ratio").unwrap_or(3.0),
                    arm_x_lengths: arm_x,
                    arm_lengths: arms,
                    position_endstops: endstops,
                    y_range: [
                        optional_number(y, "position_min").unwrap_or(0.0),
                        number(y, "configfile.settings.stepper_y.position_max")?,
                    ],
                },
            });
        }
        "rotary_delta" => {
            let sections = ["stepper_a", "stepper_b", "stepper_c"];
            return validated_kinematics(Kinematics::RotaryDelta {
                config: RotaryDeltaKinematics {
                    max_z_velocity: number(printer, "configfile.settings.printer.max_z_velocity")?,
                    minimum_z: optional_number(printer, "minimum_z_position").unwrap_or(0.0),
                    shoulder_radius: number(
                        printer,
                        "configfile.settings.printer.shoulder_radius",
                    )?,
                    shoulder_height: number(
                        printer,
                        "configfile.settings.printer.shoulder_height",
                    )?,
                    upper_arm_lengths: inherited_numbers(
                        settings,
                        sections,
                        "upper_arm_length",
                        [0.0; 3],
                    )?,
                    lower_arm_lengths: inherited_numbers(
                        settings,
                        sections,
                        "lower_arm_length",
                        [0.0; 3],
                    )?,
                    tower_angles: default_numbers(
                        settings,
                        sections,
                        "angle",
                        [30.0, 150.0, 270.0],
                    )?,
                    position_endstops: inherited_numbers(
                        settings,
                        sections,
                        "position_endstop",
                        [0.0; 3],
                    )?,
                },
            });
        }
        _ => return Ok(Kinematics::unsupported(backend, "backend is not modeled")),
    };

    let mut axis_minimum = DVec3::ZERO;
    let mut axis_maximum = DVec3::ZERO;
    for (axis, name) in ["stepper_x", "stepper_y", "stepper_z"].iter().enumerate() {
        let object = settings
            .get(*name)
            .and_then(Value::as_object)
            .ok_or_else(|| SnapshotError::MissingField(format!("configfile.settings.{name}")))?;
        axis_minimum.as_mut()[axis] =
            number(object, &format!("configfile.settings.{name}.position_min"))?;
        axis_maximum.as_mut()[axis] =
            number(object, &format!("configfile.settings.{name}.position_max"))?;
    }
    Ok(Kinematics::CartesianFamily {
        config: CartesianKinematics {
            kind,
            axis_minimum,
            axis_maximum,
            max_z_velocity: number(printer, "configfile.settings.printer.max_z_velocity")?,
            max_z_accel: number(printer, "configfile.settings.printer.max_z_accel")?,
        },
    })
}

fn invalid_geometry(backend: &str, reason: impl Into<String>) -> SnapshotError {
    SnapshotError::InvalidKinematicsGeometry {
        backend: backend.into(),
        reason: reason.into(),
    }
}

fn validated_kinematics(kinematics: Kinematics) -> Result<Kinematics, SnapshotError> {
    match &kinematics {
        Kinematics::Delta { config } => {
            if config.radius <= 0.0 || config.print_radius <= 0.0 {
                return Err(invalid_geometry("delta", "radii must be positive"));
            }
            if config.arm_lengths.iter().any(|arm| *arm <= config.radius) {
                return Err(invalid_geometry(
                    "delta",
                    "every arm_length must exceed delta_radius",
                ));
            }
            if config
                .step_distances
                .iter()
                .any(|distance| *distance <= 0.0)
            {
                return Err(invalid_geometry("delta", "step distances must be positive"));
            }
            let min_arm = config
                .arm_lengths
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min);
            let half_step = config
                .step_distances
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min)
                * 0.5;
            let ratio = 4.0 * 3.0;
            if min_arm.powi(2) / (ratio * ratio + 1.0) <= half_step.powi(2) {
                return Err(invalid_geometry(
                    "delta",
                    "step distance is too large for the configured arm geometry",
                ));
            }
            let max_z = config
                .position_endstops
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min);
            if config.minimum_z > max_z {
                return Err(invalid_geometry(
                    "delta",
                    "minimum_z_position exceeds maximum Z",
                ));
            }
        }
        Kinematics::Polar { config }
            if config.maximum_radius <= 0.0 || config.minimum_z >= config.maximum_z =>
        {
            return Err(invalid_geometry(
                "polar",
                "arm radius and Z range must be non-empty",
            ));
        }
        Kinematics::Deltesian { config } => {
            if config
                .arm_lengths
                .iter()
                .zip(config.arm_x_lengths)
                .any(|(arm, arm_x)| *arm <= arm_x || arm_x <= 0.0)
            {
                return Err(invalid_geometry(
                    "deltesian",
                    "each arm_length must exceed arm_x_length",
                ));
            }
            if !(0.0..=90.0).contains(&config.minimum_angle) || config.slow_ratio < 0.0 {
                return Err(invalid_geometry("deltesian", "invalid angle or slow ratio"));
            }
            let cosine = config.minimum_angle.to_radians().cos();
            let x_min = (-config.arm_x_lengths[0])
                .max(-(cosine * config.arm_lengths[1] - config.arm_x_lengths[1]))
                .ceil();
            let x_max = config.arm_x_lengths[1]
                .min(cosine * config.arm_lengths[0] - config.arm_x_lengths[0])
                .floor();
            let max_width = (x_max - x_min).min(x_max * 2.0).min(-x_min * 2.0);
            if config
                .print_width
                .is_some_and(|width| width < 0.0 || width > max_width)
            {
                return Err(invalid_geometry(
                    "deltesian",
                    "print_width exceeds the kinematic X range",
                ));
            }
            if config.y_range[0] >= config.y_range[1] {
                return Err(invalid_geometry("deltesian", "Y range must be non-empty"));
            }
            let x_limits = match config.print_width {
                Some(width) if width != 0.0 => [-width * 0.5, width * 0.5],
                _ => [x_min, x_max],
            };
            let abs_endstops = std::array::from_fn::<_, 2, _>(|index| {
                config.position_endstops[index]
                    + (config.arm_lengths[index].powi(2) - config.arm_x_lengths[index].powi(2))
                        .sqrt()
            });
            let pillars_z_max = |x: f64| {
                (0..2)
                    .map(|index| {
                        let horizontal = if index == 0 {
                            config.arm_x_lengths[index] + x
                        } else {
                            config.arm_x_lengths[index] - x
                        };
                        abs_endstops[index]
                            - (config.arm_lengths[index].powi(2) - horizontal.powi(2)).sqrt()
                    })
                    .fold(f64::INFINITY, f64::min)
            };
            let max_z = pillars_z_max(x_limits[0]).min(pillars_z_max(x_limits[1]));
            if !max_z.is_finite() || config.minimum_z > max_z {
                return Err(invalid_geometry(
                    "deltesian",
                    "minimum_z_position exceeds the geometric maximum Z",
                ));
            }
        }
        Kinematics::RotaryDelta { config } => {
            if config.shoulder_radius <= 0.0
                || config.shoulder_height <= 0.0
                || config.upper_arm_lengths.iter().any(|arm| *arm <= 0.0)
                || config.lower_arm_lengths.iter().any(|arm| *arm <= 0.0)
            {
                return Err(invalid_geometry(
                    "rotary_delta",
                    "radii, heights, and arm lengths are inconsistent",
                ));
            }
            if config.minimum_z
                > config
                    .position_endstops
                    .iter()
                    .copied()
                    .fold(f64::INFINITY, f64::min)
            {
                return Err(invalid_geometry(
                    "rotary_delta",
                    "minimum_z_position exceeds maximum Z",
                ));
            }
            for index in 0..3 {
                let dx = -config.shoulder_radius;
                let dy = config.position_endstops[index] - config.shoulder_height;
                if dy == 0.0 {
                    return Err(invalid_geometry(
                        "rotary_delta",
                        "an endstop lies in the shoulder plane",
                    ));
                }
                let upper2 = config.upper_arm_lengths[index].powi(2);
                let lower2 = config.lower_arm_lengths[index].powi(2);
                let c1 = 0.5 / dy * (dx * dx + dy * dy + upper2 - lower2);
                let c2 = dx / dy;
                if (c2 * c2 + 1.0) * upper2 - c1 * c1 < 0.0 {
                    return Err(invalid_geometry(
                        "rotary_delta",
                        "endstop geometry has no real arm solution",
                    ));
                }
            }
        }
        _ => {}
    }
    Ok(kinematics)
}

fn parse_extruders(
    settings: &BTreeMap<String, Value>,
) -> Result<BTreeMap<String, ExtruderSnapshot>, SnapshotError> {
    settings
        .iter()
        .filter(|(name, _)| is_extruder_object(name))
        .map(|(name, value)| {
            let object = value.as_object().ok_or_else(|| {
                SnapshotError::MissingField(format!("configfile.settings.{name}"))
            })?;
            Ok((
                name.clone(),
                ExtruderSnapshot {
                    nozzle_diameter: number(
                        object,
                        &format!("configfile.settings.{name}.nozzle_diameter"),
                    )?,
                    filament_diameter: number(
                        object,
                        &format!("configfile.settings.{name}.filament_diameter"),
                    )?,
                    max_extrude_only_velocity: number(
                        object,
                        &format!("configfile.settings.{name}.max_extrude_only_velocity"),
                    )?,
                    max_extrude_only_accel: number(
                        object,
                        &format!("configfile.settings.{name}.max_extrude_only_accel"),
                    )?,
                    max_extrude_only_distance: number(
                        object,
                        &format!("configfile.settings.{name}.max_extrude_only_distance"),
                    )?,
                    instantaneous_corner_velocity: number(
                        object,
                        &format!("configfile.settings.{name}.instantaneous_corner_velocity"),
                    )?,
                    max_extrude_cross_section: number(
                        object,
                        &format!("configfile.settings.{name}.max_extrude_cross_section"),
                    )?,
                },
            ))
        })
        .collect()
}

fn apply_runtime_limits(
    limits: &mut PrinterLimits,
    toolhead: &BTreeMap<String, Value>,
    gcode_move: &BTreeMap<String, Value>,
) -> Result<(), SnapshotError> {
    let toolhead: serde_json::Map<String, Value> = toolhead.clone().into_iter().collect();
    limits.set_max_velocity(number(&toolhead, "toolhead.max_velocity")?);
    limits.set_max_acceleration(number(&toolhead, "toolhead.max_accel")?);
    limits.set_square_corner_velocity(number(&toolhead, "toolhead.square_corner_velocity")?);
    limits.set_minimum_cruise_ratio(number(&toolhead, "toolhead.minimum_cruise_ratio")?);
    let active_extruder = toolhead
        .get("extruder")
        .and_then(Value::as_str)
        .ok_or_else(|| SnapshotError::MissingField("toolhead.extruder".into()))?;
    if !limits.extruders.contains_key(active_extruder) {
        return Err(SnapshotError::UnknownActiveExtruder(active_extruder.into()));
    }
    limits.initial_extruder = Some(active_extruder.into());
    limits.initial_coordinate_mode = bool_mode(gcode_move, "absolute_coordinates")?;
    limits.initial_extrusion_mode = bool_mode(gcode_move, "absolute_extrude")?;
    Ok(())
}

fn bool_mode(object: &BTreeMap<String, Value>, field: &str) -> Result<PositionMode, SnapshotError> {
    object
        .get(field)
        .and_then(Value::as_bool)
        .map(|absolute| {
            if absolute {
                PositionMode::Absolute
            } else {
                PositionMode::Relative
            }
        })
        .ok_or_else(|| SnapshotError::MissingField(format!("gcode_move.{field}")))
}

fn is_extruder_object(name: &str) -> bool {
    name == "extruder"
        || name
            .strip_prefix("extruder")
            .is_some_and(|suffix| !suffix.is_empty() && suffix.chars().all(|c| c.is_ascii_digit()))
}

fn redact_url(source_url: &str) -> String {
    Url::parse(source_url)
        .map(|mut url| {
            let _ = url.set_username("");
            let _ = url.set_password(None);
            url.set_query(None);
            url.to_string()
        })
        .unwrap_or_else(|_| source_url.into())
}

fn now_unix_seconds() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

type OfflineSections = BTreeMap<String, BTreeMap<String, String>>;

/// Resolve a Klipper configuration using the include and SAVE_CONFIG ordering in
/// `klippy/configfile.py` at Klipper f0892d82b0f1c1228454f09eb508eddde2250f4b.
/// Only settings consumed by the estimator are materialized; other sections are
/// retained as an explicit warning instead of being assigned guessed defaults.
pub fn load_offline_snapshot(
    config_root: &str,
    root_filename: &str,
) -> Result<ConfigSnapshot, SnapshotError> {
    let root_relative = Path::new(root_filename);
    if root_relative.is_absolute() {
        return Err(SnapshotError::OfflineAbsolutePath(root_filename.into()));
    }
    let root = fs::canonicalize(config_root).map_err(|source| SnapshotError::OfflineRoot {
        path: config_root.into(),
        source,
    })?;
    let config_path = confined_canonicalize(&root, &root.join(root_relative))?;
    let mut sections = OfflineSections::new();
    let mut visited = BTreeSet::new();
    parse_offline_file(&root, &config_path, &mut sections, &mut visited, true)?;

    let settings = resolve_offline_settings(&sections)?;
    let extruders = parse_extruders(&settings)?;
    let mut limits = limits_from_settings(&settings, &extruders)?;
    limits.recalculate();

    let supported = [
        "printer",
        "stepper_x",
        "stepper_y",
        "stepper_z",
        "stepper_a",
        "stepper_b",
        "stepper_c",
        "stepper_bed",
        "stepper_arm",
        "stepper_left",
        "stepper_right",
        "firmware_retraction",
        "gcode_arcs",
    ];
    let unsupported: Vec<_> = sections
        .keys()
        .filter(|name| !supported.contains(&name.as_str()) && !is_extruder_object(name))
        .cloned()
        .collect();
    let mut warnings = Vec::new();
    if !unsupported.is_empty() {
        warnings.push(format!(
            "offline configuration sections not modeled by the estimator: {}",
            unsupported.join(", ")
        ));
    }

    let relative_location = config_path
        .strip_prefix(&root)
        .unwrap_or(&config_path)
        .to_string_lossy();
    let mut snapshot = ConfigSnapshot {
        schema_version: CONFIG_SNAPSHOT_SCHEMA_VERSION,
        source: SnapshotSource {
            kind: SnapshotSourceKind::OfflineConfiguration,
            location: Some(format!("{}:{}", root.display(), relative_location)),
            selection: SnapshotSelection::ConfigurationDefault,
        },
        retrieved_at_unix_seconds: now_unix_seconds(),
        klipper_version: None,
        fingerprint: String::new(),
        accuracy: SnapshotAccuracy::Complete,
        warnings,
        limits,
        configfile_settings: settings,
        extruders,
        runtime: None,
    };
    apply_kinematics_classification(&mut snapshot);
    snapshot.refresh_fingerprint();
    Ok(snapshot)
}

fn confined_canonicalize(root: &Path, path: &Path) -> Result<PathBuf, SnapshotError> {
    let canonical = fs::canonicalize(path).map_err(|source| SnapshotError::OfflineRead {
        path: path.display().to_string(),
        source,
    })?;
    if !canonical.starts_with(root) {
        return Err(SnapshotError::OfflineOutsideRoot {
            path: canonical.display().to_string(),
            root: root.display().to_string(),
        });
    }
    Ok(canonical)
}

fn parse_offline_file(
    root: &Path,
    path: &Path,
    sections: &mut OfflineSections,
    visited: &mut BTreeSet<PathBuf>,
    is_root: bool,
) -> Result<(), SnapshotError> {
    let path = confined_canonicalize(root, path)?;
    if !visited.insert(path.clone()) {
        return Err(SnapshotError::OfflineRecursiveInclude(
            path.display().to_string(),
        ));
    }
    let data = fs::read_to_string(&path).map_err(|source| SnapshotError::OfflineRead {
        path: path.display().to_string(),
        source,
    })?;
    let (regular, autosave) = if is_root {
        split_save_config(&path, &data)?
    } else {
        (data.replace("\r\n", "\n"), None)
    };
    parse_offline_text(root, &path, &regular, sections, visited)?;
    if let Some(autosave) = autosave {
        let mut saved = OfflineSections::new();
        parse_ini_chunk(&path, 1, &autosave, &mut saved)?;
        for (section, options) in saved {
            let target = sections.entry(section).or_default();
            for (option, value) in options {
                target.entry(option).or_insert(value);
            }
        }
    }
    visited.remove(&path);
    Ok(())
}

fn split_save_config(path: &Path, data: &str) -> Result<(String, Option<String>), SnapshotError> {
    const HEADER: &str = "#*# <---------------------- SAVE_CONFIG ---------------------->";
    const NOTICE: &str = "#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.";
    let normalized = data.replace("\r\n", "\n");
    let mut lines: Vec<&str> = normalized.split('\n').collect();
    let Some(index) = lines.iter().position(|line| *line == HEADER) else {
        if lines.iter().any(|line| line.starts_with("#*# ")) {
            return offline_parse_error(path, 1, "corrupted SAVE_CONFIG marker");
        }
        return Ok((normalized, None));
    };
    if lines.get(index + 1) != Some(&NOTICE) || lines.get(index + 2) != Some(&"#*#") {
        return offline_parse_error(path, index + 1, "corrupted SAVE_CONFIG header");
    }
    if lines[..index].iter().any(|line| line.starts_with("#*# ")) {
        return offline_parse_error(path, index + 1, "duplicate or misplaced SAVE_CONFIG data");
    }
    let autosave_lines = lines.split_off(index + 3);
    lines.truncate(index);
    let mut decoded = Vec::with_capacity(autosave_lines.len());
    for (offset, line) in autosave_lines.into_iter().enumerate() {
        if line.is_empty() || line == "#*#" {
            decoded.push("");
        } else if let Some(value) = line.strip_prefix("#*# ") {
            decoded.push(value);
        } else {
            return offline_parse_error(
                path,
                index + 4 + offset,
                "content after SAVE_CONFIG header is not auto-generated data",
            );
        }
    }
    Ok((lines.join("\n"), Some(decoded.join("\n"))))
}

fn parse_offline_text(
    root: &Path,
    path: &Path,
    data: &str,
    sections: &mut OfflineSections,
    visited: &mut BTreeSet<PathBuf>,
) -> Result<(), SnapshotError> {
    let mut chunk = Vec::new();
    let mut chunk_start = 1;
    for (index, original) in data.lines().enumerate() {
        let line = strip_ini_comment(original).trim();
        if let Some(header) = section_header(line) {
            if let Some(include) = header.strip_prefix("include ") {
                parse_ini_chunk(path, chunk_start, &chunk.join("\n"), sections)?;
                chunk.clear();
                for include_path in resolve_include(root, path, include.trim())? {
                    parse_offline_file(root, &include_path, sections, visited, false)?;
                }
                chunk_start = index + 2;
                continue;
            } else if header == "include" {
                return offline_parse_error(path, index + 1, "empty include specification");
            }
        }
        chunk.push(original);
    }
    parse_ini_chunk(path, chunk_start, &chunk.join("\n"), sections)
}

fn strip_ini_comment(line: &str) -> &str {
    let hash = line.find('#').unwrap_or(line.len());
    let semicolon = line
        .char_indices()
        .find(|(index, character)| {
            *character == ';'
                && (*index == 0
                    || line[..*index]
                        .chars()
                        .next_back()
                        .is_some_and(char::is_whitespace))
        })
        .map(|(index, _)| index)
        .unwrap_or(line.len());
    let comment = hash.min(semicolon);
    &line[..comment]
}

fn section_header(line: &str) -> Option<&str> {
    line.strip_prefix('[')?.strip_suffix(']').map(str::trim)
}

fn parse_ini_chunk(
    path: &Path,
    start_line: usize,
    data: &str,
    sections: &mut OfflineSections,
) -> Result<(), SnapshotError> {
    let mut current_section: Option<String> = None;
    let mut last_option: Option<String> = None;
    for (offset, original) in data.lines().enumerate() {
        let line_number = start_line + offset;
        let uncommented = strip_ini_comment(original);
        let trimmed = uncommented.trim();
        if trimmed.is_empty() {
            continue;
        }
        if let Some(header) = section_header(trimmed) {
            if header.is_empty() || header.starts_with("include ") {
                return offline_parse_error(path, line_number, "invalid section header");
            }
            let section = header.to_ascii_lowercase();
            sections.entry(section.clone()).or_default();
            current_section = Some(section);
            last_option = None;
            continue;
        }
        if original.starts_with(char::is_whitespace) {
            let section = current_section
                .as_ref()
                .ok_or_else(|| SnapshotError::OfflineParse {
                    path: path.display().to_string(),
                    line: line_number,
                    message: "continuation without a section".into(),
                })?;
            let option = last_option
                .as_ref()
                .ok_or_else(|| SnapshotError::OfflineParse {
                    path: path.display().to_string(),
                    line: line_number,
                    message: "continuation without an option".into(),
                })?;
            sections
                .get_mut(section)
                .and_then(|values| values.get_mut(option))
                .expect("current option must exist")
                .push_str(&format!("\n{trimmed}"));
            continue;
        }
        let section = current_section
            .as_ref()
            .ok_or_else(|| SnapshotError::OfflineParse {
                path: path.display().to_string(),
                line: line_number,
                message: "option outside a section".into(),
            })?;
        let delimiter = trimmed
            .char_indices()
            .find(|(_, character)| *character == '=' || *character == ':')
            .map(|(index, _)| index)
            .ok_or_else(|| SnapshotError::OfflineParse {
                path: path.display().to_string(),
                line: line_number,
                message: "option must use '=' or ':'".into(),
            })?;
        let option = trimmed[..delimiter].trim().to_ascii_lowercase();
        if option.is_empty() {
            return offline_parse_error(path, line_number, "empty option name");
        }
        let value = trimmed[delimiter + 1..].trim().to_string();
        sections
            .entry(section.clone())
            .or_default()
            .insert(option.clone(), value);
        last_option = Some(option);
    }
    Ok(())
}

fn offline_parse_error<T>(
    path: &Path,
    line: usize,
    message: impl Into<String>,
) -> Result<T, SnapshotError> {
    Err(SnapshotError::OfflineParse {
        path: path.display().to_string(),
        line,
        message: message.into(),
    })
}

fn resolve_include(
    root: &Path,
    source_file: &Path,
    include: &str,
) -> Result<Vec<PathBuf>, SnapshotError> {
    let include_path = Path::new(include);
    if include_path.is_absolute()
        || include_path
            .components()
            .any(|component| matches!(component, Component::Prefix(_) | Component::RootDir))
    {
        return Err(SnapshotError::OfflineAbsolutePath(include.into()));
    }
    let base = source_file.parent().unwrap_or(root);
    let has_magic = include
        .chars()
        .any(|character| matches!(character, '*' | '?' | '['));
    let candidates = expand_path_pattern(root, base, include_path)?;
    if candidates.is_empty() && !has_magic {
        return Err(SnapshotError::OfflineMissingInclude {
            include: include.into(),
            source_file: source_file.display().to_string(),
        });
    }
    let mut resolved = Vec::with_capacity(candidates.len());
    let mut canonical_sources = BTreeMap::new();
    for candidate in candidates {
        match confined_canonicalize(root, &candidate) {
            Ok(path) => {
                if let Some(first) = canonical_sources.insert(path.clone(), candidate.clone()) {
                    return Err(SnapshotError::OfflineAmbiguousInclude {
                        include: include.into(),
                        source_file: source_file.display().to_string(),
                        first: first.display().to_string(),
                        second: candidate.display().to_string(),
                    });
                }
                resolved.push(path);
            }
            Err(SnapshotError::OfflineRead { source, .. })
                if !has_magic && source.kind() == std::io::ErrorKind::NotFound =>
            {
                return Err(SnapshotError::OfflineMissingInclude {
                    include: include.into(),
                    source_file: source_file.display().to_string(),
                });
            }
            Err(error) => return Err(error),
        }
    }
    resolved.sort();
    Ok(resolved)
}

fn expand_path_pattern(
    root: &Path,
    base: &Path,
    pattern: &Path,
) -> Result<Vec<PathBuf>, SnapshotError> {
    let mut paths = vec![base.to_path_buf()];
    for component in pattern.components() {
        let Component::Normal(name) = component else {
            if component == Component::CurDir {
                continue;
            }
            if component == Component::ParentDir {
                paths = paths.into_iter().map(|path| path.join("..")).collect();
                continue;
            }
            return Ok(Vec::new());
        };
        let pattern = name.to_string_lossy();
        let magic = pattern
            .chars()
            .any(|character| matches!(character, '*' | '?' | '['));
        let mut next = Vec::new();
        for path in paths {
            if magic {
                let confined = match confined_canonicalize(root, &path) {
                    Ok(path) => path,
                    Err(SnapshotError::OfflineRead { source, .. })
                        if source.kind() == std::io::ErrorKind::NotFound =>
                    {
                        continue;
                    }
                    Err(error) => return Err(error),
                };
                let mut entries: Vec<_> = match fs::read_dir(&confined) {
                    Ok(entries) => entries.collect::<Result<_, _>>().map_err(|source| {
                        SnapshotError::OfflineRead {
                            path: confined.display().to_string(),
                            source,
                        }
                    })?,
                    Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
                    Err(source) => {
                        return Err(SnapshotError::OfflineRead {
                            path: confined.display().to_string(),
                            source,
                        })
                    }
                };
                entries.sort_by_key(|entry| entry.file_name());
                for entry in entries {
                    let name = entry.file_name();
                    let name = name.to_string_lossy();
                    if (!name.starts_with('.') || pattern.starts_with('.'))
                        && glob_component_matches(&pattern, &name)
                    {
                        next.push(entry.path());
                    }
                }
            } else {
                let candidate = path.join(name);
                next.push(candidate);
            }
        }
        paths = next;
    }
    Ok(paths)
}

fn glob_component_matches(pattern: &str, value: &str) -> bool {
    fn matches(pattern: &[char], value: &[char]) -> bool {
        match pattern.split_first() {
            None => value.is_empty(),
            Some(('*', rest)) => {
                matches(rest, value) || (!value.is_empty() && matches(pattern, &value[1..]))
            }
            Some(('?', rest)) => !value.is_empty() && matches(rest, &value[1..]),
            Some(('[', rest)) => {
                let Some(end) = rest.iter().position(|character| *character == ']') else {
                    return !value.is_empty() && value[0] == '[' && matches(rest, &value[1..]);
                };
                if value.is_empty() {
                    return false;
                }
                let class = &rest[..end];
                let negated = class
                    .first()
                    .is_some_and(|character| *character == '!' || *character == '^');
                let class = if negated { &class[1..] } else { class };
                let mut found = false;
                let mut index = 0;
                while index < class.len() {
                    if index + 2 < class.len() && class[index + 1] == '-' {
                        found |= class[index] <= value[0] && value[0] <= class[index + 2];
                        index += 3;
                    } else {
                        found |= class[index] == value[0];
                        index += 1;
                    }
                }
                (found != negated) && matches(&rest[end + 1..], &value[1..])
            }
            Some((literal, rest)) => {
                !value.is_empty() && *literal == value[0] && matches(rest, &value[1..])
            }
        }
    }
    matches(
        &pattern.chars().collect::<Vec<_>>(),
        &value.chars().collect::<Vec<_>>(),
    )
}

fn resolve_offline_settings(
    sections: &OfflineSections,
) -> Result<BTreeMap<String, Value>, SnapshotError> {
    let printer = sections
        .get("printer")
        .ok_or_else(|| SnapshotError::OfflineMissingOption {
            section: "printer".into(),
            option: "max_velocity".into(),
        })?;
    let max_velocity = required_float(printer, "printer", "max_velocity", |value| value > 0.0)?;
    let max_accel = required_float(printer, "printer", "max_accel", |value| value > 0.0)?;
    let mut resolved = BTreeMap::new();
    let mut resolved_printer = serde_json::Map::new();
    resolved_printer.insert("max_velocity".into(), json!(max_velocity));
    resolved_printer.insert("max_accel".into(), json!(max_accel));
    resolved_printer.insert(
        "minimum_cruise_ratio".into(),
        json!(optional_float(
            printer,
            "printer",
            "minimum_cruise_ratio",
            0.5,
            |value| { (0.0..1.0).contains(&value) }
        )?),
    );
    resolved_printer.insert(
        "square_corner_velocity".into(),
        json!(optional_float(
            printer,
            "printer",
            "square_corner_velocity",
            5.0,
            |value| { value >= 0.0 }
        )?),
    );
    if let Some(kinematics) = printer.get("kinematics") {
        resolved_printer.insert("kinematics".into(), json!(kinematics));
    }
    if let Some(value) = printer.get("max_accel_to_decel") {
        resolved_printer.insert(
            "max_accel_to_decel".into(),
            json!(parse_float(
                "printer",
                "max_accel_to_decel",
                value,
                |number| { number > 0.0 && number <= max_accel }
            )?),
        );
    }
    let kinematics = printer.get("kinematics").map(String::as_str);
    let has_klipper_z_velocity = matches!(
        kinematics,
        Some(
            "cartesian"
                | "corexy"
                | "corexz"
                | "hybrid_corexy"
                | "hybrid_corexz"
                | "generic_cartesian"
                | "delta"
                | "polar"
                | "deltesian"
                | "rotary_delta"
        )
    );
    if let Some(value) = printer.get("max_z_velocity") {
        resolved_printer.insert(
            "max_z_velocity".into(),
            json!(parse_float("printer", "max_z_velocity", value, |number| {
                number > 0.0 && number <= max_velocity
            })?),
        );
    } else if has_klipper_z_velocity {
        resolved_printer.insert("max_z_velocity".into(), json!(max_velocity));
    }
    if let Some(value) = printer.get("max_z_accel") {
        resolved_printer.insert(
            "max_z_accel".into(),
            json!(parse_float("printer", "max_z_accel", value, |number| {
                number > 0.0 && number <= max_accel
            })?),
        );
    } else if has_klipper_z_velocity && kinematics != Some("rotary_delta") {
        resolved_printer.insert("max_z_accel".into(), json!(max_accel));
    }

    if matches!(
        kinematics,
        Some("cartesian" | "corexy" | "corexz" | "hybrid_corexy" | "hybrid_corexz")
    ) {
        for name in ["stepper_x", "stepper_y", "stepper_z"] {
            let options =
                sections
                    .get(name)
                    .ok_or_else(|| SnapshotError::OfflineMissingOption {
                        section: name.into(),
                        option: "position_max".into(),
                    })?;
            let position_min = optional_float(options, name, "position_min", 0.0, f64::is_finite)?;
            let position_max =
                required_float(options, name, "position_max", |value| value > position_min)?;
            let position_endstop = required_float(options, name, "position_endstop", |value| {
                value >= position_min && value <= position_max
            })?;
            resolved.insert(
                name.into(),
                json!({
                    "position_min": position_min,
                    "position_max": position_max,
                    "position_endstop": position_endstop,
                }),
            );
        }
    }
    match kinematics {
        Some("delta") => {
            let radius = required_float(printer, "printer", "delta_radius", |value| value > 0.0)?;
            let print_radius =
                optional_float(printer, "printer", "print_radius", radius, |value| {
                    value > 0.0
                })?;
            let minimum_z = optional_float(
                printer,
                "printer",
                "minimum_z_position",
                0.0,
                f64::is_finite,
            )?;
            let names = ["stepper_a", "stepper_b", "stepper_c"];
            let first =
                sections
                    .get(names[0])
                    .ok_or_else(|| SnapshotError::OfflineMissingOption {
                        section: names[0].into(),
                        option: "arm_length".into(),
                    })?;
            let default_arm =
                required_float(first, names[0], "arm_length", |value| value > radius)?;
            let default_endstop =
                required_float(first, names[0], "position_endstop", f64::is_finite)?;
            for (index, name) in names.iter().enumerate() {
                let options =
                    sections
                        .get(*name)
                        .ok_or_else(|| SnapshotError::OfflineMissingOption {
                            section: (*name).into(),
                            option: "microsteps".into(),
                        })?;
                let arm = optional_float(options, name, "arm_length", default_arm, |value| {
                    value > radius
                })?;
                let endstop = optional_float(
                    options,
                    name,
                    "position_endstop",
                    default_endstop,
                    f64::is_finite,
                )?;
                let angle = optional_float(
                    options,
                    name,
                    "angle",
                    [210.0, 330.0, 90.0][index],
                    f64::is_finite,
                )?;
                resolved.insert(
                    (*name).into(),
                    json!({
                        "arm_length": arm,
                        "position_endstop": endstop,
                        "angle": angle,
                        "step_distance": offline_step_distance(options, name)?,
                    }),
                );
            }
            let maximum_z = names
                .iter()
                .filter_map(|name| resolved.get(*name)?.get("position_endstop")?.as_f64())
                .fold(f64::INFINITY, f64::min);
            if minimum_z > maximum_z {
                return Err(SnapshotError::OfflineValue {
                    section: "printer".into(),
                    option: "minimum_z_position".into(),
                    value: minimum_z.to_string(),
                    message: "value exceeds the delta maximum Z".into(),
                });
            }
            resolved_printer.insert("delta_radius".into(), json!(radius));
            resolved_printer.insert("print_radius".into(), json!(print_radius));
            resolved_printer.insert("minimum_z_position".into(), json!(minimum_z));
        }
        Some("polar") => {
            let arm =
                sections
                    .get("stepper_arm")
                    .ok_or_else(|| SnapshotError::OfflineMissingOption {
                        section: "stepper_arm".into(),
                        option: "position_max".into(),
                    })?;
            let z =
                sections
                    .get("stepper_z")
                    .ok_or_else(|| SnapshotError::OfflineMissingOption {
                        section: "stepper_z".into(),
                        option: "position_max".into(),
                    })?;
            let radius = required_float(arm, "stepper_arm", "position_max", |value| value > 0.0)?;
            let z_min = optional_float(z, "stepper_z", "position_min", 0.0, f64::is_finite)?;
            let z_max = required_float(z, "stepper_z", "position_max", |value| value > z_min)?;
            resolved.insert("stepper_arm".into(), json!({ "position_max": radius }));
            resolved.insert(
                "stepper_z".into(),
                json!({ "position_min": z_min, "position_max": z_max }),
            );
            resolved_printer.insert(
                "max_angular_velocity".into(),
                json!(optional_float(
                    printer,
                    "printer",
                    "max_angular_velocity",
                    0.0,
                    |value| value >= 0.0
                )?),
            );
        }
        Some("deltesian") => {
            let names = ["stepper_left", "stepper_right"];
            let left =
                sections
                    .get(names[0])
                    .ok_or_else(|| SnapshotError::OfflineMissingOption {
                        section: names[0].into(),
                        option: "arm_length".into(),
                    })?;
            let default_arm_x =
                required_float(left, names[0], "arm_x_length", |value| value > 0.0)?;
            let default_arm =
                required_float(left, names[0], "arm_length", |value| value > default_arm_x)?;
            let default_endstop =
                required_float(left, names[0], "position_endstop", f64::is_finite)?;
            for name in names {
                let options =
                    sections
                        .get(name)
                        .ok_or_else(|| SnapshotError::OfflineMissingOption {
                            section: name.into(),
                            option: "arm_length".into(),
                        })?;
                let arm_x =
                    optional_float(options, name, "arm_x_length", default_arm_x, |value| {
                        value > 0.0
                    })?;
                let arm = optional_float(options, name, "arm_length", default_arm, |value| {
                    value > arm_x
                })?;
                let endstop = optional_float(
                    options,
                    name,
                    "position_endstop",
                    default_endstop,
                    f64::is_finite,
                )?;
                resolved.insert(name.into(), json!({ "arm_x_length": arm_x, "arm_length": arm, "position_endstop": endstop }));
            }
            let y =
                sections
                    .get("stepper_y")
                    .ok_or_else(|| SnapshotError::OfflineMissingOption {
                        section: "stepper_y".into(),
                        option: "position_max".into(),
                    })?;
            let y_min = optional_float(y, "stepper_y", "position_min", 0.0, f64::is_finite)?;
            let y_max = required_float(y, "stepper_y", "position_max", |value| value > y_min)?;
            resolved.insert(
                "stepper_y".into(),
                json!({ "position_min": y_min, "position_max": y_max }),
            );
            resolved_printer.insert(
                "minimum_z_position".into(),
                json!(optional_float(
                    printer,
                    "printer",
                    "minimum_z_position",
                    0.0,
                    f64::is_finite
                )?),
            );
            resolved_printer.insert(
                "min_angle".into(),
                json!(optional_float(
                    printer,
                    "printer",
                    "min_angle",
                    5.0,
                    |value| (0.0..=90.0).contains(&value)
                )?),
            );
            resolved_printer.insert(
                "slow_ratio".into(),
                json!(optional_float(
                    printer,
                    "printer",
                    "slow_ratio",
                    3.0,
                    |value| value >= 0.0
                )?),
            );
            if let Some(value) = printer.get("print_width") {
                resolved_printer.insert(
                    "print_width".into(),
                    json!(parse_float(
                        "printer",
                        "print_width",
                        value,
                        |number| number >= 0.0
                    )?),
                );
            }
        }
        Some("rotary_delta") => {
            let shoulder_radius =
                required_float(printer, "printer", "shoulder_radius", |value| value > 0.0)?;
            let shoulder_height =
                required_float(printer, "printer", "shoulder_height", |value| value > 0.0)?;
            let names = ["stepper_a", "stepper_b", "stepper_c"];
            let first =
                sections
                    .get(names[0])
                    .ok_or_else(|| SnapshotError::OfflineMissingOption {
                        section: names[0].into(),
                        option: "upper_arm_length".into(),
                    })?;
            let default_upper =
                required_float(first, names[0], "upper_arm_length", |value| value > 0.0)?;
            let default_lower =
                required_float(first, names[0], "lower_arm_length", |value| value > 0.0)?;
            let default_endstop =
                required_float(first, names[0], "position_endstop", f64::is_finite)?;
            for (index, name) in names.iter().enumerate() {
                let options =
                    sections
                        .get(*name)
                        .ok_or_else(|| SnapshotError::OfflineMissingOption {
                            section: (*name).into(),
                            option: "upper_arm_length".into(),
                        })?;
                resolved.insert((*name).into(), json!({
                    "upper_arm_length": optional_float(options, name, "upper_arm_length", default_upper, |value| value > 0.0)?,
                    "lower_arm_length": optional_float(options, name, "lower_arm_length", default_lower, |value| value > 0.0)?,
                    "position_endstop": optional_float(options, name, "position_endstop", default_endstop, f64::is_finite)?,
                    "angle": optional_float(options, name, "angle", [30.0, 150.0, 270.0][index], f64::is_finite)?,
                }));
            }
            resolved_printer.insert(
                "minimum_z_position".into(),
                json!(optional_float(
                    printer,
                    "printer",
                    "minimum_z_position",
                    0.0,
                    f64::is_finite
                )?),
            );
            resolved_printer.insert("shoulder_radius".into(), json!(shoulder_radius));
            resolved_printer.insert("shoulder_height".into(), json!(shoulder_height));
        }
        _ => {}
    }
    resolved.insert("printer".into(), Value::Object(resolved_printer));
    if sections.contains_key("dual_carriage") {
        resolved.insert("dual_carriage".into(), json!({}));
    }

    let mut configured_extruders: Vec<_> = sections
        .iter()
        .filter(|(name, _)| is_extruder_object(name))
        .collect();
    configured_extruders.sort_by_key(|(name, _)| {
        name.strip_prefix("extruder")
            .filter(|suffix| !suffix.is_empty())
            .and_then(|suffix| suffix.parse::<usize>().ok())
            .unwrap_or(0)
    });
    for (expected, (name, _)) in configured_extruders.iter().enumerate() {
        let actual = if name.as_str() == "extruder" {
            0
        } else {
            name["extruder".len()..].parse::<usize>().unwrap()
        };
        if actual != expected || actual >= 99 {
            return Err(SnapshotError::OfflineExtruderSequence((*name).clone()));
        }
    }
    for (name, options) in configured_extruders {
        let nozzle = required_float(options, name, "nozzle_diameter", |value| value > 0.0)?;
        let filament = required_float(options, name, "filament_diameter", |value| value >= nozzle)?;
        let filament_area = std::f64::consts::PI * (filament * 0.5).powi(2);
        let default_cross_section = 4.0 * nozzle.powi(2);
        let default_ratio = default_cross_section / filament_area;
        let cross_section = optional_float(
            options,
            name,
            "max_extrude_cross_section",
            default_cross_section,
            |value| value > 0.0,
        )?;
        let mut object = serde_json::Map::new();
        object.insert("nozzle_diameter".into(), json!(nozzle));
        object.insert("filament_diameter".into(), json!(filament));
        object.insert("max_extrude_cross_section".into(), json!(cross_section));
        object.insert(
            "max_extrude_only_velocity".into(),
            json!(optional_float(
                options,
                name,
                "max_extrude_only_velocity",
                max_velocity * default_ratio,
                |value| value > 0.0,
            )?),
        );
        object.insert(
            "max_extrude_only_accel".into(),
            json!(optional_float(
                options,
                name,
                "max_extrude_only_accel",
                max_accel * default_ratio,
                |value| value > 0.0,
            )?),
        );
        object.insert(
            "max_extrude_only_distance".into(),
            json!(optional_float(
                options,
                name,
                "max_extrude_only_distance",
                50.0,
                |value| value >= 0.0,
            )?),
        );
        object.insert(
            "instantaneous_corner_velocity".into(),
            json!(optional_float(
                options,
                name,
                "instantaneous_corner_velocity",
                1.0,
                |value| value >= 0.0,
            )?),
        );
        resolved.insert(name.clone(), Value::Object(object));
    }
    if !resolved.contains_key("extruder") {
        return Err(SnapshotError::OfflineMissingOption {
            section: "extruder".into(),
            option: "nozzle_diameter".into(),
        });
    }

    if let Some(options) = sections.get("firmware_retraction") {
        let mut object = serde_json::Map::new();
        for (name, default, validator) in [
            ("retract_length", 0.0, nonnegative as fn(f64) -> bool),
            ("retract_speed", 20.0, at_least_one),
            ("unretract_extra_length", 0.0, nonnegative),
            ("unretract_speed", 10.0, at_least_one),
        ] {
            object.insert(
                name.into(),
                json!(optional_float(
                    options,
                    "firmware_retraction",
                    name,
                    default,
                    validator
                )?),
            );
        }
        resolved.insert("firmware_retraction".into(), Value::Object(object));
    }
    if let Some(options) = sections.get("gcode_arcs") {
        let resolution = optional_float(options, "gcode_arcs", "resolution", 1.0, |value| {
            value > 0.0
        })?;
        resolved.insert("gcode_arcs".into(), json!({ "resolution": resolution }));
    }
    Ok(resolved)
}

fn nonnegative(value: f64) -> bool {
    value >= 0.0
}

fn at_least_one(value: f64) -> bool {
    value >= 1.0
}

fn required_float(
    options: &BTreeMap<String, String>,
    section: &str,
    option: &str,
    validator: impl Fn(f64) -> bool,
) -> Result<f64, SnapshotError> {
    let value = options
        .get(option)
        .ok_or_else(|| SnapshotError::OfflineMissingOption {
            section: section.into(),
            option: option.into(),
        })?;
    parse_float(section, option, value, validator)
}

fn optional_float(
    options: &BTreeMap<String, String>,
    section: &str,
    option: &str,
    default: f64,
    validator: impl Fn(f64) -> bool,
) -> Result<f64, SnapshotError> {
    match options.get(option) {
        Some(value) => parse_float(section, option, value, validator),
        None => Ok(default),
    }
}

fn parse_float(
    section: &str,
    option: &str,
    value: &str,
    validator: impl Fn(f64) -> bool,
) -> Result<f64, SnapshotError> {
    value
        .parse::<f64>()
        .ok()
        .filter(|number| number.is_finite() && validator(*number))
        .ok_or_else(|| SnapshotError::OfflineValue {
            section: section.into(),
            option: option.into(),
            value: value.into(),
            message: "value is outside the range accepted by Klipper".into(),
        })
}

pub fn write_cache(path: &str, snapshot: &ConfigSnapshot) -> Result<(), SnapshotError> {
    let encoded =
        serde_json::to_vec_pretty(snapshot).expect("configuration snapshot must serialize");
    fs::write(path, encoded).map_err(SnapshotError::CacheWrite)
}

pub fn read_cache(
    path: &str,
    selection: SnapshotSelection,
) -> Result<ConfigSnapshot, SnapshotError> {
    if selection == SnapshotSelection::RuntimeSnapshot {
        return Err(SnapshotError::StaleRuntimeCache);
    }
    let bytes = fs::read(path).map_err(SnapshotError::CacheRead)?;
    match serde_json::from_slice::<ConfigSnapshot>(&bytes) {
        Ok(mut snapshot) => {
            snapshot.validate()?;
            snapshot.upgrade_legacy_extruders()?;
            snapshot.source.kind = SnapshotSourceKind::MoonrakerCache;
            snapshot.source.location = Some(path.into());
            snapshot.accuracy = SnapshotAccuracy::Degraded;
            snapshot.warnings.push(
                "Moonraker was unavailable; using a cached configuration-default snapshot".into(),
            );
            Ok(snapshot)
        }
        Err(snapshot_error) => {
            let limits = serde_json::from_slice::<PrinterLimits>(&bytes)
                .map_err(|_| SnapshotError::CacheParse(snapshot_error))?;
            let mut snapshot = ConfigSnapshot::built_in_defaults();
            snapshot.source = SnapshotSource {
                kind: SnapshotSourceKind::MoonrakerCache,
                location: Some(path.into()),
                selection,
            };
            snapshot.limits = limits;
            snapshot.warnings = vec![
                "imported a legacy Moonraker cache without Klipper version or settings provenance"
                    .into(),
            ];
            snapshot.refresh_fingerprint();
            Ok(snapshot)
        }
    }
}

pub fn map_auth_error(error: &SnapshotError) -> Option<String> {
    match error {
        SnapshotError::Request(request_error)
            if request_error.status() == Some(StatusCode::UNAUTHORIZED) =>
        {
            Some(format!(
                "access denied (you may need --config_moonraker_api_key): {request_error}"
            ))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestDirectory(PathBuf);

    impl TestDirectory {
        fn new(name: &str) -> Self {
            let path = std::env::temp_dir().join(format!(
                "klipper-estimator-{name}-{}-{}",
                std::process::id(),
                now_unix_seconds()
            ));
            fs::create_dir_all(&path).unwrap();
            Self(path)
        }

        fn write(&self, relative: &str, contents: &str) {
            let path = self.0.join(relative);
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent).unwrap();
            }
            fs::write(path, contents).unwrap();
        }
    }

    impl Drop for TestDirectory {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    fn settings() -> BTreeMap<String, Value> {
        serde_json::from_value(json!({
            "printer": {
                "kinematics": "cartesian",
                "max_velocity": 300.0,
                "max_accel": 5000.0,
                "minimum_cruise_ratio": 0.5,
                "square_corner_velocity": 5.0,
                "max_z_velocity": 20.0,
                "max_z_accel": 100.0
            },
            "stepper_x": { "position_min": 0.0, "position_max": 250.0 },
            "stepper_y": { "position_min": 0.0, "position_max": 240.0 },
            "stepper_z": { "position_min": 0.0, "position_max": 220.0 },
            "extruder": {
                "nozzle_diameter": 0.4,
                "filament_diameter": 1.75,
                "max_extrude_only_velocity": 25.0,
                "max_extrude_only_accel": 1250.0,
                "max_extrude_only_distance": 50.0,
                "instantaneous_corner_velocity": 1.0,
                "max_extrude_cross_section": 5.0
            },
            "extruder1": {
                "nozzle_diameter": 0.6,
                "filament_diameter": 1.75,
                "max_extrude_only_velocity": 30.0,
                "max_extrude_only_accel": 1300.0,
                "max_extrude_only_distance": 40.0,
                "instantaneous_corner_velocity": 1.5,
                "max_extrude_cross_section": 1.44
            }
        }))
        .unwrap()
    }

    fn status() -> BTreeMap<String, Value> {
        let settings: serde_json::Map<String, Value> = settings().into_iter().collect();
        serde_json::from_value(json!({
            "configfile": { "settings": settings },
            "toolhead": {
                "max_velocity": 180.0,
                "max_accel": 2500.0,
                "minimum_cruise_ratio": 0.25,
                "square_corner_velocity": 4.0,
                "extruder": "extruder1"
            },
            "gcode_move": {
                "absolute_coordinates": false,
                "absolute_extrude": true,
                "speed_factor": 0.8,
                "extrude_factor": 0.9
            },
            "extruder": { "pressure_advance": 0.04 },
            "extruder1": { "pressure_advance": 0.02 }
        }))
        .unwrap()
    }

    #[test]
    fn configuration_default_and_runtime_modes_remain_distinct() {
        let defaults = snapshot_from_status(
            "http://printer.local",
            SnapshotSelection::ConfigurationDefault,
            Some("v0.13.0-test".into()),
            status(),
        )
        .unwrap();
        let runtime = snapshot_from_status(
            "http://printer.local",
            SnapshotSelection::RuntimeSnapshot,
            Some("v0.13.0-test".into()),
            status(),
        )
        .unwrap();

        assert_eq!(defaults.limits.max_velocity, 300.0);
        assert_eq!(runtime.limits.max_velocity, 180.0);
        assert_eq!(
            defaults.limits.initial_coordinate_mode,
            PositionMode::Absolute
        );
        assert_eq!(
            runtime.limits.initial_coordinate_mode,
            PositionMode::Relative
        );
        assert_eq!(
            runtime.limits.initial_extrusion_mode,
            PositionMode::Absolute
        );
        assert_eq!(defaults.extruders.len(), 2);
        assert_eq!(defaults.limits.initial_extruder, None);
        assert_eq!(
            runtime.limits.initial_extruder.as_deref(),
            Some("extruder1")
        );
        assert!(matches!(
            defaults.limits.kinematics,
            Kinematics::CartesianFamily { .. }
        ));
        assert_eq!(defaults.accuracy, SnapshotAccuracy::Complete);
        assert_ne!(defaults.fingerprint, runtime.fingerprint);
        defaults.validate().unwrap();
        runtime.validate().unwrap();
    }

    #[test]
    fn unsupported_kinematics_degrades_snapshot_accuracy() {
        let mut unsupported_status = status();
        unsupported_status
            .get_mut("configfile")
            .and_then(Value::as_object_mut)
            .and_then(|configfile| configfile.get_mut("settings"))
            .and_then(Value::as_object_mut)
            .and_then(|settings| settings.get_mut("printer"))
            .and_then(Value::as_object_mut)
            .unwrap()
            .insert("kinematics".into(), json!("winch"));
        let snapshot = snapshot_from_status(
            "http://printer.local",
            SnapshotSelection::ConfigurationDefault,
            Some("v0.13.0-test".into()),
            unsupported_status,
        )
        .unwrap();

        assert_eq!(snapshot.accuracy, SnapshotAccuracy::Degraded);
        assert!(matches!(
            snapshot.limits.kinematics,
            Kinematics::Unsupported { .. }
        ));
        assert!(snapshot
            .warnings
            .iter()
            .any(|warning| warning.contains("winch")));

        let mut dual_carriage_settings = settings();
        dual_carriage_settings.insert("dual_carriage".into(), json!({ "axis": "x" }));
        let extruders = parse_extruders(&dual_carriage_settings).unwrap();
        let limits = limits_from_settings(&dual_carriage_settings, &extruders).unwrap();
        assert!(matches!(limits.kinematics, Kinematics::Unsupported { .. }));
    }

    #[test]
    fn nonlinear_moonraker_settings_import_geometry() {
        let cases = [
            (
                "delta",
                json!({
                    "printer": { "kinematics": "delta", "max_velocity": 300.0, "max_accel": 3000.0, "minimum_cruise_ratio": 0.5, "square_corner_velocity": 5.0, "max_z_velocity": 150.0, "max_z_accel": 1000.0, "delta_radius": 174.75, "print_radius": 170.0, "minimum_z_position": 0.0 },
                    "stepper_a": { "arm_length": 333.0, "position_endstop": 297.05, "rotation_distance": 40.0, "microsteps": 16.0 },
                    "stepper_b": { "arm_length": 333.0, "position_endstop": 297.05, "rotation_distance": 40.0, "microsteps": 16.0 },
                    "stepper_c": { "arm_length": 333.0, "position_endstop": 297.05, "rotation_distance": 40.0, "microsteps": 16.0 }
                }),
            ),
            (
                "polar",
                json!({
                    "printer": { "kinematics": "polar", "max_velocity": 300.0, "max_accel": 3000.0, "minimum_cruise_ratio": 0.5, "square_corner_velocity": 5.0, "max_z_velocity": 25.0, "max_z_accel": 30.0, "max_angular_velocity": 5.0 },
                    "stepper_arm": { "position_max": 300.0 },
                    "stepper_z": { "position_min": 0.0, "position_max": 200.0 }
                }),
            ),
            (
                "deltesian",
                json!({
                    "printer": { "kinematics": "deltesian", "max_velocity": 300.0, "max_accel": 3000.0, "minimum_cruise_ratio": 0.5, "square_corner_velocity": 5.0, "max_z_velocity": 150.0, "max_z_accel": 1000.0, "minimum_z_position": 0.0, "min_angle": 5.0, "slow_ratio": 3.0 },
                    "stepper_left": { "arm_x_length": 160.0, "arm_length": 217.0, "position_endstop": 268.0 },
                    "stepper_right": { "arm_x_length": 160.0, "arm_length": 217.0, "position_endstop": 268.0 },
                    "stepper_y": { "position_min": 0.0, "position_max": 200.0 }
                }),
            ),
            (
                "rotary_delta",
                json!({
                    "printer": { "kinematics": "rotary_delta", "max_velocity": 300.0, "max_accel": 3000.0, "minimum_cruise_ratio": 0.5, "square_corner_velocity": 5.0, "max_z_velocity": 50.0, "shoulder_radius": 33.9, "shoulder_height": 412.9, "minimum_z_position": 0.0 },
                    "stepper_a": { "upper_arm_length": 170.0, "lower_arm_length": 320.0, "position_endstop": 252.0 },
                    "stepper_b": { "upper_arm_length": 170.0, "lower_arm_length": 320.0, "position_endstop": 252.0 },
                    "stepper_c": { "upper_arm_length": 170.0, "lower_arm_length": 320.0, "position_endstop": 252.0 }
                }),
            ),
        ];
        for (backend, value) in cases {
            let settings: BTreeMap<String, Value> = serde_json::from_value(value).unwrap();
            let printer = settings["printer"].as_object().unwrap();
            let kinematics = kinematics_from_settings(&settings, printer).unwrap();
            assert!(matches!(
                (backend, kinematics),
                ("delta", Kinematics::Delta { .. })
                    | ("polar", Kinematics::Polar { .. })
                    | ("deltesian", Kinematics::Deltesian { .. })
                    | ("rotary_delta", Kinematics::RotaryDelta { .. })
            ));
        }
    }

    #[test]
    fn invalid_nonlinear_geometry_fails_loading() {
        let settings: BTreeMap<String, Value> = serde_json::from_value(json!({
            "printer": { "kinematics": "delta", "max_velocity": 300.0, "max_accel": 3000.0, "max_z_velocity": 150.0, "max_z_accel": 1000.0, "delta_radius": 200.0 },
            "stepper_a": { "arm_length": 190.0, "position_endstop": 250.0, "rotation_distance": 40.0, "microsteps": 16.0 },
            "stepper_b": { "arm_length": 190.0, "position_endstop": 250.0, "rotation_distance": 40.0, "microsteps": 16.0 },
            "stepper_c": { "arm_length": 190.0, "position_endstop": 250.0, "rotation_distance": 40.0, "microsteps": 16.0 }
        })).unwrap();
        let error = kinematics_from_settings(&settings, settings["printer"].as_object().unwrap())
            .unwrap_err();
        assert!(matches!(
            error,
            SnapshotError::InvalidKinematicsGeometry { .. }
        ));
    }

    #[test]
    fn nonlinear_offline_configurations_resolve_without_degradation() {
        const EXTRUDER: &str = r#"
[extruder]
nozzle_diameter: 0.4
filament_diameter: 1.75
"#;
        let cases = [
            (
                "delta",
                r#"[printer]
kinematics: delta
max_velocity: 300
max_accel: 3000
max_z_velocity: 150
max_z_accel: 1000
delta_radius: 174.75
[stepper_a]
microsteps: 16
rotation_distance: 40
position_endstop: 297.05
arm_length: 333
[stepper_b]
microsteps: 16
rotation_distance: 40
[stepper_c]
microsteps: 16
rotation_distance: 40
"#,
            ),
            (
                "polar",
                r#"[printer]
kinematics: polar
max_velocity: 300
max_accel: 3000
max_z_velocity: 25
max_z_accel: 30
max_angular_velocity: 5
[stepper_bed]
[stepper_arm]
position_max: 300
[stepper_z]
position_min: 0
position_max: 200
"#,
            ),
            (
                "deltesian",
                r#"[printer]
kinematics: deltesian
max_velocity: 300
max_accel: 3000
max_z_velocity: 150
max_z_accel: 1000
[stepper_left]
position_endstop: 268
arm_length: 217
arm_x_length: 160
[stepper_right]
[stepper_y]
position_min: 0
position_max: 200
"#,
            ),
            (
                "rotary_delta",
                r#"[printer]
kinematics: rotary_delta
max_velocity: 300
max_accel: 3000
max_z_velocity: 50
shoulder_radius: 33.9
shoulder_height: 412.9
[stepper_a]
position_endstop: 252
upper_arm_length: 170
lower_arm_length: 320
[stepper_b]
[stepper_c]
"#,
            ),
        ];
        for (backend, config) in cases {
            let directory = TestDirectory::new(&format!("offline-{backend}"));
            directory.write("printer.cfg", &format!("{config}{EXTRUDER}"));
            let snapshot =
                load_offline_snapshot(directory.0.to_str().unwrap(), "printer.cfg").unwrap();
            assert_eq!(snapshot.accuracy, SnapshotAccuracy::Complete);
            assert!(matches!(
                (backend, snapshot.limits.kinematics),
                ("delta", Kinematics::Delta { .. })
                    | ("polar", Kinematics::Polar { .. })
                    | ("deltesian", Kinematics::Deltesian { .. })
                    | ("rotary_delta", Kinematics::RotaryDelta { .. })
            ));
        }
    }

    #[test]
    fn fingerprint_changes_with_effective_limits() {
        let mut first = ConfigSnapshot::built_in_defaults();
        let mut second = first.clone();
        second
            .limits
            .set_max_velocity(first.limits.max_velocity + 1.0);
        second.refresh_fingerprint();

        assert_ne!(first.fingerprint, second.fingerprint);
        first.fingerprint.push('0');
        assert!(matches!(
            first.validate(),
            Err(SnapshotError::FingerprintMismatch { .. })
        ));
    }

    #[test]
    fn exported_snapshot_round_trips_with_provenance() {
        let snapshot = snapshot_from_status(
            "http://printer.local",
            SnapshotSelection::ConfigurationDefault,
            Some("v0.13.0-test".into()),
            status(),
        )
        .unwrap();
        let encoded = serde_json::to_string_pretty(&snapshot).unwrap();
        let imported: ConfigSnapshot = serde_json::from_str(&encoded).unwrap();

        imported.validate().unwrap();
        assert_eq!(imported.source, snapshot.source);
        assert_eq!(imported.klipper_version, snapshot.klipper_version);
        assert_eq!(imported.fingerprint, snapshot.fingerprint);
        assert_eq!(imported.configfile_settings, snapshot.configfile_settings);

        let imported_json5 = config::Config::builder()
            .add_source(config::File::from_str(&encoded, config::FileFormat::Json5))
            .build()
            .unwrap()
            .try_deserialize::<ConfigSnapshot>()
            .unwrap();
        imported_json5.validate().unwrap();
        assert_eq!(imported_json5.fingerprint, snapshot.fingerprint);
    }

    #[test]
    fn legacy_snapshot_upgrades_to_per_extruder_limits_after_validation() {
        let mut legacy = snapshot_from_status(
            "http://printer.local",
            SnapshotSelection::ConfigurationDefault,
            Some("v0.13.0-test".into()),
            status(),
        )
        .unwrap();
        legacy.limits.extruders.clear();
        legacy.limits.initial_extruder = None;
        legacy.refresh_fingerprint();

        let encoded = serde_json::to_vec(&legacy).unwrap();
        let mut imported: ConfigSnapshot = serde_json::from_slice(&encoded).unwrap();
        imported.validate().unwrap();
        imported.upgrade_legacy_extruders().unwrap();

        assert_eq!(imported.limits.extruders.len(), 2);
        assert_eq!(
            imported.limits.extruders["extruder1"].max_extrude_only_velocity,
            30.0
        );
        assert!(imported
            .warnings
            .iter()
            .any(|warning| warning.contains("migrated legacy snapshot")));
        imported.validate().unwrap();
    }

    #[test]
    fn runtime_mode_rejects_cache_before_reading_it() {
        let error =
            read_cache("does-not-need-to-exist", SnapshotSelection::RuntimeSnapshot).unwrap_err();
        assert!(matches!(error, SnapshotError::StaleRuntimeCache));
    }

    #[test]
    fn cache_fallback_is_machine_readable_and_keeps_original_retrieval_time() {
        let snapshot = snapshot_from_status(
            "http://printer.local",
            SnapshotSelection::ConfigurationDefault,
            Some("v0.13.0-test".into()),
            status(),
        )
        .unwrap();
        let path = std::env::temp_dir().join(format!(
            "klipper-estimator-snapshot-{}-{}.json",
            std::process::id(),
            now_unix_seconds()
        ));
        fs::write(&path, serde_json::to_vec(&snapshot).unwrap()).unwrap();

        let cached = read_cache(
            path.to_str().unwrap(),
            SnapshotSelection::ConfigurationDefault,
        )
        .unwrap();
        fs::remove_file(path).unwrap();

        assert_eq!(cached.accuracy, SnapshotAccuracy::Degraded);
        assert_eq!(
            cached.retrieved_at_unix_seconds,
            snapshot.retrieved_at_unix_seconds
        );
        assert!(cached
            .warnings
            .iter()
            .any(|warning| warning.contains("cached configuration-default snapshot")));
        assert_eq!(cached.summary().accuracy, SnapshotAccuracy::Degraded);
    }

    #[test]
    fn discovery_builds_a_post_object_query() {
        let available = [
            "configfile",
            "toolhead",
            "gcode_move",
            "extruder",
            "extruder1",
        ]
        .into_iter()
        .map(str::to_string)
        .collect();
        let objects = requested_objects(&available);
        let request = object_query_request(
            &reqwest::blocking::Client::new(),
            Url::parse("http://printer.local/printer/objects/query").unwrap(),
            Some("secret"),
            &objects,
        )
        .unwrap();

        assert_eq!(request.method(), reqwest::Method::POST);
        assert_eq!(request.headers()["X-Api-Key"], "secret");
        let body: Value =
            serde_json::from_slice(request.body().unwrap().as_bytes().unwrap()).unwrap();
        assert!(body["objects"]["extruder1"].is_null());
        assert!(body["objects"].get("firmware_retraction").is_none());
    }

    #[test]
    fn offline_include_save_config_and_defaults_match_moonraker_settings() {
        let directory = TestDirectory::new("offline-equivalence");
        directory.write(
            "printer.cfg",
            r#"[printer]
kinematics: cartesian
max_velocity: 300
[include conf.d/*.cfg]
[printer]
max_accel: 5000
[stepper_x]
position_endstop: 0
position_max: 250
[stepper_y]
position_endstop: 0
position_max: 240
[stepper_z]
position_endstop: 0
position_max: 220
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [printer]
#*# max_accel = 900
"#,
        );
        directory.write(
            "conf.d/10-extruder.cfg",
            r#"[extruder]
nozzle_diameter: 0.4
filament_diameter: 1.75
[firmware_retraction]
"#,
        );
        directory.write(
            "conf.d/20-limits.cfg",
            r#"[printer]
max_velocity: 250
[gcode_arcs]
"#,
        );

        let offline = load_offline_snapshot(directory.0.to_str().unwrap(), "printer.cfg").unwrap();
        assert_eq!(
            offline.source.kind,
            SnapshotSourceKind::OfflineConfiguration
        );
        assert_eq!(offline.limits.max_velocity, 250.0);
        assert_eq!(offline.limits.max_acceleration, 5000.0);
        assert_eq!(offline.limits.minimum_cruise_ratio, Some(0.5));
        match &offline.limits.kinematics {
            Kinematics::CartesianFamily { config } => {
                assert_eq!(config.kind, CartesianKinematicsKind::Cartesian);
                assert_eq!(config.axis_maximum, DVec3::new(250.0, 240.0, 220.0));
            }
            other => panic!("expected resolved Cartesian kinematics, got {:?}", other),
        }
        assert_eq!(offline.limits.mm_per_arc_segment, Some(1.0));
        assert_eq!(
            offline
                .limits
                .firmware_retraction
                .as_ref()
                .unwrap()
                .retract_speed,
            20.0
        );

        let settings = offline.configfile_settings.clone();
        let settings_object: serde_json::Map<String, Value> = settings.into_iter().collect();
        let comparison_status: BTreeMap<String, Value> = serde_json::from_value(json!({
            "configfile": { "settings": settings_object },
            "toolhead": {},
            "gcode_move": {},
            "extruder": {}
        }))
        .unwrap();
        let moonraker = snapshot_from_status(
            "http://printer.local",
            SnapshotSelection::ConfigurationDefault,
            None,
            comparison_status,
        )
        .unwrap();
        assert_eq!(offline.fingerprint, moonraker.fingerprint);
        offline.validate().unwrap();
    }

    #[test]
    fn bundled_offline_tree_matches_equivalent_moonraker_snapshot() {
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../tests/fixtures/offline_config")
            .canonicalize()
            .unwrap();
        let offline = load_offline_snapshot(fixture.to_str().unwrap(), "printer.cfg").unwrap();
        let settings: serde_json::Map<String, Value> =
            offline.configfile_settings.clone().into_iter().collect();
        let status = serde_json::from_value(json!({
            "configfile": { "settings": settings },
            "toolhead": {},
            "gcode_move": {},
            "extruder": {},
            "extruder1": {}
        }))
        .unwrap();
        let moonraker = snapshot_from_status(
            "http://printer.local",
            SnapshotSelection::ConfigurationDefault,
            None,
            status,
        )
        .unwrap();

        assert_eq!(offline.fingerprint, moonraker.fingerprint);
        assert_eq!(offline.extruders, moonraker.extruders);
        assert_eq!(offline.limits.square_corner_velocity, 6.0);
        assert_eq!(
            offline
                .limits
                .firmware_retraction
                .as_ref()
                .unwrap()
                .retract_length,
            0.8
        );
        assert!(offline
            .warnings
            .iter()
            .any(|warning| warning.contains("bed_mesh")));
    }

    #[test]
    fn offline_configuration_rejects_cycles_missing_and_outside_includes() {
        let directory = TestDirectory::new("offline-errors");
        directory.write("cycle.cfg", "[include nested/cycle.cfg]\n");
        directory.write("nested/cycle.cfg", "[include ../cycle.cfg]\n");
        assert!(matches!(
            load_offline_snapshot(directory.0.to_str().unwrap(), "cycle.cfg"),
            Err(SnapshotError::OfflineRecursiveInclude(_))
        ));

        directory.write("missing.cfg", "[include absent.cfg]\n");
        assert!(matches!(
            load_offline_snapshot(directory.0.to_str().unwrap(), "missing.cfg"),
            Err(SnapshotError::OfflineMissingInclude { .. })
        ));

        let outside = directory.0.parent().unwrap().join(format!(
            "klipper-estimator-outside-{}-{}",
            std::process::id(),
            now_unix_seconds()
        ));
        fs::write(&outside, "[printer]\n").unwrap();
        directory.write(
            "outside.cfg",
            &format!(
                "[include ../{}]\n",
                outside.file_name().unwrap().to_string_lossy()
            ),
        );
        assert!(matches!(
            load_offline_snapshot(directory.0.to_str().unwrap(), "outside.cfg"),
            Err(SnapshotError::OfflineOutsideRoot { .. })
        ));
        fs::remove_file(outside).unwrap();

        #[cfg(unix)]
        {
            use std::os::unix::fs::symlink;

            directory.write("shared.cfg", "[printer]\n");
            fs::create_dir_all(directory.0.join("aliases")).unwrap();
            symlink("../shared.cfg", directory.0.join("aliases/first.cfg")).unwrap();
            symlink("../shared.cfg", directory.0.join("aliases/second.cfg")).unwrap();
            directory.write("ambiguous.cfg", "[include aliases/*.cfg]\n");
            assert!(matches!(
                load_offline_snapshot(directory.0.to_str().unwrap(), "ambiguous.cfg"),
                Err(SnapshotError::OfflineAmbiguousInclude { .. })
            ));
        }
    }

    #[test]
    fn offline_configuration_rejects_corrupt_save_config_and_invalid_values() {
        let directory = TestDirectory::new("offline-invalid");
        directory.write(
            "corrupt.cfg",
            "#*# <---------------------- SAVE_CONFIG ---------------------->\nmodified\n",
        );
        assert!(matches!(
            load_offline_snapshot(directory.0.to_str().unwrap(), "corrupt.cfg"),
            Err(SnapshotError::OfflineParse { .. })
        ));

        directory.write(
            "value.cfg",
            "[printer]\nmax_velocity: fast\nmax_accel: 1000\n[extruder]\nnozzle_diameter: 0.4\nfilament_diameter: 1.75\n",
        );
        assert!(matches!(
            load_offline_snapshot(directory.0.to_str().unwrap(), "value.cfg"),
            Err(SnapshotError::OfflineValue { .. })
        ));

        directory.write(
            "extruder-gap.cfg",
            "[printer]\nmax_velocity: 100\nmax_accel: 1000\n[extruder]\nnozzle_diameter: 0.4\nfilament_diameter: 1.75\n[extruder2]\nnozzle_diameter: 0.4\nfilament_diameter: 1.75\n",
        );
        assert!(matches!(
            load_offline_snapshot(directory.0.to_str().unwrap(), "extruder-gap.cfg"),
            Err(SnapshotError::OfflineExtruderSequence(_))
        ));
    }
}
