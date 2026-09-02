use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use lib_klipper::glam::DVec3;
use lib_klipper::planner::{FirmwareRetractionOptions, MoveChecker, PositionMode, PrinterLimits};
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

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ExtruderSnapshot {
    pub max_extrude_only_velocity: f64,
    pub max_extrude_only_accel: f64,
    pub instantaneous_corner_velocity: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_extrude_cross_section: Option<f64>,
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
    snapshot.refresh_fingerprint();
    Ok(snapshot)
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
    limits.move_checkers.push(MoveChecker::ExtruderLimiter {
        max_velocity: primary_extruder.max_extrude_only_velocity,
        max_accel: primary_extruder.max_extrude_only_accel,
    });

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
                    max_extrude_only_velocity: number(
                        object,
                        &format!("configfile.settings.{name}.max_extrude_only_velocity"),
                    )?,
                    max_extrude_only_accel: number(
                        object,
                        &format!("configfile.settings.{name}.max_extrude_only_accel"),
                    )?,
                    instantaneous_corner_velocity: number(
                        object,
                        &format!("configfile.settings.{name}.instantaneous_corner_velocity"),
                    )?,
                    max_extrude_cross_section: optional_number(object, "max_extrude_cross_section"),
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

    let supported = ["printer", "firmware_retraction", "gcode_arcs"];
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
    let has_klipper_z_limits = matches!(
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
        )
    );
    if let Some(value) = printer.get("max_z_velocity") {
        resolved_printer.insert(
            "max_z_velocity".into(),
            json!(parse_float("printer", "max_z_velocity", value, |number| {
                number > 0.0 && number <= max_velocity
            })?),
        );
    } else if has_klipper_z_limits {
        resolved_printer.insert("max_z_velocity".into(), json!(max_velocity));
    }
    if let Some(value) = printer.get("max_z_accel") {
        resolved_printer.insert(
            "max_z_accel".into(),
            json!(parse_float("printer", "max_z_accel", value, |number| {
                number > 0.0 && number <= max_accel
            })?),
        );
    } else if has_klipper_z_limits {
        resolved_printer.insert("max_z_accel".into(), json!(max_accel));
    }
    resolved.insert("printer".into(), Value::Object(resolved_printer));

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
                "max_velocity": 300.0,
                "max_accel": 5000.0,
                "minimum_cruise_ratio": 0.5,
                "square_corner_velocity": 5.0,
                "max_z_velocity": 20.0,
                "max_z_accel": 100.0
            },
            "extruder": {
                "max_extrude_only_velocity": 25.0,
                "max_extrude_only_accel": 1250.0,
                "instantaneous_corner_velocity": 1.0,
                "max_extrude_cross_section": 5.0
            },
            "extruder1": {
                "max_extrude_only_velocity": 30.0,
                "max_extrude_only_accel": 1300.0,
                "instantaneous_corner_velocity": 1.5
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
                "square_corner_velocity": 4.0
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
        assert_ne!(defaults.fingerprint, runtime.fingerprint);
        defaults.validate().unwrap();
        runtime.validate().unwrap();
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
