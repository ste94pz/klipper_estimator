use std::collections::{BTreeMap, BTreeSet};
use std::fs;
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
}
