use std::collections::{BTreeMap, HashMap};
use std::io::Read;
use std::time::{SystemTime, UNIX_EPOCH};

use reqwest::blocking::Client;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use url::Url;

use crate::duration::DurationEstimate;

pub const CALIBRATION_MARKER_PREFIX: &str = "; klipper_estimator calibration: ";
const CALIBRATION_MARKER_SCHEMA_VERSION: u32 = 1;
const CALIBRATION_MODEL_VERSION: u32 = 1;
const PAUSE_TOLERANCE_SECONDS: f64 = 1.0;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationMarker {
    pub schema_version: u32,
    pub estimator_version: String,
    pub configuration_fingerprint: String,
    pub gcode_fingerprint: String,
    pub baseline_expected_time: f64,
}

impl CalibrationMarker {
    pub fn new(
        configuration_fingerprint: String,
        gcode_fingerprint: String,
        baseline_expected_time: f64,
    ) -> Self {
        Self {
            schema_version: CALIBRATION_MARKER_SCHEMA_VERSION,
            estimator_version: env!("TOOL_VERSION").into(),
            configuration_fingerprint,
            gcode_fingerprint,
            baseline_expected_time,
        }
    }

    pub fn to_comment(&self) -> String {
        format!(
            "{CALIBRATION_MARKER_PREFIX}{}",
            serde_json::to_string(self).expect("calibration marker must serialize")
        )
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationStatus {
    Applied,
    InsufficientData,
    Unavailable,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CalibrationApplicability {
    pub configuration_fingerprint: String,
    pub baseline: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CalibrationErrorMetrics {
    pub median_absolute_error: f64,
    pub root_mean_squared_error: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ResidualInterval {
    pub confidence: f64,
    pub lower: f64,
    pub upper: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct HistoryCalibrationModel {
    pub model_version: u32,
    pub sample_count: usize,
    pub generated_at_unix_seconds: u64,
    pub newest_sample_age_seconds: u64,
    pub oldest_sample_age_seconds: u64,
    pub applicability: CalibrationApplicability,
    pub median_residual: f64,
    pub error_metrics: CalibrationErrorMetrics,
    pub residual_interval: ResidualInterval,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CalibratedPrediction {
    pub expected_total_time: f64,
    pub confidence: f64,
    pub lower: f64,
    pub upper: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CalibrationReport {
    pub status: CalibrationStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    pub rejected_samples: BTreeMap<String, usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<HistoryCalibrationModel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub prediction: Option<CalibratedPrediction>,
}

impl CalibrationReport {
    pub(crate) fn unavailable(reason: impl Into<String>) -> Self {
        Self {
            status: CalibrationStatus::Unavailable,
            reason: Some(reason.into()),
            rejected_samples: BTreeMap::new(),
            model: None,
            prediction: None,
        }
    }

    pub fn apply(&mut self, estimate: &mut DurationEstimate) {
        let model = match &self.model {
            Some(model) => model,
            None => return,
        };
        let uncalibrated = estimate.expected_total_time;
        let calibrated = (uncalibrated + model.median_residual).max(estimate.deterministic_time);
        let adjustment = calibrated - uncalibrated;
        estimate.add_expected("history_calibration", adjustment);
        self.prediction = Some(CalibratedPrediction {
            expected_total_time: calibrated,
            confidence: model.residual_interval.confidence,
            lower: (uncalibrated + model.residual_interval.lower).max(estimate.deterministic_time),
            upper: (uncalibrated + model.residual_interval.upper).max(estimate.deterministic_time),
        });
    }
}

#[derive(Debug, Deserialize)]
struct MoonrakerRoot<T> {
    result: T,
}

#[derive(Debug, Deserialize)]
struct HistoryList {
    jobs: Vec<HistoryJob>,
}

#[derive(Debug, Clone, Deserialize)]
struct HistoryJob {
    exists: bool,
    status: String,
    filename: String,
    end_time: Option<f64>,
    print_duration: f64,
    total_duration: f64,
    #[serde(default)]
    metadata: HistoryMetadata,
}

#[derive(Debug, Clone, Default, Deserialize)]
struct HistoryMetadata {
    #[serde(default)]
    uuid: String,
    #[serde(default)]
    file_processors: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
struct CalibrationSample {
    residual: f64,
    end_time: u64,
}

pub fn fetch_history_calibration(
    source_url: &str,
    api_key: Option<&str>,
    configuration_fingerprint: &str,
    limit: usize,
    max_age_seconds: u64,
    min_samples: usize,
) -> CalibrationReport {
    let now = now_unix_seconds();
    match fetch_history_calibration_inner(
        source_url,
        api_key,
        configuration_fingerprint,
        limit,
        max_age_seconds,
        min_samples,
        now,
    ) {
        Ok(report) => report,
        Err(error) => CalibrationReport::unavailable(error),
    }
}

fn fetch_history_calibration_inner(
    source_url: &str,
    api_key: Option<&str>,
    configuration_fingerprint: &str,
    limit: usize,
    max_age_seconds: u64,
    min_samples: usize,
    now: u64,
) -> Result<CalibrationReport, String> {
    let client = Client::new();
    let mut history_url = endpoint(source_url, &["server", "history", "list"])?;
    history_url
        .query_pairs_mut()
        .append_pair("limit", &limit.to_string())
        .append_pair("order", "desc");
    let response = send_get(&client, history_url, api_key)?
        .json::<MoonrakerRoot<HistoryList>>()
        .map_err(|error| format!("could not parse Moonraker history: {error}"))?;

    Ok(build_history_calibration(
        response.result.jobs,
        configuration_fingerprint,
        max_age_seconds,
        min_samples,
        now,
        |job| fetch_marker(&client, source_url, api_key, &job.filename),
    ))
}

fn build_history_calibration(
    jobs: Vec<HistoryJob>,
    configuration_fingerprint: &str,
    max_age_seconds: u64,
    min_samples: usize,
    now: u64,
    mut marker_for_job: impl FnMut(&HistoryJob) -> Result<CalibrationMarker, String>,
) -> CalibrationReport {
    let mut file_cache = HashMap::<(String, String), Result<CalibrationMarker, String>>::new();
    let mut rejected = BTreeMap::new();
    let mut samples = Vec::new();
    for job in jobs {
        let preliminary_rejection = validate_job(&job, now, max_age_seconds);
        if let Some(reason) = preliminary_rejection {
            reject(&mut rejected, reason);
            continue;
        }

        let cache_key = (job.filename.clone(), job.metadata.uuid.clone());
        let marker = file_cache
            .entry(cache_key)
            .or_insert_with(|| marker_for_job(&job));
        let marker = match marker {
            Ok(marker) => marker,
            Err(_) => {
                reject(&mut rejected, "unverifiable_gcode");
                continue;
            }
        };
        if marker.configuration_fingerprint != configuration_fingerprint {
            reject(&mut rejected, "configuration_mismatch");
            continue;
        }
        if !marker.baseline_expected_time.is_finite() || marker.baseline_expected_time <= 0.0 {
            reject(&mut rejected, "invalid_baseline");
            continue;
        }
        let residual = job.total_duration - marker.baseline_expected_time;
        if residual.abs() > marker.baseline_expected_time.max(600.0) {
            reject(&mut rejected, "residual_outlier");
            continue;
        }
        samples.push(CalibrationSample {
            residual,
            end_time: job.end_time.unwrap_or_default() as u64,
        });
    }

    fit_model(
        samples,
        rejected,
        configuration_fingerprint,
        min_samples,
        now,
    )
}

fn validate_job(job: &HistoryJob, now: u64, max_age_seconds: u64) -> Option<&'static str> {
    if job.status != "completed" {
        return Some("incomplete_status");
    }
    if !job.exists || job.metadata.uuid.is_empty() {
        return Some("modified_or_missing_gcode");
    }
    if !job
        .metadata
        .file_processors
        .iter()
        .any(|processor| processor == "klipper_estimator")
    {
        return Some("not_estimator_processed");
    }
    let end_time = match job.end_time {
        Some(end_time) => end_time,
        None => return Some("invalid_end_time"),
    };
    if !end_time.is_finite() || end_time < 0.0 || end_time > now as f64 {
        return Some("invalid_end_time");
    }
    if now.saturating_sub(end_time as u64) > max_age_seconds {
        return Some("stale");
    }
    if !job.total_duration.is_finite()
        || !job.print_duration.is_finite()
        || job.total_duration <= 0.0
        || job.print_duration <= 0.0
    {
        return Some("invalid_duration");
    }
    if (job.total_duration - job.print_duration).abs() > PAUSE_TOLERANCE_SECONDS {
        return Some("possible_pause");
    }
    None
}

fn fit_model(
    mut samples: Vec<CalibrationSample>,
    rejected_samples: BTreeMap<String, usize>,
    configuration_fingerprint: &str,
    min_samples: usize,
    now: u64,
) -> CalibrationReport {
    if samples.len() < min_samples {
        return CalibrationReport {
            status: CalibrationStatus::InsufficientData,
            reason: Some(format!(
                "history supplied {} verified samples; at least {min_samples} are required",
                samples.len()
            )),
            rejected_samples,
            model: None,
            prediction: None,
        };
    }

    samples.sort_by(|left, right| {
        left.residual
            .partial_cmp(&right.residual)
            .expect("validated residuals must be finite")
    });
    let residuals = samples
        .iter()
        .map(|sample| sample.residual)
        .collect::<Vec<_>>();
    let median = quantile(&residuals, 0.5);
    let absolute_errors = residuals
        .iter()
        .map(|residual| (residual - median).abs())
        .collect::<Vec<_>>();
    let squared_error_sum = residuals
        .iter()
        .map(|residual| (residual - median).powi(2))
        .sum::<f64>();
    let newest = samples.iter().map(|sample| sample.end_time).max().unwrap();
    let oldest = samples.iter().map(|sample| sample.end_time).min().unwrap();

    CalibrationReport {
        status: CalibrationStatus::Applied,
        reason: None,
        rejected_samples,
        model: Some(HistoryCalibrationModel {
            model_version: CALIBRATION_MODEL_VERSION,
            sample_count: samples.len(),
            generated_at_unix_seconds: now,
            newest_sample_age_seconds: now.saturating_sub(newest),
            oldest_sample_age_seconds: now.saturating_sub(oldest),
            applicability: CalibrationApplicability {
                configuration_fingerprint: configuration_fingerprint.into(),
                baseline: "expected_total_time_before_history_calibration".into(),
            },
            median_residual: median,
            error_metrics: CalibrationErrorMetrics {
                median_absolute_error: {
                    let mut errors = absolute_errors;
                    errors.sort_by(|left, right| {
                        left.partial_cmp(right).expect("errors must be finite")
                    });
                    quantile(&errors, 0.5)
                },
                root_mean_squared_error: (squared_error_sum / residuals.len() as f64).sqrt(),
            },
            residual_interval: ResidualInterval {
                confidence: 0.8,
                lower: quantile(&residuals, 0.1),
                upper: quantile(&residuals, 0.9),
            },
        }),
        prediction: None,
    }
}

fn quantile(sorted: &[f64], quantile: f64) -> f64 {
    let position = quantile * (sorted.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    let fraction = position - lower as f64;
    sorted[lower] + (sorted[upper] - sorted[lower]) * fraction
}

fn fetch_marker(
    client: &Client,
    source_url: &str,
    api_key: Option<&str>,
    filename: &str,
) -> Result<CalibrationMarker, String> {
    let segments = filename
        .split('/')
        .filter(|segment| !segment.is_empty())
        .collect::<Vec<_>>();
    if segments.is_empty()
        || segments
            .iter()
            .any(|segment| *segment == "." || *segment == "..")
    {
        return Err("history contains an invalid G-code path".into());
    }
    let mut all_segments = vec!["server", "files", "gcodes"];
    all_segments.extend(segments);
    let url = endpoint(source_url, &all_segments)?;
    let bytes = send_get(client, url, api_key)?
        .bytes()
        .map_err(|error| format!("could not read history G-code: {error}"))?;
    parse_verified_marker(&bytes)
}

fn parse_verified_marker(bytes: &[u8]) -> Result<CalibrationMarker, String> {
    let text = std::str::from_utf8(bytes).map_err(|_| "history G-code is not UTF-8")?;
    let marker_start = text
        .rfind(CALIBRATION_MARKER_PREFIX)
        .ok_or("history G-code has no calibration marker")?;
    if marker_start != 0 && bytes[marker_start - 1] != b'\n' {
        return Err("calibration marker is not on its own line".into());
    }
    let marker_line = text[marker_start..].trim_end_matches(['\r', '\n']);
    if marker_line.contains('\n') || marker_line.contains('\r') {
        return Err("calibration marker is not the final G-code line".into());
    }
    let marker: CalibrationMarker =
        serde_json::from_str(&marker_line[CALIBRATION_MARKER_PREFIX.len()..])
            .map_err(|error| format!("invalid calibration marker: {error}"))?;
    if marker.schema_version != CALIBRATION_MARKER_SCHEMA_VERSION {
        return Err("unsupported calibration marker schema".into());
    }
    let actual = format!("{:x}", Sha256::digest(&bytes[..marker_start]));
    if marker.gcode_fingerprint != actual {
        return Err("calibration marker G-code fingerprint mismatch".into());
    }
    Ok(marker)
}

fn send_get(
    client: &Client,
    url: Url,
    api_key: Option<&str>,
) -> Result<reqwest::blocking::Response, String> {
    let mut request = client.get(url);
    if let Some(api_key) = api_key {
        request = request.header("X-Api-Key", api_key);
    }
    request
        .send()
        .and_then(reqwest::blocking::Response::error_for_status)
        .map_err(|error| format!("Moonraker history request failed: {error}"))
}

fn endpoint(source_url: &str, segments: &[&str]) -> Result<Url, String> {
    let mut url =
        Url::parse(source_url).map_err(|error| format!("invalid Moonraker URL: {error}"))?;
    url.set_query(None);
    let mut path = url
        .path_segments_mut()
        .map_err(|_| "Moonraker URL cannot be a base URL")?;
    path.pop_if_empty();
    path.extend(segments);
    drop(path);
    Ok(url)
}

pub fn fingerprint_reader(mut reader: impl Read) -> std::io::Result<String> {
    let mut digest = Sha256::new();
    std::io::copy(&mut reader, &mut DigestWriter(&mut digest))?;
    Ok(format!("{:x}", digest.finalize()))
}

struct DigestWriter<'a>(&'a mut Sha256);

impl std::io::Write for DigestWriter<'_> {
    fn write(&mut self, buffer: &[u8]) -> std::io::Result<usize> {
        self.0.update(buffer);
        Ok(buffer.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

fn reject(rejected: &mut BTreeMap<String, usize>, reason: &str) {
    *rejected.entry(reason.into()).or_default() += 1;
}

fn now_unix_seconds() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn job(status: &str, end_time: f64, print: f64, total: f64) -> HistoryJob {
        HistoryJob {
            exists: true,
            status: status.into(),
            filename: "cube.gcode".into(),
            end_time: Some(end_time),
            print_duration: print,
            total_duration: total,
            metadata: HistoryMetadata {
                uuid: "file-1".into(),
                file_processors: vec!["klipper_estimator".into()],
            },
        }
    }

    #[test]
    fn marker_verifies_the_exact_preceding_bytes() {
        let body = b"G1 X10\n";
        let marker =
            CalibrationMarker::new("config".into(), format!("{:x}", Sha256::digest(body)), 10.0);
        let contents = format!(
            "{}{}\n",
            std::str::from_utf8(body).unwrap(),
            marker.to_comment()
        );
        assert_eq!(parse_verified_marker(contents.as_bytes()).unwrap(), marker);

        let changed = contents.replacen("X10", "X11", 1);
        assert!(parse_verified_marker(changed.as_bytes()).is_err());
    }

    #[test]
    fn rejects_incomplete_stale_and_possibly_paused_jobs() {
        assert_eq!(
            validate_job(&job("cancelled", 900.0, 10.0, 10.0), 1_000, 200),
            Some("incomplete_status")
        );
        assert_eq!(
            validate_job(&job("completed", 700.0, 10.0, 10.0), 1_000, 200),
            Some("stale")
        );
        assert_eq!(
            validate_job(&job("completed", 900.0, 10.0, 12.0), 1_000, 200),
            Some("possible_pause")
        );
    }

    #[test]
    fn robust_model_is_reproducible_and_reports_error() {
        let samples = vec![
            CalibrationSample {
                residual: 9.0,
                end_time: 900,
            },
            CalibrationSample {
                residual: 10.0,
                end_time: 850,
            },
            CalibrationSample {
                residual: 11.0,
                end_time: 800,
            },
            CalibrationSample {
                residual: 100.0,
                end_time: 750,
            },
        ];
        let report = fit_model(samples, BTreeMap::new(), "config", 3, 1_000);
        let model = report.model.unwrap();
        assert_eq!(model.sample_count, 4);
        assert_eq!(model.median_residual, 10.5);
        assert_eq!(model.newest_sample_age_seconds, 100);
        assert_eq!(model.oldest_sample_age_seconds, 250);
        assert_eq!(model.residual_interval.lower, 9.3);
        assert!((model.residual_interval.upper - 73.3).abs() < 1e-9);
        assert_eq!(model.error_metrics.median_absolute_error, 1.0);
    }

    #[test]
    fn fixed_history_fixture_filters_before_fitting() {
        let history: MoonrakerRoot<HistoryList> =
            serde_json::from_str(include_str!("../../tests/fixtures/history/jobs.json")).unwrap();
        let report =
            build_history_calibration(history.result.jobs, "config-a", 500, 3, 1_000, |job| {
                Ok(CalibrationMarker::new(
                    if job.filename == "wrong-config.gcode" {
                        "config-b"
                    } else {
                        "config-a"
                    }
                    .into(),
                    "fixture-hash".into(),
                    100.0,
                ))
            });

        let model = report.model.unwrap();
        assert_eq!(model.sample_count, 3);
        assert_eq!(model.median_residual, 10.0);
        assert_eq!(report.rejected_samples["incomplete_status"], 1);
        assert_eq!(report.rejected_samples["stale"], 1);
        assert_eq!(report.rejected_samples["modified_or_missing_gcode"], 1);
        assert_eq!(report.rejected_samples["configuration_mismatch"], 1);
        assert_eq!(report.rejected_samples["possible_pause"], 1);
    }

    #[test]
    fn calibration_keeps_deterministic_time_available() {
        let mut report = fit_model(
            vec![
                CalibrationSample {
                    residual: 8.0,
                    end_time: 900,
                },
                CalibrationSample {
                    residual: 10.0,
                    end_time: 850,
                },
                CalibrationSample {
                    residual: 12.0,
                    end_time: 800,
                },
            ],
            BTreeMap::new(),
            "config",
            3,
            1_000,
        );
        let mut estimate = DurationEstimate::default();
        estimate.add_deterministic("motion", 100.0);
        report.apply(&mut estimate);
        assert_eq!(estimate.deterministic_time, 100.0);
        assert_eq!(estimate.expected_total_time, 110.0);
        assert_eq!(report.prediction.unwrap().lower, 108.4);
    }
}
