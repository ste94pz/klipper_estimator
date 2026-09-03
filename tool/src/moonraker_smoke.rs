#[cfg(test)]
mod tests {
    use std::env;

    use serde::{Deserialize, Serialize};

    use crate::calibration::{CalibrationReport, fetch_history_calibration_with_client};
    use crate::config_snapshot::{
        ConfigSnapshot, SnapshotAccuracy, SnapshotSelection, fetch_moonraker_snapshot_with_client,
    };
    use crate::moonraker::{ReadOnlyMoonrakerClient, RequestRecord};

    #[derive(Debug, Deserialize)]
    struct MoonrakerRoot<T> {
        result: T,
    }

    #[derive(Debug, Deserialize)]
    struct ServerInfo {
        moonraker_version: Option<String>,
        klippy_connected: Option<bool>,
        klippy_state: Option<String>,
    }

    #[derive(Debug, Serialize)]
    struct SnapshotReport {
        klipper_version: Option<String>,
        backend: String,
        accuracy: SnapshotAccuracy,
        fingerprint: String,
    }

    impl From<&ConfigSnapshot> for SnapshotReport {
        fn from(snapshot: &ConfigSnapshot) -> Self {
            Self {
                klipper_version: snapshot.klipper_version.clone(),
                backend: snapshot.limits.kinematics.backend_name().into(),
                accuracy: snapshot.accuracy,
                fingerprint: snapshot.fingerprint.clone(),
            }
        }
    }

    #[derive(Debug, Serialize)]
    struct SmokeReport {
        moonraker_version: Option<String>,
        klippy_connected: Option<bool>,
        klippy_state: Option<String>,
        configuration_default: SnapshotReport,
        configuration_default_fingerprint_stable: bool,
        runtime_snapshot: SnapshotReport,
        history: CalibrationReport,
        requests: Vec<RequestRecord>,
    }

    /// Opt-in real-printer smoke test. The transport used by every operation
    /// rejects non-read-only routes before network I/O.
    #[test]
    #[ignore = "requires MOONRAKER_URL and explicit access to a real Moonraker instance"]
    fn real_moonraker_read_only_smoke() {
        let source_url = env::var("MOONRAKER_URL").expect("MOONRAKER_URL must be set");
        let api_key = env::var("MOONRAKER_API_KEY").ok();
        let history_limit = env::var("MOONRAKER_HISTORY_LIMIT")
            .ok()
            .map(|value| {
                value
                    .parse::<usize>()
                    .expect("invalid MOONRAKER_HISTORY_LIMIT")
            })
            .unwrap_or(10);
        let client = ReadOnlyMoonrakerClient::new(&source_url, api_key.as_deref()).unwrap();

        let server = client
            .get(&["server", "info"])
            .unwrap()
            .json::<MoonrakerRoot<ServerInfo>>()
            .unwrap()
            .result;
        let configuration_default = fetch_moonraker_snapshot_with_client(
            &client,
            &source_url,
            SnapshotSelection::ConfigurationDefault,
        )
        .unwrap();
        let repeated_default = fetch_moonraker_snapshot_with_client(
            &client,
            &source_url,
            SnapshotSelection::ConfigurationDefault,
        )
        .unwrap();
        let runtime_snapshot = fetch_moonraker_snapshot_with_client(
            &client,
            &source_url,
            SnapshotSelection::RuntimeSnapshot,
        )
        .unwrap();
        let history = fetch_history_calibration_with_client(
            &client,
            &configuration_default.fingerprint,
            history_limit,
            90 * 24 * 60 * 60,
            1,
        );

        let report = SmokeReport {
            moonraker_version: server.moonraker_version,
            klippy_connected: server.klippy_connected,
            klippy_state: server.klippy_state,
            configuration_default: SnapshotReport::from(&configuration_default),
            configuration_default_fingerprint_stable: configuration_default.fingerprint
                == repeated_default.fingerprint,
            runtime_snapshot: SnapshotReport::from(&runtime_snapshot),
            history,
            requests: client.request_log(),
        };
        assert!(report.configuration_default_fingerprint_stable);
        println!("{}", serde_json::to_string_pretty(&report).unwrap());
    }
}
