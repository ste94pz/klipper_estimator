use anyhow::Context;
use lib_klipper::planner::{Planner, PrinterLimits};

use clap::Parser;
use config::{Value, ValueKind};
use once_cell::sync::OnceCell;
#[macro_use]
extern crate lazy_static;

mod calibration;
mod cmd;
mod config_snapshot;
mod duration;

use config_snapshot::{
    apply_kinematics_classification, apply_klipper_compatibility, fetch_moonraker_snapshot,
    load_offline_snapshot, map_auth_error, read_cache, write_cache, ConfigSnapshot,
    SnapshotAccuracy, SnapshotSelection, SnapshotSource, SnapshotSourceKind,
};

#[derive(Parser, Debug)]
#[clap(version = env!("TOOL_VERSION"), author = "Lasse Dalegaard <dalegaard@gmail.com>")]
pub struct Opts {
    #[clap(long = "config_moonraker_url")]
    config_moonraker: Option<String>,
    #[clap(long = "config_moonraker_api_key")]
    config_moonraker_api_key: Option<String>,
    #[clap(long = "config_moonraker_ignore_error")]
    config_moonraker_ignore_error: bool,
    #[clap(long = "config_moonraker_cache_file")]
    config_moonraker_cache_file: Option<String>,
    #[clap(
        long = "config_moonraker_mode",
        arg_enum,
        default_value_t = SnapshotSelection::ConfigurationDefault
    )]
    config_moonraker_mode: SnapshotSelection,

    /// Klipper root configuration, relative to --config_klipper_root
    #[clap(
        long = "config_klipper_file",
        requires = "config-klipper-root",
        conflicts_with = "config-moonraker"
    )]
    config_klipper_file: Option<String>,
    /// Directory that confines the root configuration and all of its includes
    #[clap(
        long = "config_klipper_root",
        requires = "config-klipper-file",
        conflicts_with = "config-moonraker"
    )]
    config_klipper_root: Option<String>,

    #[clap(long = "config_file")]
    config_filename: Option<String>,

    #[clap(short = 'c')]
    config_override: Vec<String>,

    #[clap(subcommand)]
    cmd: SubCommand,

    #[clap(skip)]
    config: OnceCell<ConfigSnapshot>,
}

impl Opts {
    fn moonraker_connection(&self) -> Option<(&str, Option<&str>)> {
        self.config_moonraker
            .as_deref()
            .map(|url| (url, self.config_moonraker_api_key.as_deref()))
    }
    fn printer_limits(&self) -> &PrinterLimits {
        &self.config_snapshot().limits
    }

    fn config_snapshot(&self) -> &ConfigSnapshot {
        match self.config.get() {
            Some(snapshot) => snapshot,
            None => match self.load_config() {
                Ok(snapshot) => {
                    let _ = self.config.set(snapshot);
                    self.config
                        .get()
                        .expect("configuration was just initialized")
                }
                Err(e) => {
                    eprintln!("Failed to load printer configuration: {}", e);
                    std::process::exit(1);
                }
            },
        }
    }

    fn opt_parse(s: &str) -> anyhow::Result<(&str, Value)> {
        let eqat = match s.find('=') {
            None => anyhow::bail!("invalid config override, format key=value"),
            Some(idx) => idx,
        };
        let key = &s[..eqat];
        let value = &s[eqat + 1..];
        let parser: fn(&str) -> anyhow::Result<ValueKind> = match key {
            "max_accel_to_decel" => |v: &str| Ok(ValueKind::Float(v.parse()?)),
            "minimum_cruise_ratio" => |v: &str| Ok(ValueKind::Float(v.parse()?)),
            _ => |v: &str| Ok(ValueKind::String(v.to_string())),
        };
        Ok((
            key,
            Value::new(
                None,
                parser(value)
                    .with_context(|| format!("failed to parse config override '{key}'"))?,
            ),
        ))
    }

    fn load_config(&self) -> anyhow::Result<ConfigSnapshot> {
        use config::Config;

        let mut snapshot = if let (Some(root), Some(filename)) =
            (&self.config_klipper_root, &self.config_klipper_file)
        {
            if self.config_moonraker_mode != SnapshotSelection::ConfigurationDefault {
                anyhow::bail!(
                    "offline Klipper configuration supports only configuration-default mode"
                );
            }
            load_offline_snapshot(root, filename)?
        } else if let Some(url) = &self.config_moonraker {
            match fetch_moonraker_snapshot(
                url,
                self.config_moonraker_api_key.as_deref(),
                self.config_moonraker_mode,
            ) {
                Ok(snapshot) => {
                    if let Some(cache_file) = self.config_moonraker_cache_file.as_deref() {
                        if let Err(error) = write_cache(cache_file, &snapshot) {
                            eprintln!("Could not write Moonraker cache: {error}");
                        }
                    }
                    snapshot
                }
                Err(error) if self.config_moonraker_ignore_error => {
                    let cache_file = self.config_moonraker_cache_file.as_deref().ok_or_else(|| {
                        anyhow::anyhow!(
                            "Moonraker is unavailable and no cache file was configured; refusing to use generic limits: {error}"
                        )
                    })?;
                    eprintln!("Could not get configuration from Moonraker: {error}");
                    read_cache(cache_file, self.config_moonraker_mode)?
                }
                Err(error) => {
                    if let Some(message) = map_auth_error(&error) {
                        anyhow::bail!(message);
                    }
                    return Err(error.into());
                }
            }
        } else {
            ConfigSnapshot::built_in_defaults()
        };

        let mut builder = Config::builder().add_source(config::File::from_str(
            &serde_json::to_string(&snapshot.limits)?,
            config::FileFormat::Json,
        ));

        if let Some(filename) = &self.config_filename {
            let file_config = Config::builder()
                .add_source(config::File::new(filename, config::FileFormat::Json5))
                .build()?;
            match file_config.clone().try_deserialize::<ConfigSnapshot>() {
                Ok(mut imported) => {
                    imported.validate()?;
                    imported.upgrade_legacy_extruders()?;
                    builder = builder.add_source(config::File::from_str(
                        &serde_json::to_string(&imported.limits)?,
                        config::FileFormat::Json,
                    ));
                    if self.config_moonraker.is_some() {
                        imported.source.kind = SnapshotSourceKind::Merged;
                        imported.source.location = Some(filename.clone());
                        imported.warnings.push(
                            "configuration file overrides the live Moonraker snapshot".into(),
                        );
                    }
                    snapshot = imported;
                }
                Err(_) => {
                    builder =
                        builder.add_source(config::File::new(filename, config::FileFormat::Json5));
                    snapshot.source = SnapshotSource {
                        kind: if self.config_moonraker.is_some() {
                            SnapshotSourceKind::Merged
                        } else {
                            SnapshotSourceKind::ConfigurationFile
                        },
                        location: Some(filename.clone()),
                        selection: snapshot.source.selection,
                    };
                    snapshot.accuracy = SnapshotAccuracy::Complete;
                    snapshot.warnings.retain(|warning| {
                        !warning.starts_with("using built-in generic printer limits")
                    });
                    if self.config_moonraker.is_some() {
                        snapshot.warnings.push(
                            "legacy JSON5 configuration overrides the live Moonraker snapshot"
                                .into(),
                        );
                    } else {
                        snapshot.warnings.push(
                            "legacy JSON5 configuration has no Klipper version or retrieval provenance"
                                .into(),
                        );
                    }
                }
            }
        }

        builder = self
            .config_override
            .iter()
            .try_fold(builder, |builder, opt| {
                let (k, v) = Self::opt_parse(opt)?;
                Ok::<_, anyhow::Error>(builder.set_override(k, v)?)
            })?;

        let mut limits = builder.build()?.try_deserialize::<PrinterLimits>()?;
        limits.recalculate();
        snapshot.limits = limits;
        apply_kinematics_classification(&mut snapshot);
        apply_klipper_compatibility(&mut snapshot);
        if !self.config_override.is_empty() {
            snapshot.source.kind = SnapshotSourceKind::Merged;
            snapshot
                .warnings
                .push("command-line configuration overrides are included in this snapshot".into());
        }
        snapshot.refresh_fingerprint();
        Ok(snapshot)
    }

    fn make_planner(&self) -> Planner {
        Planner::from_limits(self.printer_limits().clone())
    }
}

#[derive(Parser, Debug)]
enum SubCommand {
    Estimate(cmd::estimate::EstimateCmd),
    DumpMoves(cmd::estimate::DumpMovesCmd),
    PostProcess(cmd::post_process::PostProcessCmd),
    DumpConfig(cmd::dump_config::DumpConfigCmd),
}

impl SubCommand {
    fn run(&self, opts: &Opts) {
        match self {
            Self::Estimate(i) => i.run(opts),
            Self::DumpMoves(i) => i.run(opts),
            Self::PostProcess(i) => i.run(opts),
            Self::DumpConfig(i) => i.run(opts),
        }
    }
}

fn main() {
    let opts = Opts::parse();
    opts.cmd.run(&opts);
}
