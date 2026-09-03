use clap::Parser;

use crate::Opts;

#[derive(Parser, Debug)]
pub struct DumpConfigCmd;

impl DumpConfigCmd {
    pub fn run(&self, opts: &Opts) -> anyhow::Result<()> {
        serde_json::to_writer_pretty(std::io::stdout(), opts.config_snapshot()?)?;
        Ok(())
    }
}
