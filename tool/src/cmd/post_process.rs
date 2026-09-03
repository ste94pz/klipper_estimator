use std::collections::VecDeque;
use std::ffi::OsString;
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

use clap::Parser;

use lib_klipper::gcode::{
    GCodeCommand, GCodeOperation, GCodeReader, GCodeTraditionalParams, parse_gcode,
};
use lib_klipper::planner::Planner;
use lib_klipper::slicer::SlicerPreset;

use crate::Opts;
use crate::calibration::{CALIBRATION_MARKER_PREFIX, CalibrationMarker, fingerprint_reader};
use crate::duration::DurationEstimate;

#[derive(Parser, Debug)]
pub struct PostProcessCmd {
    #[clap(parse(try_from_str))]
    filename: PathBuf,
}

trait GCodeInterceptor: std::fmt::Debug {
    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        let _ = command;
        let _ = result;
    }

    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        let _ = command;
        let _ = result;
        None
    }
}

#[derive(Debug, Default)]
struct NoopGCodeInterceptor {}

impl GCodeInterceptor for NoopGCodeInterceptor {}

#[derive(Debug, Default)]
struct M73GcodeInterceptor {
    time_buffer: VecDeque<f64>,
}

impl GCodeInterceptor for M73GcodeInterceptor {
    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        if matches!(
            command.op,
            GCodeOperation::Traditional {
                letter: 'M',
                code: 73,
                ..
            }
        ) {
            self.time_buffer.push_back(result.total_time);
        }
    }

    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        if !matches!(
            command.op,
            GCodeOperation::Traditional {
                letter: 'M',
                code: 73,
                ..
            }
        ) {
            return None;
        }
        let next = self.time_buffer.pop_front()?;
        let params = vec![
            ('P', format!("{:.3}", (next / result.total_time * 100.0))),
            (
                'R',
                format!("{}", ((result.total_time - next) / 60.0).round()),
            ),
        ];
        Some(GCodeCommand {
            op: GCodeOperation::Traditional {
                letter: 'M',
                code: 73,
                params: GCodeTraditionalParams::from_vec(params),
            },
            comment: None,
        })
    }
}

#[derive(Debug, Default)]
struct PSSSGCodeInterceptor {
    m73_interceptor: M73GcodeInterceptor,
}

impl PSSSGCodeInterceptor {
    fn format_dhms(mut time: f64) -> String {
        use std::fmt::Write;
        let mut out = String::new();
        time = time.ceil();
        let d = (time / 86400.0).floor();
        if d > 0.0 {
            write!(out, " {:.0}d", d).unwrap();
        }
        time %= 86400.0;
        let h = (time / 3600.0).floor();
        if h > 0.0 {
            write!(out, " {:.0}h", h).unwrap();
        }
        time %= 3600.0;
        let m = (time / 60.0).floor();
        if m > 0.0 {
            write!(out, " {:.0}m", m).unwrap();
        }
        time %= 60.0;
        let s = time;
        write!(out, " {:.0}s", s).unwrap();
        out
    }
}

impl GCodeInterceptor for PSSSGCodeInterceptor {
    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        self.m73_interceptor.post_command(command, result);
    }

    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        if let Some(cmd) = self.m73_interceptor.output_process(command, result) {
            return Some(cmd);
        }

        if let Some(com) = &command.comment
            && let Some(end) = com
                .strip_prefix(" estimated printing time (")
                .and_then(|rest| rest.find(") ="))
        {
            let prefix_end = " estimated printing time (".len() + end + ") =".len();
            return Some(GCodeCommand {
                op: GCodeOperation::Nop,
                comment: Some(format!(
                    "{}{}",
                    &com[..prefix_end],
                    Self::format_dhms(result.total_time)
                )),
            });
        }

        None
    }
}

#[derive(Debug, Default)]
struct IdeaMakerGCodeInterceptor {
    time_buffer: VecDeque<f64>,
}

impl GCodeInterceptor for IdeaMakerGCodeInterceptor {
    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        if let Some(com) = &command.comment
            && com.starts_with("PRINTING_TIME: ")
        {
            self.time_buffer.push_back(result.total_time);
        }
    }

    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        if let Some(com) = &command.comment {
            if com.starts_with("Print Time: ") {
                return Some(GCodeCommand {
                    op: GCodeOperation::Nop,
                    comment: Some(format!("Print Time: {:.0}", result.total_time.ceil())),
                });
            } else if com.starts_with("PRINTING_TIME: ") {
                if let Some(next) = self.time_buffer.front() {
                    return Some(GCodeCommand {
                        op: GCodeOperation::Nop,
                        comment: Some(format!("PRINTING_TIME: {:.0}", next.ceil())),
                    });
                }
            } else if com.starts_with("REMAINING_TIME: ")
                && let Some(next) = self.time_buffer.pop_front()
            {
                return Some(GCodeCommand {
                    op: GCodeOperation::Nop,
                    comment: Some(format!(
                        "REMAINING_TIME: {:.0}",
                        (result.total_time - next).ceil()
                    )),
                });
            }
        }
        None
    }
}

#[derive(Debug, Default)]
struct CuraGCodeInterceptor {
    time_buffer: VecDeque<f64>,
}

impl GCodeInterceptor for CuraGCodeInterceptor {
    fn post_command(&mut self, command: &GCodeCommand, result: &mut PostProcessEstimationResult) {
        if let Some(com) = &command.comment
            && com.starts_with("TIME_ELAPSED:")
        {
            self.time_buffer.push_back(result.total_time);
        }
    }

    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        if let Some(com) = &command.comment {
            if com.starts_with("TIME:") {
                return Some(GCodeCommand {
                    op: GCodeOperation::Nop,
                    comment: Some(format!("TIME:{:.0}", result.total_time.ceil())),
                });
            } else if com.starts_with("PRINT.TIME:") {
                return Some(GCodeCommand {
                    op: GCodeOperation::Nop,
                    comment: Some(format!("PRINT.TIME:{:.0}", result.total_time.ceil())),
                });
            } else if com.starts_with("TIME_ELAPSED:")
                && let Some(next) = self.time_buffer.pop_front()
            {
                return Some(GCodeCommand {
                    op: GCodeOperation::Nop,
                    comment: Some(format!("TIME_ELAPSED:{:.0}", (next).ceil())),
                });
            }
        }
        None
    }
}

#[derive(Debug, Default)]
struct Simplify3DGCodeInterceptor {}

impl Simplify3DGCodeInterceptor {
    fn format_dhms(mut time: f64) -> String {
        use std::fmt::Write;
        let mut out = String::new();
        time = time.ceil();
        let h = (time / 3600.0).floor();
        if h > 0.0 {
            write!(out, " {:.0} hours", h).unwrap();
        }
        time %= 3600.0;
        let m = (time / 60.0).floor();
        if m > 0.0 {
            write!(out, " {:.0} minutes", m).unwrap();
        }
        time %= 60.0;
        let s = time;
        write!(out, " {:.0} sec", s).unwrap();
        out
    }
}

impl GCodeInterceptor for Simplify3DGCodeInterceptor {
    fn output_process(
        &mut self,
        command: &GCodeCommand,
        result: &PostProcessEstimationResult,
    ) -> Option<GCodeCommand> {
        if let Some(com) = &command.comment
            && com.starts_with("   Build Time: ")
        {
            return Some(GCodeCommand {
                op: GCodeOperation::Nop,
                comment: Some(format!(
                    "   Build Time:{}",
                    Self::format_dhms(result.total_time.ceil())
                )),
            });
        }
        None
    }
}

fn metadata_processor(preset: &SlicerPreset) -> Box<dyn GCodeInterceptor> {
    match preset {
        SlicerPreset::PrusaSlicer { .. } => Box::<PSSSGCodeInterceptor>::default(),
        SlicerPreset::SuperSlicer { .. } => Box::<PSSSGCodeInterceptor>::default(),
        SlicerPreset::OrcaSlicer { .. } => Box::<PSSSGCodeInterceptor>::default(),
        SlicerPreset::IdeaMaker { .. } => Box::<IdeaMakerGCodeInterceptor>::default(),
        SlicerPreset::Cura { .. } => Box::<CuraGCodeInterceptor>::default(),
        SlicerPreset::Simplify3D { .. } => Box::<Simplify3DGCodeInterceptor>::default(),
    }
}

#[derive(Debug)]
struct PostProcessEstimationResult {
    total_time: f64,
    duration: DurationEstimate,
    slicer: Option<SlicerPreset>,
}

impl std::default::Default for PostProcessEstimationResult {
    fn default() -> Self {
        PostProcessEstimationResult {
            total_time: 0.0,
            duration: DurationEstimate::default(),
            slicer: None,
        }
    }
}

#[derive(Debug)]
struct PostProcessState {
    result: PostProcessEstimationResult,
    gcode_interceptor: Box<dyn GCodeInterceptor>,
}

#[allow(clippy::derivable_impls)]
impl std::default::Default for PostProcessState {
    fn default() -> Self {
        PostProcessState {
            result: PostProcessEstimationResult::default(),
            gcode_interceptor: Box::<NoopGCodeInterceptor>::default(),
        }
    }
}

#[derive(Debug)]
struct EstimateRunner {
    state: PostProcessState,
    planner: Planner,
    // We use this buffer to synchronize planned moves with input moves
    buffer: VecDeque<(usize, GCodeCommand)>,
}

impl EstimateRunner {
    fn run<T: BufRead>(&mut self, rdr: &mut GCodeReader<T>) {
        for (n, cmd) in rdr.enumerate() {
            let cmd = cmd.expect("gcode read");

            // If we don't have a slicer figured out yet, and this is a comment, try
            if cmd.op.is_nop()
                && self.state.result.slicer.is_none()
                && let Some(comment) = cmd.comment.as_ref()
            {
                self.state.result.slicer = SlicerPreset::determine(comment);
                if let Some(preset) = self.state.result.slicer.as_ref() {
                    self.state.gcode_interceptor = metadata_processor(preset);
                }
            }

            let x = self.planner.process_cmd(&cmd);
            self.buffer.push_back((x, cmd));

            if n % 1000 == 0 {
                self.flush();
            }
        }

        self.planner.finalize();
        self.flush();
        for diagnostic in self.planner.diagnostics() {
            eprintln!("{}: {}", diagnostic.command, diagnostic.message);
        }
        for omitted in &self.state.result.duration.omitted_duration_components {
            eprintln!(
                "{}: omitted {:?} duration ({})",
                omitted.command, omitted.category, omitted.reason
            );
        }
    }

    fn flush(&mut self) {
        for c in self.planner.iter().collect::<Vec<_>>() {
            let (n, cmd) = self.buffer.front_mut().unwrap();
            self.state.result.duration.add_operation(&self.planner, &c);
            self.state.result.total_time = self.state.result.duration.expected_total_time;
            self.state
                .gcode_interceptor
                .post_command(cmd, &mut self.state.result);
            if *n <= 1 {
                let _ = self.buffer.pop_front();
            } else {
                *n -= 1;
            }
        }
    }
}

impl PostProcessCmd {
    fn estimate(&self, opts: &Opts) -> PostProcessState {
        let src = File::open(&self.filename).expect("opening gcode file failed");
        let mut rdr = GCodeReader::new(BufReader::new(src));

        let mut runner = EstimateRunner {
            state: PostProcessState::default(),
            planner: opts.make_planner(),
            buffer: VecDeque::new(),
        };
        runner.run(&mut rdr);
        runner.state
    }

    fn apply_changes(&self, mut state: PostProcessState, configuration_fingerprint: &str) {
        let src = File::open(&self.filename).expect("opening gcode file failed");
        let rdr = BufReader::new(src);

        let mut dst_name = Into::<OsString>::into(".estimate.");
        dst_name.push(self.filename.file_name().expect("invalid file name"));
        let dst_path = self
            .filename
            .parent()
            .unwrap_or_else(|| Path::new("/"))
            .join(dst_name);
        let dst = File::create(&dst_path).expect("creating target gcode file failed");
        let mut wr = BufWriter::new(dst);

        for line in rdr.lines() {
            let line = line.expect("IO error");
            if line.starts_with(CALIBRATION_MARKER_PREFIX) {
                continue;
            }
            if let Ok(cmd) = parse_gcode(&line) {
                if let Some(cmd) = state.gcode_interceptor.output_process(&cmd, &state.result) {
                    writeln!(wr, "{}", cmd).expect("IO error");
                } else {
                    writeln!(wr, "{}", line).expect("IO error");
                }
            } else {
                writeln!(wr, "{}", line).expect("IO error");
            }
        }

        writeln!(
            wr,
            "; Processed by klipper_estimator {}, {}",
            env!("TOOL_VERSION"),
            if let Some(slicer) = state.result.slicer.as_ref() {
                format!("detected slicer {}", slicer)
            } else {
                "no slicer detected".into()
            }
        )
        .expect("IO error");

        // The marker hashes every preceding byte and is deliberately excluded from its own hash.
        wr.flush().expect("IO error");
        drop(wr);
        let gcode_fingerprint = fingerprint_reader(
            File::open(&dst_path).expect("opening processed G-code for hashing failed"),
        )
        .expect("hashing processed G-code failed");
        let marker = CalibrationMarker::new(
            configuration_fingerprint.into(),
            gcode_fingerprint,
            state.result.duration.expected_total_time,
        );
        let mut dst = OpenOptions::new()
            .append(true)
            .open(&dst_path)
            .expect("opening processed G-code for calibration marker failed");
        writeln!(dst, "{}", marker.to_comment()).expect("writing calibration marker failed");
        std::fs::rename(&dst_path, &self.filename).expect("rename failed");
    }

    pub fn run(&self, opts: &Opts) {
        let state = self.estimate(opts);
        let configuration_fingerprint = opts.config_snapshot().fingerprint.clone();
        self.apply_changes(state, &configuration_fingerprint);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lib_klipper::planner::PrinterLimits;
    use std::io::Cursor;

    #[test]
    fn post_process_legacy_total_tracks_expected_total() {
        let input = b"G1 X10 F600\n; ESTIMATOR_ADD_TIME 2 measured\n";
        let mut reader = GCodeReader::new(BufReader::new(Cursor::new(input)));
        let mut runner = EstimateRunner {
            state: PostProcessState::default(),
            planner: Planner::from_limits(PrinterLimits::default()),
            buffer: VecDeque::new(),
        };

        runner.run(&mut reader);

        assert_eq!(
            runner.state.result.total_time,
            runner.state.result.duration.expected_total_time
        );
        assert_eq!(
            runner.state.result.duration.expected_total_time,
            runner.state.result.duration.deterministic_time + 2.0
        );
    }

    #[test]
    fn legacy_cura_fields_use_expected_total_time() {
        let mut interceptor = CuraGCodeInterceptor::default();
        let mut duration = DurationEstimate::default();
        duration.motion_time = 30.0;
        duration.deterministic_time = 35.0;
        duration.expected_total_time = 42.4;
        duration.total_time = 42.4;
        let result = PostProcessEstimationResult {
            total_time: 42.4,
            duration,
            slicer: Some(SlicerPreset::Cura { version: None }),
        };

        let output = interceptor
            .output_process(&parse_gcode(";TIME:999").unwrap(), &result)
            .expect("Cura TIME field should be replaced");
        assert_eq!(output.comment.as_deref(), Some("TIME:43"));
    }

    #[test]
    fn prusaslicer_estimated_time_marker_is_rewritten_without_a_regex() {
        let mut interceptor = PSSSGCodeInterceptor::default();
        let result = PostProcessEstimationResult {
            total_time: 3661.0,
            ..Default::default()
        };

        let output = interceptor
            .output_process(
                &parse_gcode("; estimated printing time (normal mode) = 1s").unwrap(),
                &result,
            )
            .expect("PrusaSlicer estimated-time marker should be replaced");

        assert_eq!(
            output.comment.as_deref(),
            Some(" estimated printing time (normal mode) = 1h 1m 1s")
        );
    }
}
