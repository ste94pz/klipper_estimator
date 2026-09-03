use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Output, Stdio};

fn binary() -> Command {
    Command::new(env!("CARGO_BIN_EXE_klipper_estimator"))
}

fn run_with_stdin(args: &[&str], input: &[u8]) -> Output {
    let mut child = binary()
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child.stdin.take().unwrap().write_all(input).unwrap();
    child.wait_with_output().unwrap()
}

fn missing_path() -> PathBuf {
    std::env::temp_dir().join(format!(
        "klipper-estimator-missing-{}-{}.gcode",
        std::process::id(),
        std::thread::current().name().unwrap_or("test")
    ))
}

fn assert_clean_error(output: &Output, expected: &str) {
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!output.status.success());
    assert!(stderr.contains(expected), "unexpected stderr: {stderr}");
    assert!(
        !stderr.contains("panicked at"),
        "unexpected panic: {stderr}"
    );
}

#[test]
fn missing_gcode_files_are_reported_without_panicking() {
    let path = missing_path();
    let path = path.to_str().unwrap();

    for subcommand in ["estimate", "dump-moves", "post-process"] {
        let output = binary().args([subcommand, path]).output().unwrap();
        assert_clean_error(&output, "failed to open G-code file");
    }
}

#[test]
fn malformed_or_unreadable_gcode_is_reported_with_a_line_number() {
    for input in [&b"?\n"[..], &[0xff, b'\n'][..]] {
        let output = run_with_stdin(&["estimate", "-"], input);
        assert_clean_error(&output, "at line 1");
    }
}

#[test]
fn non_positive_feedrate_is_a_diagnostic_instead_of_a_panic() {
    let output = run_with_stdin(&["estimate", "-", "--format", "json"], b"G1 X1 F0\n");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "unexpected stderr: {stderr}");
    assert!(!stderr.contains("panicked at"));

    let json: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(json["metadata"]["accuracy_class"], "lower_bound");
    assert_eq!(json["diagnostics"][0]["code"], "invalid_move_speed");
}
