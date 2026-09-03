const FORK_SUFFIX: &str = "-ste94pz";

fn fork_version(version: &str) -> String {
    let version = version.strip_prefix('v').unwrap_or(version);
    let suffix = if version.ends_with(FORK_SUFFIX) {
        ""
    } else {
        FORK_SUFFIX
    };
    format!("v{version}{suffix}")
}

fn main() {
    println!("cargo:rerun-if-env-changed=TOOL_VERSION");

    let version =
        std::env::var("TOOL_VERSION").unwrap_or_else(|_| env!("CARGO_PKG_VERSION").into());
    let version = fork_version(&version);

    println!("cargo:rustc-env=TOOL_VERSION={version}");
}
