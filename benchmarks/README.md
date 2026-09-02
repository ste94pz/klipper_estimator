# Large-file benchmark

`run_large_gcode.sh` generates a deterministic Cartesian-style G-code stream
and measures the release binary with GNU `time`. It records wall time, peak
resident memory, input byte size, and move count without storing a large
fixture in Git.

Run the default 500,000-move case with:

```sh
bash benchmarks/run_large_gcode.sh
```

The optional first argument changes the move count. Compare results only on the
same machine, toolchain, build profile, and move count. The first baseline for
the current implementation is recorded below after running the script.

| Date | Revision | Host | Rust | Moves | Input | Wall time | Peak RSS |
|---|---|---|---|---:|---:|---:|---:|
| 2026-09-02 | `a85b8cd` + Task 12 working tree | x86_64 development host | rustc 1.95.0 | 500,000 | 12,228,620 B | 0.71 s | 6,764 KiB |
