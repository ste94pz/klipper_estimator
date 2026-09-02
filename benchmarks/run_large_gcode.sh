#!/usr/bin/env bash
set -euo pipefail

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
move_count=${1:-500000}
case "$move_count" in
    ''|*[!0-9]*)
        echo "move count must be a positive integer" >&2
        exit 2
        ;;
esac
if (( move_count < 1 )); then
    echo "move count must be a positive integer" >&2
    exit 2
fi

temporary_directory=$(mktemp -d)
gcode_file="$temporary_directory/representative-large.gcode"
cleanup() {
    rm -f -- "$gcode_file"
    rmdir -- "$temporary_directory"
}
trap cleanup EXIT

awk -v count="$move_count" 'BEGIN {
    print "G90"
    print "M83"
    print "G1 X1 Y1 F12000"
    for (i = 0; i < count; i++) {
        x = (i % 2) ? 1 : 199
        y = 1 + (i % 199)
        printf "G1 X%d Y%d E0.02 F12000\n", x, y
    }
}' > "$gcode_file"

cargo build --manifest-path "$repository_root/Cargo.toml" --release -p klipper_estimator
echo "case=representative-large moves=$move_count bytes=$(wc -c < "$gcode_file")"
/usr/bin/time -v "$repository_root/target/release/klipper_estimator" \
    estimate --format json --omit-move-kinds --omit-layer-times "$gcode_file" \
    >/dev/null
