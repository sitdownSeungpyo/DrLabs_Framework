#!/usr/bin/env bash
# Cross-builds the `drlabs` binary for a robot PC from a macOS or Linux host.
#
#   scripts/build-target.sh                                  # aarch64 Linux, rt profile
#   scripts/build-target.sh x86_64-unknown-linux-gnu
#   scripts/build-target.sh aarch64-unknown-linux-gnu release   # keep symbols
#
# Uses cargo-zigbuild (zig as the cross linker; no Docker, no sysroot).
# One-time setup:
#   cargo install cargo-zigbuild
#   brew install zig              (macOS; or your distribution's zig package)
#   rustup target add aarch64-unknown-linux-gnu x86_64-unknown-linux-gnu
#
# Per-target CPU tuning is read from .cargo/config.toml. Building *on* the
# robot with `cargo build --profile rt` is simpler and is the recommended
# path when the robot can build; this script is for when it cannot.
#
# The Windows twin is build-target.ps1; keep the two in step.

set -euo pipefail

target="${1:-aarch64-unknown-linux-gnu}"
profile="${2:-rt}"

if ! command -v cargo-zigbuild >/dev/null 2>&1; then
    echo "cargo-zigbuild is not installed: cargo install cargo-zigbuild" >&2
    exit 1
fi
if ! command -v zig >/dev/null 2>&1; then
    echo "zig is not on PATH: brew install zig" >&2
    exit 1
fi

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

cargo zigbuild --target "$target" --profile "$profile" -p drlabs-cli

dist="$root/dist/$target"
mkdir -p "$dist"
cp -f "$root/target/$target/$profile/drlabs" "$dist/drlabs"
echo "built $dist/drlabs ($target, $profile)"
