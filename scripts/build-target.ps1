# Cross-builds the `drlabs` binary for a robot PC from this Windows machine.
#
#   .\scripts\build-target.ps1                       # aarch64 Linux, rt profile
#   .\scripts\build-target.ps1 -Target x86_64-unknown-linux-gnu
#   .\scripts\build-target.ps1 -Profile release      # keep symbols
#
# Uses cargo-zigbuild (zig as the cross linker; no Docker, no sysroot).
# One-time setup:
#   cargo install cargo-zigbuild
#   winget install zig.zig        (or put zig on PATH some other way)
#   rustup target add aarch64-unknown-linux-gnu x86_64-unknown-linux-gnu
#
# Per-target CPU tuning is read from .cargo/config.toml. Building *on* the
# robot with `cargo build --profile rt` is simpler and is the recommended
# path when the robot can build; this script is for when it cannot.

param(
    [string]$Target = "aarch64-unknown-linux-gnu",
    [string]$Profile = "rt"
)

$ErrorActionPreference = "Stop"

if (-not (Get-Command cargo-zigbuild -ErrorAction SilentlyContinue)) {
    Write-Error "cargo-zigbuild is not installed: cargo install cargo-zigbuild"
}
if (-not (Get-Command zig -ErrorAction SilentlyContinue)) {
    Write-Error "zig is not on PATH: winget install zig.zig"
}

$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    cargo zigbuild --target $Target --profile $Profile -p drlabs-cli
    if ($LASTEXITCODE -ne 0) { throw "build failed" }

    $dist = Join-Path $root "dist\$Target"
    New-Item -ItemType Directory -Force $dist | Out-Null
    Copy-Item (Join-Path $root "target\$Target\$Profile\drlabs") (Join-Path $dist "drlabs") -Force
    Write-Host "built $dist\drlabs ($Target, $Profile)"
}
finally {
    Pop-Location
}
