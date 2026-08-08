#!/bin/bash
# Builds mini_visor for real Raspberry Pi 4 hardware (linked with
# scripts/pi4.ld) and copies the result to bin-pi4/disk/mini.elf.
#
# The linker script is injected via `--config` (and .cargo/config.toml must
# NOT define any rustflags): cargo *joins* `rustflags` arrays across config
# sources — config files, CARGO_TARGET_<TRIPLE>_RUSTFLAGS, and `--config`
# alike — instead of replacing them, so the only way to guarantee exactly
# one -T script reaches the linker is to keep it as the single source.
set -e
cd "$(dirname "$0")/.."

cargo run \
    --config 'target.aarch64-unknown-none-softfloat.runner="tools-pi4/copy_minivisor.sh"' \
    --config 'target.aarch64-unknown-none-softfloat.rustflags=["-C", "link-arg=-Tscripts/pi4.ld"]' \
    "$@"
