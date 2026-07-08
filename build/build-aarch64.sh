#!/bin/sh
# Build ros2dreame as a FULLY STATIC aarch64 musl binary for the robot.
#
# Cross-links with Rust's own bundled rust-lld (a multi-flavor lld), so no system
# lld, clang, or C cross-toolchain is needed -- and no glibc, so it runs on the
# robot's ancient userland (glibc 2.23) unchanged.
#
# One-time: rustup target add aarch64-unknown-linux-musl
#
# Note on the linker: `-C linker-flavor=ld.lld` makes rustc pass `-flavor gnu`,
# which the RAW rust-lld accepts but the gcc-ld/ld.lld wrapper does NOT -- so we
# point -C linker at the raw rust-lld, not the wrapper.
set -eu
cd "$(dirname "$0")/.."

SR=$(rustc --print sysroot)
HOST=$(rustc -vV | sed -n 's/^host: //p')
LLD="$SR/lib/rustlib/$HOST/bin/rust-lld"
[ -x "$LLD" ] || { echo "ERROR: bundled rust-lld not found at $LLD"; exit 1; }

export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_RUSTFLAGS="-C linker-flavor=ld.lld -C linker=$LLD -C link-self-contained=yes"

cargo build --release --target aarch64-unknown-linux-musl "$@"

BIN=target/aarch64-unknown-linux-musl/release/ros2dreame
echo ">> built $BIN"
file "$BIN" 2>/dev/null || true
