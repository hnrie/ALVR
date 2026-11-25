#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> [E2E 1/3] Ensuring genuine Khronos OpenXR 1.1.54 loader is built..."
"${SCRIPT_DIR}/scripts/build_khronos_loader.sh"

echo "==> [E2E 2/3] Building mock_xr_runtime cdylib & e2e_khronos_probe..."
cargo build --manifest-path "${SCRIPT_DIR}/Cargo.toml" -p mock_xr_runtime -p e2e_runner

echo "==> [E2E 3/3] Running all E2E suites (Tier 1 Direct OpenXR + Tier 2 Real Khronos Loader + Tier 3 Android ELF/AAR/APK)..."
cargo test --manifest-path "${SCRIPT_DIR}/Cargo.toml" -p e2e_runner -- --nocapture

echo "==> All E2E suites passed!"
