#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ANDROID_NDK_HOME="${ANDROID_NDK_HOME:-/home/antigravity/.local/android-ndk-r26b}"
TOOLCHAIN_BIN="${ANDROID_NDK_HOME}/toolchains/llvm/prebuilt/linux-x86_64/bin"
LINKER="${TOOLCHAIN_BIN}/aarch64-linux-android28-clang"
AR="${TOOLCHAIN_BIN}/llvm-ar"

if [[ ! -x "${LINKER}" ]]; then
    echo "error: Android NDK linker not found at ${LINKER}" >&2
    echo "Run ${SCRIPT_DIR}/setup_android_env.sh first." >&2
    exit 1
fi

echo "==> [1/5] Building host x86_64 workspace (xr_api_layer + alvr-xr-packager)..."
cargo build --offline --manifest-path "${ROOT_DIR}/Cargo.toml"
cargo build --offline --release --manifest-path "${ROOT_DIR}/Cargo.toml"

echo "==> [2/5] Cross-compiling libXrApiLayer_alvr_hand_emulation.so for aarch64-linux-android..."
env -u CPATH -u LIBRARY_PATH -u PKG_CONFIG_LIBDIR \
    CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER="${LINKER}" \
    CARGO_TARGET_AARCH64_LINUX_ANDROID_AR="${AR}" \
    cargo build --offline --target aarch64-linux-android --release \
        -p xr_api_layer --manifest-path "${ROOT_DIR}/Cargo.toml"

ANDROID_SO="${ROOT_DIR}/target/aarch64-linux-android/release/libXrApiLayer_alvr_hand_emulation.so"
MANIFEST_JSON="${ROOT_DIR}/manifest/XrApiLayer_alvr_hand_emulation.json"
PACKAGER="${ROOT_DIR}/target/release/alvr-xr-packager"
DIST_DIR="${ROOT_DIR}/dist"

echo "==> [3/5] Populating ${DIST_DIR}..."
mkdir -p "${DIST_DIR}/arm64-v8a"
cp -f "${ANDROID_SO}" "${DIST_DIR}/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so"
cp -f "${MANIFEST_JSON}" "${DIST_DIR}/XrApiLayer_alvr_hand_emulation.json"

cat > "${DIST_DIR}/alvr_hand_controller.default.json" <<'EOF'
{
  "enabled": true,
  "gestures": {
    "only_touch": false,
    "pinch_touch_distance": 0.0,
    "pinch_trigger_distance": 0.25,
    "curl_touch_distance": 2.0,
    "curl_trigger_distance": 2.5,
    "joystick_deadzone": 40.0,
    "joystick_offset_horizontal": 0.0,
    "joystick_offset_vertical": 0.0,
    "joystick_range": 1.0,
    "activation_delay": 50,
    "deactivation_delay": 100,
    "repeat_delay": 100
  },
  "pose": {
    "mode": "meta_quest_native_grip",
    "left_position_offset": [
      0.04,
      -0.02,
      -0.13
    ],
    "left_rotation_offset_deg": [
      0.0,
      -45.0,
      -90.0
    ],
    "aim_mode": "prefer_fb_aim",
    "left_aim_position_offset": [
      0.0,
      -0.04,
      -0.04
    ],
    "left_aim_rotation_offset_deg": [
      -60.0,
      0.0,
      0.0
    ]
  },
  "float_to_bool_threshold": 0.5,
  "float_to_bool_hysteresis": 0.05,
  "physical_controller_grace_ms": 250
}
EOF

echo "==> [4/5] Building deterministic Android AAR..."
"${PACKAGER}" package-aar \
    --so "${DIST_DIR}/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so" \
    --manifest "${DIST_DIR}/XrApiLayer_alvr_hand_emulation.json" \
    --config "${DIST_DIR}/alvr_hand_controller.default.json" \
    --output "${DIST_DIR}/alvr-hand-controller-layer.aar"

echo "==> [5/5] Verifying built AAR and ELF binary..."
"${PACKAGER}" inspect-apk --input "${DIST_DIR}/alvr-hand-controller-layer.aar"
echo "Done! Distributable Quest artifacts ready in: ${DIST_DIR}"
