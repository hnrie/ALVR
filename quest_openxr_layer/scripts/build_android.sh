#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

find_ndk() {
    local candidates=()
    if [[ -n "${ANDROID_NDK_HOME:-}" ]]; then
        candidates+=("${ANDROID_NDK_HOME}")
    fi
    if [[ -n "${ANDROID_NDK_LATEST_HOME:-}" ]]; then
        candidates+=("${ANDROID_NDK_LATEST_HOME}")
    fi
    if [[ -n "${ANDROID_NDK_ROOT:-}" ]]; then
        candidates+=("${ANDROID_NDK_ROOT}")
    fi
    shopt -s nullglob
    for d in /opt/android-sdk/ndk/* "${ANDROID_SDK_ROOT:-/nonexistent}"/ndk/* "${ANDROID_HOME:-/nonexistent}"/ndk/* "${HOME}"/.local/android-ndk-*; do
        candidates+=("${d}")
    done
    shopt -u nullglob

    local host_tag="linux-x86_64"
    if [[ "$(uname -s)" == "Darwin" ]]; then
        host_tag="darwin-x86_64"
    fi

    for cand in "${candidates[@]}"; do
        if [[ -x "${cand}/toolchains/llvm/prebuilt/${host_tag}/bin/aarch64-linux-android28-clang" ]]; then
            echo "${cand}"
            return 0
        fi
    done
    return 1
}

ANDROID_NDK_HOME="$(find_ndk || true)"
if [[ -z "${ANDROID_NDK_HOME}" ]]; then
    echo "error: Android NDK (r26+) not found. Set ANDROID_NDK_HOME or install NDK under /opt/android-sdk/ndk/." >&2
    exit 1
fi

HOST_TAG="linux-x86_64"
if [[ "$(uname -s)" == "Darwin" ]]; then
    HOST_TAG="darwin-x86_64"
fi

TOOLCHAIN_BIN="${ANDROID_NDK_HOME}/toolchains/llvm/prebuilt/${HOST_TAG}/bin"
LINKER="${TOOLCHAIN_BIN}/aarch64-linux-android28-clang"
AR="${TOOLCHAIN_BIN}/llvm-ar"

echo "==> Using Android NDK: ${ANDROID_NDK_HOME}"

echo "==> [1/5] Building host workspace (xr_api_layer + alvr-xr-packager)..."
cargo build --manifest-path "${ROOT_DIR}/Cargo.toml" -p xr_api_layer -p android_packager
cargo build --release --manifest-path "${ROOT_DIR}/Cargo.toml" -p xr_api_layer -p android_packager

echo "==> [2/5] Cross-compiling libXrApiLayer_alvr_hand_emulation.so for aarch64-linux-android..."
env -u CPATH -u LIBRARY_PATH -u PKG_CONFIG_LIBDIR \
    CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER="${LINKER}" \
    CARGO_TARGET_AARCH64_LINUX_ANDROID_AR="${AR}" \
    cargo build --target aarch64-linux-android --release \
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
