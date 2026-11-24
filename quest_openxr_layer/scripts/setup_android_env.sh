#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$WORKSPACE_DIR"

LOCAL_DIR="${HOME}/.local"
NDK_VERSION="r26b"
NDK_DIR="${ANDROID_NDK_HOME:-${LOCAL_DIR}/android-ndk-${NDK_VERSION}}"
NDK_PARENT="$(dirname "$NDK_DIR")"
NDK_URL="https://dl.google.com/android/repository/android-ndk-${NDK_VERSION}-linux.zip"
NDK_ZIP="${NDK_PARENT}/android-ndk-${NDK_VERSION}-linux.zip"
NDK_BIN_DIR="${NDK_DIR}/toolchains/llvm/prebuilt/linux-x86_64/bin"
NDK_CLANG="${NDK_BIN_DIR}/aarch64-linux-android28-clang"
NDK_AR="${NDK_BIN_DIR}/llvm-ar"
FRAMEWORK_RES="/usr/share/android-framework-res/framework-res.apk"
REQUIRED_FREE_MB_FOR_NDK=4096

STAGING_DIR=""
cleanup() {
    if [[ -n "${NDK_ZIP:-}" && -f "$NDK_ZIP" ]]; then
        rm -f "$NDK_ZIP"
    fi
    if [[ -n "${STAGING_DIR:-}" && -d "$STAGING_DIR" ]]; then
        rm -rf "$STAGING_DIR"
    fi
}
trap cleanup EXIT

mkdir -p "$NDK_PARENT"

# 1. Disk space check on NDK parent directory (never use /tmp or / for NDK)
df_out="$(df -Pm "$NDK_PARENT")"
avail_mb="$(awk 'NR==2 {print $4}' <<<"$df_out")"
echo "[setup] Available disk space on ${NDK_PARENT}: ${avail_mb} MiB"

# 2. Rust target: aarch64-linux-android (runs from WORKSPACE_DIR to inherit rust-toolchain.toml)
active_tc="$(rustup show active-toolchain)"
echo "[setup] Active Rust toolchain: ${active_tc}"
installed_targets="$(rustup target list --installed)"
if grep -qx "aarch64-linux-android" <<<"$installed_targets"; then
    echo "[setup] Rust target aarch64-linux-android already installed"
else
    echo "[setup] Installing Rust target aarch64-linux-android..."
    rustup target add aarch64-linux-android
fi

# 3. Android NDK r26b under /home/antigravity/.local/android-ndk-r26b
if [[ -x "$NDK_CLANG" && -x "$NDK_AR" ]]; then
    echo "[setup] Android NDK ${NDK_VERSION} already present at ${NDK_DIR}"
else
    if (( avail_mb < REQUIRED_FREE_MB_FOR_NDK )); then
        echo "ERROR: Need at least ${REQUIRED_FREE_MB_FOR_NDK} MiB free on ${NDK_PARENT}, only ${avail_mb} MiB available" >&2
        exit 1
    fi
    echo "[setup] Downloading Android NDK ${NDK_VERSION} to ${NDK_ZIP}..."
    rm -f "$NDK_ZIP"
    curl --fail --location --silent --show-error \
        --retry 3 --retry-delay 3 --connect-timeout 30 \
        --output "$NDK_ZIP" "$NDK_URL"

    STAGING_DIR="${NDK_PARENT}/.android-ndk-${NDK_VERSION}.staging.$$"
    mkdir -p "$STAGING_DIR"
    echo "[setup] Extracting Android NDK ${NDK_VERSION} into ${STAGING_DIR}..."
    unzip -q "$NDK_ZIP" -d "$STAGING_DIR"
    rm -f "$NDK_ZIP"

    if [[ ! -d "${STAGING_DIR}/android-ndk-${NDK_VERSION}" ]]; then
        echo "ERROR: Unexpected NDK archive layout in ${STAGING_DIR}" >&2
        exit 1
    fi

    rm -rf "$NDK_DIR"
    mv "${STAGING_DIR}/android-ndk-${NDK_VERSION}" "$NDK_DIR"
    rmdir "$STAGING_DIR"
    STAGING_DIR=""
    echo "[setup] Installed Android NDK ${NDK_VERSION} to ${NDK_DIR}"
fi

# 4. System Android packaging tools via apt-get (skip sudo if all already present)
need_apt=0
for tool in zipalign apksigner aapt; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        need_apt=1
        break
    fi
done
if [[ ! -f "$FRAMEWORK_RES" ]]; then
    need_apt=1
fi

if (( need_apt == 1 )); then
    echo "[setup] Installing Android packaging tools (zipalign, apksigner, aapt, android-framework-res)..."
    export DEBIAN_FRONTEND=noninteractive
    if [[ "$(id -u)" -eq 0 ]]; then
        apt-get update
        apt-get install -y zipalign apksigner aapt android-framework-res
    else
        sudo apt-get update
        sudo apt-get install -y zipalign apksigner aapt android-framework-res
    fi
else
    echo "[setup] Android packaging tools already installed"
fi

# 5. End-to-end verification (pipefail-safe & zipalign exit-code-safe)
echo "[setup] Verifying installed toolchain..."

verified_targets="$(rustup target list --installed)"
grep -qx "aarch64-linux-android" <<<"$verified_targets"
echo "  rustup target: aarch64-linux-android OK"

clang_out="$(env -u CPATH -u LIBRARY_PATH -u PKG_CONFIG_LIBDIR "$NDK_CLANG" --version)"
clang_ver="${clang_out%%$'\n'*}"
echo "  NDK clang:     ${clang_ver}"

ar_out="$("$NDK_AR" --version)"
ar_ver="${ar_out%%$'\n'*}"
echo "  NDK llvm-ar:   ${ar_ver}"

ZIPALIGN_BIN="$(command -v zipalign)"
test -x "$ZIPALIGN_BIN"
zipalign_out="$("$ZIPALIGN_BIN" 2>&1 || true)"
grep -qiE "zipalign|alignment" <<<"$zipalign_out"
echo "  zipalign:      ${ZIPALIGN_BIN} OK"

apksigner_out="$(apksigner --version)"
apksigner_ver="${apksigner_out%%$'\n'*}"
echo "  apksigner:     ${apksigner_ver}"

aapt_out="$(aapt version)"
aapt_ver="${aapt_out%%$'\n'*}"
echo "  aapt:          ${aapt_ver}"

test -s "$FRAMEWORK_RES"
echo "  framework-res: ${FRAMEWORK_RES} OK"

echo ""
echo "[setup] Android environment ready."
echo "  ANDROID_NDK_HOME=${NDK_DIR}"
echo "  CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER=${NDK_CLANG}"
echo "  CARGO_TARGET_AARCH64_LINUX_ANDROID_AR=${NDK_AR}"
echo "  Cross-compile prefix: env -u CPATH -u LIBRARY_PATH -u PKG_CONFIG_LIBDIR"
