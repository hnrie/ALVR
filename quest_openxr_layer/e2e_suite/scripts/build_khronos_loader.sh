#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
E2E_SUITE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${E2E_SUITE_DIR}/target/openxr_loader_build"
OUT_DIR="${E2E_SUITE_DIR}/target/khronos_loader"
OUT_SO="${OUT_DIR}/libopenxr_loader.so"
TMP_DIR="${E2E_SUITE_DIR}/target/e2e_tmp"

REBUILD=0
for arg in "$@"; do
    case "$arg" in
        --rebuild|--rebuild-loader)
            REBUILD=1
            ;;
        -h|--help)
            echo "Usage: $0 [--rebuild]"
            echo "Builds genuine Khronos OpenXR 1.1.54 libopenxr_loader.so offline from cached openxr-sys-0.13.1."
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $arg" >&2
            exit 64
            ;;
    esac
done

mkdir -p "${TMP_DIR}"
export TMPDIR="${TMP_DIR}"

if [[ "${REBUILD}" -eq 0 && -f "${OUT_SO}" ]]; then
    echo "[build_khronos_loader] Reusing cached ${OUT_SO}"
    exit 0
fi

CARGO_HOME_DIR="${CARGO_HOME:-${HOME}/.cargo}"
OPENXR_SDK_SRC=""
shopt -s nullglob
for candidate in "${CARGO_HOME_DIR}"/registry/src/index.crates.io-*/openxr-sys-0.13.1/OpenXR-SDK; do
    if [[ -f "${candidate}/CMakeLists.txt" ]]; then
        OPENXR_SDK_SRC="${candidate}"
        break
    fi
done
shopt -u nullglob

if [[ -z "${OPENXR_SDK_SRC}" ]]; then
    echo "ERROR: Could not locate cached OpenXR-SDK under ${CARGO_HOME_DIR}/registry/src/index.crates.io-*/openxr-sys-0.13.1/OpenXR-SDK" >&2
    exit 2
fi

for tool in cmake ninja c++; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "ERROR: Required build tool '${tool}' not found in PATH" >&2
        exit 2
    fi
done

if [[ "${REBUILD}" -eq 1 ]]; then
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}" "${OUT_DIR}"

echo "[build_khronos_loader] Source: ${OPENXR_SDK_SRC}"
echo "[build_khronos_loader] Build:  ${BUILD_DIR}"

cmake -S "${OPENXR_SDK_SRC}" -B "${BUILD_DIR}" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_LOADER=ON \
    -DDYNAMIC_LOADER=ON \
    -DBUILD_WITH_SYSTEM_JSONCPP=OFF \
    -DBUILD_FORCE_GENERATION=OFF \
    -DBUILD_API_LAYERS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_CONFORMANCE_TESTS=OFF \
    -DOpenGL_GL_PREFERENCE=GLVND \
    -DPRESENTATION_BACKEND=xlib \
    -DBUILD_WITH_XCB_HEADERS=OFF \
    -DBUILD_WITH_WAYLAND_HEADERS=OFF

cmake --build "${BUILD_DIR}" --target openxr_loader -j 2

BUILT_SO="${BUILD_DIR}/src/loader/libopenxr_loader.so"
if [[ ! -e "${BUILT_SO}" ]]; then
    echo "ERROR: Expected built loader at ${BUILT_SO} not found" >&2
    exit 1
fi

cp -L "${BUILT_SO}" "${OUT_SO}.tmp"
mv -f "${OUT_SO}.tmp" "${OUT_SO}"
chmod 755 "${OUT_SO}"

echo "[build_khronos_loader] Installed ${OUT_SO}"
