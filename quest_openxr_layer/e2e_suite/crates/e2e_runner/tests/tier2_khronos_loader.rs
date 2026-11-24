//! Tier 2 Real Khronos OpenXR 1.1.54 `libopenxr_loader.so` End-to-End Tests.
//!
//! Spawns `e2e_khronos_probe` with:
//! - Genuine Khronos OpenXR 1.1.54 `libopenxr_loader.so`
//! - `XR_RUNTIME_JSON` pointing to `libmock_xr_runtime.so`
//! - `XDG_DATA_HOME` containing a **verbatim copy** of `manifest/XrApiLayer_alvr_hand_emulation.json`
//! - `LD_LIBRARY_PATH` exposing compiled `libXrApiLayer_alvr_hand_emulation.so`

use e2e_runner::{
    host_layer_so_path, khronos_loader_so_path, layer_manifest_json_path, mock_runtime_so_path,
    WorkspaceTempDir,
};
use std::fs;
use std::path::PathBuf;
use std::process::Command;

fn run_khronos_probe(
    mode: &str,
    extra_env: &[(&str, String)],
    unset_env: &[&str],
) -> std::process::Output {
    let loader_so = khronos_loader_so_path();
    assert!(
        loader_so.exists(),
        "Khronos loader missing at {} — run scripts/build_khronos_loader.sh",
        loader_so.display()
    );
    let runtime_so = mock_runtime_so_path();
    assert!(
        runtime_so.exists(),
        "libmock_xr_runtime.so missing at {}",
        runtime_so.display()
    );
    let layer_so = host_layer_so_path();
    assert!(
        layer_so.exists(),
        "Host layer .so missing at {}",
        layer_so.display()
    );
    let manifest_src = layer_manifest_json_path();
    assert!(manifest_src.exists(), "Manifest JSON missing");

    let tmp = WorkspaceTempDir::new(&format!("khronos_{mode}")).unwrap();

    // Place verbatim copy of layer JSON manifest under XDG_DATA_HOME/openxr/1/api_layers/implicit.d/
    let implicit_dir = tmp.path().join("xdg_data/openxr/1/api_layers/implicit.d");
    fs::create_dir_all(&implicit_dir).unwrap();
    fs::copy(
        &manifest_src,
        implicit_dir.join("XrApiLayer_alvr_hand_emulation.json"),
    )
    .unwrap();

    // Write XR_RUNTIME_JSON pointing to canonicalized libmock_xr_runtime.so
    let rt_canon = fs::canonicalize(&runtime_so).unwrap();
    let rt_json_path = tmp.path().join("mock_runtime.json");
    let rt_json = serde_json::json!({
        "file_format_version": "1.0.0",
        "runtime": {
            "name": "ALVR_E2E_Mock_Runtime",
            "library_path": rt_canon.to_string_lossy()
        }
    });
    fs::write(&rt_json_path, serde_json::to_string_pretty(&rt_json).unwrap()).unwrap();

    let layer_dir = fs::canonicalize(layer_so.parent().unwrap()).unwrap();
    let existing_ld = std::env::var("LD_LIBRARY_PATH").unwrap_or_default();
    let new_ld = if existing_ld.is_empty() {
        layer_dir.to_string_lossy().into_owned()
    } else {
        format!("{}:{existing_ld}", layer_dir.display())
    };

    let probe_bin = PathBuf::from(env!("CARGO_BIN_EXE_e2e_khronos_probe"));
    let mut cmd = Command::new(probe_bin);
    cmd.arg(mode)
        .env("E2E_PROBE_LOADER_SO", &loader_so)
        .env("E2E_PROBE_RUNTIME_SO", &rt_canon)
        .env("XR_RUNTIME_JSON", &rt_json_path)
        .env("XDG_DATA_HOME", tmp.path().join("xdg_data"))
        .env("LD_LIBRARY_PATH", new_ld);

    for k in unset_env {
        cmd.env_remove(k);
    }
    for (k, v) in extra_env {
        cmd.env(k, v);
    }

    cmd.output().expect("spawn e2e_khronos_probe")
}

#[test]
fn t2_khronos_loader_implicit_discovery_emulation_and_zero_leaks() {
    let out = run_khronos_probe(
        "emulate",
        &[],
        &["DISABLE_XR_APILAYER_ALVR_HAND_EMULATION", "ALVR_HAND_LAYER_CONFIG"],
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "e2e_khronos_probe emulate failed (status={:?})\nstdout:\n{stdout}\nstderr:\n{stderr}",
        out.status
    );
    assert!(stdout.contains("PROBE_OK mode=emulate"), "stdout={stdout}");
}

#[test]
fn t2_khronos_loader_custom_json_config_only_touch() {
    let tmp = WorkspaceTempDir::new("khronos_cfg").unwrap();
    let cfg_path = tmp.path().join("custom_only_touch.json");
    fs::write(
        &cfg_path,
        r#"{ "enabled": true, "gestures": { "only_touch": true } }"#,
    )
    .unwrap();

    let out = run_khronos_probe(
        "only_touch",
        &[("ALVR_HAND_LAYER_CONFIG", cfg_path.to_string_lossy().into_owned())],
        &["DISABLE_XR_APILAYER_ALVR_HAND_EMULATION"],
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "e2e_khronos_probe only_touch failed (status={:?})\nstdout:\n{stdout}\nstderr:\n{stderr}",
        out.status
    );
    assert!(stdout.contains("PROBE_OK mode=only_touch"), "stdout={stdout}");
}

#[test]
fn t2_khronos_loader_disable_env_var_skips_implicit_layer() {
    let out = run_khronos_probe(
        "disabled",
        &[("DISABLE_XR_APILAYER_ALVR_HAND_EMULATION", "1".to_owned())],
        &["ALVR_HAND_LAYER_CONFIG"],
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        out.status.success(),
        "e2e_khronos_probe disabled failed (status={:?})\nstdout:\n{stdout}\nstderr:\n{stderr}",
        out.status
    );
    assert!(stdout.contains("PROBE_OK mode=disabled"), "stdout={stdout}");
}
