use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let layer_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("quest_patcher_gui must live under quest_openxr_layer/crates/");

    let so_path = std::env::var("ALVR_EMBED_LAYER_SO")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            layer_root.join("dist/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so")
        });
    let manifest_path = std::env::var("ALVR_EMBED_LAYER_MANIFEST")
        .map(PathBuf::from)
        .unwrap_or_else(|_| layer_root.join("manifest/XrApiLayer_alvr_hand_emulation.json"));
    let default_cfg_path = std::env::var("ALVR_EMBED_DEFAULT_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|_| layer_root.join("dist/alvr_hand_controller.default.json"));

    println!("cargo:rerun-if-env-changed=ALVR_EMBED_LAYER_SO");
    println!("cargo:rerun-if-env-changed=ALVR_EMBED_LAYER_MANIFEST");
    println!("cargo:rerun-if-env-changed=ALVR_EMBED_DEFAULT_CONFIG");
    println!("cargo:rerun-if-changed={}", so_path.display());
    println!("cargo:rerun-if-changed={}", manifest_path.display());
    println!("cargo:rerun-if-changed={}", default_cfg_path.display());

    assert!(
        so_path.is_file(),
        "Prebuilt Quest arm64-v8a layer .so not found at {}. Run quest_openxr_layer/scripts/build_android.sh first or set ALVR_EMBED_LAYER_SO.",
        so_path.display()
    );
    assert!(
        manifest_path.is_file(),
        "OpenXR layer manifest JSON not found at {}",
        manifest_path.display()
    );
    assert!(
        default_cfg_path.is_file(),
        "Default LayerConfig JSON not found at {}",
        default_cfg_path.display()
    );

    println!("cargo:rustc-env=ALVR_EMBED_SO_PATH={}", so_path.display());
    println!(
        "cargo:rustc-env=ALVR_EMBED_MANIFEST_PATH={}",
        manifest_path.display()
    );
    println!(
        "cargo:rustc-env=ALVR_EMBED_CONFIG_PATH={}",
        default_cfg_path.display()
    );
}
