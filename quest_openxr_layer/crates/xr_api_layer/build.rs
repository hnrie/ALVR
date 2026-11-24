use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let map_path = manifest_dir.join("XrApiLayer.map");
    println!("cargo:rerun-if-changed={}", map_path.display());

    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let soname = "libXrApiLayer_alvr_hand_emulation.so";

    match target_os.as_str() {
        "android" => {
            println!("cargo:rustc-cdylib-link-arg=-Wl,-soname,{soname}");
            println!("cargo:rustc-cdylib-link-arg=-Wl,-z,max-page-size=16384");
            println!("cargo:rustc-cdylib-link-arg=-Wl,-z,common-page-size=16384");
            println!(
                "cargo:rustc-cdylib-link-arg=-Wl,--version-script={}",
                map_path.display()
            );
            println!("cargo:rustc-link-lib=log");
        }
        "linux" => {
            println!("cargo:rustc-cdylib-link-arg=-Wl,-soname,{soname}");
            println!(
                "cargo:rustc-cdylib-link-arg=-Wl,--version-script={}",
                map_path.display()
            );
        }
        _ => {}
    }
}
