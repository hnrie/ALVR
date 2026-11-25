//! Tier 3 Android Packaging, ELF64 ABI, Deterministic AAR, Binary AXML, Pure-Rust 16 KB Alignment,
//! Pure-Rust APK Signature Scheme v1/v2/v3 Signing, and Official Android SDK Cross-Verification E2E Tests.

use e2e_runner::{
    android_layer_so_path, dist_dir, host_layer_so_path, layer_manifest_json_path,
    packager_bin_path, WorkspaceTempDir,
};
use sha2::{Digest, Sha256};
use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::Command;
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipArchive, ZipWriter};

fn sha256_file(p: &Path) -> String {
    let bytes = fs::read(p).expect("read for sha256");
    hex::encode(Sha256::digest(&bytes))
}

fn sdk_roots() -> Vec<PathBuf> {
    let mut roots = Vec::new();
    for var in ["ANDROID_SDK_ROOT", "ANDROID_HOME"] {
        if let Ok(v) = std::env::var(var) {
            if !v.is_empty() {
                roots.push(PathBuf::from(v));
            }
        }
    }
    roots.push(PathBuf::from("/opt/android-sdk"));
    roots.push(PathBuf::from("/usr/local/lib/android/sdk"));
    roots
}

fn find_build_tool(name: &str) -> PathBuf {
    // Check SDK build-tools newest version first
    for root in sdk_roots() {
        let bt = root.join("build-tools");
        if let Ok(entries) = fs::read_dir(&bt) {
            let mut dirs: Vec<PathBuf> = entries
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .collect();
            dirs.sort();
            dirs.reverse();
            for d in dirs {
                let cand = d.join(name);
                if cand.is_file() {
                    return cand;
                }
            }
        }
    }
    PathBuf::from(name)
}

fn find_android_framework_jar() -> PathBuf {
    let deb_fw = PathBuf::from("/usr/share/android-framework-res/framework-res.apk");
    if deb_fw.is_file() {
        return deb_fw;
    }
    for root in sdk_roots() {
        let platforms = root.join("platforms");
        if let Ok(entries) = fs::read_dir(&platforms) {
            let mut dirs: Vec<PathBuf> = entries
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .collect();
            dirs.sort();
            dirs.reverse();
            for d in dirs {
                let jar = d.join("android.jar");
                if jar.is_file() {
                    return jar;
                }
            }
        }
    }
    panic!("could not find android.jar or framework-res.apk in SDK or /usr/share/android-framework-res");
}

fn find_readelf() -> PathBuf {
    for var in ["ANDROID_NDK_HOME", "ANDROID_NDK_LATEST_HOME", "ANDROID_NDK_ROOT"] {
        if let Ok(v) = std::env::var(var) {
            let cand = PathBuf::from(v)
                .join("toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-readelf");
            if cand.is_file() {
                return cand;
            }
        }
    }
    for root in sdk_roots() {
        let ndk = root.join("ndk");
        if let Ok(entries) = fs::read_dir(&ndk) {
            let mut dirs: Vec<PathBuf> = entries
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .collect();
            dirs.sort();
            dirs.reverse();
            for d in dirs {
                let cand = d.join("toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-readelf");
                if cand.is_file() {
                    return cand;
                }
            }
        }
    }
    if Command::new("llvm-readelf").arg("--version").output().is_ok() {
        return PathBuf::from("llvm-readelf");
    }
    PathBuf::from("readelf")
}

/// Builds a genuine compiled Android APK using `aapt package -I android.jar`,
/// injects minimal valid DEX + optional `lib/arm64-v8a/libopenxr_loader.so` / `libvrapi.so` / 32-bit lib,
/// and optionally pre-signs with stale v1 `jarsigner`!
fn build_synthetic_quest_apk(
    dir: &Path,
    name: &str,
    manifest_xml: &str,
    extra_entries: &[(&str, &[u8], CompressionMethod)],
    presign_v1: bool,
) -> PathBuf {
    let src_dir = dir.join(format!("{name}_src"));
    let res_dir = src_dir.join("res/values");
    fs::create_dir_all(&res_dir).unwrap();
    fs::write(src_dir.join("AndroidManifest.xml"), manifest_xml).unwrap();
    fs::write(
        res_dir.join("strings.xml"),
        r#"<?xml version="1.0" encoding="utf-8"?><resources><string name="app_name">QuestTest</string></resources>"#,
    )
    .unwrap();

    let base_apk = dir.join(format!("{name}_aapt.apk"));
    let fw = find_android_framework_jar();
    let aapt = find_build_tool("aapt");

    let out = Command::new(&aapt)
        .args(["package", "-f", "-M"])
        .arg(src_dir.join("AndroidManifest.xml"))
        .arg("-S")
        .arg(src_dir.join("res"))
        .arg("-I")
        .arg(&fw)
        .arg("-F")
        .arg(&base_apk)
        .output()
        .expect("run aapt package");
    assert!(
        out.status.success(),
        "aapt package failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );

    // Augment aapt APK with extra entries (DEX, native libs) cleanly.
    let final_apk = dir.join(format!("{name}.apk"));
    {
        let mut in_zip = ZipArchive::new(File::open(&base_apk).unwrap()).unwrap();
        let mut out_zip = ZipWriter::new(File::create(&final_apk).unwrap());
        for i in 0..in_zip.len() {
            let mut e = in_zip.by_index(i).unwrap();
            let mut buf = Vec::new();
            e.read_to_end(&mut buf).unwrap();
            let method = if e.name() == "resources.arsc" {
                CompressionMethod::Stored
            } else {
                e.compression()
            };
            out_zip
                .start_file(
                    e.name(),
                    SimpleFileOptions::default().compression_method(method),
                )
                .unwrap();
            out_zip.write_all(&buf).unwrap();
        }
        // Minimal DEX 035 header stub
        let mut dex = vec![0u8; 112];
        dex[0..8].copy_from_slice(b"dex\n035\0");
        out_zip
            .start_file(
                "classes.dex",
                SimpleFileOptions::default().compression_method(CompressionMethod::Deflated),
            )
            .unwrap();
        out_zip.write_all(&dex).unwrap();

        for (entry_name, data, method) in extra_entries {
            out_zip
                .start_file(
                    *entry_name,
                    SimpleFileOptions::default().compression_method(*method),
                )
                .unwrap();
            out_zip.write_all(data).unwrap();
        }
        out_zip.finish().unwrap();
    }

    if presign_v1 {
        let ks = dir.join(format!("{name}_v1.p12"));
        let kt = Command::new("keytool")
            .args([
                "-genkeypair",
                "-storetype",
                "PKCS12",
                "-keystore",
            ])
            .arg(&ks)
            .args([
                "-storepass",
                "stale123",
                "-keypass",
                "stale123",
                "-alias",
                "stalekey",
                "-keyalg",
                "RSA",
                "-keysize",
                "2048",
                "-validity",
                "365",
                "-dname",
                "CN=StaleV1, O=Test, C=US",
            ])
            .output()
            .expect("keytool");
        assert!(kt.status.success());
        let js = Command::new("jarsigner")
            .arg("-keystore")
            .arg(&ks)
            .args(["-storepass", "stale123", "-keypass", "stale123"])
            .arg(&final_apk)
            .arg("stalekey")
            .output()
            .expect("jarsigner");
        assert!(
            js.status.success(),
            "jarsigner failed: {}",
            String::from_utf8_lossy(&js.stderr)
        );
    }

    final_apk
}

#[test]
fn t3_android_arm64_so_elf_abi_symbols_and_16kb_alignment() {
    let so = android_layer_so_path();
    assert!(so.exists(), "Android .so missing at {}", so.display());

    let file_out = Command::new("file").arg(&so).output().expect("file");
    let file_str = String::from_utf8_lossy(&file_out.stdout);
    assert!(file_str.contains("ELF 64-bit LSB shared object"), "{file_str}");
    assert!(file_str.contains("ARM aarch64"), "{file_str}");

    let readelf = find_readelf();
    let dyn_out = Command::new(&readelf)
        .args(["--wide", "-d"])
        .arg(&so)
        .output()
        .expect("readelf -d");
    let dyn_str = String::from_utf8_lossy(&dyn_out.stdout);
    assert!(
        dyn_str.contains("[libXrApiLayer_alvr_hand_emulation.so]"),
        "missing SONAME:\n{dyn_str}"
    );
    let allowed = ["libc.so", "libdl.so", "libm.so", "liblog.so"];
    for line in dyn_str.lines().filter(|l| l.contains("(NEEDED)")) {
        assert!(
            allowed.iter().any(|a| line.contains(&format!("[{a}]"))),
            "disallowed DT_NEEDED entry: {line}"
        );
    }

    let ph_out = Command::new(&readelf)
        .args(["--wide", "-l"])
        .arg(&so)
        .output()
        .expect("readelf -l");
    let ph_str = String::from_utf8_lossy(&ph_out.stdout);
    let load_lines: Vec<&str> = ph_str
        .lines()
        .filter(|l| l.trim_start().starts_with("LOAD"))
        .collect();
    assert!(!load_lines.is_empty());
    for l in load_lines {
        let align_tok = l.split_whitespace().last().unwrap();
        let align = u64::from_str_radix(align_tok.trim_start_matches("0x"), 16).unwrap();
        assert!(align >= 0x4000, "PT_LOAD align < 16KB: {l}");
    }

    let sym_out = Command::new(&readelf)
        .args(["--wide", "--dyn-syms"])
        .arg(&so)
        .output()
        .expect("readelf --dyn-syms");
    let sym_str = String::from_utf8_lossy(&sym_out.stdout);
    let mut exported: Vec<String> = sym_str
        .lines()
        .filter(|l| l.contains("GLOBAL DEFAULT") && !l.contains(" UND "))
        .filter_map(|l| l.split_whitespace().last().map(str::to_owned))
        .collect();
    exported.sort();
    assert_eq!(
        exported,
        vec![
            "xrInitializeLoaderKHR".to_owned(),
            "xrNegotiateLoaderApiLayerInterface".to_owned()
        ]
    );
}

#[test]
fn t3_aar_contents_manifest_and_reproducible_determinism() {
    let dist_aar = dist_dir().join("alvr-hand-controller-layer.aar");
    assert!(dist_aar.exists(), "dist AAR missing at {}", dist_aar.display());

    // Inspect dist AAR via CLI --json with EMPTY PATH (zero external tools!)
    let out = Command::new(packager_bin_path())
        .env("PATH", "")
        .args(["inspect-apk", "--input"])
        .arg(&dist_aar)
        .arg("--json")
        .output()
        .expect("inspect-apk aar");
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    let v: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(v["archive_kind"], "aar");
    assert_eq!(v["has_quest_hand_tracking"], true);
    assert_eq!(v["has_layer_manifest"], true);
    assert_eq!(v["layer_manifest_valid"], true);
    assert_eq!(v["layer_so_elf"]["machine_name"], "AArch64");

    // Verify AAR plain XML manifest has minSdkVersion, targetSdkVersion, required="false", and classes.jar!
    let mut zip = ZipArchive::new(File::open(&dist_aar).unwrap()).unwrap();
    let mut manifest_xml = String::new();
    zip.by_name("AndroidManifest.xml")
        .unwrap()
        .read_to_string(&mut manifest_xml)
        .unwrap();
    assert!(manifest_xml.contains(r#"android:minSdkVersion="29""#));
    assert!(manifest_xml.contains(r#"android:targetSdkVersion="32""#));
    assert!(manifest_xml.contains(r#"android:name="oculus.software.handtracking" android:required="false""#));
    assert!(zip.by_name("classes.jar").is_ok());
    assert!(zip.by_name("R.txt").is_ok());
    assert!(zip.by_name("proguard.txt").is_ok());

    // Determinism check: build twice 1.2s apart -> identical SHA-256!
    let tmp = WorkspaceTempDir::new("aar_det").unwrap();
    let aar1 = tmp.path().join("a1.aar");
    let aar2 = tmp.path().join("a2.aar");
    for dst in [&aar1, &aar2] {
        let r = Command::new(packager_bin_path())
            .env("PATH", "")
            .args(["package-aar", "--so"])
            .arg(android_layer_so_path())
            .arg("--manifest")
            .arg(layer_manifest_json_path())
            .arg("--output")
            .arg(dst)
            .output()
            .unwrap();
        assert!(r.status.success(), "{}", String::from_utf8_lossy(&r.stderr));
        if dst == &aar1 {
            std::thread::sleep(std::time::Duration::from_millis(1200));
        }
    }
    assert_eq!(sha256_file(&aar1), sha256_file(&aar2));
}

#[test]
fn t3_pure_rust_apk_patch_16kb_align_v1_v2_v3_sign_verified_by_official_android_sdk() {
    let tmp = WorkspaceTempDir::new("apk_patch").unwrap();
    let manifest = r#"<?xml version="1.0" encoding="utf-8"?>
<manifest xmlns:android="http://schemas.android.com/apk/res/android"
    package="com.example.questopenxrgame"
    android:versionCode="1"
    android:versionName="1.0">
    <uses-sdk android:minSdkVersion="29" android:targetSdkVersion="32" />
    <application android:label="QuestTest" android:extractNativeLibs="false">
        <activity android:name=".MainActivity" android:exported="true">
            <intent-filter>
                <action android:name="android.intent.action.MAIN" />
                <category android:name="android.intent.category.LAUNCHER" />
                <category android:name="com.oculus.intent.category.VR" />
            </intent-filter>
        </activity>
    </application>
</manifest>"#;

    let dummy_loader = b"\x7fELFdummyopenxrloader";
    let in_apk = build_synthetic_quest_apk(
        tmp.path(),
        "quest_game",
        manifest,
        &[(
            "lib/arm64-v8a/libopenxr_loader.so",
            dummy_loader,
            CompressionMethod::Stored,
        )],
        true, // Pre-signed with stale v1 jarsigner signature!
    );

    // Run `alvr-xr-packager patch-apk` with EMPTY `PATH` (NO apksigner, zipalign, keytool, or java visible!)
    let patched_apk = tmp.path().join("quest_game_patched.apk");
    let custom_key_cache = tmp.path().join("persistent_debug.pem");
    let patch_out = Command::new(packager_bin_path())
        .env("PATH", "")
        .env("ALVR_QUEST_SIGNING_KEY", &custom_key_cache)
        .args(["patch-apk", "--input"])
        .arg(&in_apk)
        .arg("--output")
        .arg(&patched_apk)
        .arg("--so")
        .arg(android_layer_so_path())
        .arg("--manifest")
        .arg(layer_manifest_json_path())
        .arg("--config")
        .arg(dist_dir().join("alvr_hand_controller.default.json"))
        .output()
        .expect("run alvr-xr-packager patch-apk with empty PATH");
    assert!(
        patch_out.status.success(),
        "pure-Rust patch-apk failed with empty PATH:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&patch_out.stdout),
        String::from_utf8_lossy(&patch_out.stderr)
    );
    assert!(
        custom_key_cache.is_file(),
        "persistent debug signing key should have been created"
    );

    let aapt = find_build_tool("aapt");
    let zipalign = find_build_tool("zipalign");
    let apksigner = find_build_tool("apksigner");

    // 1. Verify patched binary AndroidManifest.xml with official Android `aapt dump xmltree`!
    let xmltree = Command::new(&aapt)
        .args(["dump", "xmltree"])
        .arg(&patched_apk)
        .arg("AndroidManifest.xml")
        .output()
        .expect("aapt dump xmltree");
    assert!(
        xmltree.status.success(),
        "aapt dump xmltree rejected patched AXML: {}",
        String::from_utf8_lossy(&xmltree.stderr)
    );
    let tree = String::from_utf8_lossy(&xmltree.stdout);
    let count_lines = |needle: &str| tree.lines().filter(|l| l.contains(needle)).count();
    assert_eq!(count_lines("com.oculus.permission.HAND_TRACKING"), 1, "xmltree:\n{tree}");
    assert_eq!(
        count_lines("org.khronos.openxr.permission.ACCESS_DETAILED_HAND_DATA"),
        1
    );
    assert_eq!(count_lines("oculus.software.handtracking"), 1);
    assert_eq!(count_lines("com.oculus.handtracking.frequency"), 1);
    assert_eq!(count_lines("com.oculus.handtracking.version"), 1);
    assert!(
        tree.contains("extractNativeLibs(0x010104ea)=(type 0x12)0x0"),
        "extractNativeLibs=false not preserved:\n{tree}"
    );

    // 2. Verify alignment with official `zipalign -c -p 4` AND `zipalign -c -P 16 4`!
    for args in [&["-c", "-p", "4"][..], &["-c", "-P", "16", "4"][..]] {
        let za = Command::new(&zipalign)
            .args(args)
            .arg(&patched_apk)
            .output()
            .expect("zipalign");
        assert!(
            za.status.success(),
            "official zipalign {args:?} failed:\n{}\n{}",
            String::from_utf8_lossy(&za.stdout),
            String::from_utf8_lossy(&za.stderr)
        );
    }

    let mut pzip = ZipArchive::new(File::open(&patched_apk).unwrap()).unwrap();
    {
        let so_entry = pzip
            .by_name("lib/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so")
            .unwrap();
        assert_eq!(so_entry.compression(), CompressionMethod::Stored);
        let off = so_entry.data_start().unwrap();
        assert_eq!(off % 16384, 0, "layer .so offset {off} is NOT 16KB aligned!");
    }
    {
        let res_entry = pzip.by_name("resources.arsc").unwrap();
        assert_eq!(res_entry.compression(), CompressionMethod::Stored);
        assert_eq!(res_entry.data_start().unwrap() % 4, 0);
    }

    // 3. Independently verify our pure-Rust v1/v2/v3 signatures with official Google `apksigner verify`!
    let verify_default = Command::new(&apksigner)
        .args(["verify", "--verbose", "--print-certs"])
        .arg(&patched_apk)
        .output()
        .expect("apksigner verify");
    let vd_str = String::from_utf8_lossy(&verify_default.stdout);
    assert!(
        verify_default.status.success(),
        "official apksigner verify failed:\n{vd_str}\n{}",
        String::from_utf8_lossy(&verify_default.stderr)
    );
    assert!(
        vd_str.contains("Verified using v3 scheme (APK Signature Scheme v3): true"),
        "{vd_str}"
    );

    // `--min-sdk-version 21` forces official apksigner to verify ALL THREE schemes (v1 + v2 + v3)
    // AND checks `X-Android-APK-Signed: 2, 3` and v2->v3 stripping protection attribute `0xbeeff00d`!
    let verify_all = Command::new(&apksigner)
        .args(["verify", "--min-sdk-version", "21", "--verbose"])
        .arg(&patched_apk)
        .output()
        .expect("apksigner verify --min-sdk-version 21");
    let va_str = String::from_utf8_lossy(&verify_all.stdout);
    assert!(
        verify_all.status.success(),
        "official apksigner verify --min-sdk-version 21 failed:\n{va_str}\n{}",
        String::from_utf8_lossy(&verify_all.stderr)
    );
    assert!(va_str.contains("Verified using v1 scheme (JAR signing): true"), "{va_str}");
    assert!(va_str.contains("Verified using v2 scheme (APK Signature Scheme v2): true"), "{va_str}");
    assert!(va_str.contains("Verified using v3 scheme (APK Signature Scheme v3): true"), "{va_str}");

    // 4. Verify pure-Rust `inspect-apk --json` with EMPTY PATH!
    let insp = Command::new(packager_bin_path())
        .env("PATH", "")
        .args(["inspect-apk", "--input"])
        .arg(&patched_apk)
        .arg("--json")
        .output()
        .unwrap();
    assert!(insp.status.success());
    let iv: serde_json::Value = serde_json::from_slice(&insp.stdout).unwrap();
    assert_eq!(iv["has_quest_hand_tracking"], true);
    assert_eq!(iv["layer_so_aligned_16k"], true);
    assert_eq!(iv["signature_verified"], true);
    assert_eq!(iv["signature_verification"]["v1_verified"], true);
    assert_eq!(iv["signature_verification"]["v2_verified"], true);
    assert_eq!(iv["signature_verification"]["v3_verified"], true);
    let cert_fp_1 = iv["signature_verification"]["cert_sha256"].as_str().unwrap().to_owned();

    // 5. Idempotency + side-by-side `--package` rename + persistent cert reuse!
    let renamed_apk = tmp.path().join("quest_game_sidebyside.apk");
    let repatch = Command::new(packager_bin_path())
        .env("PATH", "")
        .env("ALVR_QUEST_SIGNING_KEY", &custom_key_cache)
        .args(["patch-apk", "--input"])
        .arg(&patched_apk)
        .arg("--output")
        .arg(&renamed_apk)
        .arg("--so")
        .arg(android_layer_so_path())
        .arg("--manifest")
        .arg(layer_manifest_json_path())
        .arg("--package")
        .arg("com.example.questopenxrgame.hands")
        .output()
        .unwrap();
    assert!(repatch.status.success(), "{}", String::from_utf8_lossy(&repatch.stderr));

    // Verify renamed APK passes official `aapt`, `zipalign -P 16`, `apksigner`, AND qualified `.MainActivity`!
    let xmltree2 = Command::new(&aapt)
        .args(["dump", "xmltree"])
        .arg(&renamed_apk)
        .arg("AndroidManifest.xml")
        .output()
        .unwrap();
    assert!(xmltree2.status.success());
    let tree2 = String::from_utf8_lossy(&xmltree2.stdout);
    let count_lines2 = |needle: &str| tree2.lines().filter(|l| l.contains(needle)).count();
    assert_eq!(count_lines2("com.oculus.permission.HAND_TRACKING"), 1);
    assert_eq!(count_lines2("oculus.software.handtracking"), 1);
    assert_eq!(count_lines2("com.oculus.handtracking.frequency"), 1);
    assert_eq!(count_lines2("com.oculus.handtracking.version"), 1);
    assert!(tree2.contains(r#"package="com.example.questopenxrgame.hands""#), "{tree2}");
    assert!(
        tree2.contains(r#"com.example.questopenxrgame.MainActivity"#),
        "relative activity .MainActivity was not qualified with original package:\n{tree2}"
    );

    let verify_renamed = Command::new(&apksigner)
        .args(["verify", "--min-sdk-version", "21", "--verbose"])
        .arg(&renamed_apk)
        .output()
        .unwrap();
    assert!(
        verify_renamed.status.success(),
        "renamed APK failed apksigner verify:\n{}",
        String::from_utf8_lossy(&verify_renamed.stderr)
    );

    // Persistent debug key must have produced the exact same cert SHA-256!
    let insp2 = Command::new(packager_bin_path())
        .env("PATH", "")
        .args(["inspect-apk", "--input"])
        .arg(&renamed_apk)
        .arg("--json")
        .output()
        .unwrap();
    let iv2: serde_json::Value = serde_json::from_slice(&insp2.stdout).unwrap();
    assert_eq!(
        iv2["signature_verification"]["cert_sha256"].as_str(),
        Some(cert_fp_1.as_str())
    );

    // 6. Custom keytool-generated PKCS#12 (.p12) keystore interoperability test!
    let user_p12 = tmp.path().join("user_release.p12");
    let kt = Command::new("keytool")
        .args([
            "-genkeypair",
            "-storetype",
            "PKCS12",
            "-keystore",
        ])
        .arg(&user_p12)
        .args([
            "-storepass",
            "userpass99",
            "-keypass",
            "userpass99",
            "-alias",
            "questkey",
            "-keyalg",
            "RSA",
            "-keysize",
            "2048",
            "-validity",
            "365",
            "-dname",
            "CN=CustomKeystore, O=ALVRTest, C=US",
        ])
        .output()
        .expect("keytool");
    assert!(kt.status.success());

    let custom_signed = tmp.path().join("quest_game_custom_p12.apk");
    let p12_out = Command::new(packager_bin_path())
        .env("PATH", "")
        .args(["patch-apk", "--input"])
        .arg(&in_apk)
        .arg("--output")
        .arg(&custom_signed)
        .arg("--so")
        .arg(android_layer_so_path())
        .arg("--manifest")
        .arg(layer_manifest_json_path())
        .arg("--keystore")
        .arg(&user_p12)
        .arg("--ks-pass")
        .arg("userpass99")
        .arg("--ks-alias")
        .arg("questkey")
        .output()
        .unwrap();
    assert!(
        p12_out.status.success(),
        "custom .p12 signing failed: {}",
        String::from_utf8_lossy(&p12_out.stderr)
    );
    let verify_p12 = Command::new(&apksigner)
        .args(["verify", "--min-sdk-version", "21", "--verbose", "--print-certs"])
        .arg(&custom_signed)
        .output()
        .unwrap();
    let vp_str = String::from_utf8_lossy(&verify_p12.stdout);
    assert!(
        verify_p12.status.success() && vp_str.contains("CN=CustomKeystore"),
        "custom .p12 APK failed official apksigner verify:\n{vp_str}\n{}",
        String::from_utf8_lossy(&verify_p12.stderr)
    );
}

#[test]
fn t3_apk_patcher_negative_validation_cases() {
    let tmp = WorkspaceTempDir::new("apk_neg").unwrap();
    let manifest = r#"<?xml version="1.0" encoding="utf-8"?>
<manifest xmlns:android="http://schemas.android.com/apk/res/android" package="com.example.neg">
    <uses-sdk android:minSdkVersion="29" android:targetSdkVersion="32" />
    <application android:label="Neg" />
</manifest>"#;

    // 1. Legacy VrApi-only APK (no libopenxr_loader.so) -> rejected cleanly unless --allow-missing-loader
    let vrapi_apk = build_synthetic_quest_apk(
        tmp.path(),
        "vrapi_only",
        manifest,
        &[("lib/arm64-v8a/libvrapi.so", b"vrapi", CompressionMethod::Stored)],
        false,
    );
    let out1 = Command::new(packager_bin_path())
        .env("PATH", "")
        .args(["patch-apk", "--input"])
        .arg(&vrapi_apk)
        .arg("--output")
        .arg(tmp.path().join("out1.apk"))
        .arg("--so")
        .arg(android_layer_so_path())
        .arg("--manifest")
        .arg(layer_manifest_json_path())
        .output()
        .unwrap();
    assert!(!out1.status.success());
    let err1 = String::from_utf8_lossy(&out1.stderr);
    assert!(err1.contains("libvrapi.so") || err1.contains("OpenXR"), "{err1}");

    // 2. 32-bit-only APK (armeabi-v7a only, no arm64-v8a) -> rejected!
    let arm32_apk = build_synthetic_quest_apk(
        tmp.path(),
        "arm32_only",
        manifest,
        &[(
            "lib/armeabi-v7a/libopenxr_loader.so",
            b"arm32",
            CompressionMethod::Stored,
        )],
        false,
    );
    let out2 = Command::new(packager_bin_path())
        .env("PATH", "")
        .args(["patch-apk", "--input"])
        .arg(&arm32_apk)
        .arg("--output")
        .arg(tmp.path().join("out2.apk"))
        .arg("--so")
        .arg(android_layer_so_path())
        .arg("--manifest")
        .arg(layer_manifest_json_path())
        .output()
        .unwrap();
    assert!(!out2.status.success());

    // 3. Wrong ELF architecture (x86_64 host .so instead of arm64-v8a) -> rejected!
    let out3 = Command::new(packager_bin_path())
        .env("PATH", "")
        .args(["package-aar", "--so"])
        .arg(host_layer_so_path())
        .arg("--manifest")
        .arg(layer_manifest_json_path())
        .arg("--output")
        .arg(tmp.path().join("bad_arch.aar"))
        .output()
        .unwrap();
    assert!(!out3.status.success());
}
