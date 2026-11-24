//! Direct binary-AXML + 16KB-page-aligned ZIP APK patcher for unprotected non-Meta-Store
//! Quest OpenXR APKs.
//!
//! Strictly handles standard OpenXR API layer bundling, Quest hand-tracking manifest entries,
//! page alignment, and v1/v2/v3 APK signing. Contains ZERO DRM, entitlement, or anti-cheat bypasses.

use crate::axml::{patch_quest_hand_tracking_axml, AxmlPatchReport};
use crate::inspect::{
    inspect_elf, validate_layer_manifest_json, EXPECTED_APK_SO_PATH, EXPECTED_LAYER_JSON_ASSET,
};
use alvr_hand_core::LayerConfig;
use anyhow::{bail, Context, Result};
use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::Command;
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipArchive, ZipWriter};

const PAGE_ALIGN_16K: u16 = 16384;
const ALIGN_4B: u16 = 4;

#[derive(Debug, Clone)]
pub struct PatchApkOptions<'a> {
    pub input_apk: &'a Path,
    pub output_apk: &'a Path,
    pub so_path: &'a Path,
    pub manifest_json_path: &'a Path,
    pub config_json_path: Option<&'a Path>,
    pub keystore_path: Option<&'a Path>,
    pub ks_pass: Option<&'a str>,
    pub ks_alias: Option<&'a str>,
    pub skip_sign: bool,
    pub allow_missing_loader: bool,
    pub allow_host_elf: bool,
}

#[derive(Debug, Clone)]
pub struct PatchApkResult {
    pub axml_report: AxmlPatchReport,
    pub signed: bool,
    pub had_openxr_loader: bool,
    pub had_legacy_vrapi: bool,
}

fn is_stale_v1_signature_entry(name: &str) -> bool {
    let upper = name.to_ascii_uppercase();
    if !upper.starts_with("META-INF/") {
        return false;
    }
    // Only strip top-level META-INF signature artifacts (not nested subdirectories like META-INF/services/).
    let rest = &upper["META-INF/".len()..];
    if rest.contains('/') {
        return false;
    }
    rest == "MANIFEST.MF"
        || rest.ends_with(".SF")
        || rest.ends_with(".RSA")
        || rest.ends_with(".DSA")
        || rest.ends_with(".EC")
}

/// Patches an unprotected Quest OpenXR APK in-place (writing to `opts.output_apk`):
/// 1. Validates the layer `.so` ELF64 AArch64 binary and layer JSON manifest.
/// 2. Checks the input APK for binary `AndroidManifest.xml`, `arm64-v8a` / OpenXR loader presence.
/// 3. Patches binary `AndroidManifest.xml` idempotently with Quest + AOSP hand-tracking entries.
/// 4. Rewrites the ZIP preserving existing files, stripping stale v1 `META-INF` signatures,
///    storing `resources.arsc` 4-byte aligned, and storing `lib/arm64-v8a/libXrApiLayer_*.so`
///    uncompressed (`STORED`) with 16 KB (`0x4000`) data-offset alignment.
/// 5. Signs with `apksigner` (APK Signature Scheme v1 + v2 + v3) and runs `apksigner verify`
///    unless `--skip-sign` is specified.
pub fn patch_apk(opts: &PatchApkOptions<'_>) -> Result<PatchApkResult> {
    // 1. Validate layer inputs.
    let so_bytes = fs::read(opts.so_path)
        .with_context(|| format!("failed to read layer .so at {}", opts.so_path.display()))?;
    let elf = inspect_elf(&so_bytes)
        .with_context(|| format!("{} is not a valid ELF64 binary", opts.so_path.display()))?;
    if !opts.allow_host_elf && !elf.is_android_arm64_shared() {
        bail!(
            "expected Android AArch64 shared library (EM_AARCH64, ET_DYN), got {} (e_type={})",
            elf.machine_name,
            elf.e_type
        );
    }

    let manifest_json_bytes = fs::read(opts.manifest_json_path).with_context(|| {
        format!(
            "failed to read layer manifest JSON at {}",
            opts.manifest_json_path.display()
        )
    })?;
    validate_layer_manifest_json(&manifest_json_bytes)
        .context("invalid OpenXR implicit API layer JSON manifest")?;

    let config_bytes = match opts.config_json_path {
        Some(cfg_path) => {
            let raw = fs::read_to_string(cfg_path)
                .with_context(|| format!("failed to read config JSON at {}", cfg_path.display()))?;
            let parsed = LayerConfig::from_json_str(&raw)
                .context("invalid LayerConfig JSON passed to --config")?;
            Some(parsed.to_json_pretty()?.into_bytes())
        }
        None => None,
    };

    // 2. Open and inspect input APK.
    let in_file = File::open(opts.input_apk)
        .with_context(|| format!("failed to open input APK {}", opts.input_apk.display()))?;
    let mut in_zip = ZipArchive::new(in_file)
        .with_context(|| format!("{} is not a valid APK/ZIP", opts.input_apk.display()))?;

    let mut has_manifest = false;
    let mut has_arm64_dir = false;
    let mut has_any_native_libs = false;
    let mut has_openxr_loader = false;
    let mut has_legacy_vrapi = false;

    for i in 0..in_zip.len() {
        let entry = in_zip.by_index(i)?;
        let name = entry.name();
        if name == "AndroidManifest.xml" {
            has_manifest = true;
        }
        if name.starts_with("lib/") && name.ends_with(".so") {
            has_any_native_libs = true;
            if name.starts_with("lib/arm64-v8a/") {
                has_arm64_dir = true;
            }
            if name.contains("openxr") && !name.contains("XrApiLayer") {
                has_openxr_loader = true;
            }
            if name.ends_with("libvrapi.so") {
                has_legacy_vrapi = true;
            }
        }
    }

    if !has_manifest {
        bail!("input APK is missing AndroidManifest.xml");
    }
    if has_any_native_libs && !has_arm64_dir {
        bail!(
            "input APK contains native libraries but no lib/arm64-v8a/ ABI (32-bit-only APKs are incompatible with arm64-v8a Quest OpenXR layers)"
        );
    }
    if !has_openxr_loader && !opts.allow_missing_loader {
        let vrapi_hint = if has_legacy_vrapi {
            " (detected legacy libvrapi.so — legacy VrApi apps do not load OpenXR API layers)"
        } else {
            ""
        };
        bail!(
            "input APK does not appear to contain an OpenXR loader (libopenxr_loader.so){vrapi_hint}; pass --allow-missing-loader to proceed anyway"
        );
    }

    // 3. Patch binary AndroidManifest.xml.
    let orig_manifest = {
        let mut entry = in_zip.by_name("AndroidManifest.xml")?;
        let mut buf = Vec::with_capacity(entry.size() as usize);
        entry.read_to_end(&mut buf)?;
        buf
    };
    let (patched_manifest, axml_report) = patch_quest_hand_tracking_axml(&orig_manifest)
        .context("failed to patch binary AndroidManifest.xml")?;

    // 4. Write new aligned APK cleanly to a temporary sibling file (supports input == output).
    if let Some(parent) = opts.output_apk.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    let tmp_unsigned = sibling_temp_path(opts.output_apk, "unsigned.apk");
    {
        let out_file = File::create(&tmp_unsigned)
            .with_context(|| format!("failed to create {}", tmp_unsigned.display()))?;
        let mut out_zip = ZipWriter::new(out_file);

        // Copy existing entries (skipping stale v1 signatures and paths we are replacing).
        for i in 0..in_zip.len() {
            let mut entry = in_zip.by_index(i)?;
            let name = entry.name().to_owned();
            if entry.is_dir() {
                continue;
            }
            if is_stale_v1_signature_entry(&name)
                || name == "AndroidManifest.xml"
                || name == EXPECTED_APK_SO_PATH
                || name == EXPECTED_LAYER_JSON_ASSET
                || (config_bytes.is_some() && name == "assets/alvr_hand_controller.json")
            {
                continue;
            }

            let mut data = Vec::with_capacity(entry.size() as usize);
            entry.read_to_end(&mut data)?;

            let orig_method = entry.compression();
            let mut file_opts = SimpleFileOptions::default();

            if name == "resources.arsc" {
                // Android 11+ (API 30+) requires resources.arsc uncompressed and 4-byte aligned.
                file_opts = file_opts
                    .compression_method(CompressionMethod::Stored)
                    .with_alignment(ALIGN_4B);
            } else if name.ends_with(".so") && orig_method == CompressionMethod::Stored {
                // Preserve uncompressed .so with 16KB page alignment (satisfies both 4KB and 16KB!).
                file_opts = file_opts
                    .compression_method(CompressionMethod::Stored)
                    .with_alignment(PAGE_ALIGN_16K);
            } else if orig_method == CompressionMethod::Stored {
                file_opts = file_opts
                    .compression_method(CompressionMethod::Stored)
                    .with_alignment(ALIGN_4B);
            } else {
                file_opts = file_opts.compression_method(CompressionMethod::Deflated);
            }

            out_zip.start_file(&name, file_opts)?;
            out_zip.write_all(&data)?;
        }

        // Write patched binary AndroidManifest.xml.
        out_zip.start_file(
            "AndroidManifest.xml",
            SimpleFileOptions::default().compression_method(CompressionMethod::Deflated),
        )?;
        out_zip.write_all(&patched_manifest)?;

        // Write implicit OpenXR layer JSON manifest (STORED, 4-byte aligned).
        out_zip.start_file(
            EXPECTED_LAYER_JSON_ASSET,
            SimpleFileOptions::default()
                .compression_method(CompressionMethod::Stored)
                .with_alignment(ALIGN_4B),
        )?;
        out_zip.write_all(&manifest_json_bytes)?;

        // Optional runtime default config asset.
        if let Some(ref cfg) = config_bytes {
            out_zip.start_file(
                "assets/alvr_hand_controller.json",
                SimpleFileOptions::default()
                    .compression_method(CompressionMethod::Stored)
                    .with_alignment(ALIGN_4B),
            )?;
            out_zip.write_all(cfg)?;
        }

        // Write our layer .so UNCOMPRESSED (STORED) with exact 16KB (0x4000) data-offset alignment!
        // Satisfies both Android 15+ 16KB page alignment AND Quest `zipalign -c -p 4` (4KB).
        out_zip.start_file(
            EXPECTED_APK_SO_PATH,
            SimpleFileOptions::default()
                .compression_method(CompressionMethod::Stored)
                .with_alignment(PAGE_ALIGN_16K),
        )?;
        out_zip.write_all(&so_bytes)?;

        out_zip.finish()?;
    }

    // Verify 4KB page alignment with `zipalign -c -p 4` if installed.
    if let Ok(status) = Command::new("zipalign")
        .args(["-c", "-p", "4"])
        .arg(&tmp_unsigned)
        .output()
    {
        if !status.status.success() {
            let _ = fs::remove_file(&tmp_unsigned);
            bail!(
                "internal alignment check failed (`zipalign -c -p 4`): {}",
                String::from_utf8_lossy(&status.stderr)
            );
        }
    }

    // 5. Sign APK via `apksigner` (v1 + v2 + v3) unless `--skip-sign`.
    if opts.skip_sign {
        fs::rename(&tmp_unsigned, opts.output_apk).with_context(|| {
            format!(
                "failed to move {} to {}",
                tmp_unsigned.display(),
                opts.output_apk.display()
            )
        })?;
        return Ok(PatchApkResult {
            axml_report,
            signed: false,
            had_openxr_loader: has_openxr_loader,
            had_legacy_vrapi: has_legacy_vrapi,
        });
    }

    let (keystore_path, ks_pass, ks_alias, temp_ks) = match opts.keystore_path {
        Some(ks) => {
            let pass = opts.ks_pass.unwrap_or("android").to_owned();
            let alias = opts.ks_alias.map(str::to_owned);
            (ks.to_path_buf(), pass, alias, None)
        }
        None => {
            let (ks, pass, alias) = ensure_debug_keystore(opts.output_apk)?;
            (ks.clone(), pass, Some(alias), Some(ks))
        }
    };

    let tmp_signed = sibling_temp_path(opts.output_apk, "signed.apk");
    let mut sign_cmd = Command::new("apksigner");
    sign_cmd
        .arg("sign")
        .arg("--v1-signing-enabled")
        .arg("true")
        .arg("--v2-signing-enabled")
        .arg("true")
        .arg("--v3-signing-enabled")
        .arg("true")
        .arg("--ks")
        .arg(&keystore_path)
        .arg("--ks-pass")
        .arg(format!("pass:{ks_pass}"));
    if let Some(ref alias) = ks_alias {
        sign_cmd.arg("--ks-key-alias").arg(alias);
    }
    sign_cmd
        .arg("--out")
        .arg(&tmp_signed)
        .arg(&tmp_unsigned);

    let sign_out = sign_cmd
        .output()
        .context("failed to execute `apksigner` (ensure Android build-tools / apksigner is installed on PATH)")?;

    let _ = fs::remove_file(&tmp_unsigned);
    if let Some(ref tks) = temp_ks {
        let _ = fs::remove_file(tks);
    }

    if !sign_out.status.success() {
        let _ = fs::remove_file(&tmp_signed);
        bail!(
            "apksigner sign failed:\n{}{}",
            String::from_utf8_lossy(&sign_out.stdout),
            String::from_utf8_lossy(&sign_out.stderr)
        );
    }

    // Verify the freshly signed APK with `apksigner verify`.
    let verify_out = Command::new("apksigner")
        .args(["verify", "--verbose"])
        .arg(&tmp_signed)
        .output()
        .context("failed to execute `apksigner verify`")?;
    if !verify_out.status.success() {
        let _ = fs::remove_file(&tmp_signed);
        bail!(
            "apksigner verify failed:\n{}{}",
            String::from_utf8_lossy(&verify_out.stdout),
            String::from_utf8_lossy(&verify_out.stderr)
        );
    }

    fs::rename(&tmp_signed, opts.output_apk).with_context(|| {
        format!(
            "failed to move signed APK to {}",
            opts.output_apk.display()
        )
    })?;

    Ok(PatchApkResult {
        axml_report,
        signed: true,
        had_openxr_loader: has_openxr_loader,
        had_legacy_vrapi: has_legacy_vrapi,
    })
}

fn sibling_temp_path(target: &Path, suffix: &str) -> PathBuf {
    let pid = std::process::id();
    let file_name = target
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("output");
    target.with_file_name(format!(".{file_name}.{pid}.{suffix}"))
}

fn ensure_debug_keystore(near_output: &Path) -> Result<(PathBuf, String, String)> {
    let ks_path = sibling_temp_path(near_output, "debug.p12");
    if ks_path.exists() {
        let _ = fs::remove_file(&ks_path);
    }
    let pass = "android".to_owned();
    let alias = "alvrdebug".to_owned();

    let out = Command::new("keytool")
        .args([
            "-genkeypair",
            "-v",
            "-storetype",
            "PKCS12",
            "-keystore",
        ])
        .arg(&ks_path)
        .args([
            "-storepass",
            &pass,
            "-keypass",
            &pass,
            "-alias",
            &alias,
            "-keyalg",
            "RSA",
            "-keysize",
            "2048",
            "-validity",
            "10000",
            "-dname",
            "CN=ALVR Hand Layer Debug, O=ALVR, C=US",
        ])
        .output()
        .context("failed to invoke `keytool` to generate debug keystore")?;

    if !out.status.success() {
        let _ = fs::remove_file(&ks_path);
        bail!(
            "keytool failed generating debug keystore:\n{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
    }

    Ok((ks_path, pass, alias))
}
