//! Direct binary-AXML + pure-Rust 16 KB page-aligned ZIP + v1/v2/v3 APK patcher for
//! unprotected non-Meta-Store Quest OpenXR APKs.
//!
//! Strictly handles standard OpenXR API layer bundling, Quest hand-tracking manifest entries,
//! 16 KB page alignment, and APK Signature Scheme v1/v2/v3 signing with zero external tools
//! (`apksigner`, `zipalign`, `keytool`, Java are NOT required).
//! Contains ZERO DRM, entitlement, or anti-cheat bypasses.

use crate::align::{
    parse_apk_zip_layout, read_entry_uncompressed, verify_apk_alignment, AlignmentReport,
    StreamingApkZipWriter, ALIGN_16K, ALIGN_4B,
};
use crate::axml::{
    inspect_axml, patch_quest_hand_tracking_axml, rename_package_axml, AxmlPatchReport,
};
use crate::inspect::{
    inspect_elf, validate_layer_manifest_json, EXPECTED_APK_SO_PATH, EXPECTED_LAYER_JSON_ASSET,
};
use crate::sign::{
    finalize_signed_apk_stream, is_v1_signature_entry, verify_apk_signatures, ApkSignSummary,
    ApkSignatureVerification, SigningKeyPair,
};
use alvr_hand_core::LayerConfig;
use anyhow::{bail, Context, Result};
use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};

/// File-path options for [`patch_apk`].
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
    pub package: Option<&'a str>,
    pub skip_sign: bool,
    pub allow_missing_loader: bool,
    pub allow_host_elf: bool,
}

/// Embedded-bytes options for [`patch_apk_with_bytes`] (used by the self-contained Desktop GUI
/// and library callers that embed the prebuilt Quest `.so` and layer `.json`).
#[derive(Debug, Clone)]
pub struct PatchApkBytesOptions<'a> {
    pub input_apk: &'a Path,
    pub output_apk: &'a Path,
    pub so_bytes: &'a [u8],
    pub manifest_json_bytes: &'a [u8],
    pub config_json_bytes: Option<&'a [u8]>,
    pub keystore_path: Option<&'a Path>,
    pub ks_pass: Option<&'a str>,
    pub ks_alias: Option<&'a str>,
    pub ephemeral_key: bool,
    pub package: Option<&'a str>,
    pub skip_sign: bool,
    pub allow_missing_loader: bool,
    pub allow_host_elf: bool,
}

/// Progress update emitted during [`patch_apk_with_bytes`].
#[derive(Debug, Clone)]
pub struct PatchProgress {
    pub fraction: f32,
    pub message: String,
}

#[derive(Debug, Clone)]
pub struct PatchApkResult {
    pub axml_report: AxmlPatchReport,
    pub signed: bool,
    pub had_openxr_loader: bool,
    pub had_legacy_vrapi: bool,
    pub original_package: Option<String>,
    pub final_package: Option<String>,
    pub alignment: AlignmentReport,
    pub sign_summary: Option<ApkSignSummary>,
    pub signature_verification: Option<ApkSignatureVerification>,
}

/// Patches an unprotected Quest OpenXR APK using layer assets read from disk.
pub fn patch_apk(opts: &PatchApkOptions<'_>) -> Result<PatchApkResult> {
    let so_bytes = fs::read(opts.so_path)
        .with_context(|| format!("failed to read layer .so at {}", opts.so_path.display()))?;
    let manifest_json_bytes = fs::read(opts.manifest_json_path).with_context(|| {
        format!(
            "failed to read layer manifest JSON at {}",
            opts.manifest_json_path.display()
        )
    })?;
    let config_bytes = match opts.config_json_path {
        Some(cfg_path) => Some(
            fs::read(cfg_path)
                .with_context(|| format!("failed to read config JSON at {}", cfg_path.display()))?,
        ),
        None => None,
    };

    patch_apk_with_bytes(
        &PatchApkBytesOptions {
            input_apk: opts.input_apk,
            output_apk: opts.output_apk,
            so_bytes: &so_bytes,
            manifest_json_bytes: &manifest_json_bytes,
            config_json_bytes: config_bytes.as_deref(),
            keystore_path: opts.keystore_path,
            ks_pass: opts.ks_pass,
            ks_alias: opts.ks_alias,
            ephemeral_key: false,
            package: opts.package,
            skip_sign: opts.skip_sign,
            allow_missing_loader: opts.allow_missing_loader,
            allow_host_elf: opts.allow_host_elf,
        },
        |_| {},
    )
}

/// Patches an unprotected Quest OpenXR APK in a single streaming pass using in-memory layer assets:
/// 1. Validates the layer `.so` ELF64 AArch64 binary and OpenXR layer JSON manifest.
/// 2. Inspects the input APK for binary `AndroidManifest.xml`, `arm64-v8a`, and OpenXR loader.
/// 3. Patches binary `AndroidManifest.xml` idempotently with Quest + Khronos hand-tracking entries
///    (and optionally renames the package for side-by-side install while qualifying relative classes).
/// 4. Streams untouched entries without re-compression into a 16 KB page-aligned ZIP, injecting the
///    layer `.so` (`STORED`, 16 KB aligned), layer `.json` (`STORED`, 4-byte aligned), and optional
///    default runtime config (`assets/alvr_hand_controller.json`).
/// 5. Signs with pure-Rust APK Signature Scheme v1 + v2 + v3 and self-verifies alignment and all
///    three signature schemes unless `skip_sign` is set.
pub fn patch_apk_with_bytes<F>(
    opts: &PatchApkBytesOptions<'_>,
    mut on_progress: F,
) -> Result<PatchApkResult>
where
    F: FnMut(PatchProgress),
{
    let mut emit = |fraction: f32, msg: &str| {
        on_progress(PatchProgress {
            fraction: fraction.clamp(0.0, 1.0),
            message: msg.to_owned(),
        });
    };

    emit(0.02, "Validating OpenXR layer binary and manifest...");
    let elf = inspect_elf(opts.so_bytes).context("layer .so is not a valid ELF64 binary")?;
    if !opts.allow_host_elf && !elf.is_android_arm64_shared() {
        bail!(
            "expected Android AArch64 shared library (EM_AARCH64, ET_DYN), got {} (e_type={})",
            elf.machine_name,
            elf.e_type
        );
    }
    validate_layer_manifest_json(opts.manifest_json_bytes)
        .context("invalid OpenXR implicit API layer JSON manifest")?;

    let validated_config_bytes = match opts.config_json_bytes {
        Some(raw_bytes) => {
            let raw = std::str::from_utf8(raw_bytes)
                .context("LayerConfig JSON is not valid UTF-8")?;
            let parsed = LayerConfig::from_json_str(raw)
                .context("invalid LayerConfig JSON passed for embedding")?;
            Some(parsed.to_json_pretty()?.into_bytes())
        }
        None => None,
    };

    // 2. Open and parse input APK layout.
    emit(0.08, "Inspecting input APK structure...");
    let mut in_file = File::open(opts.input_apk)
        .with_context(|| format!("failed to open input APK {}", opts.input_apk.display()))?;
    let layout = parse_apk_zip_layout(&mut in_file)
        .with_context(|| format!("{} is not a valid APK/ZIP", opts.input_apk.display()))?;

    let mut manifest_entry_idx: Option<usize> = None;
    let mut has_arm64_dir = false;
    let mut has_any_native_libs = false;
    let mut has_openxr_loader = false;
    let mut has_legacy_vrapi = false;

    for (i, entry) in layout.entries.iter().enumerate() {
        let name = entry.name.as_str();
        if name == "AndroidManifest.xml" {
            manifest_entry_idx = Some(i);
        }
        if name.starts_with("lib/") && name.ends_with(".so") {
            has_any_native_libs = true;
            if name.starts_with("lib/arm64-v8a/") {
                has_arm64_dir = true;
            }
            let lower = name.to_ascii_lowercase();
            if lower.contains("openxr") && !lower.contains("xrapilayer") {
                has_openxr_loader = true;
            }
            if lower.ends_with("libvrapi.so") {
                has_legacy_vrapi = true;
            }
        }
    }

    let manifest_idx = manifest_entry_idx
        .context("input APK is missing AndroidManifest.xml")?;
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
    emit(0.14, "Patching binary AndroidManifest.xml...");
    let orig_manifest = read_entry_uncompressed(&mut in_file, &layout.entries[manifest_idx])
        .context("failed to read AndroidManifest.xml from input APK")?;
    let orig_inspection = inspect_axml(&orig_manifest)
        .context("failed to parse binary AndroidManifest.xml")?;
    let original_package = orig_inspection.package.clone();

    let (mut patched_manifest, axml_report) = patch_quest_hand_tracking_axml(&orig_manifest)
        .context("failed to patch binary AndroidManifest.xml")?;
    if let Some(newpkg) = opts.package {
        if !newpkg.trim().is_empty() {
            patched_manifest = rename_package_axml(&patched_manifest, newpkg.trim())
                .context("failed to rename package in AndroidManifest.xml")?;
        }
    }
    let final_package = inspect_axml(&patched_manifest)
        .ok()
        .and_then(|m| m.package);

    // 4. Prepare signing keypair before writing (if signing is enabled) so errors fail fast.
    let signing_creds = if opts.skip_sign {
        None
    } else {
        emit(0.18, "Loading signing credentials...");
        let (kp, source) = match opts.keystore_path {
            Some(ks_path) => {
                let kp = SigningKeyPair::from_keystore_file(ks_path, opts.ks_pass, opts.ks_alias)?;
                (kp, format!("custom keystore ({})", ks_path.display()))
            }
            None if opts.ephemeral_key => (
                SigningKeyPair::generate_ephemeral_debug()?,
                "ephemeral RSA-2048 debug key".to_owned(),
            ),
            None => SigningKeyPair::load_or_create_persistent_debug()?,
        };
        Some((kp, source))
    };

    // 5. Stream-build 16 KB aligned output APK to a temporary sibling file (supports input == output).
    if let Some(parent) = opts.output_apk.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    let tmp_out = sibling_temp_path(opts.output_apk, "patching.apk");
    let build_res = (|| -> Result<Option<ApkSignSummary>> {
        let out_file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(&tmp_out)
            .with_context(|| format!("failed to create {}", tmp_out.display()))?;
        let mut writer = StreamingApkZipWriter::new(out_file)?;

        let total_entries = layout.entries.len().max(1);
        for (idx, entry) in layout.entries.iter().enumerate() {
            if entry.is_dir() {
                continue;
            }
            let name = entry.name.as_str();
            if is_v1_signature_entry(name)
                || name == "AndroidManifest.xml"
                || name == EXPECTED_APK_SO_PATH
                || name == EXPECTED_LAYER_JSON_ASSET
                || (validated_config_bytes.is_some() && name == "assets/alvr_hand_controller.json")
            {
                continue;
            }
            writer.copy_entry_streaming(&mut in_file, entry)?;
            if idx % 64 == 0 || idx + 1 == total_entries {
                let frac = 0.22 + 0.50 * ((idx + 1) as f32 / total_entries as f32);
                emit(
                    frac,
                    &format!("Streaming & 16 KB aligning APK entries ({}/{})...", idx + 1, total_entries),
                );
            }
        }

        emit(0.75, "Injecting patched manifest, OpenXR layer JSON, and 16 KB aligned .so...");
        writer.write_deflated_bytes("AndroidManifest.xml", &patched_manifest)?;
        writer.write_stored_bytes(
            EXPECTED_LAYER_JSON_ASSET,
            opts.manifest_json_bytes,
            ALIGN_4B,
        )?;
        if let Some(ref cfg) = validated_config_bytes {
            writer.write_stored_bytes("assets/alvr_hand_controller.json", cfg, ALIGN_4B)?;
        }
        writer.write_stored_bytes(EXPECTED_APK_SO_PATH, opts.so_bytes, ALIGN_16K)?;

        if let Some((ref keypair, ref key_source)) = signing_creds {
            emit(0.84, "Signing APK with pure-Rust Scheme v1 + v2 + v3...");
            let (mut out_file, summary) =
                finalize_signed_apk_stream(writer, keypair, key_source.clone())?;
            out_file.flush()?;
            Ok(Some(summary))
        } else {
            let (mut out_file, _) = writer.finish_unsigned()?;
            out_file.flush()?;
            Ok(None)
        }
    })();

    let sign_summary = match build_res {
        Ok(s) => s,
        Err(err) => {
            let _ = fs::remove_file(&tmp_out);
            return Err(err);
        }
    };

    // 6. Self-verify 16 KB alignment and v1/v2/v3 signatures in pure Rust before committing output.
    emit(0.92, "Verifying 16 KB page alignment...");
    let alignment = match verify_apk_alignment(&tmp_out) {
        Ok(rep) if rep.is_fully_aligned_16k() => rep,
        Ok(rep) => {
            let _ = fs::remove_file(&tmp_out);
            bail!(
                "internal 16 KB alignment check failed: {:?}",
                rep.misaligned_entries
            );
        }
        Err(err) => {
            let _ = fs::remove_file(&tmp_out);
            return Err(err);
        }
    };

    let signature_verification = if sign_summary.is_some() {
        emit(0.96, "Verifying v1 + v2 + v3 APK signatures...");
        match verify_apk_signatures(&tmp_out) {
            Ok(ver) if ver.valid && ver.v1_verified && ver.v2_verified && ver.v3_verified => {
                Some(ver)
            }
            Ok(ver) => {
                let _ = fs::remove_file(&tmp_out);
                bail!(
                    "internal APK signature verification failed (v1={}, v2={}, v3={}): {:?}",
                    ver.v1_verified,
                    ver.v2_verified,
                    ver.v3_verified,
                    ver.errors
                );
            }
            Err(err) => {
                let _ = fs::remove_file(&tmp_out);
                return Err(err);
            }
        }
    } else {
        None
    };

    // Drop input file handle before atomic rename (important on Windows when input == output!).
    drop(in_file);
    #[cfg(target_os = "windows")]
    {
        if opts.output_apk.exists() {
            let _ = fs::remove_file(opts.output_apk);
        }
    }
    fs::rename(&tmp_out, opts.output_apk).with_context(|| {
        format!(
            "failed to move patched APK from {} to {}",
            tmp_out.display(),
            opts.output_apk.display()
        )
    })?;

    emit(1.0, "Done!");
    Ok(PatchApkResult {
        axml_report,
        signed: sign_summary.is_some(),
        had_openxr_loader: has_openxr_loader,
        had_legacy_vrapi: has_legacy_vrapi,
        original_package,
        final_package,
        alignment,
        sign_summary,
        signature_verification,
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
