//! `alvr-xr-packager` CLI (100% pure Rust — zero `apksigner`, `zipalign`, `keytool`, or Java needed):
//!   - `package-aar` — Build a deterministic Android `.aar` bundling the OpenXR API layer.
//!   - `patch-apk`   — Inject the OpenXR API layer and Quest hand-tracking manifest entries into
//!                     an unprotected non-Meta-Store OpenXR APK, 16 KB align, and sign (v1/v2/v3).
//!   - `align-apk`   — 16 KB page-align an APK in pure Rust (`zipalign -P 16 4` replacement).
//!   - `sign-apk`    — 16 KB align and sign an APK with APK Signature Scheme v1 + v2 + v3 in pure Rust.
//!   - `inspect-apk` — Inspect an `.apk` or `.aar` for ELF arch/alignment, manifest entries,
//!                     layer JSON validity, 16 KB alignment, and pure-Rust v1/v2/v3 signatures.

use android_packager::{
    align_and_sign_apk, align_apk, inspect_archive, package_aar, patch_apk_with_bytes,
    verify_apk_alignment, verify_apk_signatures, PackageAarOptions, PatchApkBytesOptions,
    SigningKeyPair,
};
use anyhow::{bail, Context, Result};
use pico_args::Arguments;
use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;

fn print_usage() {
    eprintln!(
        "\
alvr-xr-packager — Self-Contained Pure-Rust Quest OpenXR Hand Layer Packager & Signer

USAGE:
  alvr-xr-packager package-aar --so <path> --manifest <path> --output <out.aar> [--config <json>] [--allow-host-elf]
  alvr-xr-packager patch-apk   --input <in.apk> --output <out.apk> --so <path> --manifest <path>
                               [--config <json>] [--keystore <p12|jks|pem>] [--ks-pass <pass>] [--ks-alias <alias>]
                               [--ephemeral-key] [--package <new.package.name>] [--skip-sign]
                               [--allow-missing-loader] [--allow-host-elf]
  alvr-xr-packager align-apk   --input <in.apk> --output <out.apk>
  alvr-xr-packager sign-apk    --input <in.apk> --output <out.apk>
                               [--keystore <p12|jks|pem>] [--ks-pass <pass>] [--ks-alias <alias>] [--ephemeral-key]
  alvr-xr-packager inspect-apk --input <file.apk|file.aar> [--json]

NOTE:
  Zero external tools required (no Java, keytool, zipalign, or apksigner needed).
  patch-apk is strictly for unprotected non-Meta-Store OpenXR APKs. It performs standard OpenXR
  implicit layer bundling, hand-tracking manifest updates, 16 KB alignment, and v1/v2/v3 signing
  only — it contains zero DRM, entitlement, or anti-cheat bypasses."
    );
}

fn run() -> Result<()> {
    let mut args = Arguments::from_env();
    let Some(subcmd) = args.subcommand()? else {
        print_usage();
        bail!("missing subcommand (expected package-aar, patch-apk, align-apk, sign-apk, or inspect-apk)");
    };

    match subcmd.as_str() {
        "package-aar" => {
            let so: PathBuf = args.value_from_str("--so")?;
            let manifest: PathBuf = args.value_from_str("--manifest")?;
            let output: PathBuf = args.value_from_str("--output")?;
            let config: Option<PathBuf> = args.opt_value_from_str("--config")?;
            let allow_host_elf = args.contains("--allow-host-elf");
            let rest = args.finish();
            if !rest.is_empty() {
                bail!("unexpected extra arguments: {rest:?}");
            }

            package_aar(&PackageAarOptions {
                so_path: &so,
                manifest_json_path: &manifest,
                output_path: &output,
                config_json_path: config.as_deref(),
                allow_host_elf,
            })?;
            println!("Built AAR: {}", output.display());
        }
        "patch-apk" => {
            let input: PathBuf = args.value_from_str("--input")?;
            let output: PathBuf = args.value_from_str("--output")?;
            let so: PathBuf = args.value_from_str("--so")?;
            let manifest: PathBuf = args.value_from_str("--manifest")?;
            let config: Option<PathBuf> = args.opt_value_from_str("--config")?;
            let keystore: Option<PathBuf> = args.opt_value_from_str("--keystore")?;
            let ks_pass: Option<String> = args.opt_value_from_str("--ks-pass")?;
            let ks_alias: Option<String> = args.opt_value_from_str("--ks-alias")?;
            let package: Option<String> = args.opt_value_from_str("--package")?;
            let ephemeral_key = args.contains("--ephemeral-key");
            let skip_sign = args.contains("--skip-sign");
            let allow_missing_loader = args.contains("--allow-missing-loader");
            let allow_host_elf = args.contains("--allow-host-elf");
            let rest = args.finish();
            if !rest.is_empty() {
                bail!("unexpected extra arguments: {rest:?}");
            }

            let so_bytes = fs::read(&so)
                .with_context(|| format!("failed to read layer .so at {}", so.display()))?;
            let manifest_bytes = fs::read(&manifest)
                .with_context(|| format!("failed to read manifest at {}", manifest.display()))?;
            let config_bytes = match config {
                Some(ref p) => Some(
                    fs::read(p)
                        .with_context(|| format!("failed to read config at {}", p.display()))?,
                ),
                None => None,
            };

            let res = patch_apk_with_bytes(
                &PatchApkBytesOptions {
                    input_apk: &input,
                    output_apk: &output,
                    so_bytes: &so_bytes,
                    manifest_json_bytes: &manifest_bytes,
                    config_json_bytes: config_bytes.as_deref(),
                    keystore_path: keystore.as_deref(),
                    ks_pass: ks_pass.as_deref(),
                    ks_alias: ks_alias.as_deref(),
                    ephemeral_key,
                    package: package.as_deref(),
                    skip_sign,
                    allow_missing_loader,
                    allow_host_elf,
                },
                |_| {},
            )?;
            println!(
                "Patched APK: {} (signed={}, aligned_16k={}, added_permissions={}, added_features={}, added_meta_data={})",
                output.display(),
                res.signed,
                res.alignment.is_fully_aligned_16k(),
                res.axml_report.added_permissions.len(),
                res.axml_report.added_features.len(),
                res.axml_report.added_meta_data.len(),
            );
            if let Some(ref sig) = res.sign_summary {
                println!(
                    "Signed (v1/v2/v3): cert_sha256={} ({})",
                    sig.cert_sha256, sig.key_source
                );
            }
        }
        "align-apk" => {
            let input: PathBuf = args.value_from_str("--input")?;
            let output: PathBuf = args.value_from_str("--output")?;
            let rest = args.finish();
            if !rest.is_empty() {
                bail!("unexpected extra arguments: {rest:?}");
            }
            let report = align_apk(&input, &output)?;
            println!(
                "Aligned APK: {} (aligned_16k={}, checked_so={}, checked_stored={})",
                output.display(),
                report.is_fully_aligned_16k(),
                report.checked_so_entries,
                report.checked_stored_entries
            );
        }
        "sign-apk" => {
            let input: PathBuf = args.value_from_str("--input")?;
            let output: PathBuf = args.value_from_str("--output")?;
            let keystore: Option<PathBuf> = args.opt_value_from_str("--keystore")?;
            let ks_pass: Option<String> = args.opt_value_from_str("--ks-pass")?;
            let ks_alias: Option<String> = args.opt_value_from_str("--ks-alias")?;
            let ephemeral_key = args.contains("--ephemeral-key");
            let rest = args.finish();
            if !rest.is_empty() {
                bail!("unexpected extra arguments: {rest:?}");
            }
            let (kp, source) = match keystore {
                Some(ref p) => (
                    SigningKeyPair::from_keystore_file(p, ks_pass.as_deref(), ks_alias.as_deref())?,
                    format!("custom keystore ({})", p.display()),
                ),
                None if ephemeral_key => (
                    SigningKeyPair::generate_ephemeral_debug()?,
                    "ephemeral RSA-2048 debug key".to_owned(),
                ),
                None => SigningKeyPair::load_or_create_persistent_debug()?,
            };
            let summary = align_and_sign_apk(&input, &output, &kp, source)?;
            let align = verify_apk_alignment(&output)?;
            let ver = verify_apk_signatures(&output)?;
            if !align.is_fully_aligned_16k() || !ver.valid {
                bail!(
                    "post-sign verification failed (aligned_16k={}, sig_valid={}, errors={:?})",
                    align.is_fully_aligned_16k(),
                    ver.valid,
                    ver.errors
                );
            }
            println!(
                "Signed APK: {} (v1={}, v2={}, v3={}, cert_sha256={})",
                output.display(),
                ver.v1_verified,
                ver.v2_verified,
                ver.v3_verified,
                summary.cert_sha256
            );
        }
        "inspect-apk" => {
            let input: PathBuf = args.value_from_str("--input")?;
            let json = args.contains("--json");
            let rest = args.finish();
            if !rest.is_empty() {
                bail!("unexpected extra arguments: {rest:?}");
            }

            let report = inspect_archive(&input)?;
            if json {
                println!("{}", serde_json::to_string_pretty(&report)?);
            } else {
                println!("Archive kind:          {}", report.archive_kind);
                println!("Package:               {:?}", report.manifest.package);
                println!("Quest hand tracking:   {}", report.has_quest_hand_tracking);
                println!(
                    "Layer JSON manifest:   present={}, valid={}, name={:?}",
                    report.has_layer_manifest,
                    report.layer_manifest_valid,
                    report.layer_manifest_name
                );
                println!(
                    "Layer .so:             path={:?}, stored={}, offset={:?}, aligned_4k={}, aligned_16k={}",
                    report.layer_so_path,
                    report.layer_so_stored,
                    report.layer_so_data_offset,
                    report.layer_so_aligned_4k,
                    report.layer_so_aligned_16k
                );
                if let Some(ref elf) = report.layer_so_elf {
                    println!(
                        "Layer ELF:             {} (soname={:?}, pt_load_align=0x{:x}, needed={:?}, exports={:?})",
                        elf.machine_name,
                        elf.soname,
                        elf.min_pt_load_align,
                        elf.needed,
                        elf.exported_global_symbols
                    );
                }
                println!(
                    "OpenXR loader / VrApi: openxr_loader={}, legacy_vrapi={}, arm64={}",
                    report.has_openxr_loader, report.has_legacy_vrapi, report.has_arm64_abi
                );
                if let Some(ref al) = report.alignment {
                    println!(
                        "APK alignment:         16kb_ok={}, 4kb_so={}, 4b_stored={}, resources_arsc_ok={}",
                        al.is_fully_aligned_16k(),
                        al.aligned_4k_so,
                        al.aligned_4b,
                        al.resources_arsc_ok
                    );
                }
                if let Some(sig_ok) = report.signature_verified {
                    println!(
                        "APK signature valid:   {} ({})",
                        sig_ok,
                        report.signature_detail.as_deref().unwrap_or("")
                    );
                }
            }
        }
        "-h" | "--help" | "help" => {
            print_usage();
        }
        other => {
            print_usage();
            bail!("unknown subcommand '{other}'");
        }
    }

    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("error: {err:#}");
            ExitCode::FAILURE
        }
    }
}
