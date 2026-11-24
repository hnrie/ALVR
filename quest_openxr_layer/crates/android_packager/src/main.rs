//! `alvr-xr-packager` CLI:
//!   - `package-aar` — Build a deterministic Android `.aar` bundling the OpenXR API layer.
//!   - `patch-apk`   — Inject the OpenXR API layer and Quest hand-tracking manifest entries into
//!                     an unprotected non-Meta-Store OpenXR APK, 16KB-align, and sign (v1/v2/v3).
//!   - `inspect-apk` — Inspect an `.apk` or `.aar` for ELF arch/alignment, manifest entries,
//!                     layer JSON validity, and APK signature verification.

use android_packager::{
    inspect_archive, package_aar, patch_apk, PackageAarOptions, PatchApkOptions,
};
use anyhow::{bail, Result};
use pico_args::Arguments;
use std::path::PathBuf;
use std::process::ExitCode;

fn print_usage() {
    eprintln!(
        "\
alvr-xr-packager — Quest OpenXR Hand-to-Touch-Controller API Layer Packager

USAGE:
  alvr-xr-packager package-aar --so <path> --manifest <path> --output <out.aar> [--config <json>] [--allow-host-elf]
  alvr-xr-packager patch-apk   --input <in.apk> --output <out.apk> --so <path> --manifest <path>
                               [--config <json>] [--keystore <ks>] [--ks-pass <pass>] [--ks-alias <alias>]
                               [--skip-sign] [--allow-missing-loader] [--allow-host-elf]
  alvr-xr-packager inspect-apk --input <file.apk|file.aar> [--json]

NOTE:
  patch-apk is strictly for unprotected non-Meta-Store OpenXR APKs. It performs standard OpenXR
  implicit layer bundling, hand-tracking manifest updates, 16KB alignment, and signing only —
  it contains zero DRM, entitlement, or anti-cheat bypasses."
    );
}

fn run() -> Result<()> {
    let mut args = Arguments::from_env();
    let Some(subcmd) = args.subcommand()? else {
        print_usage();
        bail!("missing subcommand (expected package-aar, patch-apk, or inspect-apk)");
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
            let skip_sign = args.contains("--skip-sign");
            let allow_missing_loader = args.contains("--allow-missing-loader");
            let allow_host_elf = args.contains("--allow-host-elf");
            let rest = args.finish();
            if !rest.is_empty() {
                bail!("unexpected extra arguments: {rest:?}");
            }

            let res = patch_apk(&PatchApkOptions {
                input_apk: &input,
                output_apk: &output,
                so_path: &so,
                manifest_json_path: &manifest,
                config_json_path: config.as_deref(),
                keystore_path: keystore.as_deref(),
                ks_pass: ks_pass.as_deref(),
                ks_alias: ks_alias.as_deref(),
                skip_sign,
                allow_missing_loader,
                allow_host_elf,
            })?;
            println!(
                "Patched APK: {} (signed={}, added_permissions={}, added_features={}, added_meta_data={})",
                output.display(),
                res.signed,
                res.axml_report.added_permissions.len(),
                res.axml_report.added_features.len(),
                res.axml_report.added_meta_data.len(),
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
                    "OpenXR loader / VrApi: openxr_loader={}, legacy_vrapi={}",
                    report.has_openxr_loader, report.has_legacy_vrapi
                );
                if let Some(sig_ok) = report.signature_verified {
                    println!("APK signature valid:   {sig_ok}");
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
