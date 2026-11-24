//! Deterministic Android Library Archive (`.aar`) builder for the Quest OpenXR API layer.

use crate::inspect::{
    inspect_elf, validate_layer_manifest_json, EXPECTED_AAR_SO_PATH, EXPECTED_LAYER_JSON_ASSET,
};
use alvr_hand_core::LayerConfig;
use anyhow::{bail, Context, Result};
use std::fs::{self, File};
use std::io::{Cursor, Write};
use std::path::Path;
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, DateTime, ZipWriter};

/// Canonical AAR plain-text `AndroidManifest.xml` matching Khronos OpenXR AAR conventions
/// (explicit `<uses-sdk targetSdkVersion>` prevents legacy permission injection by Manifest Merger;
/// `oculus.software.handtracking android:required="false"` avoids forcing hand-tracking requirement).
pub const AAR_ANDROID_MANIFEST_XML: &str = r#"<?xml version="1.0" encoding="utf-8"?>
<manifest xmlns:android="http://schemas.android.com/apk/res/android"
    package="com.alvr.openxr.apilayer.hand_emulation">

    <uses-sdk android:minSdkVersion="29" android:targetSdkVersion="32" />

    <!-- Meta Quest hand tracking -->
    <uses-permission android:name="com.oculus.permission.HAND_TRACKING" />
    <uses-feature android:name="oculus.software.handtracking" android:required="false" />

    <!-- Khronos AOSP standardized hand tracking permission (XR_EXT_aosp_hand_tracking_permission) -->
    <uses-permission android:name="org.khronos.openxr.permission.ACCESS_DETAILED_HAND_DATA" />

    <application>
        <meta-data android:name="com.oculus.handtracking.frequency" android:value="HIGH" />
        <meta-data android:name="com.oculus.handtracking.version" android:value="V2.0" />
    </application>
</manifest>
"#;

#[derive(Debug, Clone)]
pub struct PackageAarOptions<'a> {
    pub so_path: &'a Path,
    pub manifest_json_path: &'a Path,
    pub output_path: &'a Path,
    pub config_json_path: Option<&'a Path>,
    pub allow_host_elf: bool,
}

fn deterministic_zip_options(method: CompressionMethod) -> SimpleFileOptions {
    SimpleFileOptions::default()
        .compression_method(method)
        .last_modified_time(DateTime::from_date_and_time(1980, 1, 1, 0, 0, 0).unwrap())
        .unix_permissions(0o644)
}

fn build_empty_classes_jar() -> Result<Vec<u8>> {
    let mut cursor = Cursor::new(Vec::new());
    {
        let mut jar = ZipWriter::new(&mut cursor);
        let opts = deterministic_zip_options(CompressionMethod::Deflated);
        jar.start_file("META-INF/MANIFEST.MF", opts)?;
        jar.write_all(b"Manifest-Version: 1.0\r\nCreated-By: alvr-xr-packager\r\n\r\n")?;
        jar.finish()?;
    }
    Ok(cursor.into_inner())
}

/// Builds a deterministic, spec-compliant Android `.aar` containing the OpenXR implicit API layer.
pub fn package_aar(opts: &PackageAarOptions<'_>) -> Result<()> {
    let so_bytes = fs::read(opts.so_path)
        .with_context(|| format!("failed to read layer .so at {}", opts.so_path.display()))?;
    let elf = inspect_elf(&so_bytes)
        .with_context(|| format!("{} is not a valid ELF64 binary", opts.so_path.display()))?;
    if !opts.allow_host_elf && !elf.is_android_arm64_shared() {
        bail!(
            "expected Android AArch64 shared library (EM_AARCH64, ET_DYN), got {} (e_type={}); pass --allow-host-elf only for host testing",
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
            let raw = fs::read_to_string(cfg_path).with_context(|| {
                format!("failed to read default config JSON at {}", cfg_path.display())
            })?;
            let parsed = LayerConfig::from_json_str(&raw)
                .context("invalid LayerConfig JSON passed to --config")?;
            Some(parsed.to_json_pretty()?.into_bytes())
        }
        None => None,
    };

    if let Some(parent) = opts.output_path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }

    let classes_jar = build_empty_classes_jar()?;

    // Sorted entry order + fixed DOS timestamp -> byte-for-byte deterministic output.
    let mut entries: Vec<(&str, &[u8], CompressionMethod)> = vec![
        (
            "AndroidManifest.xml",
            AAR_ANDROID_MANIFEST_XML.as_bytes(),
            CompressionMethod::Deflated,
        ),
        ("R.txt", b"", CompressionMethod::Deflated),
    ];
    if let Some(ref cfg) = config_bytes {
        entries.push((
            "assets/alvr_hand_controller.json",
            cfg.as_slice(),
            CompressionMethod::Deflated,
        ));
    }
    entries.push((
        EXPECTED_LAYER_JSON_ASSET,
        manifest_json_bytes.as_slice(),
        CompressionMethod::Deflated,
    ));
    entries.push(("classes.jar", classes_jar.as_slice(), CompressionMethod::Stored));
    entries.push((
        EXPECTED_AAR_SO_PATH,
        so_bytes.as_slice(),
        CompressionMethod::Deflated,
    ));
    entries.push(("proguard.txt", b"", CompressionMethod::Deflated));

    entries.sort_by_key(|(name, _, _)| *name);

    let out_file = File::create(opts.output_path)
        .with_context(|| format!("failed to create {}", opts.output_path.display()))?;
    let mut zip = ZipWriter::new(out_file);
    for (name, data, method) in entries {
        zip.start_file(name, deterministic_zip_options(method))?;
        zip.write_all(data)?;
    }
    zip.finish()?;

    Ok(())
}
