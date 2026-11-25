//! Self-contained pure-Rust Android packaging, binary AXML patching, 16 KB alignment,
//! APK Signature Scheme v1/v2/v3 signing/verification, and ELF/AAR/APK inspection library
//! for the Quest Standalone OpenXR Hand-to-Touch-Controller Implicit API Layer.

pub mod aar;
pub mod align;
pub mod apk;
pub mod axml;
pub mod inspect;
pub mod sign;

pub use aar::{package_aar, PackageAarOptions, AAR_ANDROID_MANIFEST_XML};
pub use align::{
    align_apk, parse_apk_zip_layout, verify_apk_alignment, AlignmentReport, ApkZipLayout,
    StreamingApkZipWriter, ALIGN_16K, ALIGN_4B, ALIGN_4K,
};
pub use apk::{
    patch_apk, patch_apk_with_bytes, PatchApkBytesOptions, PatchApkOptions, PatchApkResult,
    PatchProgress,
};
pub use axml::{
    inspect_axml, is_valid_android_package_name, patch_quest_hand_tracking_axml,
    rename_package_axml, AxmlError, AxmlPatchReport, ManifestInspection,
};
pub use inspect::{
    inspect_archive, inspect_elf, validate_layer_manifest_json, ArchiveInspection, ElfInspection,
    EXPECTED_AAR_SO_PATH, EXPECTED_APK_SO_PATH, EXPECTED_LAYER_JSON_ASSET, EXPECTED_LAYER_NAME,
    EXPECTED_SONAME,
};
pub use sign::{
    align_and_sign_apk, default_debug_key_path, finalize_signed_apk_stream, is_v1_signature_entry,
    verify_apk_signatures, ApkSignSummary, ApkSignatureVerification, SigningKeyPair,
};
