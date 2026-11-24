//! Android packaging, binary AXML patching, and ELF/AAR/APK inspection library
//! for the Quest Standalone OpenXR Hand-to-Touch-Controller Implicit API Layer.

pub mod aar;
pub mod apk;
pub mod axml;
pub mod inspect;

pub use aar::{package_aar, PackageAarOptions, AAR_ANDROID_MANIFEST_XML};
pub use apk::{patch_apk, PatchApkOptions, PatchApkResult};
pub use axml::{
    inspect_axml, patch_quest_hand_tracking_axml, AxmlError, AxmlPatchReport, ManifestInspection,
};
pub use inspect::{
    inspect_archive, inspect_elf, validate_layer_manifest_json, ArchiveInspection, ElfInspection,
};
