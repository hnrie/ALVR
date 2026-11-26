//! Self-contained cross-platform desktop GUI (`alvr-quest-patcher`) for patching unprotected
//! non-Meta-Store Quest OpenXR APKs with the ALVR Hand-to-Touch-Controller OpenXR implicit API
//! layer, pure-Rust 16 KB page alignment, and pure-Rust APK Signature Scheme v1/v2/v3 signing.
//!
//! Zero external installs required: embeds the prebuilt `arm64-v8a` `.so`, OpenXR layer manifest
//! `.json`, default runtime config, 16 KB aligner, and v1/v2/v3 signer directly in one binary.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use alvr_hand_core::{LayerConfig, PoseProfileMode};
use android_packager::{
    default_debug_key_path, inspect_archive, inspect_elf, is_valid_android_package_name,
    patch_apk_with_bytes, validate_layer_manifest_json, verify_apk_alignment,
    verify_apk_signatures, ArchiveInspection, ElfInspection, PatchApkBytesOptions, PatchApkResult,
    PatchProgress, StreamingApkZipWriter, ALIGN_4B,
};
use anyhow::{ensure, Context, Result};
use eframe::egui::{self, Color32, CornerRadius, Margin, RichText, Stroke, Vec2};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};
use std::sync::mpsc::{self, Receiver};
use std::thread;

const EMBEDDED_LAYER_SO: &[u8] = include_bytes!(env!("ALVR_EMBED_SO_PATH"));
const EMBEDDED_LAYER_MANIFEST: &[u8] = include_bytes!(env!("ALVR_EMBED_MANIFEST_PATH"));
const EMBEDDED_DEFAULT_CONFIG: &[u8] = include_bytes!(env!("ALVR_EMBED_CONFIG_PATH"));

// Visual palette
const BG_PANEL: Color32 = Color32::from_rgb(18, 22, 29);
const BG_CARD: Color32 = Color32::from_rgb(26, 32, 42);
const BG_DROP: Color32 = Color32::from_rgb(22, 34, 48);
const BORDER_SUBTLE: Color32 = Color32::from_rgb(48, 58, 74);
const ACCENT_BLUE: Color32 = Color32::from_rgb(56, 152, 255);
const ACCENT_GREEN: Color32 = Color32::from_rgb(52, 208, 128);
const ACCENT_AMBER: Color32 = Color32::from_rgb(245, 180, 65);
const ACCENT_RED: Color32 = Color32::from_rgb(240, 88, 96);
const TEXT_MUTED: Color32 = Color32::from_rgb(154, 166, 184);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SigningMode {
    PersistentDebug,
    CustomKeystore,
    Ephemeral,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConfigEmbedMode {
    DefaultPreset,
    CustomJsonFile,
    None,
}

enum WorkerEvent {
    Inspected {
        path: PathBuf,
        result: Result<ArchiveInspection, String>,
    },
    Progress(PatchProgress),
    Finished {
        output_path: PathBuf,
        result: Result<PatchApkResult, String>,
    },
}

struct PatcherApp {
    embedded_elf: Option<ElfInspection>,
    embedded_manifest_name: Option<String>,
    default_key_path_display: String,

    // Primary I/O
    input_apk: Option<PathBuf>,
    output_apk_text: String,
    inspection: Option<ArchiveInspection>,
    inspection_error: Option<String>,
    inspecting: bool,

    // Advanced options
    show_advanced: bool,
    rename_package_enabled: bool,
    rename_package_text: String,
    signing_mode: SigningMode,
    custom_keystore_path: String,
    custom_keystore_pass: String,
    custom_keystore_alias: String,
    config_embed_mode: ConfigEmbedMode,
    custom_config_path: String,
    preset_pose_mode: PoseProfileMode,
    preset_only_touch: bool,
    preset_hide_app_hand_tracking: bool,
    allow_missing_loader: bool,

    // Execution & result state
    patching: bool,
    progress_fraction: f32,
    progress_message: String,
    patch_result: Option<(PathBuf, PatchApkResult)>,
    patch_error: Option<String>,
    clipboard_feedback: Option<String>,

    // Background worker channel
    worker_rx: Option<Receiver<WorkerEvent>>,
}

impl PatcherApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        apply_dark_theme(&cc.egui_ctx);

        let embedded_elf = inspect_elf(EMBEDDED_LAYER_SO).ok();
        let embedded_manifest_name = validate_layer_manifest_json(EMBEDDED_LAYER_MANIFEST).ok();
        let default_key_path_display = default_debug_key_path()
            .map(|p| p.display().to_string())
            .unwrap_or_else(|| "ephemeral in-memory key".to_owned());

        Self {
            embedded_elf,
            embedded_manifest_name,
            default_key_path_display,
            input_apk: None,
            output_apk_text: String::new(),
            inspection: None,
            inspection_error: None,
            inspecting: false,
            show_advanced: false,
            rename_package_enabled: false,
            rename_package_text: String::new(),
            signing_mode: SigningMode::PersistentDebug,
            custom_keystore_path: String::new(),
            custom_keystore_pass: String::new(),
            custom_keystore_alias: String::new(),
            config_embed_mode: ConfigEmbedMode::DefaultPreset,
            custom_config_path: String::new(),
            preset_pose_mode: PoseProfileMode::MetaQuestNativeGrip,
            preset_only_touch: false,
            preset_hide_app_hand_tracking: true,
            allow_missing_loader: false,
            patching: false,
            progress_fraction: 0.0,
            progress_message: String::new(),
            patch_result: None,
            patch_error: None,
            clipboard_feedback: None,
            worker_rx: None,
        }
    }

    fn select_input_apk(&mut self, path: PathBuf, ctx: &egui::Context) {
        let suggested_out = default_output_path(&path);
        self.output_apk_text = suggested_out.display().to_string();
        self.input_apk = Some(path.clone());
        self.inspection = None;
        self.inspection_error = None;
        self.patch_result = None;
        self.patch_error = None;
        self.clipboard_feedback = None;
        self.inspecting = true;

        let (tx, rx) = mpsc::channel();
        self.worker_rx = Some(rx);
        let ctx_clone = ctx.clone();

        thread::spawn(move || {
            let res = inspect_archive(&path).map_err(|e| format!("{e:#}"));
            let _ = tx.send(WorkerEvent::Inspected { path, result: res });
            ctx_clone.request_repaint();
        });
    }

    fn start_patching(&mut self, ctx: &egui::Context) {
        let Some(input_apk) = self.input_apk.clone() else {
            return;
        };
        let output_apk = PathBuf::from(self.output_apk_text.trim());
        if output_apk.as_os_str().is_empty() {
            self.patch_error = Some("Please specify an output APK path.".to_owned());
            return;
        }

        let package_override = if self.rename_package_enabled {
            let trimmed = self.rename_package_text.trim();
            if !is_valid_android_package_name(trimmed) {
                self.patch_error = Some(format!(
                    "Invalid Android package name '{trimmed}' (example: com.example.app.hands)."
                ));
                return;
            }
            Some(trimmed.to_owned())
        } else {
            None
        };

        let (keystore_path, ks_pass, ks_alias, ephemeral_key) = match self.signing_mode {
            SigningMode::PersistentDebug => (None, None, None, false),
            SigningMode::Ephemeral => (None, None, None, true),
            SigningMode::CustomKeystore => {
                let p = PathBuf::from(self.custom_keystore_path.trim());
                if !p.is_file() {
                    self.patch_error = Some("Selected custom keystore file does not exist.".to_owned());
                    return;
                }
                let pass = if self.custom_keystore_pass.is_empty() {
                    None
                } else {
                    Some(self.custom_keystore_pass.clone())
                };
                let alias = if self.custom_keystore_alias.trim().is_empty() {
                    None
                } else {
                    Some(self.custom_keystore_alias.trim().to_owned())
                };
                (Some(p), pass, alias, false)
            }
        };

        let config_bytes: Option<Vec<u8>> = match self.config_embed_mode {
            ConfigEmbedMode::None => None,
            ConfigEmbedMode::DefaultPreset => {
                let mut cfg = LayerConfig::from_json_str(
                    std::str::from_utf8(EMBEDDED_DEFAULT_CONFIG).unwrap_or("{}"),
                )
                .unwrap_or_default();
                cfg.pose.mode = self.preset_pose_mode;
                cfg.gestures.only_touch = self.preset_only_touch;
                cfg.hide_hand_tracking_from_app = self.preset_hide_app_hand_tracking;
                match cfg.to_json_pretty() {
                    Ok(s) => Some(s.into_bytes()),
                    Err(e) => {
                        self.patch_error = Some(format!("Failed to serialize preset config: {e}"));
                        return;
                    }
                }
            }
            ConfigEmbedMode::CustomJsonFile => {
                let p = PathBuf::from(self.custom_config_path.trim());
                match fs::read(&p) {
                    Ok(b) => Some(b),
                    Err(e) => {
                        self.patch_error =
                            Some(format!("Failed to read custom config {}: {e}", p.display()));
                        return;
                    }
                }
            }
        };

        let allow_missing_loader = self.allow_missing_loader;

        self.patching = true;
        self.progress_fraction = 0.02;
        self.progress_message = "Starting APK patcher...".to_owned();
        self.patch_result = None;
        self.patch_error = None;
        self.clipboard_feedback = None;

        let (tx, rx) = mpsc::channel();
        self.worker_rx = Some(rx);
        let ctx_clone = ctx.clone();

        thread::spawn(move || {
            let tx_prog = tx.clone();
            let ctx_prog = ctx_clone.clone();
            let res = patch_apk_with_bytes(
                &PatchApkBytesOptions {
                    input_apk: &input_apk,
                    output_apk: &output_apk,
                    so_bytes: EMBEDDED_LAYER_SO,
                    manifest_json_bytes: EMBEDDED_LAYER_MANIFEST,
                    config_json_bytes: config_bytes.as_deref(),
                    keystore_path: keystore_path.as_deref(),
                    ks_pass: ks_pass.as_deref(),
                    ks_alias: ks_alias.as_deref(),
                    ephemeral_key,
                    package: package_override.as_deref(),
                    skip_sign: false,
                    allow_missing_loader,
                    allow_host_elf: false,
                },
                move |progress| {
                    let _ = tx_prog.send(WorkerEvent::Progress(progress));
                    ctx_prog.request_repaint();
                },
            )
            .map_err(|e| format!("{e:#}"));

            let _ = tx.send(WorkerEvent::Finished {
                output_path: output_apk,
                result: res,
            });
            ctx_clone.request_repaint();
        });
    }

    fn poll_worker(&mut self) {
        let mut disconnect = false;
        if let Some(ref rx) = self.worker_rx {
            while let Ok(ev) = rx.try_recv() {
                match ev {
                    WorkerEvent::Inspected { path, result } => {
                        if self.input_apk.as_ref() == Some(&path) {
                            self.inspecting = false;
                            match result {
                                Ok(info) => {
                                    if let Some(ref pkg) = info.manifest.package {
                                        if self.rename_package_text.is_empty() {
                                            self.rename_package_text = format!("{pkg}.hands");
                                        }
                                    }
                                    self.inspection = Some(info);
                                    self.inspection_error = None;
                                }
                                Err(err) => {
                                    self.inspection = None;
                                    self.inspection_error = Some(err);
                                }
                            }
                        }
                        disconnect = true;
                    }
                    WorkerEvent::Progress(p) => {
                        self.progress_fraction = p.fraction;
                        self.progress_message = p.message;
                    }
                    WorkerEvent::Finished {
                        output_path,
                        result,
                    } => {
                        self.patching = false;
                        match result {
                            Ok(res) => {
                                self.progress_fraction = 1.0;
                                self.progress_message = "Patched, 16 KB aligned, and signed!".to_owned();
                                self.patch_result = Some((output_path, res));
                                self.patch_error = None;
                            }
                            Err(err) => {
                                self.patch_result = None;
                                self.patch_error = Some(err);
                            }
                        }
                        disconnect = true;
                    }
                }
            }
        }
        if disconnect {
            self.worker_rx = None;
        }
    }

    fn can_start_patch(&self) -> bool {
        if self.patching || self.inspecting || self.input_apk.is_none() {
            return false;
        }
        if let Some(ref info) = self.inspection {
            if info.has_any_native_libs && !info.has_arm64_abi {
                return false;
            }
            if !info.has_openxr_loader && !self.allow_missing_loader {
                return false;
            }
        }
        true
    }
}

impl eframe::App for PatcherApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_worker();

        // Handle OS drag-and-drop onto the window
        if !self.patching {
            let dropped: Vec<PathBuf> = ctx.input(|i| {
                i.raw
                    .dropped_files
                    .iter()
                    .filter_map(|f| f.path.clone())
                    .collect()
            });
            if let Some(first_apk) = dropped
                .iter()
                .find(|p| {
                    p.extension()
                        .and_then(|e| e.to_str())
                        .is_some_and(|e| e.eq_ignore_ascii_case("apk"))
                })
                .or_else(|| dropped.first())
            {
                self.select_input_apk(first_apk.clone(), ctx);
            }
        }

        let is_hovering_file = ctx.input(|i| !i.raw.hovered_files.is_empty());

        egui::TopBottomPanel::bottom("footer_panel")
            .frame(
                egui::Frame::new()
                    .fill(BG_PANEL)
                    .inner_margin(Margin::symmetric(18, 10))
                    .stroke(Stroke::new(1.0_f32, BORDER_SUBTLE)),
            )
            .show(ctx, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label(
                        RichText::new(
                            "Scope: Unprotected non-Meta-Store OpenXR APKs only. Standard OpenXR implicit layer injection, 16 KB alignment, and v1/v2/v3 signing — zero DRM, entitlement, or anti-cheat bypasses.",
                        )
                        .size(11.5)
                        .color(TEXT_MUTED),
                    );
                });
            });

        egui::CentralPanel::default()
            .frame(
                egui::Frame::new()
                    .fill(BG_PANEL)
                    .inner_margin(Margin::same(18)),
            )
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    // Header
                    ui.horizontal(|ui| {
                        ui.vertical(|ui| {
                            ui.label(
                                RichText::new("ALVR Quest Hand-Tracking APK Patcher")
                                    .size(22.0)
                                    .strong()
                                    .color(Color32::WHITE),
                            );
                            ui.label(
                                RichText::new(
                                    "Drop an unprotected Quest OpenXR APK to bundle the ALVR Hand-to-Touch layer, 16 KB align, and sign (v1/v2/v3) with zero extra installs.",
                                )
                                .size(13.0)
                                .color(TEXT_MUTED),
                            );
                        });
                    });

                    ui.add_space(8.0);
                    // Embedded runtime pill bar
                    ui.horizontal_wrapped(|ui| {
                        let elf_ok = self
                            .embedded_elf
                            .as_ref()
                            .is_some_and(|e| e.is_android_arm64_shared() && e.min_pt_load_align >= 0x4000);
                        pill_badge(
                            ui,
                            if elf_ok {
                                "Embedded Layer: arm64-v8a • 16 KB ELF"
                            } else {
                                "Embedded Layer: Warning"
                            },
                            if elf_ok { ACCENT_GREEN } else { ACCENT_RED },
                        );
                        if let Some(ref name) = self.embedded_manifest_name {
                            pill_badge(ui, name, ACCENT_BLUE);
                        }
                        pill_badge(ui, "Pure-Rust Aligner + v1/v2/v3 Signer", ACCENT_BLUE);
                    });

                    ui.add_space(14.0);

                    // 1. Drop Zone / Input APK Selector
                    let drop_bg = if is_hovering_file {
                        Color32::from_rgb(30, 52, 78)
                    } else {
                        BG_DROP
                    };
                    let drop_stroke = if is_hovering_file {
                        Stroke::new(2.0_f32, ACCENT_BLUE)
                    } else {
                        Stroke::new(1.5_f32, BORDER_SUBTLE)
                    };

                    egui::Frame::new()
                        .fill(drop_bg)
                        .stroke(drop_stroke)
                        .corner_radius(CornerRadius::same(12))
                        .inner_margin(Margin::same(18))
                        .show(ui, |ui| {
                            ui.set_width(ui.available_width());
                            ui.vertical_centered(|ui| {
                                if is_hovering_file {
                                    ui.label(
                                        RichText::new("Release to load APK")
                                            .size(16.0)
                                            .strong()
                                            .color(ACCENT_BLUE),
                                    );
                                } else if let Some(ref p) = self.input_apk {
                                    ui.label(
                                        RichText::new(
                                            p.file_name()
                                                .and_then(|s| s.to_str())
                                                .unwrap_or("Selected APK"),
                                        )
                                        .size(16.0)
                                        .strong()
                                        .color(Color32::WHITE),
                                    );
                                    ui.label(
                                        RichText::new(p.display().to_string())
                                            .size(12.0)
                                            .color(TEXT_MUTED),
                                    );
                                } else {
                                    ui.label(
                                        RichText::new("Drop a Quest OpenXR .apk file anywhere here")
                                            .size(16.0)
                                            .strong()
                                            .color(Color32::WHITE),
                                    );
                                    ui.label(
                                        RichText::new("or pick an APK from disk below")
                                            .size(12.5)
                                            .color(TEXT_MUTED),
                                    );
                                }
                                ui.add_space(10.0);
                                let browse = ui.add_enabled(
                                    !self.patching,
                                    egui::Button::new(
                                        RichText::new("Browse APK...").size(13.5).strong(),
                                    )
                                    .min_size(Vec2::new(150.0, 32.0)),
                                );
                                if browse.clicked() {
                                    if let Some(picked) = rfd::FileDialog::new()
                                        .add_filter("Android Package (APK)", &["apk"])
                                        .pick_file()
                                    {
                                        self.select_input_apk(picked, ctx);
                                    }
                                }
                            });
                        });

                    ui.add_space(12.0);

                    // 2. Pre-flight APK Inspection Card
                    if self.inspecting {
                        card_frame(ui, |ui| {
                            ui.horizontal(|ui| {
                                ui.spinner();
                                ui.label("Inspecting APK manifest, ABIs, and OpenXR loader...");
                            });
                        });
                        ui.add_space(12.0);
                    } else if let Some(ref err) = self.inspection_error {
                        card_frame(ui, |ui| {
                            ui.label(
                                RichText::new(format!("Could not inspect APK: {err}"))
                                    .color(ACCENT_RED),
                            );
                        });
                        ui.add_space(12.0);
                    } else if let Some(ref info) = self.inspection {
                        card_frame(ui, |ui| {
                            ui.label(
                                RichText::new("APK Pre-Flight Check")
                                    .size(14.5)
                                    .strong()
                                    .color(Color32::WHITE),
                            );
                            ui.add_space(6.0);
                            egui::Grid::new("preflight_grid")
                                .num_columns(2)
                                .spacing([16.0, 6.0])
                                .show(ui, |ui| {
                                    ui.label(RichText::new("Package ID").color(TEXT_MUTED));
                                    ui.label(
                                        info.manifest
                                            .package
                                            .as_deref()
                                            .unwrap_or("(missing)"),
                                    );
                                    ui.end_row();

                                    ui.label(RichText::new("Native ABI").color(TEXT_MUTED));
                                    if info.has_arm64_abi {
                                        ui.label(
                                            RichText::new("arm64-v8a detected (Quest compatible)")
                                                .color(ACCENT_GREEN),
                                        );
                                    } else if info.has_any_native_libs {
                                        ui.label(
                                            RichText::new("No arm64-v8a ABI found (32-bit APKs cannot load arm64 Quest layers)")
                                                .color(ACCENT_RED),
                                        );
                                    } else {
                                        ui.label(
                                            RichText::new("No native libraries yet (will add lib/arm64-v8a/)")
                                                .color(ACCENT_AMBER),
                                        );
                                    }
                                    ui.end_row();

                                    ui.label(RichText::new("OpenXR Loader").color(TEXT_MUTED));
                                    if info.has_openxr_loader {
                                        ui.label(
                                            RichText::new("OpenXR loader detected")
                                                .color(ACCENT_GREEN),
                                        );
                                    } else if info.has_legacy_vrapi {
                                        ui.label(
                                            RichText::new("Legacy libvrapi.so detected (VrApi apps do NOT load OpenXR layers)")
                                                .color(ACCENT_RED),
                                        );
                                    } else {
                                        ui.label(
                                            RichText::new("No libopenxr_loader.so found (enable 'Allow missing loader' in Advanced if loaded dynamically)")
                                                .color(ACCENT_AMBER),
                                        );
                                    }
                                    ui.end_row();

                                    ui.label(RichText::new("Hand Tracking Manifest").color(TEXT_MUTED));
                                    if info.has_quest_hand_tracking {
                                        ui.label(
                                            RichText::new("Already declared (will preserve idempotently)")
                                                .color(ACCENT_GREEN),
                                        );
                                    } else {
                                        ui.label(
                                            RichText::new("Will add Quest + Khronos hand-tracking permissions, feature, and metadata")
                                                .color(ACCENT_BLUE),
                                        );
                                    }
                                    ui.end_row();

                                    ui.label(RichText::new("ALVR Hand Layer").color(TEXT_MUTED));
                                    if info.layer_so_path.is_some() && info.has_layer_manifest {
                                        ui.label(
                                            RichText::new("Already present (will refresh to embedded build)")
                                                .color(ACCENT_AMBER),
                                        );
                                    } else {
                                        ui.label(
                                            RichText::new("Ready to inject")
                                                .color(ACCENT_BLUE),
                                        );
                                    }
                                    ui.end_row();
                                });
                        });
                        ui.add_space(12.0);
                    }

                    // 3. Output Path & Advanced Options
                    card_frame(ui, |ui| {
                        ui.label(
                            RichText::new("Output Patched APK")
                                .size(14.0)
                                .strong()
                                .color(Color32::WHITE),
                        );
                        ui.add_space(6.0);
                        ui.horizontal(|ui| {
                            let te = egui::TextEdit::singleline(&mut self.output_apk_text)
                                .desired_width((ui.available_width() - 105.0).max(160.0))
                                .hint_text("Path for final signed APK...");
                            ui.add_enabled(!self.patching, te);
                            if ui
                                .add_enabled(!self.patching, egui::Button::new("Save As..."))
                                .clicked()
                            {
                                let mut dlg = rfd::FileDialog::new()
                                    .add_filter("Android Package (APK)", &["apk"]);
                                if let Some(ref inp) = self.input_apk {
                                    if let Some(parent) = inp.parent() {
                                        dlg = dlg.set_directory(parent);
                                    }
                                    if let Some(fname) =
                                        default_output_path(inp).file_name().and_then(|s| s.to_str())
                                    {
                                        dlg = dlg.set_file_name(fname);
                                    }
                                }
                                if let Some(save_path) = dlg.save_file() {
                                    self.output_apk_text = save_path.display().to_string();
                                }
                            }
                        });

                        ui.add_space(10.0);
                        let adv_label = if self.show_advanced {
                            "▼ Advanced Options (Package Rename, Signing Key, Gesture Config)"
                        } else {
                            "▶ Advanced Options (Package Rename, Signing Key, Gesture Config)"
                        };
                        if ui
                            .selectable_label(false, RichText::new(adv_label).color(ACCENT_BLUE))
                            .clicked()
                        {
                            self.show_advanced = !self.show_advanced;
                        }

                        if self.show_advanced {
                            ui.add_space(8.0);
                            ui.separator();
                            ui.add_space(6.0);

                            // Side-by-side package rename
                            ui.add_enabled_ui(!self.patching, |ui| {
                                ui.checkbox(
                                    &mut self.rename_package_enabled,
                                    "Rename package ID for side-by-side install (qualifies relative Java classes)",
                                );
                                if self.rename_package_enabled {
                                    ui.horizontal(|ui| {
                                        ui.label("New package:");
                                        ui.text_edit_singleline(&mut self.rename_package_text);
                                        if !self.rename_package_text.trim().is_empty()
                                            && !is_valid_android_package_name(
                                                self.rename_package_text.trim(),
                                            )
                                        {
                                            ui.label(
                                                RichText::new("Invalid ID").color(ACCENT_RED),
                                            );
                                        }
                                    });
                                }

                                ui.add_space(8.0);
                                ui.label(RichText::new("APK Signing Key (v1 + v2 + v3)").strong());
                                ui.radio_value(
                                    &mut self.signing_mode,
                                    SigningMode::PersistentDebug,
                                    "Auto persistent debug key (recommended for adb install -r upgrades)",
                                );
                                if self.signing_mode == SigningMode::PersistentDebug {
                                    ui.label(
                                        RichText::new(format!(
                                            "Key location: {}",
                                            self.default_key_path_display
                                        ))
                                        .size(11.5)
                                        .color(TEXT_MUTED),
                                    );
                                }
                                ui.radio_value(
                                    &mut self.signing_mode,
                                    SigningMode::CustomKeystore,
                                    "Custom keystore (.p12 / .pfx / .jks / .keystore / .pem)",
                                );
                                if self.signing_mode == SigningMode::CustomKeystore {
                                    ui.horizontal(|ui| {
                                        ui.label("Keystore:");
                                        ui.add(
                                            egui::TextEdit::singleline(
                                                &mut self.custom_keystore_path,
                                            )
                                            .desired_width(280.0),
                                        );
                                        if ui.button("Browse...").clicked() {
                                            if let Some(p) = rfd::FileDialog::new()
                                                .add_filter(
                                                    "Keystore",
                                                    &["p12", "pfx", "jks", "keystore", "pem"],
                                                )
                                                .pick_file()
                                            {
                                                self.custom_keystore_path = p.display().to_string();
                                            }
                                        }
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Password:");
                                        ui.add(
                                            egui::TextEdit::singleline(
                                                &mut self.custom_keystore_pass,
                                            )
                                            .password(true)
                                            .desired_width(150.0),
                                        );
                                        ui.label("Alias (optional):");
                                        ui.add(
                                            egui::TextEdit::singleline(
                                                &mut self.custom_keystore_alias,
                                            )
                                            .desired_width(130.0),
                                        );
                                    });
                                }
                                ui.radio_value(
                                    &mut self.signing_mode,
                                    SigningMode::Ephemeral,
                                    "Ephemeral one-time RSA-2048 key (not saved to disk)",
                                );

                                ui.add_space(8.0);
                                ui.label(
                                    RichText::new("Embedded Runtime Config (assets/alvr_hand_controller.json)")
                                        .strong(),
                                );
                                ui.horizontal(|ui| {
                                    ui.radio_value(
                                        &mut self.config_embed_mode,
                                        ConfigEmbedMode::DefaultPreset,
                                        "Preset",
                                    );
                                    ui.radio_value(
                                        &mut self.config_embed_mode,
                                        ConfigEmbedMode::CustomJsonFile,
                                        "Custom JSON file",
                                    );
                                    ui.radio_value(
                                        &mut self.config_embed_mode,
                                        ConfigEmbedMode::None,
                                        "None (use layer defaults)",
                                    );
                                });
                                match self.config_embed_mode {
                                    ConfigEmbedMode::DefaultPreset => {
                                        ui.horizontal(|ui| {
                                            ui.label("Grip Pose Mode:");
                                            egui::ComboBox::from_id_salt("pose_mode_combo")
                                                .selected_text(pose_mode_label(self.preset_pose_mode))
                                                .show_ui(ui, |ui| {
                                                    for mode in [
                                                        PoseProfileMode::MetaQuestNativeGrip,
                                                        PoseProfileMode::OpenXrSpecGrip,
                                                        PoseProfileMode::RawAlvrSteamVr,
                                                        PoseProfileMode::Custom,
                                                    ] {
                                                        ui.selectable_value(
                                                            &mut self.preset_pose_mode,
                                                            mode,
                                                            pose_mode_label(mode),
                                                        );
                                                    }
                                                });
                                            ui.checkbox(
                                                &mut self.preset_only_touch,
                                                "Touch-only gestures",
                                            );
                                        });
                                        ui.checkbox(
                                            &mut self.preset_hide_app_hand_tracking,
                                            "Hide raw hand tracking from app (keeps Meta XR / Unity apps in Touch mode instead of gray 1-hand pointer)",
                                        );
                                    }
                                    ConfigEmbedMode::CustomJsonFile => {
                                        ui.horizontal(|ui| {
                                            ui.add(
                                                egui::TextEdit::singleline(
                                                    &mut self.custom_config_path,
                                                )
                                                .desired_width(320.0)
                                                .hint_text("alvr_hand_controller.json"),
                                            );
                                            if ui.button("Browse...").clicked() {
                                                if let Some(p) = rfd::FileDialog::new()
                                                    .add_filter("JSON Config", &["json"])
                                                    .pick_file()
                                                {
                                                    self.custom_config_path =
                                                        p.display().to_string();
                                                }
                                            }
                                        });
                                    }
                                    ConfigEmbedMode::None => {}
                                }

                                ui.add_space(6.0);
                                ui.checkbox(
                                    &mut self.allow_missing_loader,
                                    "Allow missing libopenxr_loader.so (for apps that load OpenXR dynamically)",
                                );
                            });
                        }
                    });

                    ui.add_space(14.0);

                    // 4. Primary Action Button + Live Progress Bar
                    let can_patch = self.can_start_patch();
                    let btn_fill = if can_patch {
                        ACCENT_BLUE
                    } else {
                        Color32::from_rgb(45, 58, 76)
                    };
                    let patch_btn = ui.add_enabled(
                        can_patch,
                        egui::Button::new(
                            RichText::new(if self.patching {
                                "Patching & Signing APK..."
                            } else {
                                "Patch, 16 KB Align & Sign APK"
                            })
                            .size(16.0)
                            .strong()
                            .color(Color32::WHITE),
                        )
                        .fill(btn_fill)
                        .corner_radius(CornerRadius::same(10))
                        .min_size(Vec2::new(ui.available_width(), 44.0)),
                    );
                    if patch_btn.clicked() {
                        self.start_patching(ctx);
                    }

                    if self.patching {
                        ui.add_space(10.0);
                        ui.add(
                            egui::ProgressBar::new(self.progress_fraction)
                                .text(&self.progress_message)
                                .animate(true),
                        );
                    }

                    // 5. Error or Verified Success Card
                    if let Some(ref err) = self.patch_error {
                        ui.add_space(12.0);
                        egui::Frame::new()
                            .fill(Color32::from_rgb(48, 22, 26))
                            .stroke(Stroke::new(1.0_f32, ACCENT_RED))
                            .corner_radius(CornerRadius::same(10))
                            .inner_margin(Margin::same(14))
                            .show(ui, |ui| {
                                ui.set_width(ui.available_width());
                                ui.label(
                                    RichText::new("Patching Failed")
                                        .size(14.5)
                                        .strong()
                                        .color(ACCENT_RED),
                                );
                                ui.add_space(4.0);
                                ui.label(RichText::new(err).color(Color32::WHITE));
                            });
                    }

                    if let Some((ref out_path, ref res)) = self.patch_result {
                        ui.add_space(12.0);
                        egui::Frame::new()
                            .fill(Color32::from_rgb(18, 42, 32))
                            .stroke(Stroke::new(1.0_f32, ACCENT_GREEN))
                            .corner_radius(CornerRadius::same(10))
                            .inner_margin(Margin::same(14))
                            .show(ui, |ui| {
                                ui.set_width(ui.available_width());
                                ui.label(
                                    RichText::new("Patched APK Ready & Self-Verified!")
                                        .size(15.5)
                                        .strong()
                                        .color(ACCENT_GREEN),
                                );
                                ui.add_space(6.0);
                                ui.label(
                                    RichText::new(format!("Output: {}", out_path.display()))
                                        .color(Color32::WHITE),
                                );
                                ui.label(
                                    RichText::new(format!(
                                        "Manifest: +{} permissions, +{} features, +{} meta-data (package: {})",
                                        res.axml_report.added_permissions.len(),
                                        res.axml_report.added_features.len(),
                                        res.axml_report.added_meta_data.len(),
                                        res.final_package.as_deref().unwrap_or("unknown")
                                    ))
                                    .size(12.5)
                                    .color(TEXT_MUTED),
                                );
                                ui.label(
                                    RichText::new(format!(
                                        "16 KB Alignment: {} ({} stored .so, {} stored entries verified)",
                                        if res.alignment.is_fully_aligned_16k() {
                                            "VERIFIED (16 KB + 4 B)"
                                        } else {
                                            "FAILED"
                                        },
                                        res.alignment.checked_so_entries,
                                        res.alignment.checked_stored_entries,
                                    ))
                                    .size(12.5)
                                    .color(ACCENT_GREEN),
                                );
                                if let Some(ref ver) = res.signature_verification {
                                    ui.label(
                                        RichText::new(format!(
                                            "APK Signatures: v1={} • v2={} • v3={}",
                                            if ver.v1_verified { "OK" } else { "NO" },
                                            if ver.v2_verified { "OK" } else { "NO" },
                                            if ver.v3_verified { "OK" } else { "NO" },
                                        ))
                                        .size(12.5)
                                        .color(ACCENT_GREEN),
                                    );
                                }
                                if let Some(ref sig) = res.sign_summary {
                                    ui.label(
                                        RichText::new(format!(
                                            "Cert SHA-256: {}",
                                            sig.cert_sha256
                                        ))
                                        .size(11.5)
                                        .color(TEXT_MUTED),
                                    );
                                }

                                ui.add_space(10.0);
                                ui.horizontal_wrapped(|ui| {
                                    if ui.button("Open Output Folder").clicked() {
                                        reveal_in_file_manager(out_path);
                                    }
                                    if ui.button("Copy adb install -r Command").clicked() {
                                        let cmd = format!(
                                            "adb install -r \"{}\"",
                                            out_path.display()
                                        );
                                        ctx.copy_text(cmd);
                                        self.clipboard_feedback =
                                            Some("Copied adb install command to clipboard!".to_owned());
                                    }
                                    if let Some(ref msg) = self.clipboard_feedback {
                                        ui.label(RichText::new(msg).size(12.0).color(ACCENT_GREEN));
                                    }
                                });
                            });
                    }
                });
            });
    }
}

fn card_frame(ui: &mut egui::Ui, add_contents: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .fill(BG_CARD)
        .stroke(Stroke::new(1.0_f32, BORDER_SUBTLE))
        .corner_radius(CornerRadius::same(10))
        .inner_margin(Margin::same(14))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            add_contents(ui);
        });
}

fn pill_badge(ui: &mut egui::Ui, text: &str, color: Color32) {
    let bg = Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 28);
    egui::Frame::new()
        .fill(bg)
        .stroke(Stroke::new(1.0_f32, color))
        .corner_radius(CornerRadius::same(12))
        .inner_margin(Margin::symmetric(10, 4))
        .show(ui, |ui| {
            ui.label(RichText::new(text).size(11.5).strong().color(color));
        });
}

fn pose_mode_label(mode: PoseProfileMode) -> &'static str {
    match mode {
        PoseProfileMode::MetaQuestNativeGrip => "Meta Quest Native Grip (Recommended)",
        PoseProfileMode::OpenXrSpecGrip => "OpenXR Spec Grip",
        PoseProfileMode::RawAlvrSteamVr => "Raw ALVR SteamVR Offset",
        PoseProfileMode::Custom => "Custom Offsets",
    }
}

fn default_output_path(input: &Path) -> PathBuf {
    let stem = input
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("patched");
    input.with_file_name(format!("{stem}_handtracking.apk"))
}

fn reveal_in_file_manager(path: &Path) {
    #[cfg(target_os = "windows")]
    {
        let _ = Command::new("explorer")
            .arg(format!("/select,{}", path.display()))
            .spawn();
    }
    #[cfg(target_os = "macos")]
    {
        let _ = Command::new("open").arg("-R").arg(path).spawn();
    }
    #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
    {
        if let Some(parent) = path.parent() {
            let _ = Command::new("xdg-open").arg(parent).spawn();
        }
    }
}

fn apply_dark_theme(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = BG_PANEL;
    visuals.window_fill = BG_CARD;
    visuals.extreme_bg_color = Color32::from_rgb(14, 18, 24);
    visuals.widgets.inactive.bg_fill = Color32::from_rgb(36, 44, 58);
    visuals.widgets.hovered.bg_fill = Color32::from_rgb(46, 58, 76);
    visuals.widgets.active.bg_fill = ACCENT_BLUE;
    visuals.selection.bg_fill = Color32::from_rgb(38, 92, 168);
    ctx.set_visuals(visuals);
}

/// Headless smoke test (`--self-test`) verifying embedded Quest arm64-v8a ELF64, layer JSON
/// manifest, default config, and a complete synthetic APK patch -> 16 KB align -> v1/v2/v3 sign
/// -> pure-Rust verification roundtrip with zero external tools.
fn run_self_test(report_path: Option<&Path>) -> Result<()> {
    let mut log = String::new();

    // 1. Validate embedded Quest arm64-v8a .so
    let elf = inspect_elf(EMBEDDED_LAYER_SO).context("embedded layer .so is not a valid ELF64")?;
    ensure!(
        elf.is_android_arm64_shared(),
        "embedded layer .so is not Android AArch64 ET_DYN (got {})",
        elf.machine_name
    );
    ensure!(
        elf.min_pt_load_align >= 0x4000,
        "embedded layer .so PT_LOAD alignment 0x{:x} < 16 KB",
        elf.min_pt_load_align
    );
    ensure!(
        elf.exported_global_symbols
            .iter()
            .any(|s| s == "xrNegotiateLoaderApiLayerInterface")
            && elf
                .exported_global_symbols
                .iter()
                .any(|s| s == "xrInitializeLoaderKHR"),
        "embedded layer .so missing required OpenXR loader exports: {:?}",
        elf.exported_global_symbols
    );
    log.push_str(&format!(
        "ELF OK: {} ({} bytes, pt_load_align=0x{:x}, exports={:?})\n",
        elf.machine_name,
        EMBEDDED_LAYER_SO.len(),
        elf.min_pt_load_align,
        elf.exported_global_symbols
    ));

    // 2. Validate embedded manifest & default config
    let layer_name = validate_layer_manifest_json(EMBEDDED_LAYER_MANIFEST)?;
    let cfg_str = std::str::from_utf8(EMBEDDED_DEFAULT_CONFIG)?;
    let _cfg = LayerConfig::from_json_str(cfg_str)?;
    log.push_str(&format!("Manifest & Config OK: {layer_name}\n"));

    // 3. Synthesize a valid binary-AXML APK in a temp dir and run full patch+align+sign+verify
    let tmp_dir = std::env::temp_dir().join(format!("alvr_patcher_selftest_{}", std::process::id()));
    let _ = fs::remove_dir_all(&tmp_dir);
    fs::create_dir_all(&tmp_dir)?;

    let res = (|| -> Result<()> {
        let in_apk = tmp_dir.join("input_dummy.apk");
        let out_apk = tmp_dir.join("output_patched.apk");

        // Build minimal valid binary AXML manifest + dummy OpenXR loader + resources.arsc
        let axml = synth_minimal_binary_manifest("com.example.selftest");
        {
            let f = fs::File::create(&in_apk)?;
            let mut w = StreamingApkZipWriter::new(f)?;
            w.write_deflated_bytes("AndroidManifest.xml", &axml)?;
            w.write_stored_bytes("resources.arsc", b"ARSC_TEST_TABLE", ALIGN_4B)?;
            w.write_stored_bytes("lib/arm64-v8a/libopenxr_loader.so", b"\x7fELFdummy", ALIGN_4B)?;
            w.finish_unsigned()?;
        }

        let patch_res = patch_apk_with_bytes(
            &PatchApkBytesOptions {
                input_apk: &in_apk,
                output_apk: &out_apk,
                so_bytes: EMBEDDED_LAYER_SO,
                manifest_json_bytes: EMBEDDED_LAYER_MANIFEST,
                config_json_bytes: Some(EMBEDDED_DEFAULT_CONFIG),
                keystore_path: None,
                ks_pass: None,
                ks_alias: None,
                ephemeral_key: true,
                package: Some("com.example.selftest.hands"),
                skip_sign: false,
                allow_missing_loader: false,
                allow_host_elf: false,
            },
            |_| {},
        )?;

        ensure!(patch_res.signed, "self-test APK was not signed");
        ensure!(
            patch_res.final_package.as_deref() == Some("com.example.selftest.hands"),
            "package rename did not apply"
        );

        let align = verify_apk_alignment(&out_apk)?;
        ensure!(align.is_fully_aligned_16k(), "self-test alignment failed");

        let sig = verify_apk_signatures(&out_apk)?;
        ensure!(
            sig.valid && sig.v1_verified && sig.v2_verified && sig.v3_verified,
            "self-test signature verification failed: {:?}",
            sig.errors
        );

        let insp = inspect_archive(&out_apk)?;
        ensure!(
            insp.has_quest_hand_tracking && insp.layer_so_aligned_16k && insp.layer_manifest_valid,
            "self-test archive inspection incomplete"
        );

        log.push_str(&format!(
            "Roundtrip Patch+16KB+v1/v2/v3 OK: pkg={:?}, cert_sha256={:?}\n",
            insp.manifest.package, sig.cert_sha256
        ));
        Ok(())
    })();

    let _ = fs::remove_dir_all(&tmp_dir);
    res?;

    if let Some(rp) = report_path {
        fs::write(rp, &log)?;
    }
    println!("{log}");
    Ok(())
}

/// Builds a minimal valid UTF-8 binary AXML AndroidManifest for `--self-test`.
fn synth_minimal_binary_manifest(package: &str) -> Vec<u8> {
    let strings = [
        "name",
        "android",
        "http://schemas.android.com/apk/res/android",
        "manifest",
        "package",
        package,
        "application",
    ];
    let mut str_offsets = Vec::new();
    let mut str_data = Vec::new();
    for s in &strings {
        str_offsets.push(str_data.len() as u32);
        let len = s.len() as u8;
        str_data.push(len);
        str_data.push(len);
        str_data.extend_from_slice(s.as_bytes());
        str_data.push(0);
    }
    while str_data.len() % 4 != 0 {
        str_data.push(0);
    }
    let strings_start = 28u32 + (strings.len() as u32) * 4;
    let sp_size = strings_start + str_data.len() as u32;
    let mut sp = Vec::new();
    sp.extend_from_slice(&0x0001u16.to_le_bytes());
    sp.extend_from_slice(&28u16.to_le_bytes());
    sp.extend_from_slice(&sp_size.to_le_bytes());
    sp.extend_from_slice(&(strings.len() as u32).to_le_bytes());
    sp.extend_from_slice(&0u32.to_le_bytes());
    sp.extend_from_slice(&0x0100u32.to_le_bytes()); // UTF-8
    sp.extend_from_slice(&strings_start.to_le_bytes());
    sp.extend_from_slice(&0u32.to_le_bytes());
    for o in str_offsets {
        sp.extend_from_slice(&o.to_le_bytes());
    }
    sp.extend_from_slice(&str_data);

    // Resource map: index 0 -> android:name (0x01010003)
    let mut rmap = Vec::new();
    rmap.extend_from_slice(&0x0180u16.to_le_bytes());
    rmap.extend_from_slice(&8u16.to_le_bytes());
    rmap.extend_from_slice(&12u32.to_le_bytes());
    rmap.extend_from_slice(&0x0101_0003u32.to_le_bytes());

    let no_idx = 0xFFFF_FFFFu32;
    let make_ns = |ty: u16| -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&ty.to_le_bytes());
        b.extend_from_slice(&16u16.to_le_bytes());
        b.extend_from_slice(&24u32.to_le_bytes());
        b.extend_from_slice(&1u32.to_le_bytes());
        b.extend_from_slice(&no_idx.to_le_bytes());
        b.extend_from_slice(&1u32.to_le_bytes());
        b.extend_from_slice(&2u32.to_le_bytes());
        b
    };
    let make_end = |name_idx: u32| -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&0x0103u16.to_le_bytes());
        b.extend_from_slice(&16u16.to_le_bytes());
        b.extend_from_slice(&24u32.to_le_bytes());
        b.extend_from_slice(&0u32.to_le_bytes());
        b.extend_from_slice(&no_idx.to_le_bytes());
        b.extend_from_slice(&no_idx.to_le_bytes());
        b.extend_from_slice(&name_idx.to_le_bytes());
        b
    };

    // <manifest package="...">
    let mut manifest_start = Vec::new();
    manifest_start.extend_from_slice(&0x0102u16.to_le_bytes());
    manifest_start.extend_from_slice(&16u16.to_le_bytes());
    manifest_start.extend_from_slice(&56u32.to_le_bytes());
    manifest_start.extend_from_slice(&0u32.to_le_bytes());
    manifest_start.extend_from_slice(&no_idx.to_le_bytes());
    manifest_start.extend_from_slice(&no_idx.to_le_bytes());
    manifest_start.extend_from_slice(&3u32.to_le_bytes()); // "manifest"
    manifest_start.extend_from_slice(&20u16.to_le_bytes());
    manifest_start.extend_from_slice(&20u16.to_le_bytes());
    manifest_start.extend_from_slice(&1u16.to_le_bytes()); // 1 attr
    manifest_start.extend_from_slice(&0u16.to_le_bytes());
    manifest_start.extend_from_slice(&0u16.to_le_bytes());
    manifest_start.extend_from_slice(&0u16.to_le_bytes());
    // attr: package = strings[5]
    manifest_start.extend_from_slice(&no_idx.to_le_bytes());
    manifest_start.extend_from_slice(&4u32.to_le_bytes());
    manifest_start.extend_from_slice(&5u32.to_le_bytes());
    manifest_start.extend_from_slice(&8u16.to_le_bytes());
    manifest_start.push(0);
    manifest_start.push(0x03); // TYPE_STRING
    manifest_start.extend_from_slice(&5u32.to_le_bytes());

    // <application>
    let mut app_start = Vec::new();
    app_start.extend_from_slice(&0x0102u16.to_le_bytes());
    app_start.extend_from_slice(&16u16.to_le_bytes());
    app_start.extend_from_slice(&36u32.to_le_bytes());
    app_start.extend_from_slice(&0u32.to_le_bytes());
    app_start.extend_from_slice(&no_idx.to_le_bytes());
    app_start.extend_from_slice(&no_idx.to_le_bytes());
    app_start.extend_from_slice(&6u32.to_le_bytes()); // "application"
    app_start.extend_from_slice(&20u16.to_le_bytes());
    app_start.extend_from_slice(&20u16.to_le_bytes());
    app_start.extend_from_slice(&0u16.to_le_bytes());
    app_start.extend_from_slice(&0u16.to_le_bytes());
    app_start.extend_from_slice(&0u16.to_le_bytes());
    app_start.extend_from_slice(&0u16.to_le_bytes());

    let mut body = Vec::new();
    body.extend_from_slice(&sp);
    body.extend_from_slice(&rmap);
    body.extend_from_slice(&make_ns(0x0100));
    body.extend_from_slice(&manifest_start);
    body.extend_from_slice(&app_start);
    body.extend_from_slice(&make_end(6));
    body.extend_from_slice(&make_end(3));
    body.extend_from_slice(&make_ns(0x0101));

    let total = (8 + body.len()) as u32;
    let mut out = Vec::with_capacity(total as usize);
    out.extend_from_slice(&0x0003u16.to_le_bytes());
    out.extend_from_slice(&8u16.to_le_bytes());
    out.extend_from_slice(&total.to_le_bytes());
    out.extend_from_slice(&body);
    out
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "--self-test") {
        let report_path = args
            .windows(2)
            .find(|w| w[0] == "--self-test-report")
            .map(|w| PathBuf::from(&w[1]));
        return match run_self_test(report_path.as_deref()) {
            Ok(()) => ExitCode::SUCCESS,
            Err(err) => {
                if let Some(ref rp) = report_path {
                    let _ = fs::write(rp, format!("SELF-TEST FAILED: {err:#}\n"));
                }
                eprintln!("SELF-TEST FAILED: {err:#}");
                ExitCode::FAILURE
            }
        };
    }

    if args.iter().any(|a| a == "--version" || a == "-V") {
        println!("alvr-quest-patcher {}", env!("CARGO_PKG_VERSION"));
        return ExitCode::SUCCESS;
    }

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("ALVR Quest Hand-Tracking Patcher")
            .with_inner_size([760.0, 780.0])
            .with_min_inner_size([600.0, 580.0])
            .with_drag_and_drop(true),
        ..Default::default()
    };

    match eframe::run_native(
        "ALVR Quest Hand-Tracking Patcher",
        native_options,
        Box::new(|cc| Ok(Box::new(PatcherApp::new(cc)))),
    ) {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("GUI error: {err}");
            ExitCode::FAILURE
        }
    }
}
