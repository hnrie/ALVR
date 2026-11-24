//! Runtime JSON configuration, ALVR defaults, sanitization, and resilient file loader.

use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{self, Read};
use std::path::Path;
use thiserror::Error;

/// Environment variable pointing to an optional JSON configuration file override.
pub const CONFIG_ENV_VAR: &str = "ALVR_HAND_LAYER_CONFIG";

/// Environment variable that disables the implicit OpenXR API layer when set non-empty.
pub const DISABLE_ENV_VAR: &str = "DISABLE_XR_APILAYER_ALVR_HAND_EMULATION";

/// Default configuration filename checked under Android app-scoped external files directories.
pub const DEFAULT_CONFIG_FILENAME: &str = "alvr_hand_controller.json";

/// Maximum accepted config file size (1 MiB) to prevent OOM on accidental binary/special paths.
const MAX_CONFIG_BYTES: u64 = 1024 * 1024;

// Sanitization bounds
pub const MIN_DISTANCE_CM: f32 = 0.0;
pub const MAX_DISTANCE_CM: f32 = 50.0;
pub const MIN_JOYSTICK_RANGE_CM: f32 = 0.05;
pub const MAX_JOYSTICK_RANGE_CM: f32 = 50.0;
pub const MAX_JOYSTICK_OFFSET_CM: f32 = 50.0;
pub const MAX_DELAY_MS: u32 = 10_000;
pub const MAX_POSITION_OFFSET_M: f32 = 2.0;
pub const MAX_ROTATION_OFFSET_DEG: f32 = 360.0;
pub const MIN_BOOL_THRESHOLD: f32 = 0.01;
pub const MAX_BOOL_THRESHOLD: f32 = 0.99;
pub const HYSTERESIS_EDGE_MARGIN: f32 = 0.005;
pub const MAX_GRACE_MS: u32 = 30_000;

// Canonical ALVR / Quest default constants (shared source of truth with `pose.rs`)
pub const DEFAULT_LEFT_POSITION_OFFSET: [f32; 3] = [0.04, -0.02, -0.13];
pub const DEFAULT_LEFT_ROTATION_OFFSET_DEG: [f32; 3] = [0.0, -45.0, -90.0];

/// Default Oculus Touch aim offset relative to `MetaQuestNativeGrip` `/grip/pose`:
/// - `[-60.0, 0.0, 0.0]` deg pitches `-Z` down 60° from the +60°-tilted Meta Quest grip handle
///   so the aim ray points straight forward along straightened fingers (`-Z`), with `+Y` up and `+X` right.
/// - `[0.0, -0.04, -0.04]` m (in grip frame) shifts origin ~5.46 cm forward and ~1.46 cm up
///   in neutral handshake pose (near the Touch index trigger / front ring tip).
pub const DEFAULT_LEFT_AIM_POSITION_OFFSET: [f32; 3] = [0.0, -0.04, -0.04];
pub const DEFAULT_LEFT_AIM_ROTATION_OFFSET_DEG: [f32; 3] = [-60.0, 0.0, 0.0];

/// Errors encountered while reading, parsing, or serializing a [`LayerConfig`].
#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("failed to read layer config file: {0}")]
    Io(#[from] io::Error),
    #[error("failed to parse or serialize layer config JSON: {0}")]
    Json(#[from] serde_json::Error),
}

/// Controller grip pose profile controlling how ALVR hand offsets map into OpenXR `/grip/pose`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PoseProfileMode {
    /// Native Meta Quest OpenXR `/grip/pose` (undoes ALVR SteamVR `-0.11m` Z AND Quest `-15°` `O_client`).
    #[default]
    MetaQuestNativeGrip,
    /// OpenXR specification `/grip/pose` (undoes ALVR SteamVR `-0.11m` Z only -> `[±0.04, -0.02, -0.02]`).
    OpenXrSpecGrip,
    /// Verbatim ALVR SteamVR offset (`[±0.04, -0.02, -0.13]`, EulerXYZ `[0, ∓45, ∓90]` deg).
    RawAlvrSteamVr,
    /// Fully custom left-hand offsets (mirrored automatically for right hand).
    Custom,
}

/// Controls how `/input/aim/pose` is synthesized.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AimPoseMode {
    /// Prefer `XR_FB_hand_tracking_aim` `aimPose` when valid, fallback to fixed grip-to-aim offset.
    #[default]
    PreferFbAim,
    /// Always derive `/aim/pose` from synthesized `/grip/pose` + grip-to-aim offset.
    GripOffsetOnly,
}

/// Verbatim ALVR `HandTrackingInteractionConfig` parameters (`alvr/session/src/settings.rs`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct HandTrackingInteractionConfig {
    pub only_touch: bool,
    pub pinch_touch_distance: f32,
    pub pinch_trigger_distance: f32,
    pub curl_touch_distance: f32,
    pub curl_trigger_distance: f32,
    pub joystick_deadzone: f32,
    pub joystick_offset_horizontal: f32,
    pub joystick_offset_vertical: f32,
    pub joystick_range: f32,
    pub activation_delay: u32,
    pub deactivation_delay: u32,
    pub repeat_delay: u32,
}

impl Default for HandTrackingInteractionConfig {
    fn default() -> Self {
        Self {
            only_touch: false,
            pinch_touch_distance: 0.0,
            pinch_trigger_distance: 0.25,
            curl_touch_distance: 2.0,
            curl_trigger_distance: 2.5,
            joystick_deadzone: 40.0,
            joystick_offset_horizontal: 0.0,
            joystick_offset_vertical: 0.0,
            joystick_range: 1.0,
            activation_delay: 50,
            deactivation_delay: 100,
            repeat_delay: 100,
        }
    }
}

impl HandTrackingInteractionConfig {
    /// Sanitize all numeric fields into safe, finite, non-degenerate ranges (idempotent).
    pub fn sanitize(&mut self) {
        let def = Self::default();
        self.pinch_touch_distance = sanitize_f32(
            self.pinch_touch_distance,
            def.pinch_touch_distance,
            MIN_DISTANCE_CM,
            MAX_DISTANCE_CM,
        );
        self.pinch_trigger_distance = sanitize_f32(
            self.pinch_trigger_distance,
            def.pinch_trigger_distance,
            MIN_DISTANCE_CM,
            MAX_DISTANCE_CM,
        )
        .max(self.pinch_touch_distance);

        self.curl_touch_distance = sanitize_f32(
            self.curl_touch_distance,
            def.curl_touch_distance,
            MIN_DISTANCE_CM,
            MAX_DISTANCE_CM,
        );
        self.curl_trigger_distance = sanitize_f32(
            self.curl_trigger_distance,
            def.curl_trigger_distance,
            MIN_DISTANCE_CM,
            MAX_DISTANCE_CM,
        )
        .max(self.curl_touch_distance);

        self.joystick_deadzone =
            sanitize_f32(self.joystick_deadzone, def.joystick_deadzone, 0.0, 100.0);
        self.joystick_offset_horizontal = sanitize_f32(
            self.joystick_offset_horizontal,
            def.joystick_offset_horizontal,
            -MAX_JOYSTICK_OFFSET_CM,
            MAX_JOYSTICK_OFFSET_CM,
        );
        self.joystick_offset_vertical = sanitize_f32(
            self.joystick_offset_vertical,
            def.joystick_offset_vertical,
            -MAX_JOYSTICK_OFFSET_CM,
            MAX_JOYSTICK_OFFSET_CM,
        );

        // `joystick_range` is used as a divisor; non-finite or <= 0.0 falls back to default 1.0 cm.
        if !self.joystick_range.is_finite() || self.joystick_range <= 0.0 {
            self.joystick_range = def.joystick_range;
        }
        self.joystick_range = self
            .joystick_range
            .clamp(MIN_JOYSTICK_RANGE_CM, MAX_JOYSTICK_RANGE_CM);

        self.activation_delay = self.activation_delay.min(MAX_DELAY_MS);
        self.deactivation_delay = self.deactivation_delay.min(MAX_DELAY_MS);
        self.repeat_delay = self.repeat_delay.min(MAX_DELAY_MS);
    }

    /// Consuming builder wrapper around [`Self::sanitize`].
    pub fn sanitized(mut self) -> Self {
        self.sanitize();
        self
    }
}

/// Hand-to-controller pose and aim offset configuration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PoseOffsetConfig {
    pub mode: PoseProfileMode,
    pub left_position_offset: [f32; 3],
    pub left_rotation_offset_deg: [f32; 3],
    pub aim_mode: AimPoseMode,
    pub left_aim_position_offset: [f32; 3],
    pub left_aim_rotation_offset_deg: [f32; 3],
}

impl Default for PoseOffsetConfig {
    fn default() -> Self {
        Self {
            mode: PoseProfileMode::MetaQuestNativeGrip,
            left_position_offset: DEFAULT_LEFT_POSITION_OFFSET,
            left_rotation_offset_deg: DEFAULT_LEFT_ROTATION_OFFSET_DEG,
            aim_mode: AimPoseMode::PreferFbAim,
            left_aim_position_offset: DEFAULT_LEFT_AIM_POSITION_OFFSET,
            left_aim_rotation_offset_deg: DEFAULT_LEFT_AIM_ROTATION_OFFSET_DEG,
        }
    }
}

impl PoseOffsetConfig {
    /// Sanitize all offset vectors into safe, finite ranges (idempotent).
    pub fn sanitize(&mut self) {
        sanitize_array3(
            &mut self.left_position_offset,
            DEFAULT_LEFT_POSITION_OFFSET,
            -MAX_POSITION_OFFSET_M,
            MAX_POSITION_OFFSET_M,
        );
        sanitize_array3(
            &mut self.left_rotation_offset_deg,
            DEFAULT_LEFT_ROTATION_OFFSET_DEG,
            -MAX_ROTATION_OFFSET_DEG,
            MAX_ROTATION_OFFSET_DEG,
        );
        sanitize_array3(
            &mut self.left_aim_position_offset,
            DEFAULT_LEFT_AIM_POSITION_OFFSET,
            -MAX_POSITION_OFFSET_M,
            MAX_POSITION_OFFSET_M,
        );
        sanitize_array3(
            &mut self.left_aim_rotation_offset_deg,
            DEFAULT_LEFT_AIM_ROTATION_OFFSET_DEG,
            -MAX_ROTATION_OFFSET_DEG,
            MAX_ROTATION_OFFSET_DEG,
        );
    }

    /// Consuming builder wrapper around [`Self::sanitize`].
    pub fn sanitized(mut self) -> Self {
        self.sanitize();
        self
    }
}

/// Top-level runtime configuration for the Quest OpenXR hand-to-controller API layer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LayerConfig {
    pub enabled: bool,
    pub gestures: HandTrackingInteractionConfig,
    pub pose: PoseOffsetConfig,
    pub float_to_bool_threshold: f32,
    pub float_to_bool_hysteresis: f32,
    pub physical_controller_grace_ms: u32,
}

impl Default for LayerConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            gestures: HandTrackingInteractionConfig::default(),
            pose: PoseOffsetConfig::default(),
            float_to_bool_threshold: 0.5,
            float_to_bool_hysteresis: 0.05,
            physical_controller_grace_ms: 250,
        }
    }
}

impl LayerConfig {
    /// Sanitize all configuration fields into safe, finite, non-degenerate ranges (idempotent).
    pub fn sanitize(&mut self) {
        let def = Self::default();
        self.gestures.sanitize();
        self.pose.sanitize();

        self.float_to_bool_threshold = sanitize_f32(
            self.float_to_bool_threshold,
            def.float_to_bool_threshold,
            MIN_BOOL_THRESHOLD,
            MAX_BOOL_THRESHOLD,
        );

        let max_h = (self.float_to_bool_threshold - HYSTERESIS_EDGE_MARGIN)
            .min(1.0 - self.float_to_bool_threshold - HYSTERESIS_EDGE_MARGIN)
            .max(0.0);
        self.float_to_bool_hysteresis = sanitize_f32(
            self.float_to_bool_hysteresis,
            def.float_to_bool_hysteresis,
            0.0,
            max_h,
        );

        self.physical_controller_grace_ms = self.physical_controller_grace_ms.min(MAX_GRACE_MS);
    }

    /// Consuming builder wrapper around [`Self::sanitize`].
    pub fn sanitized(mut self) -> Self {
        self.sanitize();
        self
    }

    /// Deserialize a [`LayerConfig`] from a JSON string (stripping optional UTF-8 BOM) and sanitize it.
    pub fn from_json_str(json: &str) -> Result<Self, ConfigError> {
        let trimmed = json.strip_prefix('\u{feff}').unwrap_or(json);
        let cfg: Self = serde_json::from_str(trimmed)?;
        Ok(cfg.sanitized())
    }

    /// Serialize this configuration to pretty-printed JSON.
    pub fn to_json_pretty(&self) -> Result<String, ConfigError> {
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Load and sanitize a [`LayerConfig`] from disk with a 1 MiB read cap.
    pub fn load_from_file(path: impl AsRef<Path>) -> Result<Self, ConfigError> {
        let file = File::open(path.as_ref())?;
        let mut raw = Vec::new();
        let bytes_read = file.take(MAX_CONFIG_BYTES + 1).read_to_end(&mut raw)?;
        if bytes_read as u64 > MAX_CONFIG_BYTES {
            return Err(ConfigError::Io(io::Error::new(
                io::ErrorKind::InvalidData,
                "config file exceeds 1 MiB size limit",
            )));
        }
        let text = String::from_utf8(raw)
            .map_err(|err| ConfigError::Io(io::Error::new(io::ErrorKind::InvalidData, err)))?;
        Self::from_json_str(&text)
    }

    /// Load config checking `ALVR_HAND_LAYER_CONFIG` first, then `candidate_paths` in order.
    ///
    /// Always returns a valid, sanitized [`LayerConfig`] and never panics.
    pub fn load_from_paths<P: AsRef<Path>>(candidate_paths: &[P]) -> Self {
        let env_path = std::env::var_os(CONFIG_ENV_VAR).filter(|v| !v.is_empty());
        let env_ref = env_path.as_deref().map(Path::new);
        Self::load_from_paths_with_env(env_ref, candidate_paths)
    }

    /// Race-free testable loader accepting an explicit environment-override path.
    pub fn load_from_paths_with_env<P: AsRef<Path>>(
        env_override: Option<&Path>,
        candidate_paths: &[P],
    ) -> Self {
        if let Some(path) = env_override.filter(|p| !p.as_os_str().is_empty()) {
            match Self::load_from_file(path) {
                Ok(cfg) => return cfg,
                Err(err) => {
                    log::warn!(
                        "Failed to load config from {} ({}): {}; trying fallback paths",
                        CONFIG_ENV_VAR,
                        path.display(),
                        err
                    );
                }
            }
        }

        for candidate in candidate_paths {
            let path = candidate.as_ref();
            if path.as_os_str().is_empty() {
                continue;
            }
            match Self::load_from_file(path) {
                Ok(cfg) => return cfg,
                Err(ConfigError::Io(ref io_err)) if io_err.kind() == io::ErrorKind::NotFound => {
                    log::debug!("Config file not found at {}", path.display());
                }
                Err(err) => {
                    log::warn!(
                        "Failed to load config from {}: {}; continuing",
                        path.display(),
                        err
                    );
                }
            }
        }

        Self::default().sanitized()
    }
}

fn sanitize_f32(value: f32, default: f32, min: f32, max: f32) -> f32 {
    let finite = if value.is_finite() { value } else { default };
    finite.clamp(min, max)
}

fn sanitize_array3(arr: &mut [f32; 3], defaults: [f32; 3], min: f32, max: f32) {
    for i in 0..3 {
        arr[i] = sanitize_f32(arr[i], defaults[i], min, max);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU64, Ordering};

    static TEST_DIR_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Zero-dependency RAII temporary directory rooted inside Cargo's `target/` directory.
    struct TestDir {
        path: PathBuf,
    }

    impl TestDir {
        fn new(label: &str) -> Self {
            let exe = std::env::current_exe().expect("current_exe");
            let base = exe.parent().expect("exe parent");
            let id = TEST_DIR_COUNTER.fetch_add(1, Ordering::Relaxed);
            let path = base.join(format!(
                "alvr_cfg_test_{}_{}_{}",
                label,
                std::process::id(),
                id
            ));
            fs::create_dir_all(&path).expect("create test dir");
            Self { path }
        }

        fn path(&self) -> &Path {
            &self.path
        }

        fn write_bytes(&self, rel: &str, data: &[u8]) -> PathBuf {
            let full = self.path.join(rel);
            fs::write(&full, data).expect("write test file");
            full
        }
    }

    impl Drop for TestDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.path);
        }
    }

    #[test]
    fn defaults_match_alvr_verbatim() {
        let cfg = LayerConfig::default();
        assert!(cfg.enabled);
        assert_eq!(cfg.float_to_bool_threshold, 0.5);
        assert_eq!(cfg.float_to_bool_hysteresis, 0.05);
        assert_eq!(cfg.physical_controller_grace_ms, 250);

        let g = &cfg.gestures;
        assert!(!g.only_touch);
        assert_eq!(g.pinch_touch_distance, 0.0);
        assert_eq!(g.pinch_trigger_distance, 0.25);
        assert_eq!(g.curl_touch_distance, 2.0);
        assert_eq!(g.curl_trigger_distance, 2.5);
        assert_eq!(g.joystick_deadzone, 40.0);
        assert_eq!(g.joystick_offset_horizontal, 0.0);
        assert_eq!(g.joystick_offset_vertical, 0.0);
        assert_eq!(g.joystick_range, 1.0);
        assert_eq!(g.activation_delay, 50);
        assert_eq!(g.deactivation_delay, 100);
        assert_eq!(g.repeat_delay, 100);

        let p = &cfg.pose;
        assert_eq!(p.mode, PoseProfileMode::MetaQuestNativeGrip);
        assert_eq!(p.left_position_offset, [0.04, -0.02, -0.13]);
        assert_eq!(p.left_rotation_offset_deg, [0.0, -45.0, -90.0]);
        assert_eq!(p.aim_mode, AimPoseMode::PreferFbAim);
        assert_eq!(p.left_aim_position_offset, [0.0, -0.04, -0.04]);
        assert_eq!(p.left_aim_rotation_offset_deg, [-60.0, 0.0, 0.0]);

        // Defaults must already be sanitized.
        assert_eq!(cfg.clone(), cfg.sanitized());
    }

    #[test]
    fn json_round_trip_defaults_and_all_enum_variants() {
        let def = LayerConfig::default();
        let json = def.to_json_pretty().expect("serialize default");
        let parsed = LayerConfig::from_json_str(&json).expect("deserialize default");
        assert_eq!(def, parsed);

        let pose_modes = [
            (PoseProfileMode::MetaQuestNativeGrip, "meta_quest_native_grip"),
            (PoseProfileMode::OpenXrSpecGrip, "open_xr_spec_grip"),
            (PoseProfileMode::RawAlvrSteamVr, "raw_alvr_steam_vr"),
            (PoseProfileMode::Custom, "custom"),
        ];
        let aim_modes = [
            (AimPoseMode::PreferFbAim, "prefer_fb_aim"),
            (AimPoseMode::GripOffsetOnly, "grip_offset_only"),
        ];

        for (pm, pm_str) in pose_modes {
            for (am, am_str) in aim_modes {
                let custom = LayerConfig {
                    enabled: false,
                    gestures: HandTrackingInteractionConfig {
                        only_touch: true,
                        pinch_touch_distance: 0.1,
                        pinch_trigger_distance: 0.6,
                        curl_touch_distance: 1.5,
                        curl_trigger_distance: 3.0,
                        joystick_deadzone: 25.0,
                        joystick_offset_horizontal: -0.75,
                        joystick_offset_vertical: 0.5,
                        joystick_range: 1.8,
                        activation_delay: 20,
                        deactivation_delay: 80,
                        repeat_delay: 60,
                    },
                    pose: PoseOffsetConfig {
                        mode: pm,
                        left_position_offset: [0.02, -0.01, -0.05],
                        left_rotation_offset_deg: [10.0, -30.0, -85.0],
                        aim_mode: am,
                        left_aim_position_offset: [0.01, -0.03, -0.06],
                        left_aim_rotation_offset_deg: [-45.0, 5.0, 0.0],
                    },
                    float_to_bool_threshold: 0.6,
                    float_to_bool_hysteresis: 0.1,
                    physical_controller_grace_ms: 400,
                };
                let pretty = custom.to_json_pretty().expect("serialize custom");
                assert!(
                    pretty.contains(&format!("\"{pm_str}\"")),
                    "missing {pm_str} in:\n{pretty}"
                );
                assert!(
                    pretty.contains(&format!("\"{am_str}\"")),
                    "missing {am_str} in:\n{pretty}"
                );
                let decoded = LayerConfig::from_json_str(&pretty).expect("deserialize custom");
                assert_eq!(custom, decoded);
            }
        }
    }

    #[test]
    fn partial_json_unknown_fields_and_utf8_bom() {
        let empty = LayerConfig::from_json_str("{}").expect("empty object");
        assert_eq!(empty, LayerConfig::default());

        let partial = "\u{feff}{\n\
            \"_comment\": \"forward compatible\",\n\
            \"gestures\": { \"only_touch\": true, \"future_knob\": 123 },\n\
            \"pose\": { \"mode\": \"open_xr_spec_grip\" }\n\
        }";
        let parsed = LayerConfig::from_json_str(partial).expect("partial with BOM");
        assert!(parsed.enabled);
        assert!(parsed.gestures.only_touch);
        assert_eq!(parsed.gestures.pinch_trigger_distance, 0.25);
        assert_eq!(parsed.gestures.activation_delay, 50);
        assert_eq!(parsed.pose.mode, PoseProfileMode::OpenXrSpecGrip);
        assert_eq!(parsed.pose.aim_mode, AimPoseMode::PreferFbAim);
        assert_eq!(parsed.pose.left_aim_position_offset, [0.0, -0.04, -0.04]);
        assert_eq!(parsed.float_to_bool_threshold, 0.5);
    }

    #[test]
    fn sanitize_stress_and_idempotence() {
        let cases = vec![
            LayerConfig {
                enabled: true,
                gestures: HandTrackingInteractionConfig {
                    only_touch: false,
                    pinch_touch_distance: f32::NAN,
                    pinch_trigger_distance: f32::INFINITY,
                    curl_touch_distance: -10.0,
                    curl_trigger_distance: f32::NEG_INFINITY,
                    joystick_deadzone: 150.0,
                    joystick_offset_horizontal: f32::NAN,
                    joystick_offset_vertical: -999.0,
                    joystick_range: 0.0,
                    activation_delay: u32::MAX,
                    deactivation_delay: 999_999,
                    repeat_delay: 20_000,
                },
                pose: PoseOffsetConfig {
                    mode: PoseProfileMode::Custom,
                    left_position_offset: [f32::NAN, f32::INFINITY, -100.0],
                    left_rotation_offset_deg: [f32::NEG_INFINITY, 720.0, f32::NAN],
                    aim_mode: AimPoseMode::GripOffsetOnly,
                    left_aim_position_offset: [50.0, -50.0, f32::NAN],
                    left_aim_rotation_offset_deg: [-999.0, f32::INFINITY, 999.0],
                },
                float_to_bool_threshold: f32::NAN,
                float_to_bool_hysteresis: -0.5,
                physical_controller_grace_ms: u32::MAX,
            },
            LayerConfig {
                enabled: true,
                gestures: HandTrackingInteractionConfig {
                    pinch_touch_distance: 5.0,
                    pinch_trigger_distance: 1.0, // touch > trigger
                    curl_touch_distance: 8.0,
                    curl_trigger_distance: 2.0, // touch > trigger
                    joystick_deadzone: -5.0,
                    joystick_range: -2.0,
                    ..HandTrackingInteractionConfig::default()
                },
                pose: PoseOffsetConfig::default(),
                float_to_bool_threshold: 0.0, // <= 0
                float_to_bool_hysteresis: 0.9,
                physical_controller_grace_ms: 0,
            },
            LayerConfig {
                enabled: true,
                gestures: HandTrackingInteractionConfig {
                    joystick_range: 0.001, // positive but below MIN_JOYSTICK_RANGE_CM
                    ..HandTrackingInteractionConfig::default()
                },
                pose: PoseOffsetConfig::default(),
                float_to_bool_threshold: 1.5, // >= 1
                float_to_bool_hysteresis: f32::INFINITY,
                physical_controller_grace_ms: 500,
            },
        ];

        for raw in cases {
            let once = raw.sanitized();
            let twice = once.clone().sanitized();
            assert_eq!(once, twice, "sanitize must be strictly idempotent");

            assert!(once.gestures.pinch_touch_distance.is_finite());
            assert!(once.gestures.pinch_trigger_distance >= once.gestures.pinch_touch_distance);
            assert!(once.gestures.curl_touch_distance.is_finite());
            assert!(once.gestures.curl_trigger_distance >= once.gestures.curl_touch_distance);
            assert!((0.0..=100.0).contains(&once.gestures.joystick_deadzone));
            assert!(
                (MIN_JOYSTICK_RANGE_CM..=MAX_JOYSTICK_RANGE_CM)
                    .contains(&once.gestures.joystick_range)
            );
            assert!(once.gestures.activation_delay <= MAX_DELAY_MS);
            assert!(once.gestures.deactivation_delay <= MAX_DELAY_MS);
            assert!(once.gestures.repeat_delay <= MAX_DELAY_MS);

            for v in once
                .pose
                .left_position_offset
                .iter()
                .chain(once.pose.left_aim_position_offset.iter())
            {
                assert!(v.is_finite() && (-MAX_POSITION_OFFSET_M..=MAX_POSITION_OFFSET_M).contains(v));
            }
            for v in once
                .pose
                .left_rotation_offset_deg
                .iter()
                .chain(once.pose.left_aim_rotation_offset_deg.iter())
            {
                assert!(
                    v.is_finite() && (-MAX_ROTATION_OFFSET_DEG..=MAX_ROTATION_OFFSET_DEG).contains(v)
                );
            }

            let t = once.float_to_bool_threshold;
            let h = once.float_to_bool_hysteresis;
            assert!((MIN_BOOL_THRESHOLD..=MAX_BOOL_THRESHOLD).contains(&t));
            assert!(h >= 0.0);
            assert!(t - h > 0.0, "lower hysteresis bound must be > 0.0");
            assert!(t + h < 1.0, "upper hysteresis bound must be < 1.0");
            assert!(once.physical_controller_grace_ms <= MAX_GRACE_MS);
        }
    }

    #[test]
    fn load_from_paths_with_env_priority_and_fallbacks() {
        let dir = TestDir::new("loader");
        let env_good = dir.write_bytes(
            "env_good.json",
            br#"{"physical_controller_grace_ms": 777, "gestures": {"only_touch": true}}"#,
        );
        let cand_good = dir.write_bytes(
            "cand_good.json",
            br#"{"physical_controller_grace_ms": 333, "pose": {"mode": "raw_alvr_steam_vr"}}"#,
        );
        let bad_json = dir.write_bytes("bad.json", b"{not valid json");
        let bad_utf8 = dir.write_bytes("bad_utf8.json", &[0xff, 0xfe, 0x00, 0x80]);
        let missing = dir.path().join("does_not_exist.json");
        let subdir = dir.path().join("a_directory");
        fs::create_dir_all(&subdir).expect("create subdir");

        // 1. Env override wins over valid candidate
        let cfg = LayerConfig::load_from_paths_with_env(Some(&env_good), &[&cand_good]);
        assert_eq!(cfg.physical_controller_grace_ms, 777);
        assert!(cfg.gestures.only_touch);

        // 2. Missing or malformed env override falls through cleanly to first valid candidate
        let cfg = LayerConfig::load_from_paths_with_env(Some(&missing), &[&bad_json, &cand_good]);
        assert_eq!(cfg.physical_controller_grace_ms, 333);
        assert_eq!(cfg.pose.mode, PoseProfileMode::RawAlvrSteamVr);

        let cfg = LayerConfig::load_from_paths_with_env(Some(&bad_json), &[&cand_good]);
        assert_eq!(cfg.physical_controller_grace_ms, 333);

        // 3. Empty env override "" is ignored cleanly
        let cfg = LayerConfig::load_from_paths_with_env(Some(Path::new("")), &[&cand_good]);
        assert_eq!(cfg.physical_controller_grace_ms, 333);

        // 4. All candidates invalid/missing/directory/invalid-UTF-8 -> defaults without panicking
        let cfg = LayerConfig::load_from_paths_with_env(
            None,
            &[
                Path::new(""),
                &missing,
                &subdir,
                &bad_json,
                &bad_utf8,
            ],
        );
        assert_eq!(cfg, LayerConfig::default());

        // 5. Oversized file (> 1 MiB) rejected cleanly and falls back to defaults
        let huge = dir.write_bytes("huge.json", &vec![b' '; (MAX_CONFIG_BYTES as usize) + 64]);
        assert!(LayerConfig::load_from_file(&huge).is_err());
        let cfg = LayerConfig::load_from_paths_with_env(Some(&huge), &[&missing]);
        assert_eq!(cfg, LayerConfig::default());
    }
}
