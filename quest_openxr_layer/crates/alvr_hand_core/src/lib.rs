//! Pure-Rust ALVR hand-tracking gesture engine, pose/velocity synthesizer,
//! virtual controller state, and configuration loader for Quest OpenXR.

use serde::{Deserialize, Serialize};

/// Number of OpenXR `XR_EXT_hand_tracking` joints (`XR_HAND_JOINT_COUNT_EXT`).
pub const HAND_JOINT_COUNT: usize = 26;

/// Handedness discriminator used across gesture, pose, and binding engines.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum HandSide {
    Left,
    Right,
}

impl HandSide {
    /// Zero-based array index (`0` for Left, `1` for Right).
    pub const fn index(self) -> usize {
        match self {
            Self::Left => 0,
            Self::Right => 1,
        }
    }

    /// Canonical OpenXR top-level user path for this hand.
    pub const fn user_path(self) -> &'static str {
        match self {
            Self::Left => "/user/hand/left",
            Self::Right => "/user/hand/right",
        }
    }
}

pub mod config;
pub mod gestures;
pub mod pose;
pub mod virtual_controller;

pub use config::{
    AimPoseMode, ConfigError, HandTrackingInteractionConfig, LayerConfig, PoseOffsetConfig,
    PoseProfileMode,
};
pub use gestures::{HandGesture, HandGestureEngine, HandGestureId, HandJoint, HysteresisState};
pub use pose::{HandJointSample, Pose, PoseSynthesizer, SynthesizedPose};
pub use virtual_controller::{
    aggregate_bool, aggregate_float, aggregate_vec2, parse_hand_user_path, resolve_binding,
    InteractionProfile, PoseSource, ResolvedBinding, VirtualComponent, VirtualHandState,
};

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{EulerRot, Quat, Vec3};

    #[test]
    fn crate_root_exports_and_hand_side_helpers() {
        assert_eq!(HAND_JOINT_COUNT, 26);
        assert_eq!(HandSide::Left.index(), 0);
        assert_eq!(HandSide::Right.index(), 1);
        assert_eq!(HandSide::Left.user_path(), "/user/hand/left");
        assert_eq!(HandSide::Right.user_path(), "/user/hand/right");
        assert_eq!(
            parse_hand_user_path(HandSide::Left.user_path()),
            Some(HandSide::Left)
        );
        assert_eq!(
            parse_hand_user_path(HandSide::Right.user_path()),
            Some(HandSide::Right)
        );

        // Pose::relative_to identity: base * target.relative_to(base) == target
        let base = Pose {
            orientation: Quat::from_euler(EulerRot::XYZ, 0.3, -0.4, 0.5),
            position: Vec3::new(0.2, 1.1, -0.4),
        };
        let target = Pose {
            orientation: Quat::from_euler(EulerRot::XYZ, -0.2, 0.7, -0.1),
            position: Vec3::new(-0.3, 1.4, -0.8),
        };
        let rel = target.relative_to(base);
        let recon = base * rel;
        assert!((recon.position - target.position).length() < 1e-5);
        assert!((1.0 - recon.orientation.dot(target.orientation).abs()) < 1e-6);

        // Shared quaternion sanitization helper matches across modules
        let q = Quat::from_xyzw(0.0, 0.0, 0.0, 0.0);
        assert_eq!(
            gestures::safe_normalize_quat(q),
            pose::normalize_quat_or_identity(q)
        );

        // Direct evaluate_gestures inspection returns all 8 ALVR gestures in order
        let mut engine = HandGestureEngine::new();
        let joints = [Pose::IDENTITY; HAND_JOINT_COUNT];
        let g = engine.evaluate_gestures(
            HandSide::Left,
            &joints,
            0,
            &HandTrackingInteractionConfig::default(),
        );
        assert_eq!(g[0].id, HandGestureId::ThumbIndexPinch);
        assert_eq!(g[1].id, HandGestureId::ThumbMiddlePinch);
        assert_eq!(g[2].id, HandGestureId::ThumbRingPinch);
        assert_eq!(g[3].id, HandGestureId::ThumbLittlePinch);
        assert_eq!(g[4].id, HandGestureId::GripCurl);
        assert_eq!(g[5].id, HandGestureId::ThumbCurl);
        assert_eq!(g[6].id, HandGestureId::JoystickX);
        assert_eq!(g[7].id, HandGestureId::JoystickY);
    }
}

