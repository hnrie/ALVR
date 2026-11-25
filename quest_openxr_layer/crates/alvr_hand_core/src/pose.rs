//! Hand-to-controller grip, aim, palm-surface pose synthesis, and `baseSpace` velocity math.

use crate::config::{
    AimPoseMode, PoseOffsetConfig, PoseProfileMode, DEFAULT_LEFT_AIM_POSITION_OFFSET,
    DEFAULT_LEFT_AIM_ROTATION_OFFSET_DEG, DEFAULT_LEFT_POSITION_OFFSET,
    DEFAULT_LEFT_ROTATION_OFFSET_DEG,
};
use crate::{HandSide, HAND_JOINT_COUNT};
use glam::{EulerRot, Quat, Vec3};
use std::f32::consts::FRAC_PI_2;
use std::ops::Mul;

pub const DEG_TO_RAD: f32 = std::f32::consts::PI / 180.0;

/// ALVR canonical left-hand-to-SteamVR-controller offset (`alvr/session/src/settings.rs:2150-2157`).
pub const ALVR_LEFT_HAND_POSITION_OFFSET: [f32; 3] = DEFAULT_LEFT_POSITION_OFFSET;
pub const ALVR_LEFT_HAND_ROTATION_OFFSET_DEG: [f32; 3] = DEFAULT_LEFT_ROTATION_OFFSET_DEG;

/// ALVR physical-controller server offset `O_server` (`settings.rs:2144`, `server_openvr/src/tracking.rs:272-300`).
pub const ALVR_SERVER_CONTROLLER_Z_OFFSET: f32 = -0.11;

/// ALVR Quest 2/3/3S/Pro physical-controller client offset `O_client` (`client_openxr/src/interaction.rs:40-43`).
pub const META_QUEST_LEFT_CLIENT_POSITION_OFFSET: [f32; 3] = [-0.005, -0.005, 0.0];
pub const META_QUEST_CLIENT_ROTATION_X_DEG: f32 = -15.0;

/// Re-exported default left aim offsets (single source of truth in [`crate::config`]).
pub const DEFAULT_AIM_POSITION_OFFSET: [f32; 3] = DEFAULT_LEFT_AIM_POSITION_OFFSET;
pub const DEFAULT_AIM_ROTATION_OFFSET_DEG: [f32; 3] = DEFAULT_LEFT_AIM_ROTATION_OFFSET_DEG;

/// Finite-difference velocity `dt` bounds (nanoseconds).
pub const MIN_VELOCITY_DT_NS: i64 = 1_000; // 1 µs (guards against sub-µs division blowup)
pub const MAX_VELOCITY_DT_NS: i64 = 500_000_000; // 0.5 s (rejects stale gaps)

const QUAT_LEN_SQ_EPS: f32 = 1e-12;
const AIM_SOURCE_GRIP: u8 = 0;
const AIM_SOURCE_FB: u8 = 1;

/// Returns `true` iff `q` is finite and non-zero (safe to normalize without division by zero).
pub fn is_valid_quat(q: Quat) -> bool {
    if !q.is_finite() {
        return false;
    }
    let len_sq = q.length_squared();
    if len_sq.is_finite() {
        len_sq > QUAT_LEN_SQ_EPS
    } else {
        let max_c = q.x.abs().max(q.y.abs()).max(q.z.abs()).max(q.w.abs());
        max_c.is_finite() && max_c > 0.0
    }
}

/// Normalizes `q` into a guaranteed finite unit quaternion, falling back to `Quat::IDENTITY`
/// on `NaN`, `±Inf`, zero, or near-zero inputs without panicking (`glam-0.32.1` has no `try_normalize`).
pub fn normalize_quat_or_identity(q: Quat) -> Quat {
    if !q.is_finite() {
        return Quat::IDENTITY;
    }
    let len_sq = q.length_squared();
    if len_sq.is_finite() {
        if len_sq <= QUAT_LEN_SQ_EPS {
            return Quat::IDENTITY;
        }
        let inv = len_sq.sqrt().recip();
        let out = Quat::from_xyzw(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
        if out.is_finite() {
            out
        } else {
            Quat::IDENTITY
        }
    } else {
        // Finite huge components whose square overflowed `f32::MAX`: rescale by max component.
        let max_c = q.x.abs().max(q.y.abs()).max(q.z.abs()).max(q.w.abs());
        if !max_c.is_finite() || max_c <= 0.0 {
            return Quat::IDENTITY;
        }
        let sx = q.x / max_c;
        let sy = q.y / max_c;
        let sz = q.z / max_c;
        let sw = q.w / max_c;
        let scaled_len = (sx * sx + sy * sy + sz * sz + sw * sw).sqrt();
        if !scaled_len.is_finite() || scaled_len <= 0.0 {
            return Quat::IDENTITY;
        }
        let out = Quat::from_xyzw(
            sx / scaled_len,
            sy / scaled_len,
            sz / scaled_len,
            sw / scaled_len,
        );
        if out.is_finite() {
            out
        } else {
            Quat::IDENTITY
        }
    }
}

/// Right-handed OpenXR rigid-body pose (`+X` right, `+Y` up, `-Z` forward).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose {
    pub orientation: Quat,
    pub position: Vec3,
}

impl Pose {
    pub const IDENTITY: Self = Self {
        orientation: Quat::IDENTITY,
        position: Vec3::ZERO,
    };

    /// Returns `true` iff `position` is finite and `orientation` is a finite non-zero quaternion.
    pub fn is_valid(self) -> bool {
        self.position.is_finite() && is_valid_quat(self.orientation)
    }

    /// Sanitizes position (non-finite -> `Vec3::ZERO`) and normalizes orientation (`Quat::IDENTITY` on invalid).
    pub fn normalize_or_identity(self) -> Self {
        Self {
            orientation: normalize_quat_or_identity(self.orientation),
            position: if self.position.is_finite() {
                self.position
            } else {
                Vec3::ZERO
            },
        }
    }

    /// Rigid transform inverse (`self * self.inverse() == IDENTITY`).
    pub fn inverse(self) -> Self {
        let q_inv = normalize_quat_or_identity(self.orientation).conjugate();
        let p = if self.position.is_finite() {
            self.position
        } else {
            Vec3::ZERO
        };
        let pos = q_inv * -p;
        Self {
            orientation: q_inv,
            position: if pos.is_finite() { pos } else { Vec3::ZERO },
        }
    }

    /// Expresses `self` in the frame of `base` (`base.inverse() * self`).
    pub fn relative_to(self, base: Pose) -> Pose {
        base.inverse() * self
    }
}

impl Mul<Pose> for Pose {
    type Output = Pose;

    fn mul(self, rhs: Pose) -> Self::Output {
        let q_lhs = normalize_quat_or_identity(self.orientation);
        let q_rhs = normalize_quat_or_identity(rhs.orientation);
        let p_lhs = if self.position.is_finite() {
            self.position
        } else {
            Vec3::ZERO
        };
        let p_rhs = if rhs.position.is_finite() {
            rhs.position
        } else {
            Vec3::ZERO
        };
        let pos = p_lhs + q_lhs * p_rhs;
        Pose {
            orientation: normalize_quat_or_identity(q_lhs * q_rhs),
            position: if pos.is_finite() { pos } else { Vec3::ZERO },
        }
    }
}

/// Single joint sample from `xrLocateHandJointsEXT` (+ optional `XrHandJointVelocitiesEXT`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct HandJointSample {
    pub pose: Pose,
    pub position_valid: bool,
    pub orientation_valid: bool,
    pub position_tracked: bool,
    pub orientation_tracked: bool,
    pub radius: f32,
    pub linear_velocity: Option<Vec3>,
    pub angular_velocity: Option<Vec3>,
}

/// Synthesized OpenXR controller pose with validity/tracking flags and `baseSpace`-frame velocity.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SynthesizedPose {
    pub pose: Pose,
    pub position_valid: bool,
    pub orientation_valid: bool,
    pub position_tracked: bool,
    pub orientation_tracked: bool,
    pub linear_velocity: Option<Vec3>,
    pub angular_velocity: Option<Vec3>,
}

impl SynthesizedPose {
    /// Composes a rigid local child offset (e.g., `poseInActionSpace` or grip-to-aim) on the right,
    /// propagating `baseSpace` rigid-body linear velocity (`v + omega x (q * p_local)`).
    pub fn with_local_offset(self, local_offset: Pose) -> Self {
        let local = local_offset.normalize_or_identity();
        let q_self = if self.orientation_valid {
            normalize_quat_or_identity(self.pose.orientation)
        } else {
            Quat::IDENTITY
        };
        let p_self = if self.position_valid && self.pose.position.is_finite() {
            self.pose.position
        } else {
            Vec3::ZERO
        };

        let orientation = if self.orientation_valid {
            normalize_quat_or_identity(q_self * local.orientation)
        } else {
            Quat::IDENTITY
        };

        let position = if self.position_valid {
            let p = if self.orientation_valid {
                p_self + q_self * local.position
            } else {
                p_self
            };
            if p.is_finite() {
                p
            } else {
                Vec3::ZERO
            }
        } else {
            Vec3::ZERO
        };

        let angular_velocity = if self.orientation_valid {
            self.angular_velocity.filter(|w| w.is_finite())
        } else {
            None
        };

        let linear_velocity = if self.position_valid {
            let base_lin = self.linear_velocity.filter(|v| v.is_finite());
            if !self.orientation_valid || local.position.length_squared() == 0.0 {
                base_lin
            } else if let (Some(v), Some(w)) = (base_lin, angular_velocity) {
                let v_out = v + w.cross(q_self * local.position);
                v_out.is_finite().then_some(v_out)
            } else {
                base_lin
            }
        } else {
            None
        };

        Self {
            pose: Pose {
                orientation,
                position,
            },
            position_valid: self.position_valid,
            orientation_valid: self.orientation_valid,
            position_tracked: self.position_valid && self.position_tracked,
            orientation_tracked: self.orientation_valid && self.orientation_tracked,
            linear_velocity,
            angular_velocity,
        }
    }
}

/// Returns ALVR's physical-controller server offset `O_server` (`[0, 0, -0.11]`, identity).
pub fn alvr_server_controller_offset() -> Pose {
    Pose {
        orientation: Quat::IDENTITY,
        position: Vec3::new(0.0, 0.0, ALVR_SERVER_CONTROLLER_Z_OFFSET),
    }
}

/// Returns ALVR's Meta Quest 2/3/3S/Pro physical-controller client offset `O_client`.
///
/// Positive-`w` form of `alvr/client_openxr/src/interaction.rs:40-43, 65-72`:
/// - Left: `[-0.005, -0.005, 0.0]`, `Rx(-15°)`
/// - Right: `[+0.005, -0.005, 0.0]`, `Rx(-15°)` (identical in `SO(3)` to `(-q.x, q.y, q.z, -q.w)`)
pub fn meta_quest_client_controller_offset(hand: HandSide) -> Pose {
    Pose {
        orientation: Quat::from_rotation_x(META_QUEST_CLIENT_ROTATION_X_DEG * DEG_TO_RAD),
        position: mirror_position(hand, META_QUEST_LEFT_CLIENT_POSITION_OFFSET),
    }
}

fn sanitize_vec3(v: [f32; 3]) -> Vec3 {
    Vec3::new(
        if v[0].is_finite() { v[0] } else { 0.0 },
        if v[1].is_finite() { v[1] } else { 0.0 },
        if v[2].is_finite() { v[2] } else { 0.0 },
    )
}

fn mirror_position(hand: HandSide, left_pos: [f32; 3]) -> Vec3 {
    let v = sanitize_vec3(left_pos);
    match hand {
        HandSide::Left => v,
        HandSide::Right => Vec3::new(-v.x, v.y, v.z),
    }
}

fn mirror_euler_xyz_deg(hand: HandSide, left_deg: [f32; 3]) -> Quat {
    if !left_deg.iter().all(|c| c.is_finite()) {
        return Quat::IDENTITY;
    }
    let (rx, ry, rz) = match hand {
        HandSide::Left => (left_deg[0], left_deg[1], left_deg[2]),
        HandSide::Right => (left_deg[0], -left_deg[1], -left_deg[2]),
    };
    normalize_quat_or_identity(Quat::from_euler(
        EulerRot::XYZ,
        rx * DEG_TO_RAD,
        ry * DEG_TO_RAD,
        rz * DEG_TO_RAD,
    ))
}

/// Post-rotation translation offset matching `alvr/server_openvr/src/tracking.rs:219-223`:
/// `Q_ctrl = Q_palm * Q_off`, `P_ctrl = P_palm + Q_ctrl * P_off`.
fn apply_post_rot_offset(palm: Pose, q_off: Quat, p_off: Vec3) -> Pose {
    let q_palm = normalize_quat_or_identity(palm.orientation);
    let q_ctrl = normalize_quat_or_identity(q_palm * q_off);
    let p_ctrl = palm.position + q_ctrl * p_off;
    Pose {
        orientation: q_ctrl,
        position: if p_ctrl.is_finite() {
            p_ctrl
        } else {
            Vec3::ZERO
        },
    }
}

fn palm_surface_rotation(hand: HandSide) -> Quat {
    match hand {
        HandSide::Left => Quat::from_rotation_z(-FRAC_PI_2),
        HandSide::Right => Quat::from_rotation_z(FRAC_PI_2),
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct PalmFallbackState {
    last_orientation: Option<Quat>,
    last_position: Option<Vec3>,
}

#[derive(Debug, Clone, Copy)]
struct ResolvedPalmAnchor {
    orientation: Quat,
    position: Vec3,
    orientation_valid: bool,
    position_valid: bool,
    orientation_tracked: bool,
    position_tracked: bool,
    live_ori_valid: bool,
    live_pos_valid: bool,
}

impl PalmFallbackState {
    fn resolve_and_update(&mut self, palm: &HandJointSample) -> ResolvedPalmAnchor {
        let live_ori_valid = palm.orientation_valid && is_valid_quat(palm.pose.orientation);
        let live_pos_valid = palm.position_valid && palm.pose.position.is_finite();

        let (orientation, orientation_valid, orientation_tracked) = if live_ori_valid {
            let q = normalize_quat_or_identity(palm.pose.orientation);
            self.last_orientation = Some(q);
            (q, true, palm.orientation_tracked)
        } else if let Some(prev_q) = self.last_orientation {
            (prev_q, true, false)
        } else {
            (Quat::IDENTITY, false, false)
        };

        let (position, position_valid, position_tracked) = if live_pos_valid {
            let p = palm.pose.position;
            self.last_position = Some(p);
            (p, true, palm.position_tracked)
        } else if let Some(prev_p) = self.last_position {
            (prev_p, true, false)
        } else {
            (Vec3::ZERO, false, false)
        };

        ResolvedPalmAnchor {
            orientation,
            position,
            orientation_valid,
            position_valid,
            orientation_tracked,
            position_tracked,
            live_ori_valid,
            live_pos_valid,
        }
    }

    fn clear(&mut self) {
        self.last_orientation = None;
        self.last_position = None;
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct FiniteDiffCache {
    prev_pos: Option<(i64, Vec3)>,
    prev_ori: Option<(i64, Quat)>,
    source_tag: u8,
}

impl FiniteDiffCache {
    fn clear(&mut self) {
        self.prev_pos = None;
        self.prev_ori = None;
        self.source_tag = 0;
    }

    fn update(
        &mut self,
        source_tag: u8,
        time_ns: i64,
        live_pos: Option<Vec3>,
        live_ori: Option<Quat>,
    ) -> (Option<Vec3>, Option<Vec3>) {
        if self.source_tag != source_tag {
            self.prev_pos = None;
            self.prev_ori = None;
            self.source_tag = source_tag;
        }

        let fd_lin = match live_pos {
            Some(p_curr) => {
                let (vel, should_store) = match self.prev_pos {
                    Some((prev_t, p_prev)) => {
                        let dt_ns = i128::from(time_ns) - i128::from(prev_t);
                        if dt_ns < 0 {
                            // Historical/reprojection query: never clobber the newer anchor.
                            (None, false)
                        } else if dt_ns >= i128::from(MIN_VELOCITY_DT_NS)
                            && dt_ns <= i128::from(MAX_VELOCITY_DT_NS)
                        {
                            let dt_s = (dt_ns as f64) * 1e-9;
                            let lin = ((p_curr - p_prev).as_dvec3() / dt_s).as_vec3();
                            (lin.is_finite().then_some(lin), true)
                        } else {
                            (None, dt_ns > 0)
                        }
                    }
                    None => (None, true),
                };
                if should_store {
                    self.prev_pos = Some((time_ns, p_curr));
                }
                vel
            }
            None => {
                self.prev_pos = None;
                None
            }
        };

        let fd_ang = match live_ori {
            Some(q_curr) => {
                let (vel, should_store) = match self.prev_ori {
                    Some((prev_t, q_prev)) => {
                        let dt_ns = i128::from(time_ns) - i128::from(prev_t);
                        if dt_ns < 0 {
                            // Historical/reprojection query: never clobber the newer anchor.
                            (None, false)
                        } else if dt_ns >= i128::from(MIN_VELOCITY_DT_NS)
                            && dt_ns <= i128::from(MAX_VELOCITY_DT_NS)
                        {
                            let dt_s = (dt_ns as f64) * 1e-9;
                            let mut dq = normalize_quat_or_identity(q_curr * q_prev.conjugate());
                            if dq.w < 0.0 {
                                dq = -dq;
                            }
                            let (axis, angle) = dq.to_axis_angle();
                            let omega = axis * ((f64::from(angle) / dt_s) as f32);
                            (omega.is_finite().then_some(omega), true)
                        } else {
                            (None, dt_ns > 0)
                        }
                    }
                    None => (None, true),
                };
                if should_store {
                    self.prev_ori = Some((time_ns, q_curr));
                }
                vel
            }
            None => {
                self.prev_ori = None;
                None
            }
        };

        (fd_lin, fd_ang)
    }
}

/// Number of concurrent `baseSpace` coordinate frames cached per hand without heap allocation.
const MAX_CACHED_BASE_SPACES: usize = 8;

#[derive(Debug, Clone, Copy, Default)]
struct HandPoseState {
    palm_fallback: PalmFallbackState,
    grip_vel: FiniteDiffCache,
    aim_vel: FiniteDiffCache,
    palm_vel: FiniteDiffCache,
}

impl HandPoseState {
    fn clear(&mut self) {
        self.palm_fallback.clear();
        self.grip_vel.clear();
        self.aim_vel.clear();
        self.palm_vel.clear();
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct BaseSpaceSlot {
    occupied: bool,
    base_space_id: u64,
    last_used_seq: u64,
    state: HandPoseState,
}

#[derive(Debug, Clone, Copy, Default)]
struct HandSpaceCache {
    seq: u64,
    slots: [BaseSpaceSlot; MAX_CACHED_BASE_SPACES],
}

impl HandSpaceCache {
    fn slot_mut(&mut self, base_space_id: u64) -> &mut HandPoseState {
        self.seq = self.seq.wrapping_add(1);
        let next_seq = self.seq;

        // 1. Existing matching slot.
        if let Some(idx) = self
            .slots
            .iter()
            .position(|s| s.occupied && s.base_space_id == base_space_id)
        {
            self.slots[idx].last_used_seq = next_seq;
            return &mut self.slots[idx].state;
        }

        // 2. First free slot, or least-recently-used slot.
        let target_idx = self
            .slots
            .iter()
            .position(|s| !s.occupied)
            .unwrap_or_else(|| {
                self.slots
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, s)| s.last_used_seq)
                    .map(|(i, _)| i)
                    .unwrap_or(0)
            });

        let slot = &mut self.slots[target_idx];
        slot.occupied = true;
        slot.base_space_id = base_space_id;
        slot.last_used_seq = next_seq;
        slot.state.clear();
        &mut slot.state
    }

    fn clear_base_space(&mut self, base_space_id: u64) {
        for slot in &mut self.slots {
            if slot.occupied && slot.base_space_id == base_space_id {
                *slot = BaseSpaceSlot::default();
            }
        }
    }

    fn clear_all(&mut self) {
        self.seq = 0;
        self.slots = [BaseSpaceSlot::default(); MAX_CACHED_BASE_SPACES];
    }
}

/// Stateful per-hand controller pose and `baseSpace` velocity synthesizer.
///
/// Holds per-hand, per-`baseSpace` `last_palm_pose` fallback state and isolated finite-difference
/// caches for `/grip/pose`, `/aim/pose`, and `/palm_ext/pose` (`/grip_surface/pose`) so queries in
/// multiple reference spaces (`LOCAL`, `STAGE`, `VIEW`) never cross-contaminate velocities or poses.
#[derive(Debug, Clone, Default)]
pub struct PoseSynthesizer {
    hands: [HandSpaceCache; 2],
}

impl PoseSynthesizer {
    /// Creates a fresh [`PoseSynthesizer`] with empty fallback and velocity caches.
    pub fn new() -> Self {
        Self::default()
    }

    /// Synthesizes `/input/grip/pose` using the default `baseSpace` cache (`base_space_id = 0`).
    pub fn compute_grip_pose(
        &mut self,
        hand: HandSide,
        joints: &[HandJointSample; HAND_JOINT_COUNT],
        time_ns: i64,
        cfg: &PoseOffsetConfig,
    ) -> SynthesizedPose {
        self.compute_grip_pose_in_space(hand, 0, joints, time_ns, cfg)
    }

    /// Synthesizes `/input/grip/pose` isolated to a specific `base_space_id`.
    pub fn compute_grip_pose_in_space(
        &mut self,
        hand: HandSide,
        base_space_id: u64,
        joints: &[HandJointSample; HAND_JOINT_COUNT],
        time_ns: i64,
        cfg: &PoseOffsetConfig,
    ) -> SynthesizedPose {
        let palm_sample = &joints[0];
        let state = self.hands[hand.index()].slot_mut(base_space_id);
        let anchor = state.palm_fallback.resolve_and_update(palm_sample);
        let out_pose = resolve_grip_from_palm(hand, anchor, cfg);

        let (linear_velocity, angular_velocity) = resolve_rigid_body_velocity(
            &mut state.grip_vel,
            0,
            time_ns,
            anchor,
            palm_sample,
            out_pose,
        );

        SynthesizedPose {
            pose: out_pose,
            position_valid: anchor.position_valid,
            orientation_valid: anchor.orientation_valid,
            position_tracked: anchor.position_tracked,
            orientation_tracked: anchor.orientation_tracked,
            linear_velocity,
            angular_velocity,
        }
    }

    /// Synthesizes `/input/aim/pose` using the default `baseSpace` cache (`base_space_id = 0`).
    pub fn compute_aim_pose(
        &mut self,
        hand: HandSide,
        joints: &[HandJointSample; HAND_JOINT_COUNT],
        fb_aim_pose: Option<Pose>,
        time_ns: i64,
        cfg: &PoseOffsetConfig,
    ) -> SynthesizedPose {
        self.compute_aim_pose_in_space(hand, 0, joints, fb_aim_pose, time_ns, cfg)
    }

    /// Synthesizes `/input/aim/pose` isolated to a specific `base_space_id`, preferring
    /// `fb_aim_pose` when `cfg.aim_mode == PreferFbAim` and valid, otherwise deriving from
    /// synthesized grip pose + grip-to-aim offset.
    pub fn compute_aim_pose_in_space(
        &mut self,
        hand: HandSide,
        base_space_id: u64,
        joints: &[HandJointSample; HAND_JOINT_COUNT],
        fb_aim_pose: Option<Pose>,
        time_ns: i64,
        cfg: &PoseOffsetConfig,
    ) -> SynthesizedPose {
        let palm_sample = &joints[0];
        let state = self.hands[hand.index()].slot_mut(base_space_id);
        // Always keep palm fallback fresh without mutating `grip_vel`.
        let anchor = state.palm_fallback.resolve_and_update(palm_sample);

        if cfg.aim_mode == AimPoseMode::PreferFbAim {
            if let Some(raw_fb) = fb_aim_pose {
                if raw_fb.is_valid() {
                    let fb_pose = raw_fb.normalize_or_identity();
                    let (linear_velocity, angular_velocity) = state.aim_vel.update(
                        AIM_SOURCE_FB,
                        time_ns,
                        Some(fb_pose.position),
                        Some(fb_pose.orientation),
                    );
                    return SynthesizedPose {
                        pose: fb_pose,
                        position_valid: true,
                        orientation_valid: true,
                        position_tracked: true,
                        orientation_tracked: true,
                        linear_velocity,
                        angular_velocity,
                    };
                }
            }
        }

        // Grip-derived fallback (never touches `grip_vel`!).
        let grip_pose = resolve_grip_from_palm(hand, anchor, cfg);
        let aim_in_grip = Pose {
            position: mirror_position(hand, cfg.left_aim_position_offset),
            orientation: mirror_euler_xyz_deg(hand, cfg.left_aim_rotation_offset_deg),
        };
        let out_pose = if anchor.orientation_valid {
            let composed = grip_pose * aim_in_grip;
            Pose {
                orientation: composed.orientation,
                position: if anchor.position_valid {
                    composed.position
                } else {
                    Vec3::ZERO
                },
            }
        } else {
            Pose {
                orientation: Quat::IDENTITY,
                position: if anchor.position_valid {
                    anchor.position
                } else {
                    Vec3::ZERO
                },
            }
        };

        let (linear_velocity, angular_velocity) = resolve_rigid_body_velocity(
            &mut state.aim_vel,
            AIM_SOURCE_GRIP,
            time_ns,
            anchor,
            palm_sample,
            out_pose,
        );

        SynthesizedPose {
            pose: out_pose,
            position_valid: anchor.position_valid,
            orientation_valid: anchor.orientation_valid,
            position_tracked: anchor.position_tracked,
            orientation_tracked: anchor.orientation_tracked,
            linear_velocity,
            angular_velocity,
        }
    }

    /// Synthesizes `/input/grip_surface/pose` and `/input/palm_ext/pose` using the default
    /// `baseSpace` cache (`base_space_id = 0`).
    pub fn compute_palm_pose(
        &mut self,
        hand: HandSide,
        joints: &[HandJointSample; HAND_JOINT_COUNT],
        time_ns: i64,
    ) -> SynthesizedPose {
        self.compute_palm_pose_in_space(hand, 0, joints, time_ns)
    }

    /// Synthesizes `/input/grip_surface/pose` and `/input/palm_ext/pose` isolated to `base_space_id`.
    pub fn compute_palm_pose_in_space(
        &mut self,
        hand: HandSide,
        base_space_id: u64,
        joints: &[HandJointSample; HAND_JOINT_COUNT],
        time_ns: i64,
    ) -> SynthesizedPose {
        let palm_sample = &joints[0];
        let state = self.hands[hand.index()].slot_mut(base_space_id);
        let anchor = state.palm_fallback.resolve_and_update(palm_sample);

        let out_pose = if anchor.orientation_valid {
            let q_out = normalize_quat_or_identity(anchor.orientation * palm_surface_rotation(hand));
            let r = if palm_sample.radius.is_finite() && palm_sample.radius > 0.0 {
                palm_sample.radius
            } else {
                0.0
            };
            let p_out = if anchor.position_valid {
                let shifted = anchor.position + anchor.orientation * Vec3::new(0.0, -r, 0.0);
                if shifted.is_finite() {
                    shifted
                } else {
                    Vec3::ZERO
                }
            } else {
                Vec3::ZERO
            };
            Pose {
                orientation: q_out,
                position: p_out,
            }
        } else {
            Pose {
                orientation: Quat::IDENTITY,
                position: if anchor.position_valid {
                    anchor.position
                } else {
                    Vec3::ZERO
                },
            }
        };

        let (linear_velocity, angular_velocity) = resolve_rigid_body_velocity(
            &mut state.palm_vel,
            0,
            time_ns,
            anchor,
            palm_sample,
            out_pose,
        );

        SynthesizedPose {
            pose: out_pose,
            position_valid: anchor.position_valid,
            orientation_valid: anchor.orientation_valid,
            position_tracked: anchor.position_tracked,
            orientation_tracked: anchor.orientation_tracked,
            linear_velocity,
            angular_velocity,
        }
    }

    /// Evicts cached pose fallback & velocity entries for a destroyed `baseSpace` handle.
    pub fn clear_base_space(&mut self, base_space_id: u64) {
        self.hands[0].clear_base_space(base_space_id);
        self.hands[1].clear_base_space(base_space_id);
    }

    /// Clears fallback palm pose and all velocity caches for a single hand.
    pub fn clear_hand(&mut self, hand: HandSide) {
        self.hands[hand.index()].clear_all();
    }

    /// Clears fallback palm pose and all velocity caches for both hands.
    pub fn reset(&mut self) {
        self.hands[0].clear_all();
        self.hands[1].clear_all();
    }
}

fn resolve_grip_from_palm(
    hand: HandSide,
    anchor: ResolvedPalmAnchor,
    cfg: &PoseOffsetConfig,
) -> Pose {
    if !anchor.orientation_valid {
        return Pose {
            orientation: Quat::IDENTITY,
            position: if anchor.position_valid {
                anchor.position
            } else {
                Vec3::ZERO
            },
        };
    }

    let palm_pose = Pose {
        orientation: anchor.orientation,
        position: if anchor.position_valid {
            anchor.position
        } else {
            Vec3::ZERO
        },
    };

    let raw = match cfg.mode {
        PoseProfileMode::RawAlvrSteamVr => {
            let q_off = mirror_euler_xyz_deg(hand, ALVR_LEFT_HAND_ROTATION_OFFSET_DEG);
            let p_off = mirror_position(hand, ALVR_LEFT_HAND_POSITION_OFFSET);
            apply_post_rot_offset(palm_pose, q_off, p_off)
        }
        PoseProfileMode::OpenXrSpecGrip => {
            let q_off = mirror_euler_xyz_deg(hand, ALVR_LEFT_HAND_ROTATION_OFFSET_DEG);
            let p_off = mirror_position(hand, ALVR_LEFT_HAND_POSITION_OFFSET);
            let steamvr = apply_post_rot_offset(palm_pose, q_off, p_off);
            steamvr * alvr_server_controller_offset().inverse()
        }
        PoseProfileMode::MetaQuestNativeGrip => {
            let q_off = mirror_euler_xyz_deg(hand, ALVR_LEFT_HAND_ROTATION_OFFSET_DEG);
            let p_off = mirror_position(hand, ALVR_LEFT_HAND_POSITION_OFFSET);
            let steamvr = apply_post_rot_offset(palm_pose, q_off, p_off);
            let xr_spec = steamvr * alvr_server_controller_offset().inverse();
            xr_spec * meta_quest_client_controller_offset(hand).inverse()
        }
        PoseProfileMode::Custom => {
            let q_off = mirror_euler_xyz_deg(hand, cfg.left_rotation_offset_deg);
            let p_off = mirror_position(hand, cfg.left_position_offset);
            apply_post_rot_offset(palm_pose, q_off, p_off)
        }
    };

    Pose {
        orientation: raw.orientation,
        position: if anchor.position_valid {
            raw.position
        } else {
            Vec3::ZERO
        },
    }
}

fn resolve_rigid_body_velocity(
    cache: &mut FiniteDiffCache,
    source_tag: u8,
    time_ns: i64,
    anchor: ResolvedPalmAnchor,
    palm_sample: &HandJointSample,
    out_pose: Pose,
) -> (Option<Vec3>, Option<Vec3>) {
    // Always keep finite-difference cache updated on live-valid components so seamless fallback works.
    let (fd_lin, fd_ang) = cache.update(
        source_tag,
        time_ns,
        anchor.live_pos_valid.then_some(out_pose.position),
        anchor.live_ori_valid.then_some(out_pose.orientation),
    );

    let angular_velocity = if anchor.live_ori_valid {
        palm_sample
            .angular_velocity
            .filter(|w| w.is_finite())
            .or(fd_ang)
    } else {
        None
    };

    let linear_velocity = if anchor.live_pos_valid {
        if let Some(v_palm) = palm_sample.linear_velocity.filter(|v| v.is_finite()) {
            let r_base = out_pose.position - anchor.position;
            let v_out = if anchor.orientation_valid && r_base.length_squared() > 0.0 {
                v_palm + angular_velocity.map_or(Vec3::ZERO, |w| w.cross(r_base))
            } else {
                v_palm
            };
            v_out.is_finite().then_some(v_out)
        } else {
            fd_lin
        }
    } else {
        None
    };

    (linear_velocity, angular_velocity)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_vec3_near(a: Vec3, b: Vec3, eps: f32) {
        assert!(
            (a - b).length() <= eps,
            "vec3 mismatch: got {a:?}, expected {b:?} (diff={}, eps={eps})",
            (a - b).length()
        );
    }

    fn assert_quat_same_rotation(a: Quat, b: Quat, eps: f32) {
        assert!(
            (a.length() - 1.0).abs() <= 1e-5,
            "quat a not unit: {a:?} (len={})",
            a.length()
        );
        let dot = a.dot(b).abs();
        assert!(
            (1.0 - dot) <= eps,
            "quat rotation mismatch: got {a:?}, expected {b:?} (|dot|={dot})"
        );
    }

    fn make_joints_with_palm(sample: HandJointSample) -> [HandJointSample; HAND_JOINT_COUNT] {
        let mut joints = [HandJointSample::default(); HAND_JOINT_COUNT];
        joints[0] = sample;
        joints
    }

    fn valid_palm(pose: Pose, radius: f32) -> HandJointSample {
        HandJointSample {
            pose,
            position_valid: true,
            orientation_valid: true,
            position_tracked: true,
            orientation_tracked: true,
            radius,
            linear_velocity: None,
            angular_velocity: None,
        }
    }

    fn mirror_yz_vec3(v: Vec3) -> Vec3 {
        Vec3::new(-v.x, v.y, v.z)
    }

    fn mirror_yz_quat(q: Quat) -> Quat {
        Quat::from_xyzw(q.x, -q.y, -q.z, q.w)
    }

    #[test]
    fn pose_algebra_and_quat_sanitization() {
        let a = Pose {
            orientation: Quat::from_euler(EulerRot::XYZ, 0.4, -0.7, 1.1),
            position: Vec3::new(0.3, -0.5, 1.2),
        };
        let b = Pose {
            orientation: Quat::from_euler(EulerRot::XYZ, -0.2, 0.9, -0.3),
            position: Vec3::new(-0.1, 0.8, -0.4),
        };
        let c = Pose {
            orientation: Quat::from_euler(EulerRot::XYZ, 0.6, 0.2, -1.3),
            position: Vec3::new(0.2, 0.1, -0.9),
        };

        // Identity & associativity
        let id_mul = a * Pose::IDENTITY;
        assert_vec3_near(id_mul.position, a.position, 1e-6);
        assert_quat_same_rotation(id_mul.orientation, a.orientation, 1e-6);

        let lhs = (a * b) * c;
        let rhs = a * (b * c);
        assert_vec3_near(lhs.position, rhs.position, 1e-5);
        assert_quat_same_rotation(lhs.orientation, rhs.orientation, 1e-5);

        // Inverse round-trips
        let prod1 = a * a.inverse();
        let prod2 = a.inverse() * a;
        assert_vec3_near(prod1.position, Vec3::ZERO, 1e-5);
        assert_quat_same_rotation(prod1.orientation, Quat::IDENTITY, 1e-6);
        assert_vec3_near(prod2.position, Vec3::ZERO, 1e-5);
        assert_quat_same_rotation(prod2.orientation, Quat::IDENTITY, 1e-6);

        // Sanitization edge cases
        let bad_quats = [
            Quat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            Quat::from_xyzw(1e-10, 0.0, 0.0, 0.0),
            Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0),
            Quat::from_xyzw(f32::INFINITY, 0.0, 0.0, 0.0),
            Quat::from_xyzw(f32::NEG_INFINITY, 1.0, 0.0, 0.0),
        ];
        for q in bad_quats {
            assert!(!is_valid_quat(q), "expected invalid quat: {q:?}");
            let norm = normalize_quat_or_identity(q);
            assert!(norm.is_finite());
            assert!((norm.length() - 1.0).abs() < 1e-6);
            assert_eq!(norm, Quat::IDENTITY);
        }

        // Non-unit finite & huge overflow-squared finite quats
        let q5 = Quat::from_xyzw(0.0, 0.0, 0.0, 5.0);
        assert!(is_valid_quat(q5));
        assert_quat_same_rotation(normalize_quat_or_identity(q5), Quat::IDENTITY, 1e-6);

        let q_huge = Quat::from_xyzw(1e25, 0.0, 0.0, 0.0);
        assert!(is_valid_quat(q_huge));
        let q_huge_n = normalize_quat_or_identity(q_huge);
        assert!(q_huge_n.is_finite());
        assert!((q_huge_n.length() - 1.0).abs() < 1e-6);
        assert_quat_same_rotation(q_huge_n, Quat::from_xyzw(1.0, 0.0, 0.0, 0.0), 1e-6);

        // Pose::is_valid
        assert!(Pose {
            orientation: q5,
            position: Vec3::new(1.0, 2.0, 3.0)
        }
        .is_valid());
        assert!(!Pose {
            orientation: Quat::IDENTITY,
            position: Vec3::new(f32::NAN, 0.0, 0.0)
        }
        .is_valid());
        assert!(!Pose {
            orientation: Quat::IDENTITY,
            position: Vec3::new(f32::INFINITY, 0.0, 0.0)
        }
        .is_valid());
        assert!(!Pose {
            orientation: Quat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            position: Vec3::ZERO
        }
        .is_valid());
    }

    #[test]
    fn all_four_grip_modes_numerical_spot_checks_and_identities() {
        let id_joints = make_joints_with_palm(valid_palm(Pose::IDENTITY, 0.015));

        // 1. RawAlvrSteamVr at identity palm
        let mut synth = PoseSynthesizer::new();
        let cfg_raw = PoseOffsetConfig {
            mode: PoseProfileMode::RawAlvrSteamVr,
            ..PoseOffsetConfig::default()
        };
        let raw_l = synth.compute_grip_pose(HandSide::Left, &id_joints, 1_000_000, &cfg_raw);
        let raw_r = synth.compute_grip_pose(HandSide::Right, &id_joints, 1_000_000, &cfg_raw);
        assert_vec3_near(raw_l.pose.position, Vec3::new(0.077782, -0.04, -0.106066), 1e-5);
        assert_quat_same_rotation(
            raw_l.pose.orientation,
            Quat::from_xyzw(0.270598, -0.270598, -0.653281, 0.653281),
            1e-5,
        );
        assert_vec3_near(raw_r.pose.position, Vec3::new(-0.077782, -0.04, -0.106066), 1e-5);
        assert_quat_same_rotation(
            raw_r.pose.orientation,
            Quat::from_xyzw(0.270598, 0.270598, 0.653281, 0.653281),
            1e-5,
        );

        // 2. OpenXrSpecGrip == direct post-rotation [±0.04, -0.02, -0.02] on arbitrary palm pose
        let arb_palm = Pose {
            orientation: Quat::from_euler(EulerRot::XYZ, 0.35, -0.82, 0.47),
            position: Vec3::new(-0.24, 1.18, -0.31),
        };
        let arb_joints = make_joints_with_palm(valid_palm(arb_palm, 0.015));
        let cfg_spec = PoseOffsetConfig {
            mode: PoseProfileMode::OpenXrSpecGrip,
            ..PoseOffsetConfig::default()
        };
        let cfg_meta = PoseOffsetConfig {
            mode: PoseProfileMode::MetaQuestNativeGrip,
            ..PoseOffsetConfig::default()
        };

        for hand in [HandSide::Left, HandSide::Right] {
            let mut s1 = PoseSynthesizer::new();
            let p_raw = s1.compute_grip_pose(hand, &arb_joints, 10_000_000, &cfg_raw).pose;
            let p_spec = s1.compute_grip_pose(hand, &arb_joints, 10_000_000, &cfg_spec).pose;
            let p_meta = s1.compute_grip_pose(hand, &arb_joints, 10_000_000, &cfg_meta).pose;

            // Direct post-rotation check for OpenXrSpecGrip
            let q_off = mirror_euler_xyz_deg(hand, ALVR_LEFT_HAND_ROTATION_OFFSET_DEG);
            let direct_spec = apply_post_rot_offset(
                arb_palm,
                q_off,
                mirror_position(hand, [0.04, -0.02, -0.02]),
            );
            assert_vec3_near(p_spec.position, direct_spec.position, 1e-5);
            assert_quat_same_rotation(p_spec.orientation, direct_spec.orientation, 1e-6);

            // Exact ALVR round-trip: (P_meta * O_client) * O_server == P_raw
            let roundtrip =
                (p_meta * meta_quest_client_controller_offset(hand)) * alvr_server_controller_offset();
            assert_vec3_near(roundtrip.position, p_raw.position, 1e-5);
            assert_quat_same_rotation(roundtrip.orientation, p_raw.orientation, 1e-6);

            // Also verify equivalence with ALVR interaction.rs right-hand mirror `(-q.x, q.y, q.z, -q.w)`
            if hand == HandSide::Right {
                let q_left = Quat::from_rotation_x(META_QUEST_CLIENT_ROTATION_X_DEG * DEG_TO_RAD);
                let q_right_alvr = Quat::from_xyzw(-q_left.x, q_left.y, q_left.z, -q_left.w);
                let o_client_alvr = Pose {
                    orientation: q_right_alvr,
                    position: Vec3::new(0.005, -0.005, 0.0),
                };
                let rt_alvr = (p_meta * o_client_alvr) * alvr_server_controller_offset();
                assert_vec3_near(rt_alvr.position, p_raw.position, 1e-5);
                assert_quat_same_rotation(rt_alvr.orientation, p_raw.orientation, 1e-6);
            }
        }

        // 3. MetaQuestNativeGrip spot-check at identity palm
        let mut s2 = PoseSynthesizer::new();
        let meta_l = s2.compute_grip_pose(HandSide::Left, &id_joints, 1_000_000, &cfg_meta);
        let meta_r = s2.compute_grip_pose(HandSide::Right, &id_joints, 1_000_000, &cfg_meta);
        assert_vec3_near(meta_l.pose.position, Vec3::new(0.0025, -0.045, -0.023954), 1e-5);
        assert_quat_same_rotation(
            meta_l.pose.orientation,
            Quat::from_xyzw(0.353553, -0.353553, -0.612372, 0.612372),
            1e-5,
        );
        assert_vec3_near(meta_r.pose.position, Vec3::new(-0.0025, -0.045, -0.023954), 1e-5);
        assert_quat_same_rotation(
            meta_r.pose.orientation,
            Quat::from_xyzw(0.353553, 0.353553, 0.612372, 0.612372),
            1e-5,
        );

        // 4. Custom mode + non-finite config sanitization
        let cfg_custom = PoseOffsetConfig {
            mode: PoseProfileMode::Custom,
            left_position_offset: [0.02, 0.03, -0.07],
            left_rotation_offset_deg: [15.0, -30.0, -45.0],
            ..PoseOffsetConfig::default()
        };
        let cust_r = s2.compute_grip_pose(HandSide::Right, &arb_joints, 2_000_000, &cfg_custom);
        let expected_q = arb_palm.orientation
            * Quat::from_euler(
                EulerRot::XYZ,
                15.0 * DEG_TO_RAD,
                30.0 * DEG_TO_RAD,
                45.0 * DEG_TO_RAD,
            );
        let expected_p = arb_palm.position + expected_q * Vec3::new(-0.02, 0.03, -0.07);
        assert_vec3_near(cust_r.pose.position, expected_p, 1e-5);
        assert_quat_same_rotation(cust_r.pose.orientation, expected_q, 1e-6);

        let cfg_nan = PoseOffsetConfig {
            mode: PoseProfileMode::Custom,
            left_position_offset: [f32::NAN, f32::INFINITY, -0.05],
            left_rotation_offset_deg: [0.0, f32::NAN, 0.0],
            ..PoseOffsetConfig::default()
        };
        let nan_out = s2.compute_grip_pose(HandSide::Left, &id_joints, 3_000_000, &cfg_nan);
        assert!(nan_out.pose.is_valid());
        assert_quat_same_rotation(nan_out.pose.orientation, Quat::IDENTITY, 1e-6);
        assert_vec3_near(nan_out.pose.position, Vec3::new(0.0, 0.0, -0.05), 1e-6);
    }

    #[test]
    fn yz_mirror_symmetry_across_all_modes_aim_and_palm() {
        let left_palm = Pose {
            orientation: Quat::from_euler(EulerRot::XYZ, 0.42, -0.55, 0.78),
            position: Vec3::new(-0.22, 1.05, -0.35),
        };
        let right_palm = Pose {
            orientation: mirror_yz_quat(left_palm.orientation),
            position: mirror_yz_vec3(left_palm.position),
        };
        let left_joints = make_joints_with_palm(valid_palm(left_palm, 0.014));
        let right_joints = make_joints_with_palm(valid_palm(right_palm, 0.014));

        for mode in [
            PoseProfileMode::MetaQuestNativeGrip,
            PoseProfileMode::OpenXrSpecGrip,
            PoseProfileMode::RawAlvrSteamVr,
            PoseProfileMode::Custom,
        ] {
            let cfg = PoseOffsetConfig {
                mode,
                left_position_offset: [0.03, -0.01, -0.08],
                left_rotation_offset_deg: [12.0, -35.0, -80.0],
                aim_mode: AimPoseMode::GripOffsetOnly,
                left_aim_position_offset: [0.01, -0.04, -0.04],
                left_aim_rotation_offset_deg: [-60.0, 5.0, -3.0],
            };
            let mut synth = PoseSynthesizer::new();
            let gl = synth.compute_grip_pose(HandSide::Left, &left_joints, 1_000_000, &cfg).pose;
            let gr = synth.compute_grip_pose(HandSide::Right, &right_joints, 1_000_000, &cfg).pose;
            assert_vec3_near(gr.position, mirror_yz_vec3(gl.position), 1e-5);
            assert_quat_same_rotation(gr.orientation, mirror_yz_quat(gl.orientation), 1e-5);

            let al = synth
                .compute_aim_pose(HandSide::Left, &left_joints, None, 1_000_000, &cfg)
                .pose;
            let ar = synth
                .compute_aim_pose(HandSide::Right, &right_joints, None, 1_000_000, &cfg)
                .pose;
            assert_vec3_near(ar.position, mirror_yz_vec3(al.position), 1e-5);
            assert_quat_same_rotation(ar.orientation, mirror_yz_quat(al.orientation), 1e-5);
        }

        let mut synth = PoseSynthesizer::new();
        let pl = synth.compute_palm_pose(HandSide::Left, &left_joints, 1_000_000).pose;
        let pr = synth.compute_palm_pose(HandSide::Right, &right_joints, 1_000_000).pose;
        assert_vec3_near(pr.position, mirror_yz_vec3(pl.position), 1e-5);
        assert_quat_same_rotation(pr.orientation, mirror_yz_quat(pl.orientation), 1e-5);
    }

    #[test]
    fn palm_anchor_per_component_fallback_and_clear_hand() {
        let mut synth = PoseSynthesizer::new();
        let cfg = PoseOffsetConfig::default();

        // 1. Never-seen invalid gj[0] -> all flags false, IDENTITY, None vel
        let invalid_joints = make_joints_with_palm(HandJointSample {
            position_valid: false,
            orientation_valid: false,
            position_tracked: true,    // malformed runtime: tracked without valid
            orientation_tracked: true, // must be clamped to false!
            ..HandJointSample::default()
        });
        let s0 = synth.compute_grip_pose(HandSide::Left, &invalid_joints, 1_000_000, &cfg);
        assert!(!s0.position_valid && !s0.orientation_valid);
        assert!(!s0.position_tracked && !s0.orientation_tracked);
        assert_eq!(s0.pose, Pose::IDENTITY);
        assert!(s0.linear_velocity.is_none() && s0.angular_velocity.is_none());

        // 2. Live valid sample caches both components
        let p1 = Pose {
            orientation: Quat::from_rotation_z(FRAC_PI_2),
            position: Vec3::new(-0.2, 1.0, -0.3),
        };
        let mut live1 = valid_palm(p1, 0.015);
        live1.linear_velocity = Some(Vec3::new(0.5, 0.0, 0.0));
        live1.angular_velocity = Some(Vec3::new(0.0, 1.0, 0.0));
        let j1 = make_joints_with_palm(live1);
        let s1 = synth.compute_grip_pose(HandSide::Left, &j1, 10_000_000, &cfg);
        assert!(s1.position_valid && s1.orientation_valid);
        assert!(s1.position_tracked && s1.orientation_tracked);
        assert!(s1.linear_velocity.is_some() && s1.angular_velocity.is_some());

        // Also seed Right hand to ensure clear_hand(Left) doesn't affect Right
        let _ = synth.compute_grip_pose(HandSide::Right, &j1, 10_000_000, &cfg);

        // 3. Lose ONLY position_valid (live ori remains)
        let mut lost_pos = live1;
        lost_pos.position_valid = false;
        lost_pos.position_tracked = true; // must be cleared to false
        lost_pos.pose.position = Vec3::new(99.0, 99.0, 99.0);
        lost_pos.pose.orientation = Quat::from_rotation_z(FRAC_PI_2 + 0.1);
        let s2 = synth.compute_grip_pose(
            HandSide::Left,
            &make_joints_with_palm(lost_pos),
            20_000_000,
            &cfg,
        );
        assert!(s2.position_valid && !s2.position_tracked);
        assert!(s2.orientation_valid && s2.orientation_tracked);
        assert!(s2.linear_velocity.is_none());
        assert_eq!(s2.angular_velocity, Some(Vec3::new(0.0, 1.0, 0.0)));

        // 4. Lose ONLY orientation_valid (live pos remains)
        let mut lost_ori = live1;
        lost_ori.orientation_valid = false;
        lost_ori.orientation_tracked = true;
        lost_ori.pose.orientation = Quat::from_xyzw(f32::NAN, 0.0, 0.0, 0.0);
        let s3 = synth.compute_grip_pose(
            HandSide::Left,
            &make_joints_with_palm(lost_ori),
            30_000_000,
            &cfg,
        );
        assert!(s3.position_valid && s3.position_tracked);
        assert!(s3.orientation_valid && !s3.orientation_tracked);
        assert!(s3.angular_velocity.is_none());
        assert_eq!(s3.linear_velocity, Some(Vec3::new(0.5, 0.0, 0.0)));

        // 5. Lose BOTH -> both valid=true, tracked=false, velocities=None
        let mut lost_both = live1;
        lost_both.position_valid = false;
        lost_both.orientation_valid = false;
        let s4 = synth.compute_grip_pose(
            HandSide::Left,
            &make_joints_with_palm(lost_both),
            40_000_000,
            &cfg,
        );
        assert!(s4.position_valid && s4.orientation_valid);
        assert!(!s4.position_tracked && !s4.orientation_tracked);
        assert!(s4.linear_velocity.is_none() && s4.angular_velocity.is_none());

        // 6. clear_hand(Left) clears Left only; Right stays cached
        synth.clear_hand(HandSide::Left);
        let s5_l = synth.compute_grip_pose(
            HandSide::Left,
            &make_joints_with_palm(lost_both),
            50_000_000,
            &cfg,
        );
        let s5_r = synth.compute_grip_pose(
            HandSide::Right,
            &make_joints_with_palm(lost_both),
            50_000_000,
            &cfg,
        );
        assert!(!s5_l.position_valid && !s5_l.orientation_valid);
        assert!(s5_r.position_valid && s5_r.orientation_valid && !s5_r.position_tracked);
    }

    #[test]
    fn compute_aim_pose_modes_neutral_handshake_and_source_switch_guard() {
        let mut synth = PoseSynthesizer::new();
        let cfg_pref = PoseOffsetConfig::default();

        // Neutral handshake pose: thumb UP (+Y), fingers FORWARD (-Z), palm INWARD
        let left_handshake = Pose {
            orientation: Quat::from_rotation_z(FRAC_PI_2),
            position: Vec3::ZERO,
        };
        let right_handshake = Pose {
            orientation: Quat::from_rotation_z(-FRAC_PI_2),
            position: Vec3::ZERO,
        };
        let j_left = make_joints_with_palm(valid_palm(left_handshake, 0.015));
        let j_right = make_joints_with_palm(valid_palm(right_handshake, 0.015));

        // 1. Neutral handshake grip-derived aim aligns -Z forward, +Y up, +X right on BOTH hands
        for (hand, joints) in [(HandSide::Left, &j_left), (HandSide::Right, &j_right)] {
            let aim = synth.compute_aim_pose(hand, joints, None, 10_000_000, &cfg_pref);
            assert!(aim.position_valid && aim.orientation_valid);
            let fwd = aim.pose.orientation * Vec3::NEG_Z;
            let up = aim.pose.orientation * Vec3::Y;
            let right = aim.pose.orientation * Vec3::X;
            assert_vec3_near(fwd, Vec3::NEG_Z, 1e-5);
            assert_vec3_near(up, Vec3::Y, 1e-5);
            assert_vec3_near(right, Vec3::X, 1e-5);
            // Aim origin is forward (-Z) and slightly up (+Y) from grip origin
            let grip = synth.compute_grip_pose(hand, joints, 10_000_000, &cfg_pref);
            let delta = aim.pose.position - grip.pose.position;
            assert!(delta.z < -0.04, "aim should sit forward of grip: {delta:?}");
            assert!(delta.y > 0.01, "aim should sit slightly above grip: {delta:?}");
        }

        // 2. PreferFbAim uses valid fb_aim_pose and computes finite-diff velocity across frames
        let mut s_fb = PoseSynthesizer::new();
        let fb1 = Pose {
            orientation: Quat::from_xyzw(0.0, 0.0, 0.0, 2.0), // non-unit finite -> normalized
            position: Vec3::new(0.1, 1.2, -0.4),
        };
        let fb2 = Pose {
            orientation: Quat::IDENTITY,
            position: Vec3::new(0.2, 1.2, -0.4), // +0.1 m over 100 ms => +1.0 m/s X
        };
        let r1 = s_fb.compute_aim_pose(HandSide::Left, &j_left, Some(fb1), 100_000_000, &cfg_pref);
        assert!(r1.position_valid && r1.position_tracked && r1.orientation_tracked);
        assert_quat_same_rotation(r1.pose.orientation, Quat::IDENTITY, 1e-6);
        assert!(r1.linear_velocity.is_none());

        let r2 = s_fb.compute_aim_pose(HandSide::Left, &j_left, Some(fb2), 200_000_000, &cfg_pref);
        assert_vec3_near(r2.linear_velocity.expect("fb aim vel"), Vec3::new(1.0, 0.0, 0.0), 1e-4);

        // 3. Source switch from Some(fb) -> None at dt=11ms MUST NOT emit a huge velocity spike
        let r3 = s_fb.compute_aim_pose(HandSide::Left, &j_left, None, 211_000_000, &cfg_pref);
        assert!(
            r3.linear_velocity.is_none() && r3.angular_velocity.is_none(),
            "source switch must reset aim finite-diff cache, got lin={:?} ang={:?}",
            r3.linear_velocity,
            r3.angular_velocity
        );

        // 4. Invalid fb_aim_pose (NaN / zero quat) falls back to grip-derived aim
        let bad_fb = Pose {
            orientation: Quat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            position: Vec3::new(0.5, 0.5, 0.5),
        };
        let fallback_out =
            s_fb.compute_aim_pose(HandSide::Left, &j_left, Some(bad_fb), 250_000_000, &cfg_pref);
        let expected_grip_aim =
            s_fb.compute_aim_pose(HandSide::Left, &j_left, None, 260_000_000, &cfg_pref);
        assert_vec3_near(fallback_out.pose.position, expected_grip_aim.pose.position, 1e-6);

        // 5. GripOffsetOnly ignores valid Some(fb_aim_pose)
        let cfg_only = PoseOffsetConfig {
            aim_mode: AimPoseMode::GripOffsetOnly,
            ..PoseOffsetConfig::default()
        };
        let only_out =
            s_fb.compute_aim_pose(HandSide::Left, &j_left, Some(fb2), 300_000_000, &cfg_only);
        assert_vec3_near(only_out.pose.position, expected_grip_aim.pose.position, 1e-6);
    }

    #[test]
    fn compute_palm_pose_neutral_handshake_and_radius_shift() {
        let mut synth = PoseSynthesizer::new();

        // Neutral handshake: Left Q_palm = Rz(+90°), Right Q_palm = Rz(-90°)
        let l_palm = valid_palm(
            Pose {
                orientation: Quat::from_rotation_z(FRAC_PI_2),
                position: Vec3::new(-0.2, 1.0, -0.3),
            },
            0.015,
        );
        let r_palm = valid_palm(
            Pose {
                orientation: Quat::from_rotation_z(-FRAC_PI_2),
                position: Vec3::new(0.2, 1.0, -0.3),
            },
            0.015,
        );
        let jl = make_joints_with_palm(l_palm);
        let jr = make_joints_with_palm(r_palm);

        let pl = synth.compute_palm_pose(HandSide::Left, &jl, 10_000_000);
        let pr = synth.compute_palm_pose(HandSide::Right, &jr, 10_000_000);

        // Both produce world-aligned +X=(1,0,0), +Y=(0,1,0), -Z=(0,0,-1) (IDENTITY orientation)
        assert_quat_same_rotation(pl.pose.orientation, Quat::IDENTITY, 1e-6);
        assert_quat_same_rotation(pr.pose.orientation, Quat::IDENTITY, 1e-6);

        // Left palm surface shifted inward (+X = +0.015); Right shifted inward (-X = -0.015)
        assert_vec3_near(pl.pose.position, Vec3::new(-0.185, 1.0, -0.3), 1e-6);
        assert_vec3_near(pr.pose.position, Vec3::new(0.185, 1.0, -0.3), 1e-6);

        // Zero, negative, or NaN radius -> zero shift
        for bad_r in [0.0, -0.02, f32::NAN] {
            let p = valid_palm(Pose::IDENTITY, bad_r);
            let out = synth.compute_palm_pose(HandSide::Left, &make_joints_with_palm(p), 20_000_000);
            assert_vec3_near(out.pose.position, Vec3::ZERO, 1e-6);
        }
    }

    #[test]
    fn base_space_velocity_rigid_body_finite_diff_antipodal_and_cache_isolation() {
        let cfg = PoseOffsetConfig::default();

        // 1. Rigid-body velocity matches numerical derivative of P_ctrl(t) and keeps omega in baseSpace
        let q0 = Quat::from_euler(EulerRot::XYZ, 0.3, -0.5, 0.8);
        let p0 = Vec3::new(-0.15, 1.1, -0.35);
        let v_palm = Vec3::new(0.4, -0.2, 0.3);
        let omega = Vec3::new(0.7, -1.2, 0.5); // baseSpace rad/s

        let mut palm0 = valid_palm(Pose { orientation: q0, position: p0 }, 0.015);
        palm0.linear_velocity = Some(v_palm);
        palm0.angular_velocity = Some(omega);
        let mut s_rb = PoseSynthesizer::new();
        let g0 = s_rb.compute_grip_pose(HandSide::Left, &make_joints_with_palm(palm0), 10_000_000, &cfg);
        assert_vec3_near(g0.angular_velocity.expect("rb omega"), omega, 1e-6);

        // Centered numerical derivative of grip position
        let eps = 1e-4_f32;
        let step_pose = |dt: f32| {
            let dq = Quat::from_scaled_axis(omega * dt);
            let sample = valid_palm(
                Pose {
                    orientation: normalize_quat_or_identity(dq * q0),
                    position: p0 + v_palm * dt,
                },
                0.015,
            );
            PoseSynthesizer::new()
                .compute_grip_pose(HandSide::Left, &make_joints_with_palm(sample), 10_000_000, &cfg)
                .pose
                .position
        };
        let num_v = (step_pose(eps) - step_pose(-eps)) / (2.0 * eps);
        assert_vec3_near(g0.linear_velocity.expect("rb lin"), num_v, 1e-3);

        // 2. Finite-difference recovery with large Android-scale timestamps & antipodal -q1
        let dt_ns = 10_000_000_i64; // 10 ms
        let dt_s = 0.01_f32;
        let t0 = 1_000_000_000_000_000_i64; // ~11.5 days uptime in ns
        let t1 = t0 + dt_ns;
        let q1 = normalize_quat_or_identity(Quat::from_scaled_axis(omega * dt_s) * q0);
        let q1_antipodal = -q1; // opposite hemisphere, exact same 3D rotation!

        let j_f0 = make_joints_with_palm(valid_palm(Pose { orientation: q0, position: p0 }, 0.015));
        let j_f1_anti = make_joints_with_palm(valid_palm(
            Pose {
                orientation: q1_antipodal,
                position: p0 + v_palm * dt_s,
            },
            0.015,
        ));

        // Cache isolation: call grip, aim, and palm at the SAME timestamps on frame 0 and frame 1!
        let mut s_iso = PoseSynthesizer::new();
        let g_f0 = s_iso.compute_grip_pose(HandSide::Left, &j_f0, t0, &cfg);
        let a_f0 = s_iso.compute_aim_pose(HandSide::Left, &j_f0, None, t0, &cfg);
        let p_f0 = s_iso.compute_palm_pose(HandSide::Left, &j_f0, t0);
        assert!(g_f0.linear_velocity.is_none() && a_f0.linear_velocity.is_none() && p_f0.linear_velocity.is_none());

        let g_f1 = s_iso.compute_grip_pose(HandSide::Left, &j_f1_anti, t1, &cfg);
        let a_f1 = s_iso.compute_aim_pose(HandSide::Left, &j_f1_anti, None, t1, &cfg);
        let p_f1 = s_iso.compute_palm_pose(HandSide::Left, &j_f1_anti, t1);

        // All three caches must recover exact baseSpace angular velocity (no 359° explosion!)
        assert_vec3_near(g_f1.angular_velocity.expect("grip fd omega"), omega, 1e-3);
        assert_vec3_near(a_f1.angular_velocity.expect("aim fd omega"), omega, 1e-3);
        assert_vec3_near(p_f1.angular_velocity.expect("palm fd omega"), omega, 1e-3);

        // And exact finite-difference linear velocities
        assert_vec3_near(
            g_f1.linear_velocity.expect("grip fd lin"),
            (g_f1.pose.position - g_f0.pose.position) / dt_s,
            1e-4,
        );
        assert_vec3_near(
            a_f1.linear_velocity.expect("aim fd lin"),
            (a_f1.pose.position - a_f0.pose.position) / dt_s,
            1e-4,
        );
        assert_vec3_near(
            p_f1.linear_velocity.expect("palm fd lin"),
            (p_f1.pose.position - p_f0.pose.position) / dt_s,
            1e-4,
        );

        // 3. dt validation: dt == 0, dt < 0, dt > MAX_VELOCITY_DT_NS -> None
        let g_same = s_iso.compute_grip_pose(HandSide::Left, &j_f1_anti, t1, &cfg);
        assert!(g_same.linear_velocity.is_none() && g_same.angular_velocity.is_none());
        let g_back = s_iso.compute_grip_pose(HandSide::Left, &j_f1_anti, t1 - 5_000_000, &cfg);
        assert!(g_back.linear_velocity.is_none() && g_back.angular_velocity.is_none());
        let g_stale =
            s_iso.compute_grip_pose(HandSide::Left, &j_f1_anti, t1 + MAX_VELOCITY_DT_NS + 1, &cfg);
        assert!(g_stale.linear_velocity.is_none() && g_stale.angular_velocity.is_none());

        // 4. Non-finite runtime velocity is rejected cleanly
        let mut nan_vel_palm = valid_palm(Pose { orientation: q0, position: p0 }, 0.015);
        nan_vel_palm.linear_velocity = Some(Vec3::NAN);
        nan_vel_palm.angular_velocity = Some(Vec3::INFINITY);
        let g_nan_vel = PoseSynthesizer::new().compute_grip_pose(
            HandSide::Left,
            &make_joints_with_palm(nan_vel_palm),
            10_000_000,
            &cfg,
        );
        assert!(g_nan_vel.linear_velocity.is_none());
        assert!(g_nan_vel.angular_velocity.is_none());

        // 5. SynthesizedPose::with_local_offset preserves rigid-body velocity identity
        let action_offset = Pose {
            orientation: Quat::from_euler(EulerRot::XYZ, 0.2, -0.4, 0.6),
            position: Vec3::new(0.03, -0.05, 0.08),
        };
        let shifted = g0.with_local_offset(action_offset);
        let expected_v = g0.linear_velocity.unwrap()
            + g0.angular_velocity
                .unwrap()
                .cross(g0.pose.orientation * action_offset.position);
        assert_vec3_near(shifted.linear_velocity.expect("shifted lin"), expected_v, 1e-5);
        assert_vec3_near(
            shifted.angular_velocity.expect("shifted ang"),
            g0.angular_velocity.unwrap(),
            1e-6,
        );
    }

    #[test]
    fn multi_base_space_isolation_and_historical_query_preservation() {
        let cfg = PoseOffsetConfig::default();
        let mut synth = PoseSynthesizer::new();

        const SPACE_LOCAL: u64 = 100;
        const SPACE_STAGE: u64 = 200;

        // Frame 0 at t=10ms: LOCAL at (0, 1, 0), STAGE at (5, 0, -3)
        let t0 = 10_000_000_i64;
        let j_local_0 = make_joints_with_palm(valid_palm(
            Pose {
                orientation: Quat::IDENTITY,
                position: Vec3::new(0.0, 1.0, 0.0),
            },
            0.015,
        ));
        let j_stage_0 = make_joints_with_palm(valid_palm(
            Pose {
                orientation: Quat::IDENTITY,
                position: Vec3::new(5.0, 0.0, -3.0),
            },
            0.015,
        ));
        let g_l0 = synth.compute_grip_pose_in_space(HandSide::Left, SPACE_LOCAL, &j_local_0, t0, &cfg);
        let g_s0 = synth.compute_grip_pose_in_space(HandSide::Left, SPACE_STAGE, &j_stage_0, t0, &cfg);
        assert!(g_l0.linear_velocity.is_none());
        assert!(g_s0.linear_velocity.is_none());

        // Frame 1 at t=20ms (dt=10ms): LOCAL moved +0.01m X (+1 m/s), STAGE moved +0.02m Y (+2 m/s).
        // Alternating spaces must NOT produce cross-space 500 m/s velocity spikes!
        let t1 = 20_000_000_i64;
        let j_local_1 = make_joints_with_palm(valid_palm(
            Pose {
                orientation: Quat::IDENTITY,
                position: Vec3::new(0.01, 1.0, 0.0),
            },
            0.015,
        ));
        let j_stage_1 = make_joints_with_palm(valid_palm(
            Pose {
                orientation: Quat::IDENTITY,
                position: Vec3::new(5.0, 0.02, -3.0),
            },
            0.015,
        ));
        let g_l1 = synth.compute_grip_pose_in_space(HandSide::Left, SPACE_LOCAL, &j_local_1, t1, &cfg);
        let g_s1 = synth.compute_grip_pose_in_space(HandSide::Left, SPACE_STAGE, &j_stage_1, t1, &cfg);
        assert_vec3_near(g_l1.linear_velocity.expect("local vel"), Vec3::new(1.0, 0.0, 0.0), 1e-4);
        assert_vec3_near(g_s1.linear_velocity.expect("stage vel"), Vec3::new(0.0, 2.0, 0.0), 1e-4);

        // Historical reprojection query at t1 - 5ms returns None WITHOUT overwriting t1 anchor!
        let j_hist = make_joints_with_palm(valid_palm(
            Pose {
                orientation: Quat::IDENTITY,
                position: Vec3::new(99.0, 99.0, 99.0),
            },
            0.015,
        ));
        let g_hist = synth.compute_grip_pose_in_space(
            HandSide::Left,
            SPACE_LOCAL,
            &j_hist,
            t1 - 5_000_000,
            &cfg,
        );
        assert!(g_hist.linear_velocity.is_none());

        // Frame 2 at t=30ms: LOCAL moved another +0.01m X -> STILL computes exact +1.0 m/s!
        let t2 = 30_000_000_i64;
        let j_local_2 = make_joints_with_palm(valid_palm(
            Pose {
                orientation: Quat::IDENTITY,
                position: Vec3::new(0.02, 1.0, 0.0),
            },
            0.015,
        ));
        let g_l2 = synth.compute_grip_pose_in_space(HandSide::Left, SPACE_LOCAL, &j_local_2, t2, &cfg);
        assert_vec3_near(g_l2.linear_velocity.expect("local vel after hist"), Vec3::new(1.0, 0.0, 0.0), 1e-4);

        // Fallback isolation: clearing SPACE_LOCAL does not lose SPACE_STAGE fallback
        synth.clear_base_space(SPACE_LOCAL);
        let invalid = make_joints_with_palm(HandJointSample::default());
        let l_after = synth.compute_grip_pose_in_space(HandSide::Left, SPACE_LOCAL, &invalid, t2 + 10_000_000, &cfg);
        let s_after = synth.compute_grip_pose_in_space(HandSide::Left, SPACE_STAGE, &invalid, t2 + 10_000_000, &cfg);
        assert!(!l_after.position_valid);
        assert!(s_after.position_valid && !s_after.position_tracked);
    }
}
