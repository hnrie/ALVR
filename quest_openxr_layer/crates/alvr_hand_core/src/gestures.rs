//! Faithful port of ALVR's `HandGestureManager` (`alvr/server_core/src/hand_gestures.rs`)
//! and `HysteresisThreshold` (`alvr/server_core/src/input_mapping.rs`), using a monotonic
//! millisecond clock and safe quaternion sanitization.

use crate::config::HandTrackingInteractionConfig;
use crate::pose::{is_valid_quat, normalize_quat_or_identity};
use crate::virtual_controller::VirtualHandState;
use crate::{HandSide, LayerConfig, Pose, HAND_JOINT_COUNT};
use glam::{Quat, Vec2, Vec3, Vec4};

// Verbatim ALVR anatomical radii (meters) — alvr/server_core/src/hand_gestures.rs:113-118
pub const THUMB_RADIUS: f32 = 0.0075;
pub const INDEX_RADIUS: f32 = 0.0065;
pub const MIDDLE_RADIUS: f32 = 0.0065;
pub const RING_RADIUS: f32 = 0.006;
pub const LITTLE_RADIUS: f32 = 0.005;
pub const PALM_DEPTH: f32 = 0.005;

/// 1:1 with `XrHandJointEXT` (`0..26`).
#[repr(usize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HandJoint {
    Palm = 0,
    Wrist = 1,
    ThumbMetacarpal = 2,
    ThumbProximal = 3,
    ThumbDistal = 4,
    ThumbTip = 5,
    IndexMetacarpal = 6,
    IndexProximal = 7,
    IndexIntermediate = 8,
    IndexDistal = 9,
    IndexTip = 10,
    MiddleMetacarpal = 11,
    MiddleProximal = 12,
    MiddleIntermediate = 13,
    MiddleDistal = 14,
    MiddleTip = 15,
    RingMetacarpal = 16,
    RingProximal = 17,
    RingIntermediate = 18,
    RingDistal = 19,
    RingTip = 20,
    LittleMetacarpal = 21,
    LittleProximal = 22,
    LittleIntermediate = 23,
    LittleDistal = 24,
    LittleTip = 25,
}

impl HandJoint {
    /// Returns the `0..26` array index matching `XrHandJointEXT`.
    pub const fn idx(self) -> usize {
        self as usize
    }
}

/// Verbatim ALVR gesture identifiers (`alvr/server_core/src/hand_gestures.rs:71-87`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HandGestureId {
    ThumbIndexPinch,
    ThumbMiddlePinch,
    ThumbRingPinch,
    ThumbLittlePinch,
    ThumbCurl,
    IndexCurl,
    MiddleCurl,
    RingCurl,
    LittleCurl,
    GripCurl,
    JoystickX,
    JoystickY,
}

/// Single evaluated ALVR hand gesture snapshot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HandGesture {
    pub id: HandGestureId,
    pub active: bool,
    pub clicked: bool,
    pub touching: bool,
    pub value: f32,
}

/// Verbatim ALVR hysteresis state (`alvr/server_core/src/input_mapping.rs:667-687`)
/// with boundary saturation and non-finite guards.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HysteresisState {
    active: bool,
}

impl HysteresisState {
    pub const fn new() -> Self {
        Self { active: false }
    }

    pub const fn is_active(self) -> bool {
        self.active
    }

    pub fn reset(&mut self) {
        self.active = false;
    }

    /// Updates state from scalar `value` using `threshold ± hysteresis`.
    ///
    /// Standard band (`low > 0.0 && high < 1.0`): exact ALVR strict `< low` to release
    /// and strict `> high` to press, holding inside `[low, high]`.
    /// Boundary saturation prevents permanently stuck/unreachable booleans at `0.0` or `1.0`.
    pub fn update(&mut self, value: f32, threshold: f32, hysteresis: f32) -> bool {
        if !value.is_finite() || !threshold.is_finite() {
            return self.active;
        }
        let dev = if hysteresis.is_finite() {
            hysteresis.max(0.0)
        } else {
            0.0
        };
        let low = threshold - dev;
        let high = threshold + dev;

        if self.active {
            let release = if low > 0.0 {
                value < low
            } else {
                value <= 0.0
            };
            if release {
                self.active = false;
            }
        } else {
            let press = if high < 1.0 {
                value > high
            } else if threshold <= 1.0 {
                value >= 1.0
            } else {
                value > high
            };
            if press {
                self.active = true;
            }
        }
        self.active
    }
}

/// Safe unit-quaternion normalization delegating to the shared [`normalize_quat_or_identity`] helper.
pub fn safe_normalize_quat(q: Quat) -> Quat {
    normalize_quat_or_identity(q)
}

/// Shortest-arc nlerp pose interpolation matching ALVR's `lerp_pose`, guarded against degenerate inputs.
pub fn lerp_pose(a: Pose, b: Pose, fac: f32) -> Pose {
    let t = if fac.is_finite() { fac } else { 0.5 };
    let qa = normalize_quat_or_identity(a.orientation);
    let mut qb = normalize_quat_or_identity(b.orientation);
    if qa.dot(qb) < 0.0 {
        qb = -qb;
    }
    let blended = Vec4::from(qa) * (1.0 - t) + Vec4::from(qb) * t;
    let orientation = normalize_quat_or_identity(Quat::from_vec4(blended));
    let position = if a.position.is_finite() && b.position.is_finite() {
        let p = a.position.lerp(b.position, t);
        if p.is_finite() {
            p
        } else {
            Vec3::ZERO
        }
    } else {
        Vec3::ZERO
    };
    Pose {
        orientation,
        position,
    }
}

/// Strict `<` surface distance check matching `alvr/server_core/src/hand_gestures.rs:450-460`.
pub fn test_gesture_dist(a: Pose, r1: f32, b: Pose, r2: f32, activation_dist: f32) -> bool {
    if !a.position.is_finite()
        || !b.position.is_finite()
        || !r1.is_finite()
        || !r2.is_finite()
        || !activation_dist.is_finite()
    {
        return false;
    }
    let dist = a.position.distance(b.position);
    let thresh = activation_dist + r1 + r2;
    dist.is_finite() && thresh.is_finite() && dist < thresh
}

/// Verbatim ALVR hover scalar matching `alvr/server_core/src/hand_gestures.rs:462-477`
/// (with `(max_dist + r1 + r2)` denominator and non-finite / zero-denominator guards).
pub fn get_gesture_hover(
    a: Pose,
    r1: f32,
    b: Pose,
    r2: f32,
    min_dist: f32,
    max_dist: f32,
) -> f32 {
    if !a.position.is_finite()
        || !b.position.is_finite()
        || !r1.is_finite()
        || !r2.is_finite()
        || !min_dist.is_finite()
        || !max_dist.is_finite()
    {
        return 0.0;
    }
    let dist = a.position.distance(b.position);
    let denom = max_dist + r1 + r2;
    if !dist.is_finite() || !denom.is_finite() {
        return 0.0;
    }
    if denom <= 1e-9 {
        return if dist <= min_dist + r1 + r2 { 1.0 } else { 0.0 };
    }
    let raw = 1.0 - (dist - min_dist - r1 - r2) / denom;
    if raw.is_finite() {
        raw.clamp(0.0, 1.0)
    } else {
        0.0
    }
}

/// Verbatim ALVR joystick axis projection matching `alvr/server_core/src/hand_gestures.rs:480-498`,
/// guarded against zero `joy_radius` and degenerate basis vectors.
pub fn get_joystick_values(
    center: Pose,
    anchor: Pose,
    joy_radius: f32,
    hori_vec: Vec3,
    vert_vec: Vec3,
    offset_hori: f32,
    offset_vert: f32,
) -> Vec2 {
    if !center.position.is_finite()
        || !anchor.position.is_finite()
        || !joy_radius.is_finite()
        || joy_radius <= 1e-9
        || !offset_hori.is_finite()
        || !offset_vert.is_finite()
    {
        return Vec2::ZERO;
    }

    let delta = anchor.position - center.position;
    let hori_len = hori_vec.length();
    let vert_len = vert_vec.length();

    let x = if hori_vec.is_finite() && hori_len > 1e-9 && hori_len.is_finite() {
        let v = (delta.dot(hori_vec) / hori_len + offset_hori) / joy_radius;
        if v.is_finite() {
            v.clamp(-1.0, 1.0)
        } else {
            0.0
        }
    } else {
        0.0
    };

    let y = if vert_vec.is_finite() && vert_len > 1e-9 && vert_len.is_finite() {
        let v = (delta.dot(vert_vec) / vert_len + offset_vert) / joy_radius;
        if v.is_finite() {
            v.clamp(-1.0, 1.0)
        } else {
            0.0
        }
    } else {
        0.0
    };

    Vec2::new(x, y)
}

#[derive(Debug, Clone, Copy, Default)]
struct GestureAction {
    last_activated: Option<u64>,
    last_deactivated: Option<u64>,
    entering: bool,
    entering_since: u64,
    exiting: bool,
    exiting_since: u64,
    active: bool,
}

#[derive(Debug, Clone, Default)]
struct PerHandState {
    /// Indexed: 0 = ThumbIndex, 1 = ThumbMiddle, 2 = ThumbRing, 3 = ThumbLittle
    pinches: [GestureAction; 4],
    trigger_hysteresis: HysteresisState,
    squeeze_hysteresis: HysteresisState,
}

/// Stateful per-hand ALVR gesture evaluation and controller input projection engine.
#[derive(Debug, Clone, Default)]
pub struct HandGestureEngine {
    left: PerHandState,
    right: PerHandState,
}

impl HandGestureEngine {
    /// Creates a new [`HandGestureEngine`] with fresh debounce and hysteresis states.
    pub fn new() -> Self {
        Self::default()
    }

    fn hand_state_mut(&mut self, hand: HandSide) -> &mut PerHandState {
        match hand {
            HandSide::Left => &mut self.left,
            HandSide::Right => &mut self.right,
        }
    }

    fn hand_state(&self, hand: HandSide) -> &PerHandState {
        match hand {
            HandSide::Left => &self.left,
            HandSide::Right => &self.right,
        }
    }

    /// Evaluates the 8 ALVR gestures (`[ThumbIndexPinch, ThumbMiddlePinch, ThumbRingPinch,
    /// ThumbLittlePinch, GripCurl, ThumbCurl, JoystickX, JoystickY]`) verbatim for one hand,
    /// assuming all 26 joints are position- and orientation-valid.
    pub fn evaluate_gestures(
        &mut self,
        hand: HandSide,
        joints: &[Pose; HAND_JOINT_COUNT],
        now_ms: u64,
        cfg: &HandTrackingInteractionConfig,
    ) -> [HandGesture; 8] {
        let samples = std::array::from_fn(|i| crate::pose::HandJointSample {
            pose: joints[i],
            position_valid: true,
            orientation_valid: true,
            position_tracked: true,
            orientation_tracked: true,
            radius: 0.0,
            linear_velocity: None,
            angular_velocity: None,
        });
        self.evaluate_gestures_samples(hand, &samples, now_ms, cfg)
    }

    /// Evaluates the 8 ALVR gestures honoring per-joint `position_valid` / `orientation_valid`
    /// flags from `xrLocateHandJointsEXT`.
    ///
    /// Occluded or untracked joints with `position_valid == false` (which OpenXR runtimes often
    /// zero-fill to `(0, 0, 0)`) are treated as out-of-range / `0.0` so they never false-trigger
    /// pinches, curls, or joystick deflections.
    pub fn evaluate_gestures_samples(
        &mut self,
        hand: HandSide,
        samples: &[crate::pose::HandJointSample; HAND_JOINT_COUNT],
        now_ms: u64,
        cfg: &HandTrackingInteractionConfig,
    ) -> [HandGesture; 8] {
        let pos_ok = |j: HandJoint| -> bool {
            let s = &samples[j.idx()];
            s.position_valid && s.pose.position.is_finite()
        };
        let ori_ok = |j: HandJoint| -> bool {
            let s = &samples[j.idx()];
            s.orientation_valid && is_valid_quat(s.pose.orientation)
        };
        let pose_of = |j: HandJoint| -> Pose { samples[j.idx()].pose };

        let pinch_min = cfg.pinch_touch_distance * 0.01;
        let pinch_max = cfg.pinch_trigger_distance * 0.01;
        let curl_min = cfg.curl_touch_distance * 0.01;
        let curl_max = cfg.curl_trigger_distance * 0.01;

        let palm = pose_of(HandJoint::Palm);
        let thumb_tip = pose_of(HandJoint::ThumbTip);
        let index_metacarpal = pose_of(HandJoint::IndexMetacarpal);
        let index_proximal = pose_of(HandJoint::IndexProximal);
        let index_intermediate = pose_of(HandJoint::IndexIntermediate);
        let index_distal = pose_of(HandJoint::IndexDistal);
        let index_tip = pose_of(HandJoint::IndexTip);
        let middle_metacarpal = pose_of(HandJoint::MiddleMetacarpal);
        let middle_proximal = pose_of(HandJoint::MiddleProximal);
        let middle_tip = pose_of(HandJoint::MiddleTip);
        let ring_metacarpal = pose_of(HandJoint::RingMetacarpal);
        let ring_proximal = pose_of(HandJoint::RingProximal);
        let ring_tip = pose_of(HandJoint::RingTip);
        let little_metacarpal = pose_of(HandJoint::LittleMetacarpal);
        let little_proximal = pose_of(HandJoint::LittleProximal);
        let little_tip = pose_of(HandJoint::LittleTip);

        let thumb_pos_ok = pos_ok(HandJoint::ThumbTip);

        // 1. Four debounced pinches (guarded by per-joint position_valid)
        let mut eval_pinch = |id: HandGestureId,
                              pinch_idx: usize,
                              finger_joint: HandJoint,
                              finger_pose: Pose,
                              finger_radius: f32|
         -> HandGesture {
            let pair_valid = thumb_pos_ok && pos_ok(finger_joint);
            let in_range = pair_valid
                && test_gesture_dist(
                    thumb_tip,
                    THUMB_RADIUS,
                    finger_pose,
                    finger_radius,
                    pinch_max,
                );
            let active = self.update_pinch_debounce(
                hand,
                pinch_idx,
                in_range,
                cfg.repeat_delay,
                cfg.activation_delay,
                cfg.deactivation_delay,
                now_ms,
            );
            let clicked = pair_valid
                && test_gesture_dist(
                    thumb_tip,
                    THUMB_RADIUS,
                    finger_pose,
                    finger_radius,
                    pinch_min,
                );
            let value = if pair_valid {
                get_gesture_hover(
                    thumb_tip,
                    THUMB_RADIUS,
                    finger_pose,
                    finger_radius,
                    pinch_min,
                    pinch_max,
                )
            } else {
                0.0
            };
            HandGesture {
                id,
                active,
                clicked,
                touching: in_range,
                value,
            }
        };

        let pinch_idx = eval_pinch(
            HandGestureId::ThumbIndexPinch,
            0,
            HandJoint::IndexTip,
            index_tip,
            INDEX_RADIUS,
        );
        let pinch_mid = eval_pinch(
            HandGestureId::ThumbMiddlePinch,
            1,
            HandJoint::MiddleTip,
            middle_tip,
            MIDDLE_RADIUS,
        );
        let pinch_ring = eval_pinch(
            HandGestureId::ThumbRingPinch,
            2,
            HandJoint::RingTip,
            ring_tip,
            RING_RADIUS,
        );
        let pinch_little = eval_pinch(
            HandGestureId::ThumbLittlePinch,
            3,
            HandJoint::LittleTip,
            little_tip,
            LITTLE_RADIUS,
        );

        // 2. Finger curls & GripCurl (guarded by per-joint position_valid)
        let thumb_curl_valid = pos_ok(HandJoint::Palm) && thumb_pos_ok;
        let thumb_curl = if thumb_curl_valid {
            get_gesture_hover(palm, PALM_DEPTH, thumb_tip, THUMB_RADIUS, curl_min, curl_max)
        } else {
            0.0
        };
        let index_curl = if pos_ok(HandJoint::IndexMetacarpal)
            && pos_ok(HandJoint::IndexProximal)
            && pos_ok(HandJoint::IndexTip)
        {
            get_gesture_hover(
                lerp_pose(index_metacarpal, index_proximal, 0.5),
                PALM_DEPTH,
                index_tip,
                INDEX_RADIUS,
                curl_min,
                curl_max,
            )
        } else {
            0.0
        };
        let middle_curl = if pos_ok(HandJoint::MiddleMetacarpal)
            && pos_ok(HandJoint::MiddleProximal)
            && pos_ok(HandJoint::MiddleTip)
        {
            get_gesture_hover(
                lerp_pose(middle_metacarpal, middle_proximal, 0.5),
                PALM_DEPTH,
                middle_tip,
                MIDDLE_RADIUS,
                curl_min,
                curl_max,
            )
        } else {
            0.0
        };
        let ring_curl = if pos_ok(HandJoint::RingMetacarpal)
            && pos_ok(HandJoint::RingProximal)
            && pos_ok(HandJoint::RingTip)
        {
            get_gesture_hover(
                lerp_pose(ring_metacarpal, ring_proximal, 0.5),
                PALM_DEPTH,
                ring_tip,
                RING_RADIUS,
                curl_min,
                curl_max,
            )
        } else {
            0.0
        };
        let little_curl = if pos_ok(HandJoint::LittleMetacarpal)
            && pos_ok(HandJoint::LittleProximal)
            && pos_ok(HandJoint::LittleTip)
        {
            get_gesture_hover(
                lerp_pose(little_metacarpal, little_proximal, 0.5),
                PALM_DEPTH,
                little_tip,
                LITTLE_RADIUS,
                curl_min,
                curl_max,
            )
        } else {
            0.0
        };

        let grip_curl = (middle_curl + ring_curl + little_curl) / 3.0;
        let grip = HandGesture {
            id: HandGestureId::GripCurl,
            active: grip_curl > 0.0,
            clicked: grip_curl == 1.0,
            touching: grip_curl > 0.0,
            value: grip_curl,
        };

        let thumb = HandGesture {
            id: HandGestureId::ThumbCurl,
            active: thumb_curl_valid && thumb_curl >= 0.0,
            touching: thumb_curl_valid && thumb_curl >= 0.0,
            clicked: thumb_curl_valid && thumb_curl >= 0.5,
            value: thumb_curl,
        };

        // 3. Joystick (4-gate contact, handedness mirroring, square per-axis deadzone)
        let joystick_range = cfg.joystick_range * 0.01;
        let joy_range_valid = joystick_range.is_finite() && joystick_range > 1e-9;
        let joy_joints_valid = thumb_pos_ok
            && pos_ok(HandJoint::IndexIntermediate)
            && pos_ok(HandJoint::IndexDistal)
            && ori_ok(HandJoint::IndexIntermediate)
            && ori_ok(HandJoint::IndexDistal);

        let joystick_center = lerp_pose(index_intermediate, index_distal, 0.5);
        let idx_inter_ori = normalize_quat_or_identity(index_intermediate.orientation);

        // Exact ALVR basis frames:
        // - `joystick_up` uses `joystick_center.orientation`
        // - `joystick_horizontal_vec` & `joystick_vertical_vec` use `gj[8].orientation`
        let joystick_up = joystick_center.orientation.mul_vec3(match hand {
            HandSide::Left => Vec3::X,
            HandSide::Right => Vec3::NEG_X,
        });
        let joystick_horizontal_vec = idx_inter_ori.mul_vec3(match hand {
            HandSide::Left => Vec3::Y,
            HandSide::Right => Vec3::NEG_Y,
        });
        let joystick_vertical_vec = idx_inter_ori.mul_vec3(Vec3::Z);

        let offset_hori_dir = match hand {
            HandSide::Left => 1.0,
            HandSide::Right => -1.0,
        };
        let offset_hori = if cfg.joystick_offset_horizontal.is_finite() {
            cfg.joystick_offset_horizontal * 0.01 * offset_hori_dir
        } else {
            0.0
        };
        let offset_vert = if cfg.joystick_offset_vertical.is_finite() {
            cfg.joystick_offset_vertical * 0.01
        } else {
            0.0
        };

        let joystick_pos = if joy_range_valid && joy_joints_valid {
            get_joystick_values(
                joystick_center,
                thumb_tip,
                joystick_range,
                joystick_horizontal_vec,
                joystick_vertical_vec,
                offset_hori,
                offset_vert,
            )
        } else {
            Vec2::ZERO
        };

        let dist_gate = joy_range_valid
            && joy_joints_valid
            && joystick_center.position.is_finite()
            && thumb_tip.position.is_finite()
            && joystick_center.position.distance(thumb_tip.position) <= joystick_range * 3.0;

        let up_len = joystick_up.length();
        let up_gate = if joy_range_valid
            && joy_joints_valid
            && joystick_center.position.is_finite()
            && thumb_tip.position.is_finite()
            && up_len > 1e-9
            && up_len.is_finite()
        {
            let proj = (thumb_tip.position - joystick_center.position).dot(joystick_up) / up_len;
            proj.is_finite() && proj <= joystick_range * 2.0
        } else {
            false
        };

        let joystick_contact = index_curl >= 0.75 && grip_curl > 0.5 && dist_gate && up_gate;

        let joystick_deadzone = if cfg.joystick_deadzone.is_finite() {
            (cfg.joystick_deadzone * 0.01).max(0.0)
        } else {
            0.40
        };

        let joy_x = HandGesture {
            id: HandGestureId::JoystickX,
            active: joystick_contact,
            touching: joystick_contact,
            clicked: false,
            value: if joystick_contact && joystick_pos.x.abs() >= joystick_deadzone {
                joystick_pos.x
            } else {
                0.0
            },
        };
        let joy_y = HandGesture {
            id: HandGestureId::JoystickY,
            active: joystick_contact,
            touching: joystick_contact,
            clicked: false,
            value: if joystick_contact && joystick_pos.y.abs() >= joystick_deadzone {
                joystick_pos.y
            } else {
                0.0
            },
        };

        [
            pinch_idx,
            pinch_mid,
            pinch_ring,
            pinch_little,
            grip,
            thumb,
            joy_x,
            joy_y,
        ]
    }

    /// Evaluates one hand's 26-joint skeleton (assuming all joints valid) and projects all ALVR
    /// gesture bindings into a [`VirtualHandState`] snapshot.
    pub fn update_hand(
        &mut self,
        hand: HandSide,
        joints: &[Pose; HAND_JOINT_COUNT],
        now_ms: u64,
        cfg: &LayerConfig,
    ) -> VirtualHandState {
        let samples = std::array::from_fn(|i| crate::pose::HandJointSample {
            pose: joints[i],
            position_valid: true,
            orientation_valid: true,
            position_tracked: true,
            orientation_tracked: true,
            radius: 0.0,
            linear_velocity: None,
            angular_velocity: None,
        });
        self.update_hand_samples(hand, &samples, now_ms, cfg)
    }

    /// Evaluates one hand's 26-joint `HandJointSample` array honoring per-joint validity flags,
    /// and projects all ALVR gesture bindings into a [`VirtualHandState`] snapshot.
    pub fn update_hand_samples(
        &mut self,
        hand: HandSide,
        samples: &[crate::pose::HandJointSample; HAND_JOINT_COUNT],
        now_ms: u64,
        cfg: &LayerConfig,
    ) -> VirtualHandState {
        if !cfg.enabled {
            return self.clear_hand(hand);
        }

        let [pinch_idx, pinch_mid, pinch_ring, pinch_lit, grip, thumb_curl, joy_x, joy_y] =
            self.evaluate_gestures_samples(hand, samples, now_ms, &cfg.gestures);

        let only_touch = cfg.gestures.only_touch;
        let trigger_click = !only_touch && pinch_idx.active && pinch_idx.clicked;

        let state = VirtualHandState {
            tracked: true,
            trigger_value: if !only_touch && pinch_idx.active {
                pinch_idx.value
            } else {
                0.0
            },
            trigger_touch: pinch_idx.active && pinch_idx.touching,
            trigger_click,
            squeeze_value: if !only_touch && grip.active {
                grip.value
            } else {
                0.0
            },
            squeeze_click: !only_touch && grip.active && grip.clicked,
            thumbstick: if !only_touch && joy_x.active {
                Vec2::new(joy_x.value, joy_y.value)
            } else {
                Vec2::ZERO
            },
            thumbstick_touch: joy_x.active && joy_x.touching,
            thumbstick_click: !only_touch && thumb_curl.active && thumb_curl.clicked,
            primary_click: !only_touch && pinch_ring.active && pinch_ring.clicked,
            primary_touch: pinch_ring.active && pinch_ring.touching,
            secondary_click: !only_touch && pinch_mid.active && pinch_mid.clicked,
            secondary_touch: pinch_mid.active && pinch_mid.touching,
            menu_click: match hand {
                HandSide::Left => !only_touch && pinch_lit.active && pinch_lit.clicked,
                HandSide::Right => false,
            },
            select_click: trigger_click,
            thumbrest_touch: false,
        };

        let per_hand = self.hand_state_mut(hand);
        if only_touch {
            per_hand.trigger_hysteresis.reset();
            per_hand.squeeze_hysteresis.reset();
        } else {
            per_hand.trigger_hysteresis.update(
                state.trigger_value,
                cfg.float_to_bool_threshold,
                cfg.float_to_bool_hysteresis,
            );
            per_hand.squeeze_hysteresis.update(
                state.squeeze_value,
                cfg.float_to_bool_threshold,
                cfg.float_to_bool_hysteresis,
            );
        }

        state
    }

    /// Clears all debounce and hysteresis states for `hand` and returns an untracked zero snapshot.
    pub fn clear_hand(&mut self, hand: HandSide) -> VirtualHandState {
        *self.hand_state_mut(hand) = PerHandState::default();
        VirtualHandState::default()
    }

    /// Clears all debounce and hysteresis states for both hands.
    pub fn reset(&mut self) {
        self.left = PerHandState::default();
        self.right = PerHandState::default();
    }

    /// Current trigger float-to-bool hysteresis state for `hand`.
    pub fn trigger_hysteresis_pressed(&self, hand: HandSide) -> bool {
        self.hand_state(hand).trigger_hysteresis.is_active()
    }

    /// Current squeeze float-to-bool hysteresis state for `hand`.
    pub fn squeeze_hysteresis_pressed(&self, hand: HandSide) -> bool {
        self.hand_state(hand).squeeze_hysteresis.is_active()
    }

    #[allow(clippy::too_many_arguments)]
    fn update_pinch_debounce(
        &mut self,
        hand: HandSide,
        pinch_idx: usize,
        in_range: bool,
        repeat_delay: u32,
        in_delay: u32,
        out_delay: u32,
        now_ms: u64,
    ) -> bool {
        let g = &mut self.hand_state_mut(hand).pinches[pinch_idx];

        if in_range {
            g.exiting = false;
        } else {
            g.entering = false;
        }

        // Transitioning from inactive to active (exact ALVR strict `<` <=> `elapsed > delay`)
        if in_range && !g.active {
            let repeat_ok = match g.last_deactivated {
                None => true,
                Some(last_deact) => now_ms.saturating_sub(last_deact) > u64::from(repeat_delay),
            };
            if repeat_ok {
                if g.entering {
                    if now_ms.saturating_sub(g.entering_since) > u64::from(in_delay) {
                        g.last_activated = Some(now_ms);
                        g.entering = false;
                        g.active = true;
                    }
                } else {
                    g.entering = true;
                    g.entering_since = now_ms;
                }
            }
        }

        // Transitioning from active to inactive
        if !in_range && g.active {
            if g.exiting {
                if now_ms.saturating_sub(g.exiting_since) > u64::from(out_delay) {
                    g.last_deactivated = Some(now_ms);
                    g.exiting = false;
                    g.active = false;
                }
            } else {
                g.exiting = true;
                g.exiting_since = now_ms;
            }
        }

        g.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    fn pose_at(x: f32, y: f32, z: f32) -> Pose {
        Pose {
            orientation: Quat::IDENTITY,
            position: Vec3::new(x, y, z),
        }
    }

    /// Creates a neutral open-hand skeleton where all joints sit far apart (no pinches, no curls, no joystick).
    fn open_hand_skeleton() -> [Pose; HAND_JOINT_COUNT] {
        let mut gj = [Pose::IDENTITY; HAND_JOINT_COUNT];
        for (i, slot) in gj.iter_mut().enumerate() {
            // Place every joint 0.5 m apart so no accidental pinch/curl occurs
            slot.position = Vec3::new(i as f32 * 0.5, 0.0, 0.0);
        }
        gj
    }

    #[test]
    fn constants_and_joint_indices_match_alvr_and_openxr() {
        assert_eq!(THUMB_RADIUS, 0.0075);
        assert_eq!(INDEX_RADIUS, 0.0065);
        assert_eq!(MIDDLE_RADIUS, 0.0065);
        assert_eq!(RING_RADIUS, 0.006);
        assert_eq!(LITTLE_RADIUS, 0.005);
        assert_eq!(PALM_DEPTH, 0.005);
        assert_eq!(HAND_JOINT_COUNT, 26);

        let all = [
            HandJoint::Palm,
            HandJoint::Wrist,
            HandJoint::ThumbMetacarpal,
            HandJoint::ThumbProximal,
            HandJoint::ThumbDistal,
            HandJoint::ThumbTip,
            HandJoint::IndexMetacarpal,
            HandJoint::IndexProximal,
            HandJoint::IndexIntermediate,
            HandJoint::IndexDistal,
            HandJoint::IndexTip,
            HandJoint::MiddleMetacarpal,
            HandJoint::MiddleProximal,
            HandJoint::MiddleIntermediate,
            HandJoint::MiddleDistal,
            HandJoint::MiddleTip,
            HandJoint::RingMetacarpal,
            HandJoint::RingProximal,
            HandJoint::RingIntermediate,
            HandJoint::RingDistal,
            HandJoint::RingTip,
            HandJoint::LittleMetacarpal,
            HandJoint::LittleProximal,
            HandJoint::LittleIntermediate,
            HandJoint::LittleDistal,
            HandJoint::LittleTip,
        ];
        for (expected, joint) in all.iter().enumerate() {
            assert_eq!(joint.idx(), expected);
        }
    }

    #[test]
    fn lerp_pose_shortest_arc_and_degenerate_inputs() {
        let q = Quat::from_rotation_y(FRAC_PI_2);
        let a = Pose {
            orientation: q,
            position: Vec3::new(0.0, 0.0, 0.0),
        };
        let b_neg = Pose {
            orientation: -q,
            position: Vec3::new(2.0, 4.0, 6.0),
        };
        // Shortest-arc between q and -q must stay at ±q (not collapse to zero/NaN)
        let mid_same = lerp_pose(a, b_neg, 0.5);
        assert!(mid_same.orientation.is_finite());
        assert!((mid_same.orientation.length() - 1.0).abs() < 1e-6);
        assert!((mid_same.orientation.dot(q).abs() - 1.0).abs() < 1e-6);
        assert_eq!(mid_same.position, Vec3::new(1.0, 2.0, 3.0));

        // 90 deg rotation interpolated at 0.5 -> 45 deg
        let mid_rot = lerp_pose(Pose::IDENTITY, a, 0.5);
        let expected_45 = Quat::from_rotation_y(FRAC_PI_2 * 0.5);
        assert!((mid_rot.orientation.dot(expected_45).abs() - 1.0).abs() < 1e-5);

        // Degenerate inputs never panic or leak NaN
        let bad_cases = [
            (
                Pose {
                    orientation: Quat::from_xyzw(0.0, 0.0, 0.0, 0.0),
                    position: Vec3::ZERO,
                },
                Pose {
                    orientation: Quat::from_xyzw(0.0, 0.0, 0.0, 5.0),
                    position: Vec3::ONE,
                },
                0.5,
            ),
            (
                Pose {
                    orientation: Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0),
                    position: Vec3::new(f32::INFINITY, 0.0, 0.0),
                },
                Pose::IDENTITY,
                f32::NAN,
            ),
        ];
        for (pa, pb, fac) in bad_cases {
            let out = lerp_pose(pa, pb, fac);
            assert!(out.orientation.is_finite());
            assert!((out.orientation.length() - 1.0).abs() < 1e-6);
            assert!(out.position.is_finite());
        }
    }

    #[test]
    fn test_gesture_dist_and_get_gesture_hover_exact_alvr_anchors() {
        let a = pose_at(0.0, 0.0, 0.0);
        // Strict `<` boundary: THUMB_RADIUS + INDEX_RADIUS + 0.0025 = 0.0165
        assert!(!test_gesture_dist(
            a,
            THUMB_RADIUS,
            pose_at(0.0165, 0.0, 0.0),
            INDEX_RADIUS,
            0.0025
        ));
        assert!(test_gesture_dist(
            a,
            THUMB_RADIUS,
            pose_at(0.0165 - 1e-5, 0.0, 0.0),
            INDEX_RADIUS,
            0.0025
        ));

        // ThumbIndexPinch hover anchors (pinch_min=0.0, pinch_max=0.0025, r1+r2=0.014, denom=0.0165)
        let h_1 = get_gesture_hover(
            a,
            THUMB_RADIUS,
            pose_at(0.014, 0.0, 0.0),
            INDEX_RADIUS,
            0.0,
            0.0025,
        );
        let h_mid = get_gesture_hover(
            a,
            THUMB_RADIUS,
            pose_at(0.0165, 0.0, 0.0),
            INDEX_RADIUS,
            0.0,
            0.0025,
        );
        let h_0 = get_gesture_hover(
            a,
            THUMB_RADIUS,
            pose_at(0.0305, 0.0, 0.0),
            INDEX_RADIUS,
            0.0,
            0.0025,
        );
        assert!((h_1 - 1.0).abs() < 1e-6);
        assert!((h_mid - (28.0 / 33.0)).abs() < 1e-5, "got {h_mid}");
        assert!(h_0.abs() < 1e-6);

        // MiddleCurl anchors (curl_min=0.02, curl_max=0.025, r1+r2=0.0115, denom=0.0365)
        assert!(
            (get_gesture_hover(
                a,
                PALM_DEPTH,
                pose_at(0.0315, 0.0, 0.0),
                MIDDLE_RADIUS,
                0.02,
                0.025
            ) - 1.0)
                .abs()
                < 1e-6
        );
        assert!(
            (get_gesture_hover(
                a,
                PALM_DEPTH,
                pose_at(0.04975, 0.0, 0.0),
                MIDDLE_RADIUS,
                0.02,
                0.025
            ) - 0.5)
                .abs()
                < 1e-5
        );
        assert!(
            get_gesture_hover(
                a,
                PALM_DEPTH,
                pose_at(0.0680, 0.0, 0.0),
                MIDDLE_RADIUS,
                0.02,
                0.025
            )
            .abs()
                < 1e-6
        );

        // ThumbCurl anchors (curl_min=0.02, curl_max=0.025, r1+r2=0.0125, denom=0.0375)
        assert!(
            (get_gesture_hover(
                a,
                PALM_DEPTH,
                pose_at(0.0325, 0.0, 0.0),
                THUMB_RADIUS,
                0.02,
                0.025
            ) - 1.0)
                .abs()
                < 1e-6
        );
        assert!(
            (get_gesture_hover(
                a,
                PALM_DEPTH,
                pose_at(0.05125, 0.0, 0.0),
                THUMB_RADIUS,
                0.02,
                0.025
            ) - 0.5)
                .abs()
                < 1e-5
        );
        assert!(
            get_gesture_hover(
                a,
                PALM_DEPTH,
                pose_at(0.0700, 0.0, 0.0),
                THUMB_RADIUS,
                0.02,
                0.025
            )
            .abs()
                < 1e-6
        );

        // Zero / negative denominator & NaN / Inf guards
        let zero_d = get_gesture_hover(
            a,
            THUMB_RADIUS,
            pose_at(0.010, 0.0, 0.0),
            INDEX_RADIUS,
            0.0,
            -(THUMB_RADIUS + INDEX_RADIUS),
        );
        assert!(zero_d.is_finite() && (0.0..=1.0).contains(&zero_d));
        let neg_d = get_gesture_hover(
            a,
            THUMB_RADIUS,
            pose_at(0.050, 0.0, 0.0),
            INDEX_RADIUS,
            0.0,
            -0.1,
        );
        assert_eq!(neg_d, 0.0);
        assert_eq!(
            get_gesture_hover(
                pose_at(f32::NAN, 0.0, 0.0),
                THUMB_RADIUS,
                a,
                INDEX_RADIUS,
                0.0,
                0.0025
            ),
            0.0
        );
    }

    #[test]
    fn debounce_state_machine_strict_boundaries_repeat_delay_and_release_tail() {
        let cfg = LayerConfig::default(); // activation=50, deactivation=100, repeat=100
        let mut engine = HandGestureEngine::new();

        let mut pinched = open_hand_skeleton();
        pinched[HandJoint::ThumbTip.idx()] = pose_at(0.0, 0.0, 0.0);
        pinched[HandJoint::IndexTip.idx()] = pose_at(0.010, 0.0, 0.0); // < 0.014 -> clicked & in_range

        // Hover-range just outside pinch_max (0.0165) but inside hover ramp (< 0.0305)
        let mut release_tail = open_hand_skeleton();
        release_tail[HandJoint::ThumbTip.idx()] = pose_at(0.0, 0.0, 0.0);
        release_tail[HandJoint::IndexTip.idx()] = pose_at(0.018, 0.0, 0.0); // !in_range, hover > 0

        let open = open_hand_skeleton();

        // 1. First activation starting at now_ms = 0 must NOT be blocked by repeat_delay!
        let s0 = engine.update_hand(HandSide::Left, &pinched, 0, &cfg);
        assert!(!s0.trigger_click && !s0.trigger_touch && s0.trigger_value == 0.0);

        // At t=50 (elapsed == activation_delay == 50), strict `<` means NOT active yet!
        let s50 = engine.update_hand(HandSide::Left, &pinched, 50, &cfg);
        assert!(!s50.trigger_click && !s50.trigger_touch && s50.trigger_value == 0.0);

        // At t=51 (elapsed == 51 > 50), becomes active!
        let s51 = engine.update_hand(HandSide::Left, &pinched, 51, &cfg);
        assert!(s51.trigger_click && s51.trigger_touch && (s51.trigger_value - 1.0).abs() < 1e-6);

        // 2. Exiting starts at t=100; check deactivation_delay release tail!
        // While waiting out 100ms deactivation_delay, click & touch drop immediately,
        // while trigger_value continues ramping via get_gesture_hover until t=201!
        let s100 = engine.update_hand(HandSide::Left, &release_tail, 100, &cfg);
        assert!(!s100.trigger_click && !s100.trigger_touch);
        assert!(
            s100.trigger_value > 0.7 && s100.trigger_value < 0.9,
            "expected hover tail while active, got {}",
            s100.trigger_value
        );

        // At t=200 (elapsed == 100 == deactivation_delay), still active
        let s200 = engine.update_hand(HandSide::Left, &release_tail, 200, &cfg);
        assert!(!s200.trigger_click && s200.trigger_value > 0.7);

        // At t=201 (elapsed == 101 > 100), deactivates -> trigger_value snaps to 0.0
        let s201 = engine.update_hand(HandSide::Left, &release_tail, 201, &cfg);
        assert_eq!(s201.trigger_value, 0.0);

        // 3. Repeat block after deactivation at t=201 (repeat_delay=100):
        // At t=301 (elapsed == 100), still blocked (entering does not even start!)
        let s301 = engine.update_hand(HandSide::Left, &pinched, 301, &cfg);
        assert!(!s301.trigger_click);
        // At t=302 (elapsed == 101 > 100), starts entering (entering_since = 302)
        let s302 = engine.update_hand(HandSide::Left, &pinched, 302, &cfg);
        assert!(!s302.trigger_click);
        // At t=352 (352 - 302 == 50), not yet active
        let s352 = engine.update_hand(HandSide::Left, &pinched, 352, &cfg);
        assert!(!s352.trigger_click);
        // At t=353 (353 - 302 == 51 > 50), activates!
        let s353 = engine.update_hand(HandSide::Left, &pinched, 353, &cfg);
        assert!(s353.trigger_click);

        // 4. Interrupted exiting: leave at t=400, return at t=450 -> stays active & resets exiting_since
        let _ = engine.update_hand(HandSide::Left, &open, 400, &cfg);
        let s450 = engine.update_hand(HandSide::Left, &pinched, 450, &cfg);
        assert!(s450.trigger_click);
        // Leave again at t=460 -> at t=550 (550-460 = 90 <= 100), still active (would have exited if not reset)
        let _ = engine.update_hand(HandSide::Left, &release_tail, 460, &cfg);
        let s550 = engine.update_hand(HandSide::Left, &release_tail, 550, &cfg);
        assert!(s550.trigger_value > 0.7);

        // 5. Interrupted entering & backwards clock step never underflow or spuriously trigger
        engine.reset();
        let _ = engine.update_hand(HandSide::Left, &pinched, 1000, &cfg);
        // Backwards clock jump before entering_since
        let s_back = engine.update_hand(HandSide::Left, &pinched, 900, &cfg);
        assert!(!s_back.trigger_click);
        // Drop out at t=1030 -> resets entering
        let _ = engine.update_hand(HandSide::Left, &open, 1030, &cfg);
        // Re-enter at t=1040 -> entering_since=1040; at t=1060 only 20ms elapsed (not 60ms)
        let _ = engine.update_hand(HandSide::Left, &pinched, 1040, &cfg);
        let s1060 = engine.update_hand(HandSide::Left, &pinched, 1060, &cfg);
        assert!(!s1060.trigger_click);
        let s1091 = engine.update_hand(HandSide::Left, &pinched, 1091, &cfg);
        assert!(s1091.trigger_click);
    }

    #[test]
    fn all_four_pinches_and_left_right_asymmetry() {
        let cfg = LayerConfig::default();

        // Helper to activate a given finger pinch after debounce (t=0 then t=60)
        let run_pinch = |hand: HandSide, finger: HandJoint| -> VirtualHandState {
            let mut engine = HandGestureEngine::new();
            let mut gj = open_hand_skeleton();
            gj[HandJoint::ThumbTip.idx()] = pose_at(0.0, 0.0, 0.0);
            gj[finger.idx()] = pose_at(0.005, 0.0, 0.0);
            let _ = engine.update_hand(hand, &gj, 0, &cfg);
            engine.update_hand(hand, &gj, 60, &cfg)
        };

        // ThumbIndexPinch -> trigger + select on both hands
        for hand in [HandSide::Left, HandSide::Right] {
            let s = run_pinch(hand, HandJoint::IndexTip);
            assert!(s.trigger_click && s.trigger_touch && s.select_click);
            assert!((s.trigger_value - 1.0).abs() < 1e-6);
            assert!(!s.primary_click && !s.secondary_click && !s.menu_click);
        }

        // ThumbMiddlePinch -> secondary (Left Y / Right B)
        for hand in [HandSide::Left, HandSide::Right] {
            let s = run_pinch(hand, HandJoint::MiddleTip);
            assert!(s.secondary_click && s.secondary_touch);
            assert!(!s.primary_click && !s.trigger_click && !s.menu_click);
        }

        // ThumbRingPinch -> primary (Left X / Right A)
        for hand in [HandSide::Left, HandSide::Right] {
            let s = run_pinch(hand, HandJoint::RingTip);
            assert!(s.primary_click && s.primary_touch);
            assert!(!s.secondary_click && !s.trigger_click && !s.menu_click);
        }

        // ThumbLittlePinch -> Left menu_click ONLY; Right menu_click MUST be false!
        let s_left = run_pinch(HandSide::Left, HandJoint::LittleTip);
        let s_right = run_pinch(HandSide::Right, HandJoint::LittleTip);
        assert!(s_left.menu_click);
        assert!(!s_right.menu_click);
    }

    #[test]
    fn curls_grip_curl_and_thumb_curl() {
        let cfg = LayerConfig::default();
        let mut engine = HandGestureEngine::new();

        // Partial grip: middle=1.0, ring=1.0, little=0.0 -> squeeze_value = 2/3, squeeze_click = false
        let mut gj = open_hand_skeleton();
        gj[HandJoint::MiddleMetacarpal.idx()] = pose_at(0.0, 0.0, 0.0);
        gj[HandJoint::MiddleProximal.idx()] = pose_at(0.0, 0.0, 0.0);
        gj[HandJoint::MiddleTip.idx()] = pose_at(0.010, 0.0, 0.0); // curl = 1.0
        gj[HandJoint::RingMetacarpal.idx()] = pose_at(0.0, 0.2, 0.0);
        gj[HandJoint::RingProximal.idx()] = pose_at(0.0, 0.2, 0.0);
        gj[HandJoint::RingTip.idx()] = pose_at(0.010, 0.2, 0.0); // curl = 1.0
        // little remains far away -> curl = 0.0

        let s_part = engine.update_hand(HandSide::Left, &gj, 10, &cfg);
        assert!((s_part.squeeze_value - (2.0 / 3.0)).abs() < 1e-5);
        assert!(!s_part.squeeze_click);
        assert!(engine.squeeze_hysteresis_pressed(HandSide::Left));

        // Full grip: add little=1.0 -> squeeze_value = 1.0, squeeze_click = true
        gj[HandJoint::LittleMetacarpal.idx()] = pose_at(0.0, 0.4, 0.0);
        gj[HandJoint::LittleProximal.idx()] = pose_at(0.0, 0.4, 0.0);
        gj[HandJoint::LittleTip.idx()] = pose_at(0.010, 0.4, 0.0);
        let s_full = engine.update_hand(HandSide::Left, &gj, 20, &cfg);
        assert!((s_full.squeeze_value - 1.0).abs() < 1e-6);
        assert!(s_full.squeeze_click);

        // ThumbCurl exact boundary (0.05125 m => thumb_curl = 0.5, no debounce)
        let mut gj_tc = open_hand_skeleton();
        gj_tc[HandJoint::Palm.idx()] = pose_at(0.0, 0.0, 0.0);
        gj_tc[HandJoint::ThumbTip.idx()] = pose_at(0.0512, 0.0, 0.0);
        assert!(
            engine
                .update_hand(HandSide::Right, &gj_tc, 0, &cfg)
                .thumbstick_click
        );
        gj_tc[HandJoint::ThumbTip.idx()] = pose_at(0.0513, 0.0, 0.0);
        assert!(
            !engine
                .update_hand(HandSide::Right, &gj_tc, 1, &cfg)
                .thumbstick_click
        );
    }

    #[test]
    fn joystick_four_gates_basis_frames_mirroring_deadzone_and_guards() {
        let cfg = LayerConfig::default(); // range=1.0cm (0.01m), deadzone=40% (0.40)
        let mut engine = HandGestureEngine::new();

        // Build base skeleton with full index curl (1.0), full grip curl (1.0),
        // joystick_center at origin with identity orientations.
        let make_joy_hand = |thumb_offset: Vec3| -> [Pose; HAND_JOINT_COUNT] {
            let mut gj = open_hand_skeleton();
            // Full grip curl (middle, ring, little at 1.0)
            for (m, p, t, y) in [
                (
                    HandJoint::MiddleMetacarpal,
                    HandJoint::MiddleProximal,
                    HandJoint::MiddleTip,
                    1.0,
                ),
                (
                    HandJoint::RingMetacarpal,
                    HandJoint::RingProximal,
                    HandJoint::RingTip,
                    2.0,
                ),
                (
                    HandJoint::LittleMetacarpal,
                    HandJoint::LittleProximal,
                    HandJoint::LittleTip,
                    3.0,
                ),
            ] {
                gj[m.idx()] = pose_at(0.0, y, 0.0);
                gj[p.idx()] = pose_at(0.0, y, 0.0);
                gj[t.idx()] = pose_at(0.01, y, 0.0);
            }
            // Full index curl (1.0)
            gj[HandJoint::IndexMetacarpal.idx()] = pose_at(0.0, 4.0, 0.0);
            gj[HandJoint::IndexProximal.idx()] = pose_at(0.0, 4.0, 0.0);
            gj[HandJoint::IndexTip.idx()] = pose_at(0.01, 4.0, 0.0);

            // Joystick center = lerp(gj[8], gj[9], 0.5) at (0, 0, 0)
            gj[HandJoint::IndexIntermediate.idx()] = pose_at(0.0, 0.0, 0.0);
            gj[HandJoint::IndexDistal.idx()] = pose_at(0.0, 0.0, 0.0);
            gj[HandJoint::ThumbTip.idx()] = pose_at(thumb_offset.x, thumb_offset.y, thumb_offset.z);
            gj
        };

        // 1. Handedness mirroring: +Y delta -> +X on Left, -X on Right; +Z delta -> +Y on both
        let gj_pos = make_joy_hand(Vec3::new(0.0, 0.008, 0.006)); // (0.8, 0.6) on Left
        let sl = engine.update_hand(HandSide::Left, &gj_pos, 10, &cfg);
        let sr = engine.update_hand(HandSide::Right, &gj_pos, 10, &cfg);
        assert!(sl.thumbstick_touch && sr.thumbstick_touch);
        assert!((sl.thumbstick.x - 0.8).abs() < 1e-5 && (sl.thumbstick.y - 0.6).abs() < 1e-5);
        assert!((sr.thumbstick.x - (-0.8)).abs() < 1e-5 && (sr.thumbstick.y - 0.6).abs() < 1e-5);

        // Horizontal offset mirrors across hands; vertical offset does NOT mirror
        let mut cfg_off = cfg.clone();
        cfg_off.gestures.joystick_offset_horizontal = 0.5; // +0.5 cm => +0.5 on Left, -0.5 on Right
        cfg_off.gestures.joystick_offset_vertical = 0.6; // +0.6 cm => +0.6 on both
        let gj_zero = make_joy_hand(Vec3::ZERO);
        let sl_o = engine.update_hand(HandSide::Left, &gj_zero, 20, &cfg_off);
        let sr_o = engine.update_hand(HandSide::Right, &gj_zero, 20, &cfg_off);
        assert!((sl_o.thumbstick.x - 0.5).abs() < 1e-5 && (sl_o.thumbstick.y - 0.6).abs() < 1e-5);
        assert!((sr_o.thumbstick.x - (-0.5)).abs() < 1e-5 && (sr_o.thumbstick.y - 0.6).abs() < 1e-5);

        // 2. Square per-axis deadzone (40%): (0.399, 0.40) -> (0.0, 0.40) without rescaling
        let gj_dz = make_joy_hand(Vec3::new(0.0, 0.00399, 0.0040));
        let s_dz = engine.update_hand(HandSide::Left, &gj_dz, 30, &cfg);
        assert!(s_dz.thumbstick_touch);
        assert_eq!(s_dz.thumbstick.x, 0.0);
        assert!((s_dz.thumbstick.y - 0.40).abs() < 1e-5);

        // 3. Toggle each of the 4 contact gates individually:
        // Gate 1: index_curl >= 0.75 (for index: r1+r2=0.0115, min=0.02, denom=0.0365 -> d = 0.0315 + 0.25*0.0365 = 0.040625)
        let mut gj_g1 = make_joy_hand(Vec3::new(0.0, 0.008, 0.0));
        gj_g1[HandJoint::IndexTip.idx()] = pose_at(0.0407, 4.0, 0.0); // index_curl < 0.75
        assert!(!engine.update_hand(HandSide::Left, &gj_g1, 40, &cfg).thumbstick_touch);
        gj_g1[HandJoint::IndexTip.idx()] = pose_at(0.0406, 4.0, 0.0); // index_curl >= 0.75
        assert!(engine.update_hand(HandSide::Left, &gj_g1, 41, &cfg).thumbstick_touch);

        // Gate 2: grip_curl > 0.5 (strictly > 0.5)
        let mut gj_g2 = make_joy_hand(Vec3::new(0.0, 0.008, 0.0));
        // middle=1.0, ring=0.5, little=0.0 => grip_curl = 0.5 (NOT > 0.5!)
        gj_g2[HandJoint::RingTip.idx()] = pose_at(0.0495, 2.0, 0.0); // ring: min+r=0.031, denom=0.036 -> 0.5 at 0.049
        gj_g2[HandJoint::LittleTip.idx()] = pose_at(10.0, 3.0, 0.0); // 0.0
        assert!(!engine.update_hand(HandSide::Left, &gj_g2, 42, &cfg).thumbstick_touch);
        gj_g2[HandJoint::RingTip.idx()] = pose_at(0.0480, 2.0, 0.0); // > 0.5
        assert!(engine.update_hand(HandSide::Left, &gj_g2, 43, &cfg).thumbstick_touch);

        // Gate 3: distance <= joystick_range * 3.0 (= 0.03 m)
        let gj_g3_fail = make_joy_hand(Vec3::new(0.0, 0.0301, 0.0));
        assert!(!engine.update_hand(HandSide::Left, &gj_g3_fail, 44, &cfg).thumbstick_touch);
        let gj_g3_pass = make_joy_hand(Vec3::new(0.0, 0.0300, 0.0));
        assert!(engine.update_hand(HandSide::Left, &gj_g3_pass, 45, &cfg).thumbstick_touch);

        // Gate 4: signed up_proj <= joystick_range * 2.0 (= 0.02 m); negative -0.025 passes!
        let gj_g4_fail = make_joy_hand(Vec3::new(0.0201, 0.0, 0.0));
        assert!(!engine.update_hand(HandSide::Left, &gj_g4_fail, 46, &cfg).thumbstick_touch);
        let gj_g4_pass = make_joy_hand(Vec3::new(0.0200, 0.0, 0.0));
        assert!(engine.update_hand(HandSide::Left, &gj_g4_pass, 47, &cfg).thumbstick_touch);
        let gj_g4_neg = make_joy_hand(Vec3::new(-0.025, 0.008, 0.0)); // dist=0.0262 <= 0.03, up_proj=-0.025 <= 0.02
        assert!(engine.update_hand(HandSide::Left, &gj_g4_neg, 48, &cfg).thumbstick_touch);

        // 4. Distinct orientations on gj[8] vs gj[9]:
        // gj[8] = IDENTITY, gj[9] = Rz(+90 deg) -> joystick_center.ori = Rz(+45 deg).
        // For Left: joystick_up = (cos45, sin45, 0) while hori_vec = (0, 1, 0) from gj[8]!
        // Place thumb at (0.015, 0.015, 0): dist=0.0212 <= 0.03;
        // projection onto Rz(45)*X = 0.015*sqrt(2) = 0.0212 > 0.02 -> Gate 4 FAILS!
        // (If joystick_up mistakenly used gj[8].ori = X, proj would be 0.015 <= 0.02 and wrongly pass!)
        let mut gj_rot = make_joy_hand(Vec3::new(0.015, 0.015, 0.0));
        gj_rot[HandJoint::IndexDistal.idx()].orientation = Quat::from_rotation_z(FRAC_PI_2);
        assert!(
            !engine.update_hand(HandSide::Left, &gj_rot, 50, &cfg).thumbstick_touch,
            "joystick_up must use joystick_center.orientation, not gj[8].orientation"
        );
        // Move thumb to (0.0, 0.008, 0.0): up_proj = 0.008*cos45 = 0.00566 <= 0.02 (passes!),
        // and hori projection uses gj[8].ori (Y) giving exact +0.8 (NOT 0.8*cos45 = 0.566!)
        gj_rot[HandJoint::ThumbTip.idx()] = pose_at(0.0, 0.008, 0.0);
        let s_rot = engine.update_hand(HandSide::Left, &gj_rot, 51, &cfg);
        assert!(s_rot.thumbstick_touch);
        assert!(
            (s_rot.thumbstick.x - 0.8).abs() < 1e-5,
            "hori_vec must use gj[8].orientation, got {}",
            s_rot.thumbstick.x
        );

        // 5. Defense-in-depth guards: joystick_range == 0.0 and zero-quaternion gj[8]/gj[9] -> finite Vec2::ZERO
        let mut cfg_zero_r = cfg.clone();
        cfg_zero_r.gestures.joystick_range = 0.0;
        let s_zr = engine.update_hand(HandSide::Left, &gj_pos, 60, &cfg_zero_r);
        assert!(!s_zr.thumbstick_touch && s_zr.thumbstick == Vec2::ZERO);

        let mut gj_zero_q = make_joy_hand(Vec3::new(0.0, 0.008, 0.006));
        gj_zero_q[HandJoint::IndexIntermediate.idx()].orientation =
            Quat::from_xyzw(0.0, 0.0, 0.0, 0.0);
        gj_zero_q[HandJoint::IndexDistal.idx()].orientation = Quat::from_xyzw(0.0, 0.0, 0.0, 0.0);
        let s_zq = engine.update_hand(HandSide::Left, &gj_zero_q, 61, &cfg);
        assert!(!s_zq.thumbstick_touch && s_zq.thumbstick == Vec2::ZERO);
    }

    #[test]
    fn hysteresis_state_boundaries_saturation_and_guards() {
        // Default 0.5 ± 0.05
        let mut h = HysteresisState::new();
        assert!(!h.update(0.55, 0.5, 0.05)); // strict > 0.55
        assert!(h.update(0.551, 0.5, 0.05));
        assert!(h.update(0.45, 0.5, 0.05)); // strict < 0.45
        assert!(!h.update(0.449, 0.5, 0.05));

        // Zero hysteresis 0.5 ± 0.0
        let mut hz = HysteresisState::new();
        assert!(!hz.update(0.5, 0.5, 0.0));
        assert!(hz.update(0.501, 0.5, 0.0));
        assert!(hz.update(0.5, 0.5, 0.0));
        assert!(!hz.update(0.499, 0.5, 0.0));

        // Negative hysteresis clamped to 0.0 (no oscillation at 0.5)
        let mut hn = HysteresisState::new();
        assert!(hn.update(0.6, 0.5, -0.2));
        assert!(hn.update(0.5, 0.5, -0.2));
        assert!(hn.update(0.5, 0.5, -0.2));

        // Saturated low boundary (0.05 ± 0.05): releases cleanly at 0.0
        let mut hlow = HysteresisState::new();
        assert!(hlow.update(0.2, 0.05, 0.05));
        assert!(!hlow.update(0.0, 0.05, 0.05));

        // Saturated high boundaries (0.95 ± 0.05 and 1.0 ± 0.0): press cleanly at 1.0
        let mut hhigh = HysteresisState::new();
        assert!(!hhigh.update(0.99, 0.95, 0.05));
        assert!(hhigh.update(1.0, 0.95, 0.05));

        let mut h1 = HysteresisState::new();
        assert!(!h1.update(0.99, 1.0, 0.0));
        assert!(h1.update(1.0, 1.0, 0.0));
        assert!(!h1.update(0.99, 1.0, 0.0));

        // NaN preserves state
        let mut hnan = HysteresisState::new();
        assert!(hnan.update(0.9, 0.5, 0.05));
        assert!(hnan.update(f32::NAN, 0.5, 0.05));
    }

    #[test]
    fn only_touch_clear_hand_reset_and_disabled_config() {
        let mut engine = HandGestureEngine::new();
        let cfg = LayerConfig::default();

        // Build a skeleton triggering ALL gestures simultaneously:
        // thumb at origin; all 4 fingertips at (0.005, 0, 0) -> all 4 pinches active;
        // palm at origin -> thumb_curl = 1.0;
        // index/middle/ring/little meta & prox at origin -> all curls = 1.0 -> grip = 1.0;
        // index_inter & index_distal at (0, -0.008, -0.008) -> joystick active with (+0.8, +0.8).
        let mut all_active = open_hand_skeleton();
        all_active[HandJoint::Palm.idx()] = pose_at(0.0, 0.0, 0.0);
        all_active[HandJoint::ThumbTip.idx()] = pose_at(0.0, 0.0, 0.0);
        for j in [
            HandJoint::IndexMetacarpal,
            HandJoint::IndexProximal,
            HandJoint::MiddleMetacarpal,
            HandJoint::MiddleProximal,
            HandJoint::RingMetacarpal,
            HandJoint::RingProximal,
            HandJoint::LittleMetacarpal,
            HandJoint::LittleProximal,
        ] {
            all_active[j.idx()] = pose_at(0.0, 0.0, 0.0);
        }
        for j in [
            HandJoint::IndexTip,
            HandJoint::MiddleTip,
            HandJoint::RingTip,
            HandJoint::LittleTip,
        ] {
            all_active[j.idx()] = pose_at(0.005, 0.0, 0.0);
        }
        all_active[HandJoint::IndexIntermediate.idx()] = pose_at(0.0, -0.008, -0.008);
        all_active[HandJoint::IndexDistal.idx()] = pose_at(0.0, -0.008, -0.008);

        // Activate on both hands
        let _ = engine.update_hand(HandSide::Left, &all_active, 0, &cfg);
        let _ = engine.update_hand(HandSide::Right, &all_active, 0, &cfg);
        let sl = engine.update_hand(HandSide::Left, &all_active, 60, &cfg);
        let sr = engine.update_hand(HandSide::Right, &all_active, 60, &cfg);
        assert!(sl.trigger_click && sl.squeeze_click && sl.primary_click && sl.secondary_click);
        assert!(sl.menu_click && sl.thumbstick_click && sl.select_click && sl.thumbstick_touch);
        assert!(sl.thumbstick.x > 0.5 && sl.thumbstick.y > 0.5);
        assert!(engine.trigger_hysteresis_pressed(HandSide::Left));
        assert!(engine.squeeze_hysteresis_pressed(HandSide::Left));
        assert!(sr.trigger_click && !sr.menu_click);

        // Toggle only_touch = true mid-press: touch channels stay true, clicks/values/hysteresis release!
        let mut cfg_ot = cfg.clone();
        cfg_ot.gestures.only_touch = true;
        let sl_ot = engine.update_hand(HandSide::Left, &all_active, 70, &cfg_ot);
        assert!(sl_ot.tracked);
        assert!(sl_ot.trigger_touch);
        assert!(sl_ot.primary_touch);
        assert!(sl_ot.secondary_touch);
        assert!(sl_ot.thumbstick_touch);
        assert_eq!(sl_ot.trigger_value, 0.0);
        assert!(!sl_ot.trigger_click);
        assert_eq!(sl_ot.squeeze_value, 0.0);
        assert!(!sl_ot.squeeze_click);
        assert_eq!(sl_ot.thumbstick, Vec2::ZERO);
        assert!(!sl_ot.thumbstick_click);
        assert!(!sl_ot.primary_click);
        assert!(!sl_ot.secondary_click);
        assert!(!sl_ot.menu_click);
        assert!(!sl_ot.select_click);
        assert!(!engine.trigger_hysteresis_pressed(HandSide::Left));
        assert!(!engine.squeeze_hysteresis_pressed(HandSide::Left));

        // clear_hand(Left) clears Left without affecting Right, and removes prior repeat_delay block
        let cleared = engine.clear_hand(HandSide::Left);
        assert_eq!(cleared, VirtualHandState::default());
        assert!(engine.trigger_hysteresis_pressed(HandSide::Right));
        // Re-entering on Left immediately at t=71 (not blocked by repeat_delay) succeeds at t=122
        let _ = engine.update_hand(HandSide::Left, &all_active, 71, &cfg);
        let s_react = engine.update_hand(HandSide::Left, &all_active, 122, &cfg);
        assert!(s_react.trigger_click);

        // cfg.enabled = false returns default untracked state and clears hand state
        let mut cfg_off = cfg.clone();
        cfg_off.enabled = false;
        let s_dis = engine.update_hand(HandSide::Right, &all_active, 130, &cfg_off);
        assert_eq!(s_dis, VirtualHandState::default());
        assert!(!engine.trigger_hysteresis_pressed(HandSide::Right));

        // reset() clears both hands
        engine.reset();
        assert!(!engine.trigger_hysteresis_pressed(HandSide::Left));
        assert!(!engine.squeeze_hysteresis_pressed(HandSide::Left));
    }

    #[test]
    fn occluded_invalid_zeroed_joints_never_false_trigger_gestures() {
        let cfg = LayerConfig::default();
        let mut engine = HandGestureEngine::new();

        // All joints defaulted (position_valid = false, pose = IDENTITY at (0,0,0)):
        // Previously passing raw poses caused distance 0.0 -> EVERY pinch/curl fired!
        let invalid_all = [crate::pose::HandJointSample::default(); HAND_JOINT_COUNT];
        let _ = engine.update_hand_samples(HandSide::Left, &invalid_all, 0, &cfg);
        let s = engine.update_hand_samples(HandSide::Left, &invalid_all, 80, &cfg);
        assert!(s.tracked);
        assert!(!s.trigger_click && !s.trigger_touch && s.trigger_value == 0.0);
        assert!(!s.primary_click && !s.primary_touch);
        assert!(!s.secondary_click && !s.secondary_touch);
        assert!(!s.menu_click && !s.select_click);
        assert!(!s.squeeze_click && s.squeeze_value == 0.0);
        assert!(!s.thumbstick_click && !s.thumbstick_touch && s.thumbstick == Vec2::ZERO);

        // Partially occluded hand: open valid hand, EXCEPT index tip & thumb tip lose position_valid
        // and get zeroed to (0,0,0) -> MUST NOT trigger pinch or index curl!
        let open = open_hand_skeleton();
        let mut samples = std::array::from_fn(|i| crate::pose::HandJointSample {
            pose: open[i],
            position_valid: true,
            orientation_valid: true,
            position_tracked: true,
            orientation_tracked: true,
            radius: 0.008,
            linear_velocity: None,
            angular_velocity: None,
        });
        samples[HandJoint::ThumbTip.idx()].pose = Pose::IDENTITY;
        samples[HandJoint::ThumbTip.idx()].position_valid = false;
        samples[HandJoint::IndexTip.idx()].pose = Pose::IDENTITY;
        samples[HandJoint::IndexTip.idx()].position_valid = false;

        let _ = engine.update_hand_samples(HandSide::Left, &samples, 100, &cfg);
        let s2 = engine.update_hand_samples(HandSide::Left, &samples, 200, &cfg);
        assert!(!s2.trigger_click && !s2.trigger_touch && s2.trigger_value == 0.0);
        assert!(!s2.thumbstick_click && !s2.thumbstick_touch);

        // Active pinch followed by joint occlusion exits cleanly after deactivation_delay
        let mut pinched = samples;
        pinched[HandJoint::ThumbTip.idx()].pose = pose_at(0.0, 0.0, 0.0);
        pinched[HandJoint::ThumbTip.idx()].position_valid = true;
        pinched[HandJoint::IndexTip.idx()].pose = pose_at(0.008, 0.0, 0.0);
        pinched[HandJoint::IndexTip.idx()].position_valid = true;
        let _ = engine.update_hand_samples(HandSide::Left, &pinched, 300, &cfg);
        let s_on = engine.update_hand_samples(HandSide::Left, &pinched, 360, &cfg);
        assert!(s_on.trigger_click);

        // Now occlude index tip -> value drops immediately, debounce deactivates after > 100ms
        let mut occluded = pinched;
        occluded[HandJoint::IndexTip.idx()].position_valid = false;
        let s_occ0 = engine.update_hand_samples(HandSide::Left, &occluded, 400, &cfg);
        assert!(!s_occ0.trigger_click && s_occ0.trigger_value == 0.0);
        let s_occ1 = engine.update_hand_samples(HandSide::Left, &occluded, 505, &cfg);
        assert!(!s_occ1.trigger_click && !s_occ1.trigger_touch && s_occ1.trigger_value == 0.0);
    }
}
