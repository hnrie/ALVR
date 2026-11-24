use serde::{Deserialize, Serialize};

pub const MOCK_RUNTIME_MAGIC: u64 = 0x414C_5652_4D4F_434B; // "ALVRMOCK"
pub const HAND_JOINT_COUNT: usize = 26;

pub const DEFAULT_INITIAL_TIME_NS: i64 = 1_000_000_000;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MockExtensionSpec {
    pub name: String,
    pub version: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MockPose {
    /// Quaternion [x, y, z, w]
    pub orientation: [f32; 4],
    /// Position [x, y, z] in meters
    pub position: [f32; 3],
}

impl Default for MockPose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl MockPose {
    pub const IDENTITY: Self = Self {
        orientation: [0.0, 0.0, 0.0, 1.0],
        position: [0.0, 0.0, 0.0],
    };

    pub fn from_quat_pos(q: glam::Quat, p: glam::Vec3) -> Self {
        Self {
            orientation: [q.x, q.y, q.z, q.w],
            position: [p.x, p.y, p.z],
        }
    }

    pub fn to_quat_pos(self) -> (glam::Quat, glam::Vec3) {
        let q = glam::Quat::from_xyzw(
            self.orientation[0],
            self.orientation[1],
            self.orientation[2],
            self.orientation[3],
        );
        let p = glam::Vec3::from_array(self.position);
        (q, p)
    }

    pub fn to_xr(self) -> openxr_sys::Posef {
        openxr_sys::Posef {
            orientation: openxr_sys::Quaternionf {
                x: self.orientation[0],
                y: self.orientation[1],
                z: self.orientation[2],
                w: self.orientation[3],
            },
            position: openxr_sys::Vector3f {
                x: self.position[0],
                y: self.position[1],
                z: self.position[2],
            },
        }
    }

    pub fn from_xr(p: openxr_sys::Posef) -> Self {
        Self {
            orientation: [
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            ],
            position: [p.position.x, p.position.y, p.position.z],
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MockRuntimeConfig {
    pub supported_extensions: Vec<MockExtensionSpec>,
    pub supports_hand_tracking: bool,
    /// OpenXR XrResult raw i32 returned by xrCreateHandTrackerEXT (0 = XR_SUCCESS).
    pub create_hand_tracker_error: i32,
    /// OpenXR XrResult raw i32 returned by xrCreateInstance (0 = XR_SUCCESS).
    pub create_instance_error: i32,
    /// Artificial sleep duration (ms) executed outside mutex inside xrPollEvent.
    pub poll_event_delay_ms: u64,
    /// Reported runtime API version [major, minor, patch].
    pub api_version: [u32; 3],
    /// Pose of STAGE reference space origin expressed in LOCAL space (T_local<-stage).
    pub stage_to_local_pose: MockPose,
    /// Pose of VIEW reference space origin expressed in LOCAL space (T_local<-view).
    pub view_to_local_pose: MockPose,
    /// Pose of LOCAL_FLOOR reference space origin expressed in LOCAL space (T_local<-local_floor).
    pub local_floor_to_local_pose: MockPose,
    /// Initial session state when xrCreateSession is called (default: FOCUSED = 5 for direct tests,
    /// or IDLE = 1 if caller drives full state transitions).
    pub initial_session_state: i32,
}

impl Default for MockRuntimeConfig {
    fn default() -> Self {
        Self {
            supported_extensions: vec![
                MockExtensionSpec {
                    name: "XR_EXT_hand_tracking".to_string(),
                    version: 4,
                },
                MockExtensionSpec {
                    name: "XR_FB_hand_tracking_aim".to_string(),
                    version: 2,
                },
                MockExtensionSpec {
                    name: "XR_KHR_locate_spaces".to_string(),
                    version: 1,
                },
                MockExtensionSpec {
                    name: "XR_EXT_palm_pose".to_string(),
                    version: 3,
                },
                MockExtensionSpec {
                    name: "XR_META_touch_controller_plus".to_string(),
                    version: 1,
                },
                MockExtensionSpec {
                    name: "XR_FB_touch_controller_pro".to_string(),
                    version: 1,
                },
                MockExtensionSpec {
                    name: "XR_MND_headless".to_string(),
                    version: 3,
                },
            ],
            supports_hand_tracking: true,
            create_hand_tracker_error: 0,
            create_instance_error: 0,
            poll_event_delay_ms: 0,
            api_version: [1, 1, 54],
            stage_to_local_pose: MockPose::IDENTITY,
            view_to_local_pose: MockPose::IDENTITY,
            local_floor_to_local_pose: MockPose::IDENTITY,
            initial_session_state: openxr_sys::SessionState::FOCUSED.into_raw(),
        }
    }
}

/// Default full valid+tracked SpaceLocationFlags bitmask:
/// ORIENTATION_VALID (1) | POSITION_VALID (2) | ORIENTATION_TRACKED (4) | POSITION_TRACKED (8) = 0x0F.
pub const FULL_LOCATION_FLAGS: u64 = 0x0F;
/// LINEAR_VALID (1) | ANGULAR_VALID (2) = 0x03.
pub const FULL_VELOCITY_FLAGS: u64 = 0x03;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MockJointInput {
    pub pose: MockPose,
    pub location_flags: u64,
    pub radius: f32,
    pub velocity_flags: u64,
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
}

impl Default for MockJointInput {
    fn default() -> Self {
        Self {
            pose: MockPose::IDENTITY,
            location_flags: FULL_LOCATION_FLAGS,
            radius: 0.008,
            velocity_flags: 0,
            linear_velocity: [0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.0],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MockAimInput {
    /// XrHandTrackingAimFlagsFB bitmask (COMPUTED=1, VALID=2, etc.)
    pub status: u64,
    pub aim_pose: MockPose,
    pub pinch_strength_index: f32,
    pub pinch_strength_middle: f32,
    pub pinch_strength_ring: f32,
    pub pinch_strength_little: f32,
}

impl Default for MockAimInput {
    fn default() -> Self {
        Self {
            status: 0,
            aim_pose: MockPose::IDENTITY,
            pinch_strength_index: 0.0,
            pinch_strength_middle: 0.0,
            pinch_strength_ring: 0.0,
            pinch_strength_little: 0.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MockHandState {
    pub is_active: bool,
    pub joints: Vec<MockJointInput>,
    pub aim: Option<MockAimInput>,
}

impl Default for MockHandState {
    fn default() -> Self {
        Self {
            is_active: false,
            joints: vec![MockJointInput::default(); HAND_JOINT_COUNT],
            aim: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MockPhysicalControllerState {
    pub active: bool,
    /// Optional interaction profile path string; if None when active, runtime picks first suggested profile.
    pub interaction_profile: Option<String>,
    pub trigger_value: f32,
    pub trigger_touch: bool,
    pub trigger_click: bool,
    pub squeeze_value: f32,
    pub squeeze_click: bool,
    pub thumbstick: [f32; 2],
    pub thumbstick_touch: bool,
    pub thumbstick_click: bool,
    pub primary_click: bool,
    pub primary_touch: bool,
    pub secondary_click: bool,
    pub secondary_touch: bool,
    pub menu_click: bool,
    pub system_click: bool,
    pub select_click: bool,
    pub thumbrest_touch: bool,
    pub grip_pose: MockPose,
    pub grip_location_flags: u64,
    pub grip_velocity_flags: u64,
    pub grip_linear_velocity: [f32; 3],
    pub grip_angular_velocity: [f32; 3],
    pub aim_pose: MockPose,
    pub aim_location_flags: u64,
    pub palm_pose: MockPose,
    pub palm_location_flags: u64,
    pub grip_surface_pose: MockPose,
    pub grip_surface_location_flags: u64,
}

impl Default for MockPhysicalControllerState {
    fn default() -> Self {
        Self {
            active: false,
            interaction_profile: None,
            trigger_value: 0.0,
            trigger_touch: false,
            trigger_click: false,
            squeeze_value: 0.0,
            squeeze_click: false,
            thumbstick: [0.0, 0.0],
            thumbstick_touch: false,
            thumbstick_click: false,
            primary_click: false,
            primary_touch: false,
            secondary_click: false,
            secondary_touch: false,
            menu_click: false,
            system_click: false,
            select_click: false,
            thumbrest_touch: false,
            grip_pose: MockPose::IDENTITY,
            grip_location_flags: FULL_LOCATION_FLAGS,
            grip_velocity_flags: 0,
            grip_linear_velocity: [0.0, 0.0, 0.0],
            grip_angular_velocity: [0.0, 0.0, 0.0],
            aim_pose: MockPose::IDENTITY,
            aim_location_flags: FULL_LOCATION_FLAGS,
            palm_pose: MockPose::IDENTITY,
            palm_location_flags: FULL_LOCATION_FLAGS,
            grip_surface_pose: MockPose::IDENTITY,
            grip_surface_location_flags: FULL_LOCATION_FLAGS,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum MockEvent {
    SessionStateChanged {
        /// Optional explicit session handle (0 = latest live session of the instance).
        #[serde(default)]
        session: u64,
        state: i32,
        #[serde(default)]
        time_ns: i64,
    },
    InteractionProfileChanged {
        #[serde(default)]
        session: u64,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CreateInstanceAttempt {
    pub api_version: [u32; 3],
    pub enabled_extensions: Vec<String>,
    pub enabled_api_layers: Vec<String>,
    pub result: i32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MockHapticCall {
    pub session: u64,
    pub action: u64,
    pub subaction_path: String,
    pub duration_ns: i64,
    pub frequency: f32,
    pub amplitude: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MockHapticStopCall {
    pub session: u64,
    pub action: u64,
    pub subaction_path: String,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct MockDiagnostics {
    pub runtime_magic: u64,
    pub negotiate_calls: u64,
    pub create_instance_attempts: Vec<CreateInstanceAttempt>,
    pub get_system_properties_hand_tracking_queried: u64,
    pub hand_trackers_created: u64,
    pub hand_trackers_destroyed: u64,
    pub hand_tracker_create_hands: Vec<i32>,
    pub locate_hand_joints_calls: u64,
    pub sync_actions_calls: u64,
    pub poll_event_calls: u64,
    pub locate_space_calls: u64,
    pub locate_spaces_calls: u64,
    pub haptic_apply_calls: Vec<MockHapticCall>,
    pub haptic_stop_calls: Vec<MockHapticStopCall>,
    pub leaked_hand_trackers_on_destroy_session: u64,
    pub leaked_spaces_on_destroy_session: u64,
    pub leaked_sessions_on_destroy_instance: u64,
    pub leaked_action_sets_on_destroy_instance: u64,
    pub destroy_order_log: Vec<String>,
    pub current_time_ns: i64,
    pub live_instances: usize,
    pub live_sessions: usize,
    pub live_hand_trackers: usize,
    pub live_spaces: usize,
}
