//! Thread-safe global OpenXR handle tables, dispatch table, and per-instance/session state.

use alvr_hand_core::{
    HandGestureEngine, HandSide, HysteresisState, LayerConfig, Pose, PoseSource, PoseSynthesizer,
    VirtualHandState,
};
use glam::Vec2;
use openxr_sys as xr;
use openxr_sys::Handle;
use parking_lot::{Mutex, RwLock};
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::{Arc, LazyLock};
use std::time::Instant;

/// Canonical OpenXR API layer name advertised in manifest JSON and negotiation.
pub const LAYER_NAME: &str = "XR_APILAYER_ALVR_hand_emulation";

/// Extension names consumed internally by the layer.
pub const EXT_HAND_TRACKING: &str = "XR_EXT_hand_tracking";
pub const EXT_FB_HAND_TRACKING_AIM: &str = "XR_FB_hand_tracking_aim";
pub const EXT_KHR_LOCATE_SPACES: &str = "XR_KHR_locate_spaces";

/// Downstream OpenXR function pointer table populated per `XrInstance`.
#[derive(Clone, Copy)]
pub struct NextDispatch {
    pub get_instance_proc_addr: xr::pfn::GetInstanceProcAddr,
    pub destroy_instance: xr::pfn::DestroyInstance,
    pub poll_event: xr::pfn::PollEvent,
    pub get_system_properties: xr::pfn::GetSystemProperties,
    pub create_session: xr::pfn::CreateSession,
    pub destroy_session: xr::pfn::DestroySession,
    pub begin_session: xr::pfn::BeginSession,
    pub end_session: xr::pfn::EndSession,
    pub wait_frame: Option<xr::pfn::WaitFrame>,
    pub string_to_path: xr::pfn::StringToPath,
    pub path_to_string: xr::pfn::PathToString,
    pub create_reference_space: xr::pfn::CreateReferenceSpace,
    pub create_action_space: xr::pfn::CreateActionSpace,
    pub destroy_space: xr::pfn::DestroySpace,
    pub locate_space: xr::pfn::LocateSpace,
    pub locate_spaces: Option<xr::pfn::LocateSpaces>,
    pub locate_spaces_khr: Option<xr::pfn::LocateSpacesKHR>,
    pub create_action_set: xr::pfn::CreateActionSet,
    pub destroy_action_set: xr::pfn::DestroyActionSet,
    pub create_action: xr::pfn::CreateAction,
    pub destroy_action: xr::pfn::DestroyAction,
    pub suggest_interaction_profile_bindings: xr::pfn::SuggestInteractionProfileBindings,
    pub attach_session_action_sets: xr::pfn::AttachSessionActionSets,
    pub get_current_interaction_profile: xr::pfn::GetCurrentInteractionProfile,
    pub sync_actions: xr::pfn::SyncActions,
    pub get_action_state_boolean: xr::pfn::GetActionStateBoolean,
    pub get_action_state_float: xr::pfn::GetActionStateFloat,
    pub get_action_state_vector2f: xr::pfn::GetActionStateVector2f,
    pub get_action_state_pose: xr::pfn::GetActionStatePose,
    pub enumerate_bound_sources_for_action: xr::pfn::EnumerateBoundSourcesForAction,
    pub get_input_source_localized_name: xr::pfn::GetInputSourceLocalizedName,
    pub apply_haptic_feedback: xr::pfn::ApplyHapticFeedback,
    pub stop_haptic_feedback: xr::pfn::StopHapticFeedback,
    // XR_EXT_hand_tracking (optional if runtime lacks hand tracking)
    pub create_hand_tracker_ext: Option<xr::pfn::CreateHandTrackerEXT>,
    pub destroy_hand_tracker_ext: Option<xr::pfn::DestroyHandTrackerEXT>,
    pub locate_hand_joints_ext: Option<xr::pfn::LocateHandJointsEXT>,
}

/// One suggested `(action, binding_path)` entry stored per interaction profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SuggestedBindingEntry {
    pub action: u64,
    pub binding_path: u64,
}

/// Mutable per-instance registries (never held across downstream FFI calls).
#[derive(Default)]
pub struct InstanceInner {
    pub path_to_str: HashMap<u64, String>,
    pub str_to_path: HashMap<String, u64>,
    pub suggested_bindings: HashMap<u64, Vec<SuggestedBindingEntry>>,
    pub pending_events: VecDeque<xr::Session>,
    pub sessions: Vec<u64>,
    pub action_sets: Vec<u64>,
    pub actions: Vec<u64>,
}

impl InstanceInner {
    pub fn record_path(&mut self, path: xr::Path, s: String) {
        let raw = path.into_raw();
        if raw != 0 {
            self.str_to_path.insert(s.clone(), raw);
            self.path_to_str.insert(raw, s);
        }
    }

    pub fn path_str(&self, path: u64) -> Option<&str> {
        self.path_to_str.get(&path).map(String::as_str)
    }
}

/// State associated with one `XrInstance`.
pub struct InstanceState {
    pub handle: xr::Instance,
    pub next: NextDispatch,
    pub config: LayerConfig,
    pub disabled_by_env: bool,
    pub app_api_version: xr::Version,
    pub app_enabled_hand_tracking: bool,
    pub app_enabled_fb_aim: bool,
    pub app_enabled_locate_spaces_khr: bool,
    pub layer_has_hand_tracking: bool,
    pub layer_has_fb_aim: bool,
    pub left_hand_path: xr::Path,
    pub right_hand_path: xr::Path,
    pub inner: RwLock<InstanceInner>,
}

impl InstanceState {
    pub fn emulation_enabled(&self) -> bool {
        !self.disabled_by_env && self.config.enabled && self.layer_has_hand_tracking
    }

    pub fn hand_user_path(&self, hand: HandSide) -> xr::Path {
        match hand {
            HandSide::Left => self.left_hand_path,
            HandSide::Right => self.right_hand_path,
        }
    }

    pub fn path_to_hand(&self, path: xr::Path) -> Option<HandSide> {
        if path == self.left_hand_path && path != xr::Path::NULL {
            Some(HandSide::Left)
        } else if path == self.right_hand_path && path != xr::Path::NULL {
            Some(HandSide::Right)
        } else {
            let inner = self.inner.read();
            inner
                .path_str(path.into_raw())
                .and_then(alvr_hand_core::parse_hand_user_path)
        }
    }
}

/// Metadata tracked for each `XrActionSet`.
pub struct ActionSetMeta {
    pub handle: xr::ActionSet,
    pub instance: Arc<InstanceState>,
    pub priority: u32,
    pub actions: RwLock<Vec<u64>>,
}

/// Metadata tracked for each `XrAction`.
pub struct ActionMeta {
    pub handle: xr::Action,
    pub action_set: u64,
    pub instance: Arc<InstanceState>,
    pub action_type: xr::ActionType,
    pub subaction_paths: Vec<u64>,
}

impl ActionMeta {
    /// Returns `true` if `subaction_path` is `XR_NULL_PATH` or was declared on `xrCreateAction`.
    pub fn supports_subaction(&self, subaction_path: xr::Path) -> bool {
        let raw = subaction_path.into_raw();
        raw == 0 || self.subaction_paths.contains(&raw)
    }

    /// Returns all query keys to snapshot during `xrSyncActions`: always `0` (`XR_NULL_PATH`)
    /// plus each declared subaction path.
    pub fn snapshot_subaction_keys(&self) -> Vec<u64> {
        let mut keys = Vec::with_capacity(1 + self.subaction_paths.len());
        keys.push(0);
        for &p in &self.subaction_paths {
            if p != 0 && !keys.contains(&p) {
                keys.push(p);
            }
        }
        keys
    }
}

/// Classification of an `XrSpace`.
#[derive(Debug, Clone, Copy)]
pub enum SpaceKind {
    Reference {
        reference_type: xr::ReferenceSpaceType,
        pose_in_reference_space: Pose,
    },
    Action {
        action: u64,
        subaction_path: u64,
        pose_in_action_space: Pose,
    },
}

/// Metadata tracked for each `XrSpace`.
pub struct SpaceMeta {
    pub handle: xr::Space,
    pub session: Arc<SessionState>,
    pub kind: SpaceKind,
}

/// Layer-owned OpenXR handles attached to a session (must be destroyed BEFORE `next.destroy_session`).
#[derive(Default)]
pub struct HandTrackerResources {
    pub left_tracker: xr::HandTrackerEXT,
    pub right_tracker: xr::HandTrackerEXT,
    pub local_space: xr::Space,
    pub available: bool,
}

impl HandTrackerResources {
    pub fn tracker(&self, hand: HandSide) -> xr::HandTrackerEXT {
        match hand {
            HandSide::Left => self.left_tracker,
            HandSide::Right => self.right_tracker,
        }
    }
}

/// Per-hand arbitration between physical controller passthrough and hand-tracking emulation.
#[derive(Debug, Clone, Copy, Default)]
pub struct HandArbitrationState {
    pub emulated: bool,
    pub hand_tracked: bool,
    pub last_physical_active_ms: Option<u64>,
    pub effective_profile: u64,
    pub runtime_profile: u64,
}

/// Frozen typed value stored per `(action, subaction_path)` across `xrSyncActions`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SnapshotValue {
    Bool(bool),
    Float(f32),
    Vec2(Vec2),
    Pose,
}

/// Frozen OpenXR action state snapshot returned by `xrGetActionState*`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActionSnapshot {
    pub is_active: bool,
    pub value: SnapshotValue,
    pub changed_since_last_sync: bool,
    pub last_change_time: i64,
}

/// Previous-sync state used to compute exact OpenXR `changedSinceLastSync` transitions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrevActionState {
    pub was_active: bool,
    pub value: SnapshotValue,
    pub last_change_time: i64,
}

/// Monotonic dual-source clock bridging wall-clock `Instant` and runtime `XrTime` (ns).
pub struct MonotonicXrClock {
    wall_start: Instant,
    xr_origin_ns: Option<i64>,
    last_xr_time_ns: i64,
    last_now_ms: u64,
}

impl Default for MonotonicXrClock {
    fn default() -> Self {
        Self::new()
    }
}

impl MonotonicXrClock {
    pub fn new() -> Self {
        Self {
            wall_start: Instant::now(),
            xr_origin_ns: None,
            last_xr_time_ns: 0,
            last_now_ms: 0,
        }
    }

    /// Observes a valid positive runtime `XrTime` (in nanoseconds).
    pub fn observe_xr_time(&mut self, time_ns: i64) {
        if time_ns > 0 {
            if self.xr_origin_ns.is_none() {
                self.xr_origin_ns = Some(time_ns);
            }
            if time_ns > self.last_xr_time_ns {
                self.last_xr_time_ns = time_ns;
            }
        }
    }

    /// Returns `(monotonic_gesture_ms, positive_xr_time_ns)` guaranteed monotonic and `xr_time > 0`.
    pub fn sample(&mut self) -> (u64, i64) {
        let wall_ns = self.wall_start.elapsed().as_nanos().min(i64::MAX as u128) as i64;
        let wall_ms = (wall_ns / 1_000_000) as u64;

        let xr_elapsed_ms = match self.xr_origin_ns {
            Some(origin) => (self.last_xr_time_ns.saturating_sub(origin).max(0) / 1_000_000) as u64,
            None => 0,
        };

        let now_ms = self.last_now_ms.max(wall_ms).max(xr_elapsed_ms);
        self.last_now_ms = now_ms;

        let xr_time_ns = if self.last_xr_time_ns > 0 {
            let origin = self.xr_origin_ns.unwrap_or(self.last_xr_time_ns);
            let wall_projected = origin.saturating_add(wall_ns);
            self.last_xr_time_ns.max(wall_projected)
        } else {
            // Positive synthetic fallback when no runtime XrTime has been observed yet.
            1_000_000_000i64.saturating_add(wall_ns)
        };
        if xr_time_ns > self.last_xr_time_ns {
            self.last_xr_time_ns = xr_time_ns;
        }

        (now_ms, xr_time_ns)
    }
}

/// Mutable per-session state (never held across downstream FFI calls).
pub struct SessionInner {
    pub session_state: xr::SessionState,
    pub is_running: bool,
    pub attached: bool,
    pub attached_action_sets: HashSet<u64>,
    pub active_synced_sets: HashMap<u64, HashSet<u64>>,
    pub last_sync_focused: bool,
    pub hand_arb: [HandArbitrationState; 2],
    pub gesture_engine: HandGestureEngine,
    pub pose_synthesizer: PoseSynthesizer,
    pub last_hand_states: [VirtualHandState; 2],
    pub float_to_bool_hysteresis: HashMap<(u64, u64, usize), HysteresisState>,
    pub action_snapshots: HashMap<(u64, u64), ActionSnapshot>,
    pub prev_action_values: HashMap<(u64, u64), PrevActionState>,
    pub sticky_pose_source: HashMap<(u64, u64), (HandSide, PoseSource)>,
    pub clock: MonotonicXrClock,
    pub spaces: Vec<u64>,
}

impl SessionInner {
    pub fn new() -> Self {
        Self {
            session_state: xr::SessionState::IDLE,
            is_running: false,
            attached: false,
            attached_action_sets: HashSet::new(),
            active_synced_sets: HashMap::new(),
            last_sync_focused: false,
            hand_arb: [HandArbitrationState::default(); 2],
            gesture_engine: HandGestureEngine::new(),
            pose_synthesizer: PoseSynthesizer::new(),
            last_hand_states: [VirtualHandState::default(); 2],
            float_to_bool_hysteresis: HashMap::new(),
            action_snapshots: HashMap::new(),
            prev_action_values: HashMap::new(),
            sticky_pose_source: HashMap::new(),
            clock: MonotonicXrClock::new(),
            spaces: Vec::new(),
        }
    }

    pub fn any_hand_emulated(&self) -> bool {
        self.hand_arb[0].emulated || self.hand_arb[1].emulated
    }

    pub fn is_hand_emulated(&self, hand: HandSide) -> bool {
        self.hand_arb[hand.index()].emulated
    }
}

/// State associated with one `XrSession`.
pub struct SessionState {
    pub handle: xr::Session,
    pub instance: Arc<InstanceState>,
    pub system_id: xr::SystemId,
    pub trackers: Mutex<HandTrackerResources>,
    pub inner: Mutex<SessionInner>,
}

/// Global thread-safe OpenXR handle registries.
pub static INSTANCES: LazyLock<RwLock<HashMap<u64, Arc<InstanceState>>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));
pub static SESSIONS: LazyLock<RwLock<HashMap<u64, Arc<SessionState>>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));
pub static ACTION_SETS: LazyLock<RwLock<HashMap<u64, Arc<ActionSetMeta>>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));
pub static ACTIONS: LazyLock<RwLock<HashMap<u64, Arc<ActionMeta>>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));
pub static SPACES: LazyLock<RwLock<HashMap<u64, Arc<SpaceMeta>>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));

pub fn get_instance(handle: xr::Instance) -> Option<Arc<InstanceState>> {
    INSTANCES.read().get(&handle.into_raw()).cloned()
}

pub fn get_session(handle: xr::Session) -> Option<Arc<SessionState>> {
    SESSIONS.read().get(&handle.into_raw()).cloned()
}

pub fn get_action_set(handle: xr::ActionSet) -> Option<Arc<ActionSetMeta>> {
    ACTION_SETS.read().get(&handle.into_raw()).cloned()
}

pub fn get_action(handle: xr::Action) -> Option<Arc<ActionMeta>> {
    ACTIONS.read().get(&handle.into_raw()).cloned()
}

pub fn get_space(handle: xr::Space) -> Option<Arc<SpaceMeta>> {
    SPACES.read().get(&handle.into_raw()).cloned()
}
