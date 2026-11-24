use crate::math::{is_valid_xr_pose, normalize_safe, pose_inv, pose_mul, rotate_vec_into_base, transform_pose_into_base};
use crate::types::{
    CreateInstanceAttempt, DEFAULT_INITIAL_TIME_NS, FULL_LOCATION_FLAGS, FULL_VELOCITY_FLAGS,
    HAND_JOINT_COUNT, MOCK_RUNTIME_MAGIC, MockDiagnostics, MockEvent, MockHandState,
    MockHapticCall, MockHapticStopCall, MockPhysicalControllerState, MockPose, MockRuntimeConfig,
};
use glam::{Vec2, Vec3};
use openxr_sys as xr;
use openxr_sys::Handle as _;
use parking_lot::Mutex;
use std::collections::{HashMap, HashSet, VecDeque};
use std::ffi::CStr;
use std::os::raw::{c_char, c_void};
use std::time::Duration;

#[derive(Debug, Clone)]
pub(crate) struct InstanceData {
    #[allow(dead_code)]
    pub id: u64,
    pub api_version: xr::Version,
    pub enabled_extensions: HashSet<String>,
    pub path_to_str: HashMap<u64, String>,
    pub str_to_path: HashMap<String, u64>,
    pub next_path_id: u64,
    pub suggested_bindings: HashMap<u64, Vec<(u64, u64)>>, // profile_path -> Vec<(action_id, binding_path)>
    pub suggested_profile_order: Vec<u64>,
    pub event_queue: VecDeque<MockEvent>,
}

impl InstanceData {
    fn new(id: u64, api_version: xr::Version, enabled_extensions: HashSet<String>) -> Self {
        Self {
            id,
            api_version,
            enabled_extensions,
            path_to_str: HashMap::new(),
            str_to_path: HashMap::new(),
            next_path_id: 1,
            suggested_bindings: HashMap::new(),
            suggested_profile_order: Vec::new(),
            event_queue: VecDeque::new(),
        }
    }

    pub fn intern_path(&mut self, s: &str) -> u64 {
        if let Some(&id) = self.str_to_path.get(s) {
            return id;
        }
        let id = self.next_path_id;
        self.next_path_id += 1;
        self.str_to_path.insert(s.to_string(), id);
        self.path_to_str.insert(id, s.to_string());
        id
    }

    pub fn resolve_path(&self, id: u64) -> Option<&str> {
        self.path_to_str.get(&id).map(String::as_str)
    }

    pub fn lookup_path_id(&self, s: &str) -> Option<u64> {
        self.str_to_path.get(s).copied()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ActionSnapshotValue {
    Bool(bool),
    Float(f32),
    Vec2([f32; 2]),
    Pose,
}

#[derive(Debug, Clone, Copy)]
struct ActionSnapshot {
    is_active: bool,
    value: ActionSnapshotValue,
    changed_since_last_sync: bool,
    last_change_time: i64,
}

#[derive(Debug, Clone)]
pub(crate) struct SessionData {
    pub id: u64,
    pub owner_instance: u64,
    pub state: xr::SessionState,
    pub running: bool,
    pub attached_action_sets: HashSet<u64>,
    /// Keyed by (action_id, subaction_path_raw).
    snapshots: HashMap<(u64, u64), ActionSnapshot>,
}

#[derive(Debug, Clone)]
pub(crate) struct ActionSetData {
    pub id: u64,
    pub owner_instance: u64,
    #[allow(dead_code)]
    pub name: String,
    #[allow(dead_code)]
    pub localized_name: String,
    #[allow(dead_code)]
    pub priority: u32,
    pub attached_sessions: HashSet<u64>,
    pub actions: Vec<u64>,
}

#[derive(Debug, Clone)]
pub(crate) struct ActionData {
    pub id: u64,
    pub owner_instance: u64,
    pub owner_action_set: u64,
    #[allow(dead_code)]
    pub name: String,
    #[allow(dead_code)]
    pub localized_name: String,
    pub action_type: xr::ActionType,
    pub subaction_paths: Vec<u64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SpaceKind {
    Reference(xr::ReferenceSpaceType),
    Action {
        action_id: u64,
        subaction_path: u64,
    },
}

#[derive(Debug, Clone)]
pub(crate) struct SpaceData {
    pub id: u64,
    pub owner_session: u64,
    pub owner_instance: u64,
    pub kind: SpaceKind,
    pub pose_in_space: MockPose,
}

#[derive(Debug, Clone)]
pub(crate) struct HandTrackerData {
    pub id: u64,
    pub owner_session: u64,
    pub owner_instance: u64,
    pub hand: xr::HandEXT,
}

#[derive(Debug, Clone)]
pub(crate) struct SwapchainData {
    #[allow(dead_code)]
    pub id: u64,
    pub owner_session: u64,
}

struct RuntimeInner {
    config: MockRuntimeConfig,
    hands: [MockHandState; 2],
    physical: [MockPhysicalControllerState; 2],
    current_time_ns: i64,
    next_handle: u64,
    instances: HashMap<u64, InstanceData>,
    sessions: HashMap<u64, SessionData>,
    action_sets: HashMap<u64, ActionSetData>,
    actions: HashMap<u64, ActionData>,
    spaces: HashMap<u64, SpaceData>,
    hand_trackers: HashMap<u64, HandTrackerData>,
    swapchains: HashMap<u64, SwapchainData>,
    pending_global_events: VecDeque<MockEvent>,
    diagnostics: MockDiagnostics,
}

impl RuntimeInner {
    fn new() -> Self {
        let mut diag = MockDiagnostics::default();
        diag.runtime_magic = MOCK_RUNTIME_MAGIC;
        diag.current_time_ns = DEFAULT_INITIAL_TIME_NS;
        Self {
            config: MockRuntimeConfig::default(),
            hands: [MockHandState::default(), MockHandState::default()],
            physical: [
                MockPhysicalControllerState::default(),
                MockPhysicalControllerState::default(),
            ],
            current_time_ns: DEFAULT_INITIAL_TIME_NS,
            next_handle: 0x1000,
            instances: HashMap::new(),
            sessions: HashMap::new(),
            action_sets: HashMap::new(),
            actions: HashMap::new(),
            spaces: HashMap::new(),
            hand_trackers: HashMap::new(),
            swapchains: HashMap::new(),
            pending_global_events: VecDeque::new(),
            diagnostics: diag,
        }
    }

    fn alloc_handle(&mut self) -> u64 {
        let h = self.next_handle;
        self.next_handle += 1;
        h
    }
}

pub struct MockRuntimeEngine {
    inner: Mutex<RuntimeInner>,
}

impl Default for MockRuntimeEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl MockRuntimeEngine {
    pub fn new() -> Self {
        Self {
            inner: Mutex::new(RuntimeInner::new()),
        }
    }

    pub fn reset(&self) {
        let mut g = self.inner.lock();
        *g = RuntimeInner::new();
    }

    pub fn configure(&self, cfg: MockRuntimeConfig) {
        let mut g = self.inner.lock();
        g.config = cfg;
    }

    pub fn config(&self) -> MockRuntimeConfig {
        self.inner.lock().config.clone()
    }

    pub fn set_hand_state(&self, hand: usize, mut state: MockHandState) {
        if hand > 1 {
            return;
        }
        if state.joints.len() < HAND_JOINT_COUNT {
            state
                .joints
                .resize(HAND_JOINT_COUNT, Default::default());
        } else if state.joints.len() > HAND_JOINT_COUNT {
            state.joints.truncate(HAND_JOINT_COUNT);
        }
        self.inner.lock().hands[hand] = state;
    }

    pub fn set_physical_controller(&self, hand: usize, state: MockPhysicalControllerState) {
        if hand > 1 {
            return;
        }
        self.inner.lock().physical[hand] = state;
    }

    pub fn push_event(&self, event: MockEvent) {
        let mut g = self.inner.lock();
        if let MockEvent::SessionStateChanged { session, state, .. } = &event {
            let target_sid = if *session != 0 {
                Some(*session)
            } else {
                g.sessions.keys().copied().max()
            };
            if let Some(sid) = target_sid {
                if let Some(s) = g.sessions.get_mut(&sid) {
                    s.state = xr::SessionState::from_raw(*state);
                    let iid = s.owner_instance;
                    if let Some(inst) = g.instances.get_mut(&iid) {
                        inst.event_queue.push_back(event);
                        return;
                    }
                }
            }
        } else if let MockEvent::InteractionProfileChanged { session } = &event {
            let target_sid = if *session != 0 {
                Some(*session)
            } else {
                g.sessions.keys().copied().max()
            };
            if let Some(sid) = target_sid {
                if let Some(s) = g.sessions.get(&sid) {
                    let iid = s.owner_instance;
                    if let Some(inst) = g.instances.get_mut(&iid) {
                        inst.event_queue.push_back(event);
                        return;
                    }
                }
            }
        }
        if let Some(inst) = g.instances.values_mut().next() {
            inst.event_queue.push_back(event);
        } else {
            g.pending_global_events.push_back(event);
        }
    }

    pub fn set_session_state(&self, state: xr::SessionState) {
        let mut g = self.inner.lock();
        for s in g.sessions.values_mut() {
            s.state = state;
        }
    }

    pub fn set_session_state_for(&self, session: xr::Session, state: xr::SessionState) {
        let mut g = self.inner.lock();
        if let Some(s) = g.sessions.get_mut(&session.into_raw()) {
            s.state = state;
        }
    }

    pub fn advance_time_ns(&self, delta_ns: i64) -> i64 {
        let mut g = self.inner.lock();
        g.current_time_ns = g.current_time_ns.saturating_add(delta_ns);
        g.diagnostics.current_time_ns = g.current_time_ns;
        g.current_time_ns
    }

    pub fn current_time_ns(&self) -> i64 {
        self.inner.lock().current_time_ns
    }

    pub fn diagnostics(&self) -> MockDiagnostics {
        let g = self.inner.lock();
        let mut d = g.diagnostics.clone();
        d.runtime_magic = MOCK_RUNTIME_MAGIC;
        d.current_time_ns = g.current_time_ns;
        d.live_instances = g.instances.len();
        d.live_sessions = g.sessions.len();
        d.live_hand_trackers = g.hand_trackers.len();
        d.live_spaces = g.spaces.len();
        d
    }

    // ---- Negotiation ----

    pub fn record_negotiate(&self) -> xr::Version {
        let mut g = self.inner.lock();
        g.diagnostics.negotiate_calls += 1;
        let [maj, min, pat] = g.config.api_version;
        xr::Version::new(maj as u16, min as u16, pat)
    }

    // ---- Pre-Instance & Instance ----

    pub fn enumerate_instance_extension_properties(
        &self,
        layer_name: Option<&str>,
    ) -> Result<Vec<(String, u32)>, xr::Result> {
        if let Some(name) = layer_name {
            if !name.is_empty() {
                return Err(xr::Result::ERROR_API_LAYER_NOT_PRESENT);
            }
        }
        let g = self.inner.lock();
        Ok(g.config
            .supported_extensions
            .iter()
            .map(|e| (e.name.clone(), e.version))
            .collect())
    }

    pub unsafe fn create_instance(
        &self,
        create_info: *const xr::InstanceCreateInfo,
        instance_out: *mut xr::Instance,
    ) -> xr::Result {
        if create_info.is_null() || instance_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*create_info };
        if info.ty != xr::StructureType::INSTANCE_CREATE_INFO {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        let mut enabled_exts = Vec::new();
        if info.enabled_extension_count > 0 {
            if info.enabled_extension_names.is_null() {
                return xr::Result::ERROR_VALIDATION_FAILURE;
            }
            for i in 0..info.enabled_extension_count as usize {
                let p = unsafe { *info.enabled_extension_names.add(i) };
                if p.is_null() {
                    return xr::Result::ERROR_VALIDATION_FAILURE;
                }
                let s = unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned();
                enabled_exts.push(s);
            }
        }

        let mut enabled_layers = Vec::new();
        if info.enabled_api_layer_count > 0 && !info.enabled_api_layer_names.is_null() {
            for i in 0..info.enabled_api_layer_count as usize {
                let p = unsafe { *info.enabled_api_layer_names.add(i) };
                if !p.is_null() {
                    enabled_layers.push(unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned());
                }
            }
        }

        let api_v = info.application_info.api_version;
        let effective_ver = if api_v.into_raw() == 0 {
            let g = self.inner.lock();
            let [maj, min, pat] = g.config.api_version;
            xr::Version::new(maj as u16, min as u16, pat)
        } else {
            api_v
        };

        let mut g = self.inner.lock();
        if g.config.create_instance_error != 0 {
            let res = xr::Result::from_raw(g.config.create_instance_error);
            g.diagnostics.create_instance_attempts.push(CreateInstanceAttempt {
                api_version: [
                    effective_ver.major() as u32,
                    effective_ver.minor() as u32,
                    effective_ver.patch(),
                ],
                enabled_extensions: enabled_exts,
                enabled_api_layers: enabled_layers,
                result: res.into_raw(),
            });
            return res;
        }

        let supported_set: HashSet<&str> = g
            .config
            .supported_extensions
            .iter()
            .map(|e| e.name.as_str())
            .collect();
        for ext in &enabled_exts {
            if !supported_set.contains(ext.as_str()) {
                g.diagnostics.create_instance_attempts.push(CreateInstanceAttempt {
                    api_version: [
                        effective_ver.major() as u32,
                        effective_ver.minor() as u32,
                        effective_ver.patch(),
                    ],
                    enabled_extensions: enabled_exts,
                    enabled_api_layers: enabled_layers,
                    result: xr::Result::ERROR_EXTENSION_NOT_PRESENT.into_raw(),
                });
                return xr::Result::ERROR_EXTENSION_NOT_PRESENT;
            }
        }

        let id = g.alloc_handle();
        let ext_set: HashSet<String> = enabled_exts.iter().cloned().collect();
        let mut inst = InstanceData::new(id, effective_ver, ext_set);
        while let Some(ev) = g.pending_global_events.pop_front() {
            inst.event_queue.push_back(ev);
        }
        g.instances.insert(id, inst);
        g.diagnostics.create_instance_attempts.push(CreateInstanceAttempt {
            api_version: [
                effective_ver.major() as u32,
                effective_ver.minor() as u32,
                effective_ver.patch(),
            ],
            enabled_extensions: enabled_exts,
            enabled_api_layers: enabled_layers,
            result: xr::Result::SUCCESS.into_raw(),
        });

        unsafe {
            *instance_out = xr::Instance::from_raw(id);
        }
        xr::Result::SUCCESS
    }

    pub fn destroy_instance(&self, instance: xr::Instance) -> xr::Result {
        let id = instance.into_raw();
        let mut g = self.inner.lock();
        if !g.instances.contains_key(&id) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }

        let leaked_sessions: Vec<u64> = g
            .sessions
            .values()
            .filter(|s| s.owner_instance == id)
            .map(|s| s.id)
            .collect();
        g.diagnostics.leaked_sessions_on_destroy_instance += leaked_sessions.len() as u64;

        let leaked_action_sets: Vec<u64> = g
            .action_sets
            .values()
            .filter(|a| a.owner_instance == id)
            .map(|a| a.id)
            .collect();
        g.diagnostics.leaked_action_sets_on_destroy_instance += leaked_action_sets.len() as u64;

        g.diagnostics
            .destroy_order_log
            .push(format!("xrDestroyInstance(instance={id})"));

        // Cleanup leaked children belonging to this instance
        for sid in leaked_sessions {
            g.sessions.remove(&sid);
        }
        for asid in leaked_action_sets {
            g.action_sets.remove(&asid);
        }
        g.actions.retain(|_, a| a.owner_instance != id);
        g.spaces.retain(|_, s| s.owner_instance != id);
        g.hand_trackers.retain(|_, h| h.owner_instance != id);
        let live_sessions: HashSet<u64> = g.sessions.keys().copied().collect();
        g.swapchains.retain(|_, sc| live_sessions.contains(&sc.owner_session));
        g.instances.remove(&id);
        xr::Result::SUCCESS
    }

    pub fn instance_has_extension(&self, instance: xr::Instance, ext: &str) -> Result<bool, xr::Result> {
        let g = self.inner.lock();
        let inst = g
            .instances
            .get(&instance.into_raw())
            .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;
        Ok(inst.enabled_extensions.contains(ext))
    }

    pub fn instance_api_version(&self, instance: xr::Instance) -> Result<xr::Version, xr::Result> {
        let g = self.inner.lock();
        let inst = g
            .instances
            .get(&instance.into_raw())
            .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;
        Ok(inst.api_version)
    }

    pub fn instance_exists(&self, instance: xr::Instance) -> bool {
        self.inner.lock().instances.contains_key(&instance.into_raw())
    }

    pub unsafe fn get_instance_properties(
        &self,
        instance: xr::Instance,
        props: *mut xr::InstanceProperties,
    ) -> xr::Result {
        if props.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let g = self.inner.lock();
        if !g.instances.contains_key(&instance.into_raw()) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }
        let p = unsafe { &mut *props };
        let [maj, min, pat] = g.config.api_version;
        p.runtime_version = xr::Version::new(maj as u16, min as u16, pat);
        write_cstr_buf(&mut p.runtime_name, "ALVR_E2E_Mock_OpenXR_Runtime");
        xr::Result::SUCCESS
    }

    pub unsafe fn poll_event(
        &self,
        instance: xr::Instance,
        event_data: *mut xr::EventDataBuffer,
    ) -> xr::Result {
        if event_data.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let delay_ms = {
            let mut g = self.inner.lock();
            if !g.instances.contains_key(&instance.into_raw()) {
                return xr::Result::ERROR_HANDLE_INVALID;
            }
            g.diagnostics.poll_event_calls += 1;
            g.config.poll_event_delay_ms
        };
        if delay_ms > 0 {
            std::thread::sleep(Duration::from_millis(delay_ms));
        }

        let mut g = self.inner.lock();
        let current_time_ns = g.current_time_ns;
        let latest_session = g
            .sessions
            .values()
            .filter(|s| s.owner_instance == instance.into_raw())
            .map(|s| s.id)
            .max()
            .unwrap_or(0);
        let Some(inst) = g.instances.get_mut(&instance.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let Some(ev) = inst.event_queue.pop_front() else {
            return xr::Result::EVENT_UNAVAILABLE;
        };

        match ev {
            MockEvent::SessionStateChanged {
                session,
                state,
                time_ns,
            } => {
                let sid = if session != 0 { session } else { latest_session };
                let out = unsafe { &mut *(event_data as *mut xr::EventDataSessionStateChanged) };
                out.ty = xr::StructureType::EVENT_DATA_SESSION_STATE_CHANGED;
                out.next = std::ptr::null();
                out.session = xr::Session::from_raw(sid);
                out.state = xr::SessionState::from_raw(state);
                out.time = xr::Time::from_nanos(if time_ns > 0 { time_ns } else { current_time_ns });
            }
            MockEvent::InteractionProfileChanged { session } => {
                let sid = if session != 0 { session } else { latest_session };
                let out =
                    unsafe { &mut *(event_data as *mut xr::EventDataInteractionProfileChanged) };
                out.ty = xr::StructureType::EVENT_DATA_INTERACTION_PROFILE_CHANGED;
                out.next = std::ptr::null();
                out.session = xr::Session::from_raw(sid);
            }
        }
        xr::Result::SUCCESS
    }

    // ---- System ----

    pub unsafe fn get_system(
        &self,
        instance: xr::Instance,
        get_info: *const xr::SystemGetInfo,
        system_id: *mut xr::SystemId,
    ) -> xr::Result {
        if get_info.is_null() || system_id.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let g = self.inner.lock();
        if !g.instances.contains_key(&instance.into_raw()) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }
        let info = unsafe { &*get_info };
        if info.form_factor != xr::FormFactor::HEAD_MOUNTED_DISPLAY {
            return xr::Result::ERROR_FORM_FACTOR_UNSUPPORTED;
        }
        unsafe {
            *system_id = xr::SystemId::from_raw(1);
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn get_system_properties(
        &self,
        instance: xr::Instance,
        system_id: xr::SystemId,
        properties: *mut xr::SystemProperties,
    ) -> xr::Result {
        if properties.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        if system_id != xr::SystemId::from_raw(1) {
            return xr::Result::ERROR_SYSTEM_INVALID;
        }
        let mut g = self.inner.lock();
        if !g.instances.contains_key(&instance.into_raw()) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }
        let supports_ht = g.config.supports_hand_tracking;

        let props = unsafe { &mut *properties };
        props.system_id = system_id;
        props.vendor_id = 0x2833; // Meta vendor ID
        write_cstr_buf(&mut props.system_name, "ALVR_E2E_Mock_Quest_HMD");
        props.graphics_properties = xr::SystemGraphicsProperties {
            max_swapchain_image_height: 4096,
            max_swapchain_image_width: 4096,
            max_layer_count: 16,
        };
        props.tracking_properties = xr::SystemTrackingProperties {
            orientation_tracking: xr::TRUE,
            position_tracking: xr::TRUE,
        };

        let mut next = props.next;
        while !next.is_null() {
            let header = unsafe { &mut *(next as *mut xr::BaseOutStructure) };
            if header.ty == xr::StructureType::SYSTEM_HAND_TRACKING_PROPERTIES_EXT {
                let ht = unsafe { &mut *(next as *mut xr::SystemHandTrackingPropertiesEXT) };
                ht.supports_hand_tracking = xr::Bool32::from(supports_ht);
                g.diagnostics.get_system_properties_hand_tracking_queried += 1;
            }
            next = header.next as *mut c_void;
        }

        xr::Result::SUCCESS
    }

    // ---- Session & Frame ----

    pub unsafe fn create_session(
        &self,
        instance: xr::Instance,
        create_info: *const xr::SessionCreateInfo,
        session_out: *mut xr::Session,
    ) -> xr::Result {
        if create_info.is_null() || session_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*create_info };
        if info.system_id != xr::SystemId::from_raw(1) {
            return xr::Result::ERROR_SYSTEM_INVALID;
        }
        let mut g = self.inner.lock();
        if !g.instances.contains_key(&instance.into_raw()) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }
        let sid = g.alloc_handle();
        let init_state = xr::SessionState::from_raw(g.config.initial_session_state);
        let running = matches!(
            init_state,
            xr::SessionState::SYNCHRONIZED
                | xr::SessionState::VISIBLE
                | xr::SessionState::FOCUSED
        );
        g.sessions.insert(
            sid,
            SessionData {
                id: sid,
                owner_instance: instance.into_raw(),
                state: init_state,
                running,
                attached_action_sets: HashSet::new(),
                snapshots: HashMap::new(),
            },
        );
        unsafe {
            *session_out = xr::Session::from_raw(sid);
        }
        xr::Result::SUCCESS
    }

    pub fn destroy_session(&self, session: xr::Session) -> xr::Result {
        let sid = session.into_raw();
        let mut g = self.inner.lock();
        if !g.sessions.contains_key(&sid) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }

        let leaked_trackers: Vec<u64> = g
            .hand_trackers
            .values()
            .filter(|t| t.owner_session == sid)
            .map(|t| t.id)
            .collect();
        let leaked_spaces: Vec<u64> = g
            .spaces
            .values()
            .filter(|s| s.owner_session == sid)
            .map(|s| s.id)
            .collect();

        g.diagnostics.leaked_hand_trackers_on_destroy_session += leaked_trackers.len() as u64;
        g.diagnostics.leaked_spaces_on_destroy_session += leaked_spaces.len() as u64;
        g.diagnostics
            .destroy_order_log
            .push(format!("xrDestroySession(session={sid})"));

        for tid in leaked_trackers {
            g.hand_trackers.remove(&tid);
        }
        for spid in leaked_spaces {
            g.spaces.remove(&spid);
        }
        g.swapchains.retain(|_, sc| sc.owner_session != sid);
        for aset in g.action_sets.values_mut() {
            aset.attached_sessions.remove(&sid);
        }
        g.sessions.remove(&sid);
        xr::Result::SUCCESS
    }

    pub fn begin_session(&self, session: xr::Session) -> xr::Result {
        let mut g = self.inner.lock();
        let Some(s) = g.sessions.get_mut(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        s.running = true;
        if s.state == xr::SessionState::IDLE || s.state == xr::SessionState::READY {
            s.state = xr::SessionState::FOCUSED;
        }
        xr::Result::SUCCESS
    }

    pub fn end_session(&self, session: xr::Session) -> xr::Result {
        let mut g = self.inner.lock();
        let Some(s) = g.sessions.get_mut(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        s.running = false;
        s.state = xr::SessionState::IDLE;
        xr::Result::SUCCESS
    }

    pub fn request_exit_session(&self, session: xr::Session) -> xr::Result {
        let mut g = self.inner.lock();
        let Some(s) = g.sessions.get_mut(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let iid = s.owner_instance;
        let sid = s.id;
        let t = g.current_time_ns;
        if let Some(inst) = g.instances.get_mut(&iid) {
            inst.event_queue.push_back(MockEvent::SessionStateChanged {
                session: sid,
                state: xr::SessionState::STOPPING.into_raw(),
                time_ns: t,
            });
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn wait_frame(
        &self,
        session: xr::Session,
        frame_state: *mut xr::FrameState,
    ) -> xr::Result {
        if frame_state.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let g = self.inner.lock();
        if !g.sessions.contains_key(&session.into_raw()) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }
        let fs = unsafe { &mut *frame_state };
        fs.predicted_display_time = xr::Time::from_nanos(g.current_time_ns.max(1));
        fs.predicted_display_period = xr::Duration::from_nanos(11_111_111);
        fs.should_render = xr::TRUE;
        xr::Result::SUCCESS
    }

    pub fn session_exists(&self, session: xr::Session) -> bool {
        self.inner.lock().sessions.contains_key(&session.into_raw())
    }

    // ---- Swapchains ----

    pub unsafe fn create_swapchain(
        &self,
        session: xr::Session,
        swapchain_out: *mut xr::Swapchain,
    ) -> xr::Result {
        if swapchain_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let mut g = self.inner.lock();
        if !g.sessions.contains_key(&session.into_raw()) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }
        let id = g.alloc_handle();
        g.swapchains.insert(
            id,
            SwapchainData {
                id,
                owner_session: session.into_raw(),
            },
        );
        unsafe {
            *swapchain_out = xr::Swapchain::from_raw(id);
        }
        xr::Result::SUCCESS
    }

    pub fn destroy_swapchain(&self, swapchain: xr::Swapchain) -> xr::Result {
        let mut g = self.inner.lock();
        if g.swapchains.remove(&swapchain.into_raw()).is_some() {
            xr::Result::SUCCESS
        } else {
            xr::Result::ERROR_HANDLE_INVALID
        }
    }

    pub fn swapchain_exists(&self, swapchain: xr::Swapchain) -> bool {
        self.inner
            .lock()
            .swapchains
            .contains_key(&swapchain.into_raw())
    }

    // ---- Paths ----

    pub unsafe fn string_to_path(
        &self,
        instance: xr::Instance,
        path_string: *const c_char,
        path_out: *mut xr::Path,
    ) -> xr::Result {
        if path_string.is_null() || path_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let Ok(s) = (unsafe { CStr::from_ptr(path_string) }).to_str() else {
            return xr::Result::ERROR_PATH_FORMAT_INVALID;
        };
        if !is_valid_openxr_path(s) {
            return xr::Result::ERROR_PATH_FORMAT_INVALID;
        }
        let mut g = self.inner.lock();
        let Some(inst) = g.instances.get_mut(&instance.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let id = inst.intern_path(s);
        unsafe {
            *path_out = xr::Path::from_raw(id);
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn path_to_string(
        &self,
        instance: xr::Instance,
        path: xr::Path,
        buffer_capacity_input: u32,
        buffer_count_output: *mut u32,
        buffer: *mut c_char,
    ) -> xr::Result {
        if buffer_count_output.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let g = self.inner.lock();
        let Some(inst) = g.instances.get(&instance.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if path == xr::Path::NULL {
            return xr::Result::ERROR_PATH_INVALID;
        }
        let Some(s) = inst.resolve_path(path.into_raw()) else {
            return xr::Result::ERROR_PATH_INVALID;
        };
        let needed = (s.len() + 1) as u32;
        unsafe {
            *buffer_count_output = needed;
        }
        if buffer_capacity_input == 0 {
            return xr::Result::SUCCESS;
        }
        if buffer_capacity_input < needed {
            return xr::Result::ERROR_SIZE_INSUFFICIENT;
        }
        if buffer.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        unsafe {
            std::ptr::copy_nonoverlapping(s.as_ptr(), buffer as *mut u8, s.len());
            *buffer.add(s.len()) = 0;
        }
        xr::Result::SUCCESS
    }

    // ---- Actions & Bindings ----

    pub unsafe fn create_action_set(
        &self,
        instance: xr::Instance,
        create_info: *const xr::ActionSetCreateInfo,
        action_set_out: *mut xr::ActionSet,
    ) -> xr::Result {
        if create_info.is_null() || action_set_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*create_info };
        let name = read_cstr_arr(&info.action_set_name);
        let loc = read_cstr_arr(&info.localized_action_set_name);
        if name.is_empty() || loc.is_empty() {
            return xr::Result::ERROR_NAME_INVALID;
        }
        let mut g = self.inner.lock();
        if !g.instances.contains_key(&instance.into_raw()) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }
        let id = g.alloc_handle();
        g.action_sets.insert(
            id,
            ActionSetData {
                id,
                owner_instance: instance.into_raw(),
                name,
                localized_name: loc,
                priority: info.priority,
                attached_sessions: HashSet::new(),
                actions: Vec::new(),
            },
        );
        unsafe {
            *action_set_out = xr::ActionSet::from_raw(id);
        }
        xr::Result::SUCCESS
    }

    pub fn destroy_action_set(&self, action_set: xr::ActionSet) -> xr::Result {
        let id = action_set.into_raw();
        let mut g = self.inner.lock();
        let Some(aset) = g.action_sets.remove(&id) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        g.diagnostics
            .destroy_order_log
            .push(format!("xrDestroyActionSet(action_set={id})"));
        for aid in aset.actions {
            g.actions.remove(&aid);
        }
        for s in g.sessions.values_mut() {
            s.attached_action_sets.remove(&id);
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn create_action(
        &self,
        action_set: xr::ActionSet,
        create_info: *const xr::ActionCreateInfo,
        action_out: *mut xr::Action,
    ) -> xr::Result {
        if create_info.is_null() || action_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*create_info };
        let name = read_cstr_arr(&info.action_name);
        let loc = read_cstr_arr(&info.localized_action_name);
        if name.is_empty() || loc.is_empty() {
            return xr::Result::ERROR_NAME_INVALID;
        }
        let mut subaction_paths = Vec::new();
        if info.count_subaction_paths > 0 {
            if info.subaction_paths.is_null() {
                return xr::Result::ERROR_VALIDATION_FAILURE;
            }
            for i in 0..info.count_subaction_paths as usize {
                subaction_paths.push((unsafe { *info.subaction_paths.add(i) }).into_raw());
            }
        }

        let mut g = self.inner.lock();
        let Some(aset) = g.action_sets.get(&action_set.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if !aset.attached_sessions.is_empty() {
            return xr::Result::ERROR_ACTIONSETS_ALREADY_ATTACHED;
        }
        let owner_instance = aset.owner_instance;
        let id = g.alloc_handle();
        g.actions.insert(
            id,
            ActionData {
                id,
                owner_instance,
                owner_action_set: action_set.into_raw(),
                name,
                localized_name: loc,
                action_type: info.action_type,
                subaction_paths,
            },
        );
        if let Some(aset_mut) = g.action_sets.get_mut(&action_set.into_raw()) {
            aset_mut.actions.push(id);
        }
        unsafe {
            *action_out = xr::Action::from_raw(id);
        }
        xr::Result::SUCCESS
    }

    pub fn destroy_action(&self, action: xr::Action) -> xr::Result {
        let id = action.into_raw();
        let mut g = self.inner.lock();
        let Some(act) = g.actions.remove(&id) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        g.diagnostics
            .destroy_order_log
            .push(format!("xrDestroyAction(action={id})"));
        if let Some(aset) = g.action_sets.get_mut(&act.owner_action_set) {
            aset.actions.retain(|&x| x != id);
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn suggest_interaction_profile_bindings(
        &self,
        instance: xr::Instance,
        suggested_bindings: *const xr::InteractionProfileSuggestedBinding,
    ) -> xr::Result {
        if suggested_bindings.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*suggested_bindings };
        if info.interaction_profile == xr::Path::NULL {
            return xr::Result::ERROR_PATH_INVALID;
        }
        if info.count_suggested_bindings > 0 && info.suggested_bindings.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        let mut pairs = Vec::with_capacity(info.count_suggested_bindings as usize);
        for i in 0..info.count_suggested_bindings as usize {
            let b = unsafe { *info.suggested_bindings.add(i) };
            pairs.push((b.action.into_raw(), b.binding.into_raw()));
        }

        let mut g = self.inner.lock();
        if !g.instances.contains_key(&instance.into_raw()) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }
        // Check immutability: none of the referenced action sets may already be attached
        for &(aid, bpath) in &pairs {
            let Some(act) = g.actions.get(&aid) else {
                return xr::Result::ERROR_HANDLE_INVALID;
            };
            if act.owner_instance != instance.into_raw() {
                return xr::Result::ERROR_HANDLE_INVALID;
            }
            if let Some(aset) = g.action_sets.get(&act.owner_action_set) {
                if !aset.attached_sessions.is_empty() {
                    return xr::Result::ERROR_ACTIONSETS_ALREADY_ATTACHED;
                }
            }
            let Some(inst) = g.instances.get(&instance.into_raw()) else {
                return xr::Result::ERROR_HANDLE_INVALID;
            };
            if bpath == 0 || inst.resolve_path(bpath).is_none() {
                return xr::Result::ERROR_PATH_INVALID;
            }
        }

        let Some(inst) = g.instances.get_mut(&instance.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let profile_id = info.interaction_profile.into_raw();
        if inst.resolve_path(profile_id).is_none() {
            return xr::Result::ERROR_PATH_INVALID;
        }
        if !inst.suggested_profile_order.contains(&profile_id) {
            inst.suggested_profile_order.push(profile_id);
        }
        // OpenXR spec replace-on-repeat semantics:
        inst.suggested_bindings.insert(profile_id, pairs);
        xr::Result::SUCCESS
    }

    pub unsafe fn attach_session_action_sets(
        &self,
        session: xr::Session,
        attach_info: *const xr::SessionActionSetsAttachInfo,
    ) -> xr::Result {
        if attach_info.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*attach_info };
        if info.count_action_sets == 0 || info.action_sets.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let mut set_ids = Vec::with_capacity(info.count_action_sets as usize);
        for i in 0..info.count_action_sets as usize {
            set_ids.push((unsafe { *info.action_sets.add(i) }).into_raw());
        }

        let mut g = self.inner.lock();
        let Some(s) = g.sessions.get(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if !s.attached_action_sets.is_empty() {
            return xr::Result::ERROR_ACTIONSETS_ALREADY_ATTACHED;
        }
        let owner_instance = s.owner_instance;
        for &asid in &set_ids {
            let Some(aset) = g.action_sets.get(&asid) else {
                return xr::Result::ERROR_HANDLE_INVALID;
            };
            if aset.owner_instance != owner_instance {
                return xr::Result::ERROR_HANDLE_INVALID;
            }
        }
        for &asid in &set_ids {
            if let Some(aset) = g.action_sets.get_mut(&asid) {
                aset.attached_sessions.insert(session.into_raw());
            }
        }
        if let Some(s_mut) = g.sessions.get_mut(&session.into_raw()) {
            for asid in set_ids {
                s_mut.attached_action_sets.insert(asid);
            }
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn get_current_interaction_profile(
        &self,
        session: xr::Session,
        top_level_user_path: xr::Path,
        state_out: *mut xr::InteractionProfileState,
    ) -> xr::Result {
        if state_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let g = self.inner.lock();
        let Some(s) = g.sessions.get(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if s.attached_action_sets.is_empty() {
            return xr::Result::ERROR_ACTIONSET_NOT_ATTACHED;
        }
        let Some(inst) = g.instances.get(&s.owner_instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let Some(user_str) = inst.resolve_path(top_level_user_path.into_raw()) else {
            return xr::Result::ERROR_PATH_INVALID;
        };
        let hand_idx = match user_str {
            "/user/hand/left" => Some(0usize),
            "/user/hand/right" => Some(1usize),
            _ => None,
        };
        let profile_path = hand_idx
            .and_then(|hi| active_profile_for_hand(&g, inst, hi))
            .map(xr::Path::from_raw)
            .unwrap_or(xr::Path::NULL);

        unsafe {
            (*state_out).interaction_profile = profile_path;
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn sync_actions(
        &self,
        session: xr::Session,
        sync_info: *const xr::ActionsSyncInfo,
    ) -> xr::Result {
        if sync_info.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*sync_info };
        let mut active_sets = Vec::with_capacity(info.count_active_action_sets as usize);
        if info.count_active_action_sets > 0 {
            if info.active_action_sets.is_null() {
                return xr::Result::ERROR_VALIDATION_FAILURE;
            }
            for i in 0..info.count_active_action_sets as usize {
                let a = unsafe { *info.active_action_sets.add(i) };
                active_sets.push((a.action_set.into_raw(), a.subaction_path.into_raw()));
            }
        }

        let mut g = self.inner.lock();
        g.diagnostics.sync_actions_calls += 1;

        let Some(s) = g.sessions.get(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        for &(asid, _) in &active_sets {
            if !s.attached_action_sets.contains(&asid) {
                return xr::Result::ERROR_ACTIONSET_NOT_ATTACHED;
            }
        }

        let owner_instance = s.owner_instance;
        let state = s.state;
        let attached_sets: Vec<u64> = s.attached_action_sets.iter().copied().collect();
        let prev_snapshots = s.snapshots.clone();
        let now_ns = g.current_time_ns;

        if state != xr::SessionState::FOCUSED {
            if let Some(s_mut) = g.sessions.get_mut(&session.into_raw()) {
                s_mut.snapshots.clear();
            }
            return xr::Result::SESSION_NOT_FOCUSED;
        }

        let Some(inst) = g.instances.get(&owner_instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let left_path_id = inst.lookup_path_id("/user/hand/left");
        let right_path_id = inst.lookup_path_id("/user/hand/right");
        let active_profiles = [
            active_profile_for_hand(&g, inst, 0),
            active_profile_for_hand(&g, inst, 1),
        ];

        // Collect all actions in attached action sets
        let mut action_ids = Vec::new();
        for asid in &attached_sets {
            if let Some(aset) = g.action_sets.get(asid) {
                action_ids.extend_from_slice(&aset.actions);
            }
        }

        let mut new_snapshots = HashMap::new();
        for aid in action_ids {
            let Some(act) = g.actions.get(&aid) else {
                continue;
            };
            let mut query_subactions = vec![0u64];
            for &sp in &act.subaction_paths {
                if !query_subactions.contains(&sp) {
                    query_subactions.push(sp);
                }
            }

            for query_sp in query_subactions {
                // Determine if (action_set, query_sp) is enabled by active_sets
                let set_active_subactions: Vec<u64> = active_sets
                    .iter()
                    .filter(|&&(asid, _)| asid == act.owner_action_set)
                    .map(|&(_, sp)| sp)
                    .collect();

                if set_active_subactions.is_empty() {
                    continue;
                }

                // Which hands are allowed by query_sp AND set_active_subactions?
                let mut hands_allowed = [false, false];
                for hand_idx in 0..2usize {
                    let hand_pid = if hand_idx == 0 {
                        left_path_id
                    } else {
                        right_path_id
                    };
                    let query_allows = if query_sp == 0 {
                        true
                    } else {
                        hand_pid == Some(query_sp)
                    };
                    let sync_allows = set_active_subactions.iter().any(|&asp| {
                        asp == 0 || hand_pid == Some(asp)
                    });
                    hands_allowed[hand_idx] = query_allows && sync_allows && g.physical[hand_idx].active;
                }

                // Gather all matching binding paths for this action across active hands
                let mut matched_paths: Vec<(usize, &str)> = Vec::new();
                for hand_idx in 0..2usize {
                    if !hands_allowed[hand_idx] {
                        continue;
                    }
                    let Some(prof_id) = active_profiles[hand_idx] else {
                        continue;
                    };
                    let prefix = if hand_idx == 0 {
                        "/user/hand/left/"
                    } else {
                        "/user/hand/right/"
                    };
                    if let Some(bindings) = inst.suggested_bindings.get(&prof_id) {
                        for &(baid, bpath_id) in bindings {
                            if baid == aid {
                                if let Some(bstr) = inst.resolve_path(bpath_id) {
                                    if bstr.starts_with(prefix) {
                                        matched_paths.push((hand_idx, bstr));
                                    }
                                }
                            }
                        }
                    }
                }

                if matched_paths.is_empty() {
                    continue;
                }

                let val = evaluate_physical_action(&g.physical, act.action_type, &matched_paths);
                let prev = prev_snapshots.get(&(aid, query_sp));
                let (changed, last_change_time) = match prev {
                    Some(p) if p.is_active => {
                        if p.value != val {
                            (true, now_ns)
                        } else {
                            (false, p.last_change_time)
                        }
                    }
                    _ => (false, now_ns),
                };
                new_snapshots.insert(
                    (aid, query_sp),
                    ActionSnapshot {
                        is_active: true,
                        value: val,
                        changed_since_last_sync: changed,
                        last_change_time,
                    },
                );
            }
        }

        if let Some(s_mut) = g.sessions.get_mut(&session.into_raw()) {
            s_mut.snapshots = new_snapshots;
        }

        xr::Result::SUCCESS
    }

    fn validate_action_query<'a>(
        g: &'a RuntimeInner,
        session: xr::Session,
        action: xr::Action,
        subaction_path: xr::Path,
        expected_type: xr::ActionType,
    ) -> Result<(&'a SessionData, &'a ActionData), xr::Result> {
        let s = g
            .sessions
            .get(&session.into_raw())
            .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;
        let act = g
            .actions
            .get(&action.into_raw())
            .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;
        if !s.attached_action_sets.contains(&act.owner_action_set) {
            return Err(xr::Result::ERROR_ACTIONSET_NOT_ATTACHED);
        }
        if act.action_type != expected_type {
            return Err(xr::Result::ERROR_ACTION_TYPE_MISMATCH);
        }
        if subaction_path != xr::Path::NULL
            && !act.subaction_paths.contains(&subaction_path.into_raw())
        {
            return Err(xr::Result::ERROR_PATH_UNSUPPORTED);
        }
        Ok((s, act))
    }

    pub unsafe fn get_action_state_boolean(
        &self,
        session: xr::Session,
        get_info: *const xr::ActionStateGetInfo,
        state_out: *mut xr::ActionStateBoolean,
    ) -> xr::Result {
        if get_info.is_null() || state_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*get_info };
        let g = self.inner.lock();
        let (s, _) = match Self::validate_action_query(
            &g,
            session,
            info.action,
            info.subaction_path,
            xr::ActionType::BOOLEAN_INPUT,
        ) {
            Ok(v) => v,
            Err(e) => return e,
        };
        let out = unsafe { &mut *state_out };
        if let Some(snap) = s
            .snapshots
            .get(&(info.action.into_raw(), info.subaction_path.into_raw()))
            .filter(|sn| sn.is_active)
        {
            let b = match snap.value {
                ActionSnapshotValue::Bool(v) => v,
                _ => false,
            };
            out.current_state = xr::Bool32::from(b);
            out.changed_since_last_sync = xr::Bool32::from(snap.changed_since_last_sync);
            out.last_change_time = xr::Time::from_nanos(snap.last_change_time);
            out.is_active = xr::TRUE;
        } else {
            out.current_state = xr::FALSE;
            out.changed_since_last_sync = xr::FALSE;
            out.last_change_time = xr::Time::from_nanos(0);
            out.is_active = xr::FALSE;
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn get_action_state_float(
        &self,
        session: xr::Session,
        get_info: *const xr::ActionStateGetInfo,
        state_out: *mut xr::ActionStateFloat,
    ) -> xr::Result {
        if get_info.is_null() || state_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*get_info };
        let g = self.inner.lock();
        let (s, _) = match Self::validate_action_query(
            &g,
            session,
            info.action,
            info.subaction_path,
            xr::ActionType::FLOAT_INPUT,
        ) {
            Ok(v) => v,
            Err(e) => return e,
        };
        let out = unsafe { &mut *state_out };
        if let Some(snap) = s
            .snapshots
            .get(&(info.action.into_raw(), info.subaction_path.into_raw()))
            .filter(|sn| sn.is_active)
        {
            let f = match snap.value {
                ActionSnapshotValue::Float(v) => v,
                _ => 0.0,
            };
            out.current_state = f;
            out.changed_since_last_sync = xr::Bool32::from(snap.changed_since_last_sync);
            out.last_change_time = xr::Time::from_nanos(snap.last_change_time);
            out.is_active = xr::TRUE;
        } else {
            out.current_state = 0.0;
            out.changed_since_last_sync = xr::FALSE;
            out.last_change_time = xr::Time::from_nanos(0);
            out.is_active = xr::FALSE;
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn get_action_state_vector2f(
        &self,
        session: xr::Session,
        get_info: *const xr::ActionStateGetInfo,
        state_out: *mut xr::ActionStateVector2f,
    ) -> xr::Result {
        if get_info.is_null() || state_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*get_info };
        let g = self.inner.lock();
        let (s, _) = match Self::validate_action_query(
            &g,
            session,
            info.action,
            info.subaction_path,
            xr::ActionType::VECTOR2F_INPUT,
        ) {
            Ok(v) => v,
            Err(e) => return e,
        };
        let out = unsafe { &mut *state_out };
        if let Some(snap) = s
            .snapshots
            .get(&(info.action.into_raw(), info.subaction_path.into_raw()))
            .filter(|sn| sn.is_active)
        {
            let v = match snap.value {
                ActionSnapshotValue::Vec2(v) => v,
                _ => [0.0, 0.0],
            };
            out.current_state = xr::Vector2f { x: v[0], y: v[1] };
            out.changed_since_last_sync = xr::Bool32::from(snap.changed_since_last_sync);
            out.last_change_time = xr::Time::from_nanos(snap.last_change_time);
            out.is_active = xr::TRUE;
        } else {
            out.current_state = xr::Vector2f { x: 0.0, y: 0.0 };
            out.changed_since_last_sync = xr::FALSE;
            out.last_change_time = xr::Time::from_nanos(0);
            out.is_active = xr::FALSE;
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn get_action_state_pose(
        &self,
        session: xr::Session,
        get_info: *const xr::ActionStateGetInfo,
        state_out: *mut xr::ActionStatePose,
    ) -> xr::Result {
        if get_info.is_null() || state_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*get_info };
        let g = self.inner.lock();
        let (s, _) = match Self::validate_action_query(
            &g,
            session,
            info.action,
            info.subaction_path,
            xr::ActionType::POSE_INPUT,
        ) {
            Ok(v) => v,
            Err(e) => return e,
        };
        let out = unsafe { &mut *state_out };
        let active = s
            .snapshots
            .get(&(info.action.into_raw(), info.subaction_path.into_raw()))
            .map(|sn| sn.is_active)
            .unwrap_or(false);
        out.is_active = xr::Bool32::from(active);
        xr::Result::SUCCESS
    }

    pub unsafe fn enumerate_bound_sources_for_action(
        &self,
        session: xr::Session,
        enumerate_info: *const xr::BoundSourcesForActionEnumerateInfo,
        source_capacity_input: u32,
        source_count_output: *mut u32,
        sources: *mut xr::Path,
    ) -> xr::Result {
        if enumerate_info.is_null() || source_count_output.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*enumerate_info };
        let g = self.inner.lock();
        let Some(s) = g.sessions.get(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let Some(act) = g.actions.get(&info.action.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if !s.attached_action_sets.contains(&act.owner_action_set) {
            return xr::Result::ERROR_ACTIONSET_NOT_ATTACHED;
        }
        let Some(inst) = g.instances.get(&s.owner_instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };

        let mut bound = Vec::new();
        for hand_idx in 0..2usize {
            if !g.physical[hand_idx].active {
                continue;
            }
            let Some(prof_id) = active_profile_for_hand(&g, inst, hand_idx) else {
                continue;
            };
            let prefix = if hand_idx == 0 {
                "/user/hand/left/"
            } else {
                "/user/hand/right/"
            };
            if let Some(bindings) = inst.suggested_bindings.get(&prof_id) {
                for &(baid, bpath_id) in bindings {
                    if baid == act.id {
                        if let Some(bstr) = inst.resolve_path(bpath_id) {
                            if bstr.starts_with(prefix) && !bound.contains(&bpath_id) {
                                bound.push(bpath_id);
                            }
                        }
                    }
                }
            }
        }

        let needed = bound.len() as u32;
        unsafe {
            *source_count_output = needed;
        }
        if source_capacity_input == 0 {
            return xr::Result::SUCCESS;
        }
        if source_capacity_input < needed {
            return xr::Result::ERROR_SIZE_INSUFFICIENT;
        }
        if sources.is_null() && needed > 0 {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        for (i, pid) in bound.into_iter().enumerate() {
            unsafe {
                *sources.add(i) = xr::Path::from_raw(pid);
            }
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn get_input_source_localized_name(
        &self,
        session: xr::Session,
        get_info: *const xr::InputSourceLocalizedNameGetInfo,
        buffer_capacity_input: u32,
        buffer_count_output: *mut u32,
        buffer: *mut c_char,
    ) -> xr::Result {
        if get_info.is_null() || buffer_count_output.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*get_info };
        if info.which_components.is_empty() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let g = self.inner.lock();
        let Some(s) = g.sessions.get(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if s.attached_action_sets.is_empty() {
            return xr::Result::ERROR_ACTIONSET_NOT_ATTACHED;
        }
        let Some(inst) = g.instances.get(&s.owner_instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let Some(path_str) = inst.resolve_path(info.source_path.into_raw()) else {
            return xr::Result::ERROR_PATH_INVALID;
        };

        let mut parts = Vec::new();
        if info
            .which_components
            .contains(xr::InputSourceLocalizedNameFlags::USER_PATH)
        {
            if path_str.starts_with("/user/hand/left") {
                parts.push("Left Hand");
            } else if path_str.starts_with("/user/hand/right") {
                parts.push("Right Hand");
            } else {
                parts.push("User");
            }
        }
        if info
            .which_components
            .contains(xr::InputSourceLocalizedNameFlags::INTERACTION_PROFILE)
        {
            parts.push("Oculus Touch Controller");
        }
        if info
            .which_components
            .contains(xr::InputSourceLocalizedNameFlags::COMPONENT)
        {
            let comp = path_str
                .rsplit('/')
                .next()
                .unwrap_or(path_str);
            parts.push(comp);
        }
        let localized = parts.join(" ");
        let needed = (localized.len() + 1) as u32;
        unsafe {
            *buffer_count_output = needed;
        }
        if buffer_capacity_input == 0 {
            return xr::Result::SUCCESS;
        }
        if buffer_capacity_input < needed {
            return xr::Result::ERROR_SIZE_INSUFFICIENT;
        }
        if buffer.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        unsafe {
            std::ptr::copy_nonoverlapping(localized.as_ptr(), buffer as *mut u8, localized.len());
            *buffer.add(localized.len()) = 0;
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn apply_haptic_feedback(
        &self,
        session: xr::Session,
        haptic_action_info: *const xr::HapticActionInfo,
        haptic_feedback: *const xr::HapticBaseHeader,
    ) -> xr::Result {
        if haptic_action_info.is_null() || haptic_feedback.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*haptic_action_info };
        let mut g = self.inner.lock();
        let (state, owner_instance) = match Self::validate_action_query(
            &g,
            session,
            info.action,
            info.subaction_path,
            xr::ActionType::VIBRATION_OUTPUT,
        ) {
            Ok((s, _)) => (s.state, s.owner_instance),
            Err(e) => return e,
        };
        if state != xr::SessionState::FOCUSED {
            return xr::Result::SESSION_NOT_FOCUSED;
        }
        let sub_str = if info.subaction_path == xr::Path::NULL {
            String::new()
        } else {
            g.instances
                .get(&owner_instance)
                .and_then(|i| i.resolve_path(info.subaction_path.into_raw()))
                .unwrap_or("")
                .to_string()
        };
        let header = unsafe { &*haptic_feedback };
        let (duration_ns, frequency, amplitude) =
            if header.ty == xr::StructureType::HAPTIC_VIBRATION {
                let vib = unsafe { &*(haptic_feedback as *const xr::HapticVibration) };
                (vib.duration.as_nanos(), vib.frequency, vib.amplitude)
            } else {
                (0, 0.0, 0.0)
            };
        g.diagnostics.haptic_apply_calls.push(MockHapticCall {
            session: session.into_raw(),
            action: info.action.into_raw(),
            subaction_path: sub_str,
            duration_ns,
            frequency,
            amplitude,
        });
        xr::Result::SUCCESS
    }

    pub unsafe fn stop_haptic_feedback(
        &self,
        session: xr::Session,
        haptic_action_info: *const xr::HapticActionInfo,
    ) -> xr::Result {
        if haptic_action_info.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*haptic_action_info };
        let mut g = self.inner.lock();
        let (state, owner_instance) = match Self::validate_action_query(
            &g,
            session,
            info.action,
            info.subaction_path,
            xr::ActionType::VIBRATION_OUTPUT,
        ) {
            Ok((s, _)) => (s.state, s.owner_instance),
            Err(e) => return e,
        };
        if state != xr::SessionState::FOCUSED {
            return xr::Result::SESSION_NOT_FOCUSED;
        }
        let sub_str = if info.subaction_path == xr::Path::NULL {
            String::new()
        } else {
            g.instances
                .get(&owner_instance)
                .and_then(|i| i.resolve_path(info.subaction_path.into_raw()))
                .unwrap_or("")
                .to_string()
        };
        g.diagnostics.haptic_stop_calls.push(MockHapticStopCall {
            session: session.into_raw(),
            action: info.action.into_raw(),
            subaction_path: sub_str,
        });
        xr::Result::SUCCESS
    }

    // ---- Spaces ----

    pub unsafe fn create_reference_space(
        &self,
        session: xr::Session,
        create_info: *const xr::ReferenceSpaceCreateInfo,
        space_out: *mut xr::Space,
    ) -> xr::Result {
        if create_info.is_null() || space_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*create_info };
        if !is_valid_xr_pose(info.pose_in_reference_space) {
            return xr::Result::ERROR_POSE_INVALID;
        }
        let mut g = self.inner.lock();
        let Some(s) = g.sessions.get(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let owner_instance = s.owner_instance;
        let id = g.alloc_handle();
        g.spaces.insert(
            id,
            SpaceData {
                id,
                owner_session: session.into_raw(),
                owner_instance,
                kind: SpaceKind::Reference(info.reference_space_type),
                pose_in_space: MockPose::from_xr(info.pose_in_reference_space),
            },
        );
        unsafe {
            *space_out = xr::Space::from_raw(id);
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn create_action_space(
        &self,
        session: xr::Session,
        create_info: *const xr::ActionSpaceCreateInfo,
        space_out: *mut xr::Space,
    ) -> xr::Result {
        if create_info.is_null() || space_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*create_info };
        if !is_valid_xr_pose(info.pose_in_action_space) {
            return xr::Result::ERROR_POSE_INVALID;
        }
        let mut g = self.inner.lock();
        let Some(s) = g.sessions.get(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let owner_instance = s.owner_instance;
        let Some(act) = g.actions.get(&info.action.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if act.action_type != xr::ActionType::POSE_INPUT {
            return xr::Result::ERROR_ACTION_TYPE_MISMATCH;
        }
        if info.subaction_path != xr::Path::NULL
            && !act.subaction_paths.contains(&info.subaction_path.into_raw())
        {
            return xr::Result::ERROR_PATH_UNSUPPORTED;
        }
        let id = g.alloc_handle();
        g.spaces.insert(
            id,
            SpaceData {
                id,
                owner_session: session.into_raw(),
                owner_instance,
                kind: SpaceKind::Action {
                    action_id: info.action.into_raw(),
                    subaction_path: info.subaction_path.into_raw(),
                },
                pose_in_space: MockPose::from_xr(info.pose_in_action_space),
            },
        );
        unsafe {
            *space_out = xr::Space::from_raw(id);
        }
        xr::Result::SUCCESS
    }

    pub fn destroy_space(&self, space: xr::Space) -> xr::Result {
        let id = space.into_raw();
        let mut g = self.inner.lock();
        let Some(sp) = g.spaces.remove(&id) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        g.diagnostics.destroy_order_log.push(format!(
            "xrDestroySpace(session={}, space={})",
            sp.owner_session, id
        ));
        xr::Result::SUCCESS
    }

    pub unsafe fn locate_space(
        &self,
        space: xr::Space,
        base_space: xr::Space,
        time: xr::Time,
        location: *mut xr::SpaceLocation,
    ) -> xr::Result {
        if location.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        if time.as_nanos() <= 0 {
            return xr::Result::ERROR_TIME_INVALID;
        }
        let mut g = self.inner.lock();
        g.diagnostics.locate_space_calls += 1;

        let (flags, pose, vel_opt) = match compute_space_relation(&g, space, base_space) {
            Ok(v) => v,
            Err(e) => return e,
        };

        let loc = unsafe { &mut *location };
        loc.location_flags = xr::SpaceLocationFlags::from_raw(flags);
        loc.pose = pose.to_xr();

        let mut next = loc.next;
        while !next.is_null() {
            let hdr = unsafe { &mut *(next as *mut xr::BaseOutStructure) };
            if hdr.ty == xr::StructureType::SPACE_VELOCITY {
                let sv = unsafe { &mut *(next as *mut xr::SpaceVelocity) };
                if let Some((vflags, lin, ang)) = vel_opt {
                    sv.velocity_flags = xr::SpaceVelocityFlags::from_raw(vflags);
                    sv.linear_velocity = xr::Vector3f {
                        x: lin.x,
                        y: lin.y,
                        z: lin.z,
                    };
                    sv.angular_velocity = xr::Vector3f {
                        x: ang.x,
                        y: ang.y,
                        z: ang.z,
                    };
                } else {
                    sv.velocity_flags = xr::SpaceVelocityFlags::EMPTY;
                    sv.linear_velocity = xr::Vector3f {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    };
                    sv.angular_velocity = xr::Vector3f {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    };
                }
            }
            next = hdr.next as *mut c_void;
        }
        xr::Result::SUCCESS
    }

    pub unsafe fn locate_spaces(
        &self,
        session: xr::Session,
        locate_info: *const xr::SpacesLocateInfo,
        space_locations: *mut xr::SpaceLocations,
    ) -> xr::Result {
        if locate_info.is_null() || space_locations.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*locate_info };
        let locs = unsafe { &mut *space_locations };
        if info.time.as_nanos() <= 0 {
            return xr::Result::ERROR_TIME_INVALID;
        }
        if info.space_count == 0
            || info.spaces.is_null()
            || locs.locations.is_null()
            || locs.location_count != info.space_count
        {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        // Validate optional chained XrSpaceVelocities before mutating anything
        let mut vel_struct: Option<*mut xr::SpaceVelocities> = None;
        let mut next = locs.next;
        while !next.is_null() {
            let hdr = unsafe { &mut *(next as *mut xr::BaseOutStructure) };
            if hdr.ty == xr::StructureType::SPACE_VELOCITIES {
                let sv = unsafe { &mut *(next as *mut xr::SpaceVelocities) };
                if sv.velocity_count != info.space_count || sv.velocities.is_null() {
                    return xr::Result::ERROR_VALIDATION_FAILURE;
                }
                vel_struct = Some(next as *mut xr::SpaceVelocities);
            }
            next = hdr.next as *mut c_void;
        }

        let mut g = self.inner.lock();
        g.diagnostics.locate_spaces_calls += 1;
        if !g.sessions.contains_key(&session.into_raw()) {
            return xr::Result::ERROR_HANDLE_INVALID;
        }

        for i in 0..info.space_count as usize {
            let sp = unsafe { *info.spaces.add(i) };
            let (flags, pose, vel_opt) = match compute_space_relation(&g, sp, info.base_space) {
                Ok(v) => v,
                Err(e) => return e,
            };
            unsafe {
                let out_loc = &mut *locs.locations.add(i);
                out_loc.location_flags = xr::SpaceLocationFlags::from_raw(flags);
                out_loc.pose = pose.to_xr();
                if let Some(sv_ptr) = vel_struct {
                    let out_vel = &mut *(*sv_ptr).velocities.add(i);
                    if let Some((vflags, lin, ang)) = vel_opt {
                        out_vel.velocity_flags = xr::SpaceVelocityFlags::from_raw(vflags);
                        out_vel.linear_velocity = xr::Vector3f {
                            x: lin.x,
                            y: lin.y,
                            z: lin.z,
                        };
                        out_vel.angular_velocity = xr::Vector3f {
                            x: ang.x,
                            y: ang.y,
                            z: ang.z,
                        };
                    } else {
                        out_vel.velocity_flags = xr::SpaceVelocityFlags::EMPTY;
                        out_vel.linear_velocity = xr::Vector3f {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        };
                        out_vel.angular_velocity = xr::Vector3f {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        };
                    }
                }
            }
        }
        xr::Result::SUCCESS
    }

    // ---- XR_EXT_hand_tracking + XR_FB_hand_tracking_aim ----

    pub unsafe fn create_hand_tracker_ext(
        &self,
        session: xr::Session,
        create_info: *const xr::HandTrackerCreateInfoEXT,
        hand_tracker_out: *mut xr::HandTrackerEXT,
    ) -> xr::Result {
        if create_info.is_null() || hand_tracker_out.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*create_info };
        let mut g = self.inner.lock();
        let Some(s) = g.sessions.get(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let owner_instance = s.owner_instance;
        let Some(inst) = g.instances.get(&owner_instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if !inst.enabled_extensions.contains("XR_EXT_hand_tracking") {
            return xr::Result::ERROR_EXTENSION_NOT_PRESENT;
        }
        if g.config.create_hand_tracker_error != 0 {
            return xr::Result::from_raw(g.config.create_hand_tracker_error);
        }
        if !g.config.supports_hand_tracking {
            return xr::Result::ERROR_FEATURE_UNSUPPORTED;
        }
        if info.hand != xr::HandEXT::LEFT && info.hand != xr::HandEXT::RIGHT {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        let id = g.alloc_handle();
        g.hand_trackers.insert(
            id,
            HandTrackerData {
                id,
                owner_session: session.into_raw(),
                owner_instance,
                hand: info.hand,
            },
        );
        g.diagnostics.hand_trackers_created += 1;
        g.diagnostics
            .hand_tracker_create_hands
            .push(info.hand.into_raw());

        unsafe {
            *hand_tracker_out = xr::HandTrackerEXT::from_raw(id);
        }
        xr::Result::SUCCESS
    }

    pub fn destroy_hand_tracker_ext(&self, hand_tracker: xr::HandTrackerEXT) -> xr::Result {
        let id = hand_tracker.into_raw();
        let mut g = self.inner.lock();
        let Some(ht) = g.hand_trackers.remove(&id) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        g.diagnostics.hand_trackers_destroyed += 1;
        g.diagnostics.destroy_order_log.push(format!(
            "xrDestroyHandTrackerEXT(session={}, tracker={})",
            ht.owner_session, id
        ));
        xr::Result::SUCCESS
    }

    pub unsafe fn locate_hand_joints_ext(
        &self,
        hand_tracker: xr::HandTrackerEXT,
        locate_info: *const xr::HandJointsLocateInfoEXT,
        locations: *mut xr::HandJointLocationsEXT,
    ) -> xr::Result {
        if locate_info.is_null() || locations.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = unsafe { &*locate_info };
        let locs = unsafe { &mut *locations };
        if info.time.as_nanos() <= 0 {
            return xr::Result::ERROR_TIME_INVALID;
        }
        if locs.joint_count != HAND_JOINT_COUNT as u32 || locs.joint_locations.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        let mut g = self.inner.lock();
        g.diagnostics.locate_hand_joints_calls += 1;

        let Some(ht) = g.hand_trackers.get(&hand_tracker.into_raw()).cloned() else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let Some(base_sp) = g.spaces.get(&info.base_space.into_raw()).cloned() else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if base_sp.owner_session != ht.owner_session {
            return xr::Result::ERROR_HANDLE_INVALID;
        }
        let fb_aim_enabled = g
            .instances
            .get(&ht.owner_instance)
            .map(|i| i.enabled_extensions.contains("XR_FB_hand_tracking_aim"))
            .unwrap_or(false);

        let base_in_local = resolve_space_in_local(&g, &base_sp).map(|r| r.0);
        let base_pose = match base_in_local {
            Some(p) => p,
            None => {
                locs.is_active = xr::FALSE;
                return xr::Result::SUCCESS;
            }
        };
        let base_is_identity = pose_is_exact_identity(base_pose);

        let hand_idx = if ht.hand == xr::HandEXT::LEFT { 0 } else { 1 };
        let hand = g.hands[hand_idx].clone();

        locs.is_active = xr::Bool32::from(hand.is_active);
        for i in 0..HAND_JOINT_COUNT {
            let j = hand.joints[i];
            let out_pose = if !hand.is_active || base_is_identity {
                j.pose
            } else {
                transform_pose_into_base(j.pose, base_pose)
            };
            unsafe {
                let dst = &mut *locs.joint_locations.add(i);
                dst.location_flags = xr::SpaceLocationFlags::from_raw(j.location_flags);
                dst.pose = out_pose.to_xr();
                dst.radius = j.radius;
            }
        }

        // Walk next chain for XrHandJointVelocitiesEXT and XrHandTrackingAimStateFB
        let mut next = locs.next;
        while !next.is_null() {
            let hdr = unsafe { &mut *(next as *mut xr::BaseOutStructure) };
            if hdr.ty == xr::StructureType::HAND_JOINT_VELOCITIES_EXT {
                let vels = unsafe { &mut *(next as *mut xr::HandJointVelocitiesEXT) };
                if vels.joint_count != HAND_JOINT_COUNT as u32 || vels.joint_velocities.is_null() {
                    return xr::Result::ERROR_VALIDATION_FAILURE;
                }
                for i in 0..HAND_JOINT_COUNT {
                    let j = hand.joints[i];
                    let (lin, ang) = if base_is_identity {
                        (
                            Vec3::from_array(j.linear_velocity),
                            Vec3::from_array(j.angular_velocity),
                        )
                    } else {
                        (
                            rotate_vec_into_base(Vec3::from_array(j.linear_velocity), base_pose),
                            rotate_vec_into_base(Vec3::from_array(j.angular_velocity), base_pose),
                        )
                    };
                    unsafe {
                        let dst = &mut *vels.joint_velocities.add(i);
                        dst.velocity_flags = xr::SpaceVelocityFlags::from_raw(j.velocity_flags);
                        dst.linear_velocity = xr::Vector3f {
                            x: lin.x,
                            y: lin.y,
                            z: lin.z,
                        };
                        dst.angular_velocity = xr::Vector3f {
                            x: ang.x,
                            y: ang.y,
                            z: ang.z,
                        };
                    }
                }
            } else if hdr.ty == xr::StructureType::HAND_TRACKING_AIM_STATE_FB && fb_aim_enabled {
                let aim_out = unsafe { &mut *(next as *mut xr::HandTrackingAimStateFB) };
                if let Some(aim) = hand.aim {
                    let aim_pose = if base_is_identity {
                        aim.aim_pose
                    } else {
                        transform_pose_into_base(aim.aim_pose, base_pose)
                    };
                    aim_out.status = xr::HandTrackingAimFlagsFB::from_raw(aim.status);
                    aim_out.aim_pose = aim_pose.to_xr();
                    aim_out.pinch_strength_index = aim.pinch_strength_index;
                    aim_out.pinch_strength_middle = aim.pinch_strength_middle;
                    aim_out.pinch_strength_ring = aim.pinch_strength_ring;
                    aim_out.pinch_strength_little = aim.pinch_strength_little;
                } else {
                    aim_out.status = xr::HandTrackingAimFlagsFB::EMPTY;
                    aim_out.aim_pose = xr::Posef::IDENTITY;
                    aim_out.pinch_strength_index = 0.0;
                    aim_out.pinch_strength_middle = 0.0;
                    aim_out.pinch_strength_ring = 0.0;
                    aim_out.pinch_strength_little = 0.0;
                }
            }
            next = hdr.next as *mut c_void;
        }

        xr::Result::SUCCESS
    }
}

fn pose_is_exact_identity(p: MockPose) -> bool {
    p.orientation == [0.0, 0.0, 0.0, 1.0] && p.position == [0.0, 0.0, 0.0]
}

fn active_profile_for_hand(
    g: &RuntimeInner,
    inst: &InstanceData,
    hand_idx: usize,
) -> Option<u64> {
    let phys = &g.physical[hand_idx];
    if !phys.active {
        return None;
    }
    if let Some(ref prof_str) = phys.interaction_profile {
        if let Some(pid) = inst.lookup_path_id(prof_str) {
            if inst.suggested_bindings.contains_key(&pid) {
                return Some(pid);
            }
        }
    }
    inst.suggested_profile_order.first().copied()
}

fn resolve_space_in_local(
    g: &RuntimeInner,
    sp: &SpaceData,
) -> Option<(MockPose, u64, Option<(u64, Vec3, Vec3)>)> {
    match sp.kind {
        SpaceKind::Reference(ref_ty) => {
            let base = match ref_ty {
                xr::ReferenceSpaceType::LOCAL => MockPose::IDENTITY,
                xr::ReferenceSpaceType::STAGE => g.config.stage_to_local_pose,
                xr::ReferenceSpaceType::VIEW => g.config.view_to_local_pose,
                xr::ReferenceSpaceType::LOCAL_FLOOR => g.config.local_floor_to_local_pose,
                _ => MockPose::IDENTITY,
            };
            let (q, p) = pose_mul(base.to_quat_pos(), sp.pose_in_space.to_quat_pos());
            Some((
                MockPose::from_quat_pos(q, p),
                FULL_LOCATION_FLAGS,
                Some((FULL_VELOCITY_FLAGS, Vec3::ZERO, Vec3::ZERO)),
            ))
        }
        SpaceKind::Action {
            action_id,
            subaction_path,
        } => {
            let inst = g.instances.get(&sp.owner_instance)?;
            let left_pid = inst.lookup_path_id("/user/hand/left");
            let right_pid = inst.lookup_path_id("/user/hand/right");
            let hand_candidates: Vec<usize> = if subaction_path == 0 {
                vec![0, 1]
            } else if left_pid == Some(subaction_path) {
                vec![0]
            } else if right_pid == Some(subaction_path) {
                vec![1]
            } else {
                vec![]
            };
            for hand_idx in hand_candidates {
                if !g.physical[hand_idx].active {
                    continue;
                }
                let Some(prof_id) = active_profile_for_hand(g, inst, hand_idx) else {
                    continue;
                };
                let prefix = if hand_idx == 0 {
                    "/user/hand/left/"
                } else {
                    "/user/hand/right/"
                };
                let binding_str = inst.suggested_bindings.get(&prof_id).and_then(|bs| {
                    bs.iter().find_map(|&(baid, bpath_id)| {
                        if baid == action_id {
                            let s = inst.resolve_path(bpath_id)?;
                            if s.starts_with(prefix) {
                                Some(s)
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                });
                let Some(bstr) = binding_str else {
                    continue;
                };
                let phys = &g.physical[hand_idx];
                let (base_pose, loc_flags, vel_opt) = if bstr.contains("/aim") {
                    (phys.aim_pose, phys.aim_location_flags, None)
                } else if bstr.contains("/palm_ext") {
                    (phys.palm_pose, phys.palm_location_flags, None)
                } else if bstr.contains("/grip_surface") {
                    (
                        phys.grip_surface_pose,
                        phys.grip_surface_location_flags,
                        None,
                    )
                } else {
                    let v = if phys.grip_velocity_flags != 0 {
                        Some((
                            phys.grip_velocity_flags,
                            Vec3::from_array(phys.grip_linear_velocity),
                            Vec3::from_array(phys.grip_angular_velocity),
                        ))
                    } else {
                        None
                    };
                    (phys.grip_pose, phys.grip_location_flags, v)
                };
                let (q_base, p_base) = base_pose.to_quat_pos();
                let (q_sub, p_sub) = sp.pose_in_space.to_quat_pos();
                let (q_final, p_final) = pose_mul((q_base, p_base), (q_sub, p_sub));
                let lever_arm = normalize_safe(q_base) * p_sub;
                let adjusted_vel = vel_opt.map(|(vf, lin, ang)| (vf, lin + ang.cross(lever_arm), ang));
                return Some((MockPose::from_quat_pos(q_final, p_final), loc_flags, adjusted_vel));
            }
            None
        }
    }
}

fn compute_space_relation(
    g: &RuntimeInner,
    space: xr::Space,
    base_space: xr::Space,
) -> Result<(u64, MockPose, Option<(u64, Vec3, Vec3)>), xr::Result> {
    let target = g
        .spaces
        .get(&space.into_raw())
        .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;
    let base = g
        .spaces
        .get(&base_space.into_raw())
        .ok_or(xr::Result::ERROR_HANDLE_INVALID)?;
    if target.owner_session != base.owner_session {
        return Err(xr::Result::ERROR_HANDLE_INVALID);
    }

    // Same-entity OpenXR rule: always VALID | TRACKED even when untracked
    if target.kind == base.kind {
        let rel = pose_mul(
            pose_inv(base.pose_in_space.to_quat_pos()),
            target.pose_in_space.to_quat_pos(),
        );
        return Ok((
            FULL_LOCATION_FLAGS,
            MockPose::from_quat_pos(rel.0, rel.1),
            Some((FULL_VELOCITY_FLAGS, Vec3::ZERO, Vec3::ZERO)),
        ));
    }

    let Some((t_pose, t_flags, t_vel)) = resolve_space_in_local(g, target) else {
        return Ok((0, MockPose::IDENTITY, None));
    };
    let Some((b_pose, b_flags, b_vel)) = resolve_space_in_local(g, base) else {
        return Ok((0, MockPose::IDENTITY, None));
    };

    let combined_flags = t_flags & b_flags;
    let rel_pose = transform_pose_into_base(t_pose, b_pose);

    let rel_vel = match (t_vel, b_vel) {
        (Some((tvf, t_lin, t_ang)), Some((bvf, b_lin, b_ang))) => {
            let vf = tvf & bvf;
            if vf == 0 {
                None
            } else {
                let (qb, pb) = b_pose.to_quat_pos();
                let (_, pt) = t_pose.to_quat_pos();
                let qb_inv = normalize_safe(qb).conjugate();
                let ang_local = t_ang - b_ang;
                let lin_local = t_lin - b_lin - b_ang.cross(pt - pb);
                Some((vf, qb_inv * lin_local, qb_inv * ang_local))
            }
        }
        _ => None,
    };

    Ok((combined_flags, rel_pose, rel_vel))
}

fn evaluate_physical_action(
    physical: &[MockPhysicalControllerState; 2],
    action_type: xr::ActionType,
    matched_paths: &[(usize, &str)],
) -> ActionSnapshotValue {
    match action_type {
        xr::ActionType::BOOLEAN_INPUT => {
            let mut any = false;
            for &(hand, path) in matched_paths {
                let c = &physical[hand];
                let b = eval_phys_bool(c, path);
                any |= b;
            }
            ActionSnapshotValue::Bool(any)
        }
        xr::ActionType::FLOAT_INPUT => {
            let mut best = 0.0f32;
            for &(hand, path) in matched_paths {
                let c = &physical[hand];
                let f = eval_phys_float(c, path);
                if f.abs() > best.abs() {
                    best = f;
                }
            }
            ActionSnapshotValue::Float(best)
        }
        xr::ActionType::VECTOR2F_INPUT => {
            let mut best = Vec2::ZERO;
            for &(hand, path) in matched_paths {
                let c = &physical[hand];
                let v = eval_phys_vec2(c, path);
                if v.length_squared() > best.length_squared() {
                    best = v;
                }
            }
            ActionSnapshotValue::Vec2([best.x, best.y])
        }
        xr::ActionType::POSE_INPUT => ActionSnapshotValue::Pose,
        _ => ActionSnapshotValue::Bool(false),
    }
}

fn eval_phys_bool(c: &MockPhysicalControllerState, path: &str) -> bool {
    if path.ends_with("/trigger/touch") {
        c.trigger_touch
    } else if path.ends_with("/trigger/click") {
        c.trigger_click || c.trigger_value >= 0.5
    } else if path.ends_with("/trigger/value") || path.ends_with("/trigger") {
        c.trigger_click || c.trigger_value >= 0.5
    } else if path.ends_with("/squeeze/click")
        || path.ends_with("/squeeze/value")
        || path.ends_with("/squeeze")
    {
        c.squeeze_click || c.squeeze_value >= 0.5
    } else if path.ends_with("/thumbstick/touch") {
        c.thumbstick_touch
    } else if path.ends_with("/thumbstick/click") || path.ends_with("/thumbstick") {
        c.thumbstick_click
    } else if path.ends_with("/x/click")
        || path.ends_with("/x")
        || path.ends_with("/a/click")
        || path.ends_with("/a")
    {
        c.primary_click
    } else if path.ends_with("/x/touch") || path.ends_with("/a/touch") {
        c.primary_touch
    } else if path.ends_with("/y/click")
        || path.ends_with("/y")
        || path.ends_with("/b/click")
        || path.ends_with("/b")
    {
        c.secondary_click
    } else if path.ends_with("/y/touch") || path.ends_with("/b/touch") {
        c.secondary_touch
    } else if path.ends_with("/menu/click") || path.ends_with("/menu") {
        c.menu_click
    } else if path.ends_with("/system/click") || path.ends_with("/system") {
        c.system_click
    } else if path.ends_with("/select/click") || path.ends_with("/select") {
        c.select_click || c.trigger_click || c.trigger_value >= 0.5
    } else if path.ends_with("/thumbrest/touch") || path.ends_with("/thumbrest") {
        c.thumbrest_touch
    } else {
        false
    }
}

fn eval_phys_float(c: &MockPhysicalControllerState, path: &str) -> f32 {
    if path.ends_with("/trigger/value") || path.ends_with("/trigger") || path.ends_with("/trigger/force") {
        c.trigger_value
    } else if path.ends_with("/squeeze/value") || path.ends_with("/squeeze") {
        c.squeeze_value
    } else if path.ends_with("/thumbstick/x") {
        c.thumbstick[0]
    } else if path.ends_with("/thumbstick/y") {
        c.thumbstick[1]
    } else if eval_phys_bool(c, path) {
        1.0
    } else {
        0.0
    }
}

fn eval_phys_vec2(c: &MockPhysicalControllerState, path: &str) -> Vec2 {
    if path.contains("/thumbstick") {
        Vec2::new(c.thumbstick[0], c.thumbstick[1])
    } else {
        Vec2::ZERO
    }
}

pub fn is_valid_openxr_path(s: &str) -> bool {
    if s.is_empty() || s.len() >= xr::MAX_PATH_LENGTH {
        return false;
    }
    if !s.starts_with('/') || s.ends_with('/') {
        return false;
    }
    for seg in s[1..].split('/') {
        if seg.is_empty() || seg == "." || seg == ".." {
            return false;
        }
        if !seg
            .bytes()
            .all(|b| matches!(b, b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.'))
        {
            return false;
        }
    }
    true
}

pub(crate) fn write_cstr_buf(dst: &mut [c_char], s: &str) {
    dst.fill(0);
    let max_len = dst.len().saturating_sub(1);
    let bytes = s.as_bytes();
    let copy_len = bytes.len().min(max_len);
    for i in 0..copy_len {
        dst[i] = bytes[i] as c_char;
    }
}

fn read_cstr_arr(buf: &[c_char]) -> String {
    let len = buf.iter().position(|&c| c == 0).unwrap_or(buf.len());
    let bytes: Vec<u8> = buf[..len].iter().map(|&c| c as u8).collect();
    String::from_utf8_lossy(&bytes).into_owned()
}
