//! Opaque-box E2E test infrastructure: artifact resolution, RAII workspace temp dirs,
//! independent 26-joint hand pose builders, and Harness A (direct `dlopen` mock loader).

use anyhow::{Context, Result};
use glam::{EulerRot, Quat, Vec3};
use libloading::{Library, Symbol};
use mock_xr_core::{
    global_engine, rt_create_api_layer_instance_terminator, rt_get_instance_proc_addr,
    MockAimInput, MockHandState, MockJointInput, MockPose, MockRuntimeConfig,
    FULL_LOCATION_FLAGS, FULL_VELOCITY_FLAGS, HAND_JOINT_COUNT,
};
use openxr_sys as xr;
use openxr_sys::Handle as _;
use parking_lot::Mutex;
use std::ffi::{c_char, CStr, CString};
use std::fs;
use std::mem::size_of;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::LazyLock;

/// Global mutex serializing in-process Harness A tests that share `global_engine()`.
pub static HARNESS_LOCK: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

static TMP_COUNTER: AtomicU64 = AtomicU64::new(0);

/// RAII temp directory rooted under `e2e_suite/target/e2e_tmp` (stays off small `/` mount).
pub struct WorkspaceTempDir {
    path: PathBuf,
}

impl WorkspaceTempDir {
    pub fn new(label: &str) -> Result<Self> {
        let base = e2e_suite_dir().join("target").join("e2e_tmp");
        let id = TMP_COUNTER.fetch_add(1, Ordering::Relaxed);
        let path = base.join(format!("{label}_{}_{id}", std::process::id()));
        fs::create_dir_all(&path)?;
        Ok(Self { path })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for WorkspaceTempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.path);
    }
}

pub fn e2e_suite_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

pub fn quest_layer_dir() -> PathBuf {
    e2e_suite_dir().parent().unwrap().to_path_buf()
}

pub fn host_layer_so_path() -> PathBuf {
    if let Ok(p) = std::env::var("E2E_HOST_LAYER_SO") {
        return PathBuf::from(p);
    }
    let dbg = quest_layer_dir().join("target/debug/libXrApiLayer_alvr_hand_emulation.so");
    if dbg.exists() {
        return dbg;
    }
    quest_layer_dir().join("target/release/libXrApiLayer_alvr_hand_emulation.so")
}

pub fn android_layer_so_path() -> PathBuf {
    if let Ok(p) = std::env::var("E2E_ANDROID_LAYER_SO") {
        return PathBuf::from(p);
    }
    quest_layer_dir().join("dist/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so")
}

pub fn layer_manifest_json_path() -> PathBuf {
    if let Ok(p) = std::env::var("E2E_LAYER_MANIFEST_JSON") {
        return PathBuf::from(p);
    }
    quest_layer_dir().join("manifest/XrApiLayer_alvr_hand_emulation.json")
}

pub fn dist_dir() -> PathBuf {
    if let Ok(p) = std::env::var("E2E_DIST_DIR") {
        return PathBuf::from(p);
    }
    quest_layer_dir().join("dist")
}

pub fn packager_bin_path() -> PathBuf {
    if let Ok(p) = std::env::var("E2E_PACKAGER_BIN") {
        return PathBuf::from(p);
    }
    let rel = quest_layer_dir().join("target/release/alvr-xr-packager");
    if rel.exists() {
        return rel;
    }
    quest_layer_dir().join("target/debug/alvr-xr-packager")
}

pub fn khronos_loader_so_path() -> PathBuf {
    e2e_suite_dir().join("target/khronos_loader/libopenxr_loader.so")
}

pub fn mock_runtime_so_path() -> PathBuf {
    let dbg = e2e_suite_dir().join("target/debug/libmock_xr_runtime.so");
    if dbg.exists() {
        return dbg;
    }
    e2e_suite_dir().join("target/release/libmock_xr_runtime.so")
}

/// Independent 26-joint hand skeleton builder for testing ALVR gestures cleanly.
pub struct HandSkeletonBuilder {
    pub hand_index: usize, // 0 = Left, 1 = Right
    pub palm_pos: Vec3,
    pub palm_ori: Quat,
    pub joints: Vec<MockJointInput>,
    pub aim: Option<MockAimInput>,
}

impl HandSkeletonBuilder {
    /// Builds an open, neutral hand with fingers well separated (no pinches, no curls, no joystick).
    pub fn open(hand_index: usize, palm_pos: Vec3, palm_ori: Quat) -> Self {
        let mut joints = vec![MockJointInput::default(); HAND_JOINT_COUNT];
        // Spread fingers well outside pinch/curl thresholds:
        // Palm at (0, 0, 0), Wrist at (0, 0, +0.07), tips 10+ cm away.
        let local_positions: [Vec3; HAND_JOINT_COUNT] = [
            Vec3::new(0.0, 0.0, 0.0),         // 0 Palm
            Vec3::new(0.0, 0.0, 0.07),        // 1 Wrist
            Vec3::new(-0.04, 0.0, 0.04),      // 2 ThumbMetacarpal
            Vec3::new(-0.06, 0.0, 0.01),      // 3 ThumbProximal
            Vec3::new(-0.08, 0.0, -0.02),     // 4 ThumbDistal
            Vec3::new(-0.10, 0.0, -0.05),     // 5 ThumbTip (far from palm & all tips)
            Vec3::new(-0.025, 0.0, -0.02),    // 6 IndexMetacarpal
            Vec3::new(-0.025, 0.0, -0.05),    // 7 IndexProximal
            Vec3::new(-0.025, 0.0, -0.08),    // 8 IndexIntermediate
            Vec3::new(-0.025, 0.0, -0.10),    // 9 IndexDistal
            Vec3::new(-0.025, 0.0, -0.13),    // 10 IndexTip
            Vec3::new(0.0, 0.0, -0.02),       // 11 MiddleMetacarpal
            Vec3::new(0.0, 0.0, -0.05),       // 12 MiddleProximal
            Vec3::new(0.0, 0.0, -0.085),      // 13 MiddleIntermediate
            Vec3::new(0.0, 0.0, -0.11),       // 14 MiddleDistal
            Vec3::new(0.0, 0.0, -0.14),       // 15 MiddleTip
            Vec3::new(0.025, 0.0, -0.02),     // 16 RingMetacarpal
            Vec3::new(0.025, 0.0, -0.05),     // 17 RingProximal
            Vec3::new(0.025, 0.0, -0.08),     // 18 RingIntermediate
            Vec3::new(0.025, 0.0, -0.105),    // 19 RingDistal
            Vec3::new(0.025, 0.0, -0.13),     // 20 RingTip
            Vec3::new(0.05, 0.0, -0.02),      // 21 LittleMetacarpal
            Vec3::new(0.05, 0.0, -0.045),     // 22 LittleProximal
            Vec3::new(0.05, 0.0, -0.07),      // 23 LittleIntermediate
            Vec3::new(0.05, 0.0, -0.09),      // 24 LittleDistal
            Vec3::new(0.05, 0.0, -0.115),     // 25 LittleTip
        ];
        for (i, lp) in local_positions.iter().enumerate() {
            let p = palm_pos + palm_ori * *lp;
            joints[i] = MockJointInput {
                pose: MockPose::from_quat_pos(palm_ori, p),
                location_flags: FULL_LOCATION_FLAGS,
                radius: 0.008,
                velocity_flags: 0,
                linear_velocity: [0.0; 3],
                angular_velocity: [0.0; 3],
            };
        }
        Self {
            hand_index,
            palm_pos,
            palm_ori,
            joints,
            aim: None,
        }
    }

    /// Sets joint `idx` local position relative to palm.
    pub fn set_joint_local_pos(mut self, idx: usize, local: Vec3) -> Self {
        let p = self.palm_pos + self.palm_ori * local;
        self.joints[idx].pose = MockPose::from_quat_pos(self.palm_ori, p);
        self
    }

    /// Pinches thumb tip (5) directly onto target finger tip (`10=index, 15=middle, 20=ring, 25=little`).
    pub fn with_pinch(mut self, finger_tip_idx: usize, separation_m: f32) -> Self {
        let target = Vec3::from_array(self.joints[finger_tip_idx].pose.position);
        let thumb_pos = target + self.palm_ori * Vec3::new(0.0, separation_m, 0.0);
        self.joints[5].pose = MockPose::from_quat_pos(self.palm_ori, thumb_pos);
        self
    }

    /// Curls middle (15), ring (20), and little (25) tips into their metacarpal-proximal midpoints (full GripCurl = 1.0).
    pub fn with_grip_curl(mut self) -> Self {
        for (meta, prox, tip) in [(11, 12, 15), (16, 17, 20), (21, 22, 25)] {
            let m = Vec3::from_array(self.joints[meta].pose.position);
            let p = Vec3::from_array(self.joints[prox].pose.position);
            let mid = (m + p) * 0.5;
            self.joints[tip].pose = MockPose::from_quat_pos(self.palm_ori, mid);
        }
        self
    }

    /// Curls index tip (10) onto its metacarpal-proximal midpoint (index_curl = 1.0).
    pub fn with_index_curl(mut self) -> Self {
        let m = Vec3::from_array(self.joints[6].pose.position);
        let p = Vec3::from_array(self.joints[7].pose.position);
        let mid = (m + p) * 0.5;
        self.joints[10].pose = MockPose::from_quat_pos(self.palm_ori, mid);
        self
    }

    /// Curls thumb tip (5) into the palm center (ThumbCurl = 1.0 -> thumbstick click).
    pub fn with_thumb_curl(mut self) -> Self {
        self.joints[5].pose = MockPose::from_quat_pos(self.palm_ori, self.palm_pos);
        self
    }

    /// Places hand into ALVR joystick pose (curled grip + curled index + thumb positioned relative
    /// to midpoint of index intermediate [8] & distal [9] along joystick X/Y in meters).
    pub fn with_joystick(mut self, dx_m: f32, dy_m: f32) -> Self {
        self = self.with_grip_curl().with_index_curl();
        let idx_int = Vec3::from_array(self.joints[8].pose.position);
        let idx_dis = Vec3::from_array(self.joints[9].pose.position);
        let center = (idx_int + idx_dis) * 0.5;
        let hori = self.palm_ori
            * if self.hand_index == 0 {
                Vec3::Y
            } else {
                Vec3::NEG_Y
            };
        let vert = self.palm_ori * Vec3::Z;
        let thumb_pos = center + hori * dx_m + vert * dy_m;
        self.joints[5].pose = MockPose::from_quat_pos(self.palm_ori, thumb_pos);
        self
    }

    pub fn with_palm_velocity(mut self, lin: Vec3, ang: Vec3) -> Self {
        self.joints[0].velocity_flags = FULL_VELOCITY_FLAGS;
        self.joints[0].linear_velocity = lin.to_array();
        self.joints[0].angular_velocity = ang.to_array();
        self
    }

    pub fn with_fb_aim(mut self, aim_pose: MockPose, valid: bool) -> Self {
        self.aim = Some(MockAimInput {
            status: if valid { 0x03 } else { 0x00 }, // COMPUTED | VALID
            aim_pose,
            ..Default::default()
        });
        self
    }

    pub fn build(self) -> MockHandState {
        MockHandState {
            is_active: true,
            joints: self.joints,
            aim: self.aim,
        }
    }
}

/// Direct `dlopen` OpenXR API Layer harness (Harness A).
pub struct DirectLayerHarness {
    _lib: Library,
    pub negotiate: xr::pfn::NegotiateLoaderApiLayerInterface,
    pub init_loader: Option<xr::pfn::InitializeLoaderKHR>,
    pub layer_gipa: xr::pfn::GetInstanceProcAddr,
    pub layer_create_instance: xr::pfn::CreateApiLayerInstance,
}

impl DirectLayerHarness {
    pub fn load() -> Result<Self> {
        let so_path = host_layer_so_path();
        anyhow::ensure!(
            so_path.exists(),
            "Host layer .so not found at {} — run cargo build first",
            so_path.display()
        );
        unsafe {
            let lib = Library::new(&so_path)
                .with_context(|| format!("dlopen failed for {}", so_path.display()))?;
            let negotiate: Symbol<xr::pfn::NegotiateLoaderApiLayerInterface> = lib
                .get(b"xrNegotiateLoaderApiLayerInterface\0")
                .context("missing symbol xrNegotiateLoaderApiLayerInterface")?;
            let negotiate = *negotiate;
            let init_loader: Option<xr::pfn::InitializeLoaderKHR> = lib
                .get::<xr::pfn::InitializeLoaderKHR>(b"xrInitializeLoaderKHR\0")
                .ok()
                .map(|s| *s);

            let li = xr::NegotiateLoaderInfo {
                struct_type: xr::LoaderInterfaceStructs::LOADER_INFO,
                struct_version: xr::LOADER_INFO_STRUCT_VERSION as u32,
                struct_size: size_of::<xr::NegotiateLoaderInfo>(),
                min_interface_version: 1,
                max_interface_version: 1,
                min_api_version: xr::Version::new(1, 0, 0),
                max_api_version: xr::Version::new(1, 1, 54),
            };
            let mut req = xr::NegotiateApiLayerRequest {
                struct_type: xr::LoaderInterfaceStructs::API_LAYER_REQUEST,
                struct_version: xr::API_LAYER_INFO_STRUCT_VERSION as u32,
                struct_size: size_of::<xr::NegotiateApiLayerRequest>(),
                layer_interface_version: 0,
                layer_api_version: xr::Version::from_raw(0),
                get_instance_proc_addr: None,
                create_api_layer_instance: None,
            };
            let name = CString::new("XR_APILAYER_ALVR_hand_emulation")?;
            let res = negotiate(&li, name.as_ptr(), &mut req);
            anyhow::ensure!(
                res == xr::Result::SUCCESS,
                "layer negotiation failed: {res:?}"
            );
            Ok(Self {
                _lib: lib,
                negotiate,
                init_loader,
                layer_gipa: req.get_instance_proc_addr.context("null layer gipa")?,
                layer_create_instance: req
                    .create_api_layer_instance
                    .context("null layer create_api_layer_instance")?,
            })
        }
    }

    pub fn create_instance(
        &self,
        api_version: xr::Version,
        extensions: &[&str],
    ) -> Result<xr::Instance> {
        let ext_cstrs: Vec<CString> = extensions
            .iter()
            .map(|s| CString::new(*s).unwrap())
            .collect();
        let ext_ptrs: Vec<*const c_char> = ext_cstrs.iter().map(|s| s.as_ptr()).collect();

        let mut app_info = xr::ApplicationInfo {
            application_name: [0; xr::MAX_APPLICATION_NAME_SIZE],
            application_version: 1,
            engine_name: [0; xr::MAX_ENGINE_NAME_SIZE],
            engine_version: 1,
            api_version,
        };
        for (i, b) in b"E2EQuestApp".iter().enumerate() {
            app_info.application_name[i] = *b as c_char;
        }

        let create_info = xr::InstanceCreateInfo {
            ty: xr::StructureType::INSTANCE_CREATE_INFO,
            next: std::ptr::null(),
            create_flags: xr::InstanceCreateFlags::EMPTY,
            application_info: app_info,
            enabled_api_layer_count: 0,
            enabled_api_layer_names: std::ptr::null(),
            enabled_extension_count: ext_ptrs.len() as u32,
            enabled_extension_names: if ext_ptrs.is_empty() {
                std::ptr::null()
            } else {
                ext_ptrs.as_ptr()
            },
        };

        let mut next_info = xr::ApiLayerNextInfo {
            struct_type: xr::LoaderInterfaceStructs::API_LAYER_NEXT_INFO,
            struct_version: xr::API_LAYER_NEXT_INFO_STRUCT_VERSION as u32,
            struct_size: size_of::<xr::ApiLayerNextInfo>(),
            layer_name: [0; xr::MAX_API_LAYER_NAME_SIZE],
            next_get_instance_proc_addr: Some(rt_get_instance_proc_addr),
            next_create_api_layer_instance: Some(rt_create_api_layer_instance_terminator),
            next: std::ptr::null_mut(),
        };
        for (i, b) in b"XR_APILAYER_ALVR_hand_emulation".iter().enumerate() {
            next_info.layer_name[i] = *b as c_char;
        }

        let layer_info = xr::ApiLayerCreateInfo {
            struct_type: xr::LoaderInterfaceStructs::API_LAYER_CREATE_INFO,
            struct_version: xr::API_LAYER_CREATE_INFO_STRUCT_VERSION as u32,
            struct_size: size_of::<xr::ApiLayerCreateInfo>(),
            loader_instance: std::ptr::null_mut(),
            settings_file_location: [0; xr::API_LAYER_MAX_SETTINGS_PATH_SIZE],
            next_info: &mut next_info,
        };

        let mut instance = xr::Instance::NULL;
        let res = unsafe { (self.layer_create_instance)(&create_info, &layer_info, &mut instance) };
        anyhow::ensure!(
            res == xr::Result::SUCCESS,
            "xrCreateApiLayerInstance failed: {res:?}"
        );
        Ok(instance)
    }

    pub unsafe fn pfn<T>(&self, instance: xr::Instance, name: &str) -> Result<T> {
        let cs = CString::new(name)?;
        let mut func: Option<xr::pfn::VoidFunction> = None;
        let res = unsafe { (self.layer_gipa)(instance, cs.as_ptr(), &mut func) };
        anyhow::ensure!(
            res == xr::Result::SUCCESS,
            "xrGetInstanceProcAddr({name}) failed: {res:?}"
        );
        let f = func.with_context(|| format!("null proc addr for {name}"))?;
        Ok(unsafe { std::mem::transmute_copy::<xr::pfn::VoidFunction, T>(&f) })
    }

    pub fn try_pfn_raw(
        &self,
        instance: xr::Instance,
        name: &str,
    ) -> (xr::Result, Option<xr::pfn::VoidFunction>) {
        let cs = CString::new(name).unwrap();
        let mut func: Option<xr::pfn::VoidFunction> = None;
        let res = unsafe { (self.layer_gipa)(instance, cs.as_ptr(), &mut func) };
        (res, func)
    }
}

/// Resets `global_engine()` with `cfg` for a clean test.
pub fn reset_mock_runtime(cfg: MockRuntimeConfig) {
    global_engine().reset();
    global_engine().configure(cfg);
}

/// Advances both mock runtime `XrTime` and wall clock (by `delta_ms`) for deterministic debounce tests.
pub fn advance_dual_clock_ms(delta_ms: u64) -> i64 {
    std::thread::sleep(std::time::Duration::from_millis(delta_ms));
    global_engine().advance_time_ns((delta_ms as i64) * 1_000_000)
}

/// Helper to create a `Quat` from Euler XYZ degrees cleanly.
pub fn quat_euler_xyz_deg(x: f32, y: f32, z: f32) -> Quat {
    let d2r = std::f32::consts::PI / 180.0;
    Quat::from_euler(EulerRot::XYZ, x * d2r, y * d2r, z * d2r)
}

/// Helper reading C string buffer returned by OpenXR.
pub fn c_buf_to_string(buf: &[c_char]) -> String {
    unsafe { CStr::from_ptr(buf.as_ptr()) }
        .to_string_lossy()
        .into_owned()
}
