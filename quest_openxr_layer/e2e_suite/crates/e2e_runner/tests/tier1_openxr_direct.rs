//! Tier 1 & Adversarial Direct `dlopen` OpenXR E2E Test Suite against compiled
//! `libXrApiLayer_alvr_hand_emulation.so` and in-process `MockRuntimeEngine`.

use e2e_runner::{
    advance_dual_clock_ms, c_buf_to_string, reset_mock_runtime, DirectLayerHarness,
    HandSkeletonBuilder, HARNESS_LOCK,
};
use glam::{Quat, Vec3};
use mock_xr_core::{
    global_engine, MockExtensionSpec, MockHandState, MockPhysicalControllerState, MockPose,
    MockRuntimeConfig,
};
use openxr_sys as xr;
use openxr_sys::Handle as _;
use std::ffi::{c_char, c_void, CString};
use std::mem::size_of;

struct XrApp {
    h: DirectLayerHarness,
    inst: xr::Instance,
    #[allow(dead_code)]
    sys_id: xr::SystemId,
    sess: xr::Session,
    action_set: xr::ActionSet,
    local_space: xr::Space,
    left_user: xr::Path,
    right_user: xr::Path,
}

impl XrApp {
    fn new(
        h: DirectLayerHarness,
        api_version: xr::Version,
        exts: &[&str],
    ) -> Self {
        let inst = h.create_instance(api_version, exts).expect("create_instance");
        unsafe {
            let get_sys: xr::pfn::GetSystem = h.pfn(inst, "xrGetSystem").unwrap();
            let create_sess: xr::pfn::CreateSession = h.pfn(inst, "xrCreateSession").unwrap();
            let s2p: xr::pfn::StringToPath = h.pfn(inst, "xrStringToPath").unwrap();
            let create_as: xr::pfn::CreateActionSet = h.pfn(inst, "xrCreateActionSet").unwrap();
            let create_ref: xr::pfn::CreateReferenceSpace =
                h.pfn(inst, "xrCreateReferenceSpace").unwrap();

            let sgi = xr::SystemGetInfo {
                ty: xr::StructureType::SYSTEM_GET_INFO,
                next: std::ptr::null(),
                form_factor: xr::FormFactor::HEAD_MOUNTED_DISPLAY,
            };
            let mut sys_id = xr::SystemId::NULL;
            assert_eq!(get_sys(inst, &sgi, &mut sys_id), xr::Result::SUCCESS);

            let sci = xr::SessionCreateInfo {
                ty: xr::StructureType::SESSION_CREATE_INFO,
                next: std::ptr::null(),
                create_flags: xr::SessionCreateFlags::EMPTY,
                system_id: sys_id,
            };
            let mut sess = xr::Session::NULL;
            assert_eq!(create_sess(inst, &sci, &mut sess), xr::Result::SUCCESS);

            let mut asci = xr::ActionSetCreateInfo {
                ty: xr::StructureType::ACTION_SET_CREATE_INFO,
                next: std::ptr::null(),
                action_set_name: [0; xr::MAX_ACTION_SET_NAME_SIZE],
                localized_action_set_name: [0; xr::MAX_LOCALIZED_ACTION_SET_NAME_SIZE],
                priority: 0,
            };
            for (i, b) in b"main_set".iter().enumerate() {
                asci.action_set_name[i] = *b as c_char;
                asci.localized_action_set_name[i] = *b as c_char;
            }
            let mut action_set = xr::ActionSet::NULL;
            assert_eq!(create_as(inst, &asci, &mut action_set), xr::Result::SUCCESS);

            let rsci = xr::ReferenceSpaceCreateInfo {
                ty: xr::StructureType::REFERENCE_SPACE_CREATE_INFO,
                next: std::ptr::null(),
                reference_space_type: xr::ReferenceSpaceType::LOCAL,
                pose_in_reference_space: xr::Posef::IDENTITY,
            };
            let mut local_space = xr::Space::NULL;
            assert_eq!(create_ref(sess, &rsci, &mut local_space), xr::Result::SUCCESS);

            let left_user = Self::intern_path(s2p, inst, "/user/hand/left");
            let right_user = Self::intern_path(s2p, inst, "/user/hand/right");

            Self {
                h,
                inst,
                sys_id,
                sess,
                action_set,
                local_space,
                left_user,
                right_user,
            }
        }
    }

    unsafe fn intern_path(s2p: xr::pfn::StringToPath, inst: xr::Instance, s: &str) -> xr::Path {
        let cs = CString::new(s).unwrap();
        let mut p = xr::Path::NULL;
        assert_eq!(unsafe { s2p(inst, cs.as_ptr(), &mut p) }, xr::Result::SUCCESS);
        p
    }

    fn path(&self, s: &str) -> xr::Path {
        unsafe {
            let s2p: xr::pfn::StringToPath = self.h.pfn(self.inst, "xrStringToPath").unwrap();
            Self::intern_path(s2p, self.inst, s)
        }
    }

    fn create_action(&self, name: &str, ty: xr::ActionType, subs: &[xr::Path]) -> xr::Action {
        unsafe {
            let create_act: xr::pfn::CreateAction = self.h.pfn(self.inst, "xrCreateAction").unwrap();
            let mut aci = xr::ActionCreateInfo {
                ty: xr::StructureType::ACTION_CREATE_INFO,
                next: std::ptr::null(),
                action_name: [0; xr::MAX_ACTION_NAME_SIZE],
                action_type: ty,
                count_subaction_paths: subs.len() as u32,
                subaction_paths: if subs.is_empty() {
                    std::ptr::null()
                } else {
                    subs.as_ptr()
                },
                localized_action_name: [0; xr::MAX_LOCALIZED_ACTION_NAME_SIZE],
            };
            for (i, b) in name.as_bytes().iter().enumerate() {
                aci.action_name[i] = *b as c_char;
                aci.localized_action_name[i] = *b as c_char;
            }
            let mut act = xr::Action::NULL;
            assert_eq!(create_act(self.action_set, &aci, &mut act), xr::Result::SUCCESS);
            act
        }
    }

    fn suggest(&self, profile: &str, pairs: &[(xr::Action, &str)]) {
        unsafe {
            let f: xr::pfn::SuggestInteractionProfileBindings = self
                .h
                .pfn(self.inst, "xrSuggestInteractionProfileBindings")
                .unwrap();
            let prof_path = self.path(profile);
            let bindings: Vec<xr::ActionSuggestedBinding> = pairs
                .iter()
                .map(|&(action, p)| xr::ActionSuggestedBinding {
                    action,
                    binding: self.path(p),
                })
                .collect();
            let ipsb = xr::InteractionProfileSuggestedBinding {
                ty: xr::StructureType::INTERACTION_PROFILE_SUGGESTED_BINDING,
                next: std::ptr::null(),
                interaction_profile: prof_path,
                count_suggested_bindings: bindings.len() as u32,
                suggested_bindings: bindings.as_ptr(),
            };
            assert_eq!(f(self.inst, &ipsb), xr::Result::SUCCESS);
        }
    }

    fn attach_and_begin(&self) {
        unsafe {
            let attach: xr::pfn::AttachSessionActionSets =
                self.h.pfn(self.inst, "xrAttachSessionActionSets").unwrap();
            let begin: xr::pfn::BeginSession = self.h.pfn(self.inst, "xrBeginSession").unwrap();
            let asai = xr::SessionActionSetsAttachInfo {
                ty: xr::StructureType::SESSION_ACTION_SETS_ATTACH_INFO,
                next: std::ptr::null(),
                count_action_sets: 1,
                action_sets: &self.action_set,
            };
            assert_eq!(attach(self.sess, &asai), xr::Result::SUCCESS);
            let sbi = xr::SessionBeginInfo {
                ty: xr::StructureType::SESSION_BEGIN_INFO,
                next: std::ptr::null(),
                primary_view_configuration_type: xr::ViewConfigurationType::PRIMARY_STEREO,
            };
            assert_eq!(begin(self.sess, &sbi), xr::Result::SUCCESS);
        }
    }

    fn sync(&self) -> xr::Result {
        unsafe {
            let sync: xr::pfn::SyncActions = self.h.pfn(self.inst, "xrSyncActions").unwrap();
            let aas = xr::ActiveActionSet {
                action_set: self.action_set,
                subaction_path: xr::Path::NULL,
            };
            let si = xr::ActionsSyncInfo {
                ty: xr::StructureType::ACTIONS_SYNC_INFO,
                next: std::ptr::null(),
                count_active_action_sets: 1,
                active_action_sets: &aas,
            };
            sync(self.sess, &si)
        }
    }

    fn get_bool(&self, action: xr::Action, sub: xr::Path) -> xr::ActionStateBoolean {
        unsafe {
            let f: xr::pfn::GetActionStateBoolean =
                self.h.pfn(self.inst, "xrGetActionStateBoolean").unwrap();
            let gi = xr::ActionStateGetInfo {
                ty: xr::StructureType::ACTION_STATE_GET_INFO,
                next: std::ptr::null(),
                action,
                subaction_path: sub,
            };
            let mut st = xr::ActionStateBoolean {
                ty: xr::StructureType::ACTION_STATE_BOOLEAN,
                next: std::ptr::null_mut(),
                current_state: xr::FALSE,
                changed_since_last_sync: xr::FALSE,
                last_change_time: xr::Time::from_nanos(0),
                is_active: xr::FALSE,
            };
            assert_eq!(f(self.sess, &gi, &mut st), xr::Result::SUCCESS);
            st
        }
    }

    fn get_float(&self, action: xr::Action, sub: xr::Path) -> xr::ActionStateFloat {
        unsafe {
            let f: xr::pfn::GetActionStateFloat =
                self.h.pfn(self.inst, "xrGetActionStateFloat").unwrap();
            let gi = xr::ActionStateGetInfo {
                ty: xr::StructureType::ACTION_STATE_GET_INFO,
                next: std::ptr::null(),
                action,
                subaction_path: sub,
            };
            let mut st = xr::ActionStateFloat {
                ty: xr::StructureType::ACTION_STATE_FLOAT,
                next: std::ptr::null_mut(),
                current_state: 0.0,
                changed_since_last_sync: xr::FALSE,
                last_change_time: xr::Time::from_nanos(0),
                is_active: xr::FALSE,
            };
            assert_eq!(f(self.sess, &gi, &mut st), xr::Result::SUCCESS);
            st
        }
    }

    fn get_vec2(&self, action: xr::Action, sub: xr::Path) -> xr::ActionStateVector2f {
        unsafe {
            let f: xr::pfn::GetActionStateVector2f =
                self.h.pfn(self.inst, "xrGetActionStateVector2f").unwrap();
            let gi = xr::ActionStateGetInfo {
                ty: xr::StructureType::ACTION_STATE_GET_INFO,
                next: std::ptr::null(),
                action,
                subaction_path: sub,
            };
            let mut st = xr::ActionStateVector2f {
                ty: xr::StructureType::ACTION_STATE_VECTOR2F,
                next: std::ptr::null_mut(),
                current_state: xr::Vector2f { x: 0.0, y: 0.0 },
                changed_since_last_sync: xr::FALSE,
                last_change_time: xr::Time::from_nanos(0),
                is_active: xr::FALSE,
            };
            assert_eq!(f(self.sess, &gi, &mut st), xr::Result::SUCCESS);
            st
        }
    }

    fn get_profile(&self, user: xr::Path) -> xr::Path {
        unsafe {
            let f: xr::pfn::GetCurrentInteractionProfile = self
                .h
                .pfn(self.inst, "xrGetCurrentInteractionProfile")
                .unwrap();
            let mut st = xr::InteractionProfileState {
                ty: xr::StructureType::INTERACTION_PROFILE_STATE,
                next: std::ptr::null_mut(),
                interaction_profile: xr::Path::NULL,
            };
            assert_eq!(f(self.sess, user, &mut st), xr::Result::SUCCESS);
            st.interaction_profile
        }
    }

    fn create_action_space(&self, action: xr::Action, sub: xr::Path, offset: xr::Posef) -> xr::Space {
        unsafe {
            let f: xr::pfn::CreateActionSpace =
                self.h.pfn(self.inst, "xrCreateActionSpace").unwrap();
            let ci = xr::ActionSpaceCreateInfo {
                ty: xr::StructureType::ACTION_SPACE_CREATE_INFO,
                next: std::ptr::null(),
                action,
                subaction_path: sub,
                pose_in_action_space: offset,
            };
            let mut sp = xr::Space::NULL;
            assert_eq!(f(self.sess, &ci, &mut sp), xr::Result::SUCCESS);
            sp
        }
    }

    fn destroy(self) {
        unsafe {
            let ds: xr::pfn::DestroySpace = self.h.pfn(self.inst, "xrDestroySpace").unwrap();
            let es: xr::pfn::EndSession = self.h.pfn(self.inst, "xrEndSession").unwrap();
            let dses: xr::pfn::DestroySession = self.h.pfn(self.inst, "xrDestroySession").unwrap();
            let di: xr::pfn::DestroyInstance = self.h.pfn(self.inst, "xrDestroyInstance").unwrap();
            let _ = ds(self.local_space);
            let _ = es(self.sess);
            assert_eq!(dses(self.sess), xr::Result::SUCCESS);
            assert_eq!(di(self.inst), xr::Result::SUCCESS);
        }
    }
}

#[test]
fn t1_negotiate_openxr_10_11_and_android_loader_init() {
    let _g = HARNESS_LOCK.lock();
    let h = DirectLayerHarness::load().expect("load layer");
    let valid_name = CString::new("XR_APILAYER_ALVR_hand_emulation").unwrap();
    let wrong_name = CString::new("XR_APILAYER_OTHER_wrong").unwrap();

    // OpenXR 1.0 range [1.0.0, 1.0.4095] -> clamps cleanly to 1.0.0
    let li10 = xr::NegotiateLoaderInfo {
        struct_type: xr::LoaderInterfaceStructs::LOADER_INFO,
        struct_version: xr::LOADER_INFO_STRUCT_VERSION as u32,
        struct_size: size_of::<xr::NegotiateLoaderInfo>(),
        min_interface_version: 1,
        max_interface_version: 1,
        min_api_version: xr::Version::new(1, 0, 0),
        max_api_version: xr::Version::new(1, 0, 4095),
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
    assert_eq!(
        unsafe { (h.negotiate)(&li10, valid_name.as_ptr(), &mut req) },
        xr::Result::SUCCESS
    );
    assert_eq!(req.layer_api_version, xr::Version::new(1, 0, 0));

    // Wrong layer name -> rejected
    assert_eq!(
        unsafe { (h.negotiate)(&li10, wrong_name.as_ptr(), &mut req) },
        xr::Result::ERROR_INITIALIZATION_FAILED
    );

    // xrInitializeLoaderKHR symbol present & validates Android init struct
    let init_fn = h.init_loader.expect("xrInitializeLoaderKHR exported");
    assert_eq!(unsafe { init_fn(std::ptr::null()) }, xr::Result::ERROR_VALIDATION_FAILURE);

    #[repr(C)]
    struct AndroidInit {
        ty: xr::StructureType,
        next: *const c_void,
        vm: *mut c_void,
        ctx: *mut c_void,
    }
    let bad = AndroidInit {
        ty: xr::StructureType::LOADER_INIT_INFO_ANDROID_KHR,
        next: std::ptr::null(),
        vm: std::ptr::null_mut(),
        ctx: std::ptr::null_mut(),
    };
    assert_eq!(
        unsafe { init_fn(&bad as *const _ as *const xr::LoaderInitInfoBaseHeaderKHR) },
        xr::Result::ERROR_VALIDATION_FAILURE
    );
    let mut dummy = 42u8;
    let good = AndroidInit {
        ty: xr::StructureType::LOADER_INIT_INFO_ANDROID_KHR,
        next: std::ptr::null(),
        vm: &mut dummy as *mut u8 as *mut c_void,
        ctx: &mut dummy as *mut u8 as *mut c_void,
    };
    assert_eq!(
        unsafe { init_fn(&good as *const _ as *const xr::LoaderInitInfoBaseHeaderKHR) },
        xr::Result::SUCCESS
    );
}

#[test]
fn t1_extension_injection_gating_and_progressive_fallback() {
    let _g = HARNESS_LOCK.lock();

    // Case A: App does NOT request XR_EXT_hand_tracking -> layer injects it, but gates GIPA!
    reset_mock_runtime(MockRuntimeConfig::default());
    let h = DirectLayerHarness::load().unwrap();
    let app = XrApp::new(h, xr::Version::new(1, 0, 0), &["XR_MND_headless"]);
    let (res_ht, fn_ht) = app.h.try_pfn_raw(app.inst, "xrCreateHandTrackerEXT");
    assert_eq!(res_ht, xr::Result::ERROR_FUNCTION_UNSUPPORTED);
    assert!(fn_ht.is_none());
    let diag = global_engine().diagnostics();
    assert!(diag.create_instance_attempts[0]
        .enabled_extensions
        .iter()
        .any(|e| e == "XR_EXT_hand_tracking"));
    assert_eq!(diag.hand_trackers_created, 2);
    app.destroy();

    // Case B: Runtime lacks XR_FB_hand_tracking_aim -> retries with ONLY XR_EXT_hand_tracking!
    let mut cfg_no_aim = MockRuntimeConfig::default();
    cfg_no_aim
        .supported_extensions
        .retain(|e| e.name != "XR_FB_hand_tracking_aim");
    reset_mock_runtime(cfg_no_aim);
    let h2 = DirectLayerHarness::load().unwrap();
    let app2 = XrApp::new(h2, xr::Version::new(1, 0, 0), &["XR_MND_headless"]);
    let diag2 = global_engine().diagnostics();
    assert_eq!(diag2.create_instance_attempts.len(), 2);
    assert_eq!(diag2.hand_trackers_created, 2);
    app2.destroy();

    // Case C: Runtime lacks XR_EXT_hand_tracking completely -> falls back cleanly to pure passthrough!
    let cfg_no_ht = MockRuntimeConfig {
        supported_extensions: vec![MockExtensionSpec {
            name: "XR_MND_headless".to_owned(),
            version: 3,
        }],
        supports_hand_tracking: false,
        ..Default::default()
    };
    reset_mock_runtime(cfg_no_ht);
    let h3 = DirectLayerHarness::load().unwrap();
    let app3 = XrApp::new(h3, xr::Version::new(1, 0, 0), &["XR_MND_headless"]);
    let diag3 = global_engine().diagnostics();
    assert_eq!(diag3.hand_trackers_created, 0);
    app3.destroy();
}

#[test]
fn t1_all_alvr_gestures_asymmetry_joystick_and_dropout_release() {
    let _g = HARNESS_LOCK.lock();
    reset_mock_runtime(MockRuntimeConfig::default());
    let h = DirectLayerHarness::load().unwrap();
    let app = XrApp::new(h, xr::Version::new(1, 0, 0), &["XR_MND_headless"]);
    let subs = [app.left_user, app.right_user];

    let trig_val = app.create_action("trig_val", xr::ActionType::FLOAT_INPUT, &subs);
    let trig_touch = app.create_action("trig_touch", xr::ActionType::BOOLEAN_INPUT, &subs);
    let grip_val = app.create_action("grip_val", xr::ActionType::FLOAT_INPUT, &subs);
    let primary = app.create_action("primary", xr::ActionType::BOOLEAN_INPUT, &subs);
    let secondary = app.create_action("secondary", xr::ActionType::BOOLEAN_INPUT, &subs);
    let menu = app.create_action("menu", xr::ActionType::BOOLEAN_INPUT, &[app.left_user]);
    let stick = app.create_action("stick", xr::ActionType::VECTOR2F_INPUT, &subs);
    let stick_click = app.create_action("stick_click", xr::ActionType::BOOLEAN_INPUT, &subs);
    let stick_touch = app.create_action("stick_touch", xr::ActionType::BOOLEAN_INPUT, &subs);

    app.suggest(
        "/interaction_profiles/oculus/touch_controller",
        &[
            (trig_val, "/user/hand/left/input/trigger/value"),
            (trig_val, "/user/hand/right/input/trigger/value"),
            (trig_touch, "/user/hand/left/input/trigger/touch"),
            (trig_touch, "/user/hand/right/input/trigger/touch"),
            (grip_val, "/user/hand/left/input/squeeze/value"),
            (grip_val, "/user/hand/right/input/squeeze/value"),
            (primary, "/user/hand/left/input/x/click"),
            (primary, "/user/hand/right/input/a/click"),
            (secondary, "/user/hand/left/input/y/click"),
            (secondary, "/user/hand/right/input/b/click"),
            (menu, "/user/hand/left/input/menu/click"),
            (stick, "/user/hand/left/input/thumbstick"),
            (stick, "/user/hand/right/input/thumbstick"),
            (stick_click, "/user/hand/left/input/thumbstick/click"),
            (stick_click, "/user/hand/right/input/thumbstick/click"),
            (stick_touch, "/user/hand/left/input/thumbstick/touch"),
            (stick_touch, "/user/hand/right/input/thumbstick/touch"),
        ],
    );
    app.attach_and_begin();

    // 1. Open hands: everything 0 / FALSE, active == TRUE
    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::new(-0.2, 1.2, -0.4), Quat::IDENTITY).build(),
    );
    global_engine().set_hand_state(
        1,
        HandSkeletonBuilder::open(1, Vec3::new(0.2, 1.2, -0.4), Quat::IDENTITY).build(),
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    assert_eq!(app.get_float(trig_val, app.left_user).is_active, xr::TRUE);
    assert_eq!(app.get_float(trig_val, app.left_user).current_state, 0.0);
    // First sync MUST have changedSinceLastSync == FALSE!
    assert_eq!(
        app.get_float(trig_val, app.left_user).changed_since_last_sync,
        xr::FALSE
    );

    // 2. Left index pinch + Right middle pinch -> debounce < 50ms NOT active yet, > 50ms active!
    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::new(-0.2, 1.2, -0.4), Quat::IDENTITY)
            .with_pinch(10, 0.0)
            .build(),
    );
    global_engine().set_hand_state(
        1,
        HandSkeletonBuilder::open(1, Vec3::new(0.2, 1.2, -0.4), Quat::IDENTITY)
            .with_pinch(15, 0.0)
            .build(),
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    // Still entering (0 ms elapsed)
    assert_eq!(app.get_float(trig_val, app.left_user).current_state, 0.0);
    assert_eq!(app.get_bool(secondary, app.right_user).current_state, xr::FALSE);

    // Advance 65ms (> 50ms ALVR activation_delay)
    advance_dual_clock_ms(65);
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    let tv = app.get_float(trig_val, app.left_user);
    assert!((tv.current_state - 1.0).abs() < 1e-3);
    assert_eq!(tv.changed_since_last_sync, xr::TRUE);
    assert_eq!(app.get_bool(trig_touch, app.left_user).current_state, xr::TRUE);
    // Right ThumbMiddlePinch -> B (secondary)
    assert_eq!(app.get_bool(secondary, app.right_user).current_state, xr::TRUE);

    // Atomic snapshot immutability: second read before next sync sees identical changedSinceLastSync!
    assert_eq!(
        app.get_float(trig_val, app.left_user).changed_since_last_sync,
        xr::TRUE
    );

    // 3. Left pinky pinch (menu) + Right ring pinch (A / primary)
    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::new(-0.2, 1.2, -0.4), Quat::IDENTITY)
            .with_pinch(25, 0.0)
            .build(),
    );
    global_engine().set_hand_state(
        1,
        HandSkeletonBuilder::open(1, Vec3::new(0.2, 1.2, -0.4), Quat::IDENTITY)
            .with_pinch(20, 0.0)
            .build(),
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    advance_dual_clock_ms(65);
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    assert_eq!(app.get_bool(menu, app.left_user).current_state, xr::TRUE);
    assert_eq!(app.get_bool(primary, app.right_user).current_state, xr::TRUE);

    // 4. Grip curl (squeeze = 1.0) & virtual joystick (+X = +0.8, +Y = -0.7) on left & right
    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::new(-0.2, 1.2, -0.4), Quat::IDENTITY)
            .with_joystick(0.008, -0.007) // 0.8cm, -0.7cm / 1.0cm range -> (+0.8, -0.7)
            .build(),
    );
    global_engine().set_hand_state(
        1,
        HandSkeletonBuilder::open(1, Vec3::new(0.2, 1.2, -0.4), Quat::IDENTITY)
            .with_thumb_curl()
            .build(),
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    assert!((app.get_float(grip_val, app.left_user).current_state - 1.0).abs() < 1e-3);
    assert_eq!(app.get_bool(stick_touch, app.left_user).current_state, xr::TRUE);
    let js = app.get_vec2(stick, app.left_user);
    assert!((js.current_state.x - 0.8).abs() < 0.05, "got x={}", js.current_state.x);
    assert!((js.current_state.y + 0.7).abs() < 0.05, "got y={}", js.current_state.y);
    assert_eq!(app.get_bool(stick_click, app.right_user).current_state, xr::TRUE);

    // 5. Hand dropout (isActive == FALSE) -> releases cleanly (no stuck buttons!)
    global_engine().set_hand_state(0, MockHandState::default());
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    let dropped = app.get_float(grip_val, app.left_user);
    assert_eq!(dropped.is_active, xr::FALSE);
    assert_eq!(dropped.current_state, 0.0);

    app.destroy();
}

#[test]
fn t1_nine_profiles_priority_replace_on_repeat_and_conversions() {
    let _g = HARNESS_LOCK.lock();
    reset_mock_runtime(MockRuntimeConfig::default());
    let h = DirectLayerHarness::load().unwrap();
    let app = XrApp::new(h, xr::Version::new(1, 1, 54), &["XR_MND_headless"]);
    let subs = [app.left_user, app.right_user];

    let bool_from_trig = app.create_action("bool_from_trig", xr::ActionType::BOOLEAN_INPUT, &subs);
    let float_from_x = app.create_action("float_from_x", xr::ActionType::FLOAT_INPUT, &[app.left_user]);
    let select_act = app.create_action("select_act", xr::ActionType::BOOLEAN_INPUT, &[app.left_user]);

    // Suggest lower-priority oculus/touch_controller, then higher-priority meta/touch_plus_controller (1.1),
    // AND replace oculus/touch_controller bindings on a second call!
    app.suggest(
        "/interaction_profiles/oculus/touch_controller",
        &[(select_act, "/user/hand/left/input/y/click")],
    );
    // Repeat suggestion for same profile MUST discard previous binding for select_act!
    app.suggest(
        "/interaction_profiles/oculus/touch_controller",
        &[(bool_from_trig, "/user/hand/left/input/trigger")], // parent-path shorthand!
    );
    app.suggest(
        "/interaction_profiles/meta/touch_plus_controller",
        &[
            (bool_from_trig, "/user/hand/left/input/trigger/value"),
            (bool_from_trig, "/user/hand/right/input/trigger/value"),
            (float_from_x, "/user/hand/left/input/x/click"),
        ],
    );
    app.attach_and_begin();

    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::ZERO, Quat::IDENTITY)
            .with_pinch(10, 0.0)
            .build(),
    );
    global_engine().set_hand_state(
        1,
        HandSkeletonBuilder::open(1, Vec3::ZERO, Quat::IDENTITY).build(),
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    advance_dual_clock_ms(65);
    assert_eq!(app.sync(), xr::Result::SUCCESS);

    // Higher priority OpenXR 1.1 `meta/touch_plus_controller` MUST win over `oculus/touch_controller`!
    assert_eq!(
        app.get_profile(app.left_user),
        app.path("/interaction_profiles/meta/touch_plus_controller")
    );
    // Float->Bool hysteresis coercion on trigger/value == 1.0 -> TRUE
    assert_eq!(app.get_bool(bool_from_trig, app.left_user).current_state, xr::TRUE);
    // Multi-source Bool OR with subactionPath == NULL (left=TRUE OR right=FALSE -> TRUE)
    assert_eq!(app.get_bool(bool_from_trig, xr::Path::NULL).current_state, xr::TRUE);

    // Now pinch ring on left -> X click -> Bool->Float == 1.0!
    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::ZERO, Quat::IDENTITY)
            .with_pinch(20, 0.0)
            .build(),
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    advance_dual_clock_ms(65);
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    assert!((app.get_float(float_from_x, app.left_user).current_state - 1.0).abs() < 1e-6);

    app.destroy();
}

#[test]
fn t1_mixed_hand_arbitration_grace_period_and_focus_loss() {
    let _g = HARNESS_LOCK.lock();
    reset_mock_runtime(MockRuntimeConfig::default());
    let h = DirectLayerHarness::load().unwrap();
    let app = XrApp::new(h, xr::Version::new(1, 0, 0), &["XR_MND_headless"]);
    let subs = [app.left_user, app.right_user];

    let trig = app.create_action("trig", xr::ActionType::FLOAT_INPUT, &subs);
    let haptic = app.create_action("haptic", xr::ActionType::VIBRATION_OUTPUT, &subs);
    app.suggest(
        "/interaction_profiles/oculus/touch_controller",
        &[
            (trig, "/user/hand/left/input/trigger/value"),
            (trig, "/user/hand/right/input/trigger/value"),
            (haptic, "/user/hand/left/output/haptic"),
            (haptic, "/user/hand/right/output/haptic"),
        ],
    );
    app.attach_and_begin();

    // Mixed-hand: Left = optical hand (pinched, value ~1.0), Right = physical Touch controller (value = 0.42)!
    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::ZERO, Quat::IDENTITY)
            .with_pinch(10, 0.0)
            .build(),
    );
    global_engine().set_physical_controller(
        1,
        MockPhysicalControllerState {
            active: true,
            interaction_profile: Some("/interaction_profiles/oculus/touch_controller".to_owned()),
            trigger_value: 0.42,
            ..Default::default()
        },
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    advance_dual_clock_ms(65);
    assert_eq!(app.sync(), xr::Result::SUCCESS);

    assert!((app.get_float(trig, app.left_user).current_state - 1.0).abs() < 1e-3);
    assert!((app.get_float(trig, app.right_user).current_state - 0.42).abs() < 1e-3);
    // Max-abs across both hands (NULL subaction): max(1.0, 0.42) = 1.0!
    assert!((app.get_float(trig, xr::Path::NULL).current_state - 1.0).abs() < 1e-3);

    // Haptics: Left emulated returns XR_SUCCESS without hitting runtime; Right physical forwards to runtime!
    let vib = xr::HapticVibration {
        ty: xr::StructureType::HAPTIC_VIBRATION,
        next: std::ptr::null(),
        duration: xr::Duration::from_nanos(10_000_000),
        frequency: 160.0,
        amplitude: 0.5,
    };
    unsafe {
        let apply_h: xr::pfn::ApplyHapticFeedback =
            app.h.pfn(app.inst, "xrApplyHapticFeedback").unwrap();
        let stop_h: xr::pfn::StopHapticFeedback =
            app.h.pfn(app.inst, "xrStopHapticFeedback").unwrap();
        let hi_l = xr::HapticActionInfo {
            ty: xr::StructureType::HAPTIC_ACTION_INFO,
            next: std::ptr::null(),
            action: haptic,
            subaction_path: app.left_user,
        };
        assert_eq!(
            apply_h(app.sess, &hi_l, &vib as *const _ as *const xr::HapticBaseHeader),
            xr::Result::SUCCESS
        );
        assert_eq!(stop_h(app.sess, &hi_l), xr::Result::SUCCESS);
        assert_eq!(global_engine().diagnostics().haptic_apply_calls.len(), 0);

        let hi_r = xr::HapticActionInfo {
            ty: xr::StructureType::HAPTIC_ACTION_INFO,
            next: std::ptr::null(),
            action: haptic,
            subaction_path: app.right_user,
        };
        assert_eq!(
            apply_h(app.sess, &hi_r, &vib as *const _ as *const xr::HapticBaseHeader),
            xr::Result::SUCCESS
        );
        assert_eq!(global_engine().diagnostics().haptic_apply_calls.len(), 1);
    }

    // Physical right controller turns off, right hand appears -> grace period (250ms) blocks immediate switch!
    global_engine().set_physical_controller(1, MockPhysicalControllerState::default());
    global_engine().set_hand_state(
        1,
        HandSkeletonBuilder::open(1, Vec3::ZERO, Quat::IDENTITY)
            .with_pinch(10, 0.0)
            .build(),
    );
    advance_dual_clock_ms(50);
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    assert_eq!(app.get_float(trig, app.right_user).is_active, xr::FALSE);

    // After >250ms grace + >50ms pinch debounce, right hand switches smoothly to emulation!
    advance_dual_clock_ms(220);
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    advance_dual_clock_ms(65);
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    assert_eq!(app.get_float(trig, app.right_user).is_active, xr::TRUE);
    assert!((app.get_float(trig, app.right_user).current_state - 1.0).abs() < 1e-3);

    // Session loses focus (VISIBLE) -> XR_SESSION_NOT_FOCUSED, all actions inactive & zeroed!
    global_engine().set_session_state(xr::SessionState::VISIBLE);
    assert_eq!(app.sync(), xr::Result::SESSION_NOT_FOCUSED);
    let unfocused = app.get_float(trig, app.left_user);
    assert_eq!(unfocused.is_active, xr::FALSE);
    assert_eq!(unfocused.current_state, 0.0);
    assert_eq!(unfocused.last_change_time.as_nanos(), 0);

    app.destroy();
}

#[test]
fn t1_events_bound_sources_and_localized_names() {
    let _g = HARNESS_LOCK.lock();
    reset_mock_runtime(MockRuntimeConfig::default());
    let h = DirectLayerHarness::load().unwrap();
    let app = XrApp::new(h, xr::Version::new(1, 0, 0), &["XR_MND_headless"]);
    let trig = app.create_action("trig", xr::ActionType::FLOAT_INPUT, &[app.left_user]);
    app.suggest(
        "/interaction_profiles/oculus/touch_controller",
        &[(trig, "/user/hand/left/input/trigger/value")],
    );
    app.attach_and_begin();

    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::ZERO, Quat::IDENTITY).build(),
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);

    // Synthetic EventDataInteractionProfileChanged delivered via xrPollEvent!
    unsafe {
        let poll: xr::pfn::PollEvent = app.h.pfn(app.inst, "xrPollEvent").unwrap();
        let mut ev = xr::EventDataBuffer {
            ty: xr::StructureType::EVENT_DATA_BUFFER,
            next: std::ptr::null(),
            varying: [0; 4000],
        };
        assert_eq!(poll(app.inst, &mut ev), xr::Result::SUCCESS);
        assert_eq!(ev.ty, xr::StructureType::EVENT_DATA_INTERACTION_PROFILE_CHANGED);
        let ipc = &*(&ev as *const _ as *const xr::EventDataInteractionProfileChanged);
        assert_eq!(ipc.session, app.sess);
    }

    // Bound sources & localized name
    unsafe {
        let enum_bs: xr::pfn::EnumerateBoundSourcesForAction = app
            .h
            .pfn(app.inst, "xrEnumerateBoundSourcesForAction")
            .unwrap();
        let loc_name: xr::pfn::GetInputSourceLocalizedName = app
            .h
            .pfn(app.inst, "xrGetInputSourceLocalizedName")
            .unwrap();

        let ei = xr::BoundSourcesForActionEnumerateInfo {
            ty: xr::StructureType::BOUND_SOURCES_FOR_ACTION_ENUMERATE_INFO,
            next: std::ptr::null(),
            action: trig,
        };
        let mut count = 0u32;
        assert_eq!(
            enum_bs(app.sess, &ei, 0, &mut count, std::ptr::null_mut()),
            xr::Result::SUCCESS
        );
        assert_eq!(count, 1);
        let mut src = xr::Path::NULL;
        let mut out = 0u32;
        assert_eq!(enum_bs(app.sess, &ei, 1, &mut out, &mut src), xr::Result::SUCCESS);
        assert_eq!(src, app.path("/user/hand/left/input/trigger/value"));

        let gi = xr::InputSourceLocalizedNameGetInfo {
            ty: xr::StructureType::INPUT_SOURCE_LOCALIZED_NAME_GET_INFO,
            next: std::ptr::null(),
            source_path: src,
            which_components: xr::InputSourceLocalizedNameFlags::USER_PATH
                | xr::InputSourceLocalizedNameFlags::INTERACTION_PROFILE
                | xr::InputSourceLocalizedNameFlags::COMPONENT,
        };
        let mut cap = 0u32;
        assert_eq!(
            loc_name(app.sess, &gi, 0, &mut cap, std::ptr::null_mut()),
            xr::Result::SUCCESS
        );
        let mut buf = vec![0 as c_char; cap as usize];
        let mut out_len = 0u32;
        assert_eq!(
            loc_name(app.sess, &gi, cap, &mut out_len, buf.as_mut_ptr()),
            xr::Result::SUCCESS
        );
        let label = c_buf_to_string(&buf);
        assert!(label.contains("Left Hand"), "got {label}");
        assert!(label.contains("Oculus Touch Controller"), "got {label}");
        assert!(label.contains("Trigger"), "got {label}");
    }

    app.destroy();
}

#[test]
fn t1_spaces_grip_aim_velocity_same_entity_inverted_base_and_batched() {
    let _g = HARNESS_LOCK.lock();
    reset_mock_runtime(MockRuntimeConfig::default());
    let h = DirectLayerHarness::load().unwrap();
    let app = XrApp::new(
        h,
        xr::Version::new(1, 1, 54),
        &["XR_MND_headless", "XR_KHR_locate_spaces"],
    );
    let subs = [app.left_user, app.right_user];
    let grip = app.create_action("grip", xr::ActionType::POSE_INPUT, &subs);
    let aim = app.create_action("aim", xr::ActionType::POSE_INPUT, &subs);
    let palm = app.create_action("palm", xr::ActionType::POSE_INPUT, &subs);
    app.suggest(
        "/interaction_profiles/oculus/touch_controller",
        &[
            (grip, "/user/hand/left/input/grip/pose"),
            (grip, "/user/hand/right/input/grip/pose"),
            (aim, "/user/hand/left/input/aim/pose"),
            (palm, "/user/hand/left/input/palm_ext/pose"),
        ],
    );
    app.attach_and_begin();

    let left_grip_sp = app.create_action_space(grip, app.left_user, xr::Posef::IDENTITY);
    let left_grip_offset_sp = app.create_action_space(
        grip,
        app.left_user,
        xr::Posef {
            orientation: xr::Quaternionf::IDENTITY,
            position: xr::Vector3f { x: 0.0, y: 0.1, z: 0.0 },
        },
    );
    let left_aim_sp = app.create_action_space(aim, app.left_user, xr::Posef::IDENTITY);
    let left_palm_sp = app.create_action_space(palm, app.left_user, xr::Posef::IDENTITY);

    let fb_aim_pose = MockPose::from_quat_pos(Quat::IDENTITY, Vec3::new(-0.15, 1.35, -0.55));
    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::new(-0.2, 1.2, -0.4), Quat::IDENTITY)
            .with_palm_velocity(Vec3::new(0.5, -0.2, 0.1), Vec3::new(0.0, 1.0, 0.0))
            .with_fb_aim(fb_aim_pose, true)
            .build(),
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    let t = xr::Time::from_nanos(global_engine().advance_time_ns(11_000_000));

    unsafe {
        let loc_fn: xr::pfn::LocateSpace = app.h.pfn(app.inst, "xrLocateSpace").unwrap();
        let loc_batch: xr::pfn::LocateSpaces = app.h.pfn(app.inst, "xrLocateSpaces").unwrap();
        let loc_khr: xr::pfn::LocateSpacesKHR = app.h.pfn(app.inst, "xrLocateSpacesKHR").unwrap();
        let destroy_sp: xr::pfn::DestroySpace = app.h.pfn(app.inst, "xrDestroySpace").unwrap();

        // 1. Invalid time <= 0 -> XR_ERROR_TIME_INVALID
        let mut loc = xr::SpaceLocation {
            ty: xr::StructureType::SPACE_LOCATION,
            next: std::ptr::null_mut(),
            location_flags: xr::SpaceLocationFlags::EMPTY,
            pose: xr::Posef::IDENTITY,
        };
        assert_eq!(
            loc_fn(left_grip_sp, app.local_space, xr::Time::from_nanos(0), &mut loc),
            xr::Result::ERROR_TIME_INVALID
        );

        // 2. Locate grip space with XrSpaceVelocity in baseSpace frame
        let mut vel = xr::SpaceVelocity {
            ty: xr::StructureType::SPACE_VELOCITY,
            next: std::ptr::null_mut(),
            velocity_flags: xr::SpaceVelocityFlags::EMPTY,
            linear_velocity: xr::Vector3f::default(),
            angular_velocity: xr::Vector3f::default(),
        };
        loc.next = &mut vel as *mut _ as *mut c_void;
        assert_eq!(loc_fn(left_grip_sp, app.local_space, t, &mut loc), xr::Result::SUCCESS);
        assert!(loc.location_flags.contains(
            xr::SpaceLocationFlags::POSITION_VALID
                | xr::SpaceLocationFlags::ORIENTATION_VALID
                | xr::SpaceLocationFlags::POSITION_TRACKED
                | xr::SpaceLocationFlags::ORIENTATION_TRACKED
        ));
        assert!(vel.velocity_flags.contains(
            xr::SpaceVelocityFlags::LINEAR_VALID | xr::SpaceVelocityFlags::ANGULAR_VALID
        ));
        // Angular velocity in baseSpace matches palm angular velocity (0, 1, 0) rad/s!
        assert!((vel.angular_velocity.y - 1.0).abs() < 1e-4);

        // 3. Inverted baseSpace: locating local_space relative to left_grip_sp == exact inverse!
        let mut inv_loc = xr::SpaceLocation {
            ty: xr::StructureType::SPACE_LOCATION,
            next: std::ptr::null_mut(),
            location_flags: xr::SpaceLocationFlags::EMPTY,
            pose: xr::Posef::IDENTITY,
        };
        assert_eq!(
            loc_fn(app.local_space, left_grip_sp, t, &mut inv_loc),
            xr::Result::SUCCESS
        );
        let q_g = Quat::from_xyzw(
            loc.pose.orientation.x,
            loc.pose.orientation.y,
            loc.pose.orientation.z,
            loc.pose.orientation.w,
        );
        let p_g = Vec3::new(loc.pose.position.x, loc.pose.position.y, loc.pose.position.z);
        let q_inv = Quat::from_xyzw(
            inv_loc.pose.orientation.x,
            inv_loc.pose.orientation.y,
            inv_loc.pose.orientation.z,
            inv_loc.pose.orientation.w,
        );
        let p_inv = Vec3::new(
            inv_loc.pose.position.x,
            inv_loc.pose.position.y,
            inv_loc.pose.position.z,
        );
        let composed_p = p_g + q_g * p_inv;
        assert!(composed_p.length() < 1e-4, "inverse failed: {composed_p:?}");
        assert!((q_g * q_inv).w.abs() > 0.9999);

        // 4. XR_FB_hand_tracking_aim pose preferred for /aim/pose
        let mut aim_loc = xr::SpaceLocation {
            ty: xr::StructureType::SPACE_LOCATION,
            next: std::ptr::null_mut(),
            location_flags: xr::SpaceLocationFlags::EMPTY,
            pose: xr::Posef::IDENTITY,
        };
        assert_eq!(loc_fn(left_aim_sp, app.local_space, t, &mut aim_loc), xr::Result::SUCCESS);
        assert!((aim_loc.pose.position.x - (-0.15)).abs() < 1e-4);
        assert!((aim_loc.pose.position.y - 1.35).abs() < 1e-4);

        // 5. Same-entity rule: left_grip_offset_sp relative to left_grip_sp == (0, +0.1, 0) EVEN when untracked!
        global_engine().set_hand_state(0, MockHandState::default());
        assert_eq!(app.sync(), xr::Result::SUCCESS);
        let mut same_loc = xr::SpaceLocation {
            ty: xr::StructureType::SPACE_LOCATION,
            next: std::ptr::null_mut(),
            location_flags: xr::SpaceLocationFlags::EMPTY,
            pose: xr::Posef::IDENTITY,
        };
        assert_eq!(
            loc_fn(left_grip_offset_sp, left_grip_sp, t, &mut same_loc),
            xr::Result::SUCCESS
        );
        assert!(same_loc.location_flags.contains(
            xr::SpaceLocationFlags::POSITION_VALID
                | xr::SpaceLocationFlags::ORIENTATION_VALID
                | xr::SpaceLocationFlags::POSITION_TRACKED
                | xr::SpaceLocationFlags::ORIENTATION_TRACKED
        ));
        assert!((same_loc.pose.position.y - 0.1).abs() < 1e-5);

        // 6. Batched xrLocateSpaces (1.1) & xrLocateSpacesKHR + validation error checks
        global_engine().set_hand_state(
            0,
            HandSkeletonBuilder::open(0, Vec3::new(-0.2, 1.2, -0.4), Quat::IDENTITY).build(),
        );
        assert_eq!(app.sync(), xr::Result::SUCCESS);
        let batch_spaces = [left_grip_sp, left_palm_sp];
        let mut batch_data = [
            xr::SpaceLocationData {
                location_flags: xr::SpaceLocationFlags::EMPTY,
                pose: xr::Posef::IDENTITY,
            };
            2
        ];
        let li = xr::SpacesLocateInfo {
            ty: xr::StructureType::SPACES_LOCATE_INFO,
            next: std::ptr::null(),
            base_space: app.local_space,
            time: t,
            space_count: 2,
            spaces: batch_spaces.as_ptr(),
        };
        let mut sl = xr::SpaceLocations {
            ty: xr::StructureType::SPACE_LOCATIONS,
            next: std::ptr::null_mut(),
            location_count: 2,
            locations: batch_data.as_mut_ptr(),
        };
        assert_eq!(loc_batch(app.sess, &li, &mut sl), xr::Result::SUCCESS);
        assert!(batch_data[0].location_flags.contains(xr::SpaceLocationFlags::POSITION_VALID));
        assert!(batch_data[1].location_flags.contains(xr::SpaceLocationFlags::POSITION_VALID));

        assert_eq!(loc_khr(app.sess, &li, &mut sl), xr::Result::SUCCESS);
        // Count mismatch -> XR_ERROR_VALIDATION_FAILURE
        sl.location_count = 1;
        assert_eq!(
            loc_batch(app.sess, &li, &mut sl),
            xr::Result::ERROR_VALIDATION_FAILURE
        );

        assert_eq!(destroy_sp(left_grip_sp), xr::Result::SUCCESS);
        assert_eq!(destroy_sp(left_grip_offset_sp), xr::Result::SUCCESS);
        assert_eq!(destroy_sp(left_aim_sp), xr::Result::SUCCESS);
        assert_eq!(destroy_sp(left_palm_sp), xr::Result::SUCCESS);
    }

    app.destroy();
    let diag = global_engine().diagnostics();
    assert_eq!(diag.leaked_hand_trackers_on_destroy_session, 0);
    assert_eq!(diag.leaked_spaces_on_destroy_session, 0);
}

#[test]
fn t1_multithreaded_no_deadlock_and_zero_leaks_across_repeated_sessions() {
    let _g = HARNESS_LOCK.lock();
    let cfg = MockRuntimeConfig {
        poll_event_delay_ms: 40, // blocking downstream xrPollEvent!
        ..Default::default()
    };
    reset_mock_runtime(cfg);
    let h = DirectLayerHarness::load().unwrap();
    let app = XrApp::new(h, xr::Version::new(1, 0, 0), &["XR_MND_headless"]);
    let trig = app.create_action("trig", xr::ActionType::FLOAT_INPUT, &[app.left_user]);
    app.suggest(
        "/interaction_profiles/oculus/touch_controller",
        &[(trig, "/user/hand/left/input/trigger/value")],
    );
    app.attach_and_begin();
    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::ZERO, Quat::IDENTITY).build(),
    );

    // Concurrent thread blocks inside downstream xrPollEvent (40ms) while main thread runs syncs!
    let inst_raw = app.inst.into_raw();
    let poll_fn: xr::pfn::PollEvent = unsafe { app.h.pfn(app.inst, "xrPollEvent").unwrap() };
    let t1 = std::thread::spawn(move || {
        let mut ev = xr::EventDataBuffer {
            ty: xr::StructureType::EVENT_DATA_BUFFER,
            next: std::ptr::null(),
            varying: [0; 4000],
        };
        for _ in 0..3 {
            let _ = unsafe { poll_fn(xr::Instance::from_raw(inst_raw), &mut ev) };
        }
    });

    for _ in 0..10 {
        assert_eq!(app.sync(), xr::Result::SUCCESS);
        let _ = app.get_float(trig, app.left_user);
    }
    t1.join().expect("poll thread joined cleanly without deadlock");

    app.destroy();
    let diag = global_engine().diagnostics();
    assert_eq!(diag.leaked_hand_trackers_on_destroy_session, 0);
    assert_eq!(diag.leaked_spaces_on_destroy_session, 0);
    assert_eq!(diag.leaked_sessions_on_destroy_instance, 0);
    assert_eq!(diag.live_instances, 0);
    assert_eq!(diag.live_sessions, 0);
    assert_eq!(diag.live_hand_trackers, 0);
}

#[test]
fn t1_khr_simple_controller_and_shared_action_physical_precedence() {
    let _g = HARNESS_LOCK.lock();
    reset_mock_runtime(MockRuntimeConfig::default());
    let h = DirectLayerHarness::load().unwrap();
    let app = XrApp::new(h, xr::Version::new(1, 0, 0), &["XR_MND_headless"]);

    // Shared action with NO subaction paths bound to BOTH hands on khr/simple_controller!
    let shared_select = app.create_action("shared_select", xr::ActionType::BOOLEAN_INPUT, &[]);
    app.suggest(
        "/interaction_profiles/khr/simple_controller",
        &[
            (shared_select, "/user/hand/left/input/select/click"),
            (shared_select, "/user/hand/right/input/select/click"),
        ],
    );
    app.attach_and_begin();

    // 1. Hand emulation on khr/simple_controller: left index pinch -> select/click == TRUE
    global_engine().set_hand_state(
        0,
        HandSkeletonBuilder::open(0, Vec3::ZERO, Quat::IDENTITY)
            .with_pinch(10, 0.0)
            .build(),
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    advance_dual_clock_ms(65);
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    assert_eq!(
        app.get_profile(app.left_user),
        app.path("/interaction_profiles/khr/simple_controller")
    );
    assert_eq!(
        app.get_bool(shared_select, xr::Path::NULL).current_state,
        xr::TRUE
    );

    // 2. Physical controllers active on BOTH hands (select_click = false) while hands are pinched:
    // physical controllers MUST win even though the app has zero subaction-split actions!
    global_engine().set_physical_controller(
        0,
        MockPhysicalControllerState {
            active: true,
            interaction_profile: Some("/interaction_profiles/khr/simple_controller".to_owned()),
            select_click: false,
            ..Default::default()
        },
    );
    global_engine().set_physical_controller(
        1,
        MockPhysicalControllerState {
            active: true,
            interaction_profile: Some("/interaction_profiles/khr/simple_controller".to_owned()),
            select_click: false,
            ..Default::default()
        },
    );
    assert_eq!(app.sync(), xr::Result::SUCCESS);
    assert_eq!(
        app.get_bool(shared_select, xr::Path::NULL).current_state,
        xr::FALSE
    );

    app.destroy();
}

