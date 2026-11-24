//! Isolated subprocess probe that loads the **genuine Khronos OpenXR 1.1.54 `libopenxr_loader.so`**
//! alongside `libmock_xr_runtime.so` and exercises real implicit API layer discovery via
//! `XDG_DATA_HOME`, `XR_RUNTIME_JSON`, `DISABLE_XR_APILAYER_ALVR_HAND_EMULATION`, and
//! `ALVR_HAND_LAYER_CONFIG`.

use anyhow::{bail, Context, Result};
use e2e_runner::HandSkeletonBuilder;
use glam::{Quat, Vec3};
use libloading::{Library, Symbol};
use mock_xr_core::{MockDiagnostics, MockHandState};
use openxr_sys as xr;
use openxr_sys::Handle as _;
use std::ffi::{c_char, CStr, CString};
use std::path::PathBuf;

fn load_sym<T: Copy>(lib: &Library, name: &[u8]) -> Result<T> {
    let s: Symbol<T> = unsafe { lib.get(name) }
        .with_context(|| format!("missing symbol {:?}", String::from_utf8_lossy(name)))?;
    Ok(*s)
}

fn c_path(
    s2p: xr::pfn::StringToPath,
    inst: xr::Instance,
    s: &str,
) -> Result<xr::Path> {
    let cs = CString::new(s)?;
    let mut p = xr::Path::NULL;
    let res = unsafe { s2p(inst, cs.as_ptr(), &mut p) };
    anyhow::ensure!(res == xr::Result::SUCCESS, "xrStringToPath({s}) -> {res:?}");
    Ok(p)
}

fn main() -> Result<()> {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "emulate".to_owned());
    let loader_so = PathBuf::from(
        std::env::var("E2E_PROBE_LOADER_SO").context("missing E2E_PROBE_LOADER_SO")?,
    );
    let runtime_so = PathBuf::from(
        std::env::var("E2E_PROBE_RUNTIME_SO").context("missing E2E_PROBE_RUNTIME_SO")?,
    );

    // Pre-open canonicalized mock_xr_runtime.so so glibc dlopen shares the SAME global instance
    // with the Khronos loader's dlopen (zero-IPC deterministic state control & leak checks!).
    let rt_canon = std::fs::canonicalize(&runtime_so)?;
    let rt_lib = unsafe { Library::new(&rt_canon) }
        .with_context(|| format!("failed to pre-dlopen {}", rt_canon.display()))?;
    let mock_reset: extern "C" fn() = load_sym(&rt_lib, b"mock_xr_reset\0")?;
    let mock_set_hand: unsafe extern "C" fn(u32, *const c_char) -> i32 =
        load_sym(&rt_lib, b"mock_xr_set_hand_json\0")?;
    let mock_advance_time: extern "C" fn(i64) -> i64 =
        load_sym(&rt_lib, b"mock_xr_advance_time_ns\0")?;
    let mock_get_diag: extern "C" fn() -> *mut c_char =
        load_sym(&rt_lib, b"mock_xr_get_diagnostics_json\0")?;
    let mock_free_str: unsafe extern "C" fn(*mut c_char) =
        load_sym(&rt_lib, b"mock_xr_free_string\0")?;
    mock_reset();

    // Load genuine Khronos OpenXR 1.1.54 loader.
    let loader = unsafe { Library::new(&loader_so) }
        .with_context(|| format!("failed to dlopen {}", loader_so.display()))?;

    let enum_layers: xr::pfn::EnumerateApiLayerProperties =
        load_sym(&loader, b"xrEnumerateApiLayerProperties\0")?;
    let create_instance: xr::pfn::CreateInstance = load_sym(&loader, b"xrCreateInstance\0")?;
    let destroy_instance: xr::pfn::DestroyInstance = load_sym(&loader, b"xrDestroyInstance\0")?;
    let get_system: xr::pfn::GetSystem = load_sym(&loader, b"xrGetSystem\0")?;
    let create_session: xr::pfn::CreateSession = load_sym(&loader, b"xrCreateSession\0")?;
    let destroy_session: xr::pfn::DestroySession = load_sym(&loader, b"xrDestroySession\0")?;
    let begin_session: xr::pfn::BeginSession = load_sym(&loader, b"xrBeginSession\0")?;
    let end_session: xr::pfn::EndSession = load_sym(&loader, b"xrEndSession\0")?;
    let s2p: xr::pfn::StringToPath = load_sym(&loader, b"xrStringToPath\0")?;
    let create_action_set: xr::pfn::CreateActionSet = load_sym(&loader, b"xrCreateActionSet\0")?;
    let create_action: xr::pfn::CreateAction = load_sym(&loader, b"xrCreateAction\0")?;
    let suggest: xr::pfn::SuggestInteractionProfileBindings =
        load_sym(&loader, b"xrSuggestInteractionProfileBindings\0")?;
    let attach: xr::pfn::AttachSessionActionSets =
        load_sym(&loader, b"xrAttachSessionActionSets\0")?;
    let sync: xr::pfn::SyncActions = load_sym(&loader, b"xrSyncActions\0")?;
    let get_float: xr::pfn::GetActionStateFloat = load_sym(&loader, b"xrGetActionStateFloat\0")?;
    let get_bool: xr::pfn::GetActionStateBoolean =
        load_sym(&loader, b"xrGetActionStateBoolean\0")?;
    let get_profile: xr::pfn::GetCurrentInteractionProfile =
        load_sym(&loader, b"xrGetCurrentInteractionProfile\0")?;
    let create_ref_space: xr::pfn::CreateReferenceSpace =
        load_sym(&loader, b"xrCreateReferenceSpace\0")?;
    let create_act_space: xr::pfn::CreateActionSpace =
        load_sym(&loader, b"xrCreateActionSpace\0")?;
    let destroy_space: xr::pfn::DestroySpace = load_sym(&loader, b"xrDestroySpace\0")?;
    let locate_space: xr::pfn::LocateSpace = load_sym(&loader, b"xrLocateSpace\0")?;

    // 1. Enumerate API layers via genuine Khronos loader!
    let mut layer_count = 0u32;
    let res = unsafe { enum_layers(0, &mut layer_count, std::ptr::null_mut()) };
    anyhow::ensure!(
        res == xr::Result::SUCCESS,
        "xrEnumerateApiLayerProperties count failed: {res:?}"
    );
    let mut layers = vec![
        xr::ApiLayerProperties {
            ty: xr::StructureType::API_LAYER_PROPERTIES,
            next: std::ptr::null_mut(),
            layer_name: [0; xr::MAX_API_LAYER_NAME_SIZE],
            spec_version: xr::Version::from_raw(0),
            layer_version: 0,
            description: [0; xr::MAX_API_LAYER_DESCRIPTION_SIZE],
        };
        layer_count as usize
    ];
    if layer_count > 0 {
        let mut out_count = 0u32;
        let res2 = unsafe { enum_layers(layer_count, &mut out_count, layers.as_mut_ptr()) };
        anyhow::ensure!(
            res2 == xr::Result::SUCCESS,
            "xrEnumerateApiLayerProperties fill failed: {res2:?}"
        );
    }
    let layer_names: Vec<String> = layers
        .iter()
        .map(|l| {
            unsafe { CStr::from_ptr(l.layer_name.as_ptr()) }
                .to_string_lossy()
                .into_owned()
        })
        .collect();

    let discovered = layer_names
        .iter()
        .any(|n| n == "XR_APILAYER_ALVR_hand_emulation");

    if mode == "disabled" {
        // When DISABLE_XR_APILAYER_ALVR_HAND_EMULATION=1 is set, the Khronos loader skips our implicit layer!
        anyhow::ensure!(
            !discovered,
            "Expected Khronos loader to skip layer when DISABLE_XR_APILAYER_ALVR_HAND_EMULATION=1, but found {layer_names:?}"
        );
    } else {
        anyhow::ensure!(
            discovered,
            "Expected Khronos loader to discover XR_APILAYER_ALVR_hand_emulation, found: {layer_names:?}"
        );
    }

    // 2. Create OpenXR instance (app does NOT request XR_EXT_hand_tracking; our implicit layer injects it!).
    let mut app_info = xr::ApplicationInfo {
        application_name: [0; xr::MAX_APPLICATION_NAME_SIZE],
        application_version: 1,
        engine_name: [0; xr::MAX_ENGINE_NAME_SIZE],
        engine_version: 1,
        api_version: xr::Version::new(1, 0, 0),
    };
    for (i, b) in b"KhronosProbe".iter().enumerate() {
        app_info.application_name[i] = *b as c_char;
    }
    let headless_ext = CString::new("XR_MND_headless")?;
    let ext_ptrs = [headless_ext.as_ptr()];
    let ici = xr::InstanceCreateInfo {
        ty: xr::StructureType::INSTANCE_CREATE_INFO,
        next: std::ptr::null(),
        create_flags: xr::InstanceCreateFlags::EMPTY,
        application_info: app_info,
        enabled_api_layer_count: 0,
        enabled_api_layer_names: std::ptr::null(),
        enabled_extension_count: ext_ptrs.len() as u32,
        enabled_extension_names: ext_ptrs.as_ptr(),
    };
    let mut inst = xr::Instance::NULL;
    let res = unsafe { create_instance(&ici, &mut inst) };
    anyhow::ensure!(
        res == xr::Result::SUCCESS,
        "Khronos xrCreateInstance failed: {res:?}"
    );

    let sgi = xr::SystemGetInfo {
        ty: xr::StructureType::SYSTEM_GET_INFO,
        next: std::ptr::null(),
        form_factor: xr::FormFactor::HEAD_MOUNTED_DISPLAY,
    };
    let mut sys_id = xr::SystemId::NULL;
    anyhow::ensure!(
        unsafe { get_system(inst, &sgi, &mut sys_id) } == xr::Result::SUCCESS
    );

    let sci = xr::SessionCreateInfo {
        ty: xr::StructureType::SESSION_CREATE_INFO,
        next: std::ptr::null(),
        create_flags: xr::SessionCreateFlags::EMPTY,
        system_id: sys_id,
    };
    let mut sess = xr::Session::NULL;
    anyhow::ensure!(
        unsafe { create_session(inst, &sci, &mut sess) } == xr::Result::SUCCESS
    );

    let sbi = xr::SessionBeginInfo {
        ty: xr::StructureType::SESSION_BEGIN_INFO,
        next: std::ptr::null(),
        primary_view_configuration_type: xr::ViewConfigurationType::PRIMARY_STEREO,
    };
    anyhow::ensure!(unsafe { begin_session(sess, &sbi) } == xr::Result::SUCCESS);

    // 3. Create action set + trigger float + trigger touch bool + grip pose, bind to oculus/touch_controller.
    let mut asci = xr::ActionSetCreateInfo {
        ty: xr::StructureType::ACTION_SET_CREATE_INFO,
        next: std::ptr::null(),
        action_set_name: [0; xr::MAX_ACTION_SET_NAME_SIZE],
        localized_action_set_name: [0; xr::MAX_LOCALIZED_ACTION_SET_NAME_SIZE],
        priority: 0,
    };
    for (i, b) in b"gameplay".iter().enumerate() {
        asci.action_set_name[i] = *b as c_char;
        asci.localized_action_set_name[i] = *b as c_char;
    }
    let mut action_set = xr::ActionSet::NULL;
    anyhow::ensure!(
        unsafe { create_action_set(inst, &asci, &mut action_set) } == xr::Result::SUCCESS
    );

    let left_user = c_path(s2p, inst, "/user/hand/left")?;
    let make_action = |name: &str, ty: xr::ActionType| -> Result<xr::Action> {
        let mut aci = xr::ActionCreateInfo {
            ty: xr::StructureType::ACTION_CREATE_INFO,
            next: std::ptr::null(),
            action_name: [0; xr::MAX_ACTION_NAME_SIZE],
            action_type: ty,
            count_subaction_paths: 1,
            subaction_paths: &left_user,
            localized_action_name: [0; xr::MAX_LOCALIZED_ACTION_NAME_SIZE],
        };
        for (i, b) in name.as_bytes().iter().enumerate() {
            aci.action_name[i] = *b as c_char;
            aci.localized_action_name[i] = *b as c_char;
        }
        let mut act = xr::Action::NULL;
        let r = unsafe { create_action(action_set, &aci, &mut act) };
        anyhow::ensure!(r == xr::Result::SUCCESS, "create_action({name}) -> {r:?}");
        Ok(act)
    };

    let trig_val_act = make_action("trigger_val", xr::ActionType::FLOAT_INPUT)?;
    let trig_touch_act = make_action("trigger_touch", xr::ActionType::BOOLEAN_INPUT)?;
    let grip_pose_act = make_action("grip_pose", xr::ActionType::POSE_INPUT)?;

    let touch_profile = c_path(s2p, inst, "/interaction_profiles/oculus/touch_controller")?;
    let bindings = [
        xr::ActionSuggestedBinding {
            action: trig_val_act,
            binding: c_path(s2p, inst, "/user/hand/left/input/trigger/value")?,
        },
        xr::ActionSuggestedBinding {
            action: trig_touch_act,
            binding: c_path(s2p, inst, "/user/hand/left/input/trigger/touch")?,
        },
        xr::ActionSuggestedBinding {
            action: grip_pose_act,
            binding: c_path(s2p, inst, "/user/hand/left/input/grip/pose")?,
        },
    ];
    let ipsb = xr::InteractionProfileSuggestedBinding {
        ty: xr::StructureType::INTERACTION_PROFILE_SUGGESTED_BINDING,
        next: std::ptr::null(),
        interaction_profile: touch_profile,
        count_suggested_bindings: bindings.len() as u32,
        suggested_bindings: bindings.as_ptr(),
    };
    anyhow::ensure!(unsafe { suggest(inst, &ipsb) } == xr::Result::SUCCESS);

    let asai = xr::SessionActionSetsAttachInfo {
        ty: xr::StructureType::SESSION_ACTION_SETS_ATTACH_INFO,
        next: std::ptr::null(),
        count_action_sets: 1,
        action_sets: &action_set,
    };
    anyhow::ensure!(unsafe { attach(sess, &asai) } == xr::Result::SUCCESS);

    // Create LOCAL space + left grip action space.
    let rsci = xr::ReferenceSpaceCreateInfo {
        ty: xr::StructureType::REFERENCE_SPACE_CREATE_INFO,
        next: std::ptr::null(),
        reference_space_type: xr::ReferenceSpaceType::LOCAL,
        pose_in_reference_space: xr::Posef::IDENTITY,
    };
    let mut local_space = xr::Space::NULL;
    anyhow::ensure!(unsafe { create_ref_space(sess, &rsci, &mut local_space) } == xr::Result::SUCCESS);

    let asci_space = xr::ActionSpaceCreateInfo {
        ty: xr::StructureType::ACTION_SPACE_CREATE_INFO,
        next: std::ptr::null(),
        action: grip_pose_act,
        subaction_path: left_user,
        pose_in_action_space: xr::Posef::IDENTITY,
    };
    let mut grip_space = xr::Space::NULL;
    anyhow::ensure!(unsafe { create_act_space(sess, &asci_space, &mut grip_space) } == xr::Result::SUCCESS);

    // 4. Inject left-hand Thumb+Index pinch in mock runtime, advance > 50ms debounce, and sync!
    let pinched: MockHandState =
        HandSkeletonBuilder::open(0, Vec3::new(-0.2, 1.2, -0.4), Quat::IDENTITY)
            .with_pinch(10, 0.0)
            .build();
    let pinched_json = CString::new(serde_json::to_string(&pinched)?)?;
    anyhow::ensure!(unsafe { mock_set_hand(0, pinched_json.as_ptr()) } == 0);

    let aas = xr::ActiveActionSet {
        action_set,
        subaction_path: xr::Path::NULL,
    };
    let sync_info = xr::ActionsSyncInfo {
        ty: xr::StructureType::ACTIONS_SYNC_INFO,
        next: std::ptr::null(),
        count_active_action_sets: 1,
        active_action_sets: &aas,
    };

    // Frame 1 (starts debounce)
    anyhow::ensure!(unsafe { sync(sess, &sync_info) } == xr::Result::SUCCESS);
    // Advance 80ms (> 50ms ALVR activation_delay)
    std::thread::sleep(std::time::Duration::from_millis(80));
    let t_now = mock_advance_time(80_000_000);
    // Frame 2 (pinch active!)
    anyhow::ensure!(unsafe { sync(sess, &sync_info) } == xr::Result::SUCCESS);

    let mut flt_state = xr::ActionStateFloat {
        ty: xr::StructureType::ACTION_STATE_FLOAT,
        next: std::ptr::null_mut(),
        current_state: 0.0,
        changed_since_last_sync: xr::FALSE,
        last_change_time: xr::Time::from_nanos(0),
        is_active: xr::FALSE,
    };
    let gi_val = xr::ActionStateGetInfo {
        ty: xr::StructureType::ACTION_STATE_GET_INFO,
        next: std::ptr::null(),
        action: trig_val_act,
        subaction_path: left_user,
    };
    anyhow::ensure!(unsafe { get_float(sess, &gi_val, &mut flt_state) } == xr::Result::SUCCESS);

    let mut touch_state = xr::ActionStateBoolean {
        ty: xr::StructureType::ACTION_STATE_BOOLEAN,
        next: std::ptr::null_mut(),
        current_state: xr::FALSE,
        changed_since_last_sync: xr::FALSE,
        last_change_time: xr::Time::from_nanos(0),
        is_active: xr::FALSE,
    };
    let gi_touch = xr::ActionStateGetInfo {
        ty: xr::StructureType::ACTION_STATE_GET_INFO,
        next: std::ptr::null(),
        action: trig_touch_act,
        subaction_path: left_user,
    };
    anyhow::ensure!(unsafe { get_bool(sess, &gi_touch, &mut touch_state) } == xr::Result::SUCCESS);

    let mut prof_state = xr::InteractionProfileState {
        ty: xr::StructureType::INTERACTION_PROFILE_STATE,
        next: std::ptr::null_mut(),
        interaction_profile: xr::Path::NULL,
    };
    anyhow::ensure!(unsafe { get_profile(sess, left_user, &mut prof_state) } == xr::Result::SUCCESS);

    let mut loc = xr::SpaceLocation {
        ty: xr::StructureType::SPACE_LOCATION,
        next: std::ptr::null_mut(),
        location_flags: xr::SpaceLocationFlags::EMPTY,
        pose: xr::Posef::IDENTITY,
    };
    anyhow::ensure!(
        unsafe { locate_space(grip_space, local_space, xr::Time::from_nanos(t_now), &mut loc) }
            == xr::Result::SUCCESS
    );

    match mode.as_str() {
        "emulate" => {
            anyhow::ensure!(flt_state.is_active == xr::TRUE, "trigger_val not active");
            anyhow::ensure!(
                (flt_state.current_state - 1.0).abs() < 1e-3,
                "expected trigger_val ~1.0, got {}",
                flt_state.current_state
            );
            anyhow::ensure!(touch_state.is_active == xr::TRUE && touch_state.current_state == xr::TRUE);
            anyhow::ensure!(
                prof_state.interaction_profile == touch_profile,
                "expected active profile oculus/touch_controller"
            );
            anyhow::ensure!(
                loc.location_flags.contains(
                    xr::SpaceLocationFlags::POSITION_VALID
                        | xr::SpaceLocationFlags::ORIENTATION_VALID
                        | xr::SpaceLocationFlags::POSITION_TRACKED
                        | xr::SpaceLocationFlags::ORIENTATION_TRACKED
                ),
                "expected tracked grip pose, got flags={:?}",
                loc.location_flags
            );
        }
        "only_touch" => {
            // Custom ALVR_HAND_LAYER_CONFIG sets only_touch=true: touch=TRUE, trigger_val=0.0!
            anyhow::ensure!(touch_state.is_active == xr::TRUE && touch_state.current_state == xr::TRUE);
            anyhow::ensure!(
                flt_state.is_active == xr::TRUE && flt_state.current_state == 0.0,
                "expected trigger_val == 0.0 in only_touch mode, got {}",
                flt_state.current_state
            );
        }
        "disabled" => {
            // Layer disabled by env var -> mock runtime has no physical controller, so inactive!
            anyhow::ensure!(flt_state.is_active == xr::FALSE, "expected inactive when disabled");
            anyhow::ensure!(prof_state.interaction_profile == xr::Path::NULL);
        }
        other => bail!("unknown probe mode '{other}'"),
    }

    // 5. Clean teardown & verify ZERO leaked hand trackers or spaces in mock runtime!
    anyhow::ensure!(unsafe { destroy_space(grip_space) } == xr::Result::SUCCESS);
    anyhow::ensure!(unsafe { destroy_space(local_space) } == xr::Result::SUCCESS);
    anyhow::ensure!(unsafe { end_session(sess) } == xr::Result::SUCCESS);
    anyhow::ensure!(unsafe { destroy_session(sess) } == xr::Result::SUCCESS);
    anyhow::ensure!(unsafe { destroy_instance(inst) } == xr::Result::SUCCESS);

    let diag_ptr = mock_get_diag();
    anyhow::ensure!(!diag_ptr.is_null(), "mock_xr_get_diagnostics_json returned null");
    let diag_str = unsafe { CStr::from_ptr(diag_ptr) }.to_string_lossy().into_owned();
    unsafe { mock_free_str(diag_ptr) };
    let diag: MockDiagnostics = serde_json::from_str(&diag_str)?;

    anyhow::ensure!(
        diag.leaked_hand_trackers_on_destroy_session == 0,
        "leaked hand trackers on xrDestroySession: {}",
        diag.leaked_hand_trackers_on_destroy_session
    );
    anyhow::ensure!(
        diag.leaked_spaces_on_destroy_session == 0,
        "leaked spaces on xrDestroySession: {}",
        diag.leaked_spaces_on_destroy_session
    );
    if mode != "disabled" {
        anyhow::ensure!(
            diag.hand_trackers_created == 2 && diag.hand_trackers_destroyed == 2,
            "expected 2 created & 2 destroyed hand trackers, got created={} destroyed={}",
            diag.hand_trackers_created,
            diag.hand_trackers_destroyed
        );
    }

    println!("PROBE_OK mode={mode} trackers_created={} trackers_destroyed={}", diag.hand_trackers_created, diag.hand_trackers_destroyed);
    Ok(())
}
