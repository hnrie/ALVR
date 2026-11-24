//! Shared OpenXR 1.0/1.1 + `XR_EXT_hand_tracking` + `XR_FB_hand_tracking_aim` mock runtime core
//! with C-ABI dispatch trampolines usable both in-process (Harness A) and inside `mock_xr_runtime`
//! cdylib (Harness B with genuine Khronos `libopenxr_loader.so`).

pub mod engine;
pub mod math;
pub mod types;

pub use engine::MockRuntimeEngine;
pub use types::*;

use openxr_sys as xr;
use openxr_sys::Handle as _;
use std::ffi::CStr;
use std::mem::{size_of, transmute};
use std::os::raw::c_char;
use std::sync::LazyLock;

static GLOBAL_ENGINE: LazyLock<MockRuntimeEngine> = LazyLock::new(MockRuntimeEngine::new);

pub fn global_engine() -> &'static MockRuntimeEngine {
    &GLOBAL_ENGINE
}

/// Loader-Runtime interface v1 negotiation for `libmock_xr_runtime.so` and in-process mock loader.
pub unsafe fn negotiate_runtime_interface(
    loader_info: *const xr::NegotiateLoaderInfo,
    runtime_request: *mut xr::NegotiateRuntimeRequest,
) -> xr::Result {
    if loader_info.is_null() || runtime_request.is_null() {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    let li = unsafe { &*loader_info };
    let req = unsafe { &mut *runtime_request };
    if li.struct_type != xr::LoaderInterfaceStructs::LOADER_INFO
        || li.struct_version != xr::LOADER_INFO_STRUCT_VERSION as u32
        || li.struct_size != size_of::<xr::NegotiateLoaderInfo>()
    {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    if req.struct_type != xr::LoaderInterfaceStructs::RUNTIME_REQUEST
        || req.struct_version != xr::RUNTIME_INFO_STRUCT_VERSION as u32
        || req.struct_size != size_of::<xr::NegotiateRuntimeRequest>()
    {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    let ver = global_engine().record_negotiate();
    let clamped = ver
        .into_raw()
        .clamp(li.min_api_version.into_raw(), li.max_api_version.into_raw());
    req.runtime_interface_version = xr::CURRENT_LOADER_RUNTIME_VERSION as u32;
    req.runtime_api_version = xr::Version::from_raw(clamped);
    req.get_instance_proc_addr = Some(rt_get_instance_proc_addr);
    xr::Result::SUCCESS
}

/// Terminator `xrCreateApiLayerInstance` used by Harness A (direct dlopen mock loader).
pub unsafe extern "system" fn rt_create_api_layer_instance_terminator(
    info: *const xr::InstanceCreateInfo,
    _layer_info: *const xr::ApiLayerCreateInfo,
    instance: *mut xr::Instance,
) -> xr::Result {
    unsafe { global_engine().create_instance(info, instance) }
}

pub unsafe extern "system" fn rt_get_instance_proc_addr(
    instance: xr::Instance,
    name: *const c_char,
    function: *mut Option<xr::pfn::VoidFunction>,
) -> xr::Result {
    if function.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    unsafe {
        *function = None;
    }
    if name.is_null() {
        return xr::Result::ERROR_FUNCTION_UNSUPPORTED;
    }
    let Ok(fn_name) = unsafe { CStr::from_ptr(name) }.to_str() else {
        return xr::Result::ERROR_FUNCTION_UNSUPPORTED;
    };

    if instance == xr::Instance::NULL {
        let pre: Option<xr::pfn::VoidFunction> = unsafe {
            match fn_name {
                "xrEnumerateInstanceExtensionProperties" => Some(transmute::<
                    xr::pfn::EnumerateInstanceExtensionProperties,
                    xr::pfn::VoidFunction,
                >(
                    rt_enumerate_instance_extension_properties
                )),
                "xrCreateInstance" => Some(transmute::<
                    xr::pfn::CreateInstance,
                    xr::pfn::VoidFunction,
                >(rt_create_instance)),
                _ => None,
            }
        };
        return if let Some(f) = pre {
            unsafe {
                *function = Some(f);
            }
            xr::Result::SUCCESS
        } else {
            xr::Result::ERROR_HANDLE_INVALID
        };
    }

    if !global_engine().instance_exists(instance) {
        return xr::Result::ERROR_HANDLE_INVALID;
    }

    let api_ver = global_engine()
        .instance_api_version(instance)
        .unwrap_or_else(|_| xr::Version::new(1, 0, 0));
    let has_ht = global_engine()
        .instance_has_extension(instance, "XR_EXT_hand_tracking")
        .unwrap_or(false);
    let has_locate_khr = global_engine()
        .instance_has_extension(instance, "XR_KHR_locate_spaces")
        .unwrap_or(false);

    let f: Option<xr::pfn::VoidFunction> = unsafe {
        match fn_name {
            "xrGetInstanceProcAddr" => Some(transmute::<
                xr::pfn::GetInstanceProcAddr,
                xr::pfn::VoidFunction,
            >(rt_get_instance_proc_addr)),
            "xrEnumerateInstanceExtensionProperties" => Some(transmute::<
                xr::pfn::EnumerateInstanceExtensionProperties,
                xr::pfn::VoidFunction,
            >(
                rt_enumerate_instance_extension_properties
            )),
            "xrCreateInstance" => Some(transmute::<
                xr::pfn::CreateInstance,
                xr::pfn::VoidFunction,
            >(rt_create_instance)),
            "xrDestroyInstance" => Some(transmute::<
                xr::pfn::DestroyInstance,
                xr::pfn::VoidFunction,
            >(rt_destroy_instance)),
            "xrGetInstanceProperties" => Some(transmute::<
                xr::pfn::GetInstanceProperties,
                xr::pfn::VoidFunction,
            >(rt_get_instance_properties)),
            "xrPollEvent" => Some(transmute::<xr::pfn::PollEvent, xr::pfn::VoidFunction>(
                rt_poll_event,
            )),
            "xrGetSystem" => Some(transmute::<xr::pfn::GetSystem, xr::pfn::VoidFunction>(
                rt_get_system,
            )),
            "xrGetSystemProperties" => Some(transmute::<
                xr::pfn::GetSystemProperties,
                xr::pfn::VoidFunction,
            >(rt_get_system_properties)),
            "xrCreateSession" => Some(transmute::<
                xr::pfn::CreateSession,
                xr::pfn::VoidFunction,
            >(rt_create_session)),
            "xrDestroySession" => Some(transmute::<
                xr::pfn::DestroySession,
                xr::pfn::VoidFunction,
            >(rt_destroy_session)),
            "xrBeginSession" => Some(transmute::<
                xr::pfn::BeginSession,
                xr::pfn::VoidFunction,
            >(rt_begin_session)),
            "xrEndSession" => Some(transmute::<xr::pfn::EndSession, xr::pfn::VoidFunction>(
                rt_end_session,
            )),
            "xrRequestExitSession" => Some(transmute::<
                xr::pfn::RequestExitSession,
                xr::pfn::VoidFunction,
            >(rt_request_exit_session)),
            "xrWaitFrame" => Some(transmute::<xr::pfn::WaitFrame, xr::pfn::VoidFunction>(
                rt_wait_frame,
            )),
            "xrCreateSwapchain" => Some(transmute::<
                xr::pfn::CreateSwapchain,
                xr::pfn::VoidFunction,
            >(rt_create_swapchain)),
            "xrDestroySwapchain" => Some(transmute::<
                xr::pfn::DestroySwapchain,
                xr::pfn::VoidFunction,
            >(rt_destroy_swapchain)),
            "xrStringToPath" => Some(transmute::<xr::pfn::StringToPath, xr::pfn::VoidFunction>(
                rt_string_to_path,
            )),
            "xrPathToString" => Some(transmute::<xr::pfn::PathToString, xr::pfn::VoidFunction>(
                rt_path_to_string,
            )),
            "xrCreateActionSet" => Some(transmute::<
                xr::pfn::CreateActionSet,
                xr::pfn::VoidFunction,
            >(rt_create_action_set)),
            "xrDestroyActionSet" => Some(transmute::<
                xr::pfn::DestroyActionSet,
                xr::pfn::VoidFunction,
            >(rt_destroy_action_set)),
            "xrCreateAction" => Some(transmute::<
                xr::pfn::CreateAction,
                xr::pfn::VoidFunction,
            >(rt_create_action)),
            "xrDestroyAction" => Some(transmute::<
                xr::pfn::DestroyAction,
                xr::pfn::VoidFunction,
            >(rt_destroy_action)),
            "xrSuggestInteractionProfileBindings" => Some(transmute::<
                xr::pfn::SuggestInteractionProfileBindings,
                xr::pfn::VoidFunction,
            >(
                rt_suggest_interaction_profile_bindings
            )),
            "xrAttachSessionActionSets" => Some(transmute::<
                xr::pfn::AttachSessionActionSets,
                xr::pfn::VoidFunction,
            >(rt_attach_session_action_sets)),
            "xrGetCurrentInteractionProfile" => Some(transmute::<
                xr::pfn::GetCurrentInteractionProfile,
                xr::pfn::VoidFunction,
            >(rt_get_current_interaction_profile)),
            "xrSyncActions" => Some(transmute::<xr::pfn::SyncActions, xr::pfn::VoidFunction>(
                rt_sync_actions,
            )),
            "xrGetActionStateBoolean" => Some(transmute::<
                xr::pfn::GetActionStateBoolean,
                xr::pfn::VoidFunction,
            >(rt_get_action_state_boolean)),
            "xrGetActionStateFloat" => Some(transmute::<
                xr::pfn::GetActionStateFloat,
                xr::pfn::VoidFunction,
            >(rt_get_action_state_float)),
            "xrGetActionStateVector2f" => Some(transmute::<
                xr::pfn::GetActionStateVector2f,
                xr::pfn::VoidFunction,
            >(rt_get_action_state_vector2f)),
            "xrGetActionStatePose" => Some(transmute::<
                xr::pfn::GetActionStatePose,
                xr::pfn::VoidFunction,
            >(rt_get_action_state_pose)),
            "xrEnumerateBoundSourcesForAction" => Some(transmute::<
                xr::pfn::EnumerateBoundSourcesForAction,
                xr::pfn::VoidFunction,
            >(rt_enumerate_bound_sources_for_action)),
            "xrGetInputSourceLocalizedName" => Some(transmute::<
                xr::pfn::GetInputSourceLocalizedName,
                xr::pfn::VoidFunction,
            >(rt_get_input_source_localized_name)),
            "xrApplyHapticFeedback" => Some(transmute::<
                xr::pfn::ApplyHapticFeedback,
                xr::pfn::VoidFunction,
            >(rt_apply_haptic_feedback)),
            "xrStopHapticFeedback" => Some(transmute::<
                xr::pfn::StopHapticFeedback,
                xr::pfn::VoidFunction,
            >(rt_stop_haptic_feedback)),
            "xrCreateReferenceSpace" => Some(transmute::<
                xr::pfn::CreateReferenceSpace,
                xr::pfn::VoidFunction,
            >(rt_create_reference_space)),
            "xrCreateActionSpace" => Some(transmute::<
                xr::pfn::CreateActionSpace,
                xr::pfn::VoidFunction,
            >(rt_create_action_space)),
            "xrDestroySpace" => Some(transmute::<
                xr::pfn::DestroySpace,
                xr::pfn::VoidFunction,
            >(rt_destroy_space)),
            "xrLocateSpace" => Some(transmute::<xr::pfn::LocateSpace, xr::pfn::VoidFunction>(
                rt_locate_space,
            )),
            "xrLocateSpaces" if (api_ver.major(), api_ver.minor()) >= (1, 1) => Some(transmute::<
                xr::pfn::LocateSpaces,
                xr::pfn::VoidFunction,
            >(
                rt_locate_spaces
            )),
            "xrLocateSpacesKHR" if has_locate_khr => Some(transmute::<
                xr::pfn::LocateSpacesKHR,
                xr::pfn::VoidFunction,
            >(rt_locate_spaces)),
            "xrCreateHandTrackerEXT" if has_ht => Some(transmute::<
                xr::pfn::CreateHandTrackerEXT,
                xr::pfn::VoidFunction,
            >(rt_create_hand_tracker_ext)),
            "xrDestroyHandTrackerEXT" if has_ht => Some(transmute::<
                xr::pfn::DestroyHandTrackerEXT,
                xr::pfn::VoidFunction,
            >(rt_destroy_hand_tracker_ext)),
            "xrLocateHandJointsEXT" if has_ht => Some(transmute::<
                xr::pfn::LocateHandJointsEXT,
                xr::pfn::VoidFunction,
            >(rt_locate_hand_joints_ext)),
            _ => None,
        }
    };

    if let Some(fp) = f {
        unsafe {
            *function = Some(fp);
        }
        xr::Result::SUCCESS
    } else {
        xr::Result::ERROR_FUNCTION_UNSUPPORTED
    }
}

pub unsafe extern "system" fn rt_enumerate_instance_extension_properties(
    layer_name: *const c_char,
    property_capacity_input: u32,
    property_count_output: *mut u32,
    properties: *mut xr::ExtensionProperties,
) -> xr::Result {
    if property_count_output.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    let layer_opt = if layer_name.is_null() {
        None
    } else {
        Some(unsafe { CStr::from_ptr(layer_name) }.to_string_lossy().into_owned())
    };
    let exts = match global_engine().enumerate_instance_extension_properties(layer_opt.as_deref()) {
        Ok(v) => v,
        Err(e) => return e,
    };
    unsafe {
        *property_count_output = exts.len() as u32;
    }
    if property_capacity_input == 0 {
        return xr::Result::SUCCESS;
    }
    if (property_capacity_input as usize) < exts.len() {
        return xr::Result::ERROR_SIZE_INSUFFICIENT;
    }
    if !exts.is_empty() && properties.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }
    for (i, (name, ver)) in exts.into_iter().enumerate() {
        let dst = unsafe { &mut *properties.add(i) };
        dst.ty = xr::StructureType::EXTENSION_PROPERTIES;
        dst.next = std::ptr::null_mut();
        dst.extension_version = ver;
        engine::write_cstr_buf(&mut dst.extension_name, &name);
    }
    xr::Result::SUCCESS
}

pub unsafe extern "system" fn rt_create_instance(
    info: *const xr::InstanceCreateInfo,
    instance: *mut xr::Instance,
) -> xr::Result {
    unsafe { global_engine().create_instance(info, instance) }
}
pub unsafe extern "system" fn rt_destroy_instance(instance: xr::Instance) -> xr::Result {
    global_engine().destroy_instance(instance)
}
pub unsafe extern "system" fn rt_get_instance_properties(
    instance: xr::Instance,
    props: *mut xr::InstanceProperties,
) -> xr::Result {
    unsafe { global_engine().get_instance_properties(instance, props) }
}
pub unsafe extern "system" fn rt_poll_event(
    instance: xr::Instance,
    buf: *mut xr::EventDataBuffer,
) -> xr::Result {
    unsafe { global_engine().poll_event(instance, buf) }
}
pub unsafe extern "system" fn rt_get_system(
    instance: xr::Instance,
    info: *const xr::SystemGetInfo,
    sys_id: *mut xr::SystemId,
) -> xr::Result {
    unsafe { global_engine().get_system(instance, info, sys_id) }
}
pub unsafe extern "system" fn rt_get_system_properties(
    instance: xr::Instance,
    sys_id: xr::SystemId,
    props: *mut xr::SystemProperties,
) -> xr::Result {
    unsafe { global_engine().get_system_properties(instance, sys_id, props) }
}
pub unsafe extern "system" fn rt_create_session(
    instance: xr::Instance,
    info: *const xr::SessionCreateInfo,
    session: *mut xr::Session,
) -> xr::Result {
    unsafe { global_engine().create_session(instance, info, session) }
}
pub unsafe extern "system" fn rt_destroy_session(session: xr::Session) -> xr::Result {
    global_engine().destroy_session(session)
}
pub unsafe extern "system" fn rt_begin_session(
    session: xr::Session,
    _info: *const xr::SessionBeginInfo,
) -> xr::Result {
    global_engine().begin_session(session)
}
pub unsafe extern "system" fn rt_end_session(session: xr::Session) -> xr::Result {
    global_engine().end_session(session)
}
pub unsafe extern "system" fn rt_request_exit_session(session: xr::Session) -> xr::Result {
    global_engine().request_exit_session(session)
}
pub unsafe extern "system" fn rt_wait_frame(
    session: xr::Session,
    _info: *const xr::FrameWaitInfo,
    state: *mut xr::FrameState,
) -> xr::Result {
    unsafe { global_engine().wait_frame(session, state) }
}
pub unsafe extern "system" fn rt_create_swapchain(
    session: xr::Session,
    _info: *const xr::SwapchainCreateInfo,
    swapchain: *mut xr::Swapchain,
) -> xr::Result {
    unsafe { global_engine().create_swapchain(session, swapchain) }
}
pub unsafe extern "system" fn rt_destroy_swapchain(swapchain: xr::Swapchain) -> xr::Result {
    global_engine().destroy_swapchain(swapchain)
}
pub unsafe extern "system" fn rt_string_to_path(
    instance: xr::Instance,
    path_string: *const c_char,
    path: *mut xr::Path,
) -> xr::Result {
    unsafe { global_engine().string_to_path(instance, path_string, path) }
}
pub unsafe extern "system" fn rt_path_to_string(
    instance: xr::Instance,
    path: xr::Path,
    cap: u32,
    out: *mut u32,
    buf: *mut c_char,
) -> xr::Result {
    unsafe { global_engine().path_to_string(instance, path, cap, out, buf) }
}
pub unsafe extern "system" fn rt_create_action_set(
    instance: xr::Instance,
    info: *const xr::ActionSetCreateInfo,
    set: *mut xr::ActionSet,
) -> xr::Result {
    unsafe { global_engine().create_action_set(instance, info, set) }
}
pub unsafe extern "system" fn rt_destroy_action_set(set: xr::ActionSet) -> xr::Result {
    global_engine().destroy_action_set(set)
}
pub unsafe extern "system" fn rt_create_action(
    set: xr::ActionSet,
    info: *const xr::ActionCreateInfo,
    action: *mut xr::Action,
) -> xr::Result {
    unsafe { global_engine().create_action(set, info, action) }
}
pub unsafe extern "system" fn rt_destroy_action(action: xr::Action) -> xr::Result {
    global_engine().destroy_action(action)
}
pub unsafe extern "system" fn rt_suggest_interaction_profile_bindings(
    instance: xr::Instance,
    sb: *const xr::InteractionProfileSuggestedBinding,
) -> xr::Result {
    unsafe { global_engine().suggest_interaction_profile_bindings(instance, sb) }
}
pub unsafe extern "system" fn rt_attach_session_action_sets(
    session: xr::Session,
    info: *const xr::SessionActionSetsAttachInfo,
) -> xr::Result {
    unsafe { global_engine().attach_session_action_sets(session, info) }
}
pub unsafe extern "system" fn rt_get_current_interaction_profile(
    session: xr::Session,
    user_path: xr::Path,
    state: *mut xr::InteractionProfileState,
) -> xr::Result {
    unsafe { global_engine().get_current_interaction_profile(session, user_path, state) }
}
pub unsafe extern "system" fn rt_sync_actions(
    session: xr::Session,
    info: *const xr::ActionsSyncInfo,
) -> xr::Result {
    unsafe { global_engine().sync_actions(session, info) }
}
pub unsafe extern "system" fn rt_get_action_state_boolean(
    session: xr::Session,
    info: *const xr::ActionStateGetInfo,
    state: *mut xr::ActionStateBoolean,
) -> xr::Result {
    unsafe { global_engine().get_action_state_boolean(session, info, state) }
}
pub unsafe extern "system" fn rt_get_action_state_float(
    session: xr::Session,
    info: *const xr::ActionStateGetInfo,
    state: *mut xr::ActionStateFloat,
) -> xr::Result {
    unsafe { global_engine().get_action_state_float(session, info, state) }
}
pub unsafe extern "system" fn rt_get_action_state_vector2f(
    session: xr::Session,
    info: *const xr::ActionStateGetInfo,
    state: *mut xr::ActionStateVector2f,
) -> xr::Result {
    unsafe { global_engine().get_action_state_vector2f(session, info, state) }
}
pub unsafe extern "system" fn rt_get_action_state_pose(
    session: xr::Session,
    info: *const xr::ActionStateGetInfo,
    state: *mut xr::ActionStatePose,
) -> xr::Result {
    unsafe { global_engine().get_action_state_pose(session, info, state) }
}
pub unsafe extern "system" fn rt_enumerate_bound_sources_for_action(
    session: xr::Session,
    info: *const xr::BoundSourcesForActionEnumerateInfo,
    cap: u32,
    out: *mut u32,
    sources: *mut xr::Path,
) -> xr::Result {
    unsafe { global_engine().enumerate_bound_sources_for_action(session, info, cap, out, sources) }
}
pub unsafe extern "system" fn rt_get_input_source_localized_name(
    session: xr::Session,
    info: *const xr::InputSourceLocalizedNameGetInfo,
    cap: u32,
    out: *mut u32,
    buf: *mut c_char,
) -> xr::Result {
    unsafe { global_engine().get_input_source_localized_name(session, info, cap, out, buf) }
}
pub unsafe extern "system" fn rt_apply_haptic_feedback(
    session: xr::Session,
    info: *const xr::HapticActionInfo,
    fb: *const xr::HapticBaseHeader,
) -> xr::Result {
    unsafe { global_engine().apply_haptic_feedback(session, info, fb) }
}
pub unsafe extern "system" fn rt_stop_haptic_feedback(
    session: xr::Session,
    info: *const xr::HapticActionInfo,
) -> xr::Result {
    unsafe { global_engine().stop_haptic_feedback(session, info) }
}
pub unsafe extern "system" fn rt_create_reference_space(
    session: xr::Session,
    info: *const xr::ReferenceSpaceCreateInfo,
    space: *mut xr::Space,
) -> xr::Result {
    unsafe { global_engine().create_reference_space(session, info, space) }
}
pub unsafe extern "system" fn rt_create_action_space(
    session: xr::Session,
    info: *const xr::ActionSpaceCreateInfo,
    space: *mut xr::Space,
) -> xr::Result {
    unsafe { global_engine().create_action_space(session, info, space) }
}
pub unsafe extern "system" fn rt_destroy_space(space: xr::Space) -> xr::Result {
    global_engine().destroy_space(space)
}
pub unsafe extern "system" fn rt_locate_space(
    space: xr::Space,
    base_space: xr::Space,
    time: xr::Time,
    loc: *mut xr::SpaceLocation,
) -> xr::Result {
    unsafe { global_engine().locate_space(space, base_space, time, loc) }
}
pub unsafe extern "system" fn rt_locate_spaces(
    session: xr::Session,
    info: *const xr::SpacesLocateInfo,
    locs: *mut xr::SpaceLocations,
) -> xr::Result {
    unsafe { global_engine().locate_spaces(session, info, locs) }
}
pub unsafe extern "system" fn rt_create_hand_tracker_ext(
    session: xr::Session,
    info: *const xr::HandTrackerCreateInfoEXT,
    tracker: *mut xr::HandTrackerEXT,
) -> xr::Result {
    unsafe { global_engine().create_hand_tracker_ext(session, info, tracker) }
}
pub unsafe extern "system" fn rt_destroy_hand_tracker_ext(
    tracker: xr::HandTrackerEXT,
) -> xr::Result {
    global_engine().destroy_hand_tracker_ext(tracker)
}
pub unsafe extern "system" fn rt_locate_hand_joints_ext(
    tracker: xr::HandTrackerEXT,
    info: *const xr::HandJointsLocateInfoEXT,
    locs: *mut xr::HandJointLocationsEXT,
) -> xr::Result {
    unsafe { global_engine().locate_hand_joints_ext(tracker, info, locs) }
}
