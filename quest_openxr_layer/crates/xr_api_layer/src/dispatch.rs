//! OpenXR API Layer negotiation, `xrCreateApiLayerInstance`, `xrGetInstanceProcAddr` distributed
//! dispatch, and instance/session/event lifecycle hooks.

use crate::android::{android_asset_config, android_config_candidates, init_logging};
use crate::input;
use crate::spaces;
use crate::state::{
    get_instance, get_session, HandTrackerResources, InstanceInner, InstanceState, NextDispatch,
    SessionInner, SessionState, ACTION_SETS, ACTIONS, EXT_FB_HAND_TRACKING_AIM, EXT_HAND_TRACKING,
    EXT_KHR_LOCATE_SPACES, INSTANCES, LAYER_NAME, SESSIONS, SPACES,
};
use alvr_hand_core::config::{LayerConfig, DISABLE_ENV_VAR};
use alvr_hand_core::VirtualHandState;
use openxr_sys as xr;
use openxr_sys::Handle;
use parking_lot::{Mutex, RwLock};
use std::ffi::{c_char, c_void, CStr, CString};
use std::mem::{size_of, transmute};
use std::path::PathBuf;
use std::sync::Arc;

/// Helper to resolve a downstream OpenXR command via `next_get_instance_proc_addr`.
unsafe fn load_pfn<T>(
    gipa: xr::pfn::GetInstanceProcAddr,
    instance: xr::Instance,
    name: &'static [u8],
) -> Option<T> {
    let mut func: Option<xr::pfn::VoidFunction> = None;
    let res = gipa(instance, name.as_ptr() as *const c_char, &mut func);
    if res == xr::Result::SUCCESS {
        func.map(|f| std::mem::transmute_copy::<xr::pfn::VoidFunction, T>(&f))
    } else {
        None
    }
}

/// Core implementation of `xrNegotiateLoaderApiLayerInterface`.
///
/// Validates loader/request headers, checks layer name (if non-null), clamps `layer_api_version`
/// into `[min_api_version, max_api_version]` across OpenXR 1.0 and 1.1 ignoring patch version,
/// and returns our `xrGetInstanceProcAddr` and `xrCreateApiLayerInstance` entry points.
pub unsafe fn negotiate_loader_api_layer_interface(
    loader_info: *const xr::NegotiateLoaderInfo,
    layer_name: *const c_char,
    api_layer_request: *mut xr::NegotiateApiLayerRequest,
) -> xr::Result {
    init_logging();

    if loader_info.is_null() || api_layer_request.is_null() {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }

    let li = &*loader_info;
    let req = &mut *api_layer_request;

    if li.struct_type != xr::LoaderInterfaceStructs::LOADER_INFO
        || li.struct_version != xr::LOADER_INFO_STRUCT_VERSION as u32
        || li.struct_size != size_of::<xr::NegotiateLoaderInfo>()
    {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }

    if req.struct_type != xr::LoaderInterfaceStructs::API_LAYER_REQUEST
        || req.struct_version != xr::API_LAYER_INFO_STRUCT_VERSION as u32
        || req.struct_size != size_of::<xr::NegotiateApiLayerRequest>()
    {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }

    if !layer_name.is_null() {
        match CStr::from_ptr(layer_name).to_str() {
            Ok(name) if name == LAYER_NAME => {}
            _ => return xr::Result::ERROR_INITIALIZATION_FAILED,
        }
    }

    let cur_iface = xr::CURRENT_LOADER_API_LAYER_VERSION as u32;
    if li.min_interface_version > cur_iface || li.max_interface_version < cur_iface {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }

    // OpenXR 1.0 / 1.1 API version negotiation ignoring patch mismatch (Edge Case #1).
    let min_raw = li.min_api_version.into_raw();
    let max_raw = li.max_api_version.into_raw();
    if min_raw > max_raw {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }
    let min_mm = (li.min_api_version.major(), li.min_api_version.minor());
    let max_mm = (li.max_api_version.major(), li.max_api_version.minor());
    if max_mm < (1, 0) || min_mm > (1, 1) {
        return xr::Result::ERROR_INITIALIZATION_FAILED;
    }

    let preferred = if max_mm >= (1, 1) {
        xr::Version::new(1, 1, 0)
    } else {
        xr::Version::new(1, 0, 0)
    };
    let clamped_raw = preferred.into_raw().clamp(min_raw, max_raw);

    req.layer_interface_version = cur_iface;
    req.layer_api_version = xr::Version::from_raw(clamped_raw);
    req.get_instance_proc_addr = Some(layer_get_instance_proc_addr);
    req.create_api_layer_instance = Some(layer_create_api_layer_instance);

    xr::Result::SUCCESS
}

/// Layer chain `xrCreateApiLayerInstance`.
pub unsafe extern "system" fn layer_create_api_layer_instance(
    info: *const xr::InstanceCreateInfo,
    layer_info: *const xr::ApiLayerCreateInfo,
    instance: *mut xr::Instance,
) -> xr::Result {
    crate::ffi_guard(|| {
        init_logging();

        if info.is_null() || layer_info.is_null() || instance.is_null() {
            return xr::Result::ERROR_INITIALIZATION_FAILED;
        }
        let li = &*layer_info;
        if li.struct_type != xr::LoaderInterfaceStructs::API_LAYER_CREATE_INFO
            || li.struct_version != xr::API_LAYER_CREATE_INFO_STRUCT_VERSION as u32
            || li.struct_size != size_of::<xr::ApiLayerCreateInfo>()
            || li.next_info.is_null()
        {
            return xr::Result::ERROR_INITIALIZATION_FAILED;
        }

        let ni = &*li.next_info;
        if ni.struct_type != xr::LoaderInterfaceStructs::API_LAYER_NEXT_INFO
            || ni.struct_version != xr::API_LAYER_NEXT_INFO_STRUCT_VERSION as u32
            || ni.struct_size != size_of::<xr::ApiLayerNextInfo>()
        {
            return xr::Result::ERROR_INITIALIZATION_FAILED;
        }

        let Some(next_gipa) = ni.next_get_instance_proc_addr else {
            return xr::Result::ERROR_INITIALIZATION_FAILED;
        };
        let Some(next_create) = ni.next_create_api_layer_instance else {
            return xr::Result::ERROR_INITIALIZATION_FAILED;
        };

        // Advance `next_info` for the downstream layer / loader terminator.
        let mut next_layer_info = *li;
        next_layer_info.next_info = ni.next;

        // Inspect application-requested extensions.
        let orig_info = &*info;
        let mut orig_exts: Vec<String> =
            Vec::with_capacity(orig_info.enabled_extension_count as usize);
        if orig_info.enabled_extension_count > 0 && !orig_info.enabled_extension_names.is_null() {
            let slice = std::slice::from_raw_parts(
                orig_info.enabled_extension_names,
                orig_info.enabled_extension_count as usize,
            );
            for &ptr in slice {
                if !ptr.is_null() {
                    if let Ok(s) = CStr::from_ptr(ptr).to_str() {
                        orig_exts.push(s.to_owned());
                    }
                }
            }
        }

        let app_enabled_hand_tracking = orig_exts.iter().any(|e| e == EXT_HAND_TRACKING);
        let app_enabled_fb_aim = orig_exts.iter().any(|e| e == EXT_FB_HAND_TRACKING_AIM);
        let app_enabled_locate_spaces_khr = orig_exts.iter().any(|e| e == EXT_KHR_LOCATE_SPACES);

        // Load runtime configuration:
        // ALVR_HAND_LAYER_CONFIG > Android externalFilesDir > filesDir > settings_file_location > APK asset > defaults.
        let disabled_by_env = std::env::var_os(DISABLE_ENV_VAR).is_some_and(|v| !v.is_empty());
        let mut candidates: Vec<PathBuf> = android_config_candidates();
        if li.settings_file_location[0] != 0 {
            if let Ok(settings_path) = CStr::from_ptr(li.settings_file_location.as_ptr()).to_str() {
                if !settings_path.is_empty() {
                    candidates.push(PathBuf::from(settings_path));
                }
            }
        }
        let config = LayerConfig::try_load_from_paths(&candidates)
            .or_else(android_asset_config)
            .unwrap_or_else(|| LayerConfig::default().sanitized());

        // Inject XR_EXT_hand_tracking + optional XR_FB_hand_tracking_aim with progressive fallback.
        let mut layer_has_hand_tracking = app_enabled_hand_tracking;
        let mut layer_has_fb_aim = app_enabled_fb_aim;

        let create_res = if !disabled_by_env && config.enabled {
            // Attempt 1: inject both XR_EXT_hand_tracking and XR_FB_hand_tracking_aim.
            let res_both = call_create_with_extra_exts(
                orig_info,
                &orig_exts,
                &[EXT_HAND_TRACKING, EXT_FB_HAND_TRACKING_AIM],
                next_create,
                &next_layer_info,
                instance,
            );
            if res_both == xr::Result::SUCCESS {
                layer_has_hand_tracking = true;
                layer_has_fb_aim = true;
                res_both
            } else if res_both == xr::Result::ERROR_EXTENSION_NOT_PRESENT {
                // Attempt 2: inject ONLY XR_EXT_hand_tracking (runtime may lack FB aim extension).
                let res_ht = if !app_enabled_fb_aim {
                    call_create_with_extra_exts(
                        orig_info,
                        &orig_exts,
                        &[EXT_HAND_TRACKING],
                        next_create,
                        &next_layer_info,
                        instance,
                    )
                } else {
                    res_both
                };

                if res_ht == xr::Result::SUCCESS {
                    layer_has_hand_tracking = true;
                    layer_has_fb_aim = app_enabled_fb_aim;
                    res_ht
                } else if res_ht == xr::Result::ERROR_EXTENSION_NOT_PRESENT
                    && !app_enabled_hand_tracking
                {
                    // Attempt 3: runtime lacks XR_EXT_hand_tracking -> pure passthrough fallback!
                    log::warn!(
                        "Runtime lacks {EXT_HAND_TRACKING}; {LAYER_NAME} entering passthrough mode"
                    );
                    layer_has_hand_tracking = false;
                    layer_has_fb_aim = false;
                    next_create(info, &next_layer_info, instance)
                } else {
                    res_ht
                }
            } else {
                res_both
            }
        } else {
            next_create(info, &next_layer_info, instance)
        };

        if create_res != xr::Result::SUCCESS {
            return create_res;
        }

        let created_instance = *instance;

        // Populate downstream dispatch table; destroy created_instance on failure so nothing leaks!
        let Some(next_dispatch) = build_next_dispatch(next_gipa, created_instance) else {
            if let Some(destroy_fn) =
                load_pfn::<xr::pfn::DestroyInstance>(next_gipa, created_instance, b"xrDestroyInstance\0")
            {
                let _ = destroy_fn(created_instance);
            }
            *instance = xr::Instance::NULL;
            return xr::Result::ERROR_INITIALIZATION_FAILED;
        };

        // Intern canonical hand top-level paths upfront.
        let left_hand_path = intern_path(&next_dispatch, created_instance, b"/user/hand/left\0");
        let right_hand_path = intern_path(&next_dispatch, created_instance, b"/user/hand/right\0");

        let mut inner = InstanceInner::default();
        if left_hand_path != xr::Path::NULL {
            inner.record_path(left_hand_path, "/user/hand/left".to_owned());
        }
        if right_hand_path != xr::Path::NULL {
            inner.record_path(right_hand_path, "/user/hand/right".to_owned());
        }

        let state = Arc::new(InstanceState {
            handle: created_instance,
            next: next_dispatch,
            config,
            disabled_by_env,
            app_api_version: orig_info.application_info.api_version,
            app_enabled_hand_tracking,
            app_enabled_fb_aim,
            app_enabled_locate_spaces_khr,
            layer_has_hand_tracking: layer_has_hand_tracking
                && next_dispatch.create_hand_tracker_ext.is_some()
                && next_dispatch.destroy_hand_tracker_ext.is_some()
                && next_dispatch.locate_hand_joints_ext.is_some(),
            layer_has_fb_aim,
            left_hand_path,
            right_hand_path,
            inner: RwLock::new(inner),
        });

        INSTANCES
            .write()
            .insert(created_instance.into_raw(), state);

        xr::Result::SUCCESS
    })
}

/// Layer distributed `xrGetInstanceProcAddr`.
pub unsafe extern "system" fn layer_get_instance_proc_addr(
    instance: xr::Instance,
    name: *const c_char,
    function: *mut Option<xr::pfn::VoidFunction>,
) -> xr::Result {
    crate::ffi_guard(|| {
        if function.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        *function = None;
        if name.is_null() {
            return xr::Result::ERROR_FUNCTION_UNSUPPORTED;
        }
        let Ok(fn_name) = CStr::from_ptr(name).to_str() else {
            return xr::Result::ERROR_FUNCTION_UNSUPPORTED;
        };

        // Pre-instance lookup rules (`api_initialization.adoc:50-130`).
        if instance == xr::Instance::NULL {
            return match fn_name {
                "xrInitializeLoaderKHR" => {
                    *function = Some(transmute::<
                        xr::pfn::InitializeLoaderKHR,
                        xr::pfn::VoidFunction,
                    >(crate::xrInitializeLoaderKHR));
                    xr::Result::SUCCESS
                }
                "xrGetInstanceProcAddr" => {
                    *function = Some(transmute::<
                        xr::pfn::GetInstanceProcAddr,
                        xr::pfn::VoidFunction,
                    >(layer_get_instance_proc_addr));
                    xr::Result::SUCCESS
                }
                "xrEnumerateApiLayerProperties"
                | "xrEnumerateInstanceExtensionProperties"
                | "xrCreateInstance" => xr::Result::ERROR_FUNCTION_UNSUPPORTED,
                _ => xr::Result::ERROR_HANDLE_INVALID,
            };
        }

        let Some(inst) = get_instance(instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };

        // Gate injected XR_EXT_hand_tracking functions if the host app did NOT request the extension (Edge Case #6)!
        if !inst.app_enabled_hand_tracking
            && matches!(
                fn_name,
                "xrCreateHandTrackerEXT" | "xrDestroyHandTrackerEXT" | "xrLocateHandJointsEXT"
            )
        {
            *function = None;
            return xr::Result::ERROR_FUNCTION_UNSUPPORTED;
        }

        // Gate OpenXR 1.1 `xrLocateSpaces` and extension `xrLocateSpacesKHR` strictly so apps
        // cannot bypass our space hook when running on an OpenXR 1.1 runtime under an OpenXR 1.0 instance.
        if fn_name == "xrLocateSpaces"
            && ((inst.app_api_version.major(), inst.app_api_version.minor()) < (1, 1)
                || inst.next.locate_spaces.is_none())
        {
            *function = None;
            return xr::Result::ERROR_FUNCTION_UNSUPPORTED;
        }
        if fn_name == "xrLocateSpacesKHR"
            && (!inst.app_enabled_locate_spaces_khr || inst.next.locate_spaces_khr.is_none())
        {
            *function = None;
            return xr::Result::ERROR_FUNCTION_UNSUPPORTED;
        }

        let local_fn: Option<xr::pfn::VoidFunction> = match fn_name {
            "xrGetInstanceProcAddr" => Some(transmute::<
                xr::pfn::GetInstanceProcAddr,
                xr::pfn::VoidFunction,
            >(layer_get_instance_proc_addr)),
            "xrDestroyInstance" => Some(transmute::<
                xr::pfn::DestroyInstance,
                xr::pfn::VoidFunction,
            >(layer_destroy_instance)),
            "xrPollEvent" => Some(transmute::<xr::pfn::PollEvent, xr::pfn::VoidFunction>(
                layer_poll_event,
            )),
            "xrGetSystemProperties" => Some(transmute::<
                xr::pfn::GetSystemProperties,
                xr::pfn::VoidFunction,
            >(layer_get_system_properties)),
            "xrCreateSession" => Some(transmute::<
                xr::pfn::CreateSession,
                xr::pfn::VoidFunction,
            >(layer_create_session)),
            "xrDestroySession" => Some(transmute::<
                xr::pfn::DestroySession,
                xr::pfn::VoidFunction,
            >(layer_destroy_session)),
            "xrBeginSession" => Some(transmute::<
                xr::pfn::BeginSession,
                xr::pfn::VoidFunction,
            >(layer_begin_session)),
            "xrEndSession" => Some(transmute::<xr::pfn::EndSession, xr::pfn::VoidFunction>(
                layer_end_session,
            )),
            "xrWaitFrame" if inst.next.wait_frame.is_some() => Some(transmute::<
                xr::pfn::WaitFrame,
                xr::pfn::VoidFunction,
            >(layer_wait_frame)),
            "xrStringToPath" => Some(transmute::<xr::pfn::StringToPath, xr::pfn::VoidFunction>(
                input::layer_string_to_path,
            )),
            "xrPathToString" => Some(transmute::<xr::pfn::PathToString, xr::pfn::VoidFunction>(
                input::layer_path_to_string,
            )),
            "xrCreateReferenceSpace" => Some(transmute::<
                xr::pfn::CreateReferenceSpace,
                xr::pfn::VoidFunction,
            >(spaces::layer_create_reference_space)),
            "xrCreateActionSpace" => Some(transmute::<
                xr::pfn::CreateActionSpace,
                xr::pfn::VoidFunction,
            >(spaces::layer_create_action_space)),
            "xrDestroySpace" => Some(transmute::<
                xr::pfn::DestroySpace,
                xr::pfn::VoidFunction,
            >(spaces::layer_destroy_space)),
            "xrLocateSpace" => Some(transmute::<xr::pfn::LocateSpace, xr::pfn::VoidFunction>(
                spaces::layer_locate_space,
            )),
            "xrLocateSpaces" => Some(transmute::<xr::pfn::LocateSpaces, xr::pfn::VoidFunction>(
                spaces::layer_locate_spaces,
            )),
            "xrLocateSpacesKHR" => Some(transmute::<
                xr::pfn::LocateSpacesKHR,
                xr::pfn::VoidFunction,
            >(spaces::layer_locate_spaces_khr)),
            "xrCreateActionSet" => Some(transmute::<
                xr::pfn::CreateActionSet,
                xr::pfn::VoidFunction,
            >(input::layer_create_action_set)),
            "xrDestroyActionSet" => Some(transmute::<
                xr::pfn::DestroyActionSet,
                xr::pfn::VoidFunction,
            >(input::layer_destroy_action_set)),
            "xrCreateAction" => Some(transmute::<
                xr::pfn::CreateAction,
                xr::pfn::VoidFunction,
            >(input::layer_create_action)),
            "xrDestroyAction" => Some(transmute::<
                xr::pfn::DestroyAction,
                xr::pfn::VoidFunction,
            >(input::layer_destroy_action)),
            "xrSuggestInteractionProfileBindings" => Some(transmute::<
                xr::pfn::SuggestInteractionProfileBindings,
                xr::pfn::VoidFunction,
            >(
                input::layer_suggest_interaction_profile_bindings
            )),
            "xrAttachSessionActionSets" => Some(transmute::<
                xr::pfn::AttachSessionActionSets,
                xr::pfn::VoidFunction,
            >(input::layer_attach_session_action_sets)),
            "xrGetCurrentInteractionProfile" => Some(transmute::<
                xr::pfn::GetCurrentInteractionProfile,
                xr::pfn::VoidFunction,
            >(input::layer_get_current_interaction_profile)),
            "xrSyncActions" => Some(transmute::<xr::pfn::SyncActions, xr::pfn::VoidFunction>(
                input::layer_sync_actions,
            )),
            "xrGetActionStateBoolean" => Some(transmute::<
                xr::pfn::GetActionStateBoolean,
                xr::pfn::VoidFunction,
            >(input::layer_get_action_state_boolean)),
            "xrGetActionStateFloat" => Some(transmute::<
                xr::pfn::GetActionStateFloat,
                xr::pfn::VoidFunction,
            >(input::layer_get_action_state_float)),
            "xrGetActionStateVector2f" => Some(transmute::<
                xr::pfn::GetActionStateVector2f,
                xr::pfn::VoidFunction,
            >(input::layer_get_action_state_vector2f)),
            "xrGetActionStatePose" => Some(transmute::<
                xr::pfn::GetActionStatePose,
                xr::pfn::VoidFunction,
            >(input::layer_get_action_state_pose)),
            "xrEnumerateBoundSourcesForAction" => Some(transmute::<
                xr::pfn::EnumerateBoundSourcesForAction,
                xr::pfn::VoidFunction,
            >(
                input::layer_enumerate_bound_sources_for_action
            )),
            "xrGetInputSourceLocalizedName" => Some(transmute::<
                xr::pfn::GetInputSourceLocalizedName,
                xr::pfn::VoidFunction,
            >(
                input::layer_get_input_source_localized_name
            )),
            "xrApplyHapticFeedback" => Some(transmute::<
                xr::pfn::ApplyHapticFeedback,
                xr::pfn::VoidFunction,
            >(input::layer_apply_haptic_feedback)),
            "xrStopHapticFeedback" => Some(transmute::<
                xr::pfn::StopHapticFeedback,
                xr::pfn::VoidFunction,
            >(input::layer_stop_haptic_feedback)),
            _ => None,
        };

        if let Some(f) = local_fn {
            *function = Some(f);
            xr::Result::SUCCESS
        } else {
            (inst.next.get_instance_proc_addr)(instance, name, function)
        }
    })
}

pub unsafe extern "system" fn layer_destroy_instance(instance: xr::Instance) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(inst) = INSTANCES.write().remove(&instance.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };

        // Clean up any child sessions not yet destroyed by the app (including internal hand trackers & spaces).
        let remaining_sessions: Vec<u64> = inst.inner.read().sessions.clone();
        for sess_raw in remaining_sessions {
            if let Some(sess) = SESSIONS.write().remove(&sess_raw) {
                destroy_session_internal_handles(&sess);
                let sp_ids: Vec<u64> = sess.inner.lock().spaces.clone();
                let mut sp_map = SPACES.write();
                for s in sp_ids {
                    sp_map.remove(&s);
                }
            }
        }

        // Clean up action sets and actions belonging to this instance.
        let (set_ids, act_ids) = {
            let inner = inst.inner.read();
            (inner.action_sets.clone(), inner.actions.clone())
        };
        {
            let mut s_map = ACTION_SETS.write();
            for s in set_ids {
                s_map.remove(&s);
            }
        }
        {
            let mut a_map = ACTIONS.write();
            for a in act_ids {
                a_map.remove(&a);
            }
        }

        (inst.next.destroy_instance)(instance)
    })
}

pub unsafe extern "system" fn layer_poll_event(
    instance: xr::Instance,
    event_data: *mut xr::EventDataBuffer,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(inst) = get_instance(instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if event_data.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        // Deliver queued synthetic InteractionProfileChanged events (only if session is still running).
        loop {
            let next_sess = inst.inner.write().pending_events.pop_front();
            let Some(ev_sess) = next_sess else {
                break;
            };
            let is_running = get_session(ev_sess).is_some_and(|s| s.inner.lock().is_running);
            if is_running {
                let ev = xr::EventDataInteractionProfileChanged {
                    ty: xr::StructureType::EVENT_DATA_INTERACTION_PROFILE_CHANGED,
                    next: std::ptr::null(),
                    session: ev_sess,
                };
                std::ptr::write(
                    event_data as *mut xr::EventDataInteractionProfileChanged,
                    ev,
                );
                return xr::Result::SUCCESS;
            }
        }

        let res = (inst.next.poll_event)(instance, event_data);
        if res == xr::Result::SUCCESS
            && (*event_data).ty == xr::StructureType::EVENT_DATA_SESSION_STATE_CHANGED
        {
            let sc = &*(event_data as *const xr::EventDataSessionStateChanged);
            if let Some(sess) = get_session(sc.session) {
                let mut inner = sess.inner.lock();
                inner.session_state = sc.state;
                inner.clock.observe_xr_time(sc.time.as_nanos());
                if sc.state == xr::SessionState::FOCUSED {
                    inner.sync_reported_unfocused = false;
                } else {
                    inner.last_sync_focused = false;
                    inner.sync_reported_unfocused = true;
                }
            }
        }
        res
    })
}

pub unsafe extern "system" fn layer_get_system_properties(
    instance: xr::Instance,
    system_id: xr::SystemId,
    properties: *mut xr::SystemProperties,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(inst) = get_instance(instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        (inst.next.get_system_properties)(instance, system_id, properties)
    })
}

pub unsafe extern "system" fn layer_create_session(
    instance: xr::Instance,
    create_info: *const xr::SessionCreateInfo,
    session: *mut xr::Session,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(inst) = get_instance(instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (inst.next.create_session)(instance, create_info, session);
        if res != xr::Result::SUCCESS || create_info.is_null() || session.is_null() {
            return res;
        }

        let created_session = *session;
        let system_id = (*create_info).system_id;

        // Probe hand tracking capability and create layer-owned hand trackers + LOCAL space.
        // Never fail `xrCreateSession` if hand tracking is unsupported!
        let trackers = create_session_hand_trackers(&inst, created_session, system_id);

        let sess_state = Arc::new(SessionState {
            handle: created_session,
            instance: Arc::clone(&inst),
            system_id,
            trackers: Mutex::new(trackers),
            inner: Mutex::new(SessionInner::new()),
        });

        let raw = created_session.into_raw();
        SESSIONS.write().insert(raw, sess_state);
        inst.inner.write().sessions.push(raw);

        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_destroy_session(session: xr::Session) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = SESSIONS.write().remove(&session.into_raw()) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };

        // Remove session reference and any stale pending synthetic events from instance.
        {
            let mut inst_inner = sess.instance.inner.write();
            inst_inner.sessions.retain(|&s| s != session.into_raw());
            inst_inner.prune_session_events(session);
        }
        {
            let sp_ids: Vec<u64> = sess.inner.lock().spaces.drain(..).collect();
            let mut sp_map = SPACES.write();
            for sp in sp_ids {
                sp_map.remove(&sp);
            }
        }

        // CRITICAL (Edge Case #18): destroy ALL layer-owned child handles BEFORE `next.destroy_session`!
        destroy_session_internal_handles(&sess);

        (sess.instance.next.destroy_session)(session)
    })
}

pub unsafe extern "system" fn layer_begin_session(
    session: xr::Session,
    begin_info: *const xr::SessionBeginInfo,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (sess.instance.next.begin_session)(session, begin_info);
        if res == xr::Result::SUCCESS {
            // If hand tracker creation transiently failed during xrCreateSession (e.g., pending
            // Android runtime permission dialog), lazily retry once on xrBeginSession!
            let should_retry = {
                let t = sess.trackers.lock();
                !t.available
                    && !t.supports_hand_tracking_known_false
                    && sess.instance.emulation_enabled()
            };
            if should_retry {
                let new_trackers =
                    create_session_hand_trackers(&sess.instance, session, sess.system_id);
                let mut t = sess.trackers.lock();
                if !t.available {
                    *t = new_trackers;
                } else {
                    drop(t);
                    destroy_hand_tracker_resources(&sess.instance, new_trackers);
                }
            }
            sess.inner.lock().is_running = true;
        }
        res
    })
}

pub unsafe extern "system" fn layer_end_session(session: xr::Session) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (sess.instance.next.end_session)(session);
        if res == xr::Result::SUCCESS {
            sess.instance.inner.write().prune_session_events(session);
            let mut inner = sess.inner.lock();
            inner.is_running = false;
            inner.last_sync_focused = false;
            inner.sync_reported_unfocused = true;
            inner.active_synced_sets.clear();
            inner.action_snapshots.clear();
            inner.sticky_pose_source.clear();
            inner.gesture_engine.reset();
            inner.pose_synthesizer.reset();
            inner.last_hand_states = [VirtualHandState::default(); 2];
            for arb in &mut inner.hand_arb {
                arb.emulated = false;
                arb.hand_tracked = false;
                arb.effective_profile = 0;
            }
        }
        res
    })
}

pub unsafe extern "system" fn layer_wait_frame(
    session: xr::Session,
    frame_wait_info: *const xr::FrameWaitInfo,
    frame_state: *mut xr::FrameState,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let Some(wait_fn) = sess.instance.next.wait_frame else {
            return xr::Result::ERROR_FUNCTION_UNSUPPORTED;
        };
        let res = wait_fn(session, frame_wait_info, frame_state);
        if res == xr::Result::SUCCESS && !frame_state.is_null() {
            let t = (*frame_state).predicted_display_time.as_nanos();
            sess.inner.lock().clock.observe_xr_time(t);
        }
        res
    })
}

unsafe fn call_create_with_extra_exts(
    orig_info: &xr::InstanceCreateInfo,
    orig_exts: &[String],
    extra_exts: &[&str],
    next_create: xr::pfn::CreateApiLayerInstance,
    next_layer_info: &xr::ApiLayerCreateInfo,
    instance: *mut xr::Instance,
) -> xr::Result {
    let mut owned_cstrings: Vec<CString> = Vec::with_capacity(orig_exts.len() + extra_exts.len());
    for e in orig_exts {
        if let Ok(cs) = CString::new(e.as_str()) {
            owned_cstrings.push(cs);
        }
    }
    for &extra in extra_exts {
        if !orig_exts.iter().any(|e| e == extra) {
            if let Ok(cs) = CString::new(extra) {
                owned_cstrings.push(cs);
            }
        }
    }
    let ptrs: Vec<*const c_char> = owned_cstrings.iter().map(|s| s.as_ptr()).collect();

    let mut modified = *orig_info;
    modified.enabled_extension_count = ptrs.len() as u32;
    modified.enabled_extension_names = if ptrs.is_empty() {
        std::ptr::null()
    } else {
        ptrs.as_ptr()
    };

    next_create(&modified, next_layer_info, instance)
}

unsafe fn build_next_dispatch(
    gipa: xr::pfn::GetInstanceProcAddr,
    inst: xr::Instance,
) -> Option<NextDispatch> {
    Some(NextDispatch {
        get_instance_proc_addr: gipa,
        destroy_instance: load_pfn(gipa, inst, b"xrDestroyInstance\0")?,
        poll_event: load_pfn(gipa, inst, b"xrPollEvent\0")?,
        get_system_properties: load_pfn(gipa, inst, b"xrGetSystemProperties\0")?,
        create_session: load_pfn(gipa, inst, b"xrCreateSession\0")?,
        destroy_session: load_pfn(gipa, inst, b"xrDestroySession\0")?,
        begin_session: load_pfn(gipa, inst, b"xrBeginSession\0")?,
        end_session: load_pfn(gipa, inst, b"xrEndSession\0")?,
        wait_frame: load_pfn(gipa, inst, b"xrWaitFrame\0"),
        string_to_path: load_pfn(gipa, inst, b"xrStringToPath\0")?,
        path_to_string: load_pfn(gipa, inst, b"xrPathToString\0")?,
        create_reference_space: load_pfn(gipa, inst, b"xrCreateReferenceSpace\0")?,
        create_action_space: load_pfn(gipa, inst, b"xrCreateActionSpace\0")?,
        destroy_space: load_pfn(gipa, inst, b"xrDestroySpace\0")?,
        locate_space: load_pfn(gipa, inst, b"xrLocateSpace\0")?,
        locate_spaces: load_pfn(gipa, inst, b"xrLocateSpaces\0"),
        locate_spaces_khr: load_pfn(gipa, inst, b"xrLocateSpacesKHR\0"),
        create_action_set: load_pfn(gipa, inst, b"xrCreateActionSet\0")?,
        destroy_action_set: load_pfn(gipa, inst, b"xrDestroyActionSet\0")?,
        create_action: load_pfn(gipa, inst, b"xrCreateAction\0")?,
        destroy_action: load_pfn(gipa, inst, b"xrDestroyAction\0")?,
        suggest_interaction_profile_bindings: load_pfn(
            gipa,
            inst,
            b"xrSuggestInteractionProfileBindings\0",
        )?,
        attach_session_action_sets: load_pfn(gipa, inst, b"xrAttachSessionActionSets\0")?,
        get_current_interaction_profile: load_pfn(
            gipa,
            inst,
            b"xrGetCurrentInteractionProfile\0",
        )?,
        sync_actions: load_pfn(gipa, inst, b"xrSyncActions\0")?,
        get_action_state_boolean: load_pfn(gipa, inst, b"xrGetActionStateBoolean\0")?,
        get_action_state_float: load_pfn(gipa, inst, b"xrGetActionStateFloat\0")?,
        get_action_state_vector2f: load_pfn(gipa, inst, b"xrGetActionStateVector2f\0")?,
        get_action_state_pose: load_pfn(gipa, inst, b"xrGetActionStatePose\0")?,
        enumerate_bound_sources_for_action: load_pfn(
            gipa,
            inst,
            b"xrEnumerateBoundSourcesForAction\0",
        )?,
        get_input_source_localized_name: load_pfn(
            gipa,
            inst,
            b"xrGetInputSourceLocalizedName\0",
        )?,
        apply_haptic_feedback: load_pfn(gipa, inst, b"xrApplyHapticFeedback\0")?,
        stop_haptic_feedback: load_pfn(gipa, inst, b"xrStopHapticFeedback\0")?,
        create_hand_tracker_ext: load_pfn(gipa, inst, b"xrCreateHandTrackerEXT\0"),
        destroy_hand_tracker_ext: load_pfn(gipa, inst, b"xrDestroyHandTrackerEXT\0"),
        locate_hand_joints_ext: load_pfn(gipa, inst, b"xrLocateHandJointsEXT\0"),
    })
}

unsafe fn intern_path(
    next: &NextDispatch,
    inst: xr::Instance,
    path_cstr: &'static [u8],
) -> xr::Path {
    let mut out = xr::Path::NULL;
    let res = (next.string_to_path)(inst, path_cstr.as_ptr() as *const c_char, &mut out);
    if res == xr::Result::SUCCESS {
        out
    } else {
        xr::Path::NULL
    }
}

unsafe fn create_session_hand_trackers(
    inst: &InstanceState,
    session: xr::Session,
    system_id: xr::SystemId,
) -> HandTrackerResources {
    let mut res_out = HandTrackerResources::default();
    if !inst.emulation_enabled() {
        return res_out;
    }

    let (Some(create_ht), Some(destroy_ht)) = (
        inst.next.create_hand_tracker_ext,
        inst.next.destroy_hand_tracker_ext,
    ) else {
        return res_out;
    };

    // Check `XrSystemHandTrackingPropertiesEXT::supportsHandTracking`.
    let mut ht_props = xr::SystemHandTrackingPropertiesEXT {
        ty: xr::StructureType::SYSTEM_HAND_TRACKING_PROPERTIES_EXT,
        next: std::ptr::null_mut(),
        supports_hand_tracking: xr::FALSE,
    };
    let mut sys_props = xr::SystemProperties {
        ty: xr::StructureType::SYSTEM_PROPERTIES,
        next: &mut ht_props as *mut _ as *mut c_void,
        system_id: xr::SystemId::NULL,
        vendor_id: 0,
        system_name: [0; xr::MAX_SYSTEM_NAME_SIZE],
        graphics_properties: xr::SystemGraphicsProperties {
            max_swapchain_image_height: 0,
            max_swapchain_image_width: 0,
            max_layer_count: 0,
        },
        tracking_properties: xr::SystemTrackingProperties {
            orientation_tracking: xr::FALSE,
            position_tracking: xr::FALSE,
        },
    };

    let props_res = (inst.next.get_system_properties)(inst.handle, system_id, &mut sys_props);
    if props_res != xr::Result::SUCCESS || ht_props.supports_hand_tracking == xr::FALSE {
        log::warn!("System does not report supportsHandTracking=TRUE; entering passthrough mode");
        if props_res == xr::Result::SUCCESS {
            res_out.supports_hand_tracking_known_false = true;
        }
        return res_out;
    }

    // Create internal LOCAL reference space.
    let local_info = xr::ReferenceSpaceCreateInfo {
        ty: xr::StructureType::REFERENCE_SPACE_CREATE_INFO,
        next: std::ptr::null(),
        reference_space_type: xr::ReferenceSpaceType::LOCAL,
        pose_in_reference_space: xr::Posef::IDENTITY,
    };
    let mut local_space = xr::Space::NULL;
    if (inst.next.create_reference_space)(session, &local_info, &mut local_space)
        != xr::Result::SUCCESS
    {
        return res_out;
    }

    // Create left & right hand trackers.
    let left_info = xr::HandTrackerCreateInfoEXT {
        ty: xr::StructureType::HAND_TRACKER_CREATE_INFO_EXT,
        next: std::ptr::null(),
        hand: xr::HandEXT::LEFT,
        hand_joint_set: xr::HandJointSetEXT::DEFAULT,
    };
    let mut left_tracker = xr::HandTrackerEXT::NULL;
    if create_ht(session, &left_info, &mut left_tracker) != xr::Result::SUCCESS {
        let _ = (inst.next.destroy_space)(local_space);
        return res_out;
    }

    let right_info = xr::HandTrackerCreateInfoEXT {
        ty: xr::StructureType::HAND_TRACKER_CREATE_INFO_EXT,
        next: std::ptr::null(),
        hand: xr::HandEXT::RIGHT,
        hand_joint_set: xr::HandJointSetEXT::DEFAULT,
    };
    let mut right_tracker = xr::HandTrackerEXT::NULL;
    if create_ht(session, &right_info, &mut right_tracker) != xr::Result::SUCCESS {
        let _ = destroy_ht(left_tracker);
        let _ = (inst.next.destroy_space)(local_space);
        return res_out;
    }

    res_out.left_tracker = left_tracker;
    res_out.right_tracker = right_tracker;
    res_out.local_space = local_space;
    res_out.available = true;
    res_out
}

unsafe fn destroy_hand_tracker_resources(inst: &InstanceState, taken: HandTrackerResources) {
    if let Some(destroy_ht) = inst.next.destroy_hand_tracker_ext {
        if taken.left_tracker != xr::HandTrackerEXT::NULL {
            let _ = destroy_ht(taken.left_tracker);
        }
        if taken.right_tracker != xr::HandTrackerEXT::NULL {
            let _ = destroy_ht(taken.right_tracker);
        }
    }
    if taken.local_space != xr::Space::NULL {
        let _ = (inst.next.destroy_space)(taken.local_space);
    }
}

unsafe fn destroy_session_internal_handles(sess: &SessionState) {
    let taken = {
        let mut t = sess.trackers.lock();
        std::mem::take(&mut *t)
    };
    destroy_hand_tracker_resources(&sess.instance, taken);
}
