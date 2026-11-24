//! Mock OpenXR 1.0/1.1 Runtime `cdylib` loaded by the genuine Khronos `libopenxr_loader.so`
//! via `XR_RUNTIME_JSON`, exposing `xrNegotiateLoaderRuntimeInterface` plus JSON control hooks.

use mock_xr_core::{
    global_engine, negotiate_runtime_interface, MockEvent, MockHandState,
    MockPhysicalControllerState, MockRuntimeConfig,
};
use openxr_sys as xr;
use std::ffi::{CStr, CString};
use std::os::raw::c_char;

#[unsafe(no_mangle)]
pub unsafe extern "system" fn xrNegotiateLoaderRuntimeInterface(
    loader_info: *const xr::NegotiateLoaderInfo,
    runtime_request: *mut xr::NegotiateRuntimeRequest,
) -> xr::Result {
    unsafe { negotiate_runtime_interface(loader_info, runtime_request) }
}

#[unsafe(no_mangle)]
pub extern "C" fn mock_xr_reset() {
    global_engine().reset();
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mock_xr_configure_json(json: *const c_char) -> i32 {
    if json.is_null() {
        return -1;
    }
    let Ok(s) = unsafe { CStr::from_ptr(json) }.to_str() else {
        return -2;
    };
    let Ok(cfg) = serde_json::from_str::<MockRuntimeConfig>(s) else {
        return -3;
    };
    global_engine().configure(cfg);
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mock_xr_set_hand_json(hand: u32, json: *const c_char) -> i32 {
    if json.is_null() || hand > 1 {
        return -1;
    }
    let Ok(s) = unsafe { CStr::from_ptr(json) }.to_str() else {
        return -2;
    };
    let Ok(st) = serde_json::from_str::<MockHandState>(s) else {
        return -3;
    };
    global_engine().set_hand_state(hand as usize, st);
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mock_xr_set_physical_json(hand: u32, json: *const c_char) -> i32 {
    if json.is_null() || hand > 1 {
        return -1;
    }
    let Ok(s) = unsafe { CStr::from_ptr(json) }.to_str() else {
        return -2;
    };
    let Ok(st) = serde_json::from_str::<MockPhysicalControllerState>(s) else {
        return -3;
    };
    global_engine().set_physical_controller(hand as usize, st);
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mock_xr_push_event_json(json: *const c_char) -> i32 {
    if json.is_null() {
        return -1;
    }
    let Ok(s) = unsafe { CStr::from_ptr(json) }.to_str() else {
        return -2;
    };
    let Ok(ev) = serde_json::from_str::<MockEvent>(s) else {
        return -3;
    };
    global_engine().push_event(ev);
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn mock_xr_set_session_state(state: i32) {
    global_engine().set_session_state(xr::SessionState::from_raw(state));
}

#[unsafe(no_mangle)]
pub extern "C" fn mock_xr_advance_time_ns(delta_ns: i64) -> i64 {
    global_engine().advance_time_ns(delta_ns)
}

#[unsafe(no_mangle)]
pub extern "C" fn mock_xr_get_diagnostics_json() -> *mut c_char {
    let diag = global_engine().diagnostics();
    match serde_json::to_string(&diag).ok().and_then(|s| CString::new(s).ok()) {
        Some(cs) => cs.into_raw(),
        None => std::ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn mock_xr_free_string(ptr: *mut c_char) {
    if !ptr.is_null() {
        drop(unsafe { CString::from_raw(ptr) });
    }
}
