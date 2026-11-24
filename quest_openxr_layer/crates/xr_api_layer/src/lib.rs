//! OpenXR 1.0/1.1 Implicit API Layer (`XR_APILAYER_ALVR_hand_emulation`)
//! emulating Oculus/Meta Touch controllers from `XR_EXT_hand_tracking` using ALVR bindings.

#![allow(non_snake_case)]

pub mod android;
pub mod dispatch;
pub mod hand_tracking;
pub mod input;
pub mod profiles;
pub mod spaces;
pub mod state;

use openxr_sys as xr;
use std::ffi::c_char;

/// Panic-safe C-ABI guard ensuring Rust panics never unwind across OpenXR FFI boundaries.
#[inline]
pub(crate) fn ffi_guard<F>(f: F) -> xr::Result
where
    F: FnOnce() -> xr::Result + std::panic::UnwindSafe,
{
    match std::panic::catch_unwind(f) {
        Ok(res) => res,
        Err(_) => xr::Result::ERROR_RUNTIME_FAILURE,
    }
}

/// Mandatory OpenXR API layer loader negotiation entry point.
///
/// # Safety
/// Called by the Khronos/Meta OpenXR loader following the OpenXR Loader-Layer Interface v1 spec.
#[no_mangle]
pub unsafe extern "system" fn xrNegotiateLoaderApiLayerInterface(
    loader_info: *const xr::NegotiateLoaderInfo,
    layer_name: *const c_char,
    api_layer_request: *mut xr::NegotiateApiLayerRequest,
) -> xr::Result {
    ffi_guard(|| {
        dispatch::negotiate_loader_api_layer_interface(loader_info, layer_name, api_layer_request)
    })
}

/// Optional Android OpenXR loader initialization hook (`xrInitializeLoaderKHR`) forwarded
/// by the Khronos Android loader so the layer can read app-scoped config files.
///
/// # Safety
/// Called by the Android OpenXR loader with a valid `XrLoaderInitInfoBaseHeaderKHR` pointer.
#[no_mangle]
pub unsafe extern "system" fn xrInitializeLoaderKHR(
    loader_init_info: *const xr::LoaderInitInfoBaseHeaderKHR,
) -> xr::Result {
    ffi_guard(|| android::handle_initialize_loader_khr(loader_init_info))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CString;
    use std::mem::size_of;

    fn make_loader_info(min_api: xr::Version, max_api: xr::Version) -> xr::NegotiateLoaderInfo {
        xr::NegotiateLoaderInfo {
            struct_type: xr::LoaderInterfaceStructs::LOADER_INFO,
            struct_version: xr::LOADER_INFO_STRUCT_VERSION as u32,
            struct_size: size_of::<xr::NegotiateLoaderInfo>(),
            min_interface_version: 1,
            max_interface_version: 1,
            min_api_version: min_api,
            max_api_version: max_api,
        }
    }

    fn make_layer_request() -> xr::NegotiateApiLayerRequest {
        xr::NegotiateApiLayerRequest {
            struct_type: xr::LoaderInterfaceStructs::API_LAYER_REQUEST,
            struct_version: xr::API_LAYER_INFO_STRUCT_VERSION as u32,
            struct_size: size_of::<xr::NegotiateApiLayerRequest>(),
            layer_interface_version: 0,
            layer_api_version: xr::Version::from_raw(0),
            get_instance_proc_addr: None,
            create_api_layer_instance: None,
        }
    }

    #[test]
    fn negotiate_openxr_1_0_and_1_1_clamping() {
        let name = CString::new(state::LAYER_NAME).unwrap();

        // OpenXR 1.0 loader [1.0.0, 1.0.4095] -> must succeed with 1.0.0
        let li_10 = make_loader_info(xr::Version::new(1, 0, 0), xr::Version::new(1, 0, 4095));
        let mut req_10 = make_layer_request();
        let res = unsafe {
            xrNegotiateLoaderApiLayerInterface(&li_10, name.as_ptr(), &mut req_10)
        };
        assert_eq!(res, xr::Result::SUCCESS);
        assert_eq!(req_10.layer_interface_version, 1);
        assert_eq!(req_10.layer_api_version, xr::Version::new(1, 0, 0));
        assert!(req_10.get_instance_proc_addr.is_some());
        assert!(req_10.create_api_layer_instance.is_some());

        // OpenXR 1.1 loader [1.0.0, 1.1.54] -> must succeed with 1.1.0
        let li_11 = make_loader_info(xr::Version::new(1, 0, 0), xr::Version::new(1, 1, 54));
        let mut req_11 = make_layer_request();
        let res = unsafe {
            xrNegotiateLoaderApiLayerInterface(&li_11, std::ptr::null(), &mut req_11)
        };
        assert_eq!(res, xr::Result::SUCCESS);
        assert_eq!(req_11.layer_api_version, xr::Version::new(1, 1, 0));

        // Unsupported version [2.0.0, 2.0.0] -> must fail
        let li_20 = make_loader_info(xr::Version::new(2, 0, 0), xr::Version::new(2, 0, 0));
        let mut req_20 = make_layer_request();
        let res = unsafe {
            xrNegotiateLoaderApiLayerInterface(&li_20, name.as_ptr(), &mut req_20)
        };
        assert_eq!(res, xr::Result::ERROR_INITIALIZATION_FAILED);
    }

    #[test]
    fn negotiate_rejects_null_and_bad_headers() {
        let mut req = make_layer_request();
        assert_eq!(
            unsafe { xrNegotiateLoaderApiLayerInterface(std::ptr::null(), std::ptr::null(), &mut req) },
            xr::Result::ERROR_INITIALIZATION_FAILED
        );
        let mut bad_li = make_loader_info(xr::Version::new(1, 0, 0), xr::Version::new(1, 1, 54));
        bad_li.struct_size -= 1;
        assert_eq!(
            unsafe { xrNegotiateLoaderApiLayerInterface(&bad_li, std::ptr::null(), &mut req) },
            xr::Result::ERROR_INITIALIZATION_FAILED
        );
    }

    #[test]
    fn initialize_loader_khr_validation() {
        assert_eq!(
            unsafe { xrInitializeLoaderKHR(std::ptr::null()) },
            xr::Result::ERROR_VALIDATION_FAILURE
        );
        let bad_android = android::LoaderInitInfoAndroidKHRRaw {
            ty: xr::StructureType::LOADER_INIT_INFO_ANDROID_KHR,
            next: std::ptr::null(),
            application_vm: std::ptr::null_mut(),
            application_context: std::ptr::null_mut(),
        };
        assert_eq!(
            unsafe {
                xrInitializeLoaderKHR(&bad_android as *const _ as *const xr::LoaderInitInfoBaseHeaderKHR)
            },
            xr::Result::ERROR_VALIDATION_FAILURE
        );
        let mut dummy = 1u8;
        let ok_android = android::LoaderInitInfoAndroidKHRRaw {
            ty: xr::StructureType::LOADER_INIT_INFO_ANDROID_KHR,
            next: std::ptr::null(),
            application_vm: &mut dummy as *mut u8 as *mut std::ffi::c_void,
            application_context: &mut dummy as *mut u8 as *mut std::ffi::c_void,
        };
        assert_eq!(
            unsafe {
                xrInitializeLoaderKHR(&ok_android as *const _ as *const xr::LoaderInitInfoBaseHeaderKHR)
            },
            xr::Result::SUCCESS
        );
    }
}
