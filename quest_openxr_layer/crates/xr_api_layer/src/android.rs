//! Android JNI `xrInitializeLoaderKHR` capture and `__android_log_write` logger backend.

use alvr_hand_core::config::DEFAULT_CONFIG_FILENAME;
use openxr_sys as xr;
use parking_lot::RwLock;
use std::path::PathBuf;
use std::sync::{LazyLock, Once};

static LOG_INIT: Once = Once::new();
static ANDROID_EXTERNAL_FILES_DIR: LazyLock<RwLock<Option<PathBuf>>> =
    LazyLock::new(|| RwLock::new(None));

/// Initializes platform logging once (Android `logcat` via `__android_log_write` on Android;
/// no-op on desktop where host harnesses may already register a logger).
pub fn init_logging() {
    LOG_INIT.call_once(|| {
        #[cfg(target_os = "android")]
        {
            let _ = log::set_logger(&ANDROID_LOGGER);
            log::set_max_level(log::LevelFilter::Info);
        }
    });
}

/// Returns candidate config file paths discovered from Android loader initialization.
pub fn android_config_candidates() -> Vec<PathBuf> {
    let mut out = Vec::new();
    if let Some(dir) = ANDROID_EXTERNAL_FILES_DIR.read().as_ref() {
        out.push(dir.join(DEFAULT_CONFIG_FILENAME));
    }
    out
}

/// Records an Android external files directory (exposed for tests and JNI capture).
pub fn set_android_external_files_dir(dir: Option<PathBuf>) {
    *ANDROID_EXTERNAL_FILES_DIR.write() = dir;
}

/// Portable `#[repr(C)]` layout of `XrLoaderInitInfoAndroidKHR` (`openxr_platform.h:451-456`)
/// available on both Android targets and Linux host test builds.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct LoaderInitInfoAndroidKHRRaw {
    pub ty: xr::StructureType,
    pub next: *const std::ffi::c_void,
    pub application_vm: *mut std::ffi::c_void,
    pub application_context: *mut std::ffi::c_void,
}

/// Handles `xrInitializeLoaderKHR` forwarded by the Android OpenXR loader.
///
/// Validates `XrLoaderInitInfoAndroidKHR` fields and, on Android targets, extracts
/// `Context.getExternalFilesDir(null).getAbsolutePath()` via raw JNI without any external crate.
///
/// # Safety
/// `loader_init_info` must be null or point to a valid `XrLoaderInitInfoBaseHeaderKHR` struct.
pub unsafe fn handle_initialize_loader_khr(
    loader_init_info: *const xr::LoaderInitInfoBaseHeaderKHR,
) -> xr::Result {
    init_logging();

    if loader_init_info.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }

    let base = &*loader_init_info;
    if base.ty == xr::StructureType::LOADER_INIT_INFO_ANDROID_KHR {
        let android_info = &*(loader_init_info as *const LoaderInitInfoAndroidKHRRaw);
        if android_info.application_vm.is_null() || android_info.application_context.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        #[cfg(target_os = "android")]
        {
            if let Some(dir) = extract_external_files_dir(
                android_info.application_vm,
                android_info.application_context,
            ) {
                log::info!(
                    "Captured Android external files dir for ALVR hand layer config: {}",
                    dir.display()
                );
                set_android_external_files_dir(Some(dir));
            }
        }
    }

    xr::Result::SUCCESS
}

#[cfg(target_os = "android")]
struct AndroidLogger;

#[cfg(target_os = "android")]
static ANDROID_LOGGER: AndroidLogger = AndroidLogger;

#[cfg(target_os = "android")]
extern "C" {
    fn __android_log_write(
        prio: libc::c_int,
        tag: *const libc::c_char,
        text: *const libc::c_char,
    ) -> libc::c_int;
}

#[cfg(target_os = "android")]
impl log::Log for AndroidLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.level() <= log::Level::Info
    }

    fn log(&self, record: &log::Record<'_>) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let prio: libc::c_int = match record.level() {
            log::Level::Error => 6, // ANDROID_LOG_ERROR
            log::Level::Warn => 5,  // ANDROID_LOG_WARN
            log::Level::Info => 4,  // ANDROID_LOG_INFO
            log::Level::Debug => 3, // ANDROID_LOG_DEBUG
            log::Level::Trace => 2, // ANDROID_LOG_VERBOSE
        };
        let msg = format!("{}", record.args());
        if let Ok(c_msg) = std::ffi::CString::new(msg) {
            const TAG: &[u8] = b"ALVRHandLayer\0";
            unsafe {
                __android_log_write(
                    prio,
                    TAG.as_ptr() as *const libc::c_char,
                    c_msg.as_ptr() as *const libc::c_char,
                );
            }
        }
    }

    fn flush(&self) {}
}

/// Minimal raw-JNI lookup of `context.getExternalFilesDir(null).getAbsolutePath()`.
/// Avoids pulling in heavy `jni` crate dependencies while staying safe and exception-clean.
#[cfg(target_os = "android")]
unsafe fn extract_external_files_dir(
    vm_ptr: *mut std::ffi::c_void,
    ctx_obj: *mut std::ffi::c_void,
) -> Option<PathBuf> {
    use std::ffi::{c_char, c_int, c_void, CStr};

    const JNI_OK: c_int = 0;
    const JNI_EDETACHED: c_int = -2;
    const JNI_VERSION_1_6: c_int = 0x0001_0006;

    // JNIInvokeInterface_ function table offsets (indices in pointer array):
    // 4 = AttachCurrentThread, 5 = DetachCurrentThread, 6 = GetEnv
    let vm_vtable = *(vm_ptr as *const *const *const c_void);
    if vm_vtable.is_null() {
        return None;
    }

    type GetEnvFn = unsafe extern "C" fn(*mut c_void, *mut *mut c_void, c_int) -> c_int;
    type AttachFn = unsafe extern "C" fn(*mut c_void, *mut *mut c_void, *mut c_void) -> c_int;
    type DetachFn = unsafe extern "C" fn(*mut c_void) -> c_int;

    let get_env: GetEnvFn = std::mem::transmute(*vm_vtable.add(6));
    let attach: AttachFn = std::mem::transmute(*vm_vtable.add(4));
    let detach: DetachFn = std::mem::transmute(*vm_vtable.add(5));

    let mut env_ptr: *mut c_void = std::ptr::null_mut();
    let mut attached_here = false;
    let status = get_env(vm_ptr, &mut env_ptr, JNI_VERSION_1_6);
    if status == JNI_EDETACHED {
        if attach(vm_ptr, &mut env_ptr, std::ptr::null_mut()) != JNI_OK {
            return None;
        }
        attached_here = true;
    } else if status != JNI_OK {
        return None;
    }

    if env_ptr.is_null() {
        if attached_here {
            detach(vm_ptr);
        }
        return None;
    }

    // JNINativeInterface_ table indices (standard JNI 1.6 ABI):
    // 15 = ExceptionCheck, 16 = ExceptionClear, 23 = DeleteLocalRef,
    // 31 = GetObjectClass, 33 = GetMethodID, 34 = CallObjectMethod,
    // 169 = GetStringUTFChars, 170 = ReleaseStringUTFChars
    let env_vtable = *(env_ptr as *const *const *const c_void);
    if env_vtable.is_null() {
        if attached_here {
            detach(vm_ptr);
        }
        return None;
    }

    type ExceptionCheckFn = unsafe extern "C" fn(*mut c_void) -> u8;
    type ExceptionClearFn = unsafe extern "C" fn(*mut c_void);
    type DeleteLocalRefFn = unsafe extern "C" fn(*mut c_void, *mut c_void);
    type GetObjectClassFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> *mut c_void;
    type GetMethodIdFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *const c_char, *const c_char) -> *mut c_void;
    type CallObjMethod1Fn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, ...) -> *mut c_void;
    type GetStringUtfCharsFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut u8) -> *const c_char;
    type ReleaseStringUtfCharsFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *const c_char);

    let exception_check: ExceptionCheckFn = std::mem::transmute(*env_vtable.add(15));
    let exception_clear: ExceptionClearFn = std::mem::transmute(*env_vtable.add(16));
    let delete_local_ref: DeleteLocalRefFn = std::mem::transmute(*env_vtable.add(23));
    let get_object_class: GetObjectClassFn = std::mem::transmute(*env_vtable.add(31));
    let get_method_id: GetMethodIdFn = std::mem::transmute(*env_vtable.add(33));
    let call_object_method: CallObjMethod1Fn = std::mem::transmute(*env_vtable.add(34));
    let get_string_utf: GetStringUtfCharsFn = std::mem::transmute(*env_vtable.add(169));
    let release_string_utf: ReleaseStringUtfCharsFn = std::mem::transmute(*env_vtable.add(170));

    let clear_if_exc = || {
        if exception_check(env_ptr) != 0 {
            exception_clear(env_ptr);
            true
        } else {
            false
        }
    };

    let mut result: Option<PathBuf> = None;

    let ctx_cls = get_object_class(env_ptr, ctx_obj);
    if !ctx_cls.is_null() && !clear_if_exc() {
        let mid_ext = get_method_id(
            env_ptr,
            ctx_cls,
            b"getExternalFilesDir\0".as_ptr() as *const c_char,
            b"(Ljava/lang/String;)Ljava/io/File;\0".as_ptr() as *const c_char,
        );
        if !mid_ext.is_null() && !clear_if_exc() {
            let file_obj =
                call_object_method(env_ptr, ctx_obj, mid_ext, std::ptr::null_mut::<c_void>());
            if !file_obj.is_null() && !clear_if_exc() {
                let file_cls = get_object_class(env_ptr, file_obj);
                if !file_cls.is_null() && !clear_if_exc() {
                    let mid_path = get_method_id(
                        env_ptr,
                        file_cls,
                        b"getAbsolutePath\0".as_ptr() as *const c_char,
                        b"()Ljava/lang/String;\0".as_ptr() as *const c_char,
                    );
                    if !mid_path.is_null() && !clear_if_exc() {
                        let jstr = call_object_method(env_ptr, file_obj, mid_path);
                        if !jstr.is_null() && !clear_if_exc() {
                            let chars = get_string_utf(env_ptr, jstr, std::ptr::null_mut());
                            if !chars.is_null() {
                                if let Ok(s) = CStr::from_ptr(chars).to_str() {
                                    if !s.is_empty() {
                                        result = Some(PathBuf::from(s));
                                    }
                                }
                                release_string_utf(env_ptr, jstr, chars);
                            }
                            delete_local_ref(env_ptr, jstr);
                        }
                    }
                    delete_local_ref(env_ptr, file_cls);
                }
                delete_local_ref(env_ptr, file_obj);
            }
        }
        delete_local_ref(env_ptr, ctx_cls);
    }

    if attached_here {
        detach(vm_ptr);
    }

    result
}
