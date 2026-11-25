//! Android JNI `xrInitializeLoaderKHR` capture and `__android_log_write` logger backend.

use alvr_hand_core::config::DEFAULT_CONFIG_FILENAME;
use alvr_hand_core::LayerConfig;
use openxr_sys as xr;
use parking_lot::RwLock;
use std::ffi::{c_char, c_int, c_void, CStr};
use std::path::PathBuf;
use std::sync::{LazyLock, Once};

/// Maximum APK asset config size read via Android AssetManager (matches `alvr_hand_core`).
const MAX_ASSET_CONFIG_BYTES: usize = 1024 * 1024;

static LOG_INIT: Once = Once::new();
static ANDROID_EXTERNAL_FILES_DIR: LazyLock<RwLock<Option<PathBuf>>> =
    LazyLock::new(|| RwLock::new(None));
static ANDROID_INTERNAL_FILES_DIR: LazyLock<RwLock<Option<PathBuf>>> =
    LazyLock::new(|| RwLock::new(None));
static ANDROID_ASSET_CONFIG: LazyLock<RwLock<Option<LayerConfig>>> =
    LazyLock::new(|| RwLock::new(None));

/// Initializes platform logging once (Android `logcat` via `__android_log_write` on Android;
/// no-op on desktop where host harnesses may already register a logger).
pub fn init_logging() {
    LOG_INIT.call_once(|| {
        #[cfg(target_os = "android")]
        {
            let level = match std::env::var("ALVR_HAND_LAYER_LOG")
                .unwrap_or_default()
                .to_ascii_lowercase()
                .as_str()
            {
                "trace" | "verbose" => log::LevelFilter::Trace,
                "debug" => log::LevelFilter::Debug,
                "warn" => log::LevelFilter::Warn,
                "error" => log::LevelFilter::Error,
                "off" => log::LevelFilter::Off,
                _ => log::LevelFilter::Info,
            };
            let _ = log::set_logger(&ANDROID_LOGGER);
            log::set_max_level(level);
        }
    });
}

/// Returns candidate filesystem config paths discovered from Android loader initialization
/// in priority order: external files dir first (user-pushable via `adb push`), then internal files dir.
pub fn android_config_candidates() -> Vec<PathBuf> {
    let mut out = Vec::new();
    if let Some(dir) = ANDROID_EXTERNAL_FILES_DIR.read().as_ref() {
        out.push(dir.join(DEFAULT_CONFIG_FILENAME));
    }
    if let Some(dir) = ANDROID_INTERNAL_FILES_DIR.read().as_ref() {
        let p = dir.join(DEFAULT_CONFIG_FILENAME);
        if !out.contains(&p) {
            out.push(p);
        }
    }
    out
}

/// Returns an optional baked-in `LayerConfig` loaded from APK `assets/alvr_hand_controller.json`
/// during `xrInitializeLoaderKHR`.
pub fn android_asset_config() -> Option<LayerConfig> {
    ANDROID_ASSET_CONFIG.read().clone()
}

/// Records an Android external files directory (exposed for tests and JNI capture).
pub fn set_android_external_files_dir(dir: Option<PathBuf>) {
    *ANDROID_EXTERNAL_FILES_DIR.write() = dir;
}

/// Records an Android internal files directory (exposed for tests and JNI capture).
pub fn set_android_internal_files_dir(dir: Option<PathBuf>) {
    *ANDROID_INTERNAL_FILES_DIR.write() = dir;
}

/// Records a baked-in APK asset config (exposed for tests and JNI capture).
pub fn set_android_asset_config(cfg: Option<LayerConfig>) {
    *ANDROID_ASSET_CONFIG.write() = cfg;
}

/// Portable `#[repr(C)]` layout of `XrLoaderInitInfoAndroidKHR` (`openxr_platform.h:451-456`)
/// available on both Android targets and Linux host test builds.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct LoaderInitInfoAndroidKHRRaw {
    pub ty: xr::StructureType,
    pub next: *const c_void,
    pub application_vm: *mut c_void,
    pub application_context: *mut c_void,
}

/// Result of raw JNI Android context inspection.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct AndroidContextInfo {
    pub external_files_dir: Option<PathBuf>,
    pub internal_files_dir: Option<PathBuf>,
    pub asset_config_json: Option<String>,
}

/// Handles `xrInitializeLoaderKHR` forwarded by the Android OpenXR loader.
///
/// Validates `XrLoaderInitInfoAndroidKHR` fields and, on Android targets, extracts:
/// - `Context.getExternalFilesDir(null).getAbsolutePath()`
/// - `Context.getFilesDir().getAbsolutePath()`
/// - Optional baked-in `assets/alvr_hand_controller.json` via `Context.getAssets().open(...)`
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
            let info = extract_android_context_info_raw(
                android_info.application_vm,
                android_info.application_context,
            );
            if let Some(dir) = info.external_files_dir {
                log::info!(
                    "Captured Android external files dir for ALVR hand layer config: {}",
                    dir.display()
                );
                set_android_external_files_dir(Some(dir));
            }
            if let Some(dir) = info.internal_files_dir {
                log::debug!(
                    "Captured Android internal files dir for ALVR hand layer config: {}",
                    dir.display()
                );
                set_android_internal_files_dir(Some(dir));
            }
            if let Some(json) = info.asset_config_json {
                match LayerConfig::from_json_str(&json) {
                    Ok(cfg) => {
                        log::info!("Loaded baked-in APK asset config ({DEFAULT_CONFIG_FILENAME})");
                        set_android_asset_config(Some(cfg));
                    }
                    Err(err) => {
                        log::warn!(
                            "Ignoring malformed APK asset config ({DEFAULT_CONFIG_FILENAME}): {err}"
                        );
                    }
                }
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
        metadata.level() <= log::max_level()
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
        let raw_msg = format!("{}", record.args());
        let safe_msg = if raw_msg.contains('\0') {
            raw_msg.replace('\0', " ")
        } else {
            raw_msg
        };
        if let Ok(c_msg) = std::ffi::CString::new(safe_msg) {
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

/// Standard JNI `jvalue` union (`jni.h`, 8 bytes on 64-bit platforms).
#[repr(C)]
#[derive(Clone, Copy)]
pub union JValue {
    pub l: *mut c_void,
    pub i: i32,
    pub j: i64,
}

/// Standard JNI 1.6 `JNIInvokeInterface_` function table indices (`jni.h`).
pub(crate) const JNI_VM_ATTACH_CURRENT_THREAD: usize = 4;
pub(crate) const JNI_VM_DETACH_CURRENT_THREAD: usize = 5;
pub(crate) const JNI_VM_GET_ENV: usize = 6;

/// Standard JNI 1.6 `JNINativeInterface_` function table indices (`jni.h`).
///
/// NOTE: Indices 15 (`ExceptionOccurred`) and 16 (`ExceptionDescribe`) are NOT `ExceptionCheck`
/// or `ExceptionClear`! Using `ExceptionCheck` (228) and `ExceptionClear` (17) + `Call*MethodA`
/// (36/51/63) avoids leaking `jthrowable` refs, avoids C-varargs ABI pitfalls on AArch64, and
/// ensures pending Java exceptions are always properly cleared before returning to ART.
pub(crate) const JNI_ENV_EXCEPTION_CLEAR: usize = 17;
pub(crate) const JNI_ENV_DELETE_LOCAL_REF: usize = 23;
pub(crate) const JNI_ENV_GET_OBJECT_CLASS: usize = 31;
pub(crate) const JNI_ENV_GET_METHOD_ID: usize = 33;
pub(crate) const JNI_ENV_CALL_OBJECT_METHOD_A: usize = 36;
pub(crate) const JNI_ENV_CALL_INT_METHOD_A: usize = 51;
pub(crate) const JNI_ENV_CALL_VOID_METHOD_A: usize = 63;
pub(crate) const JNI_ENV_NEW_STRING_UTF: usize = 167;
pub(crate) const JNI_ENV_GET_STRING_UTF_CHARS: usize = 169;
pub(crate) const JNI_ENV_RELEASE_STRING_UTF_CHARS: usize = 170;
pub(crate) const JNI_ENV_NEW_BYTE_ARRAY: usize = 176;
pub(crate) const JNI_ENV_GET_BYTE_ARRAY_REGION: usize = 200;
pub(crate) const JNI_ENV_EXCEPTION_CHECK: usize = 228;

/// Minimal raw-JNI inspection of Android `Context`:
/// 1. `context.getExternalFilesDir(null).getAbsolutePath()`
/// 2. `context.getFilesDir().getAbsolutePath()`
/// 3. Optional `context.getAssets().open("alvr_hand_controller.json")` (capped at 1 MiB)
///
/// Uses non-variadic `Call*MethodA` and exact JNI 1.6 vtable offsets (`ExceptionCheck = 228`,
/// `ExceptionClear = 17`). Compiled unconditionally so host unit tests exercise the exact ABI
/// layout against a mock JNI vtable.
///
/// # Safety
/// `vm_ptr` must be a valid `JavaVM*` and `ctx_obj` must be a valid Android `Context` `jobject`.
pub unsafe fn extract_android_context_info_raw(
    vm_ptr: *mut c_void,
    ctx_obj: *mut c_void,
) -> AndroidContextInfo {
    const JNI_OK: c_int = 0;
    const JNI_EDETACHED: c_int = -2;
    const JNI_VERSION_1_6: c_int = 0x0001_0006;

    let mut info = AndroidContextInfo::default();
    if vm_ptr.is_null() || ctx_obj.is_null() {
        return info;
    }

    let vm_vtable = *(vm_ptr as *const *const *const c_void);
    if vm_vtable.is_null() {
        return info;
    }

    type GetEnvFn = unsafe extern "C" fn(*mut c_void, *mut *mut c_void, c_int) -> c_int;
    type AttachFn = unsafe extern "C" fn(*mut c_void, *mut *mut c_void, *mut c_void) -> c_int;
    type DetachFn = unsafe extern "C" fn(*mut c_void) -> c_int;

    let get_env_ptr = *vm_vtable.add(JNI_VM_GET_ENV);
    let attach_ptr = *vm_vtable.add(JNI_VM_ATTACH_CURRENT_THREAD);
    let detach_ptr = *vm_vtable.add(JNI_VM_DETACH_CURRENT_THREAD);
    if get_env_ptr.is_null() || attach_ptr.is_null() || detach_ptr.is_null() {
        return info;
    }
    let get_env: GetEnvFn = std::mem::transmute(get_env_ptr);
    let attach: AttachFn = std::mem::transmute(attach_ptr);
    let detach: DetachFn = std::mem::transmute(detach_ptr);

    let mut env_ptr: *mut c_void = std::ptr::null_mut();
    let mut attached_here = false;
    let status = get_env(vm_ptr, &mut env_ptr, JNI_VERSION_1_6);
    if status == JNI_EDETACHED {
        if attach(vm_ptr, &mut env_ptr, std::ptr::null_mut()) != JNI_OK {
            return info;
        }
        attached_here = true;
    } else if status != JNI_OK {
        return info;
    }

    if env_ptr.is_null() {
        if attached_here {
            detach(vm_ptr);
        }
        return info;
    }

    let env_vtable = *(env_ptr as *const *const *const c_void);
    if env_vtable.is_null() {
        if attached_here {
            detach(vm_ptr);
        }
        return info;
    }

    type ExceptionCheckFn = unsafe extern "C" fn(*mut c_void) -> u8;
    type ExceptionClearFn = unsafe extern "C" fn(*mut c_void);
    type DeleteLocalRefFn = unsafe extern "C" fn(*mut c_void, *mut c_void);
    type GetObjectClassFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> *mut c_void;
    type GetMethodIdFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *const c_char, *const c_char) -> *mut c_void;
    type CallObjMethodAFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *const JValue) -> *mut c_void;
    type CallIntMethodAFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *const JValue) -> i32;
    type CallVoidMethodAFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void, *const JValue);
    type NewStringUtfFn = unsafe extern "C" fn(*mut c_void, *const c_char) -> *mut c_void;
    type GetStringUtfCharsFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *mut u8) -> *const c_char;
    type ReleaseStringUtfCharsFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, *const c_char);
    type NewByteArrayFn = unsafe extern "C" fn(*mut c_void, i32) -> *mut c_void;
    type GetByteArrayRegionFn =
        unsafe extern "C" fn(*mut c_void, *mut c_void, i32, i32, *mut i8);

    macro_rules! load_jni {
        ($idx:expr, $ty:ty) => {{
            let p = *env_vtable.add($idx);
            if p.is_null() {
                if attached_here {
                    detach(vm_ptr);
                }
                return info;
            }
            std::mem::transmute::<*const c_void, $ty>(p)
        }};
    }

    let exception_check = load_jni!(JNI_ENV_EXCEPTION_CHECK, ExceptionCheckFn);
    let exception_clear = load_jni!(JNI_ENV_EXCEPTION_CLEAR, ExceptionClearFn);
    let delete_local_ref = load_jni!(JNI_ENV_DELETE_LOCAL_REF, DeleteLocalRefFn);
    let get_object_class = load_jni!(JNI_ENV_GET_OBJECT_CLASS, GetObjectClassFn);
    let get_method_id = load_jni!(JNI_ENV_GET_METHOD_ID, GetMethodIdFn);
    let call_obj_a = load_jni!(JNI_ENV_CALL_OBJECT_METHOD_A, CallObjMethodAFn);
    let call_int_a = load_jni!(JNI_ENV_CALL_INT_METHOD_A, CallIntMethodAFn);
    let call_void_a = load_jni!(JNI_ENV_CALL_VOID_METHOD_A, CallVoidMethodAFn);
    let new_string_utf = load_jni!(JNI_ENV_NEW_STRING_UTF, NewStringUtfFn);
    let get_string_utf = load_jni!(JNI_ENV_GET_STRING_UTF_CHARS, GetStringUtfCharsFn);
    let release_string_utf = load_jni!(JNI_ENV_RELEASE_STRING_UTF_CHARS, ReleaseStringUtfCharsFn);
    let new_byte_array = load_jni!(JNI_ENV_NEW_BYTE_ARRAY, NewByteArrayFn);
    let get_byte_array_region = load_jni!(JNI_ENV_GET_BYTE_ARRAY_REGION, GetByteArrayRegionFn);

    let clear_if_exc = || -> bool {
        if exception_check(env_ptr) != 0 {
            exception_clear(env_ptr);
            true
        } else {
            false
        }
    };

    // Helper: extract `file_obj.getAbsolutePath()` and release `file_obj`.
    let extract_file_path = |file_obj: *mut c_void| -> Option<PathBuf> {
        if file_obj.is_null() {
            return None;
        }
        let mut path_out = None;
        let file_cls = get_object_class(env_ptr, file_obj);
        if !clear_if_exc() && !file_cls.is_null() {
            let mid_path = get_method_id(
                env_ptr,
                file_cls,
                b"getAbsolutePath\0".as_ptr() as *const c_char,
                b"()Ljava/lang/String;\0".as_ptr() as *const c_char,
            );
            if !clear_if_exc() && !mid_path.is_null() {
                let jstr = call_obj_a(env_ptr, file_obj, mid_path, std::ptr::null());
                if !clear_if_exc() && !jstr.is_null() {
                    let chars = get_string_utf(env_ptr, jstr, std::ptr::null_mut());
                    if !clear_if_exc() && !chars.is_null() {
                        if let Ok(s) = CStr::from_ptr(chars).to_str() {
                            if !s.is_empty() {
                                path_out = Some(PathBuf::from(s));
                            }
                        }
                        release_string_utf(env_ptr, jstr, chars);
                    }
                    delete_local_ref(env_ptr, jstr);
                } else if !jstr.is_null() {
                    delete_local_ref(env_ptr, jstr);
                }
            }
            delete_local_ref(env_ptr, file_cls);
        }
        delete_local_ref(env_ptr, file_obj);
        path_out
    };

    // Never leave a pre-existing exception unhandled, and do not clobber host app exceptions.
    if exception_check(env_ptr) == 0 {
        let ctx_cls = get_object_class(env_ptr, ctx_obj);
        if !clear_if_exc() && !ctx_cls.is_null() {
            // 1. Context.getExternalFilesDir(null)
            let mid_ext = get_method_id(
                env_ptr,
                ctx_cls,
                b"getExternalFilesDir\0".as_ptr() as *const c_char,
                b"(Ljava/lang/String;)Ljava/io/File;\0".as_ptr() as *const c_char,
            );
            if !clear_if_exc() && !mid_ext.is_null() {
                let args = [JValue {
                    l: std::ptr::null_mut(),
                }];
                let file_obj = call_obj_a(env_ptr, ctx_obj, mid_ext, args.as_ptr());
                if !clear_if_exc() {
                    info.external_files_dir = extract_file_path(file_obj);
                } else if !file_obj.is_null() {
                    delete_local_ref(env_ptr, file_obj);
                }
            }

            // 2. Context.getFilesDir()
            let mid_files = get_method_id(
                env_ptr,
                ctx_cls,
                b"getFilesDir\0".as_ptr() as *const c_char,
                b"()Ljava/io/File;\0".as_ptr() as *const c_char,
            );
            if !clear_if_exc() && !mid_files.is_null() {
                let file_obj = call_obj_a(env_ptr, ctx_obj, mid_files, std::ptr::null());
                if !clear_if_exc() {
                    info.internal_files_dir = extract_file_path(file_obj);
                } else if !file_obj.is_null() {
                    delete_local_ref(env_ptr, file_obj);
                }
            }

            // 3. Optional baked-in APK asset: context.getAssets().open("alvr_hand_controller.json")
            let mid_assets = get_method_id(
                env_ptr,
                ctx_cls,
                b"getAssets\0".as_ptr() as *const c_char,
                b"()Landroid/content/res/AssetManager;\0".as_ptr() as *const c_char,
            );
            if !clear_if_exc() && !mid_assets.is_null() {
                let am_obj = call_obj_a(env_ptr, ctx_obj, mid_assets, std::ptr::null());
                if !clear_if_exc() && !am_obj.is_null() {
                    let am_cls = get_object_class(env_ptr, am_obj);
                    if !clear_if_exc() && !am_cls.is_null() {
                        let mid_open = get_method_id(
                            env_ptr,
                            am_cls,
                            b"open\0".as_ptr() as *const c_char,
                            b"(Ljava/lang/String;)Ljava/io/InputStream;\0".as_ptr()
                                as *const c_char,
                        );
                        if !clear_if_exc() && !mid_open.is_null() {
                            let asset_name_c = b"alvr_hand_controller.json\0";
                            let jname =
                                new_string_utf(env_ptr, asset_name_c.as_ptr() as *const c_char);
                            if !clear_if_exc() && !jname.is_null() {
                                let open_args = [JValue { l: jname }];
                                let stream_obj =
                                    call_obj_a(env_ptr, am_obj, mid_open, open_args.as_ptr());
                                delete_local_ref(env_ptr, jname);
                                if !clear_if_exc() && !stream_obj.is_null() {
                                    let stream_cls = get_object_class(env_ptr, stream_obj);
                                    if !clear_if_exc() && !stream_cls.is_null() {
                                        let mid_read = get_method_id(
                                            env_ptr,
                                            stream_cls,
                                            b"read\0".as_ptr() as *const c_char,
                                            b"([B)I\0".as_ptr() as *const c_char,
                                        );
                                        let mid_close = get_method_id(
                                            env_ptr,
                                            stream_cls,
                                            b"close\0".as_ptr() as *const c_char,
                                            b"()V\0".as_ptr() as *const c_char,
                                        );
                                        if !clear_if_exc() && !mid_read.is_null() {
                                            const CHUNK: usize = 4096;
                                            let jbuf = new_byte_array(env_ptr, CHUNK as i32);
                                            if !clear_if_exc() && !jbuf.is_null() {
                                                let mut raw_bytes = Vec::new();
                                                let mut local_buf = [0u8; CHUNK];
                                                let mut failed = false;
                                                loop {
                                                    let read_args = [JValue { l: jbuf }];
                                                    let n = call_int_a(
                                                        env_ptr,
                                                        stream_obj,
                                                        mid_read,
                                                        read_args.as_ptr(),
                                                    );
                                                    if clear_if_exc() {
                                                        failed = true;
                                                        break;
                                                    }
                                                    if n <= 0 {
                                                        break;
                                                    }
                                                    let n_usize = (n as usize).min(CHUNK);
                                                    if raw_bytes.len() + n_usize
                                                        > MAX_ASSET_CONFIG_BYTES
                                                    {
                                                        failed = true;
                                                        break;
                                                    }
                                                    get_byte_array_region(
                                                        env_ptr,
                                                        jbuf,
                                                        0,
                                                        n_usize as i32,
                                                        local_buf.as_mut_ptr() as *mut i8,
                                                    );
                                                    if clear_if_exc() {
                                                        failed = true;
                                                        break;
                                                    }
                                                    raw_bytes
                                                        .extend_from_slice(&local_buf[..n_usize]);
                                                }
                                                delete_local_ref(env_ptr, jbuf);
                                                if !failed && !raw_bytes.is_empty() {
                                                    if let Ok(text) = String::from_utf8(raw_bytes) {
                                                        info.asset_config_json = Some(text);
                                                    }
                                                }
                                            }
                                        }
                                        if !mid_close.is_null() {
                                            call_void_a(
                                                env_ptr,
                                                stream_obj,
                                                mid_close,
                                                std::ptr::null(),
                                            );
                                            let _ = clear_if_exc();
                                        }
                                        delete_local_ref(env_ptr, stream_cls);
                                    }
                                    delete_local_ref(env_ptr, stream_obj);
                                } else if !stream_obj.is_null() {
                                    delete_local_ref(env_ptr, stream_obj);
                                }
                            }
                        }
                        delete_local_ref(env_ptr, am_cls);
                    }
                    delete_local_ref(env_ptr, am_obj);
                } else if !am_obj.is_null() {
                    delete_local_ref(env_ptr, am_obj);
                }
            }

            delete_local_ref(env_ptr, ctx_cls);
        }
    }

    if attached_here {
        detach(vm_ptr);
    }

    info
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CString;
    use std::sync::atomic::{AtomicBool, AtomicI32, AtomicUsize, Ordering};

    struct MockJniState {
        detached_initially: bool,
        attach_calls: AtomicUsize,
        detach_calls: AtomicUsize,
        exception_pending: AtomicBool,
        exception_clears: AtomicUsize,
        live_local_refs: AtomicI32,
        fail_asset_open: bool,
        asset_bytes: Vec<u8>,
        asset_read_offset: AtomicUsize,
        ext_path: CString,
        int_path: CString,
    }

    #[repr(C)]
    struct MockVm {
        vtable: *const *const c_void,
        env_ptr: *mut c_void,
        state: *const MockJniState,
    }

    #[repr(C)]
    struct MockEnv {
        vtable: *const *const c_void,
        state: *const MockJniState,
    }

    const OBJ_CTX: usize = 0x1001;
    const OBJ_FILE_EXT: usize = 0x1002;
    const OBJ_FILE_INT: usize = 0x1003;
    const OBJ_AM: usize = 0x1004;
    const OBJ_STREAM: usize = 0x1005;
    const OBJ_STR_EXT: usize = 0x1006;
    const OBJ_STR_INT: usize = 0x1007;
    const OBJ_STR_ASSET_NAME: usize = 0x1008;
    const OBJ_BYTE_ARRAY: usize = 0x1009;
    const CLS_CTX: usize = 0x2001;
    const CLS_FILE: usize = 0x2002;
    const CLS_AM: usize = 0x2003;
    const CLS_STREAM: usize = 0x2004;
    const MID_EXT_FILES: usize = 0x3001;
    const MID_FILES: usize = 0x3002;
    const MID_ABS_PATH: usize = 0x3003;
    const MID_GET_ASSETS: usize = 0x3004;
    const MID_OPEN: usize = 0x3005;
    const MID_READ: usize = 0x3006;
    const MID_CLOSE: usize = 0x3007;

    unsafe extern "C" fn mock_get_env(
        vm: *mut c_void,
        penv: *mut *mut c_void,
        _ver: c_int,
    ) -> c_int {
        let mvm = &*(vm as *const MockVm);
        let st = &*mvm.state;
        if st.detached_initially {
            *penv = std::ptr::null_mut();
            -2 // JNI_EDETACHED
        } else {
            *penv = mvm.env_ptr;
            0
        }
    }

    unsafe extern "C" fn mock_attach(
        vm: *mut c_void,
        penv: *mut *mut c_void,
        _args: *mut c_void,
    ) -> c_int {
        let mvm = &*(vm as *const MockVm);
        (*mvm.state).attach_calls.fetch_add(1, Ordering::SeqCst);
        *penv = mvm.env_ptr;
        0
    }

    unsafe extern "C" fn mock_detach(vm: *mut c_void) -> c_int {
        let mvm = &*(vm as *const MockVm);
        (*mvm.state).detach_calls.fetch_add(1, Ordering::SeqCst);
        0
    }

    unsafe extern "C" fn mock_exc_check(env: *mut c_void) -> u8 {
        let menv = &*(env as *const MockEnv);
        u8::from((*menv.state).exception_pending.load(Ordering::SeqCst))
    }

    unsafe extern "C" fn mock_exc_clear(env: *mut c_void) {
        let menv = &*(env as *const MockEnv);
        let st = &*menv.state;
        st.exception_pending.store(false, Ordering::SeqCst);
        st.exception_clears.fetch_add(1, Ordering::SeqCst);
    }

    unsafe extern "C" fn mock_del_local_ref(env: *mut c_void, obj: *mut c_void) {
        assert!(!obj.is_null());
        let menv = &*(env as *const MockEnv);
        (*menv.state).live_local_refs.fetch_sub(1, Ordering::SeqCst);
    }

    unsafe extern "C" fn mock_get_obj_cls(env: *mut c_void, obj: *mut c_void) -> *mut c_void {
        let menv = &*(env as *const MockEnv);
        let st = &*menv.state;
        assert!(
            !st.exception_pending.load(Ordering::SeqCst),
            "ART CheckJNI violation: GetObjectClass called with pending exception!"
        );
        st.live_local_refs.fetch_add(1, Ordering::SeqCst);
        match obj as usize {
            OBJ_CTX => CLS_CTX as *mut c_void,
            OBJ_FILE_EXT | OBJ_FILE_INT => CLS_FILE as *mut c_void,
            OBJ_AM => CLS_AM as *mut c_void,
            OBJ_STREAM => CLS_STREAM as *mut c_void,
            _ => std::ptr::null_mut(),
        }
    }

    unsafe extern "C" fn mock_get_method_id(
        env: *mut c_void,
        cls: *mut c_void,
        name: *const c_char,
        _sig: *const c_char,
    ) -> *mut c_void {
        let menv = &*(env as *const MockEnv);
        assert!(
            !(*menv.state).exception_pending.load(Ordering::SeqCst),
            "ART CheckJNI violation: GetMethodID called with pending exception!"
        );
        let s = CStr::from_ptr(name).to_str().unwrap();
        let id = match (cls as usize, s) {
            (CLS_CTX, "getExternalFilesDir") => MID_EXT_FILES,
            (CLS_CTX, "getFilesDir") => MID_FILES,
            (CLS_CTX, "getAssets") => MID_GET_ASSETS,
            (CLS_FILE, "getAbsolutePath") => MID_ABS_PATH,
            (CLS_AM, "open") => MID_OPEN,
            (CLS_STREAM, "read") => MID_READ,
            (CLS_STREAM, "close") => MID_CLOSE,
            _ => 0,
        };
        id as *mut c_void
    }

    unsafe extern "C" fn mock_call_obj_a(
        env: *mut c_void,
        obj: *mut c_void,
        mid: *mut c_void,
        _args: *const JValue,
    ) -> *mut c_void {
        let menv = &*(env as *const MockEnv);
        let st = &*menv.state;
        assert!(!st.exception_pending.load(Ordering::SeqCst));
        let out = match (obj as usize, mid as usize) {
            (OBJ_CTX, MID_EXT_FILES) => OBJ_FILE_EXT,
            (OBJ_CTX, MID_FILES) => OBJ_FILE_INT,
            (OBJ_CTX, MID_GET_ASSETS) => OBJ_AM,
            (OBJ_FILE_EXT, MID_ABS_PATH) => OBJ_STR_EXT,
            (OBJ_FILE_INT, MID_ABS_PATH) => OBJ_STR_INT,
            (OBJ_AM, MID_OPEN) => {
                if st.fail_asset_open {
                    st.exception_pending.store(true, Ordering::SeqCst);
                    return std::ptr::null_mut();
                }
                OBJ_STREAM
            }
            _ => 0,
        };
        if out != 0 {
            st.live_local_refs.fetch_add(1, Ordering::SeqCst);
        }
        out as *mut c_void
    }

    unsafe extern "C" fn mock_call_int_a(
        env: *mut c_void,
        _obj: *mut c_void,
        mid: *mut c_void,
        _args: *const JValue,
    ) -> i32 {
        let menv = &*(env as *const MockEnv);
        let st = &*menv.state;
        assert!(!st.exception_pending.load(Ordering::SeqCst));
        if mid as usize == MID_READ {
            let off = st.asset_read_offset.load(Ordering::SeqCst);
            if off >= st.asset_bytes.len() {
                return -1;
            }
            let take = (st.asset_bytes.len() - off).min(16);
            st.asset_read_offset.store(off + take, Ordering::SeqCst);
            return take as i32;
        }
        -1
    }

    unsafe extern "C" fn mock_call_void_a(
        env: *mut c_void,
        _obj: *mut c_void,
        _mid: *mut c_void,
        _args: *const JValue,
    ) {
        let menv = &*(env as *const MockEnv);
        assert!(!(*menv.state).exception_pending.load(Ordering::SeqCst));
    }

    unsafe extern "C" fn mock_new_str_utf(env: *mut c_void, _s: *const c_char) -> *mut c_void {
        let menv = &*(env as *const MockEnv);
        (*menv.state).live_local_refs.fetch_add(1, Ordering::SeqCst);
        OBJ_STR_ASSET_NAME as *mut c_void
    }

    unsafe extern "C" fn mock_get_str_utf(
        env: *mut c_void,
        jstr: *mut c_void,
        _is_copy: *mut u8,
    ) -> *const c_char {
        let menv = &*(env as *const MockEnv);
        let st = &*menv.state;
        match jstr as usize {
            OBJ_STR_EXT => st.ext_path.as_ptr(),
            OBJ_STR_INT => st.int_path.as_ptr(),
            _ => std::ptr::null(),
        }
    }

    unsafe extern "C" fn mock_rel_str_utf(_env: *mut c_void, _jstr: *mut c_void, _c: *const c_char) {
    }

    unsafe extern "C" fn mock_new_byte_array(env: *mut c_void, _len: i32) -> *mut c_void {
        let menv = &*(env as *const MockEnv);
        (*menv.state).live_local_refs.fetch_add(1, Ordering::SeqCst);
        OBJ_BYTE_ARRAY as *mut c_void
    }

    unsafe extern "C" fn mock_get_byte_array_region(
        env: *mut c_void,
        _arr: *mut c_void,
        _start: i32,
        len: i32,
        buf: *mut i8,
    ) {
        let menv = &*(env as *const MockEnv);
        let st = &*menv.state;
        let end = st.asset_read_offset.load(Ordering::SeqCst);
        let start = end - (len as usize);
        std::ptr::copy_nonoverlapping(st.asset_bytes[start..end].as_ptr(), buf as *mut u8, len as usize);
    }

    fn run_mock_jni(detached: bool, fail_asset_open: bool, asset_json: &[u8]) -> (AndroidContextInfo, MockJniState) {
        let mut vm_vtable = [std::ptr::null::<c_void>(); 16];
        vm_vtable[JNI_VM_ATTACH_CURRENT_THREAD] = mock_attach as *const c_void;
        vm_vtable[JNI_VM_DETACH_CURRENT_THREAD] = mock_detach as *const c_void;
        vm_vtable[JNI_VM_GET_ENV] = mock_get_env as *const c_void;

        let mut env_vtable = [std::ptr::null::<c_void>(); 232];
        env_vtable[JNI_ENV_EXCEPTION_CLEAR] = mock_exc_clear as *const c_void;
        env_vtable[JNI_ENV_DELETE_LOCAL_REF] = mock_del_local_ref as *const c_void;
        env_vtable[JNI_ENV_GET_OBJECT_CLASS] = mock_get_obj_cls as *const c_void;
        env_vtable[JNI_ENV_GET_METHOD_ID] = mock_get_method_id as *const c_void;
        env_vtable[JNI_ENV_CALL_OBJECT_METHOD_A] = mock_call_obj_a as *const c_void;
        env_vtable[JNI_ENV_CALL_INT_METHOD_A] = mock_call_int_a as *const c_void;
        env_vtable[JNI_ENV_CALL_VOID_METHOD_A] = mock_call_void_a as *const c_void;
        env_vtable[JNI_ENV_NEW_STRING_UTF] = mock_new_str_utf as *const c_void;
        env_vtable[JNI_ENV_GET_STRING_UTF_CHARS] = mock_get_str_utf as *const c_void;
        env_vtable[JNI_ENV_RELEASE_STRING_UTF_CHARS] = mock_rel_str_utf as *const c_void;
        env_vtable[JNI_ENV_NEW_BYTE_ARRAY] = mock_new_byte_array as *const c_void;
        env_vtable[JNI_ENV_GET_BYTE_ARRAY_REGION] = mock_get_byte_array_region as *const c_void;
        env_vtable[JNI_ENV_EXCEPTION_CHECK] = mock_exc_check as *const c_void;

        let state = MockJniState {
            detached_initially: detached,
            attach_calls: AtomicUsize::new(0),
            detach_calls: AtomicUsize::new(0),
            exception_pending: AtomicBool::new(false),
            exception_clears: AtomicUsize::new(0),
            live_local_refs: AtomicI32::new(0),
            fail_asset_open,
            asset_bytes: asset_json.to_vec(),
            asset_read_offset: AtomicUsize::new(0),
            ext_path: CString::new("/sdcard/Android/data/com.quest.app/files").unwrap(),
            int_path: CString::new("/data/user/0/com.quest.app/files").unwrap(),
        };

        let mut env = MockEnv {
            vtable: env_vtable.as_ptr(),
            state: &state,
        };
        let mut vm = MockVm {
            vtable: vm_vtable.as_ptr(),
            env_ptr: &mut env as *mut MockEnv as *mut c_void,
            state: &state,
        };

        let info = unsafe {
            extract_android_context_info_raw(
                &mut vm as *mut MockVm as *mut c_void,
                OBJ_CTX as *mut c_void,
            )
        };
        (info, state)
    }

    #[test]
    fn raw_jni_vtable_offsets_extracts_dirs_and_asset_with_zero_ref_leaks() {
        let json = br#"{"gestures":{"only_touch":true}}"#;
        let (info, state) = run_mock_jni(true, false, json);
        assert_eq!(
            info.external_files_dir,
            Some(PathBuf::from("/sdcard/Android/data/com.quest.app/files"))
        );
        assert_eq!(
            info.internal_files_dir,
            Some(PathBuf::from("/data/user/0/com.quest.app/files"))
        );
        assert_eq!(
            info.asset_config_json.as_deref(),
            Some(r#"{"gestures":{"only_touch":true}}"#)
        );
        assert_eq!(state.attach_calls.load(Ordering::SeqCst), 1);
        assert_eq!(state.detach_calls.load(Ordering::SeqCst), 1);
        assert_eq!(state.live_local_refs.load(Ordering::SeqCst), 0);
        assert!(!state.exception_pending.load(Ordering::SeqCst));
    }

    #[test]
    fn raw_jni_clears_file_not_found_exception_without_leaking_refs() {
        let (info, state) = run_mock_jni(false, true, b"");
        assert_eq!(
            info.external_files_dir,
            Some(PathBuf::from("/sdcard/Android/data/com.quest.app/files"))
        );
        assert!(info.asset_config_json.is_none());
        assert_eq!(state.attach_calls.load(Ordering::SeqCst), 0);
        assert_eq!(state.detach_calls.load(Ordering::SeqCst), 0);
        assert_eq!(state.exception_clears.load(Ordering::SeqCst), 1);
        assert_eq!(state.live_local_refs.load(Ordering::SeqCst), 0);
        assert!(!state.exception_pending.load(Ordering::SeqCst));
    }
}
