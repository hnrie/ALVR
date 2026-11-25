//! Pure-Rust ELF64 binary analyzer plus `.aar` and `.apk` archive inspector.

use crate::align::{verify_apk_alignment, AlignmentReport};
use crate::axml::{inspect_axml, ManifestInspection};
use crate::sign::{verify_apk_signatures, ApkSignatureVerification};
use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{Read, Seek};
use std::path::Path;

pub const EM_AARCH64: u16 = 183;
pub const EM_X86_64: u16 = 62;
pub const ET_DYN: u16 = 3;
pub const PT_LOAD: u32 = 1;
pub const PT_DYNAMIC: u32 = 2;
pub const DT_NULL: i64 = 0;
pub const DT_NEEDED: i64 = 1;
pub const DT_STRTAB: i64 = 5;
pub const DT_SYMTAB: i64 = 6;
pub const DT_STRSZ: i64 = 10;
pub const DT_SONAME: i64 = 14;
pub const SHT_DYNSYM: u32 = 11;

pub const EXPECTED_LAYER_NAME: &str = "XR_APILAYER_ALVR_hand_emulation";
pub const EXPECTED_SONAME: &str = "libXrApiLayer_alvr_hand_emulation.so";
pub const EXPECTED_LAYER_JSON_ASSET: &str =
    "assets/openxr/1/api_layers/implicit.d/XrApiLayer_alvr_hand_emulation.json";
pub const EXPECTED_APK_SO_PATH: &str = "lib/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so";
pub const EXPECTED_AAR_SO_PATH: &str = "jni/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElfInspection {
    pub is_elf64: bool,
    pub little_endian: bool,
    pub e_type: u16,
    pub e_machine: u16,
    pub machine_name: String,
    pub soname: Option<String>,
    pub needed: Vec<String>,
    pub min_pt_load_align: u64,
    pub exported_global_symbols: Vec<String>,
}

impl ElfInspection {
    pub fn is_android_arm64_shared(&self) -> bool {
        self.is_elf64 && self.little_endian && self.e_type == ET_DYN && self.e_machine == EM_AARCH64
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArchiveInspection {
    pub archive_kind: String, // "apk" or "aar"
    pub manifest: ManifestInspection,
    pub has_quest_hand_tracking: bool,
    pub has_layer_manifest: bool,
    pub layer_manifest_valid: bool,
    pub layer_manifest_name: Option<String>,
    pub layer_so_path: Option<String>,
    pub layer_so_stored: bool,
    pub layer_so_data_offset: Option<u64>,
    pub layer_so_aligned_4k: bool,
    pub layer_so_aligned_16k: bool,
    pub layer_so_elf: Option<ElfInspection>,
    pub has_openxr_loader: bool,
    pub has_legacy_vrapi: bool,
    pub has_arm64_abi: bool,
    pub has_any_native_libs: bool,
    pub alignment: Option<AlignmentReport>,
    pub signature_verified: Option<bool>,
    pub signature_detail: Option<String>,
    pub signature_verification: Option<ApkSignatureVerification>,
}

fn r_u16(b: &[u8], o: usize) -> Result<u16> {
    let s = b.get(o..o + 2).context("ELF EOF u16")?;
    Ok(u16::from_le_bytes([s[0], s[1]]))
}
fn r_u32(b: &[u8], o: usize) -> Result<u32> {
    let s = b.get(o..o + 4).context("ELF EOF u32")?;
    Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}
fn r_u64(b: &[u8], o: usize) -> Result<u64> {
    let s = b.get(o..o + 8).context("ELF EOF u64")?;
    Ok(u64::from_le_bytes([
        s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7],
    ]))
}
fn r_i64(b: &[u8], o: usize) -> Result<i64> {
    Ok(r_u64(b, o)? as i64)
}

fn read_cstr_at(b: &[u8], off: usize) -> Option<String> {
    if off >= b.len() {
        return None;
    }
    let tail = &b[off..];
    let end = tail.iter().position(|&c| c == 0)?;
    std::str::from_utf8(&tail[..end]).ok().map(str::to_owned)
}

/// Pure-Rust ELF64 inspection (architecture, SONAME, `DT_NEEDED`, `PT_LOAD` alignment, exported symbols).
pub fn inspect_elf(bytes: &[u8]) -> Result<ElfInspection> {
    if bytes.len() < 64 || &bytes[0..4] != b"\x7fELF" {
        bail!("not an ELF binary");
    }
    let class = bytes[4];
    let data_enc = bytes[5];
    let is_elf64 = class == 2;
    let little_endian = data_enc == 1;
    if !is_elf64 || !little_endian {
        bail!("unsupported ELF class={class} endian={data_enc} (expected 64-bit little-endian)");
    }

    let e_type = r_u16(bytes, 16)?;
    let e_machine = r_u16(bytes, 18)?;
    let e_phoff = r_u64(bytes, 32)? as usize;
    let e_shoff = r_u64(bytes, 40)? as usize;
    let e_phentsize = r_u16(bytes, 54)? as usize;
    let e_phnum = r_u16(bytes, 56)? as usize;
    let e_shentsize = r_u16(bytes, 58)? as usize;
    let e_shnum = r_u16(bytes, 60)? as usize;

    let machine_name = match e_machine {
        EM_AARCH64 => "AArch64".to_owned(),
        EM_X86_64 => "x86-64".to_owned(),
        40 => "ARM".to_owned(),
        other => format!("machine({other})"),
    };

    // Parse program headers for PT_LOAD alignment and PT_DYNAMIC.
    let mut load_segments: Vec<(u64, u64, u64)> = Vec::new(); // (vaddr, offset, filesz)
    let mut min_pt_load_align: Option<u64> = None;
    let mut dyn_off: Option<usize> = None;
    let mut dyn_size: usize = 0;

    for i in 0..e_phnum {
        let off = e_phoff + i * e_phentsize;
        if off + 56 > bytes.len() {
            break;
        }
        let p_type = r_u32(bytes, off)?;
        let p_offset = r_u64(bytes, off + 8)?;
        let p_vaddr = r_u64(bytes, off + 16)?;
        let p_filesz = r_u64(bytes, off + 32)?;
        let p_align = r_u64(bytes, off + 48)?;

        if p_type == PT_LOAD {
            load_segments.push((p_vaddr, p_offset, p_filesz));
            min_pt_load_align = Some(match min_pt_load_align {
                Some(cur) => cur.min(p_align),
                None => p_align,
            });
        } else if p_type == PT_DYNAMIC {
            dyn_off = Some(p_offset as usize);
            dyn_size = p_filesz as usize;
        }
    }

    let vaddr_to_offset = |vaddr: u64| -> Option<usize> {
        for &(seg_vaddr, seg_off, seg_filesz) in &load_segments {
            if vaddr >= seg_vaddr && vaddr < seg_vaddr.saturating_add(seg_filesz) {
                return Some((seg_off + (vaddr - seg_vaddr)) as usize);
            }
        }
        None
    };

    // Parse PT_DYNAMIC for DT_NEEDED, DT_SONAME, DT_STRTAB, DT_STRSZ.
    let mut needed_offs = Vec::new();
    let mut soname_off = None;
    let mut strtab_vaddr = None;
    let mut strsz = 0usize;
    let mut _symtab_vaddr = None;

    if let Some(d_off) = dyn_off {
        let mut pos = d_off;
        let end = (d_off + dyn_size).min(bytes.len());
        while pos + 16 <= end {
            let d_tag = r_i64(bytes, pos)?;
            let d_val = r_u64(bytes, pos + 8)?;
            pos += 16;
            if d_tag == DT_NULL {
                break;
            }
            match d_tag {
                DT_NEEDED => needed_offs.push(d_val as usize),
                DT_SONAME => soname_off = Some(d_val as usize),
                DT_STRTAB => strtab_vaddr = Some(d_val),
                DT_STRSZ => strsz = d_val as usize,
                DT_SYMTAB => _symtab_vaddr = Some(d_val),
                _ => {}
            }
        }
    }

    let dynstr_off = strtab_vaddr.and_then(vaddr_to_offset);
    let dynstr_slice = dynstr_off.and_then(|o| {
        if strsz > 0 && o + strsz <= bytes.len() {
            Some(&bytes[o..o + strsz])
        } else if o < bytes.len() {
            Some(&bytes[o..])
        } else {
            None
        }
    });

    let soname = soname_off.and_then(|o| dynstr_slice.and_then(|ds| read_cstr_at(ds, o)));
    let needed: Vec<String> = needed_offs
        .into_iter()
        .filter_map(|o| dynstr_slice.and_then(|ds| read_cstr_at(ds, o)))
        .collect();

    // Parse SHT_DYNSYM section for defined STB_GLOBAL / STV_DEFAULT symbols.
    let mut exported_global_symbols = Vec::new();
    if e_shoff > 0 && e_shentsize >= 64 {
        for i in 0..e_shnum {
            let sh = e_shoff + i * e_shentsize;
            if sh + 64 > bytes.len() {
                break;
            }
            let sh_type = r_u32(bytes, sh + 4)?;
            if sh_type != SHT_DYNSYM {
                continue;
            }
            let sh_offset = r_u64(bytes, sh + 24)? as usize;
            let sh_size = r_u64(bytes, sh + 32)? as usize;
            let sh_link = r_u32(bytes, sh + 40)? as usize;
            let sh_entsize = r_u64(bytes, sh + 56)? as usize;
            if sh_entsize < 24 {
                continue;
            }
            let str_slice = if sh_link < e_shnum {
                let lsh = e_shoff + sh_link * e_shentsize;
                if lsh + 64 <= bytes.len() {
                    let loff = r_u64(bytes, lsh + 24)? as usize;
                    let lsize = r_u64(bytes, lsh + 32)? as usize;
                    bytes.get(loff..loff + lsize)
                } else {
                    dynstr_slice
                }
            } else {
                dynstr_slice
            };

            let mut pos = sh_offset;
            let end = (sh_offset + sh_size).min(bytes.len());
            while pos + sh_entsize <= end {
                let st_name = r_u32(bytes, pos)? as usize;
                let st_info = bytes[pos + 4];
                let st_other = bytes[pos + 5];
                let st_shndx = r_u16(bytes, pos + 6)?;
                let bind = st_info >> 4;
                let vis = st_other & 0x3;
                // STB_GLOBAL (1), STV_DEFAULT (0), defined section (st_shndx != 0)
                if bind == 1 && vis == 0 && st_shndx != 0 {
                    if let Some(name) = str_slice.and_then(|ss| read_cstr_at(ss, st_name)) {
                        if !name.is_empty() {
                            exported_global_symbols.push(name);
                        }
                    }
                }
                pos += sh_entsize;
            }
        }
    }
    exported_global_symbols.sort();

    Ok(ElfInspection {
        is_elf64,
        little_endian,
        e_type,
        e_machine,
        machine_name,
        soname,
        needed,
        min_pt_load_align: min_pt_load_align.unwrap_or(0),
        exported_global_symbols,
    })
}

/// Validates an OpenXR implicit API layer JSON manifest according to Khronos spec (`manifest_file.cpp`).
pub fn validate_layer_manifest_json(bytes: &[u8]) -> Result<String> {
    let val: serde_json::Value =
        serde_json::from_slice(bytes).context("layer manifest is not valid JSON")?;
    let ffv = val
        .get("file_format_version")
        .and_then(|v| v.as_str())
        .context("missing file_format_version")?;
    if ffv != "1.0.0" {
        bail!("unsupported file_format_version '{ffv}' (expected '1.0.0')");
    }
    let layer = val
        .get("api_layer")
        .and_then(|v| v.as_object())
        .context("missing api_layer object")?;
    let name = layer
        .get("name")
        .and_then(|v| v.as_str())
        .filter(|s| !s.is_empty())
        .context("missing api_layer.name")?;
    let _lib = layer
        .get("library_path")
        .and_then(|v| v.as_str())
        .filter(|s| !s.is_empty())
        .context("missing api_layer.library_path")?;
    let _api_ver = layer
        .get("api_version")
        .and_then(|v| v.as_str())
        .filter(|s| s.contains('.'))
        .context("missing api_layer.api_version")?;
    let _impl_ver = layer
        .get("implementation_version")
        .and_then(|v| v.as_str())
        .filter(|s| s.parse::<u32>().is_ok())
        .context("missing or non-integer api_layer.implementation_version")?;
    let _desc = layer
        .get("description")
        .and_then(|v| v.as_str())
        .context("missing api_layer.description")?;
    let _disable = layer
        .get("disable_environment")
        .and_then(|v| v.as_str())
        .filter(|s| !s.is_empty())
        .context("implicit OpenXR layer manifest MUST define non-empty disable_environment")?;
    Ok(name.to_owned())
}

/// Parses plain-text `AndroidManifest.xml` (for `.aar` archives) using `quick-xml`.
pub fn inspect_plain_xml_manifest(bytes: &[u8]) -> Result<ManifestInspection> {
    use quick_xml::events::Event;
    use quick_xml::Reader;

    let mut reader = Reader::from_reader(bytes);
    let mut out = ManifestInspection::default();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(e)) | Ok(Event::Empty(e)) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let mut name_attr = None;
                let mut value_attr = None;
                let mut req_attr = None;

                for attr in e.attributes().flatten() {
                    let key = String::from_utf8_lossy(attr.key.as_ref()).into_owned();
                    let val = attr
                        .unescape_value()
                        .map(|v| v.into_owned())
                        .unwrap_or_default();
                    match (tag.as_str(), key.as_str()) {
                        ("manifest", "package") => out.package = Some(val),
                        ("application", "android:extractNativeLibs") => {
                            out.extract_native_libs = val.parse::<bool>().ok();
                        }
                        (_, "android:name") => name_attr = Some(val),
                        (_, "android:value") => value_attr = Some(val),
                        (_, "android:required") => req_attr = val.parse::<bool>().ok(),
                        _ => {}
                    }
                }
                match tag.as_str() {
                    "uses-permission" => {
                        if let Some(n) = name_attr {
                            out.uses_permissions.push(n);
                        }
                    }
                    "uses-feature" => {
                        if let Some(n) = name_attr {
                            out.uses_features.push((n, req_attr));
                        }
                    }
                    "meta-data" => {
                        if let Some(n) = name_attr {
                            out.meta_data.push((n, value_attr));
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::Eof) => break,
            Err(e) => bail!("invalid plain XML manifest: {e}"),
            _ => {}
        }
        buf.clear();
    }

    Ok(out)
}

/// Inspects an `.apk` or `.aar` archive completely.
pub fn inspect_archive(path: &Path) -> Result<ArchiveInspection> {
    let file = File::open(path).with_context(|| format!("failed to open {}", path.display()))?;
    let mut zip = zip::ZipArchive::new(file)
        .with_context(|| format!("{} is not a valid ZIP/APK/AAR archive", path.display()))?;

    let is_aar = path
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("aar"))
        || zip.by_name("classes.jar").is_ok();

    // Read AndroidManifest.xml
    let manifest_bytes = read_zip_entry(&mut zip, "AndroidManifest.xml")
        .context("archive missing AndroidManifest.xml")?;
    let manifest = if manifest_bytes.starts_with(b"\x03\x00") {
        inspect_axml(&manifest_bytes).context("failed to parse binary AndroidManifest.xml")?
    } else {
        inspect_plain_xml_manifest(&manifest_bytes)
            .context("failed to parse plain-text AndroidManifest.xml")?
    };

    // Inspect layer JSON manifest
    let (has_layer_manifest, layer_manifest_valid, layer_manifest_name) =
        match read_zip_entry(&mut zip, EXPECTED_LAYER_JSON_ASSET) {
            Ok(bytes) => match validate_layer_manifest_json(&bytes) {
                Ok(name) => (true, true, Some(name)),
                Err(_) => (true, false, None),
            },
            Err(_) => (false, false, None),
        };

    // Scan entries for OpenXR loader, VrApi, ABIs, and our layer .so
    let mut has_openxr_loader = false;
    let mut has_legacy_vrapi = false;
    let mut has_arm64_abi = false;
    let mut has_any_native_libs = false;
    let mut found_so_path = None;

    for i in 0..zip.len() {
        let entry = zip.by_index(i)?;
        let name = entry.name();
        let lower = name.to_ascii_lowercase();
        if (name.starts_with("lib/") || name.starts_with("jni/")) && lower.ends_with(".so") {
            has_any_native_libs = true;
            if name.starts_with("lib/arm64-v8a/") || name.starts_with("jni/arm64-v8a/") {
                has_arm64_abi = true;
            }
            if lower.contains("openxr") && !lower.contains("xrapilayer") {
                has_openxr_loader = true;
            }
            if lower.ends_with("libvrapi.so") {
                has_legacy_vrapi = true;
            }
        }
        if name == EXPECTED_APK_SO_PATH || name == EXPECTED_AAR_SO_PATH {
            found_so_path = Some(name.to_owned());
        }
    }

    let mut layer_so_stored = false;
    let mut layer_so_data_offset = None;
    let mut layer_so_aligned_4k = false;
    let mut layer_so_aligned_16k = false;
    let mut layer_so_elf = None;

    if let Some(ref so_name) = found_so_path {
        let mut entry = zip.by_name(so_name)?;
        layer_so_stored = entry.compression() == zip::CompressionMethod::Stored;
        let data_off = entry.data_start();
        layer_so_data_offset = data_off;
        layer_so_aligned_4k = layer_so_stored && data_off.is_some_and(|o| o % 4096 == 0);
        layer_so_aligned_16k = layer_so_stored && data_off.is_some_and(|o| o % 16384 == 0);
        let mut so_bytes = Vec::new();
        entry.read_to_end(&mut so_bytes)?;
        layer_so_elf = inspect_elf(&so_bytes).ok();
    }

    drop(zip);

    // For APKs, run pure-Rust 16 KB alignment check and v1/v2/v3 cryptographic signature verification.
    let (alignment, signature_verified, signature_detail, signature_verification) = if !is_aar {
        let align_rep = verify_apk_alignment(path).ok();
        match verify_apk_signatures(path) {
            Ok(ver) => {
                let detail = format!(
                    "v1={}, v2={}, v3={}, cert_sha256={}{}",
                    ver.v1_verified,
                    ver.v2_verified,
                    ver.v3_verified,
                    ver.cert_sha256.as_deref().unwrap_or("none"),
                    if ver.errors.is_empty() {
                        String::new()
                    } else {
                        format!("; errors: {}", ver.errors.join("; "))
                    }
                );
                (align_rep, Some(ver.valid), Some(detail), Some(ver))
            }
            Err(err) => (align_rep, Some(false), Some(format!("{err:#}")), None),
        }
    } else {
        (None, None, None, None)
    };

    let has_quest_hand_tracking = manifest.has_quest_hand_tracking();

    Ok(ArchiveInspection {
        archive_kind: if is_aar { "aar" } else { "apk" }.to_owned(),
        manifest,
        has_quest_hand_tracking,
        has_layer_manifest,
        layer_manifest_valid,
        layer_manifest_name,
        layer_so_path: found_so_path,
        layer_so_stored,
        layer_so_data_offset,
        layer_so_aligned_4k,
        layer_so_aligned_16k,
        layer_so_elf,
        has_openxr_loader,
        has_legacy_vrapi,
        has_arm64_abi,
        has_any_native_libs,
        alignment,
        signature_verified,
        signature_detail,
        signature_verification,
    })
}

fn read_zip_entry<R: Read + Seek>(zip: &mut zip::ZipArchive<R>, name: &str) -> Result<Vec<u8>> {
    let mut entry = zip.by_name(name)?;
    let mut buf = Vec::with_capacity(entry.size() as usize);
    entry.read_to_end(&mut buf)?;
    Ok(buf)
}
