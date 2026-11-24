//! Direct Android binary XML (`AXML`) parser and idempotent Quest hand-tracking manifest patcher.
//!
//! Supports both UTF-8 (`flags & 0x0100`) and UTF-16LE (`flags & 0x0100 == 0`) string pools,
//! preserves all existing XML chunks, namespaces, resource IDs, and attributes bit-for-bit,
//! and cleanly handles resource-map prefix insertion with exact string-index remapping.

use serde::{Deserialize, Serialize};
use thiserror::Error;

const RES_STRING_POOL_TYPE: u16 = 0x0001;
const RES_XML_TYPE: u16 = 0x0003;
const RES_XML_START_NAMESPACE_TYPE: u16 = 0x0100;
const RES_XML_END_NAMESPACE_TYPE: u16 = 0x0101;
const RES_XML_START_ELEMENT_TYPE: u16 = 0x0102;
const RES_XML_END_ELEMENT_TYPE: u16 = 0x0103;
const RES_XML_CDATA_TYPE: u16 = 0x0104;
const RES_XML_RESOURCE_MAP_TYPE: u16 = 0x0180;

const STRING_POOL_SORTED_FLAG: u32 = 0x0001;
const STRING_POOL_UTF8_FLAG: u32 = 0x0100;

const ATTR_TYPE_STRING: u8 = 0x03;
const ATTR_TYPE_INT_BOOLEAN: u8 = 0x12;

const ANDROID_NS_URI: &str = "http://schemas.android.com/apk/res/android";

// Canonical Android framework attribute resource IDs (`android.R.attr.*`)
pub const ATTR_RES_NAME: u32 = 0x0101_0003;
pub const ATTR_RES_VALUE: u32 = 0x0101_0024;
pub const ATTR_RES_REQUIRED: u32 = 0x0101_028e;
pub const ATTR_RES_EXTRACT_NATIVE_LIBS: u32 = 0x0101_04ea;

pub const PERM_OCULUS_HAND_TRACKING: &str = "com.oculus.permission.HAND_TRACKING";
pub const PERM_KHRONOS_HAND_DATA: &str = "org.khronos.openxr.permission.ACCESS_DETAILED_HAND_DATA";
pub const FEAT_OCULUS_HAND_TRACKING: &str = "oculus.software.handtracking";
pub const META_HAND_FREQ: &str = "com.oculus.handtracking.frequency";
pub const META_HAND_FREQ_VALUE: &str = "HIGH";
pub const META_HAND_VERSION: &str = "com.oculus.handtracking.version";
pub const META_HAND_VERSION_VALUE: &str = "V2.0";

const NO_INDEX: u32 = 0xFFFF_FFFF;

#[derive(Debug, Error)]
pub enum AxmlError {
    #[error("invalid AXML binary: {0}")]
    Invalid(String),
}

/// Parsed summary of Quest hand-tracking & key Android manifest declarations.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct ManifestInspection {
    pub package: Option<String>,
    pub extract_native_libs: Option<bool>,
    pub uses_permissions: Vec<String>,
    pub uses_features: Vec<(String, Option<bool>)>,
    pub meta_data: Vec<(String, Option<String>)>,
}

impl ManifestInspection {
    pub fn has_quest_hand_tracking(&self) -> bool {
        self.uses_permissions
            .iter()
            .any(|p| p == PERM_OCULUS_HAND_TRACKING)
            && self
                .uses_features
                .iter()
                .any(|(f, _)| f == FEAT_OCULUS_HAND_TRACKING)
            && self
                .meta_data
                .iter()
                .any(|(k, _)| k == META_HAND_FREQ)
            && self
                .meta_data
                .iter()
                .any(|(k, _)| k == META_HAND_VERSION)
    }
}

/// Summary of what [`patch_quest_hand_tracking_axml`] changed.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct AxmlPatchReport {
    pub added_permissions: Vec<String>,
    pub added_features: Vec<String>,
    pub added_meta_data: Vec<String>,
}

struct ParsedStringPool {
    flags: u32,
    strings: Vec<String>,
    styles: Vec<Vec<StyleSpan>>,
}

#[derive(Debug, Clone, Copy)]
struct StyleSpan {
    name: u32,
    first_char: u32,
    last_char: u32,
}

#[derive(Debug, Clone)]
struct RawChunk {
    chunk_type: u16,
    header_size: u16,
    data: Vec<u8>, // Full chunk bytes including 8-byte header
}

struct ParsedAxml {
    string_pool: ParsedStringPool,
    resource_map: Vec<u32>,
    chunks: Vec<RawChunk>,
}

fn read_u16(b: &[u8], off: usize) -> Result<u16, AxmlError> {
    let slice = b
        .get(off..off + 2)
        .ok_or_else(|| AxmlError::Invalid(format!("unexpected EOF reading u16 at {off}")))?;
    Ok(u16::from_le_bytes([slice[0], slice[1]]))
}

fn read_u32(b: &[u8], off: usize) -> Result<u32, AxmlError> {
    let slice = b
        .get(off..off + 4)
        .ok_or_else(|| AxmlError::Invalid(format!("unexpected EOF reading u32 at {off}")))?;
    Ok(u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]))
}

fn write_u32(b: &mut [u8], off: usize, val: u32) {
    b[off..off + 4].copy_from_slice(&val.to_le_bytes());
}

fn parse_string_pool(chunk: &[u8], header_size: usize) -> Result<ParsedStringPool, AxmlError> {
    if header_size < 28 || chunk.len() < 28 {
        return Err(AxmlError::Invalid("truncated string pool header".into()));
    }
    let string_count = read_u32(chunk, 8)? as usize;
    let style_count = read_u32(chunk, 12)? as usize;
    let flags = read_u32(chunk, 16)?;
    let strings_start = read_u32(chunk, 20)? as usize;
    let styles_start = read_u32(chunk, 24)? as usize;

    let offsets_end = header_size
        .checked_add((string_count + style_count).checked_mul(4).ok_or_else(|| {
            AxmlError::Invalid("string/style count overflow".into())
        })?)
        .ok_or_else(|| AxmlError::Invalid("string pool offset overflow".into()))?;
    if offsets_end > chunk.len() {
        return Err(AxmlError::Invalid(
            "string pool offsets table exceeds chunk size".into(),
        ));
    }

    let mut string_offsets = Vec::with_capacity(string_count);
    for i in 0..string_count {
        string_offsets.push(read_u32(chunk, header_size + i * 4)? as usize);
    }

    let mut style_offsets = Vec::with_capacity(style_count);
    for i in 0..style_count {
        style_offsets.push(read_u32(chunk, header_size + (string_count + i) * 4)? as usize);
    }

    let is_utf8 = (flags & STRING_POOL_UTF8_FLAG) != 0;
    let mut strings = Vec::with_capacity(string_count);
    for off in string_offsets {
        let pos = strings_start
            .checked_add(off)
            .ok_or_else(|| AxmlError::Invalid("string entry offset overflow".into()))?;
        if pos >= chunk.len() {
            return Err(AxmlError::Invalid(format!(
                "string entry offset {pos} out of bounds ({})",
                chunk.len()
            )));
        }
        if is_utf8 {
            let (s, _) = decode_axml_utf8(&chunk[pos..])?;
            strings.push(s);
        } else {
            let (s, _) = decode_axml_utf16(&chunk[pos..])?;
            strings.push(s);
        }
    }

    let mut styles = Vec::with_capacity(style_count);
    if style_count > 0 && styles_start > 0 {
        for off in style_offsets {
            let mut pos = styles_start
                .checked_add(off)
                .ok_or_else(|| AxmlError::Invalid("style offset overflow".into()))?;
            let mut spans = Vec::new();
            loop {
                if pos + 4 > chunk.len() {
                    return Err(AxmlError::Invalid("truncated style span".into()));
                }
                let name = read_u32(chunk, pos)?;
                if name == NO_INDEX {
                    break;
                }
                if pos + 12 > chunk.len() {
                    return Err(AxmlError::Invalid("truncated style span fields".into()));
                }
                let first_char = read_u32(chunk, pos + 4)?;
                let last_char = read_u32(chunk, pos + 8)?;
                spans.push(StyleSpan {
                    name,
                    first_char,
                    last_char,
                });
                pos += 12;
            }
            styles.push(spans);
        }
    }

    Ok(ParsedStringPool {
        flags,
        strings,
        styles,
    })
}

fn decode_axml_utf8(data: &[u8]) -> Result<(String, usize), AxmlError> {
    let mut i = 0usize;
    let b0 = *data
        .get(i)
        .ok_or_else(|| AxmlError::Invalid("EOF reading utf8 char len".into()))?;
    i += 1;
    if (b0 & 0x80) != 0 {
        let _b1 = *data
            .get(i)
            .ok_or_else(|| AxmlError::Invalid("EOF reading utf8 char len hi".into()))?;
        i += 1;
    }

    let b0 = *data
        .get(i)
        .ok_or_else(|| AxmlError::Invalid("EOF reading utf8 byte len".into()))?;
    i += 1;
    let byte_len = if (b0 & 0x80) != 0 {
        let b1 = *data
            .get(i)
            .ok_or_else(|| AxmlError::Invalid("EOF reading utf8 byte len hi".into()))?;
        i += 1;
        (((b0 & 0x7F) as usize) << 8) | (b1 as usize)
    } else {
        b0 as usize
    };

    let str_bytes = data
        .get(i..i + byte_len)
        .ok_or_else(|| AxmlError::Invalid("EOF reading utf8 string bytes".into()))?;
    let s = String::from_utf8_lossy(str_bytes).into_owned();
    Ok((s, i + byte_len + 1))
}

fn decode_axml_utf16(data: &[u8]) -> Result<(String, usize), AxmlError> {
    let mut i = 0usize;
    let u0 = read_u16(data, i)?;
    i += 2;
    let unit_len = if (u0 & 0x8000) != 0 {
        let u1 = read_u16(data, i)?;
        i += 2;
        (((u0 & 0x7FFF) as usize) << 16) | (u1 as usize)
    } else {
        u0 as usize
    };

    let byte_len = unit_len
        .checked_mul(2)
        .ok_or_else(|| AxmlError::Invalid("utf16 length overflow".into()))?;
    if i + byte_len > data.len() {
        return Err(AxmlError::Invalid("EOF reading utf16 string units".into()));
    }
    let mut units = Vec::with_capacity(unit_len);
    for k in 0..unit_len {
        units.push(read_u16(data, i + k * 2)?);
    }
    let s = String::from_utf16_lossy(&units);
    Ok((s, i + byte_len + 2))
}

fn encode_axml_utf8(s: &str, out: &mut Vec<u8>) {
    let utf16_len = s.encode_utf16().count();
    if utf16_len >= 0x80 {
        out.push(((utf16_len >> 8) as u8) | 0x80);
        out.push((utf16_len & 0xFF) as u8);
    } else {
        out.push(utf16_len as u8);
    }

    let bytes = s.as_bytes();
    let byte_len = bytes.len();
    if byte_len >= 0x80 {
        out.push(((byte_len >> 8) as u8) | 0x80);
        out.push((byte_len & 0xFF) as u8);
    } else {
        out.push(byte_len as u8);
    }

    out.extend_from_slice(bytes);
    out.push(0);
}

fn encode_axml_utf16(s: &str, out: &mut Vec<u8>) {
    let units: Vec<u16> = s.encode_utf16().collect();
    let len = units.len();
    if len >= 0x8000 {
        let hi = ((len >> 16) as u16) | 0x8000;
        let lo = (len & 0xFFFF) as u16;
        out.extend_from_slice(&hi.to_le_bytes());
        out.extend_from_slice(&lo.to_le_bytes());
    } else {
        out.extend_from_slice(&(len as u16).to_le_bytes());
    }
    for u in units {
        out.extend_from_slice(&u.to_le_bytes());
    }
    out.extend_from_slice(&0u16.to_le_bytes());
}

fn serialize_string_pool(pool: &ParsedStringPool) -> Vec<u8> {
    let is_utf8 = (pool.flags & STRING_POOL_UTF8_FLAG) != 0;
    let flags = pool.flags & !STRING_POOL_SORTED_FLAG;
    let string_count = pool.strings.len() as u32;
    let style_count = pool.styles.len() as u32;

    let mut str_offsets = Vec::with_capacity(pool.strings.len());
    let mut str_data = Vec::new();
    for s in &pool.strings {
        str_offsets.push(str_data.len() as u32);
        if is_utf8 {
            encode_axml_utf8(s, &mut str_data);
        } else {
            encode_axml_utf16(s, &mut str_data);
        }
    }
    while str_data.len() % 4 != 0 {
        str_data.push(0);
    }

    let mut style_offsets = Vec::with_capacity(pool.styles.len());
    let mut style_data = Vec::new();
    if !pool.styles.is_empty() {
        for spans in &pool.styles {
            style_offsets.push(style_data.len() as u32);
            for sp in spans {
                style_data.extend_from_slice(&sp.name.to_le_bytes());
                style_data.extend_from_slice(&sp.first_char.to_le_bytes());
                style_data.extend_from_slice(&sp.last_char.to_le_bytes());
            }
            style_data.extend_from_slice(&NO_INDEX.to_le_bytes());
        }
        // Android string pool requires two trailing 0xFFFFFFFF terminators after styles.
        style_data.extend_from_slice(&NO_INDEX.to_le_bytes());
        style_data.extend_from_slice(&NO_INDEX.to_le_bytes());
        while style_data.len() % 4 != 0 {
            style_data.push(0);
        }
    }

    let header_size: u16 = 28;
    let strings_start: u32 = (header_size as u32) + (string_count + style_count) * 4;
    let styles_start: u32 = if pool.styles.is_empty() {
        0
    } else {
        strings_start + (str_data.len() as u32)
    };
    let total_size: u32 = strings_start + (str_data.len() as u32) + (style_data.len() as u32);

    let mut out = Vec::with_capacity(total_size as usize);
    out.extend_from_slice(&RES_STRING_POOL_TYPE.to_le_bytes());
    out.extend_from_slice(&header_size.to_le_bytes());
    out.extend_from_slice(&total_size.to_le_bytes());
    out.extend_from_slice(&string_count.to_le_bytes());
    out.extend_from_slice(&style_count.to_le_bytes());
    out.extend_from_slice(&flags.to_le_bytes());
    out.extend_from_slice(&strings_start.to_le_bytes());
    out.extend_from_slice(&styles_start.to_le_bytes());
    for o in str_offsets {
        out.extend_from_slice(&o.to_le_bytes());
    }
    for o in style_offsets {
        out.extend_from_slice(&o.to_le_bytes());
    }
    out.extend_from_slice(&str_data);
    out.extend_from_slice(&style_data);
    out
}

fn parse_axml(bytes: &[u8]) -> Result<ParsedAxml, AxmlError> {
    if bytes.len() < 8 {
        return Err(AxmlError::Invalid("AXML buffer shorter than 8 bytes".into()));
    }
    let root_type = read_u16(bytes, 0)?;
    let root_hdr = read_u16(bytes, 2)? as usize;
    let root_size = read_u32(bytes, 4)? as usize;
    if root_type != RES_XML_TYPE || root_hdr < 8 || root_size > bytes.len() || root_size < root_hdr
    {
        return Err(AxmlError::Invalid(format!(
            "invalid AXML root header: type=0x{root_type:04x} hdr={root_hdr} size={root_size} buf={}",
            bytes.len()
        )));
    }

    let mut pos = root_hdr;
    let mut string_pool: Option<ParsedStringPool> = None;
    let mut resource_map: Vec<u32> = Vec::new();
    let mut chunks: Vec<RawChunk> = Vec::new();

    while pos < root_size {
        if pos + 8 > root_size {
            return Err(AxmlError::Invalid(format!(
                "truncated chunk header at offset {pos}"
            )));
        }
        let chunk_type = read_u16(bytes, pos)?;
        let header_size = read_u16(bytes, pos + 2)?;
        let chunk_size = read_u32(bytes, pos + 4)? as usize;
        if chunk_size < 8 || (header_size as usize) > chunk_size || pos + chunk_size > root_size {
            return Err(AxmlError::Invalid(format!(
                "invalid chunk size {chunk_size} (hdr {header_size}) at offset {pos}"
            )));
        }
        let chunk_slice = &bytes[pos..pos + chunk_size];

        match chunk_type {
            RES_STRING_POOL_TYPE => {
                if string_pool.is_some() {
                    return Err(AxmlError::Invalid("duplicate string pool chunk".into()));
                }
                string_pool = Some(parse_string_pool(chunk_slice, header_size as usize)?);
            }
            RES_XML_RESOURCE_MAP_TYPE => {
                let body = &chunk_slice[header_size as usize..];
                if body.len() % 4 != 0 {
                    return Err(AxmlError::Invalid(
                        "resource map payload length not multiple of 4".into(),
                    ));
                }
                for i in (0..body.len()).step_by(4) {
                    resource_map.push(read_u32(body, i)?);
                }
            }
            _ => {
                chunks.push(RawChunk {
                    chunk_type,
                    header_size,
                    data: chunk_slice.to_vec(),
                });
            }
        }

        pos += chunk_size;
    }

    let string_pool = string_pool
        .ok_or_else(|| AxmlError::Invalid("AXML missing RES_STRING_POOL chunk".into()))?;

    Ok(ParsedAxml {
        string_pool,
        resource_map,
        chunks,
    })
}

fn serialize_axml(axml: &ParsedAxml) -> Vec<u8> {
    let sp_bytes = serialize_string_pool(&axml.string_pool);
    let mut res_map_bytes = Vec::new();
    if !axml.resource_map.is_empty() {
        let map_size = (8 + axml.resource_map.len() * 4) as u32;
        res_map_bytes.extend_from_slice(&RES_XML_RESOURCE_MAP_TYPE.to_le_bytes());
        res_map_bytes.extend_from_slice(&8u16.to_le_bytes());
        res_map_bytes.extend_from_slice(&map_size.to_le_bytes());
        for &id in &axml.resource_map {
            res_map_bytes.extend_from_slice(&id.to_le_bytes());
        }
    }

    let chunks_len: usize = axml.chunks.iter().map(|c| c.data.len()).sum();
    let total_size = (8 + sp_bytes.len() + res_map_bytes.len() + chunks_len) as u32;

    let mut out = Vec::with_capacity(total_size as usize);
    out.extend_from_slice(&RES_XML_TYPE.to_le_bytes());
    out.extend_from_slice(&8u16.to_le_bytes());
    out.extend_from_slice(&total_size.to_le_bytes());
    out.extend_from_slice(&sp_bytes);
    out.extend_from_slice(&res_map_bytes);
    for c in &axml.chunks {
        out.extend_from_slice(&c.data);
    }
    out
}

fn shift_idx(idx: u32, threshold: u32, delta: u32) -> u32 {
    if idx == NO_INDEX || idx < threshold {
        idx
    } else {
        idx + delta
    }
}

fn remap_chunk_string_indices(chunk: &mut RawChunk, threshold: u32, delta: u32) -> Result<(), AxmlError> {
    if delta == 0 {
        return Ok(());
    }
    let hs = chunk.header_size as usize;
    // All XML node chunks (0x0100..=0x0104) have `comment` string index at bytes 12..16.
    if hs >= 16 && chunk.data.len() >= 16 {
        let comment = read_u32(&chunk.data, 12)?;
        write_u32(&mut chunk.data, 12, shift_idx(comment, threshold, delta));
    }

    match chunk.chunk_type {
        RES_XML_START_NAMESPACE_TYPE | RES_XML_END_NAMESPACE_TYPE => {
            if chunk.data.len() >= hs + 8 {
                let prefix = read_u32(&chunk.data, hs)?;
                let uri = read_u32(&chunk.data, hs + 4)?;
                write_u32(&mut chunk.data, hs, shift_idx(prefix, threshold, delta));
                write_u32(&mut chunk.data, hs + 4, shift_idx(uri, threshold, delta));
            }
        }
        RES_XML_START_ELEMENT_TYPE => {
            if chunk.data.len() < hs + 20 {
                return Err(AxmlError::Invalid("truncated START_ELEMENT chunk".into()));
            }
            let ns = read_u32(&chunk.data, hs)?;
            let name = read_u32(&chunk.data, hs + 4)?;
            write_u32(&mut chunk.data, hs, shift_idx(ns, threshold, delta));
            write_u32(&mut chunk.data, hs + 4, shift_idx(name, threshold, delta));

            let attr_start = read_u16(&chunk.data, hs + 8)? as usize;
            let attr_size = read_u16(&chunk.data, hs + 10)? as usize;
            let attr_count = read_u16(&chunk.data, hs + 12)? as usize;

            for i in 0..attr_count {
                let off = hs + attr_start + i * attr_size;
                if off + 20 > chunk.data.len() {
                    return Err(AxmlError::Invalid("attribute out of bounds in START_ELEMENT".into()));
                }
                let a_ns = read_u32(&chunk.data, off)?;
                let a_name = read_u32(&chunk.data, off + 4)?;
                let a_raw = read_u32(&chunk.data, off + 8)?;
                let val_type = chunk.data[off + 15];
                write_u32(&mut chunk.data, off, shift_idx(a_ns, threshold, delta));
                write_u32(&mut chunk.data, off + 4, shift_idx(a_name, threshold, delta));
                write_u32(&mut chunk.data, off + 8, shift_idx(a_raw, threshold, delta));
                if val_type == ATTR_TYPE_STRING {
                    let data = read_u32(&chunk.data, off + 16)?;
                    write_u32(&mut chunk.data, off + 16, shift_idx(data, threshold, delta));
                }
            }
        }
        RES_XML_END_ELEMENT_TYPE => {
            if chunk.data.len() >= hs + 8 {
                let ns = read_u32(&chunk.data, hs)?;
                let name = read_u32(&chunk.data, hs + 4)?;
                write_u32(&mut chunk.data, hs, shift_idx(ns, threshold, delta));
                write_u32(&mut chunk.data, hs + 4, shift_idx(name, threshold, delta));
            }
        }
        RES_XML_CDATA_TYPE => {
            if chunk.data.len() >= hs + 12 {
                let data_idx = read_u32(&chunk.data, hs)?;
                write_u32(&mut chunk.data, hs, shift_idx(data_idx, threshold, delta));
                let val_type = chunk.data[hs + 7];
                if val_type == ATTR_TYPE_STRING {
                    let typed_data = read_u32(&chunk.data, hs + 8)?;
                    write_u32(
                        &mut chunk.data,
                        hs + 8,
                        shift_idx(typed_data, threshold, delta),
                    );
                }
            }
        }
        _ => {}
    }
    Ok(())
}

impl ParsedAxml {
    fn get_str(&self, idx: u32) -> Option<&str> {
        if idx == NO_INDEX {
            None
        } else {
            self.string_pool.strings.get(idx as usize).map(String::as_str)
        }
    }

    /// Finds or appends a plain (non-resource-mapped) string in the string pool.
    fn intern_plain_string(&mut self, value: &str) -> u32 {
        let min_idx = self.resource_map.len();
        for (i, s) in self.string_pool.strings.iter().enumerate() {
            // Prefer existing non-resource-mapped match, or exact match if safe.
            if s == value && i >= min_idx {
                return i as u32;
            }
        }
        // Also reuse if already anywhere in pool (plain values like "HIGH" or "android" are fine).
        if let Some(i) = self.string_pool.strings.iter().position(|s| s == value) {
            return i as u32;
        }
        let idx = self.string_pool.strings.len() as u32;
        self.string_pool.strings.push(value.to_owned());
        idx
    }

    /// Finds or inserts an attribute name string with a guaranteed Android framework resource ID
    /// inside the `0..resource_map.len()` prefix, remapping all shifted indices cleanly.
    fn intern_resource_attr(&mut self, name: &str, res_id: u32) -> Result<u32, AxmlError> {
        if let Some(pos) = self.resource_map.iter().position(|&id| id == res_id) {
            return Ok(pos as u32);
        }

        let insert_pos = self.resource_map.len();
        self.resource_map.push(res_id);
        self.string_pool.strings.insert(insert_pos, name.to_owned());

        // Remap style spans (if any) and all XML event chunks for indices >= insert_pos.
        let threshold = insert_pos as u32;
        for spans in &mut self.string_pool.styles {
            for sp in spans {
                sp.name = shift_idx(sp.name, threshold, 1);
            }
        }
        for chunk in &mut self.chunks {
            remap_chunk_string_indices(chunk, threshold, 1)?;
        }

        Ok(insert_pos as u32)
    }
}

struct ElementAttr {
    ns: u32,
    name: u32,
    res_id: u32,
    raw_value: u32,
    data_type: u8,
    data: u32,
}

fn build_start_element_chunk(name_idx: u32, mut attrs: Vec<ElementAttr>) -> RawChunk {
    // Sort attributes by Android resource ID ascending for Android AssetManager compatibility.
    attrs.sort_by_key(|a| a.res_id);

    let header_size: u16 = 16;
    let body_fixed: usize = 20;
    let attr_size: u16 = 20;
    let total_size = (header_size as usize + body_fixed + attrs.len() * (attr_size as usize)) as u32;

    let mut data = Vec::with_capacity(total_size as usize);
    data.extend_from_slice(&RES_XML_START_ELEMENT_TYPE.to_le_bytes());
    data.extend_from_slice(&header_size.to_le_bytes());
    data.extend_from_slice(&total_size.to_le_bytes());
    data.extend_from_slice(&0u32.to_le_bytes()); // line_number
    data.extend_from_slice(&NO_INDEX.to_le_bytes()); // comment

    data.extend_from_slice(&NO_INDEX.to_le_bytes()); // element ns
    data.extend_from_slice(&name_idx.to_le_bytes()); // element name
    data.extend_from_slice(&20u16.to_le_bytes()); // attribute_start (relative to header_size)
    data.extend_from_slice(&attr_size.to_le_bytes());
    data.extend_from_slice(&(attrs.len() as u16).to_le_bytes());
    data.extend_from_slice(&0u16.to_le_bytes()); // id_index
    data.extend_from_slice(&0u16.to_le_bytes()); // class_index
    data.extend_from_slice(&0u16.to_le_bytes()); // style_index

    for a in attrs {
        data.extend_from_slice(&a.ns.to_le_bytes());
        data.extend_from_slice(&a.name.to_le_bytes());
        data.extend_from_slice(&a.raw_value.to_le_bytes());
        data.extend_from_slice(&8u16.to_le_bytes()); // Res_value.size
        data.push(0u8); // Res_value.res0
        data.push(a.data_type);
        data.extend_from_slice(&a.data.to_le_bytes());
    }

    RawChunk {
        chunk_type: RES_XML_START_ELEMENT_TYPE,
        header_size,
        data,
    }
}

fn build_end_element_chunk(name_idx: u32) -> RawChunk {
    let header_size: u16 = 16;
    let total_size: u32 = 24;
    let mut data = Vec::with_capacity(total_size as usize);
    data.extend_from_slice(&RES_XML_END_ELEMENT_TYPE.to_le_bytes());
    data.extend_from_slice(&header_size.to_le_bytes());
    data.extend_from_slice(&total_size.to_le_bytes());
    data.extend_from_slice(&0u32.to_le_bytes()); // line_number
    data.extend_from_slice(&NO_INDEX.to_le_bytes()); // comment
    data.extend_from_slice(&NO_INDEX.to_le_bytes()); // ns
    data.extend_from_slice(&name_idx.to_le_bytes()); // name
    RawChunk {
        chunk_type: RES_XML_END_ELEMENT_TYPE,
        header_size,
        data,
    }
}

/// Inspects a binary `AndroidManifest.xml` (AXML) and extracts package, permissions, features,
/// meta-data, and `extractNativeLibs`.
pub fn inspect_axml(bytes: &[u8]) -> Result<ManifestInspection, AxmlError> {
    let axml = parse_axml(bytes)?;
    let mut out = ManifestInspection::default();

    for chunk in &axml.chunks {
        if chunk.chunk_type != RES_XML_START_ELEMENT_TYPE {
            continue;
        }
        let hs = chunk.header_size as usize;
        if chunk.data.len() < hs + 20 {
            continue;
        }
        let name_idx = read_u32(&chunk.data, hs + 4)?;
        let Some(tag) = axml.get_str(name_idx) else {
            continue;
        };

        let attr_start = read_u16(&chunk.data, hs + 8)? as usize;
        let attr_size = read_u16(&chunk.data, hs + 10)? as usize;
        let attr_count = read_u16(&chunk.data, hs + 12)? as usize;

        let mut attr_name_val: Option<String> = None;
        let mut attr_value_val: Option<String> = None;
        let mut attr_required_val: Option<bool> = None;

        for i in 0..attr_count {
            let off = hs + attr_start + i * attr_size;
            if off + 20 > chunk.data.len() {
                break;
            }
            let a_name_idx = read_u32(&chunk.data, off + 4)?;
            let a_raw_idx = read_u32(&chunk.data, off + 8)?;
            let val_type = chunk.data[off + 15];
            let val_data = read_u32(&chunk.data, off + 16)?;

            let a_name = axml.get_str(a_name_idx).unwrap_or("");
            let res_id = axml.resource_map.get(a_name_idx as usize).copied();

            let str_val = || {
                if val_type == ATTR_TYPE_STRING {
                    axml.get_str(val_data).map(str::to_owned)
                } else {
                    axml.get_str(a_raw_idx).map(str::to_owned)
                }
            };

            match tag {
                "manifest" if a_name == "package" => {
                    out.package = str_val();
                }
                "application"
                    if res_id == Some(ATTR_RES_EXTRACT_NATIVE_LIBS)
                        || a_name == "extractNativeLibs" =>
                {
                    if val_type == ATTR_TYPE_INT_BOOLEAN {
                        out.extract_native_libs = Some(val_data != 0);
                    }
                }
                "uses-permission" | "uses-feature" | "meta-data" => {
                    if res_id == Some(ATTR_RES_NAME) || a_name == "name" {
                        attr_name_val = str_val();
                    } else if res_id == Some(ATTR_RES_VALUE) || a_name == "value" {
                        attr_value_val = str_val();
                    } else if res_id == Some(ATTR_RES_REQUIRED) || a_name == "required" {
                        if val_type == ATTR_TYPE_INT_BOOLEAN {
                            attr_required_val = Some(val_data != 0);
                        }
                    }
                }
                _ => {}
            }
        }

        match tag {
            "uses-permission" => {
                if let Some(n) = attr_name_val {
                    out.uses_permissions.push(n);
                }
            }
            "uses-feature" => {
                if let Some(n) = attr_name_val {
                    out.uses_features.push((n, attr_required_val));
                }
            }
            "meta-data" => {
                if let Some(n) = attr_name_val {
                    out.meta_data.push((n, attr_value_val));
                }
            }
            _ => {}
        }
    }

    Ok(out)
}

pub fn rename_package_axml(bytes: &[u8], newpkg: &str) -> Result<Vec<u8>, AxmlError> {
    let oldpkg = inspect_axml(bytes)?
        .package
        .ok_or_else(|| AxmlError::Invalid("manifest has no package".into()))?;
    let dotted = format!("{oldpkg}.");
    let mut axml = parse_axml(bytes)?;
    let mut edits: Vec<(usize, usize, String)> = Vec::new();
    for (ci, chunk) in axml.chunks.iter().enumerate() {
        if chunk.chunk_type != RES_XML_START_ELEMENT_TYPE {
            continue;
        }
        let hs = chunk.header_size as usize;
        let tag = axml.get_str(read_u32(&chunk.data, hs + 4)?).unwrap_or("");
        let start = read_u16(&chunk.data, hs + 8)? as usize;
        let size = read_u16(&chunk.data, hs + 10)? as usize;
        let count = read_u16(&chunk.data, hs + 12)? as usize;
        for i in 0..count {
            let off = hs + start + i * size;
            if off + 20 > chunk.data.len() {
                break;
            }
            let aname = axml.get_str(read_u32(&chunk.data, off + 4)?).unwrap_or("");
            let idx = if chunk.data[off + 15] == ATTR_TYPE_STRING {
                read_u32(&chunk.data, off + 16)?
            } else {
                read_u32(&chunk.data, off + 8)?
            };
            let Some(val) = axml.get_str(idx) else {
                continue;
            };
            let hit = match aname {
                "package" => tag == "manifest" && val == oldpkg,
                "authorities" => val.contains(&oldpkg),
                "permission" | "readPermission" | "writePermission" => val.starts_with(&dotted),
                "name" => {
                    matches!(
                        tag,
                        "permission"
                            | "permission-tree"
                            | "permission-group"
                            | "uses-permission"
                            | "uses-permission-sdk-23"
                    ) && val.starts_with(&dotted)
                }
                _ => false,
            };
            if hit {
                edits.push((ci, off, val.replace(&oldpkg, newpkg)));
            }
        }
    }
    for (ci, off, val) in edits {
        let idx = axml.intern_plain_string(&val);
        let chunk = &mut axml.chunks[ci];
        write_u32(&mut chunk.data, off + 8, idx);
        if chunk.data[off + 15] == ATTR_TYPE_STRING {
            write_u32(&mut chunk.data, off + 16, idx);
        }
    }
    Ok(serialize_axml(&axml))
}

/// Idempotently patches a binary `AndroidManifest.xml` (AXML) to declare Meta Quest + Khronos AOSP
/// hand-tracking permissions, optional feature (`android:required="false"`), and Quest hand-tracking
/// `<meta-data>` entries (`frequency=HIGH`, `version=V2.0`).
///
/// Preserves all existing permissions, features, activities, intent-filters, and `extractNativeLibs`.
pub fn patch_quest_hand_tracking_axml(
    bytes: &[u8],
) -> Result<(Vec<u8>, AxmlPatchReport), AxmlError> {
    let existing = inspect_axml(bytes)?;
    let mut axml = parse_axml(bytes)?;
    let mut report = AxmlPatchReport::default();

    let need_oculus_perm = !existing
        .uses_permissions
        .iter()
        .any(|p| p == PERM_OCULUS_HAND_TRACKING);
    let need_khronos_perm = !existing
        .uses_permissions
        .iter()
        .any(|p| p == PERM_KHRONOS_HAND_DATA);
    let need_feature = !existing
        .uses_features
        .iter()
        .any(|(f, _)| f == FEAT_OCULUS_HAND_TRACKING);
    let need_freq = !existing
        .meta_data
        .iter()
        .any(|(k, _)| k == META_HAND_FREQ);
    let need_version = !existing
        .meta_data
        .iter()
        .any(|(k, _)| k == META_HAND_VERSION);

    if !need_oculus_perm && !need_khronos_perm && !need_feature && !need_freq && !need_version {
        return Ok((bytes.to_vec(), report));
    }

    // Intern required Android framework attribute resource IDs first (may shift plain indices).
    let attr_name_idx = axml.intern_resource_attr("name", ATTR_RES_NAME)?;
    let attr_value_idx = if need_freq || need_version {
        axml.intern_resource_attr("value", ATTR_RES_VALUE)?
    } else {
        NO_INDEX
    };
    let attr_required_idx = if need_feature {
        axml.intern_resource_attr("required", ATTR_RES_REQUIRED)?
    } else {
        NO_INDEX
    };

    // Locate android namespace URI string index (create if missing).
    let android_ns_idx = axml.intern_plain_string(ANDROID_NS_URI);

    // Find insertion points:
    // - manifest-level children (`uses-permission`, `uses-feature`) right before `<application>`
    // - application-level children (`meta-data`) right before `</application>`
    let mut app_start_pos: Option<usize> = None;
    let mut app_end_pos: Option<usize> = None;
    let mut depth = 0usize;
    let mut app_depth: Option<usize> = None;

    for (idx, chunk) in axml.chunks.iter().enumerate() {
        let hs = chunk.header_size as usize;
        match chunk.chunk_type {
            RES_XML_START_ELEMENT_TYPE => {
                let name_i = read_u32(&chunk.data, hs + 4)?;
                if axml.get_str(name_i) == Some("application") && app_start_pos.is_none() {
                    app_start_pos = Some(idx);
                    app_depth = Some(depth);
                }
                depth += 1;
            }
            RES_XML_END_ELEMENT_TYPE => {
                depth = depth.saturating_sub(1);
                let name_i = read_u32(&chunk.data, hs + 4)?;
                if axml.get_str(name_i) == Some("application")
                    && app_depth == Some(depth)
                    && app_end_pos.is_none()
                {
                    app_end_pos = Some(idx);
                }
            }
            _ => {}
        }
    }

    let app_start = app_start_pos
        .ok_or_else(|| AxmlError::Invalid("AndroidManifest.xml missing <application> element".into()))?;
    let app_end = app_end_pos
        .ok_or_else(|| AxmlError::Invalid("AndroidManifest.xml missing </application> end tag".into()))?;

    // Build chunks to insert inside `<application>` (insert at `app_end` FIRST so `app_start` stays valid!).
    let mut app_inserts: Vec<RawChunk> = Vec::new();
    if need_freq || need_version {
        let meta_tag = axml.intern_plain_string("meta-data");
        if need_freq {
            let k = axml.intern_plain_string(META_HAND_FREQ);
            let v = axml.intern_plain_string(META_HAND_FREQ_VALUE);
            app_inserts.push(build_start_element_chunk(
                meta_tag,
                vec![
                    ElementAttr {
                        ns: android_ns_idx,
                        name: attr_name_idx,
                        res_id: ATTR_RES_NAME,
                        raw_value: k,
                        data_type: ATTR_TYPE_STRING,
                        data: k,
                    },
                    ElementAttr {
                        ns: android_ns_idx,
                        name: attr_value_idx,
                        res_id: ATTR_RES_VALUE,
                        raw_value: v,
                        data_type: ATTR_TYPE_STRING,
                        data: v,
                    },
                ],
            ));
            app_inserts.push(build_end_element_chunk(meta_tag));
            report.added_meta_data.push(META_HAND_FREQ.to_owned());
        }
        if need_version {
            let k = axml.intern_plain_string(META_HAND_VERSION);
            let v = axml.intern_plain_string(META_HAND_VERSION_VALUE);
            app_inserts.push(build_start_element_chunk(
                meta_tag,
                vec![
                    ElementAttr {
                        ns: android_ns_idx,
                        name: attr_name_idx,
                        res_id: ATTR_RES_NAME,
                        raw_value: k,
                        data_type: ATTR_TYPE_STRING,
                        data: k,
                    },
                    ElementAttr {
                        ns: android_ns_idx,
                        name: attr_value_idx,
                        res_id: ATTR_RES_VALUE,
                        raw_value: v,
                        data_type: ATTR_TYPE_STRING,
                        data: v,
                    },
                ],
            ));
            app_inserts.push(build_end_element_chunk(meta_tag));
            report.added_meta_data.push(META_HAND_VERSION.to_owned());
        }
    }

    // Build chunks to insert under `<manifest>` right before `<application>`.
    let mut manifest_inserts: Vec<RawChunk> = Vec::new();
    if need_oculus_perm || need_khronos_perm {
        let perm_tag = axml.intern_plain_string("uses-permission");
        if need_oculus_perm {
            let val = axml.intern_plain_string(PERM_OCULUS_HAND_TRACKING);
            manifest_inserts.push(build_start_element_chunk(
                perm_tag,
                vec![ElementAttr {
                    ns: android_ns_idx,
                    name: attr_name_idx,
                    res_id: ATTR_RES_NAME,
                    raw_value: val,
                    data_type: ATTR_TYPE_STRING,
                    data: val,
                }],
            ));
            manifest_inserts.push(build_end_element_chunk(perm_tag));
            report
                .added_permissions
                .push(PERM_OCULUS_HAND_TRACKING.to_owned());
        }
        if need_khronos_perm {
            let val = axml.intern_plain_string(PERM_KHRONOS_HAND_DATA);
            manifest_inserts.push(build_start_element_chunk(
                perm_tag,
                vec![ElementAttr {
                    ns: android_ns_idx,
                    name: attr_name_idx,
                    res_id: ATTR_RES_NAME,
                    raw_value: val,
                    data_type: ATTR_TYPE_STRING,
                    data: val,
                }],
            ));
            manifest_inserts.push(build_end_element_chunk(perm_tag));
            report
                .added_permissions
                .push(PERM_KHRONOS_HAND_DATA.to_owned());
        }
    }

    if need_feature {
        let feat_tag = axml.intern_plain_string("uses-feature");
        let val = axml.intern_plain_string(FEAT_OCULUS_HAND_TRACKING);
        manifest_inserts.push(build_start_element_chunk(
            feat_tag,
            vec![
                ElementAttr {
                    ns: android_ns_idx,
                    name: attr_name_idx,
                    res_id: ATTR_RES_NAME,
                    raw_value: val,
                    data_type: ATTR_TYPE_STRING,
                    data: val,
                },
                ElementAttr {
                    ns: android_ns_idx,
                    name: attr_required_idx,
                    res_id: ATTR_RES_REQUIRED,
                    raw_value: NO_INDEX,
                    data_type: ATTR_TYPE_INT_BOOLEAN,
                    data: 0, // false
                },
            ],
        ));
        manifest_inserts.push(build_end_element_chunk(feat_tag));
        report
            .added_features
            .push(FEAT_OCULUS_HAND_TRACKING.to_owned());
    }

    // Splice higher index (`app_end`) first, then lower index (`app_start`).
    if !app_inserts.is_empty() {
        axml.chunks.splice(app_end..app_end, app_inserts);
    }
    if !manifest_inserts.is_empty() {
        axml.chunks.splice(app_start..app_start, manifest_inserts);
    }

    Ok((serialize_axml(&axml), report))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Synthesizes a minimal valid binary AXML manifest with either UTF-8 (`utf8=true`)
    /// or UTF-16LE (`utf8=false`) string pool, intentionally omitting `value` and `required`
    /// from the initial resource map so resource-map prefix insertion & index remapping are tested!
    fn synth_minimal_axml(utf8: bool) -> Vec<u8> {
        let mut pool = ParsedStringPool {
            flags: if utf8 { STRING_POOL_UTF8_FLAG } else { 0 },
            strings: vec![
                "name".to_owned(),        // 0 -> ATTR_RES_NAME
                "android".to_owned(),     // 1
                ANDROID_NS_URI.to_owned(), // 2
                "manifest".to_owned(),    // 3
                "package".to_owned(),     // 4
                "com.test.utf16".to_owned(), // 5
                "application".to_owned(), // 6
            ],
            styles: Vec::new(),
        };
        let _ = &mut pool;
        let resource_map = vec![ATTR_RES_NAME];

        let mut ns_start_data = Vec::new();
        ns_start_data.extend_from_slice(&RES_XML_START_NAMESPACE_TYPE.to_le_bytes());
        ns_start_data.extend_from_slice(&16u16.to_le_bytes());
        ns_start_data.extend_from_slice(&24u32.to_le_bytes());
        ns_start_data.extend_from_slice(&1u32.to_le_bytes());
        ns_start_data.extend_from_slice(&NO_INDEX.to_le_bytes());
        ns_start_data.extend_from_slice(&1u32.to_le_bytes()); // prefix "android"
        ns_start_data.extend_from_slice(&2u32.to_le_bytes()); // uri

        let mut ns_end_data = ns_start_data.clone();
        ns_end_data[0..2].copy_from_slice(&RES_XML_END_NAMESPACE_TYPE.to_le_bytes());

        let manifest_start = build_start_element_chunk(
            3,
            vec![ElementAttr {
                ns: NO_INDEX,
                name: 4,
                res_id: 0,
                raw_value: 5,
                data_type: ATTR_TYPE_STRING,
                data: 5,
            }],
        );
        let app_start = build_start_element_chunk(6, vec![]);
        let app_end = build_end_element_chunk(6);
        let manifest_end = build_end_element_chunk(3);

        let axml = ParsedAxml {
            string_pool: pool,
            resource_map,
            chunks: vec![
                RawChunk {
                    chunk_type: RES_XML_START_NAMESPACE_TYPE,
                    header_size: 16,
                    data: ns_start_data,
                },
                manifest_start,
                app_start,
                app_end,
                manifest_end,
                RawChunk {
                    chunk_type: RES_XML_END_NAMESPACE_TYPE,
                    header_size: 16,
                    data: ns_end_data,
                },
            ],
        };
        serialize_axml(&axml)
    }

    #[test]
    fn patch_utf8_and_utf16le_axml_preserves_package_shifts_resmap_and_is_idempotent() {
        for utf8 in [true, false] {
            let raw = synth_minimal_axml(utf8);
            let before = inspect_axml(&raw).unwrap();
            assert_eq!(before.package.as_deref(), Some("com.test.utf16"));
            assert!(!before.has_quest_hand_tracking());

            let (patched, report) = patch_quest_hand_tracking_axml(&raw).unwrap();
            assert_eq!(report.added_permissions.len(), 2);
            assert_eq!(report.added_features.len(), 1);
            assert_eq!(report.added_meta_data.len(), 2);

            let after = inspect_axml(&patched).unwrap();
            assert_eq!(after.package.as_deref(), Some("com.test.utf16"));
            assert!(after.has_quest_hand_tracking());
            assert_eq!(after.uses_features, vec![(FEAT_OCULUS_HAND_TRACKING.to_owned(), Some(false))]);

            // Second patch must be a strict no-op (byte-identical)
            let (patched2, report2) = patch_quest_hand_tracking_axml(&patched).unwrap();
            assert!(report2.added_permissions.is_empty());
            assert!(report2.added_features.is_empty());
            assert!(report2.added_meta_data.is_empty());
            assert_eq!(patched2, patched);
        }
    }

    #[test]
    fn corrupt_axml_returns_err() {
        assert!(inspect_axml(b"not axml").is_err());
        assert!(patch_quest_hand_tracking_axml(&[0x03, 0x00, 0x08, 0x00, 0xFF, 0x00, 0x00, 0x00]).is_err());
    }
}

