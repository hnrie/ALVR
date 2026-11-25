//! Pure-Rust streaming 16 KB APK ZIP aligner, layout parser, and alignment verifier.
//!
//! Replaces external `zipalign` completely:
//! - Aligns every uncompressed (`STORED`) `.so` file's payload offset to 16,384 bytes (`0x4000`),
//!   satisfying both Android 15+ / Quest 16 KB page alignment (`zipalign -c -P 16 4`) and
//!   legacy 4 KB alignment (`zipalign -c -p 4`).
//! - Aligns `resources.arsc` (forcing `STORED` if compressed) and all other `STORED` entries
//!   to 4-byte boundaries (Android 11+ / API 30+ requirement).
//! - Streams untouched `DEFLATED` and `STORED` entries in 64 KB chunks without re-compressing,
//!   computing uncompressed SHA-256 digests on-the-fly for APK Signature Scheme v1 in O(64 KB) RAM.
//! - Emits zero data-descriptor flags (`flags & 0x0008 == 0`) with exact sizes and CRC32 in every LFH.

use anyhow::{bail, ensure, Context, Result};
use crc32fast::Hasher as Crc32Hasher;
use flate2::write::{DeflateDecoder, DeflateEncoder};
use flate2::Compression;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::Path;

pub const LFH_SIGNATURE: u32 = 0x0403_4b50;
pub const CD_SIGNATURE: u32 = 0x0201_4b50;
pub const EOCD_SIGNATURE: u32 = 0x0605_4b50;
pub const APK_SIG_BLOCK_MAGIC: &[u8; 16] = b"APK Sig Block 42";

pub const METHOD_STORED: u16 = 0;
pub const METHOD_DEFLATED: u16 = 8;

pub const FLAG_DATA_DESCRIPTOR: u16 = 0x0008;
pub const FLAG_UTF8: u16 = 0x0800;

pub const ALIGN_4B: u64 = 4;
pub const ALIGN_4K: u64 = 4096;
pub const ALIGN_16K: u64 = 16384;

/// Fixed DOS timestamp (`1980-01-01 00:00:00`) for newly generated APK entries.
const DEFAULT_DOS_TIME: u16 = 0x0000;
const DEFAULT_DOS_DATE: u16 = 0x0021;

const STREAM_BUF_SIZE: usize = 64 * 1024;

/// Parsed ZIP End of Central Directory record.
#[derive(Debug, Clone)]
pub struct ZipEocd {
    pub eocd_offset: u64,
    pub entry_count: u16,
    pub cd_size: u64,
    pub cd_offset: u64,
    pub raw_bytes: Vec<u8>,
}

/// Byte span of an APK Signing Block immediately preceding the Central Directory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SigningBlockSpan {
    pub offset: u64,
    pub total_size: u64,
}

/// Parsed entry from the ZIP Central Directory + Local File Header.
#[derive(Debug, Clone)]
pub struct CentralDirEntry {
    pub name: String,
    pub method: u16,
    pub flags: u16,
    pub lfh_flags: u16,
    pub mod_time: u16,
    pub mod_date: u16,
    pub crc32: u32,
    pub comp_size: u64,
    pub uncomp_size: u64,
    pub lfh_offset: u64,
    pub data_offset: u64,
    pub external_attrs: u32,
}

impl CentralDirEntry {
    pub fn is_dir(&self) -> bool {
        self.name.ends_with('/')
    }
}

/// Complete low-level ZIP layout of an APK file.
#[derive(Debug, Clone)]
pub struct ApkZipLayout {
    pub file_len: u64,
    pub eocd: ZipEocd,
    pub signing_block: Option<SigningBlockSpan>,
    pub entries: Vec<CentralDirEntry>,
}

impl ApkZipLayout {
    /// End offset of Section 1 (ZIP entries) — either `signing_block.offset` or `eocd.cd_offset`.
    pub fn zip_entries_end(&self) -> u64 {
        self.signing_block
            .map(|b| b.offset)
            .unwrap_or(self.eocd.cd_offset)
    }

    pub fn find_entry(&self, name: &str) -> Option<&CentralDirEntry> {
        self.entries.iter().find(|e| e.name == name)
    }
}

/// Result of pure-Rust APK alignment verification (`zipalign -c -P 16 4` equivalent).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct AlignmentReport {
    pub aligned_4b: bool,
    pub aligned_4k_so: bool,
    pub aligned_16k_so: bool,
    pub resources_arsc_ok: bool,
    pub has_data_descriptors: bool,
    pub checked_stored_entries: usize,
    pub checked_so_entries: usize,
    pub misaligned_entries: Vec<String>,
}

impl AlignmentReport {
    pub fn is_fully_aligned_16k(&self) -> bool {
        self.aligned_4b
            && self.aligned_4k_so
            && self.aligned_16k_so
            && self.resources_arsc_ok
            && self.misaligned_entries.is_empty()
    }
}

/// Metadata for an entry written by [`StreamingApkZipWriter`].
#[derive(Debug, Clone)]
pub struct WrittenZipEntry {
    pub name: String,
    pub method: u16,
    pub flags: u16,
    pub mod_time: u16,
    pub mod_date: u16,
    pub crc32: u32,
    pub comp_size: u32,
    pub uncomp_size: u32,
    pub lfh_offset: u32,
    pub data_offset: u64,
    pub external_attrs: u32,
    pub sha256: [u8; 32],
}

fn read_u16_le(b: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([b[off], b[off + 1]])
}

fn read_u32_le(b: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
}

fn read_u64_le(b: &[u8], off: usize) -> u64 {
    u64::from_le_bytes([
        b[off],
        b[off + 1],
        b[off + 2],
        b[off + 3],
        b[off + 4],
        b[off + 5],
        b[off + 6],
        b[off + 7],
    ])
}

/// Locates and parses the ZIP End of Central Directory record.
pub fn find_eocd<R: Read + Seek>(reader: &mut R) -> Result<(u64, ZipEocd)> {
    let file_len = reader.seek(SeekFrom::End(0))?;
    if file_len < 22 {
        bail!("file too short ({file_len} bytes) to be a valid ZIP/APK");
    }
    let max_back = (65535u64 + 22).min(file_len);
    let search_start = file_len - max_back;
    reader.seek(SeekFrom::Start(search_start))?;
    let mut tail = vec![0u8; max_back as usize];
    reader.read_exact(&mut tail)?;

    // Scan backwards so embedded ZIP archives or comments cannot fool EOCD lookup.
    let mut i = tail.len().saturating_sub(22);
    loop {
        if read_u32_le(&tail, i) == EOCD_SIGNATURE {
            let comment_len = read_u16_le(&tail, i + 20) as usize;
            if i + 22 + comment_len == tail.len() {
                let disk_num = read_u16_le(&tail, i + 4);
                let cd_disk = read_u16_le(&tail, i + 6);
                ensure!(
                    disk_num == 0 && cd_disk == 0,
                    "multi-disk ZIP archives are not supported for APKs"
                );
                let entry_count = read_u16_le(&tail, i + 10);
                let cd_size = read_u32_le(&tail, i + 12) as u64;
                let cd_offset = read_u32_le(&tail, i + 16) as u64;
                ensure!(
                    entry_count != 0xFFFF && cd_size != 0xFFFF_FFFF && cd_offset != 0xFFFF_FFFF,
                    "ZIP64 APK archives (>4 GiB or >65534 entries) are not supported"
                );
                let eocd_offset = search_start + i as u64;
                ensure!(
                    cd_offset.checked_add(cd_size).is_some_and(|end| end <= eocd_offset),
                    "Central Directory bounds ({cd_offset} + {cd_size}) exceed EOCD offset ({eocd_offset})"
                );
                return Ok((
                    file_len,
                    ZipEocd {
                        eocd_offset,
                        entry_count,
                        cd_size,
                        cd_offset,
                        raw_bytes: tail[i..].to_vec(),
                    },
                ));
            }
        }
        if i == 0 {
            break;
        }
        i -= 1;
    }

    bail!("ZIP End of Central Directory (EOCD) record not found")
}

/// Parses the full ZIP Central Directory, LFH data offsets, and optional APK Signing Block.
pub fn parse_apk_zip_layout<R: Read + Seek>(reader: &mut R) -> Result<ApkZipLayout> {
    let (file_len, eocd) = find_eocd(reader)?;

    // Detect optional APK Signing Block immediately preceding the Central Directory.
    let mut signing_block = None;
    if eocd.cd_offset >= 32 {
        let footer_pos = eocd.cd_offset - 24;
        reader.seek(SeekFrom::Start(footer_pos))?;
        let mut footer = [0u8; 24];
        reader.read_exact(&mut footer)?;
        if &footer[8..24] == APK_SIG_BLOCK_MAGIC {
            let size_end = read_u64_le(&footer, 0);
            if size_end >= 24 && size_end.checked_add(8).is_some_and(|tot| tot <= eocd.cd_offset) {
                let total_size = size_end + 8;
                let block_offset = eocd.cd_offset - total_size;
                reader.seek(SeekFrom::Start(block_offset))?;
                let mut start_size_buf = [0u8; 8];
                reader.read_exact(&mut start_size_buf)?;
                let size_start = read_u64_le(&start_size_buf, 0);
                if size_start == size_end {
                    signing_block = Some(SigningBlockSpan {
                        offset: block_offset,
                        total_size,
                    });
                }
            }
        }
    }

    // Read Central Directory bytes.
    reader.seek(SeekFrom::Start(eocd.cd_offset))?;
    let mut cd_buf = vec![0u8; eocd.cd_size as usize];
    reader.read_exact(&mut cd_buf)?;

    let entries_limit = signing_block
        .map(|b| b.offset)
        .unwrap_or(eocd.cd_offset);

    let mut entries = Vec::with_capacity(eocd.entry_count as usize);
    let mut pos = 0usize;
    while pos < cd_buf.len() {
        ensure!(
            pos + 46 <= cd_buf.len(),
            "truncated Central Directory entry at offset {pos}"
        );
        ensure!(
            read_u32_le(&cd_buf, pos) == CD_SIGNATURE,
            "invalid Central Directory entry signature at offset {pos}"
        );
        let flags = read_u16_le(&cd_buf, pos + 8);
        let method = read_u16_le(&cd_buf, pos + 10);
        let mod_time = read_u16_le(&cd_buf, pos + 12);
        let mod_date = read_u16_le(&cd_buf, pos + 14);
        let crc32 = read_u32_le(&cd_buf, pos + 16);
        let comp_size = read_u32_le(&cd_buf, pos + 20) as u64;
        let uncomp_size = read_u32_le(&cd_buf, pos + 24) as u64;
        let name_len = read_u16_le(&cd_buf, pos + 28) as usize;
        let extra_len = read_u16_le(&cd_buf, pos + 30) as usize;
        let comment_len = read_u16_le(&cd_buf, pos + 32) as usize;
        let external_attrs = read_u32_le(&cd_buf, pos + 38);
        let lfh_offset = read_u32_le(&cd_buf, pos + 42) as u64;

        let rec_end = pos + 46 + name_len + extra_len + comment_len;
        ensure!(
            rec_end <= cd_buf.len(),
            "Central Directory entry extends past CD end"
        );
        let name = String::from_utf8_lossy(&cd_buf[pos + 46..pos + 46 + name_len]).into_owned();

        // Read LFH fixed header (30 bytes) to determine exact payload data_offset.
        ensure!(
            lfh_offset + 30 <= entries_limit,
            "LFH offset {lfh_offset} for '{name}' exceeds ZIP entries section ({entries_limit})"
        );
        reader.seek(SeekFrom::Start(lfh_offset))?;
        let mut lfh = [0u8; 30];
        reader.read_exact(&mut lfh)?;
        ensure!(
            read_u32_le(&lfh, 0) == LFH_SIGNATURE,
            "invalid LFH signature at {lfh_offset} for '{name}'"
        );
        let lfh_flags = read_u16_le(&lfh, 6);
        let lfh_name_len = read_u16_le(&lfh, 26) as u64;
        let lfh_extra_len = read_u16_le(&lfh, 28) as u64;
        let data_offset = lfh_offset + 30 + lfh_name_len + lfh_extra_len;
        ensure!(
            data_offset
                .checked_add(comp_size)
                .is_some_and(|end| end <= entries_limit),
            "entry '{name}' payload ({data_offset} + {comp_size}) exceeds ZIP entries section ({entries_limit})"
        );

        entries.push(CentralDirEntry {
            name,
            method,
            flags,
            lfh_flags,
            mod_time,
            mod_date,
            crc32,
            comp_size,
            uncomp_size,
            lfh_offset,
            data_offset,
            external_attrs,
        });

        pos = rec_end;
    }

    ensure!(
        entries.len() == eocd.entry_count as usize,
        "Central Directory entry count mismatch (parsed {}, EOCD claims {})",
        entries.len(),
        eocd.entry_count
    );

    Ok(ApkZipLayout {
        file_len,
        eocd,
        signing_block,
        entries,
    })
}

/// Reads and inflates a single entry from `reader` given its parsed [`CentralDirEntry`].
pub fn read_entry_uncompressed<R: Read + Seek>(
    reader: &mut R,
    entry: &CentralDirEntry,
) -> Result<Vec<u8>> {
    reader.seek(SeekFrom::Start(entry.data_offset))?;
    let mut limited = reader.take(entry.comp_size);
    match entry.method {
        METHOD_STORED => {
            let mut out = Vec::with_capacity(entry.uncomp_size as usize);
            limited.read_to_end(&mut out)?;
            ensure!(
                out.len() as u64 == entry.uncomp_size,
                "truncated STORED entry '{}'",
                entry.name
            );
            Ok(out)
        }
        METHOD_DEFLATED => {
            let mut dec = flate2::read::DeflateDecoder::new(limited);
            let mut out = Vec::with_capacity(entry.uncomp_size as usize);
            dec.read_to_end(&mut out)?;
            ensure!(
                out.len() as u64 == entry.uncomp_size,
                "uncompressed size mismatch for DEFLATED entry '{}'",
                entry.name
            );
            Ok(out)
        }
        other => bail!(
            "unsupported ZIP compression method {other} for entry '{}'",
            entry.name
        ),
    }
}

/// Streams a single entry's uncompressed bytes into a `Digest` without allocating the full entry.
pub fn digest_entry_uncompressed<R: Read + Seek, D: Digest>(
    reader: &mut R,
    entry: &CentralDirEntry,
) -> Result<Vec<u8>> {
    reader.seek(SeekFrom::Start(entry.data_offset))?;
    let mut limited = reader.take(entry.comp_size);
    let mut hasher = D::new();
    let mut crc = Crc32Hasher::new();
    let mut buf = [0u8; STREAM_BUF_SIZE];
    let mut total = 0u64;

    match entry.method {
        METHOD_STORED => loop {
            let n = limited.read(&mut buf)?;
            if n == 0 {
                break;
            }
            hasher.update(&buf[..n]);
            crc.update(&buf[..n]);
            total += n as u64;
        },
        METHOD_DEFLATED => {
            let mut dec = flate2::read::DeflateDecoder::new(limited);
            loop {
                let n = dec.read(&mut buf)?;
                if n == 0 {
                    break;
                }
                hasher.update(&buf[..n]);
                crc.update(&buf[..n]);
                total += n as u64;
            }
        }
        other => bail!(
            "unsupported ZIP compression method {other} for entry '{}'",
            entry.name
        ),
    }

    ensure!(
        total == entry.uncomp_size,
        "entry '{}' uncompressed length mismatch (got {total}, expected {})",
        entry.name,
        entry.uncomp_size
    );
    ensure!(
        crc.finalize() == entry.crc32,
        "entry '{}' CRC32 mismatch",
        entry.name
    );

    Ok(hasher.finalize().to_vec())
}

/// Returns the required byte alignment for an uncompressed APK entry.
pub fn required_stored_alignment(name: &str) -> u64 {
    if name.ends_with(".so") {
        ALIGN_16K
    } else {
        ALIGN_4B
    }
}

/// Verifies that an APK satisfies Android 4-byte (`resources.arsc` & stored entries),
/// 4 KB (`.so`), and 16 KB (`.so`) alignment rules.
pub fn verify_apk_alignment(path: &Path) -> Result<AlignmentReport> {
    let mut f = File::open(path).with_context(|| format!("failed to open {}", path.display()))?;
    verify_apk_alignment_reader(&mut f)
}

/// Verifies alignment on any `Read + Seek` APK stream.
pub fn verify_apk_alignment_reader<R: Read + Seek>(reader: &mut R) -> Result<AlignmentReport> {
    let layout = parse_apk_zip_layout(reader)?;

    let mut aligned_4b = true;
    let mut aligned_4k_so = true;
    let mut aligned_16k_so = true;
    let mut resources_arsc_ok = true;
    let mut has_data_descriptors = false;
    let mut checked_stored_entries = 0usize;
    let mut checked_so_entries = 0usize;
    let mut misaligned_entries = Vec::new();

    for e in &layout.entries {
        if (e.flags & FLAG_DATA_DESCRIPTOR) != 0 || (e.lfh_flags & FLAG_DATA_DESCRIPTOR) != 0 {
            has_data_descriptors = true;
        }
        if e.is_dir() {
            continue;
        }
        if e.name == "resources.arsc" {
            if e.method != METHOD_STORED || e.data_offset % ALIGN_4B != 0 {
                resources_arsc_ok = false;
                aligned_4b = false;
                misaligned_entries.push(format!(
                    "resources.arsc (method={}, offset={})",
                    e.method, e.data_offset
                ));
            }
        }
        if e.method == METHOD_STORED {
            checked_stored_entries += 1;
            if e.data_offset % ALIGN_4B != 0 {
                aligned_4b = false;
                misaligned_entries.push(format!(
                    "{} (offset={} not 4-byte aligned)",
                    e.name, e.data_offset
                ));
            }
            if e.name.ends_with(".so") {
                checked_so_entries += 1;
                if e.data_offset % ALIGN_4K != 0 {
                    aligned_4k_so = false;
                }
                if e.data_offset % ALIGN_16K != 0 {
                    aligned_16k_so = false;
                    misaligned_entries.push(format!(
                        "{} (offset={} not 16KB aligned)",
                        e.name, e.data_offset
                    ));
                }
            }
        }
    }

    Ok(AlignmentReport {
        aligned_4b,
        aligned_4k_so,
        aligned_16k_so,
        resources_arsc_ok,
        has_data_descriptors,
        checked_stored_entries,
        checked_so_entries,
        misaligned_entries,
    })
}

/// Internal sink that computes SHA-256, CRC32, and total uncompressed byte count while inflating.
struct HashCrcSink {
    sha256: Sha256,
    crc32: Crc32Hasher,
    len: u64,
}

impl HashCrcSink {
    fn new() -> Self {
        Self {
            sha256: Sha256::new(),
            crc32: Crc32Hasher::new(),
            len: 0,
        }
    }
}

impl Write for HashCrcSink {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.sha256.update(buf);
        self.crc32.update(buf);
        self.len += buf.len() as u64;
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Single-pass streaming APK ZIP writer with exact 16 KB `.so` and 4-byte `STORED` alignment,
/// zero data descriptors, raw-deflate passthrough, and live uncompressed SHA-256 tracking.
pub struct StreamingApkZipWriter<W: Write + Seek> {
    writer: W,
    pos: u64,
    entries: Vec<WrittenZipEntry>,
}

impl<W: Write + Seek> StreamingApkZipWriter<W> {
    pub fn new(mut writer: W) -> Result<Self> {
        writer.seek(SeekFrom::Start(0))?;
        Ok(Self {
            writer,
            pos: 0,
            entries: Vec::new(),
        })
    }

    pub fn entries(&self) -> &[WrittenZipEntry] {
        &self.entries
    }

    pub fn current_offset(&self) -> u64 {
        self.pos
    }

    fn compute_flags(name: &str) -> u16 {
        if name.is_ascii() {
            0
        } else {
            FLAG_UTF8
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn write_lfh(
        &mut self,
        name: &str,
        method: u16,
        flags: u16,
        mod_time: u16,
        mod_date: u16,
        crc32: u32,
        comp_size: u32,
        uncomp_size: u32,
        align: u64,
    ) -> Result<(u32, u64)> {
        let lfh_offset_u64 = self.pos;
        ensure!(
            lfh_offset_u64 < u32::MAX as u64,
            "APK exceeds 4 GiB standard ZIP limit at entry '{name}'"
        );
        let name_bytes = name.as_bytes();
        ensure!(
            name_bytes.len() <= u16::MAX as usize,
            "ZIP entry name too long: '{name}'"
        );

        let align = align.max(1);
        let unpadded_data_offset = lfh_offset_u64 + 30 + name_bytes.len() as u64;
        let rem = unpadded_data_offset % align;
        let pad = if rem == 0 { 0 } else { align - rem };
        ensure!(
            pad <= u16::MAX as u64,
            "alignment padding {pad} exceeds ZIP u16 extra_len"
        );
        let extra_len = pad as u16;
        let data_offset = unpadded_data_offset + pad;

        let version_needed: u16 = if method == METHOD_DEFLATED { 20 } else { 10 };
        let mut hdr = [0u8; 30];
        hdr[0..4].copy_from_slice(&LFH_SIGNATURE.to_le_bytes());
        hdr[4..6].copy_from_slice(&version_needed.to_le_bytes());
        hdr[6..8].copy_from_slice(&flags.to_le_bytes());
        hdr[8..10].copy_from_slice(&method.to_le_bytes());
        hdr[10..12].copy_from_slice(&mod_time.to_le_bytes());
        hdr[12..14].copy_from_slice(&mod_date.to_le_bytes());
        hdr[14..18].copy_from_slice(&crc32.to_le_bytes());
        hdr[18..22].copy_from_slice(&comp_size.to_le_bytes());
        hdr[22..26].copy_from_slice(&uncomp_size.to_le_bytes());
        hdr[26..28].copy_from_slice(&(name_bytes.len() as u16).to_le_bytes());
        hdr[28..30].copy_from_slice(&extra_len.to_le_bytes());

        self.writer.write_all(&hdr)?;
        self.writer.write_all(name_bytes)?;
        if extra_len > 0 {
            // Write zero-byte alignment padding identically to Android's official `zipalign`.
            const ZEROS: [u8; 4096] = [0u8; 4096];
            let mut left = extra_len as usize;
            while left > 0 {
                let n = left.min(ZEROS.len());
                self.writer.write_all(&ZEROS[..n])?;
                left -= n;
            }
        }
        self.pos = data_offset;
        Ok((lfh_offset_u64 as u32, data_offset))
    }

    /// Writes an uncompressed (`STORED`) entry from an in-memory slice with exact alignment.
    pub fn write_stored_bytes(&mut self, name: &str, data: &[u8], align: u64) -> Result<[u8; 32]> {
        ensure!(
            data.len() <= u32::MAX as usize,
            "entry '{name}' exceeds 4 GiB"
        );
        let crc32 = crc32fast::hash(data);
        let sha256: [u8; 32] = Sha256::digest(data).into();
        let size = data.len() as u32;
        let flags = Self::compute_flags(name);

        let (lfh_offset, data_offset) = self.write_lfh(
            name,
            METHOD_STORED,
            flags,
            DEFAULT_DOS_TIME,
            DEFAULT_DOS_DATE,
            crc32,
            size,
            size,
            align,
        )?;
        self.writer.write_all(data)?;
        self.pos += data.len() as u64;

        self.entries.push(WrittenZipEntry {
            name: name.to_owned(),
            method: METHOD_STORED,
            flags,
            mod_time: DEFAULT_DOS_TIME,
            mod_date: DEFAULT_DOS_DATE,
            crc32,
            comp_size: size,
            uncomp_size: size,
            lfh_offset,
            data_offset,
            external_attrs: 0,
            sha256,
        });
        Ok(sha256)
    }

    /// Writes a compressed (`DEFLATED`) entry from an in-memory slice.
    pub fn write_deflated_bytes(&mut self, name: &str, data: &[u8]) -> Result<[u8; 32]> {
        ensure!(
            data.len() <= u32::MAX as usize,
            "entry '{name}' exceeds 4 GiB"
        );
        let crc32 = crc32fast::hash(data);
        let sha256: [u8; 32] = Sha256::digest(data).into();

        let mut enc = DeflateEncoder::new(Vec::new(), Compression::default());
        enc.write_all(data)?;
        let compressed = enc.finish()?;
        ensure!(
            compressed.len() <= u32::MAX as usize,
            "compressed entry '{name}' exceeds 4 GiB"
        );

        let comp_size = compressed.len() as u32;
        let uncomp_size = data.len() as u32;
        let flags = Self::compute_flags(name);

        let (lfh_offset, data_offset) = self.write_lfh(
            name,
            METHOD_DEFLATED,
            flags,
            DEFAULT_DOS_TIME,
            DEFAULT_DOS_DATE,
            crc32,
            comp_size,
            uncomp_size,
            1,
        )?;
        self.writer.write_all(&compressed)?;
        self.pos += compressed.len() as u64;

        self.entries.push(WrittenZipEntry {
            name: name.to_owned(),
            method: METHOD_DEFLATED,
            flags,
            mod_time: DEFAULT_DOS_TIME,
            mod_date: DEFAULT_DOS_DATE,
            crc32,
            comp_size,
            uncomp_size,
            lfh_offset,
            data_offset,
            external_attrs: 0,
            sha256,
        });
        Ok(sha256)
    }

    /// Streams an existing entry from `src` into the output APK without re-compressing `DEFLATED`
    /// entries, while computing its exact uncompressed SHA-256 digest and verifying CRC32.
    ///
    /// If `entry.name == "resources.arsc"` and the input entry was compressed, it is automatically
    /// converted to `STORED` with 4-byte alignment (required by Android 11+).
    pub fn copy_entry_streaming<R: Read + Seek>(
        &mut self,
        src: &mut R,
        entry: &CentralDirEntry,
    ) -> Result<[u8; 32]> {
        let force_stored = entry.name == "resources.arsc";

        // Special case: compressed resources.arsc must be decompressed and stored 4-byte aligned.
        if force_stored && entry.method != METHOD_STORED {
            let uncompressed = read_entry_uncompressed(src, entry)?;
            return self.write_stored_bytes(&entry.name, &uncompressed, ALIGN_4B);
        }

        ensure!(
            entry.comp_size <= u32::MAX as u64 && entry.uncomp_size <= u32::MAX as u64,
            "entry '{}' exceeds 4 GiB",
            entry.name
        );

        let flags = (entry.flags & !FLAG_DATA_DESCRIPTOR) | Self::compute_flags(&entry.name);
        let align = if entry.method == METHOD_STORED {
            required_stored_alignment(&entry.name)
        } else {
            1
        };

        match entry.method {
            METHOD_STORED => {
                let (lfh_offset, data_offset) = self.write_lfh(
                    &entry.name,
                    METHOD_STORED,
                    flags,
                    entry.mod_time,
                    entry.mod_date,
                    entry.crc32,
                    entry.comp_size as u32,
                    entry.uncomp_size as u32,
                    align,
                )?;
                src.seek(SeekFrom::Start(entry.data_offset))?;
                let mut left = entry.comp_size;
                let mut buf = [0u8; STREAM_BUF_SIZE];
                let mut sha = Sha256::new();
                let mut crc = Crc32Hasher::new();

                while left > 0 {
                    let want = (left as usize).min(buf.len());
                    let n = src.read(&mut buf[..want])?;
                    ensure!(n > 0, "unexpected EOF streaming STORED entry '{}'", entry.name);
                    self.writer.write_all(&buf[..n])?;
                    sha.update(&buf[..n]);
                    crc.update(&buf[..n]);
                    left -= n as u64;
                }
                ensure!(
                    crc.finalize() == entry.crc32,
                    "CRC32 mismatch while copying STORED entry '{}'",
                    entry.name
                );

                self.pos += entry.comp_size;
                let sha256: [u8; 32] = sha.finalize().into();
                self.entries.push(WrittenZipEntry {
                    name: entry.name.clone(),
                    method: METHOD_STORED,
                    flags,
                    mod_time: entry.mod_time,
                    mod_date: entry.mod_date,
                    crc32: entry.crc32,
                    comp_size: entry.comp_size as u32,
                    uncomp_size: entry.uncomp_size as u32,
                    lfh_offset,
                    data_offset,
                    external_attrs: entry.external_attrs,
                    sha256,
                });
                Ok(sha256)
            }
            METHOD_DEFLATED => {
                let (lfh_offset, data_offset) = self.write_lfh(
                    &entry.name,
                    METHOD_DEFLATED,
                    flags,
                    entry.mod_time,
                    entry.mod_date,
                    entry.crc32,
                    entry.comp_size as u32,
                    entry.uncomp_size as u32,
                    1,
                )?;
                src.seek(SeekFrom::Start(entry.data_offset))?;
                let mut left = entry.comp_size;
                let mut buf = [0u8; STREAM_BUF_SIZE];
                let mut decoder = DeflateDecoder::new(HashCrcSink::new());

                while left > 0 {
                    let want = (left as usize).min(buf.len());
                    let n = src.read(&mut buf[..want])?;
                    ensure!(
                        n > 0,
                        "unexpected EOF streaming DEFLATED entry '{}'",
                        entry.name
                    );
                    self.writer.write_all(&buf[..n])?;
                    decoder.write_all(&buf[..n])?;
                    left -= n as u64;
                }
                let sink = decoder.finish()?;
                ensure!(
                    sink.len == entry.uncomp_size,
                    "inflated size mismatch copying DEFLATED entry '{}' (got {}, expected {})",
                    entry.name,
                    sink.len,
                    entry.uncomp_size
                );
                ensure!(
                    sink.crc32.finalize() == entry.crc32,
                    "CRC32 mismatch copying DEFLATED entry '{}'",
                    entry.name
                );

                self.pos += entry.comp_size;
                let sha256: [u8; 32] = sink.sha256.finalize().into();
                self.entries.push(WrittenZipEntry {
                    name: entry.name.clone(),
                    method: METHOD_DEFLATED,
                    flags,
                    mod_time: entry.mod_time,
                    mod_date: entry.mod_date,
                    crc32: entry.crc32,
                    comp_size: entry.comp_size as u32,
                    uncomp_size: entry.uncomp_size as u32,
                    lfh_offset,
                    data_offset,
                    external_attrs: entry.external_attrs,
                    sha256,
                });
                Ok(sha256)
            }
            other => bail!(
                "unsupported ZIP compression method {other} for input entry '{}'",
                entry.name
            ),
        }
    }

    /// Serializes the Central Directory and 22-byte EOCD record.
    pub fn build_central_directory_and_eocd(&self, cd_offset: u64) -> Result<(Vec<u8>, Vec<u8>)> {
        ensure!(
            self.entries.len() < u16::MAX as usize,
            "APK entry count ({}) exceeds standard ZIP limit (65534)",
            self.entries.len()
        );
        ensure!(
            cd_offset < u32::MAX as u64,
            "Central Directory offset ({cd_offset}) exceeds 4 GiB ZIP limit"
        );

        let mut cd = Vec::with_capacity(self.entries.len() * 64);
        for e in &self.entries {
            let name_bytes = e.name.as_bytes();
            let version_needed: u16 = if e.method == METHOD_DEFLATED { 20 } else { 10 };
            let version_made_by: u16 = 20;

            cd.extend_from_slice(&CD_SIGNATURE.to_le_bytes());
            cd.extend_from_slice(&version_made_by.to_le_bytes());
            cd.extend_from_slice(&version_needed.to_le_bytes());
            cd.extend_from_slice(&e.flags.to_le_bytes());
            cd.extend_from_slice(&e.method.to_le_bytes());
            cd.extend_from_slice(&e.mod_time.to_le_bytes());
            cd.extend_from_slice(&e.mod_date.to_le_bytes());
            cd.extend_from_slice(&e.crc32.to_le_bytes());
            cd.extend_from_slice(&e.comp_size.to_le_bytes());
            cd.extend_from_slice(&e.uncomp_size.to_le_bytes());
            cd.extend_from_slice(&(name_bytes.len() as u16).to_le_bytes());
            cd.extend_from_slice(&0u16.to_le_bytes()); // extra len (keep CD clean; padding lives in LFH)
            cd.extend_from_slice(&0u16.to_le_bytes()); // comment len
            cd.extend_from_slice(&0u16.to_le_bytes()); // disk number start
            cd.extend_from_slice(&0u16.to_le_bytes()); // internal attrs
            cd.extend_from_slice(&e.external_attrs.to_le_bytes());
            cd.extend_from_slice(&e.lfh_offset.to_le_bytes());
            cd.extend_from_slice(name_bytes);
        }

        ensure!(
            cd.len() < u32::MAX as usize,
            "Central Directory size exceeds 4 GiB ZIP limit"
        );
        let count = self.entries.len() as u16;
        let mut eocd = Vec::with_capacity(22);
        eocd.extend_from_slice(&EOCD_SIGNATURE.to_le_bytes());
        eocd.extend_from_slice(&0u16.to_le_bytes()); // disk number
        eocd.extend_from_slice(&0u16.to_le_bytes()); // cd start disk
        eocd.extend_from_slice(&count.to_le_bytes());
        eocd.extend_from_slice(&count.to_le_bytes());
        eocd.extend_from_slice(&(cd.len() as u32).to_le_bytes());
        eocd.extend_from_slice(&(cd_offset as u32).to_le_bytes());
        eocd.extend_from_slice(&0u16.to_le_bytes()); // comment length

        Ok((cd, eocd))
    }

    /// Finishes the ZIP archive without an APK Signing Block.
    pub fn finish_unsigned(mut self) -> Result<(W, Vec<WrittenZipEntry>)> {
        let cd_offset = self.pos;
        let (cd, eocd) = self.build_central_directory_and_eocd(cd_offset)?;
        self.writer.write_all(&cd)?;
        self.writer.write_all(&eocd)?;
        self.writer.flush()?;
        self.pos += (cd.len() + eocd.len()) as u64;
        Ok((self.writer, self.entries))
    }

    /// Consumes the writer after all ZIP entries (including v1 `META-INF/*` entries) are written,
    /// returning the underlying writer positioned at `zip_entries_end`, the exact `zip_entries_end`
    /// offset, the serialized Central Directory, the pre-block EOCD, and all written entry records.
    pub fn into_parts_for_v2_v3_signing(
        mut self,
    ) -> Result<(W, u64, Vec<u8>, Vec<u8>, Vec<WrittenZipEntry>)> {
        self.writer.flush()?;
        let zip_entries_end = self.pos;
        let (cd, eocd_pre_block) = self.build_central_directory_and_eocd(zip_entries_end)?;
        Ok((
            self.writer,
            zip_entries_end,
            cd,
            eocd_pre_block,
            self.entries,
        ))
    }
}

/// Pure-Rust standalone 16 KB APK aligner (`zipalign -P 16 4` replacement).
///
/// Strips any pre-existing APK Signing Block (since alignment changes entry offsets, requiring
/// re-signing afterwards). Preserves all entries with exact 16 KB `.so` and 4-byte `STORED` alignment.
pub fn align_apk(input_path: &Path, output_path: &Path) -> Result<AlignmentReport> {
    let mut in_file = File::open(input_path)
        .with_context(|| format!("failed to open input APK {}", input_path.display()))?;
    let layout = parse_apk_zip_layout(&mut in_file)?;

    let out_file = File::create(output_path)
        .with_context(|| format!("failed to create aligned APK {}", output_path.display()))?;
    let mut writer = StreamingApkZipWriter::new(out_file)?;

    for entry in &layout.entries {
        if entry.is_dir() {
            continue;
        }
        writer.copy_entry_streaming(&mut in_file, entry)?;
    }

    let (mut out_file, _) = writer.finish_unsigned()?;
    out_file.flush()?;
    drop(out_file);

    verify_apk_alignment(output_path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn streaming_writer_aligns_so_to_16k_and_resources_to_4b_and_roundtrips() {
        let mut cursor = Cursor::new(Vec::new());
        let mut writer = StreamingApkZipWriter::new(&mut cursor).unwrap();
        writer
            .write_deflated_bytes("AndroidManifest.xml", b"hello manifest bytes 123456")
            .unwrap();
        writer
            .write_stored_bytes("resources.arsc", b"ARSC_PAYLOAD_DATA", ALIGN_4B)
            .unwrap();
        writer
            .write_stored_bytes("lib/arm64-v8a/libtest.so", &[0x7f, b'E', b'L', b'F', 1, 2], ALIGN_16K)
            .unwrap();
        writer
            .write_stored_bytes("assets/data.bin", b"abc", ALIGN_4B)
            .unwrap();
        let (_, written) = writer.finish_unsigned().unwrap();

        let so_entry = written.iter().find(|e| e.name.ends_with(".so")).unwrap();
        assert_eq!(so_entry.data_offset % ALIGN_16K, 0);
        let arsc_entry = written.iter().find(|e| e.name == "resources.arsc").unwrap();
        assert_eq!(arsc_entry.data_offset % ALIGN_4B, 0);

        cursor.set_position(0);
        let report = verify_apk_alignment_reader(&mut cursor).unwrap();
        assert!(report.is_fully_aligned_16k());
        assert!(!report.has_data_descriptors);
        assert_eq!(report.checked_so_entries, 1);
        assert_eq!(report.checked_stored_entries, 3);

        // Now copy all entries via streaming copy into a second archive and verify identical content & alignment.
        cursor.set_position(0);
        let layout = parse_apk_zip_layout(&mut cursor).unwrap();
        let mut cursor2 = Cursor::new(Vec::new());
        let mut writer2 = StreamingApkZipWriter::new(&mut cursor2).unwrap();
        for e in &layout.entries {
            writer2.copy_entry_streaming(&mut cursor, e).unwrap();
        }
        writer2.finish_unsigned().unwrap();

        cursor2.set_position(0);
        let report2 = verify_apk_alignment_reader(&mut cursor2).unwrap();
        assert!(report2.is_fully_aligned_16k());

        cursor2.set_position(0);
        let layout2 = parse_apk_zip_layout(&mut cursor2).unwrap();
        let manifest_entry = layout2.find_entry("AndroidManifest.xml").unwrap();
        assert_eq!(
            read_entry_uncompressed(&mut cursor2, manifest_entry).unwrap(),
            b"hello manifest bytes 123456"
        );
    }
}
