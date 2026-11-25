//! Pure-Rust APK Signature Scheme v1 (JAR), v2, and v3 signer, verifier, and keystore loader.
//!
//! Completely eliminates external `apksigner`, `keytool`, and Java dependencies:
//! - Generates self-signed RSA-2048 X.509 v3 certificates and persists a reusable debug keypair
//!   in the user config directory (allowing seamless `adb install -r` re-installs on Quest).
//! - Loads custom user signing credentials from `.p12`/`.pfx` (PKCS#12), `.jks`/`.keystore` (JKS),
//!   or `.pem` (PKCS#8/PKCS#1 private key + X.509 certificate bundle).
//! - Produces Android-compliant APK Signature Scheme v1 (`META-INF/MANIFEST.MF`, `META-INF/CERT.SF`
//!   with `X-Android-APK-Signed: 2, 3`, `META-INF/CERT.RSA` DER PKCS#7), v2 (`0x7109871a` with
//!   v3 rollback-protection attribute `0xbeeff00d`), and v3 (`0xf05368c0`, SDK 28..MAX) with
//!   4 KB verity padding (`0x42726577`).
//! - Cryptographically verifies v1, v2, and v3 signatures and detects tampering or scheme stripping.

use crate::align::{
    digest_entry_uncompressed, parse_apk_zip_layout, read_entry_uncompressed, ApkZipLayout,
    CentralDirEntry, StreamingApkZipWriter, WrittenZipEntry, APK_SIG_BLOCK_MAGIC,
};
use anyhow::{anyhow, bail, ensure, Context, Result};
use base64::engine::general_purpose::STANDARD as BASE64;
use base64::Engine;
use p12_keystore::{KeyStore as P12KeyStore, KeyStoreEntry, Pkcs12ImportPolicy};
use rand::rngs::OsRng;
use rand::RngCore;
use rsa::pkcs1::DecodeRsaPrivateKey;
use rsa::pkcs1v15::{Signature as RsaSignature, SigningKey, VerifyingKey};
use rsa::pkcs8::{DecodePrivateKey, DecodePublicKey, EncodePrivateKey, EncodePublicKey, LineEnding};
use rsa::signature::{SignatureEncoding, Signer, Verifier};
use rsa::{RsaPrivateKey, RsaPublicKey};
use serde::{Deserialize, Serialize};
use sha1::Sha1;
use sha2::{Digest, Sha256, Sha512};
use std::collections::{BTreeMap, HashMap};
use std::fs::{self, File};
use std::io::{Cursor, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

// APK Signing Block pair IDs
const APK_SIG_SCHEME_V2_BLOCK_ID: u32 = 0x7109_871a;
const APK_SIG_SCHEME_V3_BLOCK_ID: u32 = 0xf053_68c0;
const VERITY_PADDING_BLOCK_ID: u32 = 0x4272_6577;

// APK Signature Scheme v2/v3 signature algorithm IDs
const SIG_ALGO_RSA_PKCS1_V1_5_WITH_SHA256: u32 = 0x0103;
const SIG_ALGO_RSA_PKCS1_V1_5_WITH_SHA512: u32 = 0x0104;

// v2 -> v3 stripping protection attribute ID
const STRIPPING_PROTECTION_ATTR_ID: u32 = 0xbeef_f00d;
const V3_SCHEME_VERSION_ID: u32 = 3;

// Default v3 SDK range matching Android `apksigner`
const V3_MIN_SDK: u32 = 28;
const V3_MAX_SDK: u32 = 0x7fff_ffff;

const CHUNK_SIZE_1MIB: usize = 1024 * 1024;
const VERITY_BLOCK_ALIGN: usize = 4096;

// ASN.1 DER OID payload bytes
const OID_RSA_ENCRYPTION: &[u8] = &[0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x01];
const OID_SHA256_WITH_RSA: &[u8] = &[0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x0b];
const OID_SHA256: &[u8] = &[0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01];
const OID_SHA1: &[u8] = &[0x2b, 0x0e, 0x03, 0x02, 0x1a];
const OID_SHA512: &[u8] = &[0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x03];
const OID_PKCS7_DATA: &[u8] = &[0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x07, 0x01];
const OID_PKCS7_SIGNED_DATA: &[u8] = &[0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x07, 0x02];
const OID_COMMON_NAME: &[u8] = &[0x55, 0x04, 0x03];
const OID_ORGANIZATION: &[u8] = &[0x55, 0x04, 0x0a];
const OID_COUNTRY: &[u8] = &[0x55, 0x04, 0x06];

/// Signing keypair and X.509 certificate chain used for APK Signature Scheme v1, v2, and v3.
#[derive(Clone)]
pub struct SigningKeyPair {
    pub private_key: RsaPrivateKey,
    pub cert_chain_der: Vec<Vec<u8>>,
}

impl std::fmt::Debug for SigningKeyPair {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SigningKeyPair")
            .field("cert_sha256", &self.cert_sha256_hex())
            .field("chain_len", &self.cert_chain_der.len())
            .finish()
    }
}

/// Summary of pure-Rust APK Signature Scheme v1/v2/v3 verification.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApkSignatureVerification {
    pub valid: bool,
    pub v1_present: bool,
    pub v1_verified: bool,
    pub v2_present: bool,
    pub v2_verified: bool,
    pub v3_present: bool,
    pub v3_verified: bool,
    pub cert_sha256: Option<String>,
    pub cert_subject: Option<String>,
    pub errors: Vec<String>,
}

/// Summary returned after signing an APK.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApkSignSummary {
    pub cert_sha256: String,
    pub cert_subject: String,
    pub key_source: String,
    pub v1_signed: bool,
    pub v2_signed: bool,
    pub v3_signed: bool,
}

/// Returns `true` if `name` is a JAR v1 signature metadata entry in `META-INF/`.
pub fn is_v1_signature_entry(name: &str) -> bool {
    let upper = name.to_ascii_uppercase();
    if !upper.starts_with("META-INF/") {
        return false;
    }
    let rest = &upper["META-INF/".len()..];
    if rest.contains('/') {
        return false;
    }
    rest == "MANIFEST.MF"
        || rest.ends_with(".SF")
        || rest.ends_with(".RSA")
        || rest.ends_with(".DSA")
        || rest.ends_with(".EC")
        || rest.starts_with("SIG-")
}

impl SigningKeyPair {
    /// Generates a fresh 2048-bit RSA keypair and self-signed X.509 v3 debug certificate.
    pub fn generate_self_signed(cn: &str, org: &str, country: &str) -> Result<Self> {
        let mut rng = OsRng;
        let private_key = RsaPrivateKey::new(&mut rng, 2048)
            .context("failed to generate RSA-2048 signing key")?;
        let cert_der = build_self_signed_x509_cert(&private_key, cn, org, country)?;
        Ok(Self {
            private_key,
            cert_chain_der: vec![cert_der],
        })
    }

    /// Generates an ephemeral ALVR Quest debug signing keypair.
    pub fn generate_ephemeral_debug() -> Result<Self> {
        Self::generate_self_signed("ALVR Quest Hand Layer Debug", "ALVR", "US")
    }

    /// Loads or creates a persistent debug signing keypair stored in the user's config directory
    /// so repeated patches share the same signing certificate (enabling `adb install -r`).
    pub fn load_or_create_persistent_debug() -> Result<(Self, String)> {
        if let Some(path) = default_debug_key_path() {
            if path.is_file() {
                if let Ok(bytes) = fs::read(&path) {
                    if let Ok(kp) = Self::from_pem_bytes(&bytes) {
                        return Ok((kp, format!("persistent debug key ({})", path.display())));
                    }
                }
            }
            let kp = Self::generate_ephemeral_debug()?;
            if let Some(parent) = path.parent() {
                let _ = fs::create_dir_all(parent);
            }
            if let Ok(pem) = kp.to_pem_bundle() {
                if fs::write(&path, pem.as_bytes()).is_ok() {
                    #[cfg(unix)]
                    {
                        use std::os::unix::fs::PermissionsExt;
                        let _ = fs::set_permissions(&path, fs::Permissions::from_mode(0o600));
                    }
                    return Ok((kp, format!("persistent debug key ({})", path.display())));
                }
            }
            return Ok((kp, "ephemeral debug key".to_owned()));
        }

        let kp = Self::generate_ephemeral_debug()?;
        Ok((kp, "ephemeral debug key".to_owned()))
    }

    /// Loads a signing keypair from a `.pem`, `.p12`/`.pfx` (PKCS#12), or `.jks`/`.keystore` file.
    pub fn from_keystore_file(
        path: &Path,
        password: Option<&str>,
        alias: Option<&str>,
    ) -> Result<Self> {
        let bytes = fs::read(path)
            .with_context(|| format!("failed to read keystore {}", path.display()))?;
        Self::from_keystore_bytes(&bytes, password, alias)
            .with_context(|| format!("failed to load signing key from {}", path.display()))
    }

    /// Loads a signing keypair from raw bytes, auto-detecting PEM, JKS, or PKCS#12 format.
    pub fn from_keystore_bytes(
        bytes: &[u8],
        password: Option<&str>,
        alias: Option<&str>,
    ) -> Result<Self> {
        if bytes.windows(11).any(|w| w == b"-----BEGIN ") {
            return Self::from_pem_bytes(bytes);
        }
        if bytes.starts_with(&[0xfe, 0xed, 0xfe, 0xed]) {
            return Self::from_jks_bytes(bytes, password.unwrap_or("android"), alias);
        }
        Self::from_pkcs12_bytes(bytes, password.unwrap_or("android"), alias)
    }

    /// Parses a PEM bundle containing one RSA private key (PKCS#8 or PKCS#1) and one or more
    /// `CERTIFICATE` blocks.
    pub fn from_pem_bytes(bytes: &[u8]) -> Result<Self> {
        let text = std::str::from_utf8(bytes).context("PEM file is not valid UTF-8")?;
        let mut private_key: Option<RsaPrivateKey> = None;
        let mut cert_chain_der: Vec<Vec<u8>> = Vec::new();

        for (label, der) in parse_pem_blocks(text)? {
            match label.as_str() {
                "PRIVATE KEY" => {
                    private_key = Some(
                        RsaPrivateKey::from_pkcs8_der(&der)
                            .context("failed to decode PKCS#8 RSA PRIVATE KEY from PEM")?,
                    );
                }
                "RSA PRIVATE KEY" => {
                    private_key = Some(
                        RsaPrivateKey::from_pkcs1_der(&der)
                            .context("failed to decode PKCS#1 RSA PRIVATE KEY from PEM")?,
                    );
                }
                "CERTIFICATE" => {
                    cert_chain_der.push(der);
                }
                _ => {}
            }
        }

        let private_key =
            private_key.context("PEM file does not contain a PRIVATE KEY or RSA PRIVATE KEY")?;
        ensure!(
            !cert_chain_der.is_empty(),
            "PEM file does not contain an X.509 CERTIFICATE"
        );
        Self::validate_key_matches_cert(&private_key, &cert_chain_der[0])?;
        Ok(Self {
            private_key,
            cert_chain_der,
        })
    }

    /// Parses a PKCS#12 (`.p12`/`.pfx`) keystore in pure Rust.
    pub fn from_pkcs12_bytes(
        bytes: &[u8],
        password: &str,
        alias: Option<&str>,
    ) -> Result<Self> {
        let ks = P12KeyStore::from_pkcs12(bytes, password, Pkcs12ImportPolicy::Relaxed)
            .map_err(|e| anyhow!("failed to parse PKCS#12 keystore (check password): {e}"))?;

        let chain = if let Some(target_alias) = alias {
            match ks.entry(target_alias) {
                Some(KeyStoreEntry::PrivateKeyChain(c)) => c,
                Some(_) => bail!("PKCS#12 alias '{target_alias}' is not a private key entry"),
                None => {
                    // Fallback to case-insensitive match (Java keytool lowercases aliases).
                    let found = ks.entries().find_map(|(k, v)| {
                        if k.eq_ignore_ascii_case(target_alias) {
                            if let KeyStoreEntry::PrivateKeyChain(c) = v {
                                return Some(c);
                            }
                        }
                        None
                    });
                    found.ok_or_else(|| {
                        anyhow!("alias '{target_alias}' not found in PKCS#12 keystore")
                    })?
                }
            }
        } else {
            ks.private_key_chain()
                .map(|(_, c)| c)
                .ok_or_else(|| anyhow!("no private key chain found in PKCS#12 keystore"))?
        };

        let key_der = chain.key().as_der();
        let private_key = RsaPrivateKey::from_pkcs8_der(key_der)
            .or_else(|_| RsaPrivateKey::from_pkcs1_der(key_der))
            .context("PKCS#12 private key is not a supported RSA key")?;

        let cert_chain_der: Vec<Vec<u8>> = chain
            .certs()
            .iter()
            .map(|c| c.as_der().to_vec())
            .collect();
        ensure!(
            !cert_chain_der.is_empty(),
            "PKCS#12 private key entry has no X.509 certificate"
        );
        Self::validate_key_matches_cert(&private_key, &cert_chain_der[0])?;

        Ok(Self {
            private_key,
            cert_chain_der,
        })
    }

    /// Parses a Java KeyStore (`.jks` / legacy `.keystore`) in pure Rust.
    pub fn from_jks_bytes(bytes: &[u8], password: &str, alias: Option<&str>) -> Result<Self> {
        let mut ks = jks::KeyStore::new();
        let pw_bytes = password.as_bytes();
        ks.load(&mut Cursor::new(bytes), pw_bytes)
            .map_err(|e| anyhow!("failed to parse JKS keystore (check password): {e}"))?;

        let chosen_alias = if let Some(a) = alias {
            if ks.is_private_key_entry(a) {
                a.to_owned()
            } else {
                ks.aliases()
                    .into_iter()
                    .find(|cand| cand.eq_ignore_ascii_case(a) && ks.is_private_key_entry(cand))
                    .ok_or_else(|| anyhow!("private key alias '{a}' not found in JKS keystore"))?
            }
        } else {
            ks.aliases()
                .into_iter()
                .find(|a| ks.is_private_key_entry(a))
                .ok_or_else(|| anyhow!("no private key entry found in JKS keystore"))?
        };

        let entry = ks
            .get_private_key_entry(&chosen_alias, pw_bytes)
            .map_err(|e| anyhow!("failed to decrypt JKS key '{chosen_alias}': {e}"))?;

        let private_key = RsaPrivateKey::from_pkcs8_der(&entry.private_key)
            .or_else(|_| RsaPrivateKey::from_pkcs1_der(&entry.private_key))
            .context("JKS private key is not a supported RSA key")?;
        let cert_chain_der: Vec<Vec<u8>> = entry
            .certificate_chain
            .into_iter()
            .map(|c| c.content)
            .collect();
        ensure!(
            !cert_chain_der.is_empty(),
            "JKS private key entry '{chosen_alias}' has no certificate chain"
        );
        Self::validate_key_matches_cert(&private_key, &cert_chain_der[0])?;

        Ok(Self {
            private_key,
            cert_chain_der,
        })
    }

    fn validate_key_matches_cert(private_key: &RsaPrivateKey, leaf_cert_der: &[u8]) -> Result<()> {
        let info = parse_x509_cert_info(leaf_cert_der)
            .context("leaf certificate is not a valid X.509 DER certificate")?;
        let expected_spki = RsaPublicKey::from(private_key)
            .to_public_key_der()
            .context("failed to encode RSA public key SPKI")?;
        ensure!(
            info.spki_der == expected_spki.as_bytes(),
            "private key does not match leaf X.509 certificate public key"
        );
        Ok(())
    }

    /// Exports keypair and certificate chain as a single PEM bundle.
    pub fn to_pem_bundle(&self) -> Result<String> {
        let mut out = self
            .private_key
            .to_pkcs8_pem(LineEnding::LF)
            .context("failed to serialize PKCS#8 PEM")?
            .to_string();
        if !out.ends_with('\n') {
            out.push('\n');
        }
        for cert in &self.cert_chain_der {
            out.push_str("-----BEGIN CERTIFICATE-----\n");
            let b64 = BASE64.encode(cert);
            for chunk in b64.as_bytes().chunks(64) {
                out.push_str(std::str::from_utf8(chunk).unwrap());
                out.push('\n');
            }
            out.push_str("-----END CERTIFICATE-----\n");
        }
        Ok(out)
    }

    pub fn leaf_cert_der(&self) -> &[u8] {
        &self.cert_chain_der[0]
    }

    pub fn cert_sha256_hex(&self) -> String {
        hex::encode(Sha256::digest(self.leaf_cert_der()))
    }

    pub fn cert_subject(&self) -> String {
        parse_x509_cert_info(self.leaf_cert_der())
            .map(|c| c.subject_summary)
            .unwrap_or_else(|_| "unknown".to_owned())
    }

    pub fn public_key_spki_der(&self) -> Result<Vec<u8>> {
        let doc = RsaPublicKey::from(&self.private_key)
            .to_public_key_der()
            .context("failed to encode RSA SPKI DER")?;
        Ok(doc.as_bytes().to_vec())
    }

    fn sign_rsa_sha256(&self, data: &[u8]) -> Vec<u8> {
        let signing_key = SigningKey::<Sha256>::new(self.private_key.clone());
        signing_key.sign(data).to_bytes().to_vec()
    }
}

/// Returns the default persistent debug signing key path in the user's OS config directory.
pub fn default_debug_key_path() -> Option<PathBuf> {
    if let Ok(override_path) = std::env::var("ALVR_QUEST_SIGNING_KEY") {
        if !override_path.trim().is_empty() {
            return Some(PathBuf::from(override_path));
        }
    }
    #[cfg(target_os = "windows")]
    {
        std::env::var_os("APPDATA")
            .or_else(|| std::env::var_os("LOCALAPPDATA"))
            .map(|d| PathBuf::from(d).join("alvr-quest-patcher").join("debug_signing_key.pem"))
    }
    #[cfg(target_os = "macos")]
    {
        std::env::var_os("HOME").map(|h| {
            PathBuf::from(h)
                .join("Library")
                .join("Application Support")
                .join("alvr-quest-patcher")
                .join("debug_signing_key.pem")
        })
    }
    #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
    {
        if let Some(xdg) = std::env::var_os("XDG_CONFIG_HOME") {
            if !xdg.is_empty() {
                return Some(
                    PathBuf::from(xdg)
                        .join("alvr-quest-patcher")
                        .join("debug_signing_key.pem"),
                );
            }
        }
        std::env::var_os("HOME").map(|h| {
            PathBuf::from(h)
                .join(".config")
                .join("alvr-quest-patcher")
                .join("debug_signing_key.pem")
        })
    }
}

/// Finalizes a [`StreamingApkZipWriter`] by appending APK Signature Scheme v1 entries
/// (`META-INF/MANIFEST.MF`, `META-INF/CERT.SF`, `META-INF/CERT.RSA`) and inserting an
/// APK Signing Block containing v2 (`0x7109871a`), v3 (`0xf05368c0`), and 4 KB verity padding
/// directly before the Central Directory in a single pass.
pub fn finalize_signed_apk_stream<W: Read + Write + Seek>(
    mut writer: StreamingApkZipWriter<W>,
    keypair: &SigningKeyPair,
    key_source: String,
) -> Result<(W, ApkSignSummary)> {
    // 1. Build & append v1 JAR signature entries before writing the Central Directory.
    let (manifest_mf, cert_sf, cert_rsa) = build_v1_signature_files(writer.entries(), keypair)?;
    writer.write_deflated_bytes("META-INF/MANIFEST.MF", &manifest_mf)?;
    writer.write_deflated_bytes("META-INF/CERT.SF", &cert_sf)?;
    writer.write_deflated_bytes("META-INF/CERT.RSA", &cert_rsa)?;

    // 2. Serialize Central Directory & pre-block EOCD (`cd_offset = zip_entries_end`).
    let (mut io_handle, zip_entries_end, cd_bytes, eocd_pre_block, _) =
        writer.into_parts_for_v2_v3_signing()?;

    // 3. Compute the 1 MiB chunked SHA-256 content digest across Section 1 + Section 3 + Section 4.
    let content_digest =
        compute_apk_v2_v3_sha256_digest(&mut io_handle, zip_entries_end, &cd_bytes, &eocd_pre_block)?;

    // 4. Build the APK Signing Block (v2 + v3 + 4 KB padding).
    let signing_block = build_apk_signing_block_v2_v3(&content_digest, keypair)?;

    // 5. Write Signing Block, Central Directory, and updated EOCD with shifted `cd_offset`.
    let final_cd_offset = zip_entries_end
        .checked_add(signing_block.len() as u64)
        .ok_or_else(|| anyhow!("APK size overflow adding signing block"))?;
    ensure!(
        final_cd_offset < u32::MAX as u64,
        "signed APK exceeds 4 GiB standard ZIP limit"
    );

    let mut final_eocd = eocd_pre_block;
    final_eocd[16..20].copy_from_slice(&(final_cd_offset as u32).to_le_bytes());

    io_handle.seek(SeekFrom::Start(zip_entries_end))?;
    io_handle.write_all(&signing_block)?;
    io_handle.write_all(&cd_bytes)?;
    io_handle.write_all(&final_eocd)?;
    io_handle.flush()?;

    let summary = ApkSignSummary {
        cert_sha256: keypair.cert_sha256_hex(),
        cert_subject: keypair.cert_subject(),
        key_source,
        v1_signed: true,
        v2_signed: true,
        v3_signed: true,
    };
    Ok((io_handle, summary))
}

/// Standalone pure-Rust APK align + v1/v2/v3 signer (`zipalign` + `apksigner sign` replacement).
pub fn align_and_sign_apk(
    input_apk: &Path,
    output_apk: &Path,
    keypair: &SigningKeyPair,
    key_source: String,
) -> Result<ApkSignSummary> {
    let mut in_file = File::open(input_apk)
        .with_context(|| format!("failed to open input APK {}", input_apk.display()))?;
    let layout = parse_apk_zip_layout(&mut in_file)?;

    let out_file = fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(true)
        .open(output_apk)
        .with_context(|| format!("failed to create output APK {}", output_apk.display()))?;

    let mut writer = StreamingApkZipWriter::new(out_file)?;
    for entry in &layout.entries {
        if entry.is_dir() || is_v1_signature_entry(&entry.name) {
            continue;
        }
        writer.copy_entry_streaming(&mut in_file, entry)?;
    }

    let (mut out_file, summary) = finalize_signed_apk_stream(writer, keypair, key_source)?;
    out_file.flush()?;
    Ok(summary)
}

// ============================================================================
// APK Signature Scheme v1 (JAR Signing)
// ============================================================================

fn write_jar_attr(out: &mut String, key: &str, value: &str) {
    let full = format!("{key}: {value}");
    let mut is_first = true;
    let mut rest = full.as_str();
    while !rest.is_empty() {
        let limit = if is_first { 72 } else { 71 };
        if !is_first {
            out.push(' ');
        }
        if rest.len() <= limit {
            out.push_str(rest);
            out.push_str("\r\n");
            break;
        }
        // Find largest valid UTF-8 boundary <= limit.
        let mut split = limit;
        while split > 0 && !rest.is_char_boundary(split) {
            split -= 1;
        }
        if split == 0 {
            // Single character > limit (impossible for UTF-8 <= 4 bytes), guard anyway.
            split = rest.chars().next().map_or(rest.len(), char::len_utf8);
        }
        out.push_str(&rest[..split]);
        out.push_str("\r\n");
        rest = &rest[split..];
        is_first = false;
    }
}

fn build_v1_signature_files(
    entries: &[WrittenZipEntry],
    keypair: &SigningKeyPair,
) -> Result<(Vec<u8>, Vec<u8>, Vec<u8>)> {
    let mut main_section = String::new();
    write_jar_attr(&mut main_section, "Manifest-Version", "1.0");
    write_jar_attr(&mut main_section, "Created-By", "ALVR Quest Packager");
    main_section.push_str("\r\n");

    let main_attrs_digest = Sha256::digest(main_section.as_bytes());

    let mut manifest = main_section;
    let mut sf_entries: Vec<(String, [u8; 32])> = Vec::with_capacity(entries.len());

    for e in entries {
        if is_v1_signature_entry(&e.name) {
            continue;
        }
        let mut section = String::new();
        write_jar_attr(&mut section, "Name", &e.name);
        write_jar_attr(&mut section, "SHA-256-Digest", &BASE64.encode(e.sha256));
        section.push_str("\r\n");

        let sec_digest: [u8; 32] = Sha256::digest(section.as_bytes()).into();
        manifest.push_str(&section);
        sf_entries.push((e.name.clone(), sec_digest));
    }

    let manifest_bytes = manifest.into_bytes();
    let manifest_digest = Sha256::digest(&manifest_bytes);

    let mut cert_sf = String::new();
    write_jar_attr(&mut cert_sf, "Signature-Version", "1.0");
    write_jar_attr(&mut cert_sf, "Created-By", "ALVR Quest Packager");
    write_jar_attr(
        &mut cert_sf,
        "SHA-256-Digest-Manifest",
        &BASE64.encode(manifest_digest),
    );
    write_jar_attr(
        &mut cert_sf,
        "SHA-256-Digest-Manifest-Main-Attributes",
        &BASE64.encode(main_attrs_digest),
    );
    // Required rollback protection header declaring v2 and v3 schemes are present.
    write_jar_attr(&mut cert_sf, "X-Android-APK-Signed", "2, 3");
    cert_sf.push_str("\r\n");

    for (name, sec_digest) in sf_entries {
        write_jar_attr(&mut cert_sf, "Name", &name);
        write_jar_attr(&mut cert_sf, "SHA-256-Digest", &BASE64.encode(sec_digest));
        cert_sf.push_str("\r\n");
    }

    let cert_sf_bytes = cert_sf.into_bytes();
    let cert_rsa_bytes = build_pkcs7_signed_data(&cert_sf_bytes, keypair)?;

    Ok((manifest_bytes, cert_sf_bytes, cert_rsa_bytes))
}

fn build_pkcs7_signed_data(cert_sf_bytes: &[u8], keypair: &SigningKeyPair) -> Result<Vec<u8>> {
    let cert_info = parse_x509_cert_info(keypair.leaf_cert_der())?;
    let sig_bytes = keypair.sign_rsa_sha256(cert_sf_bytes);

    let sha256_alg = der_sequence(&[der_oid(OID_SHA256), der_null()]);
    let rsa_alg = der_sequence(&[der_oid(OID_RSA_ENCRYPTION), der_null()]);

    let digest_algorithms = der_set(&[sha256_alg.clone()]);
    let encap_content_info = der_sequence(&[der_oid(OID_PKCS7_DATA)]);

    // certificates [0] IMPLICIT SET OF Certificate
    let mut certs_concat = Vec::new();
    for c in &keypair.cert_chain_der {
        certs_concat.extend_from_slice(c);
    }
    let certs_implicit = der_tlv(0xa0, &certs_concat);

    let issuer_and_serial = der_sequence(&[cert_info.issuer_tlv, cert_info.serial_tlv]);
    let signer_info = der_sequence(&[
        der_integer_u8(1),
        issuer_and_serial,
        sha256_alg,
        rsa_alg,
        der_octet_string(&sig_bytes),
    ]);
    let signer_infos = der_set(&[signer_info]);

    let signed_data = der_sequence(&[
        der_integer_u8(1),
        digest_algorithms,
        encap_content_info,
        certs_implicit,
        signer_infos,
    ]);

    let content_info = der_sequence(&[
        der_oid(OID_PKCS7_SIGNED_DATA),
        der_tlv(0xa0, &signed_data),
    ]);
    Ok(content_info)
}

// ============================================================================
// APK Signature Scheme v2 + v3 (APK Signing Block)
// ============================================================================

fn lp_u32(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(4 + data.len());
    out.extend_from_slice(&(data.len() as u32).to_le_bytes());
    out.extend_from_slice(data);
    out
}

fn hash_1mib_chunk_sha256(chunk: &[u8]) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update([0xa5u8]);
    h.update((chunk.len() as u32).to_le_bytes());
    h.update(chunk);
    h.finalize().into()
}

fn hash_1mib_chunk_sha512(chunk: &[u8]) -> [u8; 64] {
    let mut h = Sha512::new();
    h.update([0xa5u8]);
    h.update((chunk.len() as u32).to_le_bytes());
    h.update(chunk);
    h.finalize().into()
}

/// Computes the 1 MiB chunked SHA-256 content digest across Section 1 (ZIP entries),
/// Section 3 (Central Directory), and Section 4 (EOCD with `cd_offset = zip_entries_end`).
fn compute_apk_v2_v3_sha256_digest<R: Read + Seek>(
    reader: &mut R,
    zip_entries_end: u64,
    cd_bytes: &[u8],
    eocd_with_pre_block_offset: &[u8],
) -> Result<[u8; 32]> {
    let mut chunk_digests = Vec::new();
    let mut total_chunks: u32 = 0;

    // Section 1: ZIP entries (`0..zip_entries_end`)
    reader.seek(SeekFrom::Start(0))?;
    let mut buf = vec![0u8; CHUNK_SIZE_1MIB];
    let mut left = zip_entries_end;
    while left > 0 {
        let want = (left as usize).min(CHUNK_SIZE_1MIB);
        reader.read_exact(&mut buf[..want])?;
        chunk_digests.extend_from_slice(&hash_1mib_chunk_sha256(&buf[..want]));
        total_chunks = total_chunks
            .checked_add(1)
            .ok_or_else(|| anyhow!("APK chunk count overflow"))?;
        left -= want as u64;
    }

    // Section 3: Central Directory
    for chunk in cd_bytes.chunks(CHUNK_SIZE_1MIB) {
        chunk_digests.extend_from_slice(&hash_1mib_chunk_sha256(chunk));
        total_chunks = total_chunks
            .checked_add(1)
            .ok_or_else(|| anyhow!("APK chunk count overflow"))?;
    }

    // Section 4: EOCD (with cd_offset pointing to start of APK Signing Block)
    for chunk in eocd_with_pre_block_offset.chunks(CHUNK_SIZE_1MIB) {
        chunk_digests.extend_from_slice(&hash_1mib_chunk_sha256(chunk));
        total_chunks = total_chunks
            .checked_add(1)
            .ok_or_else(|| anyhow!("APK chunk count overflow"))?;
    }

    let mut top = Sha256::new();
    top.update([0x5au8]);
    top.update(total_chunks.to_le_bytes());
    top.update(&chunk_digests);
    Ok(top.finalize().into())
}

fn compute_apk_v2_v3_sha512_digest<R: Read + Seek>(
    reader: &mut R,
    zip_entries_end: u64,
    cd_bytes: &[u8],
    eocd_with_pre_block_offset: &[u8],
) -> Result<[u8; 64]> {
    let mut chunk_digests = Vec::new();
    let mut total_chunks: u32 = 0;

    reader.seek(SeekFrom::Start(0))?;
    let mut buf = vec![0u8; CHUNK_SIZE_1MIB];
    let mut left = zip_entries_end;
    while left > 0 {
        let want = (left as usize).min(CHUNK_SIZE_1MIB);
        reader.read_exact(&mut buf[..want])?;
        chunk_digests.extend_from_slice(&hash_1mib_chunk_sha512(&buf[..want]));
        total_chunks += 1;
        left -= want as u64;
    }
    for chunk in cd_bytes.chunks(CHUNK_SIZE_1MIB) {
        chunk_digests.extend_from_slice(&hash_1mib_chunk_sha512(chunk));
        total_chunks += 1;
    }
    for chunk in eocd_with_pre_block_offset.chunks(CHUNK_SIZE_1MIB) {
        chunk_digests.extend_from_slice(&hash_1mib_chunk_sha512(chunk));
        total_chunks += 1;
    }

    let mut top = Sha512::new();
    top.update([0x5au8]);
    top.update(total_chunks.to_le_bytes());
    top.update(&chunk_digests);
    Ok(top.finalize().into())
}

fn build_apk_signing_block_v2_v3(
    content_digest: &[u8; 32],
    keypair: &SigningKeyPair,
) -> Result<Vec<u8>> {
    let spki_der = keypair.public_key_spki_der()?;

    // Common digests & certificates encoding
    let mut digest_entry = Vec::new();
    digest_entry.extend_from_slice(&SIG_ALGO_RSA_PKCS1_V1_5_WITH_SHA256.to_le_bytes());
    digest_entry.extend_from_slice(&lp_u32(content_digest));
    let digests = lp_u32(&lp_u32(&digest_entry));

    let mut certs_inner = Vec::new();
    for c in &keypair.cert_chain_der {
        certs_inner.extend_from_slice(&lp_u32(c));
    }
    let certificates = lp_u32(&certs_inner);

    // ---- v2 Block (0x7109871a) ----
    let mut v2_attr = Vec::new();
    v2_attr.extend_from_slice(&STRIPPING_PROTECTION_ATTR_ID.to_le_bytes());
    v2_attr.extend_from_slice(&V3_SCHEME_VERSION_ID.to_le_bytes());
    let v2_additional_attrs = lp_u32(&lp_u32(&v2_attr));

    let mut v2_signed_data = Vec::new();
    v2_signed_data.extend_from_slice(&digests);
    v2_signed_data.extend_from_slice(&certificates);
    v2_signed_data.extend_from_slice(&v2_additional_attrs);

    let v2_sig = keypair.sign_rsa_sha256(&v2_signed_data);
    let mut v2_sig_entry = Vec::new();
    v2_sig_entry.extend_from_slice(&SIG_ALGO_RSA_PKCS1_V1_5_WITH_SHA256.to_le_bytes());
    v2_sig_entry.extend_from_slice(&lp_u32(&v2_sig));
    let v2_signatures = lp_u32(&lp_u32(&v2_sig_entry));

    let mut v2_signer = Vec::new();
    v2_signer.extend_from_slice(&lp_u32(&v2_signed_data));
    v2_signer.extend_from_slice(&v2_signatures);
    v2_signer.extend_from_slice(&lp_u32(&spki_der));
    let v2_block_val = lp_u32(&lp_u32(&v2_signer));

    // ---- v3 Block (0xf05368c0) ----
    let mut v3_signed_data = Vec::new();
    v3_signed_data.extend_from_slice(&digests);
    v3_signed_data.extend_from_slice(&certificates);
    v3_signed_data.extend_from_slice(&V3_MIN_SDK.to_le_bytes());
    v3_signed_data.extend_from_slice(&V3_MAX_SDK.to_le_bytes());
    v3_signed_data.extend_from_slice(&lp_u32(&[])); // empty additional attributes

    let v3_sig = keypair.sign_rsa_sha256(&v3_signed_data);
    let mut v3_sig_entry = Vec::new();
    v3_sig_entry.extend_from_slice(&SIG_ALGO_RSA_PKCS1_V1_5_WITH_SHA256.to_le_bytes());
    v3_sig_entry.extend_from_slice(&lp_u32(&v3_sig));
    let v3_signatures = lp_u32(&lp_u32(&v3_sig_entry));

    let mut v3_signer = Vec::new();
    v3_signer.extend_from_slice(&lp_u32(&v3_signed_data));
    v3_signer.extend_from_slice(&V3_MIN_SDK.to_le_bytes());
    v3_signer.extend_from_slice(&V3_MAX_SDK.to_le_bytes());
    v3_signer.extend_from_slice(&v3_signatures);
    v3_signer.extend_from_slice(&lp_u32(&spki_der));
    let v3_block_val = lp_u32(&lp_u32(&v3_signer));

    // Assemble APK Signing Block pairs + 4 KB verity padding pair.
    let mut pairs = Vec::new();
    append_sig_block_pair(&mut pairs, APK_SIG_SCHEME_V2_BLOCK_ID, &v2_block_val);
    append_sig_block_pair(&mut pairs, APK_SIG_SCHEME_V3_BLOCK_ID, &v3_block_val);

    let total_unpadded = 8 + pairs.len() + 8 + 16;
    let rem = total_unpadded % VERITY_BLOCK_ALIGN;
    if rem != 0 {
        let mut pad = VERITY_BLOCK_ALIGN - rem;
        if pad < 12 {
            pad += VERITY_BLOCK_ALIGN;
        }
        let zeros = vec![0u8; pad - 12];
        append_sig_block_pair(&mut pairs, VERITY_PADDING_BLOCK_ID, &zeros);
    }

    let size_field = (pairs.len() + 8 + 16) as u64;
    let mut block = Vec::with_capacity(8 + size_field as usize);
    block.extend_from_slice(&size_field.to_le_bytes());
    block.extend_from_slice(&pairs);
    block.extend_from_slice(&size_field.to_le_bytes());
    block.extend_from_slice(APK_SIG_BLOCK_MAGIC);
    debug_assert_eq!(block.len() % VERITY_BLOCK_ALIGN, 0);
    Ok(block)
}

fn append_sig_block_pair(out: &mut Vec<u8>, id: u32, val: &[u8]) {
    let pair_len = (4 + val.len()) as u64;
    out.extend_from_slice(&pair_len.to_le_bytes());
    out.extend_from_slice(&id.to_le_bytes());
    out.extend_from_slice(val);
}

// ============================================================================
// Pure-Rust APK Signature Scheme v1 / v2 / v3 Verifier
// ============================================================================

/// Cryptographically verifies an APK's v1, v2, and v3 signatures in pure Rust.
pub fn verify_apk_signatures(path: &Path) -> Result<ApkSignatureVerification> {
    let mut file = File::open(path).with_context(|| format!("failed to open {}", path.display()))?;
    verify_apk_signatures_reader(&mut file)
}

/// Cryptographically verifies an APK's v1, v2, and v3 signatures on any `Read + Seek` stream.
pub fn verify_apk_signatures_reader<R: Read + Seek>(
    reader: &mut R,
) -> Result<ApkSignatureVerification> {
    let layout = parse_apk_zip_layout(reader)?;
    let mut errors = Vec::new();

    let mut v1_present = false;
    let mut v1_verified = false;
    let mut v1_cert_der: Option<Vec<u8>> = None;
    let mut v1_declared_schemes: Vec<u32> = Vec::new();

    let mut v2_present = false;
    let mut v2_verified = false;
    let mut v2_cert_der: Option<Vec<u8>> = None;
    let mut v2_expects_v3 = false;

    let mut v3_present = false;
    let mut v3_verified = false;
    let mut v3_cert_der: Option<Vec<u8>> = None;

    // 1. Verify v1 (JAR) if META-INF/MANIFEST.MF + *.SF + *.RSA are present.
    if layout.find_entry("META-INF/MANIFEST.MF").is_some() {
        let sf_entries: Vec<&CentralDirEntry> = layout
            .entries
            .iter()
            .filter(|e| {
                let u = e.name.to_ascii_uppercase();
                u.starts_with("META-INF/") && !u[9..].contains('/') && u.ends_with(".SF")
            })
            .collect();
        if !sf_entries.is_empty() {
            v1_present = true;
            match verify_v1_signature(reader, &layout, sf_entries[0]) {
                Ok((cert_der, declared_schemes)) => {
                    v1_verified = true;
                    v1_cert_der = Some(cert_der);
                    v1_declared_schemes = declared_schemes;
                }
                Err(e) => errors.push(format!("v1 signature verification failed: {e:#}")),
            }
        }
    }

    // 2. Verify v2 & v3 inside APK Signing Block (if present).
    if let Some(sb) = layout.signing_block {
        reader.seek(SeekFrom::Start(sb.offset))?;
        let mut block = vec![0u8; sb.total_size as usize];
        reader.read_exact(&mut block)?;

        match parse_apk_signing_block_pairs(&block) {
            Ok(pairs) => {
                reader.seek(SeekFrom::Start(layout.eocd.cd_offset))?;
                let mut cd_bytes = vec![0u8; layout.eocd.cd_size as usize];
                reader.read_exact(&mut cd_bytes)?;

                let mut eocd_pre = layout.eocd.raw_bytes.clone();
                eocd_pre[16..20].copy_from_slice(&(sb.offset as u32).to_le_bytes());

                let mut cached_sha256: Option<[u8; 32]> = None;
                let mut cached_sha512: Option<[u8; 64]> = None;

                for (id, val) in pairs {
                    match id {
                        APK_SIG_SCHEME_V2_BLOCK_ID => {
                            v2_present = true;
                            match verify_v2_block(
                                reader,
                                sb.offset,
                                &cd_bytes,
                                &eocd_pre,
                                &val,
                                &mut cached_sha256,
                                &mut cached_sha512,
                            ) {
                                Ok((cert_der, expects_v3)) => {
                                    v2_verified = true;
                                    v2_cert_der = Some(cert_der);
                                    v2_expects_v3 = expects_v3;
                                }
                                Err(e) => {
                                    errors.push(format!("v2 signature verification failed: {e:#}"))
                                }
                            }
                        }
                        APK_SIG_SCHEME_V3_BLOCK_ID => {
                            v3_present = true;
                            match verify_v3_block(
                                reader,
                                sb.offset,
                                &cd_bytes,
                                &eocd_pre,
                                &val,
                                &mut cached_sha256,
                                &mut cached_sha512,
                            ) {
                                Ok(cert_der) => {
                                    v3_verified = true;
                                    v3_cert_der = Some(cert_der);
                                }
                                Err(e) => {
                                    errors.push(format!("v3 signature verification failed: {e:#}"))
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
            Err(e) => errors.push(format!("invalid APK Signing Block: {e:#}")),
        }
    }

    // 3. Enforce Android rollback/stripping protections and cross-scheme certificate consistency.
    if v1_verified {
        if v1_declared_schemes.contains(&2) && !v2_verified {
            errors.push("v1 X-Android-APK-Signed declares v2, but v2 signature is missing or invalid (possible stripping)".into());
        }
        if v1_declared_schemes.contains(&3) && !v3_verified {
            errors.push("v1 X-Android-APK-Signed declares v3, but v3 signature is missing or invalid (possible stripping)".into());
        }
    }
    if v2_verified && v2_expects_v3 && !v3_verified {
        errors.push("v2 stripping-protection attribute declares v3, but v3 signature is missing or invalid".into());
    }

    let primary_cert = v3_cert_der
        .as_ref()
        .or(v2_cert_der.as_ref())
        .or(v1_cert_der.as_ref());

    if let (Some(c1), Some(c2)) = (v1_cert_der.as_ref(), v2_cert_der.as_ref()) {
        if c1 != c2 {
            errors.push("v1 and v2 signing certificates do not match".into());
        }
    }
    if let (Some(c2), Some(c3)) = (v2_cert_der.as_ref(), v3_cert_der.as_ref()) {
        if c2 != c3 {
            errors.push("v2 and v3 signing certificates do not match".into());
        }
    }

    if !v1_present && !v2_present && !v3_present {
        errors.push("APK has no v1, v2, or v3 signatures".into());
    }

    let (cert_sha256, cert_subject) = match primary_cert {
        Some(c) => (
            Some(hex::encode(Sha256::digest(c))),
            parse_x509_cert_info(c).ok().map(|i| i.subject_summary),
        ),
        None => (None, None),
    };

    let any_verified = v1_verified || v2_verified || v3_verified;
    let valid = any_verified && errors.is_empty();

    Ok(ApkSignatureVerification {
        valid,
        v1_present,
        v1_verified,
        v2_present,
        v2_verified,
        v3_present,
        v3_verified,
        cert_sha256,
        cert_subject,
        errors,
    })
}

fn verify_v1_signature<R: Read + Seek>(
    reader: &mut R,
    layout: &ApkZipLayout,
    sf_entry: &CentralDirEntry,
) -> Result<(Vec<u8>, Vec<u32>)> {
    let manifest_entry = layout
        .find_entry("META-INF/MANIFEST.MF")
        .context("missing META-INF/MANIFEST.MF")?;
    let manifest_bytes = read_entry_uncompressed(reader, manifest_entry)?;
    let sf_bytes = read_entry_uncompressed(reader, sf_entry)?;

    let stem = &sf_entry.name[..sf_entry.name.len() - 3]; // strip .SF
    let sig_block_entry = layout
        .entries
        .iter()
        .find(|e| {
            e.name.eq_ignore_ascii_case(&format!("{stem}.RSA"))
                || e.name.eq_ignore_ascii_case(&format!("{stem}.DSA"))
                || e.name.eq_ignore_ascii_case(&format!("{stem}.EC"))
        })
        .with_context(|| format!("missing PKCS#7 signature block for {}", sf_entry.name))?;
    let sig_block_bytes = read_entry_uncompressed(reader, sig_block_entry)?;

    // Parse PKCS#7 SignedData and cryptographically verify `sf_bytes`.
    let (cert_der, sig_alg_oid, digest_alg_oid, sig_bytes) = parse_pkcs7_signed_data(&sig_block_bytes)?;
    let cert_info = parse_x509_cert_info(&cert_der)?;
    let pub_key = RsaPublicKey::from_public_key_der(&cert_info.spki_der)
        .context("failed to parse RSA public key from v1 X.509 certificate")?;
    let rsa_sig = RsaSignature::try_from(sig_bytes.as_slice())
        .map_err(|e| anyhow!("invalid RSA signature length in PKCS#7: {e}"))?;

    if digest_alg_oid == OID_SHA256 || sig_alg_oid == OID_SHA256_WITH_RSA {
        VerifyingKey::<Sha256>::new(pub_key)
            .verify(&sf_bytes, &rsa_sig)
            .map_err(|_| anyhow!("v1 PKCS#7 RSA-SHA256 signature verification failed"))?;
    } else if digest_alg_oid == OID_SHA512 {
        VerifyingKey::<Sha512>::new(pub_key)
            .verify(&sf_bytes, &rsa_sig)
            .map_err(|_| anyhow!("v1 PKCS#7 RSA-SHA512 signature verification failed"))?;
    } else if digest_alg_oid == OID_SHA1 {
        VerifyingKey::<Sha1>::new(pub_key)
            .verify(&sf_bytes, &rsa_sig)
            .map_err(|_| anyhow!("v1 PKCS#7 RSA-SHA1 signature verification failed"))?;
    } else {
        bail!("unsupported v1 PKCS#7 digest OID");
    }

    // Parse MANIFEST.MF and CERT.SF sections and verify digests.
    let (mf_main_raw, mf_sections) = parse_jar_sections(&manifest_bytes)?;
    let (_sf_main_raw, sf_sections) = parse_jar_sections(&sf_bytes)?;
    let sf_main_attrs = parse_jar_attr_map(&sf_bytes)?;

    let mut declared_schemes = Vec::new();
    if let Some(val) = sf_main_attrs.get("x-android-apk-signed") {
        for part in val.split(',') {
            if let Ok(num) = part.trim().parse::<u32>() {
                declared_schemes.push(num);
            }
        }
    }

    if let Some(expected_mf_b64) = sf_main_attrs.get("sha-256-digest-manifest") {
        let actual = BASE64.encode(Sha256::digest(&manifest_bytes));
        ensure!(
            actual == *expected_mf_b64,
            "v1 SHA-256-Digest-Manifest mismatch"
        );
    } else if let Some(expected_mf_b64) = sf_main_attrs.get("sha1-digest-manifest") {
        let actual = BASE64.encode(Sha1::digest(&manifest_bytes));
        ensure!(
            actual == *expected_mf_b64,
            "v1 SHA1-Digest-Manifest mismatch"
        );
    }
    if let Some(expected_main_b64) = sf_main_attrs.get("sha-256-digest-manifest-main-attributes") {
        let actual = BASE64.encode(Sha256::digest(&mf_main_raw));
        ensure!(
            actual == *expected_main_b64,
            "v1 SHA-256-Digest-Manifest-Main-Attributes mismatch"
        );
    }

    // Verify every non-signature APK entry is covered by MANIFEST.MF and CERT.SF.
    for entry in &layout.entries {
        if entry.is_dir() || is_v1_signature_entry(&entry.name) {
            continue;
        }
        let (mf_raw, mf_attrs) = mf_sections
            .get(&entry.name)
            .with_context(|| format!("APK entry '{}' missing from META-INF/MANIFEST.MF", entry.name))?;
        let (_sf_raw, sf_attrs) = sf_sections
            .get(&entry.name)
            .with_context(|| format!("APK entry '{}' missing from {}", entry.name, sf_entry.name))?;

        // Check SF entry digest matches MANIFEST.MF section bytes.
        if let Some(exp) = sf_attrs.get("sha-256-digest") {
            let act = BASE64.encode(Sha256::digest(mf_raw));
            ensure!(act == *exp, "v1 SF section SHA-256 mismatch for '{}'", entry.name);
        } else if let Some(exp) = sf_attrs.get("sha1-digest") {
            let act = BASE64.encode(Sha1::digest(mf_raw));
            ensure!(act == *exp, "v1 SF section SHA1 mismatch for '{}'", entry.name);
        } else {
            bail!("v1 SF entry '{}' missing SHA-256-Digest/SHA1-Digest", entry.name);
        }

        // Check MANIFEST.MF entry digest matches actual uncompressed entry bytes.
        if let Some(exp) = mf_attrs.get("sha-256-digest") {
            let act = BASE64.encode(digest_entry_uncompressed::<_, Sha256>(reader, entry)?);
            ensure!(act == *exp, "v1 file SHA-256 mismatch for '{}'", entry.name);
        } else if let Some(exp) = mf_attrs.get("sha1-digest") {
            let act = BASE64.encode(digest_entry_uncompressed::<_, Sha1>(reader, entry)?);
            ensure!(act == *exp, "v1 file SHA1 mismatch for '{}'", entry.name);
        } else {
            bail!("v1 MANIFEST.MF entry '{}' missing SHA-256-Digest/SHA1-Digest", entry.name);
        }
    }

    Ok((cert_der, declared_schemes))
}

fn parse_jar_attr_map(bytes: &[u8]) -> Result<HashMap<String, String>> {
    let text = std::str::from_utf8(bytes).context("JAR manifest/SF is not UTF-8")?;
    let mut map = HashMap::new();
    let mut cur_key: Option<String> = None;
    let mut cur_val = String::new();

    for raw_line in text.split('\n') {
        let line = raw_line.strip_suffix('\r').unwrap_or(raw_line);
        if line.is_empty() {
            break;
        }
        if let Some(cont) = line.strip_prefix(' ') {
            cur_val.push_str(cont);
        } else {
            if let Some(k) = cur_key.take() {
                map.insert(k, std::mem::take(&mut cur_val));
            }
            if let Some((k, v)) = line.split_once(':') {
                cur_key = Some(k.trim().to_ascii_lowercase());
                cur_val = v.trim_start().to_owned();
            }
        }
    }
    if let Some(k) = cur_key {
        map.insert(k, cur_val);
    }
    Ok(map)
}

#[allow(clippy::type_complexity)]
fn parse_jar_sections(
    bytes: &[u8],
) -> Result<(Vec<u8>, BTreeMap<String, (Vec<u8>, HashMap<String, String>)>)> {
    let text = std::str::from_utf8(bytes).context("JAR file is not valid UTF-8")?;
    let mut sections: Vec<(usize, usize)> = Vec::new();
    let mut start = 0usize;
    let mut pos = 0usize;
    let b = text.as_bytes();
    while pos < b.len() {
        let line_end = match b[pos..].iter().position(|&c| c == b'\n') {
            Some(rel) => pos + rel + 1,
            None => b.len(),
        };
        let line = &b[pos..line_end];
        let is_blank = line == b"\r\n" || line == b"\n";
        pos = line_end;
        if is_blank {
            if pos > start {
                sections.push((start, pos));
            }
            start = pos;
        }
    }
    if start < b.len() {
        sections.push((start, b.len()));
    }
    ensure!(!sections.is_empty(), "empty JAR manifest/SF");
    let main_raw = bytes[sections[0].0..sections[0].1].to_vec();
    let mut map = BTreeMap::new();
    for &(s, e) in &sections[1..] {
        let slice = &bytes[s..e];
        let attrs = parse_jar_attr_map(slice)?;
        if let Some(name) = attrs.get("name") {
            map.insert(name.clone(), (slice.to_vec(), attrs));
        }
    }
    Ok((main_raw, map))
}

fn parse_apk_signing_block_pairs(block: &[u8]) -> Result<Vec<(u32, Vec<u8>)>> {
    ensure!(block.len() >= 32, "APK Signing Block too short");
    let size_start = u64::from_le_bytes(block[0..8].try_into().unwrap());
    let size_end = u64::from_le_bytes(block[block.len() - 24..block.len() - 16].try_into().unwrap());
    ensure!(size_start == size_end, "APK Signing Block size header/footer mismatch");
    ensure!(
        size_start + 8 == block.len() as u64,
        "APK Signing Block size does not match slice length"
    );
    ensure!(
        &block[block.len() - 16..] == APK_SIG_BLOCK_MAGIC,
        "APK Signing Block magic mismatch"
    );

    let mut pairs = Vec::new();
    let mut pos = 8usize;
    let end = block.len() - 24;
    while pos < end {
        ensure!(pos + 12 <= end, "truncated APK Signing Block pair");
        let len = u64::from_le_bytes(block[pos..pos + 8].try_into().unwrap()) as usize;
        pos += 8;
        ensure!(len >= 4 && pos + len <= end, "invalid APK Signing Block pair length");
        let id = u32::from_le_bytes(block[pos..pos + 4].try_into().unwrap());
        let val = block[pos + 4..pos + len].to_vec();
        pairs.push((id, val));
        pos += len;
    }
    Ok(pairs)
}

struct LpCursor<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> LpCursor<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    fn remaining(&self) -> usize {
        self.buf.len().saturating_sub(self.pos)
    }
    fn read_u32(&mut self) -> Result<u32> {
        ensure!(self.pos + 4 <= self.buf.len(), "unexpected EOF reading u32");
        let v = u32::from_le_bytes(self.buf[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        Ok(v)
    }
    fn read_lp(&mut self) -> Result<&'a [u8]> {
        let len = self.read_u32()? as usize;
        ensure!(self.pos + len <= self.buf.len(), "length-prefixed slice exceeds buffer");
        let s = &self.buf[self.pos..self.pos + len];
        self.pos += len;
        Ok(s)
    }
}

fn verify_v2_block<R: Read + Seek>(
    reader: &mut R,
    zip_entries_end: u64,
    cd_bytes: &[u8],
    eocd_pre: &[u8],
    block_val: &[u8],
    cached_sha256: &mut Option<[u8; 32]>,
    cached_sha512: &mut Option<[u8; 64]>,
) -> Result<(Vec<u8>, bool)> {
    let mut root = LpCursor::new(block_val);
    let signers_buf = root.read_lp()?;
    let mut signers = LpCursor::new(signers_buf);
    ensure!(signers.remaining() > 0, "v2 block contains no signers");

    let signer_buf = signers.read_lp()?;
    let mut s = LpCursor::new(signer_buf);
    let signed_data = s.read_lp()?;
    let signatures_buf = s.read_lp()?;
    let public_key_der = s.read_lp()?;

    let sig_algo = verify_v2_v3_signatures(signed_data, signatures_buf, public_key_der)?;

    let mut sd = LpCursor::new(signed_data);
    let digests_buf = sd.read_lp()?;
    let certs_buf = sd.read_lp()?;
    let attrs_buf = sd.read_lp()?;

    verify_v2_v3_digest(
        reader,
        zip_entries_end,
        cd_bytes,
        eocd_pre,
        digests_buf,
        sig_algo,
        cached_sha256,
        cached_sha512,
    )?;

    let mut certs = LpCursor::new(certs_buf);
    let leaf_cert = certs.read_lp()?.to_vec();
    let cert_info = parse_x509_cert_info(&leaf_cert)?;
    ensure!(
        cert_info.spki_der == public_key_der,
        "v2 signer public key does not match leaf certificate SPKI"
    );

    let mut expects_v3 = false;
    let mut attrs = LpCursor::new(attrs_buf);
    while attrs.remaining() > 0 {
        let attr = attrs.read_lp()?;
        let mut a = LpCursor::new(attr);
        if a.remaining() >= 8 {
            let id = a.read_u32()?;
            if id == STRIPPING_PROTECTION_ATTR_ID {
                let target_ver = a.read_u32()?;
                if target_ver == V3_SCHEME_VERSION_ID {
                    expects_v3 = true;
                }
            }
        }
    }

    Ok((leaf_cert, expects_v3))
}

fn verify_v3_block<R: Read + Seek>(
    reader: &mut R,
    zip_entries_end: u64,
    cd_bytes: &[u8],
    eocd_pre: &[u8],
    block_val: &[u8],
    cached_sha256: &mut Option<[u8; 32]>,
    cached_sha512: &mut Option<[u8; 64]>,
) -> Result<Vec<u8>> {
    let mut root = LpCursor::new(block_val);
    let signers_buf = root.read_lp()?;
    let mut signers = LpCursor::new(signers_buf);
    ensure!(signers.remaining() > 0, "v3 block contains no signers");

    let signer_buf = signers.read_lp()?;
    let mut s = LpCursor::new(signer_buf);
    let signed_data = s.read_lp()?;
    let min_sdk_outer = s.read_u32()?;
    let max_sdk_outer = s.read_u32()?;
    let signatures_buf = s.read_lp()?;
    let public_key_der = s.read_lp()?;

    let sig_algo = verify_v2_v3_signatures(signed_data, signatures_buf, public_key_der)?;

    let mut sd = LpCursor::new(signed_data);
    let digests_buf = sd.read_lp()?;
    let certs_buf = sd.read_lp()?;
    let min_sdk_inner = sd.read_u32()?;
    let max_sdk_inner = sd.read_u32()?;
    let _attrs_buf = sd.read_lp()?;

    ensure!(
        min_sdk_outer == min_sdk_inner && max_sdk_outer == max_sdk_inner,
        "v3 signer minSdk/maxSdk mismatch between outer signer and signed_data"
    );

    verify_v2_v3_digest(
        reader,
        zip_entries_end,
        cd_bytes,
        eocd_pre,
        digests_buf,
        sig_algo,
        cached_sha256,
        cached_sha512,
    )?;

    let mut certs = LpCursor::new(certs_buf);
    let leaf_cert = certs.read_lp()?.to_vec();
    let cert_info = parse_x509_cert_info(&leaf_cert)?;
    ensure!(
        cert_info.spki_der == public_key_der,
        "v3 signer public key does not match leaf certificate SPKI"
    );
    Ok(leaf_cert)
}

fn verify_v2_v3_signatures(
    signed_data: &[u8],
    signatures_buf: &[u8],
    public_key_der: &[u8],
) -> Result<u32> {
    let pub_key = RsaPublicKey::from_public_key_der(public_key_der)
        .context("failed to parse RSA SPKI public key")?;
    let mut sigs = LpCursor::new(signatures_buf);
    let mut verified_algo = None;

    while sigs.remaining() > 0 {
        let entry = sigs.read_lp()?;
        let mut e = LpCursor::new(entry);
        let algo = e.read_u32()?;
        let sig_bytes = e.read_lp()?;
        let rsa_sig = RsaSignature::try_from(sig_bytes)
            .map_err(|err| anyhow!("invalid RSA signature length: {err}"))?;
        match algo {
            SIG_ALGO_RSA_PKCS1_V1_5_WITH_SHA256 => {
                VerifyingKey::<Sha256>::new(pub_key.clone())
                    .verify(signed_data, &rsa_sig)
                    .map_err(|_| anyhow!("RSA-PKCS1-v1.5-SHA256 signature verification failed"))?;
                verified_algo = Some(algo);
            }
            SIG_ALGO_RSA_PKCS1_V1_5_WITH_SHA512 => {
                VerifyingKey::<Sha512>::new(pub_key.clone())
                    .verify(signed_data, &rsa_sig)
                    .map_err(|_| anyhow!("RSA-PKCS1-v1.5-SHA512 signature verification failed"))?;
                verified_algo = Some(algo);
            }
            _ => {}
        }
    }

    verified_algo.ok_or_else(|| anyhow!("no supported RSA signature algorithm found in signer"))
}

#[allow(clippy::too_many_arguments)]
fn verify_v2_v3_digest<R: Read + Seek>(
    reader: &mut R,
    zip_entries_end: u64,
    cd_bytes: &[u8],
    eocd_pre: &[u8],
    digests_buf: &[u8],
    sig_algo: u32,
    cached_sha256: &mut Option<[u8; 32]>,
    cached_sha512: &mut Option<[u8; 64]>,
) -> Result<()> {
    let mut digests = LpCursor::new(digests_buf);
    while digests.remaining() > 0 {
        let entry = digests.read_lp()?;
        let mut e = LpCursor::new(entry);
        let algo = e.read_u32()?;
        let digest = e.read_lp()?;
        if algo == sig_algo {
            match algo {
                SIG_ALGO_RSA_PKCS1_V1_5_WITH_SHA256 => {
                    let actual = match cached_sha256 {
                        Some(d) => *d,
                        None => {
                            let d = compute_apk_v2_v3_sha256_digest(
                                reader,
                                zip_entries_end,
                                cd_bytes,
                                eocd_pre,
                            )?;
                            *cached_sha256 = Some(d);
                            d
                        }
                    };
                    ensure!(
                        digest == actual.as_slice(),
                        "APK content SHA-256 chunked digest mismatch (APK tampered)"
                    );
                    return Ok(());
                }
                SIG_ALGO_RSA_PKCS1_V1_5_WITH_SHA512 => {
                    let actual = match cached_sha512 {
                        Some(d) => *d,
                        None => {
                            let d = compute_apk_v2_v3_sha512_digest(
                                reader,
                                zip_entries_end,
                                cd_bytes,
                                eocd_pre,
                            )?;
                            *cached_sha512 = Some(d);
                            d
                        }
                    };
                    ensure!(
                        digest == actual.as_slice(),
                        "APK content SHA-512 chunked digest mismatch (APK tampered)"
                    );
                    return Ok(());
                }
                _ => {}
            }
        }
    }
    bail!("no matching content digest found for verified signature algorithm 0x{sig_algo:04x}")
}

// ============================================================================
// Minimal Pure-Rust ASN.1 DER / X.509 / PKCS#7 / PEM Helpers
// ============================================================================

fn der_tlv(tag: u8, value: &[u8]) -> Vec<u8> {
    let len = value.len();
    let mut out = Vec::with_capacity(1 + 5 + len);
    out.push(tag);
    if len < 0x80 {
        out.push(len as u8);
    } else {
        let len_bytes = (len as u64).to_be_bytes();
        let first_nz = len_bytes
            .iter()
            .position(|&b| b != 0)
            .unwrap_or(len_bytes.len() - 1);
        let count = len_bytes.len() - first_nz;
        out.push(0x80 | (count as u8));
        out.extend_from_slice(&len_bytes[first_nz..]);
    }
    out.extend_from_slice(value);
    out
}

fn der_concat(parts: &[Vec<u8>]) -> Vec<u8> {
    let total: usize = parts.iter().map(Vec::len).sum();
    let mut out = Vec::with_capacity(total);
    for p in parts {
        out.extend_from_slice(p);
    }
    out
}

fn der_sequence(parts: &[Vec<u8>]) -> Vec<u8> {
    der_tlv(0x30, &der_concat(parts))
}

fn der_set(parts: &[Vec<u8>]) -> Vec<u8> {
    der_tlv(0x31, &der_concat(parts))
}

fn der_oid(oid_bytes: &[u8]) -> Vec<u8> {
    der_tlv(0x06, oid_bytes)
}

fn der_null() -> Vec<u8> {
    vec![0x05, 0x00]
}

fn der_integer_u8(v: u8) -> Vec<u8> {
    der_integer_unsigned(&[v])
}

fn der_integer_unsigned(bytes: &[u8]) -> Vec<u8> {
    let trimmed = match bytes.iter().position(|&b| b != 0) {
        Some(idx) => &bytes[idx..],
        None => &[0u8],
    };
    let mut val = Vec::with_capacity(trimmed.len() + 1);
    if (trimmed[0] & 0x80) != 0 {
        val.push(0x00);
    }
    val.extend_from_slice(trimmed);
    der_tlv(0x02, &val)
}

fn der_octet_string(bytes: &[u8]) -> Vec<u8> {
    der_tlv(0x04, bytes)
}

fn der_bit_string(bytes: &[u8]) -> Vec<u8> {
    let mut val = Vec::with_capacity(bytes.len() + 1);
    val.push(0u8); // 0 unused bits
    val.extend_from_slice(bytes);
    der_tlv(0x03, &val)
}

fn der_rdn_attr(oid: &[u8], value: &str, printable: bool) -> Vec<u8> {
    let tag = if printable { 0x13 } else { 0x0c };
    der_set(&[der_sequence(&[der_oid(oid), der_tlv(tag, value.as_bytes())])])
}

fn build_self_signed_x509_cert(
    private_key: &RsaPrivateKey,
    cn: &str,
    org: &str,
    country: &str,
) -> Result<Vec<u8>> {
    let mut serial = [0u8; 16];
    OsRng.fill_bytes(&mut serial);
    serial[0] = (serial[0] & 0x7f) | 0x40; // Positive 16-byte serial number

    let spki_der = RsaPublicKey::from(private_key)
        .to_public_key_der()
        .context("failed to encode SPKI DER")?
        .as_bytes()
        .to_vec();

    let sig_alg = der_sequence(&[der_oid(OID_SHA256_WITH_RSA), der_null()]);
    let name = der_sequence(&[
        der_rdn_attr(OID_COMMON_NAME, cn, false),
        der_rdn_attr(OID_ORGANIZATION, org, false),
        der_rdn_attr(OID_COUNTRY, country, true),
    ]);
    // RFC 5280: dates through 2049 use UTCTime (0x17); 2050+ use GeneralizedTime (0x18).
    let validity = der_sequence(&[
        der_tlv(0x17, b"240101000000Z"),
        der_tlv(0x18, b"20540101000000Z"),
    ]);

    let tbs_cert = der_sequence(&[
        der_tlv(0xa0, &der_integer_u8(2)), // v3
        der_integer_unsigned(&serial),
        sig_alg.clone(),
        name.clone(), // issuer
        validity,
        name, // subject
        spki_der,
    ]);

    let signing_key = SigningKey::<Sha256>::new(private_key.clone());
    let sig_bytes = signing_key.sign(&tbs_cert).to_bytes();

    Ok(der_sequence(&[
        tbs_cert,
        sig_alg,
        der_bit_string(&sig_bytes),
    ]))
}

struct DerTlv<'a> {
    tag: u8,
    raw: &'a [u8],
    value: &'a [u8],
}

fn read_der_tlv<'a>(buf: &'a [u8], pos: &mut usize) -> Result<DerTlv<'a>> {
    ensure!(*pos + 2 <= buf.len(), "truncated DER TLV header");
    let start = *pos;
    let tag = buf[*pos];
    *pos += 1;
    let b0 = buf[*pos];
    *pos += 1;
    let len = if (b0 & 0x80) == 0 {
        b0 as usize
    } else {
        let n = (b0 & 0x7f) as usize;
        ensure!(n > 0 && n <= 4 && *pos + n <= buf.len(), "invalid DER length encoding");
        let mut v = 0usize;
        for _ in 0..n {
            v = (v << 8) | (buf[*pos] as usize);
            *pos += 1;
        }
        v
    };
    ensure!(*pos + len <= buf.len(), "DER value length exceeds buffer");
    let value = &buf[*pos..*pos + len];
    *pos += len;
    Ok(DerTlv {
        tag,
        raw: &buf[start..*pos],
        value,
    })
}

struct X509CertInfo {
    serial_tlv: Vec<u8>,
    issuer_tlv: Vec<u8>,
    spki_der: Vec<u8>,
    subject_summary: String,
}

fn parse_x509_cert_info(cert_der: &[u8]) -> Result<X509CertInfo> {
    let mut p = 0usize;
    let outer = read_der_tlv(cert_der, &mut p)?;
    ensure!(outer.tag == 0x30, "X.509 cert outer tag must be SEQUENCE");
    let mut cp = 0usize;
    let tbs = read_der_tlv(outer.value, &mut cp)?;
    ensure!(tbs.tag == 0x30, "X.509 tbsCertificate must be SEQUENCE");

    let mut tp = 0usize;
    let mut next = read_der_tlv(tbs.value, &mut tp)?;
    if next.tag == 0xa0 {
        // Optional explicit version [0]
        next = read_der_tlv(tbs.value, &mut tp)?;
    }
    ensure!(next.tag == 0x02, "X.509 serialNumber must be INTEGER");
    let serial_tlv = next.raw.to_vec();

    let _sig_alg = read_der_tlv(tbs.value, &mut tp)?;
    let issuer = read_der_tlv(tbs.value, &mut tp)?;
    ensure!(issuer.tag == 0x30, "X.509 issuer must be SEQUENCE");
    let issuer_tlv = issuer.raw.to_vec();

    let _validity = read_der_tlv(tbs.value, &mut tp)?;
    let subject = read_der_tlv(tbs.value, &mut tp)?;
    ensure!(subject.tag == 0x30, "X.509 subject must be SEQUENCE");
    let subject_summary = parse_dn_summary(subject.value);

    let spki = read_der_tlv(tbs.value, &mut tp)?;
    ensure!(spki.tag == 0x30, "X.509 SubjectPublicKeyInfo must be SEQUENCE");
    let spki_der = spki.raw.to_vec();

    Ok(X509CertInfo {
        serial_tlv,
        issuer_tlv,
        spki_der,
        subject_summary,
    })
}

fn parse_dn_summary(dn_seq_val: &[u8]) -> String {
    let mut parts = Vec::new();
    let mut p = 0usize;
    while p < dn_seq_val.len() {
        let Ok(rdn) = read_der_tlv(dn_seq_val, &mut p) else {
            break;
        };
        let mut rp = 0usize;
        if let Ok(atv) = read_der_tlv(rdn.value, &mut rp) {
            let mut ap = 0usize;
            if let (Ok(oid), Ok(val)) = (
                read_der_tlv(atv.value, &mut ap),
                read_der_tlv(atv.value, &mut ap),
            ) {
                let key = match oid.value {
                    OID_COMMON_NAME => "CN",
                    OID_ORGANIZATION => "O",
                    OID_COUNTRY => "C",
                    _ => continue,
                };
                if let Ok(s) = std::str::from_utf8(val.value) {
                    parts.push(format!("{key}={s}"));
                }
            }
        }
    }
    if parts.is_empty() {
        "X.509 Subject".to_owned()
    } else {
        parts.join(", ")
    }
}

/// Parses a PKCS#7 / CMS `SignedData` block (`META-INF/*.RSA`) and returns
/// `(leaf_cert_der, sig_alg_oid, digest_alg_oid, encrypted_digest_bytes)`.
fn parse_pkcs7_signed_data(bytes: &[u8]) -> Result<(Vec<u8>, Vec<u8>, Vec<u8>, Vec<u8>)> {
    let mut p = 0usize;
    let ci = read_der_tlv(bytes, &mut p)?;
    ensure!(ci.tag == 0x30, "PKCS#7 ContentInfo must be SEQUENCE");

    let mut cip = 0usize;
    let oid = read_der_tlv(ci.value, &mut cip)?;
    ensure!(
        oid.tag == 0x06 && oid.value == OID_PKCS7_SIGNED_DATA,
        "PKCS#7 ContentInfo is not SignedData"
    );
    let content = read_der_tlv(ci.value, &mut cip)?;
    ensure!(content.tag == 0xa0, "PKCS#7 SignedData content must be [0]");

    let mut sp = 0usize;
    let sd = read_der_tlv(content.value, &mut sp)?;
    ensure!(sd.tag == 0x30, "PKCS#7 SignedData must be SEQUENCE");

    let mut sdp = 0usize;
    let _version = read_der_tlv(sd.value, &mut sdp)?;
    let _digest_algs = read_der_tlv(sd.value, &mut sdp)?;
    let _encap = read_der_tlv(sd.value, &mut sdp)?;

    let mut cert_der: Option<Vec<u8>> = None;
    let mut signer_infos_val: Option<&[u8]> = None;

    while sdp < sd.value.len() {
        let field = read_der_tlv(sd.value, &mut sdp)?;
        match field.tag {
            0xa0 => {
                // certificates [0] IMPLICIT
                let mut cp = 0usize;
                if let Ok(c) = read_der_tlv(field.value, &mut cp) {
                    cert_der = Some(c.raw.to_vec());
                }
            }
            0xa1 => {
                // crls [1] IMPLICIT — skip
            }
            0x31 => {
                signer_infos_val = Some(field.value);
            }
            _ => {}
        }
    }

    let cert_der = cert_der.context("PKCS#7 SignedData missing X.509 certificate")?;
    let signer_infos_val = signer_infos_val.context("PKCS#7 SignedData missing signerInfos")?;

    let mut sip = 0usize;
    let signer_info = read_der_tlv(signer_infos_val, &mut sip)?;
    ensure!(signer_info.tag == 0x30, "PKCS#7 SignerInfo must be SEQUENCE");

    let mut ip = 0usize;
    let _si_ver = read_der_tlv(signer_info.value, &mut ip)?;
    let _sid = read_der_tlv(signer_info.value, &mut ip)?;
    let digest_alg_seq = read_der_tlv(signer_info.value, &mut ip)?;
    let mut dap = 0usize;
    let digest_oid = read_der_tlv(digest_alg_seq.value, &mut dap)?.value.to_vec();

    // Optional authenticatedAttributes [0] (0xa0)
    let next = read_der_tlv(signer_info.value, &mut ip)?;
    if next.tag == 0xa0 {
        bail!("PKCS#7 SignerInfo authenticatedAttributes verification not supported by lightweight verifier");
    }
    let sig_alg_seq = next;
    let mut sap = 0usize;
    let sig_oid = read_der_tlv(sig_alg_seq.value, &mut sap)?.value.to_vec();

    let sig_octets = read_der_tlv(signer_info.value, &mut ip)?;
    ensure!(
        sig_octets.tag == 0x04,
        "PKCS#7 encryptedDigest must be OCTET STRING"
    );

    Ok((cert_der, sig_oid, digest_oid, sig_octets.value.to_vec()))
}

fn parse_pem_blocks(text: &str) -> Result<Vec<(String, Vec<u8>)>> {
    let mut blocks = Vec::new();
    let mut cur_label: Option<String> = None;
    let mut cur_b64 = String::new();

    for line in text.lines() {
        let trimmed = line.trim();
        if let Some(rest) = trimmed.strip_prefix("-----BEGIN ") {
            if let Some(label) = rest.strip_suffix("-----") {
                cur_label = Some(label.to_owned());
                cur_b64.clear();
                continue;
            }
        }
        if let Some(rest) = trimmed.strip_prefix("-----END ") {
            if let Some(label) = rest.strip_suffix("-----") {
                if let Some(open_label) = cur_label.take() {
                    ensure!(
                        open_label == label,
                        "mismatched PEM END label '{label}' (expected '{open_label}')"
                    );
                    let der = BASE64
                        .decode(&cur_b64)
                        .with_context(|| format!("invalid base64 in PEM block '{label}'"))?;
                    blocks.push((label.to_owned(), der));
                    cur_b64.clear();
                }
                continue;
            }
        }
        if cur_label.is_some() {
            cur_b64.push_str(trimmed);
        }
    }

    Ok(blocks)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::align::{verify_apk_alignment_reader, ALIGN_16K, ALIGN_4B};

    fn build_signed_test_apk(keypair: &SigningKeyPair) -> Vec<u8> {
        let mut cursor = Cursor::new(Vec::new());
        let mut writer = StreamingApkZipWriter::new(&mut cursor).unwrap();
        writer
            .write_deflated_bytes(
                "AndroidManifest.xml",
                b"synthetic manifest payload with enough bytes to test compression",
            )
            .unwrap();
        writer
            .write_stored_bytes("resources.arsc", b"ARSC_TABLE_BYTES", ALIGN_4B)
            .unwrap();
        writer
            .write_stored_bytes(
                "lib/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so",
                &vec![0xABu8; 4096],
                ALIGN_16K,
            )
            .unwrap();
        // Long entry path > 72 chars to exercise JAR manifest line wrapping!
        writer
            .write_stored_bytes(
                "assets/openxr/1/api_layers/implicit.d/XrApiLayer_alvr_hand_emulation.json",
                br#"{"file_format_version":"1.0.0"}"#,
                ALIGN_4B,
            )
            .unwrap();

        let (_, summary) =
            finalize_signed_apk_stream(writer, keypair, "unit-test".to_owned()).unwrap();
        assert!(summary.v1_signed && summary.v2_signed && summary.v3_signed);
        cursor.into_inner()
    }

    #[test]
    fn pure_rust_v1_v2_v3_sign_verify_and_alignment_roundtrip() {
        let kp = SigningKeyPair::generate_ephemeral_debug().unwrap();
        let apk = build_signed_test_apk(&kp);

        // PEM export/reload roundtrip must preserve exact certificate fingerprint
        let pem = kp.to_pem_bundle().unwrap();
        let kp2 = SigningKeyPair::from_pem_bytes(pem.as_bytes()).unwrap();
        assert_eq!(kp.cert_sha256_hex(), kp2.cert_sha256_hex());

        // Alignment check passes 16KB + 4B
        let align = verify_apk_alignment_reader(&mut Cursor::new(&apk)).unwrap();
        assert!(align.is_fully_aligned_16k());

        // Signature verification passes all three schemes: v1 + v2 + v3!
        let ver = verify_apk_signatures_reader(&mut Cursor::new(&apk)).unwrap();
        assert!(ver.valid, "errors: {:?}", ver.errors);
        assert!(ver.v1_verified);
        assert!(ver.v2_verified);
        assert!(ver.v3_verified);
        assert_eq!(ver.cert_sha256.as_deref(), Some(kp.cert_sha256_hex().as_str()));
        assert!(ver.cert_subject.unwrap().contains("CN=ALVR Quest Hand Layer Debug"));
    }

    #[test]
    fn tamper_detection_catches_payload_and_central_dir_corruption() {
        let kp = SigningKeyPair::generate_ephemeral_debug().unwrap();
        let apk = build_signed_test_apk(&kp);

        // 1. Flip 1 byte inside the .so payload
        let layout = parse_apk_zip_layout(&mut Cursor::new(&apk)).unwrap();
        let so = layout
            .find_entry("lib/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so")
            .unwrap();
        let mut tampered_so = apk.clone();
        tampered_so[so.data_offset as usize + 10] ^= 0x01;
        let ver = verify_apk_signatures_reader(&mut Cursor::new(&tampered_so)).unwrap();
        assert!(!ver.valid);
        assert!(!ver.v1_verified);
        assert!(!ver.v2_verified);
        assert!(!ver.v3_verified);

        // 2. Flip 1 byte inside Central Directory (mod_time) — v2/v3 must catch immediately!
        let mut tampered_cd = apk.clone();
        tampered_cd[layout.eocd.cd_offset as usize + 12] ^= 0x01;
        let ver_cd = verify_apk_signatures_reader(&mut Cursor::new(&tampered_cd)).unwrap();
        assert!(!ver_cd.valid);
        assert!(!ver_cd.v2_verified);
        assert!(!ver_cd.v3_verified);
    }

    #[test]
    fn jks_and_p12_roundtrip_keystores_load_cleanly() {
        let kp = SigningKeyPair::generate_ephemeral_debug().unwrap();
        let pkcs8_der = kp.private_key.to_pkcs8_der().unwrap();
        let cert_der = kp.leaf_cert_der().to_vec();

        // JKS roundtrip
        let mut jks_ks = jks::KeyStore::new();
        jks_ks
            .set_private_key_entry(
                "myalias",
                jks::PrivateKeyEntry {
                    creation_time: std::time::SystemTime::now(),
                    private_key: pkcs8_der.as_bytes().to_vec(),
                    certificate_chain: vec![jks::Certificate {
                        cert_type: "X.509".to_owned(),
                        content: cert_der.clone(),
                    }],
                },
                b"secret123",
            )
            .unwrap();
        let mut jks_bytes = Vec::new();
        jks_ks.store(&mut jks_bytes, b"secret123").unwrap();

        let loaded_jks =
            SigningKeyPair::from_keystore_bytes(&jks_bytes, Some("secret123"), Some("myalias"))
                .unwrap();
        assert_eq!(loaded_jks.cert_sha256_hex(), kp.cert_sha256_hex());

        // PKCS#12 roundtrip
        let mut p12_ks = P12KeyStore::new();
        let p12_key = p12_keystore::PrivateKey::from_der(pkcs8_der.as_bytes()).unwrap();
        let p12_cert = p12_keystore::Certificate::from_der(&cert_der).unwrap();
        let chain = p12_keystore::PrivateKeyChain::new(
            b"keyid".as_slice(),
            p12_key,
            vec![p12_cert],
        );
        p12_ks.add_entry("alvrdebug", KeyStoreEntry::PrivateKeyChain(chain));
        let p12_bytes = p12_ks.writer("android").write().unwrap();

        let loaded_p12 =
            SigningKeyPair::from_keystore_bytes(&p12_bytes, Some("android"), Some("alvrdebug"))
                .unwrap();
        assert_eq!(loaded_p12.cert_sha256_hex(), kp.cert_sha256_hex());
    }
}
