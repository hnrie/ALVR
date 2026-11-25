# Quest Standalone OpenXR Hand-to-Touch-Controller Implicit API Layer (`quest_openxr_layer`)

A standalone Android (`arm64-v8a`) OpenXR 1.0/1.1 Implicit API Layer (`XR_APILAYER_ALVR_hand_emulation`) that ports ALVR's hand-tracking gesture bindings (`alvr/server_core/src/hand_gestures.rs`) and hand-to-controller pose offsets (`alvr/session/src/settings.rs`, `alvr/server_openvr/src/tracking.rs`) directly onto Meta Quest OpenXR applications so you can play standalone Quest OpenXR apps using only optical hand tracking.

> **Scope & Policy Notice:** This project strictly implements a standard Khronos OpenXR Implicit API Layer plus `.aar` and unprotected APK bundling, 16 KB alignment, and v1/v2/v3 signing tools. It contains **zero DRM, entitlement, or anti-cheat bypasses** and is intended exclusively for unprotected non-Meta-Store OpenXR APKs, sideloaded builds, or apps you build yourself.

---

## Self-Contained Desktop Patcher GUI (Windows / macOS / Linux)

The `alvr-quest-patcher` desktop app embeds everything in a **single binary with zero extra installs** (no Android SDK, Java, `keytool`, `zipalign`, or `apksigner` required):

- Embedded prebuilt 16 KB-aligned Quest `arm64-v8a` `libXrApiLayer_alvr_hand_emulation.so`
- Embedded OpenXR implicit layer manifest `XrApiLayer_alvr_hand_emulation.json` and default gesture/pose config
- Pure-Rust binary `AndroidManifest.xml` (AXML) patcher & safe side-by-side package renamer (qualifies relative Java component class names)
- Pure-Rust streaming 16 KB APK aligner (`zipalign -P 16 4` equivalent)
- Pure-Rust APK Signature Scheme v1 + v2 + v3 signer & cryptographic verifier (supports persistent auto-generated debug keys, `.p12`/`.pfx`, `.jks`/`.keystore`, and `.pem`)

Simply drag-and-drop an unprotected Quest OpenXR `.apk` into the window and click **Patch, 16 KB Align & Sign APK**.

---

## Architecture

```text
Quest OpenXR App (Unity / Unreal / Godot / Native OpenXR 1.0 or 1.1)
  │
  ▼
Khronos / Meta Android OpenXR Loader (libopenxr_loader.so)
  │  Auto-discovers assets/openxr/1/api_layers/implicit.d/XrApiLayer_alvr_hand_emulation.json
  │  Loads lib/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so
  ▼
XR_APILAYER_ALVR_hand_emulation (crates/xr_api_layer)
  ├── Injects XR_EXT_hand_tracking (+ optional XR_FB_hand_tracking_aim)
  ├── Arbitrates per-hand: physical Touch controller active → transparent passthrough;
  │                        inactive + optical hand active → ALVR gesture emulation
  ├── Runs alvr_hand_core (HandGestureEngine + PoseSynthesizer + 9 Quest/Simple profiles)
  └── Serves emulated Oculus/Meta Touch actions, grip/aim/palm poses, velocities & haptics
  ▼
Meta Quest OpenXR Runtime
```

### Supported OpenXR Interaction Profiles (9 Profiles, OpenXR 1.0 & 1.1)
1. `/interaction_profiles/meta/touch_plus_controller` (OpenXR 1.1)
2. `/interaction_profiles/meta/touch_controller_plus` (`XR_META_touch_controller_plus`)
3. `/interaction_profiles/meta/touch_pro_controller` (OpenXR 1.1)
4. `/interaction_profiles/facebook/touch_controller_pro` (`XR_FB_touch_controller_pro`)
5. `/interaction_profiles/meta/touch_controller_quest_2` (OpenXR 1.1)
6. `/interaction_profiles/meta/touch_controller_quest_1_rift_s` (OpenXR 1.1)
7. `/interaction_profiles/meta/touch_controller_rift_cv1` (OpenXR 1.1)
8. `/interaction_profiles/oculus/touch_controller` (OpenXR 1.0)
9. `/interaction_profiles/khr/simple_controller` (OpenXR 1.0)

---

## ALVR Hand Gesture Bindings

| Gesture | Left Hand Emulated Input | Right Hand Emulated Input |
|---|---|---|
| **Thumb + Index Pinch** | Trigger value / touch / click (and Simple `select/click`) | Trigger value / touch / click (and Simple `select/click`) |
| **Thumb + Middle Pinch** | **Y** click / touch | **B** click / touch |
| **Thumb + Ring Pinch** | **X** click / touch | **A** click / touch |
| **Thumb + Pinky Pinch** | **Menu** click | *(unused, matches ALVR)* |
| **Middle + Ring + Pinky Curl** | **Grip / Squeeze** analog value & click | **Grip / Squeeze** analog value & click |
| **Thumb Curl into Palm** | **Thumbstick Click** | **Thumbstick Click** |
| **Curled Fist + Thumb on Index Side** | **Thumbstick X/Y** & touch (40% default deadzone) | **Thumbstick X/Y** & touch (40% default deadzone) |

---

## Building from Source

```bash
cd quest_openxr_layer

# 1. Build Quest arm64-v8a .so, pure-Rust alvr-xr-packager CLI, and dist/ artifacts (.so, .json, .aar)
./scripts/build_android.sh

# 2. Build the self-contained desktop GUI (embeds dist/arm64-v8a/*.so, manifest, default config, aligner & signer)
cargo build --release -p quest_patcher_gui

# 3. Verify embedded binary + pure-Rust roundtrip headlessly
./target/release/alvr-quest-patcher --self-test
```

Distributable artifacts are written to `quest_openxr_layer/dist/`:
- `dist/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so` (16 KB ELF aligned, `DT_NEEDED` limited to `libc/libdl/libm/liblog`)
- `dist/XrApiLayer_alvr_hand_emulation.json`
- `dist/alvr-hand-controller-layer.aar`
- `dist/alvr_hand_controller.default.json`

---

## Integrating into Quest OpenXR Apps

### Option A — Self-Contained Desktop GUI (`alvr-quest-patcher`)
Download the release archive for Windows, macOS, or Linux, launch `alvr-quest-patcher`, drop in your unprotected OpenXR `.apk`, and click **Patch, 16 KB Align & Sign APK**.

### Option B — Pure-Rust CLI (`alvr-xr-packager`, Zero External Tools)
```bash
./target/release/alvr-xr-packager patch-apk \
  --input  /path/to/unprotected_game.apk \
  --output /path/to/unprotected_game_handtracking.apk \
  --so     ./dist/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so \
  --manifest ./dist/XrApiLayer_alvr_hand_emulation.json \
  --config ./dist/alvr_hand_controller.default.json

# Verify the resulting APK (manifest entries, 16 KB alignment, ELF, v1/v2/v3 signatures):
./target/release/alvr-xr-packager inspect-apk \
  --input /path/to/unprotected_game_handtracking.apk

# Install to Quest over ADB:
adb install -r /path/to/unprotected_game_handtracking.apk
```

*(Pass `--keystore <file.p12|.jks|.pem> --ks-pass <pass> --ks-alias <alias>` to sign with your own keystore; omit them to automatically create and reuse a persistent local RSA-2048 debug keypair in your user config directory.)*

### Option C — Drop-in Android `.aar` (Unity / Unreal / Godot / Gradle)
Copy `dist/alvr-hand-controller-layer.aar` into your Android library folder (e.g. `Assets/Plugins/Android/` in Unity or `libs/` in Gradle). Android Manifest Merger automatically merges the Quest/AOSP hand-tracking permissions, `oculus.software.handtracking` (`required="false"`), hand-tracking `meta-data`, native library, and implicit layer manifest.

---

## Runtime Tuning & Diagnostics on Quest

Configuration precedence on Quest:
1. `ALVR_HAND_LAYER_CONFIG` environment variable (if set)
2. `/sdcard/Android/data/<package.name>/files/alvr_hand_controller.json` (app external files dir)
3. `/data/user/0/<package.name>/files/alvr_hand_controller.json` (app internal files dir)
4. APK bundled `assets/alvr_hand_controller.json` (embedded by the patcher/GUI)
5. Built-in ALVR defaults

You can override gesture distances, delays, joystick deadzone/offsets, or grip/aim pose offsets without rebuilding the APK:

```bash
adb push dist/alvr_hand_controller.default.json \
  /sdcard/Android/data/<package.name>/files/alvr_hand_controller.json
```

To disable the implicit layer at runtime, set `DISABLE_XR_APILAYER_ALVR_HAND_EMULATION=1`.
Inspect layer logs on Quest via:

```bash
adb logcat -s ALVRHandLayer:V OpenXR-Loader:V
```

---

## Running Tests

```bash
# Unit tests for alvr_hand_core, xr_api_layer, and android_packager
cargo test -p alvr_hand_core -p xr_api_layer -p android_packager

# Full opaque-box 3-tier E2E suite (direct dlopen + real Khronos OpenXR 1.1.54 loader + official Android SDK aapt/zipalign/apksigner cross-verification)
./e2e_suite/run_e2e.sh
```
