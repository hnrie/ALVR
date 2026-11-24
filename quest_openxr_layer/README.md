# Quest Standalone OpenXR Hand-to-Touch-Controller Implicit API Layer (`quest_openxr_layer`)

A standalone Android (`arm64-v8a`) OpenXR 1.0/1.1 Implicit API Layer (`XR_APILAYER_ALVR_hand_emulation`) that ports ALVR's hand-tracking gesture bindings (`alvr/server_core/src/hand_gestures.rs`) and hand-to-controller pose offsets (`alvr/session/src/settings.rs`, `alvr/server_openvr/src/tracking.rs`) directly onto Meta Quest OpenXR applications so you can play standalone Quest OpenXR apps using only optical hand tracking.

> **Scope & Policy Notice:** This project strictly implements a standard Khronos OpenXR Implicit API Layer plus `.aar` and unprotected APK bundling tools. It contains **zero DRM, entitlement, or anti-cheat bypasses** and is intended exclusively for unprotected non-Meta-Store OpenXR APKs, sideloaded builds, or apps you build yourself.

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

# 1. One-time toolchain setup (Rust aarch64-linux-android, Android NDK r26b, zipalign/apksigner/aapt)
./scripts/setup_android_env.sh

# 2. Build host + Android arm64-v8a .so, alvr-xr-packager CLI, and dist/ artifacts (.so, .json, .aar)
./scripts/build_android.sh
```

Distributable artifacts are written to `quest_openxr_layer/dist/`:
- `dist/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so` (16 KB ELF aligned, `DT_NEEDED` limited to `libc/libdl/libm/liblog`)
- `dist/XrApiLayer_alvr_hand_emulation.json`
- `dist/alvr-hand-controller-layer.aar`
- `dist/alvr_hand_controller.default.json`

---

## Integrating into Quest OpenXR Apps

### Option A — Drop-in Android `.aar` (Unity / Unreal / Godot / Gradle)
Copy `dist/alvr-hand-controller-layer.aar` into your Android library folder (e.g. `Assets/Plugins/Android/` in Unity or `libs/` in Gradle). Android Manifest Merger automatically merges the Quest/AOSP hand-tracking permissions, `oculus.software.handtracking` (`required="false"`), hand-tracking `meta-data`, native library, and implicit layer manifest.

### Option B — Bundling into an Unprotected Non-Meta-Store OpenXR APK
Use the `alvr-xr-packager` CLI to inject the OpenXR implicit layer (uncompressed & 16 KB page-aligned), patch the binary `AndroidManifest.xml` idempotently, and sign with APK Signature Scheme v1+v2+v3:

```bash
./target/release/alvr-xr-packager patch-apk \
  --input  /path/to/unprotected_game.apk \
  --output /path/to/unprotected_game_handtracking.apk \
  --so     ./dist/arm64-v8a/libXrApiLayer_alvr_hand_emulation.so \
  --manifest ./dist/XrApiLayer_alvr_hand_emulation.json \
  --config ./dist/alvr_hand_controller.default.json

# Verify the resulting APK (manifest entries, 16KB .so alignment, ELF, v2/v3 signature):
./target/release/alvr-xr-packager inspect-apk \
  --input /path/to/unprotected_game_handtracking.apk

# Install to Quest over ADB:
adb install -r /path/to/unprotected_game_handtracking.apk
```

*(Pass `--keystore <file.p12> --ks-pass <pass> --ks-alias <alias>` to sign with your own keystore; omit them to auto-generate a debug keystore via `keytool`.)*

---

## Runtime Tuning & Diagnostics on Quest

You can tune gesture distances, delays, joystick deadzone/offsets, or grip/aim pose offsets without rebuilding the APK by pushing a JSON file (starting from `dist/alvr_hand_controller.default.json`) to the app's external files directory:

```bash
adb push dist/alvr_hand_controller.default.json \
  /sdcard/Android/data/<package.name>/files/alvr_hand_controller.json
```

On desktop/host tests you can also set `ALVR_HAND_LAYER_CONFIG=/path/to/config.json`.
To disable the implicit layer at runtime, set `DISABLE_XR_APILAYER_ALVR_HAND_EMULATION=1`.

Inspect layer logs on Quest via:
```bash
adb logcat -s ALVRHandLayer:V OpenXR-Loader:V
```

---

## Running Tests

```bash
# Unit tests for alvr_hand_core, xr_api_layer, and android_packager
cargo test --offline

# Full opaque-box 4-tier E2E suite (direct dlopen + real Khronos OpenXR 1.1.54 loader + real aapt/zipalign/apksigner)
cd e2e_suite && ./run_e2e.sh --all
```
