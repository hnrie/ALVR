//! Interaction profile selection and binding resolution across all 9 OpenXR 1.0/1.1 Quest Touch & Simple profiles.

use crate::state::{InstanceInner, InstanceState};
use alvr_hand_core::{resolve_binding, HandSide, InteractionProfile, ResolvedBinding};
use openxr_sys as xr;

/// Concrete binding entry resolved for an action under a specific interaction profile.
#[derive(Debug, Clone, Copy)]
pub struct BoundSource {
    pub binding_path_raw: u64,
    pub resolved: ResolvedBinding,
    pub profile: InteractionProfile,
}

/// Selects the highest-priority supported interaction profile that the application actually
/// called `xrSuggestInteractionProfileBindings` for.
///
/// Per OpenXR spec (`input.adoc:706-816`), `xrGetCurrentInteractionProfile` MUST ONLY return
/// a profile previously suggested by the app (or `XR_NULL_PATH`).
pub fn select_emulated_profile(inner: &InstanceInner) -> Option<(u64, InteractionProfile)> {
    let mut best: Option<(u64, InteractionProfile)> = None;
    for (&profile_raw, bindings) in &inner.suggested_bindings {
        if bindings.is_empty() {
            continue;
        }
        let Some(path_str) = inner.path_str(profile_raw) else {
            continue;
        };
        let Some(profile) = InteractionProfile::from_path(path_str) else {
            continue;
        };
        match best {
            Some((_, cur)) if cur.priority() <= profile.priority() => {}
            _ => best = Some((profile_raw, profile)),
        }
    }
    best
}

/// Returns all resolved bindings for `action_raw` under `profile_raw` / `profile` in `O(bindings_for_action)`,
/// optionally filtered to a specific `hand_filter` (derived from `subactionPath`).
pub fn resolve_action_sources(
    inner: &InstanceInner,
    action_raw: u64,
    profile_raw: u64,
    profile: InteractionProfile,
    hand_filter: Option<HandSide>,
) -> Vec<BoundSource> {
    let Some(binding_paths) = inner
        .bindings_by_profile_action
        .get(&(profile_raw, action_raw))
    else {
        return Vec::new();
    };
    let mut sources = Vec::with_capacity(binding_paths.len());
    for &binding_path_raw in binding_paths {
        let Some(path_str) = inner.path_str(binding_path_raw) else {
            continue;
        };
        let Some(resolved) = resolve_binding(profile, path_str) else {
            continue;
        };
        if let Some(target_hand) = hand_filter {
            if resolved.hand != target_hand {
                continue;
            }
        }
        sources.push(BoundSource {
            binding_path_raw,
            resolved,
            profile,
        });
    }
    sources
}

/// Resolves the localized name for an emulated `source_path` honoring OpenXR `whichComponents`.
pub fn format_localized_source_name(
    inst: &InstanceState,
    source_path: xr::Path,
    profile: InteractionProfile,
    which: xr::InputSourceLocalizedNameFlags,
) -> Option<String> {
    let path_str = inst
        .inner
        .read()
        .path_str(source_path.into_raw())
        .map(str::to_owned)?;
    let resolved = resolve_binding(profile, &path_str)?;

    let mut parts: Vec<&'static str> = Vec::with_capacity(3);
    if which.contains(xr::InputSourceLocalizedNameFlags::USER_PATH) {
        parts.push(match resolved.hand {
            HandSide::Left => "Left Hand",
            HandSide::Right => "Right Hand",
        });
    }
    if which.contains(xr::InputSourceLocalizedNameFlags::INTERACTION_PROFILE) {
        parts.push(profile.localized_name());
    }
    if which.contains(xr::InputSourceLocalizedNameFlags::COMPONENT) {
        parts.push(resolved.localized_component_name());
    }

    if parts.is_empty() {
        return None;
    }
    Some(parts.join(" "))
}
