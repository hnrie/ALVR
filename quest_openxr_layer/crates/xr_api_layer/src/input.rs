//! OpenXR path, action set, action, binding, sync, action state, introspection, and haptic hooks.

use crate::hand_tracking::{locate_hand_joints, LocatedHand};
use crate::profiles::{format_localized_source_name, resolve_action_sources, select_emulated_profile};
use crate::state::{
    get_action, get_action_set, get_instance, get_session, ActionMeta, ActionSetMeta,
    ActionSnapshot, PrevActionState, SessionState, SnapshotValue, SuggestedBindingEntry,
    ACTION_SETS, ACTIONS,
};
use alvr_hand_core::{
    aggregate_bool, aggregate_float, aggregate_vec2, HandSide, HysteresisState, InteractionProfile,
    PoseSource, VirtualHandState,
};
use glam::Vec2;
use openxr_sys as xr;
use openxr_sys::Handle;
use std::collections::{HashMap, HashSet};
use std::ffi::{c_char, CStr};
use std::sync::Arc;

#[derive(Debug, Clone, Copy)]
struct PhysicalActionSample {
    is_active: bool,
    value: SnapshotValue,
    changed_since_last_sync: bool,
    last_change_time: i64,
}

pub unsafe extern "system" fn layer_string_to_path(
    instance: xr::Instance,
    path_string: *const c_char,
    path: *mut xr::Path,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(inst) = get_instance(instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (inst.next.string_to_path)(instance, path_string, path);
        if res == xr::Result::SUCCESS && !path_string.is_null() && !path.is_null() {
            if let Ok(s) = CStr::from_ptr(path_string).to_str() {
                inst.inner.write().record_path(*path, s.to_owned());
            }
        }
        res
    })
}

pub unsafe extern "system" fn layer_path_to_string(
    instance: xr::Instance,
    path: xr::Path,
    buffer_capacity_input: u32,
    buffer_count_output: *mut u32,
    buffer: *mut c_char,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(inst) = get_instance(instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (inst.next.path_to_string)(
            instance,
            path,
            buffer_capacity_input,
            buffer_count_output,
            buffer,
        );
        if res == xr::Result::SUCCESS
            && buffer_capacity_input > 0
            && !buffer.is_null()
            && path != xr::Path::NULL
        {
            if let Ok(s) = CStr::from_ptr(buffer).to_str() {
                inst.inner.write().record_path(path, s.to_owned());
            }
        }
        res
    })
}

pub unsafe extern "system" fn layer_create_action_set(
    instance: xr::Instance,
    create_info: *const xr::ActionSetCreateInfo,
    action_set: *mut xr::ActionSet,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(inst) = get_instance(instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (inst.next.create_action_set)(instance, create_info, action_set);
        if res == xr::Result::SUCCESS && !create_info.is_null() && !action_set.is_null() {
            let handle = *action_set;
            let raw = handle.into_raw();
            let meta = Arc::new(ActionSetMeta {
                handle,
                instance: Arc::clone(&inst),
                priority: (*create_info).priority,
                actions: parking_lot::RwLock::new(Vec::new()),
            });
            ACTION_SETS.write().insert(raw, meta);
            inst.inner.write().action_sets.push(raw);
        }
        res
    })
}

pub unsafe extern "system" fn layer_destroy_action_set(action_set: xr::ActionSet) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(meta) = get_action_set(action_set) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (meta.instance.next.destroy_action_set)(action_set);
        if res == xr::Result::SUCCESS {
            let raw = action_set.into_raw();
            ACTION_SETS.write().remove(&raw);
            let child_actions: Vec<u64> = meta.actions.read().clone();
            {
                let mut actions_map = ACTIONS.write();
                for a in &child_actions {
                    actions_map.remove(a);
                }
            }
            let mut inst_inner = meta.instance.inner.write();
            inst_inner.action_sets.retain(|&s| s != raw);
            inst_inner.actions.retain(|a| !child_actions.contains(a));
        }
        res
    })
}

pub unsafe extern "system" fn layer_create_action(
    action_set: xr::ActionSet,
    create_info: *const xr::ActionCreateInfo,
    action: *mut xr::Action,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(set_meta) = get_action_set(action_set) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let inst = Arc::clone(&set_meta.instance);
        let res = (inst.next.create_action)(action_set, create_info, action);
        if res == xr::Result::SUCCESS && !create_info.is_null() && !action.is_null() {
            let info = &*create_info;
            let mut subaction_paths = Vec::new();
            if info.count_subaction_paths > 0 && !info.subaction_paths.is_null() {
                let slice = std::slice::from_raw_parts(
                    info.subaction_paths,
                    info.count_subaction_paths as usize,
                );
                for &p in slice {
                    subaction_paths.push(p.into_raw());
                }
            }
            let handle = *action;
            let raw = handle.into_raw();
            let meta = Arc::new(ActionMeta {
                handle,
                action_set: action_set.into_raw(),
                instance: Arc::clone(&inst),
                action_type: info.action_type,
                subaction_paths,
            });
            ACTIONS.write().insert(raw, meta);
            set_meta.actions.write().push(raw);
            inst.inner.write().actions.push(raw);
        }
        res
    })
}

pub unsafe extern "system" fn layer_destroy_action(action: xr::Action) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(meta) = get_action(action) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (meta.instance.next.destroy_action)(action);
        if res == xr::Result::SUCCESS {
            let raw = action.into_raw();
            ACTIONS.write().remove(&raw);
            if let Some(set_meta) = ACTION_SETS.read().get(&meta.action_set).cloned() {
                set_meta.actions.write().retain(|&a| a != raw);
            }
            meta.instance.inner.write().actions.retain(|&a| a != raw);
        }
        res
    })
}

pub unsafe extern "system" fn layer_suggest_interaction_profile_bindings(
    instance: xr::Instance,
    suggested_bindings: *const xr::InteractionProfileSuggestedBinding,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(inst) = get_instance(instance) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if suggested_bindings.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let sb = &*suggested_bindings;

        // Forward to downstream runtime first; only record on XR_SUCCESS.
        let res = (inst.next.suggest_interaction_profile_bindings)(instance, suggested_bindings);
        if res != xr::Result::SUCCESS {
            return res;
        }

        // Ensure interaction profile path and all binding paths are interned in our path table
        // (even if the app somehow passed paths acquired before or outside our string_to_path hook).
        ensure_path_cached(&inst, sb.interaction_profile);

        let mut entries = Vec::with_capacity(sb.count_suggested_bindings as usize);
        if sb.count_suggested_bindings > 0 && !sb.suggested_bindings.is_null() {
            let slice = std::slice::from_raw_parts(
                sb.suggested_bindings,
                sb.count_suggested_bindings as usize,
            );
            for item in slice {
                ensure_path_cached(&inst, item.binding);
                entries.push(SuggestedBindingEntry {
                    action: item.action.into_raw(),
                    binding_path: item.binding.into_raw(),
                });
            }
        }

        // OpenXR spec (`input.adoc:566-569`): repeat suggestion for same profile REPLACES prior bindings.
        inst.inner
            .write()
            .suggested_bindings
            .insert(sb.interaction_profile.into_raw(), entries);

        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_attach_session_action_sets(
    session: xr::Session,
    attach_info: *const xr::SessionActionSetsAttachInfo,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (sess.instance.next.attach_session_action_sets)(session, attach_info);
        if res == xr::Result::SUCCESS && !attach_info.is_null() {
            let info = &*attach_info;
            let mut inner = sess.inner.lock();
            inner.attached = true;
            if info.count_action_sets > 0 && !info.action_sets.is_null() {
                let slice =
                    std::slice::from_raw_parts(info.action_sets, info.count_action_sets as usize);
                for &set in slice {
                    inner.attached_action_sets.insert(set.into_raw());
                }
            }
        }
        res
    })
}

pub unsafe extern "system" fn layer_get_current_interaction_profile(
    session: xr::Session,
    top_level_user_path: xr::Path,
    interaction_profile: *mut xr::InteractionProfileState,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if interaction_profile.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        // Always call downstream first to preserve exact OpenXR validation
        // (`XR_ERROR_ACTIONSET_NOT_ATTACHED`, `XR_ERROR_PATH_UNSUPPORTED`, etc.).
        let res = (sess.instance.next.get_current_interaction_profile)(
            session,
            top_level_user_path,
            interaction_profile,
        );
        if res != xr::Result::SUCCESS {
            return res;
        }

        if let Some(hand) = sess.instance.path_to_hand(top_level_user_path) {
            let inner = sess.inner.lock();
            let arb = inner.hand_arb[hand.index()];
            if arb.emulated && arb.effective_profile != 0 {
                (*interaction_profile).interaction_profile =
                    xr::Path::from_raw(arb.effective_profile);
            }
        }

        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_sync_actions(
    session: xr::Session,
    sync_info: *const xr::ActionsSyncInfo,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if sync_info.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        let inst = Arc::clone(&sess.instance);
        let res = (inst.next.sync_actions)(session, sync_info);

        if res == xr::Result::SESSION_NOT_FOCUSED {
            let mut inner = sess.inner.lock();
            inner.last_sync_focused = false;
            inner.active_synced_sets.clear();
            inner.action_snapshots.clear();
            inner.gesture_engine.reset();
            inner.last_hand_states = [VirtualHandState::default(); 2];
            for prev in inner.prev_action_values.values_mut() {
                prev.was_active = false;
            }
            for arb in &mut inner.hand_arb {
                arb.emulated = false;
                arb.hand_tracked = false;
            }
            return xr::Result::SESSION_NOT_FOCUSED;
        }

        if res != xr::Result::SUCCESS {
            return res;
        }

        // Collect requested active action sets from sync_info.
        let info = &*sync_info;
        let mut active_sets: HashMap<u64, HashSet<u64>> = HashMap::new();
        if info.count_active_action_sets > 0 && !info.active_action_sets.is_null() {
            let slice = std::slice::from_raw_parts(
                info.active_action_sets,
                info.count_active_action_sets as usize,
            );
            for aas in slice {
                active_sets
                    .entry(aas.action_set.into_raw())
                    .or_default()
                    .insert(aas.subaction_path.into_raw());
            }
        }

        // Gather tracker handles (no lock held during downstream FFI).
        let (trackers_available, left_tracker, right_tracker, local_space) = {
            let t = sess.trackers.lock();
            (t.available, t.left_tracker, t.right_tracker, t.local_space)
        };

        // Snapshot preferred profile & bindings from instance (read lock dropped immediately).
        let preferred_profile = {
            let inst_inner = inst.inner.read();
            select_emulated_profile(&inst_inner)
        };

        // If emulation is disabled or hand tracking unavailable, record synced sets & clear emulated flags.
        if !inst.emulation_enabled() || !trackers_available {
            let mut inner = sess.inner.lock();
            inner.last_sync_focused = true;
            inner.active_synced_sets = active_sets;
            inner.action_snapshots.clear();
            for arb in &mut inner.hand_arb {
                arb.emulated = false;
                arb.hand_tracked = false;
            }
            return xr::Result::SUCCESS;
        }

        // Query downstream current interaction profiles for left & right hands (no locks held).
        let mut runtime_profiles = [0u64; 2];
        for hand in [HandSide::Left, HandSide::Right] {
            let mut st = xr::InteractionProfileState {
                ty: xr::StructureType::INTERACTION_PROFILE_STATE,
                next: std::ptr::null_mut(),
                interaction_profile: xr::Path::NULL,
            };
            let p_res = (inst.next.get_current_interaction_profile)(
                session,
                inst.hand_user_path(hand),
                &mut st,
            );
            if p_res == xr::Result::SUCCESS {
                runtime_profiles[hand.index()] = st.interaction_profile.into_raw();
            }
        }

        // Collect all actions in active_sets.
        let mut active_actions: Vec<Arc<ActionMeta>> = Vec::new();
        {
            let sets_map = ACTION_SETS.read();
            let actions_map = ACTIONS.read();
            for &set_raw in active_sets.keys() {
                if let Some(set_meta) = sets_map.get(&set_raw) {
                    for &act_raw in set_meta.actions.read().iter() {
                        if let Some(act_meta) = actions_map.get(&act_raw) {
                            active_actions.push(Arc::clone(act_meta));
                        }
                    }
                }
            }
        }

        // Probe downstream physical action states for every active (action, subaction_path)
        // WITHOUT holding any lock. This both detects physical controller presence AND supplies
        // physical samples for mixed-hand aggregation!
        let mut phys_samples: HashMap<(u64, u64), PhysicalActionSample> = HashMap::new();
        let mut observed_xr_time: i64 = 0;
        for act in &active_actions {
            if act.action_type == xr::ActionType::VIBRATION_OUTPUT {
                continue;
            }
            for sub_raw in act.snapshot_subaction_keys() {
                if !is_subaction_synced(&active_sets, act.action_set, sub_raw) {
                    continue;
                }
                if let Some(sample) =
                    probe_physical_action(&inst, session, act, xr::Path::from_raw(sub_raw))
                {
                    if sample.last_change_time > observed_xr_time {
                        observed_xr_time = sample.last_change_time;
                    }
                    phys_samples.insert((act.handle.into_raw(), sub_raw), sample);
                }
            }
        }

        // Determine per-hand physical activity using BOTH probed action activity and runtime profiles.
        let mut hand_has_any_bound_action = [false; 2];
        let mut hand_physical_action_active = [false; 2];

        {
            let inst_inner = inst.inner.read();
            for act in &active_actions {
                if act.action_type == xr::ActionType::VIBRATION_OUTPUT {
                    continue;
                }
                let act_raw = act.handle.into_raw();
                for hand in [HandSide::Left, HandSide::Right] {
                    let hand_path_raw = inst.hand_user_path(hand).into_raw();
                    // Check if this action has bindings on `hand` under the runtime profile OR preferred profile.
                    let mut bound_to_hand = false;
                    let mut bound_only_to_this_hand = false;
                    let check_profiles = [
                        runtime_profiles[hand.index()],
                        preferred_profile.map(|(p, _)| p).unwrap_or(0),
                    ];
                    for &prof_raw in &check_profiles {
                        if prof_raw == 0 {
                            continue;
                        }
                        if let Some(entries) = inst_inner.suggested_bindings.get(&prof_raw) {
                            let mut has_this = false;
                            let mut has_other = false;
                            for e in entries {
                                if e.action != act_raw {
                                    continue;
                                }
                                if let Some(pstr) = inst_inner.path_str(e.binding_path) {
                                    if pstr.starts_with(hand.user_path()) {
                                        has_this = true;
                                    } else if pstr.starts_with("/user/hand/") {
                                        has_other = true;
                                    }
                                }
                            }
                            if has_this {
                                bound_to_hand = true;
                                if !has_other {
                                    bound_only_to_this_hand = true;
                                }
                            }
                        }
                    }

                    if !bound_to_hand {
                        continue;
                    }

                    // Only count this action as distinguishable for `hand` if it splits by subaction
                    // OR binds exclusively to `hand`; otherwise fall back to `runtime_profiles[hand]`.
                    if act.subaction_paths.contains(&hand_path_raw) {
                        hand_has_any_bound_action[hand.index()] = true;
                        if phys_samples
                            .get(&(act_raw, hand_path_raw))
                            .is_some_and(|s| s.is_active)
                        {
                            hand_physical_action_active[hand.index()] = true;
                        }
                    } else if bound_only_to_this_hand {
                        hand_has_any_bound_action[hand.index()] = true;
                        if phys_samples
                            .get(&(act_raw, 0))
                            .is_some_and(|s| s.is_active)
                        {
                            hand_physical_action_active[hand.index()] = true;
                        }
                    }
                }
            }
        }

        let mut physical_active = [false; 2];
        for hand in [HandSide::Left, HandSide::Right] {
            let idx = hand.index();
            physical_active[idx] = if hand_physical_action_active[idx] {
                true
            } else if !hand_has_any_bound_action[idx] {
                // No actions bound to this hand were synced; trust runtime's active interaction profile.
                runtime_profiles[idx] != 0
            } else {
                // Actions bound to this hand WERE synced and ALL returned isActive == FALSE.
                false
            };
        }

        // Advance monotonic clock and sample `XrTime` for hand joint queries.
        let (now_ms, xr_time_ns) = {
            let mut inner = sess.inner.lock();
            if observed_xr_time > 0 {
                inner.clock.observe_xr_time(observed_xr_time);
            }
            inner.clock.sample()
        };

        // Locate both hands via XR_EXT_hand_tracking (NO locks held!).
        let query_time = xr::Time::from_nanos(xr_time_ns);
        let located_left: Option<LocatedHand> = if !physical_active[0] {
            locate_hand_joints(&inst, left_tracker, local_space, query_time, false, false)
                .ok()
                .flatten()
        } else {
            None
        };
        let located_right: Option<LocatedHand> = if !physical_active[1] {
            locate_hand_joints(&inst, right_tracker, local_space, query_time, false, false)
                .ok()
                .flatten()
        } else {
            None
        };
        let located = [located_left, located_right];

        // Resolve bindings table snapshot under `inst.inner` read lock BEFORE locking `sess.inner`
        // (preserves strict lock ordering: never nest `inst.inner` inside `sess.inner`!).
        let resolved_by_action: HashMap<u64, Vec<crate::profiles::BoundSource>> =
            if let Some((prof_raw, prof)) = preferred_profile {
                let inst_inner = inst.inner.read();
                let mut map = HashMap::new();
                for act in &active_actions {
                    let act_raw = act.handle.into_raw();
                    let sources = resolve_action_sources(&inst_inner, act_raw, prof_raw, prof, None);
                    map.insert(act_raw, sources);
                }
                map
            } else {
                HashMap::new()
            };

        let left_path_raw = inst.left_hand_path.into_raw();
        let right_path_raw = inst.right_hand_path.into_raw();

        // Update session state, arbitration, gesture engine, and freeze atomic action snapshots!
        let queue_profile_changed = {
            let mut inner = sess.inner.lock();
            inner.last_sync_focused = true;
            inner.active_synced_sets = active_sets.clone();

            let mut profile_changed = false;
            for hand in [HandSide::Left, HandSide::Right] {
                let idx = hand.index();
                let prev_effective = inner.hand_arb[idx].effective_profile;
                inner.hand_arb[idx].runtime_profile = runtime_profiles[idx];

                if physical_active[idx] {
                    inner.hand_arb[idx].last_physical_active_ms = Some(now_ms);
                    inner.hand_arb[idx].emulated = false;
                    inner.hand_arb[idx].hand_tracked = false;
                    inner.gesture_engine.clear_hand(hand);
                    inner.pose_synthesizer.clear_hand(hand);
                    inner.last_hand_states[idx] = VirtualHandState::default();
                } else if let (Some(hand_data), Some(_)) = (&located[idx], preferred_profile) {
                    inner.hand_arb[idx].hand_tracked = true;
                    let grace_elapsed = match inner.hand_arb[idx].last_physical_active_ms {
                        None => true,
                        Some(last_phys) => {
                            now_ms.saturating_sub(last_phys)
                                >= u64::from(inst.config.physical_controller_grace_ms)
                        }
                    };
                    if grace_elapsed {
                        inner.hand_arb[idx].emulated = true;
                        let st = inner.gesture_engine.update_hand(
                            hand,
                            &hand_data.poses,
                            now_ms,
                            &inst.config,
                        );
                        inner.last_hand_states[idx] = st;
                    } else {
                        inner.hand_arb[idx].emulated = false;
                        inner.gesture_engine.clear_hand(hand);
                        inner.last_hand_states[idx] = VirtualHandState::default();
                    }
                } else {
                    inner.hand_arb[idx].emulated = false;
                    inner.hand_arb[idx].hand_tracked = false;
                    inner.gesture_engine.clear_hand(hand);
                    inner.last_hand_states[idx] = VirtualHandState::default();
                }

                let new_effective = if inner.hand_arb[idx].emulated {
                    preferred_profile.map(|(p, _)| p).unwrap_or(0)
                } else {
                    runtime_profiles[idx]
                };
                inner.hand_arb[idx].effective_profile = new_effective;
                if new_effective != prev_effective && inner.is_running {
                    profile_changed = true;
                }
            }

            // Build atomic action snapshots for every active action & subaction key.
            let mut new_snapshots: HashMap<(u64, u64), ActionSnapshot> = HashMap::new();
            let hand_states = inner.last_hand_states;
            let emulated_flags = [inner.hand_arb[0].emulated, inner.hand_arb[1].emulated];

            // Mark previously tracked actions that are NOT in active_sets as inactive.
            let mut synced_keys: HashSet<(u64, u64)> = HashSet::new();

            for act in &active_actions {
                if act.action_type == xr::ActionType::VIBRATION_OUTPUT {
                    continue;
                }
                let act_raw = act.handle.into_raw();
                let sources = resolved_by_action
                    .get(&act_raw)
                    .map(Vec::as_slice)
                    .unwrap_or(&[]);

                for sub_raw in act.snapshot_subaction_keys() {
                    if !is_subaction_synced(&active_sets, act.action_set, sub_raw) {
                        continue;
                    }
                    synced_keys.insert((act_raw, sub_raw));

                    let eligible_hands: [bool; 2] = eligible_hands_for_query(
                        &active_sets,
                        act.action_set,
                        sub_raw,
                        left_path_raw,
                        right_path_raw,
                    );

                    let mut snap = compute_action_snapshot(
                        &mut inner,
                        act,
                        sub_raw,
                        sources,
                        eligible_hands,
                        emulated_flags,
                        &hand_states,
                        &phys_samples,
                        left_path_raw,
                        right_path_raw,
                        &inst.config,
                        xr_time_ns,
                    );

                    // Update prev_action_values for exact changedSinceLastSync tracking.
                    let key = (act_raw, sub_raw);
                    if !snap.is_active {
                        snap.changed_since_last_sync = false;
                        snap.last_change_time = 0;
                        snap.value = zero_value_for_type(act.action_type);
                        if let Some(prev) = inner.prev_action_values.get_mut(&key) {
                            prev.was_active = false;
                        }
                    } else {
                        match inner.prev_action_values.get(&key).copied() {
                            Some(prev) if prev.was_active => {
                                let changed = snap.value != prev.value;
                                snap.changed_since_last_sync = changed;
                                snap.last_change_time = if changed {
                                    if snap.last_change_time > 0 {
                                        snap.last_change_time
                                    } else {
                                        xr_time_ns
                                    }
                                } else {
                                    prev.last_change_time
                                };
                            }
                            _ => {
                                // First sync or inactive -> active: MUST set changedSinceLastSync = FALSE.
                                snap.changed_since_last_sync = false;
                                if snap.last_change_time <= 0 {
                                    snap.last_change_time = xr_time_ns;
                                }
                            }
                        }
                        inner.prev_action_values.insert(
                            key,
                            PrevActionState {
                                was_active: true,
                                value: snap.value,
                                last_change_time: snap.last_change_time,
                            },
                        );
                    }

                    new_snapshots.insert(key, snap);
                }
            }

            // Any previously-active action key not included in this sync becomes inactive.
            for (k, prev) in inner.prev_action_values.iter_mut() {
                if !synced_keys.contains(k) {
                    prev.was_active = false;
                }
            }

            inner.action_snapshots = new_snapshots;
            profile_changed
        };

        if queue_profile_changed {
            inst.inner.write().pending_events.push_back(session);
        }

        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_get_action_state_boolean(
    session: xr::Session,
    get_info: *const xr::ActionStateGetInfo,
    state: *mut xr::ActionStateBoolean,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if get_info.is_null() || state.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = &*get_info;
        if let Err(err) = validate_action_query(&sess, info, xr::ActionType::BOOLEAN_INPUT) {
            return err;
        }

        let use_emulation = {
            let inner = sess.inner.lock();
            inner.any_hand_emulated() || !inner.last_sync_focused
        };
        if !use_emulation {
            return (sess.instance.next.get_action_state_boolean)(session, get_info, state);
        }

        let inner = sess.inner.lock();
        if !inner.last_sync_focused {
            *state = inactive_boolean();
            return xr::Result::SUCCESS;
        }
        let key = (info.action.into_raw(), info.subaction_path.into_raw());
        match inner.action_snapshots.get(&key) {
            Some(snap) if snap.is_active => {
                let b = match snap.value {
                    SnapshotValue::Bool(v) => v,
                    _ => false,
                };
                *state = xr::ActionStateBoolean {
                    ty: xr::StructureType::ACTION_STATE_BOOLEAN,
                    next: std::ptr::null_mut(),
                    current_state: xr::Bool32::from_raw(u32::from(b)),
                    changed_since_last_sync: xr::Bool32::from_raw(u32::from(
                        snap.changed_since_last_sync,
                    )),
                    last_change_time: xr::Time::from_nanos(snap.last_change_time),
                    is_active: xr::TRUE,
                };
            }
            _ => *state = inactive_boolean(),
        }
        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_get_action_state_float(
    session: xr::Session,
    get_info: *const xr::ActionStateGetInfo,
    state: *mut xr::ActionStateFloat,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if get_info.is_null() || state.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = &*get_info;
        if let Err(err) = validate_action_query(&sess, info, xr::ActionType::FLOAT_INPUT) {
            return err;
        }

        let use_emulation = {
            let inner = sess.inner.lock();
            inner.any_hand_emulated() || !inner.last_sync_focused
        };
        if !use_emulation {
            return (sess.instance.next.get_action_state_float)(session, get_info, state);
        }

        let inner = sess.inner.lock();
        if !inner.last_sync_focused {
            *state = inactive_float();
            return xr::Result::SUCCESS;
        }
        let key = (info.action.into_raw(), info.subaction_path.into_raw());
        match inner.action_snapshots.get(&key) {
            Some(snap) if snap.is_active => {
                let f = match snap.value {
                    SnapshotValue::Float(v) => v,
                    _ => 0.0,
                };
                *state = xr::ActionStateFloat {
                    ty: xr::StructureType::ACTION_STATE_FLOAT,
                    next: std::ptr::null_mut(),
                    current_state: f,
                    changed_since_last_sync: xr::Bool32::from_raw(u32::from(
                        snap.changed_since_last_sync,
                    )),
                    last_change_time: xr::Time::from_nanos(snap.last_change_time),
                    is_active: xr::TRUE,
                };
            }
            _ => *state = inactive_float(),
        }
        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_get_action_state_vector2f(
    session: xr::Session,
    get_info: *const xr::ActionStateGetInfo,
    state: *mut xr::ActionStateVector2f,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if get_info.is_null() || state.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = &*get_info;
        if let Err(err) = validate_action_query(&sess, info, xr::ActionType::VECTOR2F_INPUT) {
            return err;
        }

        let use_emulation = {
            let inner = sess.inner.lock();
            inner.any_hand_emulated() || !inner.last_sync_focused
        };
        if !use_emulation {
            return (sess.instance.next.get_action_state_vector2f)(session, get_info, state);
        }

        let inner = sess.inner.lock();
        if !inner.last_sync_focused {
            *state = inactive_vec2();
            return xr::Result::SUCCESS;
        }
        let key = (info.action.into_raw(), info.subaction_path.into_raw());
        match inner.action_snapshots.get(&key) {
            Some(snap) if snap.is_active => {
                let v = match snap.value {
                    SnapshotValue::Vec2(v) => v,
                    _ => Vec2::ZERO,
                };
                *state = xr::ActionStateVector2f {
                    ty: xr::StructureType::ACTION_STATE_VECTOR2F,
                    next: std::ptr::null_mut(),
                    current_state: xr::Vector2f { x: v.x, y: v.y },
                    changed_since_last_sync: xr::Bool32::from_raw(u32::from(
                        snap.changed_since_last_sync,
                    )),
                    last_change_time: xr::Time::from_nanos(snap.last_change_time),
                    is_active: xr::TRUE,
                };
            }
            _ => *state = inactive_vec2(),
        }
        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_get_action_state_pose(
    session: xr::Session,
    get_info: *const xr::ActionStateGetInfo,
    state: *mut xr::ActionStatePose,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if get_info.is_null() || state.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = &*get_info;
        if let Err(err) = validate_action_query(&sess, info, xr::ActionType::POSE_INPUT) {
            return err;
        }

        let use_emulation = {
            let inner = sess.inner.lock();
            inner.any_hand_emulated() || !inner.last_sync_focused
        };
        if !use_emulation {
            return (sess.instance.next.get_action_state_pose)(session, get_info, state);
        }

        let inner = sess.inner.lock();
        if !inner.last_sync_focused {
            *state = inactive_pose();
            return xr::Result::SUCCESS;
        }
        let key = (info.action.into_raw(), info.subaction_path.into_raw());
        let is_active = inner
            .action_snapshots
            .get(&key)
            .is_some_and(|s| s.is_active);
        *state = xr::ActionStatePose {
            ty: xr::StructureType::ACTION_STATE_POSE,
            next: std::ptr::null_mut(),
            is_active: xr::Bool32::from_raw(u32::from(is_active)),
        };
        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_enumerate_bound_sources_for_action(
    session: xr::Session,
    enumerate_info: *const xr::BoundSourcesForActionEnumerateInfo,
    source_capacity_input: u32,
    source_count_output: *mut u32,
    sources: *mut xr::Path,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if enumerate_info.is_null() || source_count_output.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = &*enumerate_info;
        let Some(act) = get_action(info.action) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };

        let (attached, emulated_hands, effective_profiles) = {
            let inner = sess.inner.lock();
            (
                inner.attached && inner.attached_action_sets.contains(&act.action_set),
                [inner.hand_arb[0].emulated, inner.hand_arb[1].emulated],
                [
                    inner.hand_arb[0].effective_profile,
                    inner.hand_arb[1].effective_profile,
                ],
            )
        };
        if !attached {
            return xr::Result::ERROR_ACTIONSET_NOT_ATTACHED;
        }

        if !emulated_hands[0] && !emulated_hands[1] {
            return (sess.instance.next.enumerate_bound_sources_for_action)(
                session,
                enumerate_info,
                source_capacity_input,
                source_count_output,
                sources,
            );
        }

        // Collect downstream sources (for any physical hand) first.
        let mut merged: Vec<xr::Path> = Vec::new();
        let mut rt_count = 0u32;
        if (sess.instance.next.enumerate_bound_sources_for_action)(
            session,
            enumerate_info,
            0,
            &mut rt_count,
            std::ptr::null_mut(),
        ) == xr::Result::SUCCESS
            && rt_count > 0
        {
            let mut buf = vec![xr::Path::NULL; rt_count as usize];
            let mut written = 0u32;
            if (sess.instance.next.enumerate_bound_sources_for_action)(
                session,
                enumerate_info,
                rt_count,
                &mut written,
                buf.as_mut_ptr(),
            ) == xr::Result::SUCCESS
            {
                for p in buf.into_iter().take(written as usize) {
                    if p != xr::Path::NULL && !merged.contains(&p) {
                        merged.push(p);
                    }
                }
            }
        }

        // Append emulated binding paths for emulated hands.
        {
            let inst_inner = sess.instance.inner.read();
            for hand in [HandSide::Left, HandSide::Right] {
                if !emulated_hands[hand.index()] {
                    continue;
                }
                let prof_raw = effective_profiles[hand.index()];
                let Some(prof_str) = inst_inner.path_str(prof_raw) else {
                    continue;
                };
                let Some(prof) = InteractionProfile::from_path(prof_str) else {
                    continue;
                };
                for src in resolve_action_sources(
                    &inst_inner,
                    info.action.into_raw(),
                    prof_raw,
                    prof,
                    Some(hand),
                ) {
                    let p = xr::Path::from_raw(src.binding_path_raw);
                    if !merged.contains(&p) {
                        merged.push(p);
                    }
                }
            }
        }

        *source_count_output = merged.len() as u32;
        if source_capacity_input == 0 {
            return xr::Result::SUCCESS;
        }
        if (source_capacity_input as usize) < merged.len() {
            return xr::Result::ERROR_SIZE_INSUFFICIENT;
        }
        if !merged.is_empty() && sources.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        for (i, &p) in merged.iter().enumerate() {
            *sources.add(i) = p;
        }
        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_get_input_source_localized_name(
    session: xr::Session,
    get_info: *const xr::InputSourceLocalizedNameGetInfo,
    buffer_capacity_input: u32,
    buffer_count_output: *mut u32,
    buffer: *mut c_char,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if get_info.is_null() || buffer_count_output.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = &*get_info;
        if info.which_components == xr::InputSourceLocalizedNameFlags::EMPTY {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }

        let (attached, emulated_any, effective_profiles) = {
            let inner = sess.inner.lock();
            (
                inner.attached,
                inner.any_hand_emulated(),
                [
                    inner.hand_arb[0].effective_profile,
                    inner.hand_arb[1].effective_profile,
                ],
            )
        };
        if !attached {
            return xr::Result::ERROR_ACTIONSET_NOT_ATTACHED;
        }

        if !emulated_any {
            return (sess.instance.next.get_input_source_localized_name)(
                session,
                get_info,
                buffer_capacity_input,
                buffer_count_output,
                buffer,
            );
        }

        // Determine matching emulated profile for this source_path.
        let profile = {
            let inst_inner = sess.instance.inner.read();
            let mut found = None;
            for prof_raw in effective_profiles {
                if let Some(p) = inst_inner
                    .path_str(prof_raw)
                    .and_then(InteractionProfile::from_path)
                {
                    found = Some(p);
                    break;
                }
            }
            found.or_else(|| select_emulated_profile(&inst_inner).map(|(_, p)| p))
        };

        if let Some(prof) = profile {
            if let Some(name) = format_localized_source_name(
                &sess.instance,
                info.source_path,
                prof,
                info.which_components,
            ) {
                let bytes = name.as_bytes();
                let required = (bytes.len() + 1) as u32;
                *buffer_count_output = required;
                if buffer_capacity_input == 0 {
                    return xr::Result::SUCCESS;
                }
                if buffer_capacity_input < required {
                    return xr::Result::ERROR_SIZE_INSUFFICIENT;
                }
                if buffer.is_null() {
                    return xr::Result::ERROR_VALIDATION_FAILURE;
                }
                std::ptr::copy_nonoverlapping(bytes.as_ptr() as *const c_char, buffer, bytes.len());
                *buffer.add(bytes.len()) = 0;
                return xr::Result::SUCCESS;
            }
        }

        (sess.instance.next.get_input_source_localized_name)(
            session,
            get_info,
            buffer_capacity_input,
            buffer_count_output,
            buffer,
        )
    })
}

pub unsafe extern "system" fn layer_apply_haptic_feedback(
    session: xr::Session,
    haptic_action_info: *const xr::HapticActionInfo,
    haptic_feedback: *const xr::HapticBaseHeader,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if haptic_action_info.is_null() || haptic_feedback.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = &*haptic_action_info;
        if let Err(err) = validate_haptic_query(&sess, info) {
            return err;
        }

        let (focused, hand_emulated, any_physical_targeted) =
            haptic_target_mode(&sess, info.subaction_path);
        if !focused {
            return xr::Result::SESSION_NOT_FOCUSED;
        }

        if any_physical_targeted {
            let res = (sess.instance.next.apply_haptic_feedback)(
                session,
                haptic_action_info,
                haptic_feedback,
            );
            if res == xr::Result::SUCCESS || !hand_emulated {
                return res;
            }
        }

        // Per OpenXR spec (`input.adoc:1170-1318`), when no physical haptic motor is attached
        // for an emulated hand, returning `XR_SUCCESS` is explicitly valid.
        xr::Result::SUCCESS
    })
}

pub unsafe extern "system" fn layer_stop_haptic_feedback(
    session: xr::Session,
    haptic_action_info: *const xr::HapticActionInfo,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        if haptic_action_info.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        let info = &*haptic_action_info;
        if let Err(err) = validate_haptic_query(&sess, info) {
            return err;
        }

        let (focused, hand_emulated, any_physical_targeted) =
            haptic_target_mode(&sess, info.subaction_path);
        if !focused {
            return xr::Result::SESSION_NOT_FOCUSED;
        }

        if any_physical_targeted {
            let res = (sess.instance.next.stop_haptic_feedback)(session, haptic_action_info);
            if res == xr::Result::SUCCESS || !hand_emulated {
                return res;
            }
        }

        xr::Result::SUCCESS
    })
}

fn ensure_path_cached(inst: &crate::state::InstanceState, path: xr::Path) {
    let raw = path.into_raw();
    if raw == 0 || inst.inner.read().path_to_str.contains_key(&raw) {
        return;
    }
    let mut len = 0u32;
    let res = unsafe { (inst.next.path_to_string)(inst.handle, path, 0, &mut len, std::ptr::null_mut()) };
    if res != xr::Result::SUCCESS || len == 0 {
        return;
    }
    let mut buf = vec![0 as c_char; len as usize];
    let mut out_len = 0u32;
    let res2 =
        unsafe { (inst.next.path_to_string)(inst.handle, path, len, &mut out_len, buf.as_mut_ptr()) };
    if res2 == xr::Result::SUCCESS {
        if let Ok(s) = unsafe { CStr::from_ptr(buf.as_ptr()) }.to_str() {
            inst.inner.write().record_path(path, s.to_owned());
        }
    }
}

fn validate_action_query(
    sess: &SessionState,
    info: &xr::ActionStateGetInfo,
    expected_type: xr::ActionType,
) -> Result<(), xr::Result> {
    let Some(act) = get_action(info.action) else {
        return Err(xr::Result::ERROR_HANDLE_INVALID);
    };
    if act.action_type != expected_type {
        return Err(xr::Result::ERROR_ACTION_TYPE_MISMATCH);
    }
    if !act.supports_subaction(info.subaction_path) {
        return Err(xr::Result::ERROR_PATH_UNSUPPORTED);
    }
    let inner = sess.inner.lock();
    if !inner.attached || !inner.attached_action_sets.contains(&act.action_set) {
        return Err(xr::Result::ERROR_ACTIONSET_NOT_ATTACHED);
    }
    Ok(())
}

fn validate_haptic_query(
    sess: &SessionState,
    info: &xr::HapticActionInfo,
) -> Result<(), xr::Result> {
    let Some(act) = get_action(info.action) else {
        return Err(xr::Result::ERROR_HANDLE_INVALID);
    };
    if act.action_type != xr::ActionType::VIBRATION_OUTPUT {
        return Err(xr::Result::ERROR_ACTION_TYPE_MISMATCH);
    }
    if !act.supports_subaction(info.subaction_path) {
        return Err(xr::Result::ERROR_PATH_UNSUPPORTED);
    }
    let inner = sess.inner.lock();
    if !inner.attached || !inner.attached_action_sets.contains(&act.action_set) {
        return Err(xr::Result::ERROR_ACTIONSET_NOT_ATTACHED);
    }
    Ok(())
}

fn haptic_target_mode(sess: &SessionState, subaction_path: xr::Path) -> (bool, bool, bool) {
    let inner = sess.inner.lock();
    let focused = inner.session_state == xr::SessionState::FOCUSED || inner.last_sync_focused;
    let (em_l, em_r) = (inner.hand_arb[0].emulated, inner.hand_arb[1].emulated);
    drop(inner);

    match sess.instance.path_to_hand(subaction_path) {
        Some(HandSide::Left) => (focused, em_l, !em_l),
        Some(HandSide::Right) => (focused, em_r, !em_r),
        None => (focused, em_l || em_r, !em_l || !em_r),
    }
}

fn is_subaction_synced(
    active_sets: &HashMap<u64, HashSet<u64>>,
    action_set_raw: u64,
    subaction_raw: u64,
) -> bool {
    let Some(subs) = active_sets.get(&action_set_raw) else {
        return false;
    };
    if subs.contains(&0) {
        return true;
    }
    if subaction_raw == 0 {
        !subs.is_empty()
    } else {
        subs.contains(&subaction_raw)
    }
}

fn eligible_hands_for_query(
    active_sets: &HashMap<u64, HashSet<u64>>,
    action_set_raw: u64,
    subaction_raw: u64,
    left_path_raw: u64,
    right_path_raw: u64,
) -> [bool; 2] {
    let Some(subs) = active_sets.get(&action_set_raw) else {
        return [false, false];
    };
    let wildcard = subs.contains(&0);
    let left_synced = wildcard || (left_path_raw != 0 && subs.contains(&left_path_raw));
    let right_synced = wildcard || (right_path_raw != 0 && subs.contains(&right_path_raw));

    if subaction_raw == 0 {
        [left_synced, right_synced]
    } else if subaction_raw == left_path_raw {
        [left_synced, false]
    } else if subaction_raw == right_path_raw {
        [false, right_synced]
    } else {
        [false, false]
    }
}

unsafe fn probe_physical_action(
    inst: &crate::state::InstanceState,
    session: xr::Session,
    act: &ActionMeta,
    subaction_path: xr::Path,
) -> Option<PhysicalActionSample> {
    let get_info = xr::ActionStateGetInfo {
        ty: xr::StructureType::ACTION_STATE_GET_INFO,
        next: std::ptr::null(),
        action: act.handle,
        subaction_path,
    };

    match act.action_type {
        xr::ActionType::BOOLEAN_INPUT => {
            let mut st = inactive_boolean();
            if (inst.next.get_action_state_boolean)(session, &get_info, &mut st)
                == xr::Result::SUCCESS
            {
                return Some(PhysicalActionSample {
                    is_active: st.is_active != xr::FALSE,
                    value: SnapshotValue::Bool(st.current_state != xr::FALSE),
                    changed_since_last_sync: st.changed_since_last_sync != xr::FALSE,
                    last_change_time: st.last_change_time.as_nanos(),
                });
            }
        }
        xr::ActionType::FLOAT_INPUT => {
            let mut st = inactive_float();
            if (inst.next.get_action_state_float)(session, &get_info, &mut st)
                == xr::Result::SUCCESS
            {
                return Some(PhysicalActionSample {
                    is_active: st.is_active != xr::FALSE,
                    value: SnapshotValue::Float(st.current_state),
                    changed_since_last_sync: st.changed_since_last_sync != xr::FALSE,
                    last_change_time: st.last_change_time.as_nanos(),
                });
            }
        }
        xr::ActionType::VECTOR2F_INPUT => {
            let mut st = inactive_vec2();
            if (inst.next.get_action_state_vector2f)(session, &get_info, &mut st)
                == xr::Result::SUCCESS
            {
                return Some(PhysicalActionSample {
                    is_active: st.is_active != xr::FALSE,
                    value: SnapshotValue::Vec2(Vec2::new(
                        st.current_state.x,
                        st.current_state.y,
                    )),
                    changed_since_last_sync: st.changed_since_last_sync != xr::FALSE,
                    last_change_time: st.last_change_time.as_nanos(),
                });
            }
        }
        xr::ActionType::POSE_INPUT => {
            let mut st = inactive_pose();
            if (inst.next.get_action_state_pose)(session, &get_info, &mut st) == xr::Result::SUCCESS
            {
                return Some(PhysicalActionSample {
                    is_active: st.is_active != xr::FALSE,
                    value: SnapshotValue::Pose,
                    changed_since_last_sync: false,
                    last_change_time: 0,
                });
            }
        }
        _ => {}
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn compute_action_snapshot(
    inner: &mut crate::state::SessionInner,
    act: &ActionMeta,
    sub_raw: u64,
    sources: &[crate::profiles::BoundSource],
    eligible_hands: [bool; 2],
    emulated_flags: [bool; 2],
    hand_states: &[VirtualHandState; 2],
    phys_samples: &HashMap<(u64, u64), PhysicalActionSample>,
    left_path_raw: u64,
    right_path_raw: u64,
    cfg: &alvr_hand_core::LayerConfig,
    xr_time_ns: i64,
) -> ActionSnapshot {
    let act_raw = act.handle.into_raw();

    // Check if ALL eligible hands are non-emulated -> pure physical sample passthrough.
    let any_eligible_emulated =
        (eligible_hands[0] && emulated_flags[0]) || (eligible_hands[1] && emulated_flags[1]);
    if !any_eligible_emulated {
        if let Some(p) = phys_samples.get(&(act_raw, sub_raw)) {
            return ActionSnapshot {
                is_active: p.is_active,
                value: p.value,
                changed_since_last_sync: p.changed_since_last_sync,
                last_change_time: p.last_change_time,
            };
        }
        return ActionSnapshot {
            is_active: false,
            value: zero_value_for_type(act.action_type),
            changed_since_last_sync: false,
            last_change_time: 0,
        };
    }

    match act.action_type {
        xr::ActionType::BOOLEAN_INPUT => {
            let mut vals: Vec<bool> = Vec::new();
            let mut last_time = 0i64;

            // Collect physical contributions from non-emulated eligible hands.
            for (hand, hand_path_raw) in [
                (HandSide::Left, left_path_raw),
                (HandSide::Right, right_path_raw),
            ] {
                let idx = hand.index();
                if eligible_hands[idx] && !emulated_flags[idx] {
                    let phys_key = if act.subaction_paths.contains(&hand_path_raw) {
                        (act_raw, hand_path_raw)
                    } else {
                        (act_raw, 0)
                    };
                    if let Some(p) = phys_samples.get(&phys_key) {
                        if p.is_active {
                            if let SnapshotValue::Bool(b) = p.value {
                                vals.push(b);
                                last_time = last_time.max(p.last_change_time);
                            }
                        }
                    }
                }
            }

            // Collect emulated contributions from emulated eligible hands.
            for (src_idx, src) in sources.iter().enumerate() {
                let h_idx = src.resolved.hand.index();
                if !eligible_hands[h_idx] || !emulated_flags[h_idx] {
                    continue;
                }
                let hyst = inner
                    .float_to_bool_hysteresis
                    .entry((act_raw, sub_raw, src_idx))
                    .or_insert_with(HysteresisState::new);
                if let Some(b) = src.resolved.evaluate_bool(
                    &hand_states[h_idx],
                    hyst,
                    cfg.float_to_bool_threshold,
                    cfg.float_to_bool_hysteresis,
                ) {
                    vals.push(b);
                }
            }

            if vals.is_empty() {
                ActionSnapshot {
                    is_active: false,
                    value: SnapshotValue::Bool(false),
                    changed_since_last_sync: false,
                    last_change_time: 0,
                }
            } else {
                ActionSnapshot {
                    is_active: true,
                    value: SnapshotValue::Bool(aggregate_bool(vals)),
                    changed_since_last_sync: false,
                    last_change_time: last_time,
                }
            }
        }
        xr::ActionType::FLOAT_INPUT => {
            let mut vals: Vec<f32> = Vec::new();
            let mut last_time = 0i64;

            for (hand, hand_path_raw) in [
                (HandSide::Left, left_path_raw),
                (HandSide::Right, right_path_raw),
            ] {
                let idx = hand.index();
                if eligible_hands[idx] && !emulated_flags[idx] {
                    let phys_key = if act.subaction_paths.contains(&hand_path_raw) {
                        (act_raw, hand_path_raw)
                    } else {
                        (act_raw, 0)
                    };
                    if let Some(p) = phys_samples.get(&phys_key) {
                        if p.is_active {
                            if let SnapshotValue::Float(f) = p.value {
                                vals.push(f);
                                last_time = last_time.max(p.last_change_time);
                            }
                        }
                    }
                }
            }

            for src in sources {
                let h_idx = src.resolved.hand.index();
                if !eligible_hands[h_idx] || !emulated_flags[h_idx] {
                    continue;
                }
                if let Some(f) = src.resolved.evaluate_float(&hand_states[h_idx]) {
                    vals.push(f);
                }
            }

            if vals.is_empty() {
                ActionSnapshot {
                    is_active: false,
                    value: SnapshotValue::Float(0.0),
                    changed_since_last_sync: false,
                    last_change_time: 0,
                }
            } else {
                ActionSnapshot {
                    is_active: true,
                    value: SnapshotValue::Float(aggregate_float(vals)),
                    changed_since_last_sync: false,
                    last_change_time: last_time,
                }
            }
        }
        xr::ActionType::VECTOR2F_INPUT => {
            let mut vals: Vec<Vec2> = Vec::new();
            let mut last_time = 0i64;

            for (hand, hand_path_raw) in [
                (HandSide::Left, left_path_raw),
                (HandSide::Right, right_path_raw),
            ] {
                let idx = hand.index();
                if eligible_hands[idx] && !emulated_flags[idx] {
                    let phys_key = if act.subaction_paths.contains(&hand_path_raw) {
                        (act_raw, hand_path_raw)
                    } else {
                        (act_raw, 0)
                    };
                    if let Some(p) = phys_samples.get(&phys_key) {
                        if p.is_active {
                            if let SnapshotValue::Vec2(v) = p.value {
                                vals.push(v);
                                last_time = last_time.max(p.last_change_time);
                            }
                        }
                    }
                }
            }

            for src in sources {
                let h_idx = src.resolved.hand.index();
                if !eligible_hands[h_idx] || !emulated_flags[h_idx] {
                    continue;
                }
                if let Some(v) = src.resolved.evaluate_vec2(&hand_states[h_idx]) {
                    vals.push(v);
                }
            }

            if vals.is_empty() {
                ActionSnapshot {
                    is_active: false,
                    value: SnapshotValue::Vec2(Vec2::ZERO),
                    changed_since_last_sync: false,
                    last_change_time: 0,
                }
            } else {
                ActionSnapshot {
                    is_active: true,
                    value: SnapshotValue::Vec2(aggregate_vec2(vals)),
                    changed_since_last_sync: false,
                    last_change_time: last_time,
                }
            }
        }
        xr::ActionType::POSE_INPUT => {
            let mut active_emulated_poses: Vec<(HandSide, PoseSource)> = Vec::new();
            for src in sources {
                let hand = src.resolved.hand;
                let h_idx = hand.index();
                if !eligible_hands[h_idx] || !emulated_flags[h_idx] {
                    continue;
                }
                if let Some(pose_src) = src.resolved.pose_source() {
                    if !active_emulated_poses.contains(&(hand, pose_src)) {
                        active_emulated_poses.push((hand, pose_src));
                    }
                }
            }

            let mut any_physical_pose_active = false;
            for (hand, hand_path_raw) in [
                (HandSide::Left, left_path_raw),
                (HandSide::Right, right_path_raw),
            ] {
                let idx = hand.index();
                if eligible_hands[idx] && !emulated_flags[idx] {
                    let phys_key = if act.subaction_paths.contains(&hand_path_raw) {
                        (act_raw, hand_path_raw)
                    } else {
                        (act_raw, 0)
                    };
                    if phys_samples.get(&phys_key).is_some_and(|p| p.is_active) {
                        any_physical_pose_active = true;
                    }
                }
            }

            let key = (act_raw, sub_raw);
            if !active_emulated_poses.is_empty() {
                // Sticky single-source rule: keep previous active pose source if still active,
                // otherwise pick the first active source (only changes on xrSyncActions).
                let prev = inner.sticky_pose_source.get(&key).copied();
                let chosen = match prev {
                    Some(p) if active_emulated_poses.contains(&p) => p,
                    _ => active_emulated_poses[0],
                };
                inner.sticky_pose_source.insert(key, chosen);
                ActionSnapshot {
                    is_active: true,
                    value: SnapshotValue::Pose,
                    changed_since_last_sync: false,
                    last_change_time: xr_time_ns,
                }
            } else {
                inner.sticky_pose_source.remove(&key);
                ActionSnapshot {
                    is_active: any_physical_pose_active,
                    value: SnapshotValue::Pose,
                    changed_since_last_sync: false,
                    last_change_time: 0,
                }
            }
        }
        _ => ActionSnapshot {
            is_active: false,
            value: SnapshotValue::Bool(false),
            changed_since_last_sync: false,
            last_change_time: 0,
        },
    }
}

fn zero_value_for_type(ty: xr::ActionType) -> SnapshotValue {
    match ty {
        xr::ActionType::BOOLEAN_INPUT => SnapshotValue::Bool(false),
        xr::ActionType::FLOAT_INPUT => SnapshotValue::Float(0.0),
        xr::ActionType::VECTOR2F_INPUT => SnapshotValue::Vec2(Vec2::ZERO),
        _ => SnapshotValue::Pose,
    }
}

fn inactive_boolean() -> xr::ActionStateBoolean {
    xr::ActionStateBoolean {
        ty: xr::StructureType::ACTION_STATE_BOOLEAN,
        next: std::ptr::null_mut(),
        current_state: xr::FALSE,
        changed_since_last_sync: xr::FALSE,
        last_change_time: xr::Time::from_nanos(0),
        is_active: xr::FALSE,
    }
}

fn inactive_float() -> xr::ActionStateFloat {
    xr::ActionStateFloat {
        ty: xr::StructureType::ACTION_STATE_FLOAT,
        next: std::ptr::null_mut(),
        current_state: 0.0,
        changed_since_last_sync: xr::FALSE,
        last_change_time: xr::Time::from_nanos(0),
        is_active: xr::FALSE,
    }
}

fn inactive_vec2() -> xr::ActionStateVector2f {
    xr::ActionStateVector2f {
        ty: xr::StructureType::ACTION_STATE_VECTOR2F,
        next: std::ptr::null_mut(),
        current_state: xr::Vector2f { x: 0.0, y: 0.0 },
        changed_since_last_sync: xr::FALSE,
        last_change_time: xr::Time::from_nanos(0),
        is_active: xr::FALSE,
    }
}

fn inactive_pose() -> xr::ActionStatePose {
    xr::ActionStatePose {
        ty: xr::StructureType::ACTION_STATE_POSE,
        next: std::ptr::null_mut(),
        is_active: xr::FALSE,
    }
}
