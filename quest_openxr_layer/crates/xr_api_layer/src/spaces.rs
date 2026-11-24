//! OpenXR space lifecycle and `xrLocateSpace` / `xrLocateSpaces` / `xrLocateSpacesKHR` hooks.

use crate::hand_tracking::{locate_hand_joints, pose_from_xr, pose_to_xr, vec3_to_xr};
use crate::profiles::{resolve_action_sources, select_emulated_profile};
use crate::state::{
    get_action, get_session, get_space, SessionState, SpaceKind, SpaceMeta, SPACES,
};
use alvr_hand_core::{HandJoint, HandSide, Pose, PoseSource, SynthesizedPose};
use glam::Vec3;
use openxr_sys as xr;
use openxr_sys::Handle;
use std::ffi::c_void;
use std::sync::Arc;

#[derive(Debug, Clone, Copy)]
struct EmulatedSpaceTarget {
    hand: HandSide,
    pose_source: PoseSource,
    pose_in_action_space: Pose,
}

#[derive(Debug, Clone, Copy)]
struct LocatedResult {
    location_flags: xr::SpaceLocationFlags,
    pose: Pose,
    velocity_flags: xr::SpaceVelocityFlags,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
}

impl Default for LocatedResult {
    fn default() -> Self {
        Self {
            location_flags: xr::SpaceLocationFlags::EMPTY,
            pose: Pose::IDENTITY,
            velocity_flags: xr::SpaceVelocityFlags::EMPTY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        }
    }
}

pub unsafe extern "system" fn layer_create_reference_space(
    session: xr::Session,
    create_info: *const xr::ReferenceSpaceCreateInfo,
    space: *mut xr::Space,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (sess.instance.next.create_reference_space)(session, create_info, space);
        if res == xr::Result::SUCCESS && !create_info.is_null() && !space.is_null() {
            let info = &*create_info;
            let handle = *space;
            let raw = handle.into_raw();
            let meta = Arc::new(SpaceMeta {
                handle,
                session: Arc::clone(&sess),
                kind: SpaceKind::Reference {
                    reference_type: info.reference_space_type,
                    pose_in_reference_space: pose_from_xr(info.pose_in_reference_space),
                },
            });
            SPACES.write().insert(raw, meta);
            sess.inner.lock().spaces.push(raw);
        }
        res
    })
}

pub unsafe extern "system" fn layer_create_action_space(
    session: xr::Session,
    create_info: *const xr::ActionSpaceCreateInfo,
    space: *mut xr::Space,
) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(sess) = get_session(session) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (sess.instance.next.create_action_space)(session, create_info, space);
        if res == xr::Result::SUCCESS && !create_info.is_null() && !space.is_null() {
            let info = &*create_info;
            let handle = *space;
            let raw = handle.into_raw();
            let meta = Arc::new(SpaceMeta {
                handle,
                session: Arc::clone(&sess),
                kind: SpaceKind::Action {
                    action: info.action.into_raw(),
                    subaction_path: info.subaction_path.into_raw(),
                    pose_in_action_space: pose_from_xr(info.pose_in_action_space),
                },
            });
            SPACES.write().insert(raw, meta);
            sess.inner.lock().spaces.push(raw);
        }
        res
    })
}

pub unsafe extern "system" fn layer_destroy_space(space: xr::Space) -> xr::Result {
    crate::ffi_guard(|| {
        let Some(meta) = get_space(space) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let res = (meta.session.instance.next.destroy_space)(space);
        if res == xr::Result::SUCCESS {
            let raw = space.into_raw();
            SPACES.write().remove(&raw);
            meta.session.inner.lock().spaces.retain(|&s| s != raw);
        }
        res
    })
}

pub unsafe extern "system" fn layer_locate_space(
    space: xr::Space,
    base_space: xr::Space,
    time: xr::Time,
    location: *mut xr::SpaceLocation,
) -> xr::Result {
    crate::ffi_guard(|| {
        if location.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
        if time.as_nanos() <= 0 {
            return xr::Result::ERROR_TIME_INVALID;
        }
        let Some(space_meta) = get_space(space) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        let Some(base_meta) = get_space(base_space) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };

        let sess = Arc::clone(&space_meta.session);
        sess.inner.lock().clock.observe_xr_time(time.as_nanos());

        let vel_ptr = find_space_velocity_mut((*location).next);
        let want_vel = !vel_ptr.is_null();

        match locate_space_pair(&sess, &space_meta, &base_meta, time, want_vel) {
            LocateOutcome::Passthrough => {
                (sess.instance.next.locate_space)(space, base_space, time, location)
            }
            LocateOutcome::Synthesized(loc) => {
                (*location).location_flags = loc.location_flags;
                (*location).pose = pose_to_xr(loc.pose);
                if want_vel {
                    (*vel_ptr).velocity_flags = loc.velocity_flags;
                    (*vel_ptr).linear_velocity = vec3_to_xr(loc.linear_velocity);
                    (*vel_ptr).angular_velocity = vec3_to_xr(loc.angular_velocity);
                }
                xr::Result::SUCCESS
            }
            LocateOutcome::Error(e) => e,
        }
    })
}

pub unsafe extern "system" fn layer_locate_spaces(
    session: xr::Session,
    locate_info: *const xr::SpacesLocateInfo,
    space_locations: *mut xr::SpaceLocations,
) -> xr::Result {
    crate::ffi_guard(|| locate_spaces_impl(session, locate_info, space_locations, false))
}

pub unsafe extern "system" fn layer_locate_spaces_khr(
    session: xr::Session,
    locate_info: *const xr::SpacesLocateInfo,
    space_locations: *mut xr::SpaceLocations,
) -> xr::Result {
    crate::ffi_guard(|| locate_spaces_impl(session, locate_info, space_locations, true))
}

unsafe fn locate_spaces_impl(
    session: xr::Session,
    locate_info: *const xr::SpacesLocateInfo,
    space_locations: *mut xr::SpaceLocations,
    use_khr: bool,
) -> xr::Result {
    let Some(sess) = get_session(session) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };
    if locate_info.is_null() || space_locations.is_null() {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }

    let info = &*locate_info;
    let out = &mut *space_locations;

    if info.time.as_nanos() <= 0 {
        return xr::Result::ERROR_TIME_INVALID;
    }
    if info.space_count == 0
        || info.spaces.is_null()
        || out.location_count != info.space_count
        || out.locations.is_null()
    {
        return xr::Result::ERROR_VALIDATION_FAILURE;
    }

    let vel_struct = find_space_velocities_mut(out.next);
    let want_vel = !vel_struct.is_null();
    if want_vel {
        if (*vel_struct).velocity_count != info.space_count || (*vel_struct).velocities.is_null() {
            return xr::Result::ERROR_VALIDATION_FAILURE;
        }
    }

    let Some(base_meta) = get_space(info.base_space) else {
        return xr::Result::ERROR_HANDLE_INVALID;
    };

    sess.inner
        .lock()
        .clock
        .observe_xr_time(info.time.as_nanos());

    let space_handles = std::slice::from_raw_parts(info.spaces, info.space_count as usize);
    let mut space_metas = Vec::with_capacity(space_handles.len());
    for &sp in space_handles {
        let Some(m) = get_space(sp) else {
            return xr::Result::ERROR_HANDLE_INVALID;
        };
        space_metas.push(m);
    }

    let downstream_fn = if use_khr {
        sess.instance
            .next
            .locate_spaces_khr
            .or(sess.instance.next.locate_spaces)
    } else {
        sess.instance
            .next
            .locate_spaces
            .or(sess.instance.next.locate_spaces_khr)
    };

    // Evaluate whether any slot needs synthesis.
    let base_emulated = resolve_emulated_space_target(&sess, &base_meta).is_some();
    let any_slot_emulated = base_emulated
        || space_metas
            .iter()
            .any(|m| is_same_action_entity(&sess, m, &base_meta).is_some() || resolve_emulated_space_target(&sess, m).is_some());

    if !any_slot_emulated {
        if let Some(f) = downstream_fn {
            return f(session, locate_info, space_locations);
        }
        // Fallback loop over locate_space if downstream lacks batched entrypoint.
        let locs = std::slice::from_raw_parts_mut(out.locations, info.space_count as usize);
        let vels = if want_vel {
            Some(std::slice::from_raw_parts_mut(
                (*vel_struct).velocities,
                info.space_count as usize,
            ))
        } else {
            None
        };
        return fallback_locate_spaces_individual(&sess, space_handles, info.base_space, info.time, locs, vels);
    }

    // If base_space is NOT emulated, run downstream batch first to populate non-emulated slots cleanly.
    if !base_emulated {
        if let Some(f) = downstream_fn {
            let res = f(session, locate_info, space_locations);
            if res != xr::Result::SUCCESS {
                return res;
            }
        }
    }

    let locs = std::slice::from_raw_parts_mut(out.locations, info.space_count as usize);
    let mut vels = if want_vel {
        Some(std::slice::from_raw_parts_mut(
            (*vel_struct).velocities,
            info.space_count as usize,
        ))
    } else {
        None
    };

    for (i, sp_meta) in space_metas.iter().enumerate() {
        match locate_space_pair(&sess, sp_meta, &base_meta, info.time, want_vel) {
            LocateOutcome::Passthrough => {
                if base_emulated || downstream_fn.is_none() {
                    let mut single_loc = xr::SpaceLocation {
                        ty: xr::StructureType::SPACE_LOCATION,
                        next: std::ptr::null_mut(),
                        location_flags: xr::SpaceLocationFlags::EMPTY,
                        pose: xr::Posef::IDENTITY,
                    };
                    let mut single_vel = xr::SpaceVelocity {
                        ty: xr::StructureType::SPACE_VELOCITY,
                        next: std::ptr::null_mut(),
                        velocity_flags: xr::SpaceVelocityFlags::EMPTY,
                        linear_velocity: xr::Vector3f::default(),
                        angular_velocity: xr::Vector3f::default(),
                    };
                    if want_vel {
                        single_loc.next = &mut single_vel as *mut _ as *mut c_void;
                    }
                    let res = (sess.instance.next.locate_space)(
                        sp_meta.handle,
                        base_meta.handle,
                        info.time,
                        &mut single_loc,
                    );
                    if res != xr::Result::SUCCESS {
                        return res;
                    }
                    locs[i] = xr::SpaceLocationData {
                        location_flags: single_loc.location_flags,
                        pose: single_loc.pose,
                    };
                    if let Some(ref mut v_slice) = vels {
                        v_slice[i] = xr::SpaceVelocityData {
                            velocity_flags: single_vel.velocity_flags,
                            linear_velocity: single_vel.linear_velocity,
                            angular_velocity: single_vel.angular_velocity,
                        };
                    }
                }
            }
            LocateOutcome::Synthesized(loc) => {
                locs[i] = xr::SpaceLocationData {
                    location_flags: loc.location_flags,
                    pose: pose_to_xr(loc.pose),
                };
                if let Some(ref mut v_slice) = vels {
                    v_slice[i] = xr::SpaceVelocityData {
                        velocity_flags: loc.velocity_flags,
                        linear_velocity: vec3_to_xr(loc.linear_velocity),
                        angular_velocity: vec3_to_xr(loc.angular_velocity),
                    };
                }
            }
            LocateOutcome::Error(e) => return e,
        }
    }

    xr::Result::SUCCESS
}

unsafe fn fallback_locate_spaces_individual(
    sess: &SessionState,
    space_handles: &[xr::Space],
    base_space: xr::Space,
    time: xr::Time,
    locs: &mut [xr::SpaceLocationData],
    mut vels: Option<&mut [xr::SpaceVelocityData]>,
) -> xr::Result {
    for (i, &sp) in space_handles.iter().enumerate() {
        let mut single_loc = xr::SpaceLocation {
            ty: xr::StructureType::SPACE_LOCATION,
            next: std::ptr::null_mut(),
            location_flags: xr::SpaceLocationFlags::EMPTY,
            pose: xr::Posef::IDENTITY,
        };
        let mut single_vel = xr::SpaceVelocity {
            ty: xr::StructureType::SPACE_VELOCITY,
            next: std::ptr::null_mut(),
            velocity_flags: xr::SpaceVelocityFlags::EMPTY,
            linear_velocity: xr::Vector3f::default(),
            angular_velocity: xr::Vector3f::default(),
        };
        if vels.is_some() {
            single_loc.next = &mut single_vel as *mut _ as *mut c_void;
        }
        let res = (sess.instance.next.locate_space)(sp, base_space, time, &mut single_loc);
        if res != xr::Result::SUCCESS {
            return res;
        }
        locs[i] = xr::SpaceLocationData {
            location_flags: single_loc.location_flags,
            pose: single_loc.pose,
        };
        if let Some(ref mut v_slice) = vels {
            v_slice[i] = xr::SpaceVelocityData {
                velocity_flags: single_vel.velocity_flags,
                linear_velocity: single_vel.linear_velocity,
                angular_velocity: single_vel.angular_velocity,
            };
        }
    }
    xr::Result::SUCCESS
}

enum LocateOutcome {
    Passthrough,
    Synthesized(LocatedResult),
    Error(xr::Result),
}

/// Evaluates `space` relative to `base_space`, covering all 4 cases + OpenXR same-entity rule.
unsafe fn locate_space_pair(
    sess: &SessionState,
    space_meta: &SpaceMeta,
    base_meta: &SpaceMeta,
    time: xr::Time,
    want_vel: bool,
) -> LocateOutcome {
    // OpenXR spec (`spaces.adoc:536-542`): if two spaces track the SAME entity,
    // MUST return VALID | TRACKED with their static offset delta even when untracked!
    if let Some(delta_pose) = is_same_action_entity(sess, space_meta, base_meta) {
        return LocateOutcome::Synthesized(LocatedResult {
            location_flags: xr::SpaceLocationFlags::POSITION_VALID
                | xr::SpaceLocationFlags::ORIENTATION_VALID
                | xr::SpaceLocationFlags::POSITION_TRACKED
                | xr::SpaceLocationFlags::ORIENTATION_TRACKED,
            pose: delta_pose,
            velocity_flags: if want_vel {
                xr::SpaceVelocityFlags::LINEAR_VALID | xr::SpaceVelocityFlags::ANGULAR_VALID
            } else {
                xr::SpaceVelocityFlags::EMPTY
            },
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        });
    }

    let space_em = resolve_emulated_space_target(sess, space_meta);
    let base_em = resolve_emulated_space_target(sess, base_meta);

    match (space_em, base_em) {
        (None, None) => LocateOutcome::Passthrough,

        // Case 1: `space` is emulated, `base_space` is reference or physical.
        (Some(target), None) => {
            match synthesize_space_in_base(sess, target, base_meta.handle, time, want_vel) {
                Ok(loc) => LocateOutcome::Synthesized(loc),
                Err(e) => LocateOutcome::Error(e),
            }
        }

        // Case 2: `base_space` is emulated, `space` is reference or physical (inverted baseSpace!).
        (None, Some(base_target)) => {
            let local_space = sess.trackers.lock().local_space;
            if local_space == xr::Space::NULL {
                return LocateOutcome::Passthrough;
            }
            let base_in_local =
                match synthesize_space_in_base(sess, base_target, local_space, time, want_vel) {
                    Ok(loc) => loc,
                    Err(e) => return LocateOutcome::Error(e),
                };
            let space_in_local =
                match locate_downstream_single(sess, space_meta.handle, local_space, time, want_vel)
                {
                    Ok(loc) => loc,
                    Err(e) => return LocateOutcome::Error(e),
                };
            LocateOutcome::Synthesized(compose_relative(space_in_local, base_in_local, want_vel))
        }

        // Case 3: BOTH `space` and `base_space` are emulated (e.g., right hand in left hand frame).
        (Some(space_target), Some(base_target)) => {
            let local_space = sess.trackers.lock().local_space;
            if local_space == xr::Space::NULL {
                return LocateOutcome::Passthrough;
            }
            let space_in_local =
                match synthesize_space_in_base(sess, space_target, local_space, time, want_vel) {
                    Ok(loc) => loc,
                    Err(e) => return LocateOutcome::Error(e),
                };
            let base_in_local =
                match synthesize_space_in_base(sess, base_target, local_space, time, want_vel) {
                    Ok(loc) => loc,
                    Err(e) => return LocateOutcome::Error(e),
                };
            LocateOutcome::Synthesized(compose_relative(space_in_local, base_in_local, want_vel))
        }
    }
}

/// Checks if `space_meta` and `base_meta` refer to the exact same action-space entity (same action
/// and same resolved hand/subaction), returning `base_offset.inverse() * space_offset` if so.
fn is_same_action_entity(
    sess: &SessionState,
    space_meta: &SpaceMeta,
    base_meta: &SpaceMeta,
) -> Option<Pose> {
    if space_meta.handle == base_meta.handle {
        return Some(Pose::IDENTITY);
    }

    if let (
        SpaceKind::Action {
            action: a1,
            subaction_path: sub1,
            pose_in_action_space: p1,
        },
        SpaceKind::Action {
            action: a2,
            subaction_path: sub2,
            pose_in_action_space: p2,
        },
    ) = (space_meta.kind, base_meta.kind)
    {
        if a1 == a2 && sub1 == sub2 {
            // Only intercept when either emulation is enabled or handles refer to the same entity.
            let _ = sess;
            return Some(p2.inverse() * p1);
        }
        // Also check if both resolve to the exact same emulated (hand, pose_source).
        if let (Some(t1), Some(t2)) = (
            resolve_emulated_space_target(sess, space_meta),
            resolve_emulated_space_target(sess, base_meta),
        ) {
            if t1.hand == t2.hand && t1.pose_source == t2.pose_source {
                return Some(t2.pose_in_action_space.inverse() * t1.pose_in_action_space);
            }
        }
    }
    None
}

/// Resolves whether `space_meta` is an action space currently being emulated by hand tracking.
fn resolve_emulated_space_target(
    sess: &SessionState,
    space_meta: &SpaceMeta,
) -> Option<EmulatedSpaceTarget> {
    let SpaceKind::Action {
        action,
        subaction_path,
        pose_in_action_space,
    } = space_meta.kind
    else {
        return None;
    };

    // Snapshot session arbitration and sticky pose source under `sess.inner`, then drop lock.
    let (focused, emulated, sticky, effective_profiles) = {
        let inner = sess.inner.lock();
        (
            inner.last_sync_focused,
            [inner.hand_arb[0].emulated, inner.hand_arb[1].emulated],
            inner.sticky_pose_source.get(&(action, subaction_path)).copied(),
            [
                inner.hand_arb[0].effective_profile,
                inner.hand_arb[1].effective_profile,
            ],
        )
    };

    if !emulated[0] && !emulated[1] {
        return None;
    }

    // If session was synced unfocused, don't return stale poses (action is inactive -> passthrough/untracked).
    if !focused {
        return None;
    }

    // Prefer the sticky source chosen during `xrSyncActions` (guarantees OpenXR single-source stability).
    if let Some((hand, pose_source)) = sticky {
        if emulated[hand.index()] {
            return Some(EmulatedSpaceTarget {
                hand,
                pose_source,
                pose_in_action_space,
            });
        }
    }

    // Fallback: resolve directly from suggested bindings if space is queried after arbitration.
    let _act = get_action(xr::Action::from_raw(action))?;
    let hand_filter = sess
        .instance
        .path_to_hand(xr::Path::from_raw(subaction_path));
    let inst_inner = sess.instance.inner.read();
    let (prof_raw, prof) = {
        let mut chosen = None;
        for hand in [HandSide::Left, HandSide::Right] {
            if let Some(f) = hand_filter {
                if f != hand {
                    continue;
                }
            }
            if emulated[hand.index()] && effective_profiles[hand.index()] != 0 {
                if let Some(p) = inst_inner
                    .path_str(effective_profiles[hand.index()])
                    .and_then(alvr_hand_core::InteractionProfile::from_path)
                {
                    chosen = Some((effective_profiles[hand.index()], p));
                    break;
                }
            }
        }
        chosen.or_else(|| select_emulated_profile(&inst_inner))?
    };

    for src in resolve_action_sources(&inst_inner, action, prof_raw, prof, hand_filter) {
        if let Some(pose_source) = src.resolved.pose_source() {
            if emulated[src.resolved.hand.index()] {
                return Some(EmulatedSpaceTarget {
                    hand: src.resolved.hand,
                    pose_source,
                    pose_in_action_space,
                });
            }
        }
    }
    None
}

unsafe fn synthesize_space_in_base(
    sess: &SessionState,
    target: EmulatedSpaceTarget,
    base_space: xr::Space,
    time: xr::Time,
    want_vel: bool,
) -> Result<LocatedResult, xr::Result> {
    let tracker = sess.trackers.lock().tracker(target.hand);
    let want_fb_aim = matches!(target.pose_source, PoseSource::Aim);

    let located_opt = locate_hand_joints(
        &sess.instance,
        tracker,
        base_space,
        time,
        want_vel,
        want_fb_aim,
    )?;

    let Some(hand_data) = located_opt else {
        // Hand inactive (`isActive == XR_FALSE`): report untracked/invalid cleanly.
        return Ok(LocatedResult::default());
    };

    let cfg = &sess.instance.config.pose;
    let synth: SynthesizedPose = {
        let mut inner = sess.inner.lock();
        let base_synth = match target.pose_source {
            PoseSource::Grip => inner.pose_synthesizer.compute_grip_pose(
                target.hand,
                &hand_data.samples,
                time.as_nanos(),
                cfg,
            ),
            PoseSource::Aim => inner.pose_synthesizer.compute_aim_pose(
                target.hand,
                &hand_data.samples,
                hand_data.fb_aim_pose,
                time.as_nanos(),
                cfg,
            ),
            PoseSource::GripSurface | PoseSource::PalmExt => {
                inner.pose_synthesizer.compute_palm_pose(
                    target.hand,
                    &hand_data.samples,
                    time.as_nanos(),
                )
            }
            PoseSource::PinchExt => {
                let thumb = hand_data.samples[HandJoint::ThumbTip.idx()];
                let index = hand_data.samples[HandJoint::IndexTip.idx()];
                let valid = thumb.position_valid
                    && index.position_valid
                    && index.orientation_valid;
                SynthesizedPose {
                    pose: Pose {
                        orientation: index.pose.orientation.normalize(),
                        position: (thumb.pose.position + index.pose.position) * 0.5,
                    },
                    position_valid: valid,
                    orientation_valid: index.orientation_valid,
                    position_tracked: thumb.position_tracked && index.position_tracked,
                    orientation_tracked: index.orientation_tracked,
                    linear_velocity: index.linear_velocity,
                    angular_velocity: index.angular_velocity,
                }
            }
            PoseSource::PokeExt => {
                let index = hand_data.samples[HandJoint::IndexTip.idx()];
                SynthesizedPose {
                    pose: index.pose.normalize_or_identity(),
                    position_valid: index.position_valid,
                    orientation_valid: index.orientation_valid,
                    position_tracked: index.position_tracked,
                    orientation_tracked: index.orientation_tracked,
                    linear_velocity: index.linear_velocity,
                    angular_velocity: index.angular_velocity,
                }
            }
        };
        base_synth.with_local_offset(target.pose_in_action_space)
    };

    let mut location_flags = xr::SpaceLocationFlags::EMPTY;
    if synth.orientation_valid {
        location_flags |= xr::SpaceLocationFlags::ORIENTATION_VALID;
    }
    if synth.position_valid {
        location_flags |= xr::SpaceLocationFlags::POSITION_VALID;
    }
    if synth.orientation_tracked {
        location_flags |= xr::SpaceLocationFlags::ORIENTATION_TRACKED;
    }
    if synth.position_tracked {
        location_flags |= xr::SpaceLocationFlags::POSITION_TRACKED;
    }

    let mut velocity_flags = xr::SpaceVelocityFlags::EMPTY;
    let mut linear_velocity = Vec3::ZERO;
    let mut angular_velocity = Vec3::ZERO;
    if want_vel {
        if let Some(v) = synth.linear_velocity {
            if v.is_finite() {
                velocity_flags |= xr::SpaceVelocityFlags::LINEAR_VALID;
                linear_velocity = v;
            }
        }
        if let Some(w) = synth.angular_velocity {
            if w.is_finite() {
                velocity_flags |= xr::SpaceVelocityFlags::ANGULAR_VALID;
                angular_velocity = w;
            }
        }
    }

    Ok(LocatedResult {
        location_flags,
        pose: synth.pose.normalize_or_identity(),
        velocity_flags,
        linear_velocity,
        angular_velocity,
    })
}

unsafe fn locate_downstream_single(
    sess: &SessionState,
    space: xr::Space,
    base_space: xr::Space,
    time: xr::Time,
    want_vel: bool,
) -> Result<LocatedResult, xr::Result> {
    let mut loc = xr::SpaceLocation {
        ty: xr::StructureType::SPACE_LOCATION,
        next: std::ptr::null_mut(),
        location_flags: xr::SpaceLocationFlags::EMPTY,
        pose: xr::Posef::IDENTITY,
    };
    let mut vel = xr::SpaceVelocity {
        ty: xr::StructureType::SPACE_VELOCITY,
        next: std::ptr::null_mut(),
        velocity_flags: xr::SpaceVelocityFlags::EMPTY,
        linear_velocity: xr::Vector3f::default(),
        angular_velocity: xr::Vector3f::default(),
    };
    if want_vel {
        loc.next = &mut vel as *mut _ as *mut c_void;
    }

    let res = (sess.instance.next.locate_space)(space, base_space, time, &mut loc);
    if res != xr::Result::SUCCESS {
        return Err(res);
    }

    Ok(LocatedResult {
        location_flags: loc.location_flags,
        pose: pose_from_xr(loc.pose),
        velocity_flags: vel.velocity_flags,
        linear_velocity: Vec3::new(
            vel.linear_velocity.x,
            vel.linear_velocity.y,
            vel.linear_velocity.z,
        ),
        angular_velocity: Vec3::new(
            vel.angular_velocity.x,
            vel.angular_velocity.y,
            vel.angular_velocity.z,
        ),
    })
}

/// Composes `space_in_local` relative to `base_in_local`, transforming both pose and
/// rigid-body velocity cleanly into `base`'s coordinate frame.
fn compose_relative(
    space: LocatedResult,
    base: LocatedResult,
    want_vel: bool,
) -> LocatedResult {
    let ori_valid = space
        .location_flags
        .contains(xr::SpaceLocationFlags::ORIENTATION_VALID)
        && base
            .location_flags
            .contains(xr::SpaceLocationFlags::ORIENTATION_VALID);
    let pos_valid = ori_valid
        && space
            .location_flags
            .contains(xr::SpaceLocationFlags::POSITION_VALID)
        && base
            .location_flags
            .contains(xr::SpaceLocationFlags::POSITION_VALID);
    let ori_tracked = ori_valid
        && space
            .location_flags
            .contains(xr::SpaceLocationFlags::ORIENTATION_TRACKED)
        && base
            .location_flags
            .contains(xr::SpaceLocationFlags::ORIENTATION_TRACKED);
    let pos_tracked = pos_valid
        && space
            .location_flags
            .contains(xr::SpaceLocationFlags::POSITION_TRACKED)
        && base
            .location_flags
            .contains(xr::SpaceLocationFlags::POSITION_TRACKED);

    let mut location_flags = xr::SpaceLocationFlags::EMPTY;
    if ori_valid {
        location_flags |= xr::SpaceLocationFlags::ORIENTATION_VALID;
    }
    if pos_valid {
        location_flags |= xr::SpaceLocationFlags::POSITION_VALID;
    }
    if ori_tracked {
        location_flags |= xr::SpaceLocationFlags::ORIENTATION_TRACKED;
    }
    if pos_tracked {
        location_flags |= xr::SpaceLocationFlags::POSITION_TRACKED;
    }

    let rel_pose = if pos_valid {
        space.pose.relative_to(base.pose)
    } else if ori_valid {
        let q_b_inv = base.pose.orientation.normalize().conjugate();
        Pose {
            orientation: (q_b_inv * space.pose.orientation.normalize()).normalize(),
            position: Vec3::ZERO,
        }
    } else {
        Pose::IDENTITY
    };

    let mut velocity_flags = xr::SpaceVelocityFlags::EMPTY;
    let mut linear_velocity = Vec3::ZERO;
    let mut angular_velocity = Vec3::ZERO;

    if want_vel && ori_valid {
        let q_b_inv = base.pose.orientation.normalize().conjugate();
        let ang_ok = space
            .velocity_flags
            .contains(xr::SpaceVelocityFlags::ANGULAR_VALID)
            && base
                .velocity_flags
                .contains(xr::SpaceVelocityFlags::ANGULAR_VALID);
        if ang_ok {
            let w_rel = q_b_inv * (space.angular_velocity - base.angular_velocity);
            if w_rel.is_finite() {
                velocity_flags |= xr::SpaceVelocityFlags::ANGULAR_VALID;
                angular_velocity = w_rel;
            }
        }
        let lin_ok = pos_valid
            && space
                .velocity_flags
                .contains(xr::SpaceVelocityFlags::LINEAR_VALID)
            && base
                .velocity_flags
                .contains(xr::SpaceVelocityFlags::LINEAR_VALID)
            && base
                .velocity_flags
                .contains(xr::SpaceVelocityFlags::ANGULAR_VALID);
        if lin_ok {
            let dp = space.pose.position - base.pose.position;
            let v_rel =
                q_b_inv * (space.linear_velocity - base.linear_velocity - base.angular_velocity.cross(dp));
            if v_rel.is_finite() {
                velocity_flags |= xr::SpaceVelocityFlags::LINEAR_VALID;
                linear_velocity = v_rel;
            }
        }
    }

    LocatedResult {
        location_flags,
        pose: rel_pose.normalize_or_identity(),
        velocity_flags,
        linear_velocity,
        angular_velocity,
    }
}

unsafe fn find_space_velocity_mut(mut next: *mut c_void) -> *mut xr::SpaceVelocity {
    while !next.is_null() {
        let hdr = next as *mut xr::BaseOutStructure;
        if (*hdr).ty == xr::StructureType::SPACE_VELOCITY {
            return next as *mut xr::SpaceVelocity;
        }
        next = (*hdr).next as *mut c_void;
    }
    std::ptr::null_mut()
}

unsafe fn find_space_velocities_mut(mut next: *mut c_void) -> *mut xr::SpaceVelocities {
    while !next.is_null() {
        let hdr = next as *mut xr::BaseOutStructure;
        if (*hdr).ty == xr::StructureType::SPACE_VELOCITIES {
            return next as *mut xr::SpaceVelocities;
        }
        next = (*hdr).next as *mut c_void;
    }
    std::ptr::null_mut()
}
