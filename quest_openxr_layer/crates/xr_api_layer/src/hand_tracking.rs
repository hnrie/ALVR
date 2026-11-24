//! `XR_EXT_hand_tracking` (+ optional `XR_FB_hand_tracking_aim`) querying and FFI conversions.

use crate::state::InstanceState;
use alvr_hand_core::{HandJointSample, Pose, HAND_JOINT_COUNT};
use glam::{Quat, Vec3};
use openxr_sys as xr;
use openxr_sys::Handle;
use std::ffi::c_void;

/// Result of a successful active `xrLocateHandJointsEXT` query.
#[derive(Debug, Clone)]
pub struct LocatedHand {
    pub samples: [HandJointSample; HAND_JOINT_COUNT],
    pub poses: [Pose; HAND_JOINT_COUNT],
    pub fb_aim_pose: Option<Pose>,
}

/// Converts OpenXR `XrPosef` into `alvr_hand_core::Pose`.
#[inline]
pub fn pose_from_xr(p: xr::Posef) -> Pose {
    Pose {
        orientation: Quat::from_xyzw(
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w,
        ),
        position: Vec3::new(p.position.x, p.position.y, p.position.z),
    }
}

/// Converts `alvr_hand_core::Pose` into a unit-quaternion OpenXR `XrPosef`.
#[inline]
pub fn pose_to_xr(p: Pose) -> xr::Posef {
    let clean = p.normalize_or_identity();
    xr::Posef {
        orientation: xr::Quaternionf {
            x: clean.orientation.x,
            y: clean.orientation.y,
            z: clean.orientation.z,
            w: clean.orientation.w,
        },
        position: xr::Vector3f {
            x: clean.position.x,
            y: clean.position.y,
            z: clean.position.z,
        },
    }
}

/// Converts `glam::Vec3` into OpenXR `XrVector3f`.
#[inline]
pub fn vec3_to_xr(v: Vec3) -> xr::Vector3f {
    if v.is_finite() {
        xr::Vector3f {
            x: v.x,
            y: v.y,
            z: v.z,
        }
    } else {
        xr::Vector3f {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

/// Queries `xrLocateHandJointsEXT` for one hand tracker in `base_space` at `time`,
/// optionally chaining `XrHandJointVelocitiesEXT` and `XrHandTrackingAimStateFB`.
///
/// Returns `Ok(None)` cleanly when `locations.is_active == XR_FALSE` without inspecting
/// joint data (per OpenXR spec, joint poses are invalid when `isActive == XR_FALSE`).
pub fn locate_hand_joints(
    inst: &InstanceState,
    tracker: xr::HandTrackerEXT,
    base_space: xr::Space,
    time: xr::Time,
    want_velocities: bool,
    want_fb_aim: bool,
) -> Result<Option<LocatedHand>, xr::Result> {
    if tracker == xr::HandTrackerEXT::NULL || base_space == xr::Space::NULL {
        return Ok(None);
    }
    let Some(locate_fn) = inst.next.locate_hand_joints_ext else {
        return Ok(None);
    };

    let mut joint_locations = [xr::HandJointLocationEXT {
        location_flags: xr::SpaceLocationFlags::EMPTY,
        pose: xr::Posef::IDENTITY,
        radius: 0.0,
    }; HAND_JOINT_COUNT];

    let mut joint_velocities_buf = [xr::HandJointVelocityEXT {
        velocity_flags: xr::SpaceVelocityFlags::EMPTY,
        linear_velocity: xr::Vector3f::default(),
        angular_velocity: xr::Vector3f::default(),
    }; HAND_JOINT_COUNT];

    let mut aim_state = xr::HandTrackingAimStateFB {
        ty: xr::StructureType::HAND_TRACKING_AIM_STATE_FB,
        next: std::ptr::null_mut(),
        status: xr::HandTrackingAimFlagsFB::EMPTY,
        aim_pose: xr::Posef::IDENTITY,
        pinch_strength_index: 0.0,
        pinch_strength_middle: 0.0,
        pinch_strength_ring: 0.0,
        pinch_strength_little: 0.0,
    };

    let mut velocities_struct = xr::HandJointVelocitiesEXT {
        ty: xr::StructureType::HAND_JOINT_VELOCITIES_EXT,
        next: std::ptr::null_mut(),
        joint_count: HAND_JOINT_COUNT as u32,
        joint_velocities: joint_velocities_buf.as_mut_ptr(),
    };

    let use_fb_aim = want_fb_aim && inst.layer_has_fb_aim;
    let mut next_ptr: *mut c_void = std::ptr::null_mut();
    if use_fb_aim {
        aim_state.next = next_ptr;
        next_ptr = &mut aim_state as *mut _ as *mut c_void;
    }
    if want_velocities {
        velocities_struct.next = next_ptr;
        next_ptr = &mut velocities_struct as *mut _ as *mut c_void;
    }

    let locate_info = xr::HandJointsLocateInfoEXT {
        ty: xr::StructureType::HAND_JOINTS_LOCATE_INFO_EXT,
        next: std::ptr::null(),
        base_space,
        time,
    };

    let mut locations = xr::HandJointLocationsEXT {
        ty: xr::StructureType::HAND_JOINT_LOCATIONS_EXT,
        next: next_ptr,
        is_active: xr::FALSE,
        joint_count: HAND_JOINT_COUNT as u32,
        joint_locations: joint_locations.as_mut_ptr(),
    };

    let res = unsafe { locate_fn(tracker, &locate_info, &mut locations) };
    if res != xr::Result::SUCCESS {
        return Err(res);
    }

    if locations.is_active == xr::FALSE {
        return Ok(None);
    }

    let mut samples = [HandJointSample::default(); HAND_JOINT_COUNT];
    let mut poses = [Pose::IDENTITY; HAND_JOINT_COUNT];

    for i in 0..HAND_JOINT_COUNT {
        let loc = joint_locations[i];
        let pose = pose_from_xr(loc.pose);
        poses[i] = pose;

        let (linear_velocity, angular_velocity) = if want_velocities {
            let v = joint_velocities_buf[i];
            let lin = v
                .velocity_flags
                .contains(xr::SpaceVelocityFlags::LINEAR_VALID)
                .then(|| Vec3::new(v.linear_velocity.x, v.linear_velocity.y, v.linear_velocity.z));
            let ang = v
                .velocity_flags
                .contains(xr::SpaceVelocityFlags::ANGULAR_VALID)
                .then(|| {
                    Vec3::new(
                        v.angular_velocity.x,
                        v.angular_velocity.y,
                        v.angular_velocity.z,
                    )
                });
            (lin, ang)
        } else {
            (None, None)
        };

        samples[i] = HandJointSample {
            pose,
            position_valid: loc
                .location_flags
                .contains(xr::SpaceLocationFlags::POSITION_VALID),
            orientation_valid: loc
                .location_flags
                .contains(xr::SpaceLocationFlags::ORIENTATION_VALID),
            position_tracked: loc
                .location_flags
                .contains(xr::SpaceLocationFlags::POSITION_TRACKED),
            orientation_tracked: loc
                .location_flags
                .contains(xr::SpaceLocationFlags::ORIENTATION_TRACKED),
            radius: loc.radius,
            linear_velocity,
            angular_velocity,
        };
    }

    let fb_aim_pose = if use_fb_aim && aim_state.status.contains(xr::HandTrackingAimFlagsFB::VALID)
    {
        let p = pose_from_xr(aim_state.aim_pose);
        p.is_valid().then_some(p)
    } else {
        None
    };

    Ok(Some(LocatedHand {
        samples,
        poses,
        fb_aim_pose,
    }))
}
