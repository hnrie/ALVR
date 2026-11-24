use crate::types::MockPose;
use glam::{Quat, Vec3};

#[inline]
pub fn pose_mul(a: (Quat, Vec3), b: (Quat, Vec3)) -> (Quat, Vec3) {
    let qa = normalize_safe(a.0);
    let qb = normalize_safe(b.0);
    let q = normalize_safe(qa * qb);
    let p = a.1 + qa * b.1;
    (q, p)
}

#[inline]
pub fn pose_inv(a: (Quat, Vec3)) -> (Quat, Vec3) {
    let q_inv = normalize_safe(a.0).conjugate();
    let p_inv = -(q_inv * a.1);
    (q_inv, p_inv)
}

#[inline]
pub fn normalize_safe(q: Quat) -> Quat {
    let len = q.length();
    if len.is_finite() && len > 1e-12 {
        q / len
    } else {
        Quat::IDENTITY
    }
}

/// Validates OpenXR unit quaternion requirement (within 1% tolerance per OpenXR spec).
#[inline]
pub fn is_valid_xr_pose(pose: openxr_sys::Posef) -> bool {
    let q = pose.orientation;
    let p = pose.position;
    if !(q.x.is_finite()
        && q.y.is_finite()
        && q.z.is_finite()
        && q.w.is_finite()
        && p.x.is_finite()
        && p.y.is_finite()
        && p.z.is_finite())
    {
        return false;
    }
    let norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
    (norm - 1.0).abs() <= 0.01
}

#[inline]
pub fn transform_pose_into_base(local_from_target: MockPose, local_from_base: MockPose) -> MockPose {
    let base_from_local = pose_inv(local_from_base.to_quat_pos());
    let (q, p) = pose_mul(base_from_local, local_from_target.to_quat_pos());
    MockPose::from_quat_pos(q, p)
}

/// Transforms linear and angular velocity expressed in LOCAL into baseSpace, accounting for
/// baseSpace rotation (assuming reference space has zero velocity or given base velocity).
#[inline]
pub fn rotate_vec_into_base(v_local: Vec3, local_from_base: MockPose) -> Vec3 {
    let (q_base, _) = local_from_base.to_quat_pos();
    normalize_safe(q_base).conjugate() * v_local
}
