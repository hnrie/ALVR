//! Canonical virtual Oculus Touch / KHR Simple hand input snapshot, all 9 OpenXR 1.0/1.1
//! Quest interaction profiles, path resolver, OpenXR type conversions, and multi-source aggregation.

use crate::gestures::HysteresisState;
use crate::HandSide;
use glam::Vec2;

/// Snapshot of all emulated Oculus Touch / Touch Plus / Touch Pro / Simple inputs for ONE hand.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct VirtualHandState {
    pub tracked: bool,
    pub trigger_value: f32,
    pub trigger_touch: bool,
    pub trigger_click: bool,
    pub squeeze_value: f32,
    pub squeeze_click: bool,
    pub thumbstick: Vec2,
    pub thumbstick_touch: bool,
    pub thumbstick_click: bool,
    pub primary_click: bool,   // Left: X, Right: A (ThumbRingPinch)
    pub primary_touch: bool,
    pub secondary_click: bool, // Left: Y, Right: B (ThumbMiddlePinch)
    pub secondary_touch: bool,
    pub menu_click: bool,      // Left: menu (ThumbLittlePinch), Right: false
    pub select_click: bool,    // KHR simple (/input/select/click) mapped from trigger pinch
    pub thumbrest_touch: bool, // default false (mirrors ALVR)
}

/// All 9 supported OpenXR 1.0 / 1.1 Quest Touch & KHR Simple interaction profiles,
/// ordered from highest to lowest selection priority.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InteractionProfile {
    /// OpenXR 1.1 core: `/interaction_profiles/meta/touch_plus_controller`
    MetaTouchPlus11,
    /// `XR_META_touch_controller_plus`: `/interaction_profiles/meta/touch_controller_plus`
    MetaTouchPlusExt,
    /// OpenXR 1.1 core: `/interaction_profiles/meta/touch_pro_controller`
    MetaTouchPro11,
    /// `XR_FB_touch_controller_pro`: `/interaction_profiles/facebook/touch_controller_pro`
    FbTouchProExt,
    /// OpenXR 1.1 core: `/interaction_profiles/meta/touch_controller_quest_2`
    MetaTouchQuest2,
    /// OpenXR 1.1 core: `/interaction_profiles/meta/touch_controller_quest_1_rift_s`
    MetaTouchQuest1RiftS,
    /// OpenXR 1.1 core: `/interaction_profiles/meta/touch_controller_rift_cv1`
    MetaTouchRiftCv1,
    /// OpenXR 1.0 core: `/interaction_profiles/oculus/touch_controller`
    OculusTouch,
    /// OpenXR 1.0 core: `/interaction_profiles/khr/simple_controller`
    KhrSimple,
}

impl InteractionProfile {
    /// All 9 profiles in descending preference order (index 0 = highest priority).
    pub const ALL_BY_PRIORITY: [Self; 9] = [
        Self::MetaTouchPlus11,
        Self::MetaTouchPlusExt,
        Self::MetaTouchPro11,
        Self::FbTouchProExt,
        Self::MetaTouchQuest2,
        Self::MetaTouchQuest1RiftS,
        Self::MetaTouchRiftCv1,
        Self::OculusTouch,
        Self::KhrSimple,
    ];

    /// Parses an OpenXR interaction profile path string into a supported [`InteractionProfile`].
    pub fn from_path(path: &str) -> Option<Self> {
        match path {
            "/interaction_profiles/meta/touch_plus_controller" => Some(Self::MetaTouchPlus11),
            "/interaction_profiles/meta/touch_controller_plus" => Some(Self::MetaTouchPlusExt),
            "/interaction_profiles/meta/touch_pro_controller" => Some(Self::MetaTouchPro11),
            "/interaction_profiles/facebook/touch_controller_pro" => Some(Self::FbTouchProExt),
            "/interaction_profiles/meta/touch_controller_quest_2" => Some(Self::MetaTouchQuest2),
            "/interaction_profiles/meta/touch_controller_quest_1_rift_s" => {
                Some(Self::MetaTouchQuest1RiftS)
            }
            "/interaction_profiles/meta/touch_controller_rift_cv1" => Some(Self::MetaTouchRiftCv1),
            "/interaction_profiles/oculus/touch_controller" => Some(Self::OculusTouch),
            "/interaction_profiles/khr/simple_controller" => Some(Self::KhrSimple),
            _ => None,
        }
    }

    /// Returns the canonical OpenXR interaction profile path string.
    pub const fn as_path(self) -> &'static str {
        match self {
            Self::MetaTouchPlus11 => "/interaction_profiles/meta/touch_plus_controller",
            Self::MetaTouchPlusExt => "/interaction_profiles/meta/touch_controller_plus",
            Self::MetaTouchPro11 => "/interaction_profiles/meta/touch_pro_controller",
            Self::FbTouchProExt => "/interaction_profiles/facebook/touch_controller_pro",
            Self::MetaTouchQuest2 => "/interaction_profiles/meta/touch_controller_quest_2",
            Self::MetaTouchQuest1RiftS => {
                "/interaction_profiles/meta/touch_controller_quest_1_rift_s"
            }
            Self::MetaTouchRiftCv1 => "/interaction_profiles/meta/touch_controller_rift_cv1",
            Self::OculusTouch => "/interaction_profiles/oculus/touch_controller",
            Self::KhrSimple => "/interaction_profiles/khr/simple_controller",
        }
    }

    /// Selection priority (`1..=9`, where lower is preferred).
    pub const fn priority(self) -> u8 {
        match self {
            Self::MetaTouchPlus11 => 1,
            Self::MetaTouchPlusExt => 2,
            Self::MetaTouchPro11 => 3,
            Self::FbTouchProExt => 4,
            Self::MetaTouchQuest2 => 5,
            Self::MetaTouchQuest1RiftS => 6,
            Self::MetaTouchRiftCv1 => 7,
            Self::OculusTouch => 8,
            Self::KhrSimple => 9,
        }
    }

    /// Returns `true` for all Oculus / Meta Touch controller profiles (`false` for `KhrSimple`).
    pub const fn is_touch_family(self) -> bool {
        !matches!(self, Self::KhrSimple)
    }

    /// Per Khronos `xr.xml`, `/input/thumbrest/touch` is present on all Touch profiles EXCEPT
    /// `/interaction_profiles/meta/touch_controller_quest_1_rift_s`.
    pub const fn has_thumbrest_touch(self) -> bool {
        self.is_touch_family() && !matches!(self, Self::MetaTouchQuest1RiftS)
    }

    /// Human-readable controller name for `xrGetInputSourceLocalizedName`.
    pub const fn localized_name(self) -> &'static str {
        match self {
            Self::MetaTouchPlus11 | Self::MetaTouchPlusExt => "Meta Quest Touch Plus Controller",
            Self::MetaTouchPro11 | Self::FbTouchProExt => "Meta Quest Touch Pro Controller",
            Self::MetaTouchQuest2 => "Meta Quest 2 Touch Controller",
            Self::MetaTouchQuest1RiftS => "Oculus Quest 1 / Rift S Touch Controller",
            Self::MetaTouchRiftCv1 => "Oculus Rift CV1 Touch Controller",
            Self::OculusTouch => "Oculus Touch Controller",
            Self::KhrSimple => "Khronos Simple Controller",
        }
    }

    /// Selects the highest-priority supported profile actually suggested by the application.
    pub fn select_preferred<'a>(suggested_paths: impl IntoIterator<Item = &'a str>) -> Option<Self> {
        suggested_paths
            .into_iter()
            .filter_map(Self::from_path)
            .min_by_key(|p| p.priority())
    }
}

/// Logical pose source bound to a pose action space.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PoseSource {
    Grip,
    Aim,
    GripSurface,
    PalmExt,
    PinchExt,
    PokeExt,
}

/// Canonical OpenXR controller component resolved from a suggested binding path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VirtualComponent {
    // Boolean components
    PrimaryClick,
    PrimaryTouch,
    SecondaryClick,
    SecondaryTouch,
    MenuClick,
    SystemClick,
    SelectClick,
    TriggerTouch,
    TriggerClick,
    TriggerProximity,
    SqueezeClick,
    ThumbstickClick,
    ThumbstickTouch,
    ThumbstickDpadUp,
    ThumbstickDpadDown,
    ThumbstickDpadLeft,
    ThumbstickDpadRight,
    ThumbrestTouch,
    ThumbProximity,

    // Float components
    TriggerValue,
    TriggerForce,
    TriggerCurl,
    TriggerSlide,
    SqueezeValue,
    ThumbstickX,
    ThumbstickY,
    ThumbrestForce,
    StylusForce,

    // Vector2f component
    Thumbstick,

    // Pose components
    Pose(PoseSource),

    // Haptic output sink
    Haptic,
}

impl VirtualComponent {
    fn raw_bool(self, state: &VirtualHandState) -> Option<bool> {
        let b = match self {
            Self::PrimaryClick => state.primary_click,
            Self::PrimaryTouch => state.primary_touch,
            Self::SecondaryClick => state.secondary_click,
            Self::SecondaryTouch => state.secondary_touch,
            Self::MenuClick => state.menu_click,
            Self::SystemClick => false,
            Self::SelectClick => state.select_click,
            Self::TriggerTouch => state.trigger_touch,
            Self::TriggerClick => state.trigger_click,
            Self::TriggerProximity => state.trigger_touch || state.trigger_value > 0.0,
            Self::SqueezeClick => state.squeeze_click,
            Self::ThumbstickClick => state.thumbstick_click,
            Self::ThumbstickTouch => state.thumbstick_touch,
            Self::ThumbstickDpadUp => {
                state.thumbstick.y >= 0.5 && state.thumbstick.y.abs() >= state.thumbstick.x.abs()
            }
            Self::ThumbstickDpadDown => {
                state.thumbstick.y <= -0.5 && state.thumbstick.y.abs() >= state.thumbstick.x.abs()
            }
            Self::ThumbstickDpadRight => {
                state.thumbstick.x >= 0.5 && state.thumbstick.x.abs() > state.thumbstick.y.abs()
            }
            Self::ThumbstickDpadLeft => {
                state.thumbstick.x <= -0.5 && state.thumbstick.x.abs() > state.thumbstick.y.abs()
            }
            Self::ThumbrestTouch => state.thumbrest_touch,
            Self::ThumbProximity => {
                state.thumbstick_touch
                    || state.primary_touch
                    || state.secondary_touch
                    || state.thumbrest_touch
            }
            _ => return None,
        };
        Some(b)
    }

    fn raw_float(self, state: &VirtualHandState) -> Option<f32> {
        let f = match self {
            Self::TriggerValue | Self::TriggerForce | Self::TriggerCurl => state.trigger_value,
            Self::TriggerSlide | Self::ThumbrestForce | Self::StylusForce => 0.0,
            Self::SqueezeValue => state.squeeze_value,
            Self::ThumbstickX => state.thumbstick.x,
            Self::ThumbstickY => state.thumbstick.y,
            _ => return None,
        };
        Some(f)
    }

    /// Evaluates this component as an OpenXR Boolean action (including float->bool hysteresis coercion).
    pub fn evaluate_bool(
        self,
        state: &VirtualHandState,
        hysteresis: &mut HysteresisState,
        threshold: f32,
        deviation: f32,
    ) -> Option<bool> {
        if let Some(b) = self.raw_bool(state) {
            return Some(b);
        }
        if let Some(f) = self.raw_float(state) {
            return Some(hysteresis.update(f, threshold, deviation));
        }
        None
    }

    /// Evaluates this component as an OpenXR Float action (including bool->float `0.0`/`1.0` coercion).
    pub fn evaluate_float(self, state: &VirtualHandState) -> Option<f32> {
        if let Some(f) = self.raw_float(state) {
            return Some(f);
        }
        if let Some(b) = self.raw_bool(state) {
            return Some(if b { 1.0 } else { 0.0 });
        }
        None
    }

    /// Evaluates this component as an OpenXR Vector2f action.
    pub fn evaluate_vec2(self, state: &VirtualHandState) -> Option<Vec2> {
        match self {
            Self::Thumbstick => Some(state.thumbstick),
            _ => None,
        }
    }

    /// Returns the pose source if this component is a pose binding.
    pub const fn pose_source(self) -> Option<PoseSource> {
        match self {
            Self::Pose(src) => Some(src),
            _ => None,
        }
    }

    /// Returns `true` if this component is a haptic vibration output sink.
    pub const fn is_haptic(self) -> bool {
        matches!(self, Self::Haptic)
    }

    /// Human-readable localized component label for `xrGetInputSourceLocalizedName`.
    pub const fn localized_component_name(self, hand: HandSide) -> &'static str {
        match (self, hand) {
            (Self::PrimaryClick | Self::PrimaryTouch, HandSide::Left) => "X Button",
            (Self::PrimaryClick | Self::PrimaryTouch, HandSide::Right) => "A Button",
            (Self::SecondaryClick | Self::SecondaryTouch, HandSide::Left) => "Y Button",
            (Self::SecondaryClick | Self::SecondaryTouch, HandSide::Right) => "B Button",
            (Self::MenuClick, _) => "Menu Button",
            (Self::SystemClick, _) => "System Button",
            (Self::SelectClick, _) => "Select",
            (
                Self::TriggerValue
                | Self::TriggerTouch
                | Self::TriggerClick
                | Self::TriggerProximity
                | Self::TriggerForce
                | Self::TriggerCurl
                | Self::TriggerSlide,
                _,
            ) => "Trigger",
            (Self::SqueezeValue | Self::SqueezeClick, _) => "Grip",
            (
                Self::Thumbstick
                | Self::ThumbstickX
                | Self::ThumbstickY
                | Self::ThumbstickClick
                | Self::ThumbstickTouch
                | Self::ThumbstickDpadUp
                | Self::ThumbstickDpadDown
                | Self::ThumbstickDpadLeft
                | Self::ThumbstickDpadRight,
                _,
            ) => "Thumbstick",
            (Self::ThumbrestTouch | Self::ThumbrestForce | Self::ThumbProximity, _) => "Thumbrest",
            (Self::StylusForce, _) => "Stylus",
            (Self::Pose(PoseSource::Grip), _) => "Grip Pose",
            (Self::Pose(PoseSource::Aim), _) => "Aim Pose",
            (Self::Pose(PoseSource::GripSurface), _) => "Grip Surface Pose",
            (Self::Pose(PoseSource::PalmExt), _) => "Palm Pose",
            (Self::Pose(PoseSource::PinchExt), _) => "Pinch Pose",
            (Self::Pose(PoseSource::PokeExt), _) => "Poke Pose",
            (Self::Haptic, _) => "Haptic",
        }
    }
}

/// Pair of target hand and canonical component resolved from an OpenXR binding path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ResolvedBinding {
    pub hand: HandSide,
    pub component: VirtualComponent,
}

impl ResolvedBinding {
    pub fn evaluate_bool(
        &self,
        state: &VirtualHandState,
        hysteresis: &mut HysteresisState,
        threshold: f32,
        deviation: f32,
    ) -> Option<bool> {
        self.component
            .evaluate_bool(state, hysteresis, threshold, deviation)
    }

    pub fn evaluate_float(&self, state: &VirtualHandState) -> Option<f32> {
        self.component.evaluate_float(state)
    }

    pub fn evaluate_vec2(&self, state: &VirtualHandState) -> Option<Vec2> {
        self.component.evaluate_vec2(state)
    }

    pub const fn pose_source(&self) -> Option<PoseSource> {
        self.component.pose_source()
    }

    pub const fn is_haptic(&self) -> bool {
        self.component.is_haptic()
    }

    pub const fn localized_component_name(&self) -> &'static str {
        self.component.localized_component_name(self.hand)
    }
}

/// Parses an exact OpenXR hand top-level user path (`"/user/hand/left"` or `"/user/hand/right"`).
pub fn parse_hand_user_path(path: &str) -> Option<HandSide> {
    match path {
        "/user/hand/left" => Some(HandSide::Left),
        "/user/hand/right" => Some(HandSide::Right),
        _ => None,
    }
}

fn split_hand_binding_path(binding_path: &str) -> Option<(HandSide, &str)> {
    if binding_path.ends_with('/') || binding_path.contains("//") {
        return None;
    }
    if let Some(rest) = binding_path.strip_prefix("/user/hand/left") {
        if rest.starts_with('/') {
            return Some((HandSide::Left, rest));
        }
    }
    if let Some(rest) = binding_path.strip_prefix("/user/hand/right") {
        if rest.starts_with('/') {
            return Some((HandSide::Right, rest));
        }
    }
    None
}

fn resolve_universal_pose(subpath: &str) -> Option<PoseSource> {
    match subpath {
        "/input/grip/pose" | "/input/grip" => Some(PoseSource::Grip),
        "/input/aim/pose" | "/input/aim" => Some(PoseSource::Aim),
        "/input/grip_surface/pose" | "/input/grip_surface" => Some(PoseSource::GripSurface),
        "/input/palm_ext/pose" | "/input/palm_ext" => Some(PoseSource::PalmExt),
        "/input/pinch_ext/pose" | "/input/pinch_ext" => Some(PoseSource::PinchExt),
        "/input/poke_ext/pose" | "/input/poke_ext" => Some(PoseSource::PokeExt),
        _ => None,
    }
}

/// Resolves an OpenXR binding path under `profile` into a canonical [`ResolvedBinding`],
/// enforcing profile-specific and hand-specific rules from Khronos `xr.xml`.
pub fn resolve_binding(profile: InteractionProfile, binding_path: &str) -> Option<ResolvedBinding> {
    let (hand, subpath) = split_hand_binding_path(binding_path)?;

    // Universal pose subpaths are supported across all 9 profiles on both hands.
    if let Some(pose_src) = resolve_universal_pose(subpath) {
        return Some(ResolvedBinding {
            hand,
            component: VirtualComponent::Pose(pose_src),
        });
    }

    if profile == InteractionProfile::KhrSimple {
        let component = match subpath {
            "/input/select/click" | "/input/select" => VirtualComponent::SelectClick,
            // KHR Simple allows `/input/menu/click` on BOTH hands.
            "/input/menu/click" | "/input/menu" => VirtualComponent::MenuClick,
            "/output/haptic" => VirtualComponent::Haptic,
            _ => return None,
        };
        return Some(ResolvedBinding { hand, component });
    }

    // Touch-family hand-specific buttons
    match (hand, subpath) {
        (HandSide::Left, "/input/x/click" | "/input/x") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::PrimaryClick,
            });
        }
        (HandSide::Left, "/input/x/touch") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::PrimaryTouch,
            });
        }
        (HandSide::Left, "/input/y/click" | "/input/y") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::SecondaryClick,
            });
        }
        (HandSide::Left, "/input/y/touch") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::SecondaryTouch,
            });
        }
        (HandSide::Left, "/input/menu/click" | "/input/menu") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::MenuClick,
            });
        }
        (HandSide::Right, "/input/a/click" | "/input/a") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::PrimaryClick,
            });
        }
        (HandSide::Right, "/input/a/touch") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::PrimaryTouch,
            });
        }
        (HandSide::Right, "/input/b/click" | "/input/b") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::SecondaryClick,
            });
        }
        (HandSide::Right, "/input/b/touch") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::SecondaryTouch,
            });
        }
        (HandSide::Right, "/input/system/click" | "/input/system") => {
            return Some(ResolvedBinding {
                hand,
                component: VirtualComponent::SystemClick,
            });
        }
        _ => {}
    }

    // Common Touch-family components on both hands
    let common = match subpath {
        "/input/trigger/value" | "/input/trigger" => Some(VirtualComponent::TriggerValue),
        "/input/trigger/touch" => Some(VirtualComponent::TriggerTouch),
        "/input/trigger/click" => Some(VirtualComponent::TriggerClick),
        "/input/squeeze/value" | "/input/squeeze" => Some(VirtualComponent::SqueezeValue),
        "/input/squeeze/click" => Some(VirtualComponent::SqueezeClick),
        "/input/thumbstick" => Some(VirtualComponent::Thumbstick),
        "/input/thumbstick/x" => Some(VirtualComponent::ThumbstickX),
        "/input/thumbstick/y" => Some(VirtualComponent::ThumbstickY),
        "/input/thumbstick/click" => Some(VirtualComponent::ThumbstickClick),
        "/input/thumbstick/touch" => Some(VirtualComponent::ThumbstickTouch),
        "/input/thumbstick/dpad_up" => Some(VirtualComponent::ThumbstickDpadUp),
        "/input/thumbstick/dpad_down" => Some(VirtualComponent::ThumbstickDpadDown),
        "/input/thumbstick/dpad_left" => Some(VirtualComponent::ThumbstickDpadLeft),
        "/input/thumbstick/dpad_right" => Some(VirtualComponent::ThumbstickDpadRight),
        "/input/thumbrest/touch" if profile.has_thumbrest_touch() => {
            Some(VirtualComponent::ThumbrestTouch)
        }
        "/output/haptic" => Some(VirtualComponent::Haptic),
        _ => None,
    };
    if let Some(component) = common {
        return Some(ResolvedBinding { hand, component });
    }

    // Profile-specific Touch Plus / Touch Pro extensions (1.0 ext vs 1.1 core)
    let extended = match (profile, subpath) {
        // OpenXR 1.1 Meta Touch Plus
        (InteractionProfile::MetaTouchPlus11, "/input/trigger/force") => {
            Some(VirtualComponent::TriggerForce)
        }
        (InteractionProfile::MetaTouchPlus11, "/input/trigger/proximity") => {
            Some(VirtualComponent::TriggerProximity)
        }
        (
            InteractionProfile::MetaTouchPlus11,
            "/input/trigger_curl/value" | "/input/trigger_curl",
        ) => Some(VirtualComponent::TriggerCurl),
        (
            InteractionProfile::MetaTouchPlus11,
            "/input/trigger_slide/value" | "/input/trigger_slide",
        ) => Some(VirtualComponent::TriggerSlide),
        (InteractionProfile::MetaTouchPlus11, "/input/thumb_resting_surfaces/proximity") => {
            Some(VirtualComponent::ThumbProximity)
        }

        // OpenXR 1.0 XR_META_touch_controller_plus
        (InteractionProfile::MetaTouchPlusExt, "/input/trigger/force") => {
            Some(VirtualComponent::TriggerForce)
        }
        (InteractionProfile::MetaTouchPlusExt, "/input/trigger/proximity_meta") => {
            Some(VirtualComponent::TriggerProximity)
        }
        (InteractionProfile::MetaTouchPlusExt, "/input/trigger/curl_meta") => {
            Some(VirtualComponent::TriggerCurl)
        }
        (InteractionProfile::MetaTouchPlusExt, "/input/trigger/slide_meta") => {
            Some(VirtualComponent::TriggerSlide)
        }
        (InteractionProfile::MetaTouchPlusExt, "/input/thumb_meta/proximity_meta") => {
            Some(VirtualComponent::ThumbProximity)
        }

        // OpenXR 1.1 Meta Touch Pro
        (InteractionProfile::MetaTouchPro11, "/input/trigger/proximity") => {
            Some(VirtualComponent::TriggerProximity)
        }
        (
            InteractionProfile::MetaTouchPro11,
            "/input/trigger_curl/value" | "/input/trigger_curl",
        ) => Some(VirtualComponent::TriggerCurl),
        (
            InteractionProfile::MetaTouchPro11,
            "/input/trigger_slide/value" | "/input/trigger_slide",
        ) => Some(VirtualComponent::TriggerSlide),
        (InteractionProfile::MetaTouchPro11, "/input/thumb_resting_surfaces/proximity") => {
            Some(VirtualComponent::ThumbProximity)
        }
        (InteractionProfile::MetaTouchPro11, "/input/thumbrest/force") => {
            Some(VirtualComponent::ThumbrestForce)
        }
        (InteractionProfile::MetaTouchPro11, "/input/stylus/force") => {
            Some(VirtualComponent::StylusForce)
        }
        (
            InteractionProfile::MetaTouchPro11,
            "/output/haptic_trigger" | "/output/haptic_thumb",
        ) => Some(VirtualComponent::Haptic),

        // OpenXR 1.0 XR_FB_touch_controller_pro
        (InteractionProfile::FbTouchProExt, "/input/trigger/proximity_fb") => {
            Some(VirtualComponent::TriggerProximity)
        }
        (InteractionProfile::FbTouchProExt, "/input/trigger/curl_fb") => {
            Some(VirtualComponent::TriggerCurl)
        }
        (InteractionProfile::FbTouchProExt, "/input/trigger/slide_fb") => {
            Some(VirtualComponent::TriggerSlide)
        }
        (InteractionProfile::FbTouchProExt, "/input/thumb_fb/proximity_fb") => {
            Some(VirtualComponent::ThumbProximity)
        }
        (InteractionProfile::FbTouchProExt, "/input/thumbrest/force") => {
            Some(VirtualComponent::ThumbrestForce)
        }
        (InteractionProfile::FbTouchProExt, "/input/stylus_fb/force") => {
            Some(VirtualComponent::StylusForce)
        }
        (
            InteractionProfile::FbTouchProExt,
            "/output/haptic_trigger_fb" | "/output/haptic_thumb_fb",
        ) => Some(VirtualComponent::Haptic),

        _ => None,
    };

    extended.map(|component| ResolvedBinding { hand, component })
}

/// OpenXR multi-source boolean aggregation (`input.adoc`): logical OR (`false` if empty).
pub fn aggregate_bool(values: impl IntoIterator<Item = bool>) -> bool {
    values.into_iter().any(|v| v)
}

/// OpenXR multi-source float aggregation (`input.adoc`): finite value with largest `|v|`,
/// preserving sign (`0.0` if empty or all non-finite).
pub fn aggregate_float(values: impl IntoIterator<Item = f32>) -> f32 {
    let mut best = 0.0_f32;
    let mut best_abs = -1.0_f32;
    for v in values {
        if v.is_finite() {
            let a = v.abs();
            if a > best_abs {
                best_abs = a;
                best = v;
            }
        }
    }
    best
}

/// OpenXR multi-source Vector2f aggregation (`input.adoc`): finite vector with largest
/// `length_squared()` (`Vec2::ZERO` if empty or all non-finite).
pub fn aggregate_vec2(values: impl IntoIterator<Item = Vec2>) -> Vec2 {
    let mut best = Vec2::ZERO;
    let mut best_len_sq = -1.0_f32;
    for v in values {
        if v.is_finite() {
            let lsq = v.length_squared();
            if lsq.is_finite() && lsq > best_len_sq {
                best_len_sq = lsq;
                best = v;
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_nine_interaction_profiles_round_trip_priority_and_selection() {
        assert_eq!(InteractionProfile::ALL_BY_PRIORITY.len(), 9);
        for (idx, profile) in InteractionProfile::ALL_BY_PRIORITY.iter().enumerate() {
            assert_eq!(profile.priority(), (idx as u8) + 1);
            assert_eq!(
                InteractionProfile::from_path(profile.as_path()),
                Some(*profile)
            );
            assert!(!profile.localized_name().is_empty());
        }
        assert_eq!(
            InteractionProfile::from_path("/interaction_profiles/valve/index_controller"),
            None
        );

        // select_preferred picks highest-priority suggested profile ONLY
        assert_eq!(
            InteractionProfile::select_preferred([
                "/interaction_profiles/khr/simple_controller",
                "/interaction_profiles/oculus/touch_controller",
            ]),
            Some(InteractionProfile::OculusTouch)
        );
        assert_eq!(
            InteractionProfile::select_preferred([
                "/interaction_profiles/valve/index_controller",
                "/interaction_profiles/meta/touch_controller_plus",
                "/interaction_profiles/meta/touch_plus_controller",
            ]),
            Some(InteractionProfile::MetaTouchPlus11)
        );
        assert_eq!(
            InteractionProfile::select_preferred(["/interaction_profiles/khr/simple_controller"]),
            Some(InteractionProfile::KhrSimple)
        );
        assert_eq!(
            InteractionProfile::select_preferred(["/interaction_profiles/unknown/foo"]),
            None
        );
    }

    #[test]
    fn complete_component_path_matrix_and_hand_asymmetry() {
        assert_eq!(
            parse_hand_user_path("/user/hand/left"),
            Some(HandSide::Left)
        );
        assert_eq!(
            parse_hand_user_path("/user/hand/right"),
            Some(HandSide::Right)
        );
        assert_eq!(parse_hand_user_path("/user/head"), None);

        // Malformed paths rejected
        for bad in [
            "/user/hand/left",
            "/user/hand/left/",
            "/user/hand/left//input/x/click",
            "/user/hand/left/input/x/click/",
            "/user/head/input/x/click",
        ] {
            assert_eq!(
                resolve_binding(InteractionProfile::OculusTouch, bad),
                None,
                "expected None for {bad}"
            );
        }

        // Core OculusTouch buttons & hand-side enforcement
        assert_eq!(
            resolve_binding(InteractionProfile::OculusTouch, "/user/hand/left/input/x/click"),
            Some(ResolvedBinding {
                hand: HandSide::Left,
                component: VirtualComponent::PrimaryClick
            })
        );
        assert_eq!(
            resolve_binding(InteractionProfile::OculusTouch, "/user/hand/right/input/x/click"),
            None
        );
        assert_eq!(
            resolve_binding(InteractionProfile::OculusTouch, "/user/hand/right/input/a/click"),
            Some(ResolvedBinding {
                hand: HandSide::Right,
                component: VirtualComponent::PrimaryClick
            })
        );
        assert_eq!(
            resolve_binding(InteractionProfile::OculusTouch, "/user/hand/left/input/a/click"),
            None
        );
        assert_eq!(
            resolve_binding(InteractionProfile::OculusTouch, "/user/hand/left/input/y/touch"),
            Some(ResolvedBinding {
                hand: HandSide::Left,
                component: VirtualComponent::SecondaryTouch
            })
        );
        assert_eq!(
            resolve_binding(InteractionProfile::OculusTouch, "/user/hand/right/input/b/touch"),
            Some(ResolvedBinding {
                hand: HandSide::Right,
                component: VirtualComponent::SecondaryTouch
            })
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::OculusTouch,
                "/user/hand/right/input/system/click"
            ),
            Some(ResolvedBinding {
                hand: HandSide::Right,
                component: VirtualComponent::SystemClick
            })
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::OculusTouch,
                "/user/hand/left/input/system/click"
            ),
            None
        );

        // Menu: Left-only on Touch, BOTH hands on KHR Simple
        assert_eq!(
            resolve_binding(
                InteractionProfile::OculusTouch,
                "/user/hand/left/input/menu/click"
            )
            .map(|r| r.component),
            Some(VirtualComponent::MenuClick)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::OculusTouch,
                "/user/hand/right/input/menu/click"
            ),
            None
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::KhrSimple,
                "/user/hand/right/input/menu/click"
            )
            .map(|r| r.component),
            Some(VirtualComponent::MenuClick)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::KhrSimple,
                "/user/hand/left/input/select/click"
            )
            .map(|r| r.component),
            Some(VirtualComponent::SelectClick)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::OculusTouch,
                "/user/hand/left/input/select/click"
            ),
            None
        );

        // Thumbrest touch accepted on OculusTouch/Quest2 and rejected on Quest1/RiftS
        assert_eq!(
            resolve_binding(
                InteractionProfile::OculusTouch,
                "/user/hand/left/input/thumbrest/touch"
            )
            .map(|r| r.component),
            Some(VirtualComponent::ThumbrestTouch)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::MetaTouchQuest2,
                "/user/hand/right/input/thumbrest/touch"
            )
            .map(|r| r.component),
            Some(VirtualComponent::ThumbrestTouch)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::MetaTouchQuest1RiftS,
                "/user/hand/left/input/thumbrest/touch"
            ),
            None
        );

        // Touch Plus 1.0 ext vs 1.1 core
        assert_eq!(
            resolve_binding(
                InteractionProfile::MetaTouchPlusExt,
                "/user/hand/left/input/trigger/curl_meta"
            )
            .map(|r| r.component),
            Some(VirtualComponent::TriggerCurl)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::MetaTouchPlusExt,
                "/user/hand/left/input/trigger/proximity_meta"
            )
            .map(|r| r.component),
            Some(VirtualComponent::TriggerProximity)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::MetaTouchPlusExt,
                "/user/hand/left/input/thumb_meta/proximity_meta"
            )
            .map(|r| r.component),
            Some(VirtualComponent::ThumbProximity)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::MetaTouchPlus11,
                "/user/hand/left/input/trigger_curl/value"
            )
            .map(|r| r.component),
            Some(VirtualComponent::TriggerCurl)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::MetaTouchPlus11,
                "/user/hand/left/input/trigger/proximity"
            )
            .map(|r| r.component),
            Some(VirtualComponent::TriggerProximity)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::MetaTouchPlus11,
                "/user/hand/left/input/thumb_resting_surfaces/proximity"
            )
            .map(|r| r.component),
            Some(VirtualComponent::ThumbProximity)
        );

        // Touch Pro 1.0 ext vs 1.1 core
        assert_eq!(
            resolve_binding(
                InteractionProfile::FbTouchProExt,
                "/user/hand/right/input/trigger/curl_fb"
            )
            .map(|r| r.component),
            Some(VirtualComponent::TriggerCurl)
        );
        assert_eq!(
            resolve_binding(
                InteractionProfile::FbTouchProExt,
                "/user/hand/right/input/stylus_fb/force"
            )
            .map(|r| r.component),
            Some(VirtualComponent::StylusForce)
        );
        assert!(resolve_binding(
            InteractionProfile::FbTouchProExt,
            "/user/hand/right/output/haptic_trigger_fb"
        )
        .unwrap()
        .is_haptic());
        assert_eq!(
            resolve_binding(
                InteractionProfile::MetaTouchPro11,
                "/user/hand/right/input/stylus/force"
            )
            .map(|r| r.component),
            Some(VirtualComponent::StylusForce)
        );
        assert!(resolve_binding(
            InteractionProfile::MetaTouchPro11,
            "/user/hand/right/output/haptic_trigger"
        )
        .unwrap()
        .is_haptic());

        // XR_EXT_dpad_binding
        for (p, expected) in [
            (
                "/user/hand/left/input/thumbstick/dpad_up",
                VirtualComponent::ThumbstickDpadUp,
            ),
            (
                "/user/hand/left/input/thumbstick/dpad_down",
                VirtualComponent::ThumbstickDpadDown,
            ),
            (
                "/user/hand/left/input/thumbstick/dpad_left",
                VirtualComponent::ThumbstickDpadLeft,
            ),
            (
                "/user/hand/left/input/thumbstick/dpad_right",
                VirtualComponent::ThumbstickDpadRight,
            ),
        ] {
            assert_eq!(
                resolve_binding(InteractionProfile::OculusTouch, p).map(|r| r.component),
                Some(expected)
            );
        }

        // Universal poses across ALL 9 profiles
        for profile in InteractionProfile::ALL_BY_PRIORITY {
            for (sub, src) in [
                ("/user/hand/left/input/grip/pose", PoseSource::Grip),
                ("/user/hand/right/input/aim/pose", PoseSource::Aim),
                (
                    "/user/hand/left/input/grip_surface/pose",
                    PoseSource::GripSurface,
                ),
                ("/user/hand/right/input/palm_ext/pose", PoseSource::PalmExt),
                (
                    "/user/hand/left/input/pinch_ext/pose",
                    PoseSource::PinchExt,
                ),
                ("/user/hand/right/input/poke_ext/pose", PoseSource::PokeExt),
            ] {
                let res = resolve_binding(profile, sub).expect("universal pose");
                assert_eq!(res.pose_source(), Some(src));
            }
        }
    }

    #[test]
    fn parent_path_shorthands_and_openxr_type_conversions() {
        let mut state = VirtualHandState {
            tracked: true,
            trigger_value: 0.75,
            squeeze_value: 0.30,
            primary_click: true,
            select_click: true,
            thumbstick: Vec2::new(-0.2, 0.8),
            ..VirtualHandState::default()
        };

        // /input/trigger parent shorthand: Float -> trigger_value, Bool -> Hysteresis(trigger_value)
        let trig = resolve_binding(InteractionProfile::OculusTouch, "/user/hand/left/input/trigger")
            .unwrap();
        assert_eq!(trig.evaluate_float(&state), Some(0.75));
        let mut h = HysteresisState::new();
        assert_eq!(trig.evaluate_bool(&state, &mut h, 0.5, 0.05), Some(true));
        state.trigger_value = 0.48; // inside [0.45, 0.55] -> holds true
        assert_eq!(trig.evaluate_bool(&state, &mut h, 0.5, 0.05), Some(true));
        state.trigger_value = 0.40; // < 0.45 -> releases
        assert_eq!(trig.evaluate_bool(&state, &mut h, 0.5, 0.05), Some(false));
        assert_eq!(trig.evaluate_vec2(&state), None);

        // /input/squeeze parent shorthand
        let sqz = resolve_binding(
            InteractionProfile::OculusTouch,
            "/user/hand/right/input/squeeze",
        )
        .unwrap();
        assert_eq!(sqz.evaluate_float(&state), Some(0.30));
        let mut h_sqz = HysteresisState::new();
        assert_eq!(sqz.evaluate_bool(&state, &mut h_sqz, 0.5, 0.05), Some(false));

        // /input/x, /input/a, /input/select parent shorthands: Bool -> click, Float -> 0.0/1.0
        let x_btn =
            resolve_binding(InteractionProfile::OculusTouch, "/user/hand/left/input/x").unwrap();
        let a_btn =
            resolve_binding(InteractionProfile::OculusTouch, "/user/hand/right/input/a").unwrap();
        let sel = resolve_binding(
            InteractionProfile::KhrSimple,
            "/user/hand/left/input/select",
        )
        .unwrap();
        assert_eq!(x_btn.localized_component_name(), "X Button");
        assert_eq!(a_btn.localized_component_name(), "A Button");
        let mut dummy_h = HysteresisState::new();
        assert_eq!(x_btn.evaluate_bool(&state, &mut dummy_h, 0.5, 0.05), Some(true));
        assert_eq!(x_btn.evaluate_float(&state), Some(1.0));
        assert_eq!(a_btn.evaluate_float(&state), Some(1.0));
        assert_eq!(sel.evaluate_float(&state), Some(1.0));
        state.primary_click = false;
        assert_eq!(x_btn.evaluate_float(&state), Some(0.0));

        // /input/thumbstick: Vec2 -> Some, Bool/Float -> None
        let stick = resolve_binding(
            InteractionProfile::OculusTouch,
            "/user/hand/left/input/thumbstick",
        )
        .unwrap();
        assert_eq!(stick.evaluate_vec2(&state), Some(Vec2::new(-0.2, 0.8)));
        assert_eq!(stick.evaluate_float(&state), None);
        assert_eq!(stick.evaluate_bool(&state, &mut dummy_h, 0.5, 0.05), None);

        // D-pad up triggers for thumbstick (-0.2, 0.8)
        let dpad_up = resolve_binding(
            InteractionProfile::OculusTouch,
            "/user/hand/left/input/thumbstick/dpad_up",
        )
        .unwrap();
        assert_eq!(dpad_up.evaluate_bool(&state, &mut dummy_h, 0.5, 0.05), Some(true));

        // Pose parent shorthands
        for (p, src) in [
            ("/user/hand/left/input/grip", PoseSource::Grip),
            ("/user/hand/left/input/aim", PoseSource::Aim),
            ("/user/hand/left/input/grip_surface", PoseSource::GripSurface),
            ("/user/hand/left/input/palm_ext", PoseSource::PalmExt),
        ] {
            let b = resolve_binding(InteractionProfile::OculusTouch, p).unwrap();
            assert_eq!(b.pose_source(), Some(src));
            assert_eq!(b.evaluate_bool(&state, &mut dummy_h, 0.5, 0.05), None);
            assert_eq!(b.evaluate_float(&state), None);
            assert_eq!(b.evaluate_vec2(&state), None);
        }
    }

    #[test]
    fn multi_source_aggregation_rules() {
        assert!(!aggregate_bool([]));
        assert!(!aggregate_bool([false, false]));
        assert!(aggregate_bool([false, true, false]));

        assert_eq!(aggregate_float([]), 0.0);
        assert_eq!(aggregate_float([f32::NAN, f32::INFINITY]), 0.0);
        assert_eq!(aggregate_float([0.3, -0.8, 0.5, f32::NAN]), -0.8);
        assert_eq!(aggregate_float([-0.2, 0.9, -0.4]), 0.9);

        assert_eq!(aggregate_vec2([]), Vec2::ZERO);
        assert_eq!(
            aggregate_vec2([Vec2::new(f32::NAN, 1.0), Vec2::new(0.2, 0.2), Vec2::new(-0.9, 0.1)]),
            Vec2::new(-0.9, 0.1)
        );
    }
}
