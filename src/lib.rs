use bevy::prelude::*;

mod solver;

/// The inverse kinematics plugin.
/// Should be added to your app if you want to use this crate.
pub struct InverseKinematicsPlugin;

/// Inverse kintematics constraint to be added to the tail joint of
/// a chain of bones. The solver will attempt to match the global translation of
/// of this entity to that of the target.
#[derive(Component, Reflect)]
pub struct IkConstraint {
    /// How many bones are included in the IK constraint.
    pub chain_length: usize,
    /// Maximum number of iterations to solve this constraint.
    pub iterations: usize,
    /// Target entity. The target must have a `Transform` and `GlobalTransform`.
    pub target: Entity,
    /// Target entity for pole rotation.
    /// If this would be a person's arm, this is the direction that the elbow will point.
    pub pole_target: Option<Entity>,
    /// Pole target roll offset.
    /// If a pole target is set, the bone will roll toward the pole target.
    /// This angle is the offset to apply to the roll.
    pub pole_angle: f32,
    /// Whether this constraint is enabled. Disabled constraints will be skipped.
    pub enabled: bool,
}

impl Plugin for InverseKinematicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, solver::inverse_kinematics_system);
        app.register_type::<IkConstraint>();
    }
}
