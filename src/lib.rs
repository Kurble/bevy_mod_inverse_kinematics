use std::ops::Range;

use bevy::ecs::query::QueryEntityError;
use bevy::prelude::*;
use bevy::transform::TransformSystem;

pub struct InverseKinematicsPlugin;

/// Inverse kintematics constraint to be added to the tail joint of
/// the bone chain that should be constrained.
#[derive(Component)]
pub struct IkConstraint {
    /// How many bones are included in the IK constraint.
    pub chain_length: usize,
    /// Maximum number of iterations to solve this constraint.
    pub iterations: usize,
    /// Target entity. The target must have a `Transform` and `GlobalTransform`.
    pub target: Entity,
    /// Target entity for pole rotation.
    pub pole_target: Option<Entity>,
    /// Offset for pole rotation in radians.
    pub pole_angle: f32,
}

#[derive(Component)]
pub struct CopyLocationConstraint {
    /// Target entity. The target must have a `Transform` and `GlobalTransform`.
    pub target: Entity,
}

#[derive(Component)]
pub struct RotationConstraint {
    /// The allowable range for yaw rotation.
    pub yaw: Range<f32>,
    /// The allowable range for pitch rotation.
    pub pitch: Range<f32>,
    /// The allowable range for roll rotation.
    pub roll: Range<f32>,
}

pub fn inverse_kinematics_solver_system(
    query: Query<(Entity, &IkConstraint)>,
    parents: Query<&Parent>,
    mut transforms: Query<(&mut Transform, &mut GlobalTransform)>,
) {
    for (entity, constraint) in query.iter() {
        if let Err(e) = constraint.solve(entity, &parents, &mut transforms) {
            bevy::log::warn!("Failed to solve IK constraint: {e}");
        }
    }
}

impl IkConstraint {
    fn solve(
        &self,
        entity: Entity,
        parents: &Query<&Parent>,
        transforms: &mut Query<(&mut Transform, &mut GlobalTransform)>,
    ) -> Result<(), QueryEntityError> {
        if self.chain_length == 0 {
            return Ok(());
        }

        let mut joints = Vec::with_capacity(self.chain_length + 2);
        joints.push(entity);
        for i in 0..self.chain_length + 1 {
            joints.push(parents.get(joints[i])?.get());
        }

        let start = transforms
            .get(joints[self.chain_length - 1])?
            .1
            .translation();
        let end = transforms.get(self.target)?.1.translation();

        let target = end;
        let normal = transforms.get(joints[0])?.0.translation;
        let pole_target = None;

        Self::solve_recursive(&joints[1..], normal, target, pole_target, transforms).map(drop)
    }

    fn solve_recursive(
        chain: &[Entity],
        normal: Vec3,
        target: Vec3,
        pole_target: Option<Vec3>,
        transforms: &mut Query<(&mut Transform, &mut GlobalTransform)>,
    ) -> Result<GlobalTransform, QueryEntityError> {
        if chain.len() == 1 {
            let (_, &global_transform) = transforms.get(chain[0])?;
            return Ok(global_transform);
        }

        let (&transform, &global_transform) = transforms.get(chain[0])?;
        let parent_normal = transform.translation;

        // determine absolute rotation and translation for this bone where the tail touches the 
        // target.
        let rotation = Quat::from_rotation_arc(
            normal.normalize(),
            // todo: constrain our own position to the pole plane maybe??
            (target - global_transform.translation()).normalize(),
        );
        let translation = target - rotation.mul_vec3(normal);

        // recurse to target the parent towards the current translation
        let parent_global_transform =
            Self::solve_recursive(&chain[1..], parent_normal, translation, pole_target, transforms)?;

        // apply constraints on the way back from recursing
        let (mut transform, mut global_transform) = transforms.get_mut(chain[0]).unwrap();
        transform.rotation =
            Quat::from_affine3(&parent_global_transform.affine().inverse()) * rotation;
        *global_transform = parent_global_transform.mul_transform(*transform);

        Ok(*global_transform)
    }
}

impl Plugin for InverseKinematicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_system_to_stage(
            CoreStage::PostUpdate,
            inverse_kinematics_solver_system.after(TransformSystem::TransformPropagate),
        );
    }
}
