use std::num::NonZeroUsize;
use std::ops::Range;

use bevy::ecs::query::QueryEntityError;
use bevy::transform::TransformSystem;
use bevy::prelude::*;

pub struct InverseKinematicsPlugin;

/// Inverse kintematics constraint to be added to the tail joint of
/// the bone chain that should be constrained.
#[derive(Component)]
pub struct IkConstraint {
    /// How many bones are included in the IK constraint.
    pub chain_length: NonZeroUsize,
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
    constraints_ik: Query<(Entity, &IkConstraint)>,
    mut params: ParamSet<(
        Query<&GlobalTransform>,
        Query<(&mut Transform, &mut GlobalTransform, &Parent)>,
    )>,
) {
    for (entity, constraint) in constraints_ik.iter() {
        if let Err(e) = constraint.solve(entity, &mut params) {
            bevy::log::warn!("Failed to solve IK constraint: {e}");
        }
    }
}

impl IkConstraint {
    fn solve(
        &self,
        entity: Entity,
        params: &mut ParamSet<(
            Query<&GlobalTransform>,
            Query<(&mut Transform, &mut GlobalTransform, &Parent)>,
        )>,
    ) -> Result<(), QueryEntityError> {
        let (bone, parent) = params.p1()
            .get(entity)
            .map(|(t, _, p)| (t.translation, p.get()))?;
        let tail = params.p0().get(self.target).map(GlobalTransform::translation)?;
        let pole = if let Some(entity) = self.pole_target {
            Some(params.p0().get(entity).map(GlobalTransform::translation)?)
        } else {
            None
        };
        Self::solve_recursive(parent, bone, tail, pole, &mut params.p1(), self.chain_length.get())?;
        Ok(())
    }

    fn solve_recursive(
        // the entity to rotate
        entity: Entity,
        // the translation vector of this bone
        bone: Vec3,
        // the desired tail of this bone
        tail: Vec3,
        // the desired up vector of this bone
        pole: Option<Vec3>,
        query: &mut Query<(&mut Transform, &mut GlobalTransform, &Parent)>,
        chain: usize,
    ) -> Result<GlobalTransform, QueryEntityError> {
        if chain == 0 {
            let (_, &global_transform, _) = query.get(entity)?;
            return Ok(global_transform);
        }

        let (transform, global_transform, parent) =
            query.get(entity).map(|(&t, &g, p)| (t, g, p.get()))?;
        // the bone vector of the parent is the translation of this bone
        let parent_bone = transform.translation;

        // calculate absolute rotation in order to point this bone at the desired tail
        let rotation = Quat::from_rotation_arc(
            bone.normalize(),
            (tail - global_transform.translation()).normalize(),
        );
        // calculate absolute translation so the tip of this bone touches the desired tail.
        let head = tail - rotation.mul_vec3(bone);

        // pass the targets to the parent to obtain the final global transform of the parent
        let parent_global_transform =
            Self::solve_recursive(parent, parent_bone, head, pole, query, chain - 1)?;

        // determine the relative translation for this bone
        let (mut transform, mut global_transform, _) = query.get_mut(entity).unwrap();
        transform.rotation = Quat::from_affine3(&parent_global_transform.affine().inverse()) * rotation;
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
