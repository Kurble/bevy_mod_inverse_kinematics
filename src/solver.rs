use bevy::ecs::query::QueryEntityError;
use bevy::prelude::*;
#[cfg(feature = "debug_lines")]
use bevy_prototype_debug_lines::{DebugLines, DebugLinesPlugin};

use super::IkConstraint;

struct PoleTarget {
    origin: Vec3,
    tangent: Vec3,
    normal: Vec3,
    angle: f32,
}

pub fn inverse_kinematics_system(
    query: Query<(Entity, &IkConstraint)>,
    parents: Query<&Parent>,
    mut transforms: Query<(&mut Transform, &mut GlobalTransform)>,
    #[cfg(feature = "debug_lines")] mut debug_lines: ResMut<DebugLines>,
) {
    for (entity, constraint) in query.iter() {
        if let Err(e) = constraint.solve(
            entity,
            &parents,
            &mut transforms,
            #[cfg(feature = "debug_lines")]
            &mut debug_lines,
        ) {
            bevy::log::warn!("Failed to solve IK constraint: {e}");
        }
    }
}

impl IkConstraint {
    pub fn solve(
        &self,
        entity: Entity,
        parents: &Query<&Parent>,
        transforms: &mut Query<(&mut Transform, &mut GlobalTransform)>,
        #[cfg(feature = "debug_lines")] debug_lines: &mut DebugLines,
    ) -> Result<(), QueryEntityError> {
        if self.chain_length == 0 {
            return Ok(());
        }

        let mut joints = Vec::with_capacity(self.chain_length + 2);
        joints.push(entity);
        for i in 0..self.chain_length + 1 {
            joints.push(parents.get(joints[i])?.get());
        }

        let target = transforms.get(self.target)?.1.translation();
        let normal = transforms.get(joints[0])?.0.translation;

        let pole_target = if let Some(pole_target) = self.pole_target {
            let start = transforms.get(joints[self.chain_length])?.1.translation();
            let pole_target = transforms.get(pole_target)?.1.translation();

            #[cfg(feature = "debug_lines")]
            debug_lines.line_colored(start, target, 0.0, Color::WHITE);

            let tangent = (target - start).normalize();
            let axis = (pole_target - start).cross(tangent);
            let normal = tangent.cross(axis).normalize();

            Some(PoleTarget {
                origin: start,
                tangent,
                normal,
                angle: self.pole_angle,
            })
        } else {
            None
        };

        for _ in 0..self.iterations {
            let result = Self::solve_recursive(
                &joints[1..],
                normal,
                target,
                pole_target.as_ref(),
                transforms,
                #[cfg(feature = "debug_lines")]
                debug_lines,
            )?;

            if result.mul_vec3(normal).distance_squared(target) < 0.001 {
                return Ok(());
            }
        }

        Ok(())
    }

    fn solve_recursive(
        chain: &[Entity],
        normal: Vec3,
        target: Vec3,
        pole_target: Option<&PoleTarget>,
        transforms: &mut Query<(&mut Transform, &mut GlobalTransform)>,
        #[cfg(feature = "debug_lines")] debug_lines: &mut DebugLines,
    ) -> Result<GlobalTransform, QueryEntityError> {
        let (&transform, &global_transform) = transforms.get(chain[0])?;
        #[cfg(feature = "debug_lines")]
        {
            let p = global_transform.translation();
            debug_lines.line_colored(p, global_transform.mul_vec3(Vec3::X * 0.1), 0.0, Color::RED);
            debug_lines.line_colored(
                p,
                global_transform.mul_vec3(Vec3::Y * 0.1),
                0.0,
                Color::GREEN,
            );
            debug_lines.line_colored(
                p,
                global_transform.mul_vec3(Vec3::Z * 0.1),
                0.0,
                Color::BLUE,
            );
            debug_lines.line_colored(p, global_transform.mul_vec3(normal), 0.0, Color::YELLOW);
        }

        if chain.len() == 1 {
            return Ok(global_transform);
        }

        let parent_normal = transform.translation;

        // determine absolute rotation and translation for this bone where the tail touches the
        // target.
        let rotation = if let Some(pt) = pole_target {
            let on_pole = pt.origin
                + (global_transform.translation() - pt.origin).project_onto_normalized(pt.tangent);
            let distance = on_pole.distance(global_transform.translation());
            let from_position = on_pole + pt.normal * distance;

            let base = Quat::from_rotation_arc(normal.normalize(), Vec3::Z);

            let forward = (target - from_position)
                .try_normalize()
                .unwrap_or(pt.tangent);
            let up = forward.cross(pt.normal).normalize();
            let right = up.cross(forward);
            let orientation = Mat3::from_cols(right, up, forward) * Mat3::from_rotation_z(pt.angle);

            (Quat::from_mat3(&orientation) * base).normalize()
        } else {
            Quat::from_rotation_arc(
                normal.normalize(),
                (target - global_transform.translation()).normalize(),
            )
            .normalize()
        };
        let translation = target - rotation.mul_vec3(normal);

        // recurse to target the parent towards the current translation
        let parent_global_transform = Self::solve_recursive(
            &chain[1..],
            parent_normal,
            translation,
            pole_target,
            transforms,
            #[cfg(feature = "debug_lines")]
            debug_lines,
        )?;

        // apply constraints on the way back from recursing
        let (mut transform, mut global_transform) = transforms.get_mut(chain[0]).unwrap();
        transform.rotation = Quat::from_affine3(&parent_global_transform.affine())
            .inverse()
            .normalize()
            * rotation;
        *global_transform = parent_global_transform.mul_transform(*transform);

        #[cfg(feature = "debug_lines")]
        debug_lines.line_colored(
            global_transform.translation(),
            global_transform.mul_vec3(normal),
            0.0,
            Color::CYAN,
        );

        Ok(*global_transform)
    }
}
