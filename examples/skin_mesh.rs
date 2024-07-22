use bevy::{color::palettes::css, prelude::*, window::WindowResolution};
use bevy_mod_inverse_kinematics::*;

#[derive(Component)]
pub struct ManuallyTarget(Vec4);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: WindowResolution::new(800.0, 600.0),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(InverseKinematicsPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (setup_ik, manually_target))
        .run();
}

fn setup(
    mut commands: Commands,
    assets: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn(SpatialBundle::default())
        .with_children(|parent| {
            parent.spawn(Camera3dBundle {
                transform: Transform::from_xyz(-0.5, 1.5, 2.5)
                    .looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
                projection: bevy::render::camera::Projection::Perspective(PerspectiveProjection {
                    fov: std::f32::consts::FRAC_PI_4,
                    aspect_ratio: 1.0,
                    near: 0.1,
                    far: 100.0,
                }),
                ..default()
            });
        });

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: css::WHITE.into(),
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(-8.0, 8.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(Plane3d::default().mesh().size(5.0, 5.0))),
        material: materials.add(StandardMaterial {
            base_color: css::WHITE.into(),
            ..default()
        }),
        ..default()
    });

    commands.spawn(SceneBundle {
        scene: assets.load("skin.gltf#Scene0"),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    });
}

fn setup_ik(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    added_query: Query<(Entity, &Parent), Added<AnimationPlayer>>,
    children: Query<&Children>,
    names: Query<&Name>,
) {
    // Use the presence of `AnimationPlayer` to determine the root entity of the skeleton.
    for (entity, _parent) in added_query.iter() {
        // Try to get the entity for the right hand joint.
        let right_hand = find_entity(
            &vec![
                "Pelvis".into(),
                "Spine1".into(),
                "Spine2".into(),
                "Collar.R".into(),
                "UpperArm.R".into(),
                "ForeArm.R".into(),
                "Hand.R".into(),
            ],
            entity,
            &children,
            &names,
        )
        .unwrap();
        let target = commands
            .spawn((
                PbrBundle {
                    transform: Transform::from_xyz(0.3, 0.8, 0.2),
                    mesh: meshes.add(Sphere::new(0.05).mesh().uv(7, 7)),
                    material: materials.add(StandardMaterial {
                        base_color: css::RED.into(),
                        ..default()
                    }),
                    ..default()
                },
                ManuallyTarget(Vec4::new(0.0, 0.0, 1.0, 0.3)),
            ))
            .id();

        let pole_target = commands
            .spawn(PbrBundle {
                transform: Transform::from_xyz(-1.0, 0.4, -0.2),
                mesh: meshes.add(Sphere::new(0.05).mesh().uv(7, 7)),
                material: materials.add(StandardMaterial {
                    base_color: css::LIME.into(),
                    ..default()
                }),
                ..default()
            })
            .id();

        // Add an IK constraint to the right hand, using the targets that were created earlier.
        commands.entity(right_hand).insert(IkConstraint {
            chain_length: 2,
            iterations: 20,
            target,
            pole_target: Some(pole_target),
            pole_angle: -std::f32::consts::FRAC_PI_2,
            enabled: true,
        });
    }
}

fn find_entity(
    path: &Vec<Name>,
    root: Entity,
    children: &Query<&Children>,
    names: &Query<&Name>,
) -> Result<Entity, ()> {
    let mut current_entity = root;

    for part in path.iter() {
        let mut found = false;
        if let Ok(children) = children.get(current_entity) {
            for child in children.iter() {
                if let Ok(name) = names.get(*child) {
                    if name == part {
                        // Found a children with the right name, continue to the next part
                        current_entity = *child;
                        found = true;
                        break;
                    }
                }
            }
        }
        if !found {
            warn!("Entity not found for path {:?} on part {:?}", path, part);
            return Err(());
        }
    }

    Ok(current_entity)
}

fn manually_target(
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut target_query: Query<(&ManuallyTarget, &mut Transform)>,
    mut cursor: EventReader<CursorMoved>,
) {
    let (camera, transform) = camera_query.single();

    if let Some(event) = cursor.read().last() {
        let view = transform.compute_matrix();
        let viewport_rect = camera.logical_viewport_rect().unwrap();
        let viewport_size = viewport_rect.size();
        let adj_cursor_pos = event.position - Vec2::new(viewport_rect.min.x, viewport_rect.min.y);

        let projection = camera.clip_from_view();
        let far_ndc = projection.project_point3(Vec3::NEG_Z).z;
        let near_ndc = projection.project_point3(Vec3::Z).z;
        let cursor_ndc =
            ((adj_cursor_pos / viewport_size) * 2.0 - Vec2::ONE) * Vec2::new(1.0, -1.0);
        let ndc_to_world: Mat4 = view * projection.inverse();
        let near = ndc_to_world.project_point3(cursor_ndc.extend(near_ndc));
        let far = ndc_to_world.project_point3(cursor_ndc.extend(far_ndc));
        let ray_direction = far - near;

        for (&ManuallyTarget(plane), mut transform) in target_query.iter_mut() {
            let normal = plane.truncate();
            let d = plane.w;
            let denom = normal.dot(ray_direction);
            if denom.abs() > 0.0001 {
                let t = (normal * d - near).dot(normal) / denom;
                transform.translation = near + ray_direction * t;
            }
        }
    }
}
