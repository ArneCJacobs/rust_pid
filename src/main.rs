use bevy::prelude::*;
use bevy_inspector_egui::{quick::{WorldInspectorPlugin, ResourceInspectorPlugin}, prelude::ReflectInspectorOptions, InspectorOptions};
use bevy_xpbd_3d::prelude::*;

#[derive(Reflect, Resource, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
struct PIDConfiguration {
    #[inspector(min = 0.0, max = 10.0)]
    proportional_gain: f32,

    #[inspector(min = 0.0, max = 10.0)]
    integral_gain: f32,

    #[inspector(min = 0.0, max = 10.0)]
    derivative_gain: f32,

}



impl Default for PIDConfiguration {
    fn default() -> Self {
        Self {
            proportional_gain: 1.0,
            integral_gain: 1.0,
            derivative_gain: 1.0,
        }
    }
}

impl PIDConfiguration {
    pub fn update(&self, dt: f32, current_position: Vec3, target_position: Vec3) -> Vec3 {
        let error = target_position - current_position;
        let proportional = error * self.proportional_gain;
        // let integral = error * self.integral_gain;
        // let derivative = error * self.derivative_gain;
        proportional // + integral + derivative
    }
}

#[derive(Reflect, Resource, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
struct TargetConfiguration {
    target_position: Vec3,
}

impl Default for TargetConfiguration {
    fn default() -> Self {
        Self {
            target_position: Vec3::new(0.0, 3.0, 0.0),
        }
    }
}


#[derive(Reflect, Clone, Copy, Component, Debug, Default, PartialEq, Eq)]
struct CubeTag;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_plugins(WorldInspectorPlugin::new())

        .init_resource::<PIDConfiguration>() // `ResourceInspectorPlugin` won't initialize the resource
        .register_type::<PIDConfiguration>() // you need to register your type to display it
        .add_plugins(ResourceInspectorPlugin::<PIDConfiguration>::default())

        .init_resource::<TargetConfiguration>() // `ResourceInspectorPlugin` won't initialize the resource
        .register_type::<TargetConfiguration>() // you need to register your type to display it
        .add_plugins(ResourceInspectorPlugin::<TargetConfiguration>::default())

        .insert_resource(Gravity(Vec3::new(0.0, 0.0, 0.0)))
        .add_systems(Startup, setup)
        .add_systems(Update, update)
        .add_systems(Update, draw_target)
        .run();
}


fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Plane
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(8.0, 0.002, 8.0),
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Plane::from_size(8.0))),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
            ..default()
        },
    ));
    // Cube
    commands.spawn((
        CubeTag,
        RigidBody::Dynamic,
        // AngularVelocity(Vec3::new(2.5, 3.4, 1.6)),
        Collider::cuboid(1.0, 1.0, 1.0),
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
            material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
            transform: Transform::from_xyz(0.0, 4.0, 0.0),
            ..default()
        },
        Mass(10.0),
        ExternalForce::default(),
        LockedAxes::new()
            .lock_rotation_x()
            .lock_rotation_y()
            .lock_rotation_z(),
    ));
    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 2.0, 8.0).looking_to(Vec3::NEG_Z, Vec3::Y),
        ..default()
    });
}

fn draw_target(
    mut gizmos: Gizmos,
    target: Res<TargetConfiguration>,
) {
    gizmos.cuboid(Transform::from_translation(target.target_position), Color::GREEN);
}

fn update(
    mut cube_query: Query<(&Position, &mut ExternalForce), With<CubeTag>>,
    time: Res<Time>,
    mut gizmos: Gizmos,
    pid_controller: Res<PIDConfiguration>,
    target_config: Res<TargetConfiguration>,
) {
    let force_fector = Vec3::Y * 10.0;
    for (position, mut force) in cube_query.iter_mut() {
        // let force_vector = (Vec3::Y * 10.0) * (time.elapsed_seconds() / 2.0).sin().abs();
        // println!("position: {:?}, force_fector: {:?}", position.0, force);
        let correction = pid_controller.update(time.delta_seconds(), position.0, target_config.target_position);
        force.set_force(correction * force_fector);
        print!("position: {:?}" , position.0);
        println!();
        // force.set_force(force_vector);
        gizmos.ray(
            position.0,
            force.force(),
            Color::rgb(1.0, 1.0, 1.0),
        );
    }
}
