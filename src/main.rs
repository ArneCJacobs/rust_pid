use core::fmt::Debug;
use angle::Angle;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_xpbd_3d::{PhysicsSchedule, PhysicsStepSet, prelude::*};
use bevy_egui::{egui, EguiContext, EguiPlugin};
use bevy_inspector_egui::{
    InspectorOptions,
    prelude::ReflectInspectorOptions,
    quick::{ResourceInspectorPlugin, WorldInspectorPlugin},
};
use egui_plot::{Legend, Line, Plot, PlotPoints};
use pid_configuration::{Magnitude, PIDConfiguration};
use systems::plot;

mod angle;
mod pid_configuration;
mod systems;

const MAX_PLOT_LENGTH: usize = 200;

#[derive(Default, Reflect, Resource, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
enum DerivativeMeasurement {
    #[default] Velocity,
    ErrorRateOfChange,
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



#[derive(Reflect, Resource, InspectorOptions, Default)]
#[reflect(Resource, InspectorOptions)]
struct Config {
    pause: bool,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(bevy_inspector_egui::DefaultInspectorConfigPlugin)
        // .add_plugins(WorldInspectorPlugin::new())

        .init_resource::<PIDConfiguration<Vec3>>() // `ResourceInspectorPlugin` won't initialize the resource
        .register_type::<PIDConfiguration<Vec3>>() // you need to register your type to display it
        .add_plugins(ResourceInspectorPlugin::<PIDConfiguration<Vec3>>::default())

        .init_resource::<TargetConfiguration>() // `ResourceInspectorPlugin` won't initialize the resource
        .register_type::<TargetConfiguration>() // you need to register your type to display it
        .add_plugins(ResourceInspectorPlugin::<TargetConfiguration>::default())

        .init_resource::<Config>() // `ResourceInspectorPlugin` won't initialize the resource
        .register_type::<Config>() // you need to register your type to display it
        .add_plugins(ResourceInspectorPlugin::<Config>::default())

        // .insert_resource(Gravity(Vec3::new(0.0, 0.0, 0.0)))
        // .insert_resource(Time::new_with(Physics::fixed_hz(144.0)))
        .add_systems(Startup, setup)
        .add_systems(
            PhysicsSchedule, 
            update
                .after(PhysicsSet::Sync)
                .before(PhysicsSet::Prepare)
                .before(PhysicsSet::StepSimulation)
                .before(PhysicsStepSet::Substeps)
                .before(PhysicsStepSet::BroadPhase)
                .before(PhysicsStepSet::ReportContacts)
                .before(PhysicsStepSet::Sleeping)
                .before(PhysicsStepSet::SpatialQuery)
        )
        .add_systems(Update, draw_target)
        .add_systems(Update, plot::update_pid_plot)
        .add_systems(Update, pause_system)
        .run();
}

fn pause_system(
    mut config: ResMut<Config>,
    input: Res<Input<KeyCode>>,
    mut time: ResMut<Time<Physics>>,
) {
    if input.just_pressed(KeyCode::Space) {
        config.pause = !config.pause;
    }
    if config.pause {
        time.pause();
    } else {
        time.unpause();
    }
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

fn draw_target(mut gizmos: Gizmos, target: Res<TargetConfiguration>) {
    gizmos.cuboid(
        Transform::from_translation(target.target_position),
        Color::GREEN,
    );
}

fn update(
    mut cube_query: Query<(&Position, &mut ExternalImpulse, &LinearVelocity), With<CubeTag>>,
    time: Res<Time>,
    mut gizmos: Gizmos,
    mut pid_controller: ResMut<PIDConfiguration<Vec3>>,
    target_config: Res<TargetConfiguration>,
) {
    let force_vector = Vec3::Y * 1.0;
    for (position, mut force, _velocity) in cube_query.iter_mut() {
        // let force_vector = (Vec3::Y * 10.0) * (time.elapsed_seconds() / 2.0).sin().abs();
        // println!("position: {:?}, force_fector: {:?}", position.0, force);
        // println!("position: {:?}, velocity: {:?}, force_fector: {:?}", position.0, velocity.0, force);
        let correction = pid_controller.update(
            time.delta_seconds(),
            position.0,
            target_config.target_position,
        );
        // force.set_force(correction * force_vector);
        force.set_impulse(correction);
        // force.set_force(force_vector);
        gizmos.ray(position.0, force.impulse(), Color::rgb(1.0, 1.0, 1.0));
    }
}
