use core::fmt::Debug;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_xpbd_3d::{prelude::*, PhysicsSchedule, PhysicsStepSet};
use bevy_egui::{EguiPlugin, EguiContext, egui};
use bevy_inspector_egui::{
    prelude::ReflectInspectorOptions,
    quick::{ResourceInspectorPlugin, WorldInspectorPlugin},
    InspectorOptions,
};
use egui_plot::{Plot, Legend, Line, PlotPoints};


#[derive(Default, Reflect, Resource, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
enum DerivativeMeasurement {
    #[default] Velocity,
    ErrorRateOfChange,
}

#[derive(Reflect, Resource, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
struct PIDConfiguration<D> {
    #[inspector(min = 0.0, max = 10.0)]
    proportional_gain: f32,

    #[inspector(min = 0.0, max = 10.0)]
    derivative_gain: f32,

    #[inspector(min = 0.0, max = 10.0)]
    integral_gain: f32,

    derivate_measurement: DerivativeMeasurement,

    integral_stored: Option<D>,

    prev_error: Option<D>,
    prev_value: Option<D>,
    prev_result: Option<D>,

    #[reflect(ignore)]
    error_history: Vec<D>,
    #[reflect(ignore)]
    proportional_history: Vec<D>,
    #[reflect(ignore)]
    integral_history: Vec<D>,
    #[reflect(ignore)]
    derivative_history: Vec<D>,

    #[reflect(ignore)]
    result_history: Vec<D>,
}

impl<D> Default for PIDConfiguration<D> {
    fn default() -> Self {
        Self {
            proportional_gain: 1.0,
            derivative_gain: 1.0,
            integral_gain: 1.0,
            integral_stored: None,
            derivate_measurement: DerivativeMeasurement::Velocity,
            prev_error: None,
            prev_value: None,
            prev_result: None,
            error_history: Vec::new(),
            proportional_history: Vec::new(),
            integral_history: Vec::new(),
            derivative_history: Vec::new(),
            result_history: Vec::new(),
        }
    }
}

trait Magnitude {
    fn magnitude(&self) -> f32;
}

impl Magnitude for Vec3 {
    fn magnitude(&self) -> f32 {
        self.length()
    }
}
impl Magnitude for f32 {
    fn magnitude(&self) -> f32 {
        self.abs()
    }
}

impl<D> PIDConfiguration<D> 
    where D: std::ops::Sub<Output = D> + 
             std::ops::Add<Output = D> +
             std::ops::Div<f32, Output = D> + 
             std::ops::Neg<Output = D> + 
             std::ops::Mul<f32, Output = D> +
             Copy + Default + Debug +
             Magnitude, 

{
    pub fn update(&mut self, dt: f32, current_value: D, target_value: D) -> D {
        let error = target_value - current_value;
        let mut integral = self.integral_stored.unwrap_or(D::default());

        let proportional = error * self.proportional_gain;

        let mut derivative = D::default();

        if let Some(prev_error) = self.prev_error {

            let prev_value = self.prev_value.unwrap_or(current_value);
            derivative = match self.derivate_measurement {
                DerivativeMeasurement::Velocity => -(current_value - prev_value) / dt,
                DerivativeMeasurement::ErrorRateOfChange => (error - prev_error) / dt,
            } * self.derivative_gain;
            if derivative.magnitude() < 1e-5 {
                derivative = self.derivative_history.last().copied().unwrap();
                // println!("derivative: {:?}", derivative);
            }

            // println!("error: {:.5?}, prev_error: {:.5?}, error - prev_error: {:.5?}, derivative: {:.5?}", error, prev_error, error - prev_error, derivative);
        }

        integral = integral + error * dt;


        self.error_history.push(error);
        if self.error_history.len() > MAX_PLOT_LENGTH {
            self.error_history.remove(0);
        }
        self.proportional_history.push(proportional);
        if self.proportional_history.len() > MAX_PLOT_LENGTH {
            self.proportional_history.remove(0);
        }
        self.derivative_history.push(derivative);
        if self.derivative_history.len() > MAX_PLOT_LENGTH {
            self.derivative_history.remove(0);
        }
        self.integral_history.push(integral * self.integral_gain);
        if self.integral_history.len() > MAX_PLOT_LENGTH {
            self.integral_history.remove(0);
        }
        self.prev_error = Some(error);
        self.prev_value = Some(current_value);


        let result = proportional + derivative + integral * self.integral_gain;
        self.result_history.push(result);
        if self.result_history.len() > MAX_PLOT_LENGTH {
            self.result_history.remove(0);
        }

        self.prev_result = Some(result);
        self.integral_stored = Some(integral);
        result
    }

    // Reset the PID controller. Needs to be called if the target is moved (e.g. when it is teleported) or if the PID was disabled for a long time.
    pub fn reset(&mut self) {
        self.prev_error = None;
        self.prev_value = None;
        self.error_history.clear();
        self.proportional_history.clear();
        self.derivative_history.clear();
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

const MAX_PLOT_LENGTH: usize = 200;

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

        .init_resource::<PIDConfiguration<f32>>() // `ResourceInspectorPlugin` won't initialize the resource
        .register_type::<PIDConfiguration<f32>>() // you need to register your type to display it
        .add_plugins(ResourceInspectorPlugin::<PIDConfiguration<f32>>::default())

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
        .add_systems(Update, update_pid_plot)
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

fn get_line_from_data(name: &'_ str, data: &[f32]) -> Line {
  let points = PlotPoints::from_ys_f32(data);
  Line::new(points).name(name)
}

fn update_pid_plot(
    world: &mut World,
) {
    let Ok(egui_context) = world
        .query_filtered::<&mut EguiContext, With<PrimaryWindow>>()
        .get_single(world)
    else {
        return;
    };
    let mut egui_context = egui_context.clone();
    let pid_config = world.get_resource::<PIDConfiguration<f32>>().unwrap();

    egui::Window::new("PID Controller")
        .show(egui_context.get_mut(), |ui| {
            let plot = Plot::new("Components history")
                .legend(Legend::default());
            plot.show(ui, |plot_ui| {
                plot_ui.line(get_line_from_data("Error", &pid_config.error_history));
                plot_ui.line(get_line_from_data("Proportional", &pid_config.proportional_history));
                plot_ui.line(get_line_from_data("Derivative", &pid_config.derivative_history));
                plot_ui.line(get_line_from_data("Result", &pid_config.result_history));
                plot_ui.line(get_line_from_data("Integral", &pid_config.integral_history));

            });

        });
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
    mut pid_controller: ResMut<PIDConfiguration<f32>>,
    target_config: Res<TargetConfiguration>,
) {
    let force_vector = Vec3::Y * 1.0;
    for (position, mut force, velocity) in cube_query.iter_mut() {
        // let force_vector = (Vec3::Y * 10.0) * (time.elapsed_seconds() / 2.0).sin().abs();
        // println!("position: {:?}, force_fector: {:?}", position.0, force);
        // println!("position: {:?}, velocity: {:?}, force_fector: {:?}", position.0, velocity.0, force);
        let correction = pid_controller.update(
            time.delta_seconds(),
            position.0.y,
            target_config.target_position.y,
        );
        // force.set_force(correction * force_vector);
        force.set_impulse(correction*force_vector);
        // force.set_force(force_vector);
        gizmos.ray(position.0, force.impulse(), Color::rgb(1.0, 1.0, 1.0));
    }
}
