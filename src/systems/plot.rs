use bevy::prelude::{With, World};
use bevy_egui::{egui, EguiContext};
use bevy::window::PrimaryWindow;
use bevy::math::Vec3;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use crate::pid_configuration::{Magnitude, PIDConfiguration};

fn get_line_from_scalar_data(name: &'_ str, data: &[f32]) -> Line {
  let points = PlotPoints::from_ys_f32(data);
  Line::new(points).name(name)
}

fn get_line_from_vec_data(name: &'_ str, data: &[Vec3]) -> Line {
  let points = PlotPoints::from_ys_f32(&data.iter().map(|v| v.magnitude()).collect::<Vec<_>>());
  Line::new(points).name(name)
}

pub fn update_pid_plot(
    world: &mut World,
) {
    let Ok(egui_context) = world
        .query_filtered::<&mut EguiContext, With<PrimaryWindow>>()
        .get_single(world)
    else {
        return;
    };
    let mut egui_context = egui_context.clone();
    let pid_config = world.get_resource::<PIDConfiguration<Vec3>>().unwrap();

    egui::Window::new("PID Controller")
        .show(egui_context.get_mut(), |ui| {
            let plot = Plot::new("Components history")
                .legend(Legend::default());
            plot.show(ui, |plot_ui| {
                plot_ui.line(get_line_from_vec_data("Error", &pid_config.error_history));
                plot_ui.line(get_line_from_vec_data("Proportional", &pid_config.proportional_history));
                plot_ui.line(get_line_from_vec_data("Derivative", &pid_config.derivative_history));
                plot_ui.line(get_line_from_vec_data("Result", &pid_config.result_history));
                plot_ui.line(get_line_from_vec_data("Integral", &pid_config.integral_history));

            });

        });
}
