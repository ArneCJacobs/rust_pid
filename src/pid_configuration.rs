use std::fmt::Debug;
use bevy::prelude::{Reflect, Resource};
use bevy_inspector_egui::InspectorOptions;
use bevy::math::Vec3;
use crate::{DerivativeMeasurement, MAX_PLOT_LENGTH};

#[derive(Reflect, Resource, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
pub struct PIDConfiguration<D> {
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
    pub error_history: Vec<D>,
    #[reflect(ignore)]
    pub proportional_history: Vec<D>,
    #[reflect(ignore)]
    pub integral_history: Vec<D>,
    #[reflect(ignore)]
    pub derivative_history: Vec<D>,
    #[reflect(ignore)]
    pub result_history: Vec<D>,
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

pub trait Magnitude {
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
