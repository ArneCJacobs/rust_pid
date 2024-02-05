use crate::Magnitude;


#[derive(Copy, Clone, Default, Debug)]
pub struct Angle(f32);

impl Angle {
    pub fn new(angle: f32) -> Self {
        Self(angle)
    }

    fn make_positive(self) -> Self {
        Self(
            ((self.0 % 360.0) + 360.0) % 360.0
        )
    }
}

impl core::ops::Add<Angle> for Angle {
    type Output = Angle;

    fn add(self, rhs: Angle) -> Self::Output {
        Self(self.0 + rhs.0).make_positive()
    }
}

impl core::ops::Sub<Angle> for Angle {
    type Output = Angle;

    fn sub(self, rhs: Angle) -> Self::Output {
        Self(
            self.0 - rhs.0
        ).make_positive()
    }
}

impl core::ops::Div<f32> for Angle {
    type Output = Angle;

    fn div(self, rhs: f32) -> Self::Output {
        Self(
            self.0 / rhs
        ).make_positive()
    }
}

impl core::ops::Neg for Angle {
    type Output = Angle;
    
    fn neg(self) -> Self::Output {
        Self(
            self.0 + 180.0
        ).make_positive()
    }
}

impl core::ops::Mul<f32> for Angle {
    type Output = Angle;

    fn mul(self, rhs: f32) -> Self::Output {
        Self(
            self.0 * rhs
        ).make_positive()
    }
}

impl Magnitude for Angle {
    fn magnitude(&self) -> f32 {
        self.0
    }
}
