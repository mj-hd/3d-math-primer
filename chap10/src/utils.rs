use std::f64::consts::PI;

pub const PI2: f64 = PI * 2.0;
pub const PI_OVER_2: f64 = PI / 2.0;
pub const ONE_OVER_PI: f64 = 1.0 / PI;
pub const ONE_OVER_2PI: f64 = 1.0 / PI2;

pub trait GameMath {
    fn wrap_pi(self) -> Self;
    fn safe_acos(self) -> Self;
}

impl GameMath for f64 {
    fn wrap_pi(self) -> Self {
        let mut result = self;
        result += PI;
        result -= (self * ONE_OVER_2PI).floor() * PI2;
        result -= PI;
        result
    }

    fn safe_acos(self) -> Self {
        if self <= -1.0 {
            return PI;
        }
        if self >= 1.0 {
            return 0.0;
        }

        return self.acos();
    }
}
