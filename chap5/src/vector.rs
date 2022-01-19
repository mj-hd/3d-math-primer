use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

#[macro_export]
macro_rules! v3 {
    ($x:expr, $y:expr, $z:expr) => {
        V3::new($x, $y, $z)
    };
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct V3 {
    x: f64,
    y: f64,
    z: f64,
}

const ZERO: V3 = V3 {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

impl V3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        V3 { x, y, z }
    }

    pub fn zero(&mut self) {
        self.x = 0.0;
        self.y = 0.0;
        self.z = 0.0;
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    pub fn z(&self) -> f64 {
        self.z
    }

    pub fn mag(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(&self) -> Self {
        let mag = self.mag();

        *self / mag
    }

    pub fn dot(&self, rhs: &Self) -> Self {
        V3 {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }

    pub fn cross(&self, rhs: &Self) -> Self {
        V3 {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    pub fn distance(&self, rhs: &Self) -> f64 {
        (*self - *rhs).mag()
    }
}

impl Add for V3 {
    type Output = V3;

    fn add(self, rhs: Self) -> Self::Output {
        V3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl AddAssign for V3 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for V3 {
    type Output = V3;

    fn sub(self, rhs: Self) -> Self::Output {
        V3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl SubAssign for V3 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Neg for V3 {
    type Output = V3;

    fn neg(self) -> Self::Output {
        V3 {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl Mul<f64> for V3 {
    type Output = V3;

    fn mul(self, rhs: f64) -> Self::Output {
        V3 {
            x: rhs * self.x,
            y: rhs * self.y,
            z: rhs * self.z,
        }
    }
}

impl MulAssign<f64> for V3 {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Div<f64> for V3 {
    type Output = V3;

    fn div(self, rhs: f64) -> Self::Output {
        V3 {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl DivAssign<f64> for V3 {
    fn div_assign(&mut self, rhs: f64) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

impl Mul<V3> for f64 {
    type Output = V3;

    fn mul(self, rhs: V3) -> Self::Output {
        V3 {
            x: self * rhs.x,
            y: self * rhs.y,
            z: self * rhs.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add() {
        let a = v3!(1.0, 2.0, 3.0);
        let b = v3!(3.0, 2.0, 1.0);
        assert_eq!(a + b, v3!(4.0, 4.0, 4.0));
    }

    #[test]
    fn sub() {
        let a = v3!(1.0, 2.0, 3.0);
        let b = v3!(3.0, 2.0, 1.0);
        assert_eq!(a - b, v3!(-2.0, 0.0, 2.0));
    }

    #[test]
    fn add_assign() {
        let mut a = v3!(1.0, 2.0, 3.0);
        a += v3!(3.0, 2.0, 1.0);
        assert_eq!(a, v3!(4.0, 4.0, 4.0));
    }

    #[test]
    fn sub_assign() {
        let mut a = v3!(1.0, 2.0, 3.0);
        a -= v3!(3.0, 2.0, 1.0);
        assert_eq!(a, v3!(-2.0, 0.0, 2.0));
    }

    #[test]
    fn mul() {
        let a = v3!(1.0, 2.0, 3.0);
        assert_eq!(a * 3.0, v3!(3.0, 6.0, 9.0));
        assert_eq!(3.0 * a, v3!(3.0, 6.0, 9.0));
    }

    #[test]
    fn div() {
        let a = v3!(2.0, 4.0, 6.0);
        assert_eq!(a / 2.0, v3!(1.0, 2.0, 3.0));
    }

    #[test]
    fn mul_assign() {
        let mut a = v3!(1.0, 2.0, 3.0);
        a *= 3.0;
        assert_eq!(a, v3!(3.0, 6.0, 9.0));
    }

    #[test]
    fn div_assign() {
        let mut a = v3!(2.0, 4.0, 6.0);
        a /= 2.0;
        assert_eq!(a, v3!(1.0, 2.0, 3.0));
    }

    #[test]
    fn normalize() {
        let a = v3!(0.0, 2.0, 0.0);
        assert_eq!(a.normalize(), v3!(0.0, 1.0, 0.0));
    }

    #[test]
    fn mag() {
        let a = v3!(0.0, 2.0, 0.0);
        assert_eq!(a.mag(), 2.0);
    }

    #[test]
    fn dot() {
        let a = v3!(1.0, 2.0, 3.0);
        let b = v3!(1.0, 2.0, 3.0);
        assert_eq!(a.dot(&b), v3!(1.0, 4.0, 9.0));
    }

    #[test]
    fn cross() {
        let a = v3!(1.0, 2.0, 3.0);
        let b = v3!(3.0, 2.0, 1.0);
        assert_eq!(a.cross(&b), v3!(-4.0, 8.0, -4.0));
    }

    #[test]
    fn distance() {
        let a = v3!(1.0, 2.0, 3.0);
        let b = v3!(2.0, 2.0, 3.0);
        assert_eq!(a.distance(&b), 1.0);
    }
}
