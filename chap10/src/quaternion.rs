use std::ops::{Mul, MulAssign};

use crate::{euler_angles::EulerAngles, utils::GameMath, v3, vector::V3};

const QUATERNION_IDENTITY: Quaternion = Quaternion {
    w: 1.0,
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    fn identitiy() -> Self {
        Quaternion {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    fn rotate_x(&mut self, theta: f64) {
        let theta_over_2 = theta * 0.5;
        self.w = theta_over_2.cos();
        self.x = theta_over_2.sin();
        self.y = 0.0;
        self.z = 0.0;
    }

    fn rotate_y(&mut self, theta: f64) {
        let theta_over_2 = theta * 0.5;
        self.w = theta_over_2.cos();
        self.x = 0.0;
        self.y = theta_over_2.sin();
        self.z = 0.0;
    }

    fn rotate_z(&mut self, theta: f64) {
        let theta_over_2 = theta * 0.5;
        self.w = theta_over_2.cos();
        self.x = 0.0;
        self.y = 0.0;
        self.z = theta_over_2.sin();
    }

    fn rotate_axis(&mut self, axis: V3, theta: f64) {
        assert!(axis.mag().abs() - 1.0 < 0.01);

        let theta_over_2 = theta * 0.5;
        let sin_theta_over_2 = theta_over_2.sin();

        self.w = theta_over_2.cos();
        self.x = axis.x * sin_theta_over_2;
        self.y = axis.y * sin_theta_over_2;
        self.z = axis.z * sin_theta_over_2;
    }

    fn rotate_obj_to_inertial(&mut self, orientation: EulerAngles) {
        let p = (orientation.pitch * 0.5).sin_cos();
        let b = (orientation.bank * 0.5).sin_cos();
        let h = (orientation.heading * 0.5).sin_cos();

        self.w = h.1 * p.1 * b.1 + h.0 * p.0 * b.0;
        self.x = h.1 * p.0 * b.1 + h.0 * p.1 * b.0;
        self.y = -h.1 * p.0 * b.0 + h.0 * p.1 * b.1;
        self.z = -h.0 * p.0 * b.1 + h.1 * p.1 * b.0;
    }

    fn rotate_inertial_to_obj(&mut self, orientation: EulerAngles) {
        let p = (orientation.pitch * 0.5).sin_cos();
        let b = (orientation.bank * 0.5).sin_cos();
        let h = (orientation.heading * 0.5).sin_cos();

        self.w = h.1 * p.1 * b.1 + h.0 * p.0 * b.0;
        self.x = -h.1 * p.0 * b.1 - h.0 * p.1 * b.0;
        self.y = h.1 * p.0 * b.0 - h.0 * b.1 * p.1;
        self.z = h.0 * p.0 * b.1 - h.1 * p.1 * b.0;
    }

    fn normalize(&mut self) {
        let mag = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();

        if mag > 0.0 {
            let one_over_mag = 1.0 / mag;
            self.w *= one_over_mag;
            self.x *= one_over_mag;
            self.y *= one_over_mag;
            self.z *= one_over_mag;
        }
    }

    fn get_rotation_angle(&self) -> f64 {
        let theta_over_2 = self.w.safe_acos();
        theta_over_2 * 2.0
    }

    fn get_rotation_axis(&self) -> V3 {
        let sin_theta_over_2_sq = 1.0 - self.w * self.w;

        if sin_theta_over_2_sq <= 0.0 {
            return v3![1.0, 0.0, 0.0];
        }

        let one_over_sin_theta_over_2 = 1.0 / sin_theta_over_2_sq.sqrt();

        v3![
            self.x * one_over_sin_theta_over_2,
            self.y * one_over_sin_theta_over_2,
            self.z * one_over_sin_theta_over_2,
        ]
    }

    fn dot(&self, other: Quaternion) -> f64 {
        self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
    }

    fn slerp(&self, other: Quaternion, t: f64) -> Quaternion {
        if t <= 0.0 {
            return *self;
        }
        if t >= 1.0 {
            return other;
        }

        // 内積から角度を求める
        let mut cos_omega = self.dot(other);

        let mut w = self.w;
        let mut x = self.x;
        let mut y = self.y;
        let mut z = self.z;
        if cos_omega < 0.0 {
            w = -w;
            x = -x;
            y = -y;
            z = -z;
            cos_omega = -cos_omega;
        }

        let mut k0 = 0.0;
        let mut k1 = 0.0;

        // omegaが0に近い場合、0除算防ぐために線形補完に切り替える
        if cos_omega > 0.9999 {
            k0 = 1.0 - t;
            k1 = t;
        } else {
            let sin_omega = (1.0 - cos_omega * cos_omega).sqrt();

            let omega = sin_omega.atan2(cos_omega);

            let one_over_sign_omega = 1.0 / sin_omega;

            k0 = (((1.0 - t) * omega) * one_over_sign_omega).sin();
            k1 = ((t * omega) * one_over_sign_omega).sin();
        }

        Quaternion {
            w: k0 * self.w + k1 * other.w,
            x: k0 * self.x + k1 * other.x,
            y: k0 * self.y + k1 * other.y,
            z: k0 * self.z + k1 * other.z,
        }
    }

    // 共役
    fn conjugate(&self, other: Quaternion) -> Quaternion {
        Quaternion {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    fn pow(&self, exp: f64) -> Quaternion {
        if self.w.abs() > 0.9999 {
            return *self;
        }

        let alpha = self.w.acos();
        let new_alpha = alpha * exp;
        let mult = new_alpha.sin() / alpha.sin();

        Quaternion {
            w: new_alpha.cos(),
            x: self.x * mult,
            y: self.y * mult,
            z: self.z * mult,
        }
    }
}

impl Mul for Quaternion {
    type Output = Quaternion;

    fn mul(self, rhs: Self) -> Self::Output {
        Quaternion {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.z * rhs.y - self.y * rhs.z,
            y: self.w * rhs.y + self.y * rhs.w + self.x * rhs.z - self.z * rhs.x,
            z: self.w * rhs.z + self.z * rhs.w + self.y * rhs.x - self.x * rhs.y,
        }
    }
}

impl MulAssign for Quaternion {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}
