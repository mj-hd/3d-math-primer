use std::ops::{Mul, MulAssign};

use crate::{euler_angles::EulerAngles, quaternion::Quaternion, v3, vector::V3};

pub enum Axis {
    X,
    Y,
    Z,
}

pub struct Matrix3x4 {
    pub m11: f64,
    pub m12: f64,
    pub m13: f64,
    pub m21: f64,
    pub m22: f64,
    pub m23: f64,
    pub m31: f64,
    pub m32: f64,
    pub m33: f64,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
}

// 実質の4x4正方行列。右端は使わないので省略
impl Matrix3x4 {
    fn indentity() -> Self {
        Matrix3x4 {
            m11: 1.0,
            m12: 0.0,
            m13: 0.0,
            m21: 0.0,
            m22: 1.0,
            m23: 0.0,
            m31: 0.0,
            m32: 0.0,
            m33: 1.0,
            tx: 0.0,
            ty: 0.0,
            tz: 0.0,
        }
    }

    fn zero_translation(&mut self) {
        self.tx = 0.0;
        self.ty = 0.0;
        self.tz = 0.0;
    }

    fn translate(&mut self, d: V3) {
        self.tx = d.x();
        self.ty = d.y();
        self.tz = d.z();
    }

    fn from_translation(d: V3) -> Self {
        Self {
            tx: d.x,
            ty: d.y,
            tz: d.z,
            ..Self::indentity()
        }
    }

    fn from_local_to_parent_euler(pos: V3, orient: EulerAngles) -> Self {
        let orient_mat = RotationMatrix::from_orientation(orient);

        Self::from_local_to_parent_matrix(pos, orient_mat)
    }

    fn from_local_to_parent_matrix(pos: V3, orient: RotationMatrix) -> Self {
        Self {
            m11: orient.m11,
            m12: orient.m21,
            m13: orient.m31,
            m21: orient.m12,
            m22: orient.m22,
            m23: orient.m32,
            m31: orient.m13,
            m32: orient.m23,
            m33: orient.m33,
            tx: pos.x,
            ty: pos.y,
            tz: pos.z,
        }
    }
    fn from_parent_to_local_euler(pos: V3, orient: EulerAngles) -> Self {
        let orient_mat = RotationMatrix::from_orientation(orient);

        Self::from_parent_to_local_matrix(pos, orient_mat)
    }

    fn from_parent_to_local_matrix(pos: V3, orient: RotationMatrix) -> Self {
        Self {
            m11: orient.m11,
            m12: orient.m12,
            m13: orient.m13,
            m21: orient.m21,
            m22: orient.m22,
            m23: orient.m23,
            m31: orient.m31,
            m32: orient.m32,
            m33: orient.m33,
            tx: -(pos.x * orient.m11 + pos.y * orient.m21 + pos.z * orient.m31),
            ty: -(pos.x * orient.m12 + pos.y * orient.m22 + pos.z * orient.m32),
            tz: -(pos.x * orient.m13 + pos.y * orient.m23 + pos.z * orient.m33),
        }
    }

    fn from_rotate(axis: Axis, theta: f64) -> Self {
        let (s, c) = theta.sin_cos();

        match axis {
            Axis::X => Self {
                m22: c,
                m23: s,
                m32: -s,
                m33: c,
                ..Self::indentity()
            },
            Axis::Y => Self {
                m11: c,
                m13: -s,
                m31: s,
                m33: c,
                ..Self::indentity()
            },
            Axis::Z => Self {
                m11: c,
                m12: s,
                m21: -s,
                m22: c,
                ..Self::indentity()
            },
        }
    }

    fn from_rotate_by(axis: V3, theta: f64) -> Self {
        let (s, c) = theta.sin_cos();

        let a = 1.0 - c;
        let ax = a * axis.x;
        let ay = a * axis.y;
        let az = a * axis.z;

        Self {
            m11: ax * axis.x + c,
            m12: ax * axis.y + axis.z * s,
            m13: ax * axis.z - axis.y * s,
            m21: ay * axis.x - axis.z * s,
            m22: ay * axis.y + c,
            m23: ay * axis.z + axis.x * s,
            m31: az * axis.x + axis.y * s,
            m32: az * axis.y - axis.x * s,
            m33: az * axis.z + c,
            ..Self::indentity()
        }
    }

    fn from_quaternion(q: Quaternion) -> Self {
        let ww = 2.0 * q.w;
        let xx = 2.0 * q.x;
        let yy = 2.0 * q.y;
        let zz = 2.0 * q.z;

        Self {
            m11: 1.0 - yy * q.y - zz * q.z,
            m12: xx * q.y + ww * q.z,
            m13: xx * q.z - ww * q.x,
            m21: xx * q.y - ww * q.z,
            m22: 1.0 - xx * q.x - zz * q.z,
            m23: yy * q.z + ww * q.x,
            m31: xx * q.z + ww * q.y,
            m32: yy * q.z - ww * q.x,
            m33: 1.0 - xx * q.x - yy * q.y,
            ..Self::indentity()
        }
    }

    fn from_scale(s: V3) -> Self {
        Self {
            m11: s.x,
            m22: s.y,
            m33: s.z,
            ..Self::indentity()
        }
    }

    fn from_scale_along_axis(axis: V3, k: f64) -> Self {
        let a = k - 1.0;
        let ax = a * axis.x;
        let ay = a * axis.y;
        let az = a * axis.z;

        Self {
            m11: ax * axis.x + 1.0,
            m22: ay * axis.y + 1.0,
            m33: az * axis.z + 1.0,
            m12: ax * axis.y,
            m21: ax * axis.y,
            m13: ax * axis.z,
            m31: ax * axis.z,
            m23: ay * axis.z,
            m32: ay * axis.z,
            ..Self::indentity()
        }
    }

    fn from_shear(axis: Axis, s: f64, t: f64) -> Self {
        match axis {
            Axis::X => Self {
                m12: s,
                m13: t,
                ..Self::indentity()
            },
            Axis::Y => Self {
                m21: s,
                m23: t,
                ..Self::indentity()
            },
            Axis::Z => Self {
                m31: s,
                m32: t,
                ..Self::indentity()
            },
        }
    }

    fn from_project(n: V3) -> Self {
        Self {
            m11: 1.0 - n.x * n.x,
            m22: 1.0 - n.y * n.y,
            m33: 1.0 - n.z * n.z,
            m12: -n.x * n.y,
            m21: -n.x * n.y,
            m13: -n.x * n.z,
            m31: -n.x * n.z,
            m23: -n.y * n.z,
            m32: -n.y * n.z,
            ..Self::indentity()
        }
    }

    fn from_reflect(axis: Axis, k: f64) -> Self {
        match axis {
            Axis::X => Self {
                m11: -1.0,
                tx: 2.0 * k,
                ..Self::indentity()
            },
            Axis::Y => Self {
                m22: -1.0,
                ty: 2.0 * k,
                ..Self::indentity()
            },
            Axis::Z => Self {
                m33: -1.0,
                tz: 2.0 * k,
                ..Self::indentity()
            },
        }
    }

    fn from_reflect_by(n: V3) -> Self {
        let ax = -2.0 * n.x;
        let ay = -2.0 * n.y;
        let az = -2.0 * n.z;

        Self {
            m11: 1.0 + ax * n.x,
            m22: 1.0 + ay * n.y,
            m33: 1.0 + az * n.z,
            m12: ax * n.y,
            m21: ax * n.y,
            m13: ax * n.z,
            m31: ax * n.z,
            m23: ay * n.z,
            m32: ay * n.z,
            ..Self::indentity()
        }
    }

    fn determinant(&self) -> f64 {
        self.m11 * (self.m22 * self.m33 - self.m23 * self.m32)
            + self.m12 * (self.m23 * self.m31 - self.m21 * self.m33)
            + self.m13 * (self.m21 * self.m32 - self.m22 * self.m31)
    }

    fn inverse(&self, m: Matrix3x4) -> Matrix3x4 {
        let det = m.determinant();

        let one_over_det = 1.0 / det;

        let mut result = Self {
            m11: (m.m22 * m.m33 - m.m23 * m.m32) * one_over_det,
            m12: (m.m13 * m.m32 - m.m12 * m.m33) * one_over_det,
            m13: (m.m12 * m.m23 - m.m13 * m.m22) * one_over_det,
            m21: (m.m23 * m.m31 - m.m21 * m.m33) * one_over_det,
            m22: (m.m11 * m.m33 - m.m13 * m.m31) * one_over_det,
            m23: (m.m13 * m.m21 - m.m11 * m.m23) * one_over_det,
            m31: (m.m21 * m.m32 - m.m22 * m.m31) * one_over_det,
            m32: (m.m12 * m.m31 - m.m11 * m.m32) * one_over_det,
            m33: (m.m11 * m.m22 - m.m12 * m.m21) * one_over_det,
            ..Self::indentity()
        };

        result.tx = -(m.tx * result.m11 + m.ty * result.m21 + m.tz * result.m31);
        result.ty = -(m.tx * result.m12 + m.ty * result.m22 + m.tz * result.m32);
        result.tz = -(m.tx * result.m13 + m.ty * result.m23 + m.tz * result.m33);

        result
    }

    fn get_translation(&self) -> V3 {
        v3![self.tx, self.ty, self.ty,]
    }

    fn get_position_from_parent_to_local_matrix(&self) -> V3 {
        v3![
            -(self.tx * self.m11 + self.ty * self.m12 + self.tz * self.m13),
            -(self.tx * self.m21 + self.ty * self.m22 + self.tz * self.m23),
            -(self.tx * self.m31 + self.ty * self.m32 + self.tz * self.m33),
        ]
    }

    fn get_position_from_local_to_parent_matrix(&self) -> V3 {
        v3![self.tx, self.ty, self.tz,]
    }
}

impl Mul<Matrix3x4> for V3 {
    type Output = V3;

    fn mul(self, rhs: Matrix3x4) -> Self::Output {
        v3![
            self.x * rhs.m11 + self.y * rhs.m21 + self.z * rhs.m31 + rhs.tx,
            self.x * rhs.m12 + self.y * rhs.m22 + self.z * rhs.m32 + rhs.ty,
            self.x * rhs.m13 + self.y * rhs.m23 + self.z * rhs.m33 + rhs.tz,
        ]
    }
}

impl Mul<Matrix3x4> for Matrix3x4 {
    type Output = Matrix3x4;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            m11: self.m11 * rhs.m11 + self.m12 * rhs.m21 + self.m13 * rhs.m31,
            m12: self.m11 * rhs.m12 + self.m12 * rhs.m22 + self.m13 * rhs.m32,
            m13: self.m11 * rhs.m13 + self.m12 * rhs.m23 + self.m13 * rhs.m33,
            m21: self.m21 * rhs.m21 + self.m22 * rhs.m21 + self.m23 * rhs.m31,
            m22: self.m21 * rhs.m22 + self.m22 * rhs.m22 + self.m23 * rhs.m32,
            m23: self.m21 * rhs.m23 + self.m22 * rhs.m23 + self.m23 * rhs.m33,
            m31: self.m31 * rhs.m21 + self.m32 * rhs.m21 + self.m33 * rhs.m31,
            m32: self.m31 * rhs.m22 + self.m32 * rhs.m22 + self.m33 * rhs.m32,
            m33: self.m31 * rhs.m23 + self.m32 * rhs.m23 + self.m33 * rhs.m33,
            tx: self.tx * rhs.m11 + self.ty * rhs.m21 + self.tz * rhs.m31 + rhs.tx,
            ty: self.tx * rhs.m12 + self.ty * rhs.m22 + self.tz * rhs.m32 + rhs.ty,
            tz: self.tx * rhs.m13 + self.ty * rhs.m23 + self.tz * rhs.m33 + rhs.tz,
        }
    }
}

impl MulAssign<Matrix3x4> for Matrix3x4 {
    fn mul_assign(&mut self, rhs: Matrix3x4) {
        *self = *self * rhs;
    }
}

pub struct RotationMatrix {
    pub m11: f64,
    pub m12: f64,
    pub m13: f64,
    pub m21: f64,
    pub m22: f64,
    pub m23: f64,
    pub m31: f64,
    pub m32: f64,
    pub m33: f64,
}

impl RotationMatrix {
    fn identity() -> Self {
        RotationMatrix {
            m11: 1.0,
            m12: 0.0,
            m13: 0.0,
            m21: 0.0,
            m22: 1.0,
            m23: 0.0,
            m31: 0.0,
            m32: 0.0,
            m33: 1.0,
        }
    }

    fn from_orientation(orientation: EulerAngles) -> Self {
        let p = orientation.pitch.sin_cos();
        let b = orientation.bank.sin_cos();
        let h = orientation.heading.sin_cos();

        Self {
            m11: h.1 * b.1 + h.0 * p.0 * b.0,
            m12: -h.1 * b.0 + h.0 * p.0 * b.1,
            m13: h.0 * p.1,
            m21: b.0 * p.1,
            m22: b.1 * p.1,
            m23: -p.0,
            m31: -h.0 * b.1 + h.1 * p.0 * b.0,
            m32: b.0 * h.0 + h.1 * p.0 * b.1,
            m33: h.1 * p.1,
        }
    }

    fn from_inertial_to_obj_quaternion(q: Quaternion) -> Self {
        Self {
            m11: 1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            m12: 2.0 * (q.x * q.y + q.w * q.z),
            m13: 2.0 * (q.x * q.z - q.w * q.y),
            m21: 2.0 * (q.x * q.y - q.w * q.z),
            m22: 1.0 - 2.0 * (q.x * q.x + q.z * q.z),
            m23: 2.0 * (q.y * q.z + q.w * q.x),
            m31: 2.0 * (q.x * q.z + q.w * q.y),
            m32: 2.0 * (q.y * q.z - q.w * q.x),
            m33: 1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        }
    }

    fn from_obj_to_inertial_quaternion(&self, q: Quaternion) -> Self {
        Self {
            m11: 1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            m12: 2.0 * (q.x * q.y - q.w * q.z),
            m13: 2.0 * (q.x * q.z + q.w * q.y),
            m21: 2.0 * (q.x * q.y + q.w * q.z),
            m22: 1.0 - 2.0 * (q.x * q.x + q.z * q.z),
            m23: 2.0 * (q.y * q.z - q.w * q.x),
            m31: 2.0 * (q.x * q.z - q.w * q.y),
            m32: 2.0 * (q.y * q.z + q.w * q.x),
            m33: 1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        }
    }

    fn inertial_to_obj(&self, v: V3) -> V3 {
        v3![
            self.m11 * v.x + self.m21 * v.y + self.m31 * v.z,
            self.m12 * v.x + self.m22 * v.y + self.m32 * v.z,
            self.m13 * v.x + self.m23 * v.y + self.m33 * v.z,
        ]
    }

    fn obj_to_inertial(&self, v: V3) -> V3 {
        v3![
            self.m11 * v.x + self.m12 * v.y + self.m13 * v.z,
            self.m31 * v.x + self.m22 * v.y + self.m23 * v.z,
            self.m31 * v.x + self.m32 * v.y + self.m33 * v.z,
        ]
    }
}
