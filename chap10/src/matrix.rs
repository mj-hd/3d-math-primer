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
    pub m41: f64,
    pub m42: f64,
    pub m43: f64,
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
            m11: 0.0,
            m12: 0.0,
            m13: 0.0,
            m21: 0.0,
            m22: 0.0,
            m23: 0.0,
            m31: 0.0,
            m32: 0.0,
            m33: 0.0,
        }
    }

    fn from_orientation(orientation: EulerAngles) -> Self {}

    fn from_inertial_to_obj_quaternion(&self, other: Quaternion) -> Self {}

    fn from_obj_to_inertial_quaternion(&self, other: Quaternion) -> Self {}

    fn inertial_to_obj(&self) -> V3 {}

    fn obj_to_inertial(&self) -> V3 {}
}
