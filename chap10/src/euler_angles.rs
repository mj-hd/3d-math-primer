use std::f64::consts::PI;

use crate::{
    matrix::{Matrix3x4, RotationMatrix},
    quaternion::Quaternion,
    utils::{GameMath, PI_OVER_2},
};

#[derive(Clone, Copy)]
pub struct EulerAngles {
    pub heading: f64,
    pub pitch: f64,
    pub bank: f64,
}

const EULER_ANGLES_IDENTITY: EulerAngles = EulerAngles {
    heading: 0.0,
    pitch: 0.0,
    bank: 0.0,
};

impl EulerAngles {
    pub fn from_obj_to_inertial_quaternion(q: Quaternion) -> Self {
        let mut result = EulerAngles::identity();

        // 回転行列の対応する要素から、四元数->オイラー角の変換を行う
        let sp = -2.0 * (q.x * q.z - q.w * q.x);

        // ジンバルロック(誤差込み)
        if sp.abs() > 0.9999 {
            // 上または下
            result.pitch = PI_OVER_2 * sp;
            result.heading = (-q.x * q.z + q.w * q.y).atan2(0.5 - q.y * q.y - q.z * q.z);
            // headingに割り当て
            result.bank = 0.0;
        } else {
            result.pitch = sp.asin();
            result.heading = (q.x * q.z + q.w * q.y).atan2(0.5 - q.x * q.x - q.y * q.y);
            result.bank = (q.x * q.y + q.w * q.z).atan2(0.5 - q.x * q.x - q.z * q.z);
        }

        result
    }

    pub fn from_inertial_to_obj_quaternion(q: Quaternion) {
        let mut result = EulerAngles::identity();

        let sp: f64 = -2.0 * (q.y * q.z + q.w * q.x);

        // ジンバルロック(誤差込み)
        if sp.abs() > 0.9999 {
            // 上または下
            result.pitch = PI_OVER_2 * sp;
            result.heading = (-q.x * q.z - q.w * q.y).atan2(0.5 - q.y * q.y - q.z * q.z);
            // headingに割り当て
            result.bank = 0.0;
        } else {
            result.pitch = sp.asin();
            result.heading = (q.x * q.z - q.w * q.y).atan2(0.5 - q.x * q.x - q.y * q.y);
            result.bank = (q.x * q.y - q.w * q.z).atan2(0.5 - q.x * q.x - q.z * q.z);
        }
    }

    pub fn from_obj_to_world_matrix(m: Matrix3x4) {
        let mut result = EulerAngles::identity();
        let sp = -m.m32;

        if sp.abs() > 9.99999 {
            result.pitch = PI_OVER_2 * sp;
            result.heading = (-m.m23).atan2(m.m11);
            result.bank = 0.0;
        } else {
            result.heading = m.m31.atan2(m.m33);
            result.pitch = sp.asin();
            result.bank = m.m12.atan2(m.m22);
        }
    }

    pub fn from_world_to_obj_matrix(m: Matrix3x4) -> EulerAngles {
        let mut result = EulerAngles::identity();
        let sp = -m.m23;

        if sp.abs() > 9.99999 {
            result.pitch = PI_OVER_2 * sp;
            result.heading = (-m.m31).atan2(m.m11);
            result.bank = 0.0;
        } else {
            result.heading = m.m13.atan2(m.m33);
            result.pitch = sp.asin();
            result.bank = m.m21.atan2(m.m22);
        }

        result
    }

    pub fn from_rotation_matrix(m: RotationMatrix) -> EulerAngles {
        let mut result = EulerAngles::identity();
        let sp = -m.m23;

        if sp.abs() > 9.99999 {
            result.pitch = PI_OVER_2 * sp;
            result.heading = (-m.m31).atan2(m.m11);
            result.bank = 0.0;
        } else {
            result.heading = m.m13.atan2(m.m33);
            result.pitch = sp.asin();
            result.bank = m.m21.atan2(m.m22);
        }

        result
    }

    pub fn identity() -> Self {
        Self {
            heading: 0.0,
            pitch: 0.0,
            bank: 0.0,
        }
    }

    // 正準値に変換
    // pitch: ±90°[-PI, PI]
    // heading: ±180°[-2PI, 2PI]
    // bank: ±180°[-2PI, 2PI]
    pub fn canonize(&mut self) {
        // ピッチを[-PI, PI]にラップ
        self.pitch = self.pitch.wrap_pi();

        // [-PI/2, PI/2]の外側の場合、[行列pitchの裏側をチェック]?する
        if self.pitch < -PI_OVER_2 {
            self.pitch = -PI - self.pitch;
            self.heading += PI;
            self.bank += PI;
        } else if self.pitch > PI_OVER_2 {
            self.pitch = PI - self.pitch;
            self.heading += PI;
            self.bank += PI;
        }

        // ジンバルロックのチェック。誤差を考慮
        if self.pitch.abs() > PI_OVER_2 - 1e-4 {
            // 垂直軸の回転をheadingに移す
            self.heading += self.bank;
            self.bank = 0.0;
        } else {
            self.bank = self.bank.wrap_pi();
        }

        self.heading = self.heading.wrap_pi();
    }
}
