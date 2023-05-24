#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use crate::{addf32, constrain, Regulator};

#[derive(Clone, Copy, Debug, Default)]
pub struct Limit {
    lp: Option<f32>,
    li: Option<f32>,
    ld: Option<f32>,
    lo: Option<f32>,
}

// #[derive(Clone, Copy, Debug, Default)]
// pub struct Output {
//     pub p: f32,
//     pub i: f32,
//     pub d: f32,
//     lo: Option<f32>,
//     prev: f32,
//     ramp: f32,
//     dt: f32,
// }

// impl Output {
//     pub fn pid(&mut self) -> f32 {
//         let mut output = if let Some(lo) = self.lo {
//             constrain!(addf32!(self.p, self.i, self.d), -lo, lo)
//         } else {
//             (self.p as f64 + self.i as f64 + self.d as f64) as f32
//         };
//         if self.ramp > 0. {
//             let output_rate = (output as f64 - self.prev as f64) as f32 / self.dt;
//             if output_rate > self.ramp {
//                 output = (self.prev as f64 + (self.ramp * self.dt) as f64) as f32;
//             } else if output_rate < -self.ramp {
//                 output = (self.prev as f64 - (self.ramp * self.dt) as f64) as f32;
//             }
//         }
//         self.prev = output;
//         output
//     }

//     pub fn pi(&mut self) -> f32 {
//         let mut output = if let Some(lo) = self.lo {
//             constrain!((self.p as f64 + self.i as f64) as f32, -lo, lo)
//         } else {
//             (self.p as f64 + self.i as f64) as f32
//         };
//         if self.ramp > 0. {
//             let output_rate = (output as f64 - self.prev as f64) as f32 / self.dt;
//             if output_rate > self.ramp {
//                 output = (self.prev as f64 + (self.ramp * self.dt) as f64) as f32;
//             } else if output_rate < -self.ramp {
//                 output = (self.prev as f64 - (self.ramp * self.dt) as f64) as f32;
//             }
//         }
//         self.prev = output;
//         output
//     }

//     pub fn pd(&mut self) -> f32 {
//         let mut output = if let Some(lo) = self.lo {
//             constrain!((self.p as f64 + self.d as f64) as f32, -lo, lo)
//         } else {
//             (self.p as f64 + self.d as f64) as f32
//         };
//         if self.ramp > 0. {
//             let output_rate = (output as f64 - self.prev as f64) as f32 / self.dt;
//             if output_rate > self.ramp {
//                 output = (self.prev as f64 + (self.ramp * self.dt) as f64) as f32;
//             } else if output_rate < -self.ramp {
//                 output = (self.prev as f64 - (self.ramp * self.dt) as f64) as f32;
//             }
//         }
//         self.prev = output;
//         output
//     }
// }

// impl core::ops::Mul<f32> for Output {
//     type Output = Output;

//     fn mul(self, rhs: f32) -> Self {
//         Output {
//             p: self.p * rhs,
//             i: self.i * rhs,
//             d: self.d * rhs,
//             lo: self.lo,
//             dt: 0.,
//             ramp: self.ramp,
//             prev: self.prev,
//         }
//     }
// }

#[derive(Debug, Clone, Copy, Default)]
pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    limits: Limit,
    target: f32,
    integral: f32,
    error_prev: f32,
    timestamp_prev: i64,
    output_prev: f32,
    ramp: f32,
    pub output: f32,
    _private: (),
}

impl Pid {
    /// `integral_limit`: The maximum magnitude of the integral (anti-windup)
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Pid {
            kp,
            ki,
            kd,
            ..Default::default()
        }
    }
    pub fn limit_p(mut self, lp: f32) -> Self {
        self.limits.lp = Some(lp);
        self
    }
    pub fn limit_i(mut self, li: f32) -> Self {
        self.limits.li = Some(li);
        self
    }
    pub fn limit_d(mut self, ld: f32) -> Self {
        self.limits.ld = Some(ld);
        self
    }

    pub fn limit_out(mut self, lo: f32) -> Self {
        self.limits.lo = Some(lo);
        self
    }

    pub fn ramp(mut self, ramp: f32) -> Self {
        self.ramp = ramp;
        self
    }
}

impl Pid {
    pub fn set_kpid(&mut self, kp: f32, ki: f32, kd: f32) -> &mut Self {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
        self
    }
    pub fn set_kpi(&mut self, kp: f32, ki: f32) -> &mut Self {
        self.kp = kp;
        self.ki = ki;
        self
    }
    pub fn set_kpd(&mut self, kp: f32, kd: f32) -> &mut Self {
        self.kp = kp;
        self.kd = kd;
        self
    }
    pub fn set_kp(&mut self, kp: f32) -> &mut Self {
        self.kp = kp;
        self
    }
    pub fn set_ki(&mut self, ki: f32) -> &mut Self {
        self.ki = ki;
        self
    }

    pub fn set_kd(&mut self, kd: f32) -> &mut Self {
        self.kd = kd;
        self
    }
}

impl Regulator for Pid {
    type Input = f32;
    type Output = f32;
    fn set_point(&mut self, target: f32) -> &mut Self {
        self.target = target;
        self
    }

    fn update(&mut self, measurement: f32, timestamp_now_us: i64) -> &mut Self {
        let dt = (timestamp_now_us - self.timestamp_prev) as f32 / 1000000.;
        self.timestamp_prev = timestamp_now_us;
        let dt = if dt <= 0. || dt > 0.5 { 0.001 } else { dt };

        let Limit { lp, li, ld, lo } = self.limits;
        // BUG ESP32 rust工具链处理f32可能会出现意外结果
        let error = addf32!(self.target, -measurement);
        let p_term = if let Some(lp) = lp {
            constrain!(self.kp * error, -lp, lp)
        } else {
            self.kp * error
        };

        let i_term = if self.ki > 0. {
            addf32!(self.integral, self.ki * dt * error)
        } else {
            0.
        };

        let i_term = if let Some(li) = li {
            constrain!(i_term, -li, li)
        } else {
            i_term
        };
        let d_term = if self.kd > 0. {
            self.kd * addf32!(error, -self.error_prev) / dt
        } else {
            0.
        };

        let d_term = if let Some(ld) = ld {
            constrain!(d_term, -ld, ld)
        } else {
            d_term
        };

        let output = addf32!(p_term, i_term, d_term);

        let mut output = if let Some(lo) = lo {
            constrain!(output, -lo, lo)
        } else {
            output
        };
        if self.ramp > 0. {
            let output_rate = addf32!(output, -self.output_prev) / dt;
            if output_rate > self.ramp {
                output = addf32!(self.output_prev, self.ramp * dt);
            } else if output_rate < -self.ramp {
                output = addf32!(self.output_prev, -(self.ramp * dt));
            }
        }
        self.error_prev = error;
        self.integral = i_term;
        self.output_prev = output;
        self.output = output;
        self
    }

    fn output(&mut self) -> f32 {
        self.output
    }

    fn reset(&mut self) -> &mut Self {
        self.integral = 0.;
        self.error_prev = 0.;
        self.output = 0.;
        self
    }
}
