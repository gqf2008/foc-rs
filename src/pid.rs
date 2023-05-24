#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use crate::{constrain, Regulator};

#[derive(Clone, Copy, Debug)]
pub struct Limit {
    lp: f32,
    li: f32,
    ld: f32,
    lo: f32,
}

impl Default for Limit {
    fn default() -> Self {
        Self {
            lp: f32::MAX,
            li: f32::MAX,
            ld: f32::MAX,
            lo: f32::MAX,
        }
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    p: f32,
    i: f32,
    d: f32,
    lo: f32,
    prev: f32,
    ramp: f32,
    dt: f32,
}

impl Output {
    pub fn pid(&mut self) -> f32 {
        let mut output = constrain!(self.p + self.i + self.d, -self.lo, self.lo);
        if self.ramp > 0. {
            let output_rate = (output as f64 - self.prev as f64) as f32 / self.dt;
            if output_rate > self.ramp {
                output = (self.prev as f64 + (self.ramp * self.dt) as f64) as f32;
            } else if output_rate < -self.ramp {
                output = (self.prev as f64 - (self.ramp * self.dt) as f64) as f32;
            }
        }
        self.prev = output;
        output
    }

    pub fn pi(&mut self) -> f32 {
        let mut output = constrain!(self.p + self.i, -self.lo, self.lo);
        if self.ramp > 0. {
            let output_rate = (output as f64 - self.prev as f64) as f32 / self.dt;
            if output_rate > self.ramp {
                output = (self.prev as f64 + (self.ramp * self.dt) as f64) as f32;
            } else if output_rate < -self.ramp {
                output = (self.prev as f64 - (self.ramp * self.dt) as f64) as f32;
            }
        }
        self.prev = output;
        output
    }

    pub fn pd(&mut self) -> f32 {
        let mut output = constrain!(self.p + self.d, -self.lo, self.lo);
        if self.ramp > 0. {
            let output_rate = (output as f64 - self.prev as f64) as f32 / self.dt;
            if output_rate > self.ramp {
                output = (self.prev as f64 + (self.ramp * self.dt) as f64) as f32;
            } else if output_rate < -self.ramp {
                output = (self.prev as f64 - (self.ramp * self.dt) as f64) as f32;
            }
        }
        self.prev = output;
        output
    }
}

impl core::ops::Mul<f32> for Output {
    type Output = Output;

    fn mul(self, rhs: f32) -> Self {
        Output {
            p: self.p * rhs,
            i: self.i * rhs,
            d: self.d * rhs,
            lo: self.lo,
            dt: 0.,
            ramp: self.ramp,
            prev: self.prev,
        }
    }
}

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
    pub output: Output,
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
        self.limits.lp = lp;
        self
    }
    pub fn limit_i(mut self, li: f32) -> Self {
        self.limits.li = li;
        self
    }
    pub fn limit_d(mut self, ld: f32) -> Self {
        self.limits.ld = ld;
        self
    }

    pub fn limit_out(mut self, lo: f32) -> Self {
        self.limits.lo = lo;
        self
    }

    pub fn ramp(mut self, ramp: f32) -> Self {
        self.output.ramp = ramp;
        self
    }
}

impl Pid {
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
    type Output = Output;
    fn set_point(&mut self, target: f32) -> &mut Self {
        self.target = target;
        self
    }

    fn update(&mut self, measurement: f32, timestamp_now_us: i64) -> &mut Self {
        let dt = (timestamp_now_us - self.timestamp_prev) as f32 / 1000000.;
        self.timestamp_prev = timestamp_now_us;
        let dt = if dt <= 0. || dt > 0.5 { 0.001 } else { dt };

        let Limit { lp, li, ld, lo } = self.limits;

        let error = self.target - measurement;
        let p_term = constrain!(self.kp * error, -lp, lp);
        let i_term = if self.ki > 0. {
            (self.integral as f64
                + (self.ki * dt * (error as f64 + self.error_prev as f64) as f32) as f64)
                as f32
        } else {
            0.
        };
        let i_term = constrain!(i_term, -li, li);
        let d_term = if self.kd > 0. {
            self.kd * (error as f64 - self.error_prev as f64) as f32 / dt
        } else {
            0.
        };
        let d_term = constrain!(d_term, -ld, ld);

        self.output = Output {
            p: p_term,
            i: i_term,
            d: d_term,
            lo,
            dt,
            ramp: self.output.ramp,
            prev: self.output.prev,
        };

        self.error_prev = error;
        self.integral = i_term;
        self
    }

    fn output(&mut self) -> Output {
        self.output
    }

    fn reset(&mut self) -> &mut Self {
        self.integral = 0.;
        self.error_prev = 0.;
        self.output = Output {
            p: 0.,
            i: 0.,
            d: 0.,
            prev: 0.,
            dt: 0.,
            ramp: self.output.ramp,
            lo: self.limits.lo,
        };
        self
    }
}
