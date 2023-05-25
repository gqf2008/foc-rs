#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use crate::{constrain, Regulator};

#[derive(Clone, Copy, Debug, Default)]
pub struct Limit {
    lp: Option<f32>,
    li: Option<f32>,
    ld: Option<f32>,
    lo: Option<f32>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    lo: Option<f32>,
    prev: f32,
    ramp: f32,
    dt: f32,
}

impl Output {
    pub fn pid(&mut self) -> f32 {
        let mut output = if let Some(lo) = self.lo {
            constrain!(
                (self.p as f64 + self.i as f64 + self.d as f64) as f32,
                -lo,
                lo
            )
        } else {
            (self.p as f64 + self.i as f64 + self.d as f64) as f32
        };
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
        let mut output = if let Some(lo) = self.lo {
            constrain!((self.p as f64 + self.i as f64) as f32, -lo, lo)
        } else {
            (self.p as f64 + self.i as f64) as f32
        };
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
        let mut output = if let Some(lo) = self.lo {
            constrain!((self.p as f64 + self.d as f64) as f32, -lo, lo)
        } else {
            (self.p as f64 + self.d as f64) as f32
        };
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
    integral: f32, //最后的误差积分
    error: f32,    //最后的误差
    timestamp: i64,
    ramp: f32,
    pub output: f32, //最后的输出
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

    //计算过程
    //1. 计算dt
    //2. 计算误差=目标值-测量值
    //3. 计算积分项=误差的积分（最后的积分项+误差*dt）
    //4. 计算微分项=误差的微分（(误差-最后的误差)/dt）
    //5. 计算PID输出=比例系数*误差+积分系数*误差的积分项+微分系数*误差的微分项
    //6. 根据斜率控制PID输出
    //7. 保存误差和输出值
    fn update(&mut self, measured: f32, timestamp_now_us: i64) -> &mut Self {
        let dt = (timestamp_now_us - self.timestamp) as f32 / 1000000.;
        self.timestamp = timestamp_now_us;
        let dt = if dt <= 0. || dt > 0.5 { 0.001 } else { dt };
        let Limit { lp, li, ld, lo } = self.limits;
        // BUG ESP32 rust工具链处理f32可能会出现意外结果
        //计算误差
        let error = (self.target as f64 - measured as f64) as f32;
        // 比例项=比例系数（kp）*误差（error）
        let p = if self.kp > 0. {
            let p = self.kp * error;
            if let Some(lp) = lp {
                constrain!(p, -lp, lp)
            } else {
                p
            }
        } else {
            0.
        };
        // 积分项=积分系数（ki）* (积分项+误差*dt)
        let i = if self.ki > 0. {
            self.integral = (self.integral as f64 + (dt * error) as f64) as f32;
            let i = self.ki * self.integral;
            if let Some(li) = li {
                constrain!(i, -li, li)
            } else {
                i
            }
        } else {
            0.
        };
        // 微分项=微分系数（kd）* (误差-最后的误差)/dt
        let d = if self.kd > 0. {
            let d = (error as f64 - self.error as f64) as f32 / dt;
            let d = self.kd * d;
            if let Some(ld) = ld {
                constrain!(d, -ld, ld)
            } else {
                d
            }
        } else {
            0.
        };
        //pid输出
        let output = (p as f64 + i as f64 + d as f64) as f32;
        let mut output = if let Some(lo) = lo {
            constrain!(output, -lo, lo)
        } else {
            output
        };
        //输出环节斜率控制
        if self.ramp > 0. {
            let output_rate = (output as f64 - self.output as f64) as f32 / dt;
            if output_rate > self.ramp {
                output = (self.output as f64 + (self.ramp * dt) as f64) as f32;
            } else if output_rate < -self.ramp {
                output = (self.output as f64 - (self.ramp * dt) as f64) as f32;
            }
        }
        self.error = error;
        self.output = output;
        self
    }

    fn output(&mut self) -> f32 {
        self.output
    }

    fn reset(&mut self) -> &mut Self {
        self.integral = 0.;
        self.error = 0.;
        self.output = 0.;
        self
    }
}
