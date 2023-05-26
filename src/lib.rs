//!定义了FOC控制中控制模块、调制模块、反馈模块、传感器模块的抽象定义，
//! 实现了park、clarke变换和正弦调制、空间矢量调制算法

#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![feature(associated_type_defaults)]

pub mod fpid;
pub mod lpf;
pub mod pid;

use core::panic;
pub const PI_1_3: f32 = 1.0471975512;
pub const PI: f32 = 3.14159265359;
pub const PI_2: f32 = 1.57079632679;
pub const PI_3: f32 = 1.0471975512;
pub const _2PI: f32 = 6.28318530718;
pub const _3PI_2: f32 = 4.71238898038;
pub const RPM_TO_RADS: f32 = 0.10471975512;

pub const _1_SQRT_3: f32 = 0.57735026919;
pub const SQRT_3_2: f32 = 0.86602540378;
pub const SQRT_3: f32 = 1.73205080757;
pub const SQRT_2: f32 = 1.41421356237;

pub const MIN_ANGLE_DETECT_MOVEMENT: f32 = _2PI / 101.0;
pub const DEF_PID_CURR_P: f32 = 3.;
pub const DEF_PID_CURR_I: f32 = 300.;
pub const DEF_PID_CURR_D: f32 = 0.;
pub const DEF_PID_CURR_RAMP: f32 = 30.;
pub const DEF_POWER_SUPPLY: f32 = 12.;
pub const DEF_CURR_FILTER_Tf: f32 = 0.005;

pub const DEF_PID_VEL_P: f32 = 0.5;
pub const DEF_PID_VEL_I: f32 = 10.0;
pub const DEF_PID_VEL_D: f32 = 0.0;
pub const DEF_PID_VEL_RAMP: f32 = 1000.0;
pub const DEF_PID_VEL_LIMIT: f32 = DEF_POWER_SUPPLY;

pub const DEF_P_ANGLE_P: f32 = 20.0;
pub const DEF_VEL_LIM: f32 = 20.0;

#[macro_export]
macro_rules! constrain {
    ($a:expr,$min:expr,$max:expr) => {{
        if $a < $min {
            $min
        } else if $a > $max {
            $max
        } else {
            $a
        }
    }};
}

// 电角度求解，电角度=机械角度*极对数
#[macro_export]
macro_rules! electrical_angle {
    ($shaft_angle:expr,$pole_pairs:expr) => {{
        $shaft_angle * $pole_pairs as f32
    }};
    ($shaft_angle:expr,$pole_pairs:expr,$zero_electric_angle:expr) => {{
        (($shaft_angle * $pole_pairs as f32) as f64 - $zero_electric_angle as f64) as f32
    }};
}

// 归一化角度到 [0,2PI]
#[macro_export]
macro_rules! normalize_angle {
    ($angle:expr) => {{
        let a = libm::fmodf($angle, _2PI); //取余运算可以用于归一化，列出特殊值例子算便知
        if a.is_sign_positive() {
            a
        } else  {
            a + _2PI
        }
    }};
}

//电机驱动
pub trait Driver {
    fn set_target(&mut self, target: Voltage);
    fn max_voltage(&self) -> f32;
    fn voltage_power_supply(&self) -> f32;
}

#[derive(Debug, Clone, Copy)]
pub enum Motion {
    //力矩控制，牛米
    Torque(f32),
    //速度控制，弧度/秒
    Velocity(f32),
    //位置控制(angle)
    Position(f32),
}

#[derive(Debug, Clone, Copy)]
pub enum Target {
    Closeloop(Motion),
    Openloop(Motion),
}

impl Target {
    pub fn is_openloop(&self) -> bool {
        match self {
            Target::Openloop(_) => true,
            _ => false,
        }
    }
}

//调制方式
#[derive(Debug, Clone, Copy)]
pub enum Modulation {
    Spwm,
    Svpwm,
    SvpwmSimpleFOC,
}

//电机状态
#[derive(Debug, Clone, Copy)]
pub enum State {
    Disabled,    //禁用
    Calibrating, //正在校准
    Ready,       //就绪
    Moving,      //运动
    Error(i32),  //错误
}

pub trait Motor {
    type Regulator: Regulator;
    type Observer: Observer;
    type Feedback: Feedback;
    type Driver: Driver;
    fn disable(&mut self) -> &mut Self;
    fn enable(&mut self) -> &mut Self;
    fn move_to(&mut self, target: Target) -> &mut Self;
    //转子机械角度，弧度单位
    fn shaft_angle(&mut self) -> f32;
    //转子速度，弧度/秒
    fn shaft_velocity(&mut self) -> f32;
    //转子电角度，弧度单位
    fn electrical_angle(&mut self) -> f32;

    fn driver(&mut self) -> &mut Self::Driver;

    fn modulation(&self) -> Modulation {
        Modulation::Svpwm
    }

    fn state(&self) -> State;

    fn set_phase_voltage(&mut self, uq: f32, ud: f32, angle_el: f32) -> &mut Self {
        let modulation = self.modulation();
        let driver = self.driver();
        let voltage_limit = driver.max_voltage();
        let uvw = match modulation {
            Modulation::Spwm => Voltage::Dq(ud, uq).spwm(angle_el, voltage_limit),
            Modulation::Svpwm => Voltage::Dq(ud, uq).svpwm(angle_el, voltage_limit),
            Modulation::SvpwmSimpleFOC => {
                Voltage::Dq(ud, uq).svpwm_simplefoc(angle_el, voltage_limit)
            }
        };
        driver.set_target(uvw);
        self
    }
}

//观测器
pub trait Observer {
    type Output;
    //转子轴速度（弧度/秒）
    fn velocity(&mut self) -> Self::Output;
    //转子机械角度（弧度）
    fn angle(&mut self) -> Self::Output;
    //转子位置(0-2PI)
    fn position(&mut self) -> Self::Output;
    //转子机械角度和速度
    fn angle_velocity(&mut self) -> (Self::Output, Self::Output);
}

pub struct NoneObserver;

impl Observer for NoneObserver {
    type Output = f32;

    fn angle(&mut self) -> Self::Output {
        0.
    }
    fn velocity(&mut self) -> Self::Output {
        0.
    }

    fn position(&mut self) -> Self::Output {
        0.
    }
    fn angle_velocity(&mut self) -> (Self::Output, Self::Output) {
        (0., 0.)
    }
}

//电流反馈
pub trait Feedback {
    type Output;
    fn phase_current(&mut self) -> Self::Output;
}

pub struct NoneFeedback;

impl Feedback for NoneFeedback {
    type Output = Current;
    fn phase_current(&mut self) -> Self::Output {
        Current::Dq(0., 0.)
    }
}
//调节器
pub trait Regulator {
    type Input;
    type Output;
    //设置目标
    fn set_point(&mut self, setpoint: Self::Input) -> &mut Self;
    //更新测量值，dt表示时间隔
    fn update(&mut self, measurement: Self::Input, timestamp_us: i64) -> &mut Self;
    //获取输出
    fn output(&mut self) -> Self::Output;

    fn reset(&mut self) -> &mut Self;

    fn chain<R: Regulator>(self, next: R) -> Chain<Self, R>
    where
        Self: Sized,
    {
        Chain {
            first: self,
            second: next,
        }
    }
}

pub struct NoneRegulator;

impl Regulator for NoneRegulator {
    type Input = Option<f32>;
    type Output = Option<f32>;
    fn output(&mut self) -> Self::Output {
        None
    }
    fn reset(&mut self) -> &mut Self {
        self
    }
    fn set_point(&mut self, _setpoint: Self::Input) -> &mut Self {
        self
    }
    fn update(&mut self, _measurement: Self::Input, _timestamp_now_us: i64) -> &mut Self {
        self
    }
}

#[derive(Default, Clone, Copy)]
pub struct Chain<T, U> {
    first: T,
    second: U,
}

impl<T, U> Chain<T, U> {
    pub fn into_inner(self) -> (T, U) {
        (self.first, self.second)
    }

    pub fn get_ref(&self) -> (&T, &U) {
        (&self.first, &self.second)
    }
    pub fn get_mut(&mut self) -> (&mut T, &mut U) {
        (&mut self.first, &mut self.second)
    }
}

impl<T: Regulator, U: Regulator<Input = T::Output>> Regulator for Chain<T, U> {
    type Input = T::Input;
    type Output = U::Output;
    fn set_point(&mut self, target: Self::Input) -> &mut Self {
        self.first.set_point(target);
        self
    }
    fn update(&mut self, measurement: Self::Input, timestamp_now_us: i64) -> &mut Self {
        let output = self.first.update(measurement, timestamp_now_us).output();
        self.second.update(output, timestamp_now_us);
        self
    }
    fn output(&mut self) -> Self::Output {
        self.second.output()
    }

    fn reset(&mut self) -> &mut Self {
        self.first.reset();
        self.second.reset();
        self
    }
}

//电流
#[derive(Debug, Clone, Copy)]
pub enum Current {
    Dq(f32, f32),         //转子直角坐标系（旋转坐标系）
    αβ(f32, f32),         //定子直角坐标系（静止坐标系）
    Phase(f32, f32, f32), //相电流，三向静止坐标系
}

impl Current {
    pub fn Id(&self) -> f32 {
        match self {
            Self::Dq(d, _) => *d,
            _ => panic!(""),
        }
    }
    pub fn set_Id(&mut self, id: f32) -> &mut Self {
        match self {
            Self::Dq(d, _) => {
                *d = id;
            }
            _ => panic!(""),
        }
        self
    }
    pub fn Iq(&self) -> f32 {
        match self {
            Self::Dq(_, q) => *q,
            _ => panic!(""),
        }
    }
    pub fn set_Iq(&mut self, iq: f32) -> &mut Self {
        match self {
            Self::Dq(_, q) => {
                *q = iq;
            }
            _ => panic!(""),
        }
        self
    }
}

impl Current {
    pub fn clarke(self) -> Self {
        if let Self::Phase(ia, ib, _ic) = self {
            let α = ia;
            let β = (ia + 2.0 * ib) * _1_SQRT_3;
            return Self::αβ(α, β);
        }
        panic!()
    }

    pub fn park(self, θ: f32) -> Self {
        if let Self::αβ(α, β) = self {
            let cos = libm::cosf(θ);
            let sin = libm::sinf(θ);
            let d = α * cos + β * sin;
            let q = -α * sin + β * cos;
            return Self::Dq(d, q);
        }
        panic!()
    }
}

//电压
#[derive(Debug, Clone, Copy)]
pub enum Voltage {
    Dq(f32, f32),          //转子直角坐标系（旋转坐标系）
    αβ(f32, f32),          //定子直角坐标系（静止坐标系）
    Phase3(f32, f32, f32), //三相电压，三向静止坐标系
    Phase2(f32, f32),
    Phase(f32),
    Duty3(f32, f32, f32),
    Duty2(f32, f32),
    Duty(f32),
}
impl Voltage {
    pub fn Ud(&self) -> f32 {
        match self {
            Self::Dq(d, _) => *d,
            _ => panic!(""),
        }
    }
    pub fn set_Ud(&mut self, ud: f32) -> &mut Self {
        match self {
            Self::Dq(d, _) => {
                *d = ud;
            }
            _ => panic!(""),
        }
        self
    }
    pub fn Uq(&self) -> f32 {
        match self {
            Self::Dq(_, q) => *q,
            _ => panic!(""),
        }
    }
    pub fn set_Uq(&mut self, uq: f32) -> &mut Self {
        match self {
            Self::Dq(_, q) => {
                *q = uq;
            }
            _ => panic!(""),
        }
        self
    }
    pub fn Uα(&self) -> f32 {
        match self {
            Self::αβ(α, _) => *α,
            _ => panic!(""),
        }
    }
    pub fn Uβ(&self) -> f32 {
        match self {
            Self::αβ(_, β) => *β,
            _ => panic!(""),
        }
    }
    pub fn Ua(&self) -> f32 {
        match self {
            Self::Phase3(a, _, _) => *a,
            Self::Phase2(a, _) => *a,
            Self::Phase(a) => *a,
            _ => panic!(""),
        }
    }
    pub fn Ub(&self) -> f32 {
        match self {
            Self::Phase3(_, b, _) => *b,
            Self::Phase2(_, b) => *b,
            Self::Phase(b) => *b,
            _ => panic!(""),
        }
    }
    pub fn Uc(&self) -> f32 {
        match self {
            Self::Phase3(_, _, c) => *c,
            Self::Phase(c) => *c,
            _ => panic!(""),
        }
    }
}

impl Voltage {
    //逆Park变换
    pub fn ipark(self, θ: f32) -> Self {
        if let Self::Dq(d, q) = self {
            let cos = libm::cosf(θ);
            let sin = libm::sinf(θ);
            let α = cos * d - sin * q;
            let β = sin * d + cos * q;
            return Self::αβ(α, β);
        }
        panic!()
    }
    //逆Clarke变换
    pub fn iclarke(self) -> Self {
        if let Self::αβ(α, β) = self {
            let a = α;
            let b = -α * 0.5 + SQRT_3_2 * β;
            let c = -α * 0.5 - SQRT_3_2 * β;
            return Self::Phase3(a, b, c);
        }
        panic!()
    }

    pub fn add(self, offset: f32) -> Self {
        match self {
            Self::Phase3(a, b, c) => Self::Phase3(a + offset, b + offset, c + offset),
            Self::Phase2(a, b) => Self::Phase2(a + offset, b + offset),
            Self::Phase(u) => Self::Phase(u + offset),
            Self::Duty3(a, b, c) => Self::Duty3(a + offset, b + offset, c + offset),
            Self::Duty2(a, b) => Self::Duty2(a + offset, b + offset),
            Self::Duty(a) => Self::Duty(a + offset),
            Self::αβ(α, β) => Self::αβ(α + offset, β + offset),
            Self::Dq(d, q) => Self::Dq(d + offset, q + offset),
        }
    }
    pub fn mul(self, val: f32) -> Self {
        match self {
            Self::Phase3(a, b, c) => Self::Phase3(a * val, b * val, c * val),
            Self::Phase2(a, b) => Self::Phase2(a * val, b * val),
            Self::Phase(u) => Self::Phase(u * val),
            Self::Duty3(a, b, c) => Self::Duty3(a * val, b * val, c * val),
            Self::Duty2(a, b) => Self::Duty2(a * val, b * val),
            Self::Duty(a) => Self::Duty(a * val),
            Self::αβ(α, β) => Self::αβ(α * val, β * val),
            Self::Dq(d, q) => Self::Dq(d * val, q * val),
        }
    }
    //限幅
    pub fn limit(self, min: f32, max: f32) -> Self {
        match self {
            Self::Phase3(a, b, c) => Self::Phase3(
                constrain!(a, min, max),
                constrain!(b, min, max),
                constrain!(c, min, max),
            ),
            Self::Phase2(a, b) => Self::Phase2(constrain!(a, min, max), constrain!(b, min, max)),
            Self::Phase(u) => Self::Phase(constrain!(u, min, max)),
            Self::Duty3(a, b, c) => Self::Duty3(
                constrain!(a, min, max),
                constrain!(b, min, max),
                constrain!(c, min, max),
            ),
            Self::Duty2(a, b) => Self::Duty2(constrain!(a, min, max), constrain!(b, min, max)),
            Self::Duty(a) => Self::Duty(constrain!(a, min, max)),
            Self::αβ(α, β) => Self::αβ(constrain!(α, min, max), constrain!(β, min, max)),
            Self::Dq(d, q) => Self::Dq(constrain!(d, min, max), constrain!(q, min, max)),
        }
    }

    //求占空比
    pub fn duty(self, power_supply: f32) -> Self {
        match self {
            Self::Phase3(a, b, c) => {
                Self::Duty3(a / power_supply, b / power_supply, c / power_supply)
            }
            Self::Phase2(a, b) => Self::Duty2(a / power_supply, b / power_supply),
            Self::Phase(u) => Self::Duty(u / power_supply),
            Self::αβ(α, β) => Self::Duty2(α / power_supply, β / power_supply),
            Self::Dq(d, q) => Self::Duty2(d / power_supply, q / power_supply),
            _ => panic!(),
        }
    }
}

impl Voltage {
    //正弦调制
    pub fn spwm(self, angle: f32, voltage_limit: f32) -> Voltage {
        //中心电压
        let center = voltage_limit * 0.5;
        if let Voltage::Dq(d, q) = self {
            Voltage::Dq(d, q)
                .ipark(normalize_angle!(angle))
                .iclarke()
                .add(center)
        } else {
            panic!("")
        }
    }
    //空间矢量PWM调制
    pub fn svpwm(self, angle_el: f32, voltage_limit: f32) -> Voltage {
        let q = self.Uq();
        if let Voltage::αβ(α, β) = self.ipark(angle_el) {
            let Uout = libm::sqrtf(α * α + β * β) / voltage_limit;
            let angle_el = if q.is_sign_positive() {
                normalize_angle!(angle_el + PI_2) //加90度后是参考电压矢量的位置
            } else {
                normalize_angle!(angle_el - PI_2)
            };
            //根据角度计算当前扇区
            let sector = libm::floorf(angle_el / PI_3) + 1.;
            // BUG 单精度计算在esp32平台会出现意外结果
            let a = sector as f64 * PI_3 as f64 - angle_el as f64;
            let T1 = SQRT_3 * libm::sinf(a as f32) * Uout;
            // BUG 单精度计算在esp32平台会出现意外结果
            let a = angle_el as f64 - (sector as f64 - 1.) * PI_3 as f64;
            let T2 = SQRT_3 * libm::sinf(a as f32) * Uout;
            let T0 = 1. - T1 - T2; //零矢量作用时间
            let sector = sector as i32;
            //计算a b c相占空比时长
            let (Ta, Tb, Tc) = match sector {
                1 => {
                    let Ta = T1 + T2 + T0 / 2.;
                    let Tb = T2 + T0 / 2.;
                    let Tc = T0 / 2.;
                    (Ta, Tb, Tc)
                }
                2 => {
                    let Ta = T1 + T0 / 2.;
                    let Tb = T1 + T2 + T0 / 2.;
                    let Tc = T0 / 2.;
                    (Ta, Tb, Tc)
                }
                3 => {
                    let Ta = T0 / 2.;
                    let Tb = T1 + T2 + T0 / 2.;
                    let Tc = T2 + T0 / 2.;
                    (Ta, Tb, Tc)
                }
                4 => {
                    let Ta = T0 / 2.;
                    let Tb = T1 + T0 / 2.;
                    let Tc = T1 + T2 + T0 / 2.;
                    (Ta, Tb, Tc)
                }
                5 => {
                    let Ta = T2 + T0 / 2.;
                    let Tb = T0 / 2.;
                    let Tc = T1 + T2 + T0 / 2.;
                    (Ta, Tb, Tc)
                }
                6 => {
                    let Ta = T1 + T2 + T0 / 2.;
                    let Tb = T0 / 2.;
                    let Tc = T1 + T0 / 2.;
                    (Ta, Tb, Tc)
                }
                _ => (0., 0., 0.),
            };

            //计算相电压
            let ua = Ta * voltage_limit;
            let ub = Tb * voltage_limit;
            let uc = Tc * voltage_limit;
            Voltage::Phase3(ua, ub, uc)
        } else {
            panic!("")
        }
    }

    pub fn svpwm_simplefoc(self, angle_el: f32, voltage_limit: f32) -> Voltage {
        let q = self.Uq();
        let d = self.Ud();
        let (Uout, angle_el) = if d > 0. {
            let Uout = libm::sqrtf(d * d + q * q) / voltage_limit;
            let angle_el = normalize_angle!(angle_el + libm::atan2f(q, d)); //libm::atan2f(q, d)
            (Uout, angle_el)
        } else {
            let Uout = q / voltage_limit;
            let angle_el = normalize_angle!(angle_el + PI_2);
            (Uout, angle_el)
        };
        //根据角度计算当前扇区
        let sector = libm::floorf(angle_el / PI_3) + 1.;
        //计算两个非零矢量作用时间
        // BUG 单精度计算在esp32平台会出现意外结果，所以改用双精度计算
        let a = sector as f64 * PI_3 as f64 - angle_el as f64;
        let T1 = SQRT_3 * libm::sinf(a as f32) * Uout;
        let a = angle_el as f64 - (sector as f64 - 1.) * PI_3 as f64;
        let T2 = SQRT_3 * libm::sinf(a as f32) * Uout;
        let T0 = 1. - T1 - T2; //零矢量作用时间

        //计算a b c相占空比时长
        let sector = sector as i32;
        let (Ta, Tb, Tc) = match sector {
            1 => {
                let Ta = T1 + T2 + T0 / 2.;
                let Tb = T2 + T0 / 2.;
                let Tc = T0 / 2.;
                (Ta, Tb, Tc)
            }
            2 => {
                let Ta = T1 + T0 / 2.;
                let Tb = T1 + T2 + T0 / 2.;
                let Tc = T0 / 2.;
                (Ta, Tb, Tc)
            }
            3 => {
                let Ta = T0 / 2.;
                let Tb = T1 + T2 + T0 / 2.;
                let Tc = T2 + T0 / 2.;
                (Ta, Tb, Tc)
            }
            4 => {
                let Ta = T0 / 2.;
                let Tb = T1 + T0 / 2.;
                let Tc = T1 + T2 + T0 / 2.;
                (Ta, Tb, Tc)
            }
            5 => {
                let Ta = T2 + T0 / 2.;
                let Tb = T0 / 2.;
                let Tc = T1 + T2 + T0 / 2.;
                (Ta, Tb, Tc)
            }
            6 => {
                let Ta = T1 + T2 + T0 / 2.;
                let Tb = T0 / 2.;
                let Tc = T1 + T0 / 2.;
                (Ta, Tb, Tc)
            }
            _ => (0., 0., 0.),
        };

        //计算相电压和中心
        let ua = Ta * voltage_limit;
        let ub = Tb * voltage_limit;
        let uc = Tc * voltage_limit;
        // println!("{angle_el},{Uout},{sector},{T0},{T1},{T2},{Ta},{Tb},{Tc},{ua},{ub},{uc}");
        Voltage::Phase3(ua, ub, uc)
    }
}

impl core::ops::Add<f32> for Voltage {
    type Output = Self;
    fn add(self, rhs: f32) -> Self::Output {
        self.add(rhs)
    }
}

impl core::ops::Mul<f32> for Voltage {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self::Output {
        self.mul(rhs)
    }
}

impl core::ops::Div<f32> for Voltage {
    type Output = Self;
    fn div(self, rhs: f32) -> Self::Output {
        self.mul(rhs.recip())
    }
}
