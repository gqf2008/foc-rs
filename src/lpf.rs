//!低通滤波器
//!参考：https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/lowpass_filter.cpp
use crate::addf32;
#[derive(Debug)]
pub struct LowPassFilter {
    tf: f32, //时间常量
    timestamp_prev: i64,
    y_prev: f32,
}

impl LowPassFilter {
    pub fn new(tf: f32) -> Self {
        Self {
            tf,
            timestamp_prev: 0,
            y_prev: 0.,
        }
    }
}

impl LowPassFilter {
    pub fn do_filter(&mut self, x: f32, timestamp_us: i64) -> f32 {
        let mut dt = (timestamp_us - self.timestamp_prev) as f32 * 1e-6;
        self.timestamp_prev = timestamp_us;
        if dt.is_sign_negative() {
            dt = 1e-3;
        } else if dt > 0.3 {
            self.y_prev = x;
            return x;
        }
        // 一阶低通滤波的算法公式为：
        // Y(n)=αX(n) + (1-α)Y(n-1)
        // 式中：α=滤波系数；X(n)=本次采样值；Y(n-1)=上次滤波输出值；Y(n)=本次滤波输出值。
        //计算滤波系数
        let alpha = self.tf / (self.tf + dt);
        // let y = alpha * x + (1.0 - alpha) * self.y_prev;
        let y = addf32!(alpha * self.y_prev, addf32!(1.0, -alpha) * x);
        self.y_prev = y;
        y as f32
    }
}
