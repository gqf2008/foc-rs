pub struct Pid {
    pub P: f32,
    pub I: f32,
    pub D: f32,
    pub output_ramp: f32,
    pub limit: f32,

    error_prev: f32,
    output_prev: f32,
    integral_prev: f32,
    timestamp_prev: i64,
}

impl Pid {
    pub fn new(P: f32, I: f32, D: f32, ramp: f32, limit: f32) -> Self {
        Pid {
            P,
            I,
            D,
            limit,
            output_ramp: ramp,
            error_prev: 0.,
            output_prev: 0.,
            integral_prev: 0.,
            timestamp_prev: 0,
        }
    }
}

impl Pid {
    pub fn reset(&mut self) -> &mut Self {
        self.integral_prev = 0.;
        self.output_prev = 0.;
        self.error_prev = 0.;
        self
    }
    pub fn output(&mut self, error: f32, timestamp_us: i64) -> f32 {
        let timestamp_now = timestamp_us;
        let ts = (timestamp_now - self.timestamp_prev) as f32 / 1000000.;
        self.timestamp_prev = timestamp_now;
        let ts = if ts <= 0. || ts > 0.5 { 0.001 } else { ts };

        let proportional = self.P * error;
        let integral = if self.I > 0. {
            (self.integral_prev as f64
                + (self.I * ts * 0.5 * (error as f64 + self.error_prev as f64) as f32) as f64)
                as f32
        } else {
            0.
        };

        // antiwindup - limit the output
        let integral = if integral < -self.limit {
            -self.limit
        } else if integral > self.limit {
            self.limit
        } else {
            integral
        };

        let derivative = if self.D > 0. {
            self.D * (error as f64 - self.error_prev as f64) as f32 / ts
        } else {
            0.
        };
        let output = (proportional as f64 + integral as f64 + derivative as f64) as f32;
        let mut output = if output < -self.limit {
            -self.limit
        } else if output > self.limit {
            self.limit
        } else {
            output
        };
        println!("fpid:{proportional},{integral},{derivative},{output}");
        if self.output_ramp > 0. {
            let output_rate = (output as f64 - self.output_prev as f64) as f32 / ts;
            if output_rate > self.output_ramp {
                output = (self.output_prev as f64 + (self.output_ramp * ts) as f64) as f32;
            } else if output_rate < -self.output_ramp {
                output = (self.output_prev as f64 - (self.output_ramp * ts) as f64) as f32;
            }
        }
        self.integral_prev = integral;
        self.output_prev = output;
        self.error_prev = error;
        output
    }
}
