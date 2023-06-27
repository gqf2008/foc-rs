pub struct Pid {
    pub P: f64,
    pub I: f64,
    pub D: f64,
    pub output_ramp: f64,
    pub limit: f64,

    error_prev: f64,
    output_prev: f64,
    integral_prev: f64,
    timestamp_prev: i64,
}

impl Pid {
    pub fn new(P: f32, I: f32, D: f32, ramp: f32, limit: f32) -> Self {
        Pid {
            P: P as f64,
            I: I as f64,
            D: D as f64,
            limit: limit as f64,
            output_ramp: ramp as f64,
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
        let error = error as f64;
        let timestamp_now = timestamp_us;
        let ts = (timestamp_now - self.timestamp_prev) as f64 / 1000000.;
        self.timestamp_prev = timestamp_now;
        let ts = if ts <= 0. || ts > 0.5 { 0.001 } else { ts };

        let proportional = self.P * error;
        let integral = if self.I > 0. {
            self.integral_prev as f64 + (self.I * ts * 0.5 * (error + self.error_prev))
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
            self.D * (error - self.error_prev) / ts
        } else {
            0.
        };
        let output = proportional + integral + derivative;
        let mut output = if output < -self.limit {
            -self.limit
        } else if output > self.limit {
            self.limit
        } else {
            output
        };

        if self.output_ramp > 0. {
            let output_rate = (output - self.output_prev) / ts;
            if output_rate > self.output_ramp {
                output = self.output_prev + (self.output_ramp * ts);
            } else if output_rate < -self.output_ramp {
                output = self.output_prev - (self.output_ramp * ts);
            }
        }
        self.integral_prev = integral;
        self.output_prev = output;
        self.error_prev = error;
        output as f32
    }
}
