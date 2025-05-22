pub struct PIDController {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    previous_error: f64,
    min_limit: f64,
    max_limit: f64,
}

impl PIDController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_error: 0.0,
            min_limit: -f64::INFINITY,
            max_limit: f64::INFINITY,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
    }

    pub fn get_integral(&self) -> f64 {
        self.integral
    }

    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        self.integral += error * dt;
        let derivative = (error - self.previous_error) / dt;
        self.previous_error = error;

        let raw_voltage = self.kp * error + self.ki * self.integral + self.kd * derivative;

        raw_voltage.clamp(self.min_limit, self.max_limit)
    }

    pub fn set_output_limits(&mut self, min_limit: f64, max_limit: f64) {
        self.min_limit = min_limit;
        self.max_limit = max_limit;
    }
}

pub struct FeedForward {
    pub kg: f64,
    pub kv: f64,
    pub ka: f64,
}

impl FeedForward {
    pub fn new(kg: f64, kv: f64, ka: f64) -> Self {
        Self { kg, kv, ka }
    }
}
