pub struct PID {
    pub enabled: bool,
    pub p: f64,
    pub i: f64,
    pub d: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub val: f64,
    pub duty_cycle: f64,
    pub prev_val: f64,
    pub error: f64,
    pub prev_time: u64,
    pub target: f64,
}

#[allow(dead_code)]
impl PID {
    pub fn new() -> Self {
        PID {
            enabled: true,
            p: 0.0,
            i: 0.0,
            d: 0.0,
            kp: 1.0,
            ki: 0.0,
            kd: 0.0,
            val: 0.0,
            duty_cycle: 0.0,
            prev_val: 0.0,
            error: 0.0,
            prev_time: 0,
            target: 1.0,
        }
    }

    pub fn calculate(&mut self, position: f64, now: u64) -> f64 {
        // convert time window in seconds
        let current_error = self.target - position;

        self.p = self.kp * current_error;
        // only integrate when PID is enabled
        if self.enabled {
            // only integrate when not saturated (anti-windup) and when p part alone does not already saturate the output
            if (self.duty_cycle < 1.0 && self.duty_cycle > -1.0) && (self.p < 1.0 && self.p > -1.0)
            {
                self.i += self.ki * current_error;
            }
        }
        self.d = self.kd * (current_error - self.error) / (now - self.prev_time) as f64;
        self.error = current_error;
        self.prev_val = position;
        self.prev_time = now;
        self.val = self.p + self.i + self.d;
        self.val
    }

    pub fn get_value(&mut self, position: f64, now: u64) -> f64 {
        let pid_val = self.calculate(position, now);

        self.duty_cycle = pid_val;

        self.duty_cycle = self.target;
        self.duty_cycle
    }
}
