#[derive(Clone)]
pub struct MotorData {
    pub target: f64,
    pub actual: f64,
    pub enc: u16,
    pub p: f64,
    pub i: f64,
    pub d: f64,
    pub pwr: u16,
}

impl Default for MotorData {
    fn default() -> Self {
        MotorData {
            target: 0.0,
            actual: 0.0,
            enc: 0,
            p: 0.0,
            i: 0.0,
            d: 0.0,
            pwr: 0,
        }
    }
}
