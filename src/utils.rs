#[derive(Clone)]
pub struct MotorData {
    pub target: f32,
    pub actual: f32,
    pub enc: u32,
    pub p: f32,
    pub i: f32,
    pub d: f32,
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
