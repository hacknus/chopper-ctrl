extern crate num_traits;

pub const MAV_LENGTH: usize = 64;

pub struct Mav<T> {
    history: [Option<T>; MAV_LENGTH],
    write_idx: usize,
    count: usize,
}

impl<T> Mav<T>
where
    T: Copy + core::ops::Add<Output = T> + core::ops::Div<Output = T> + From<u8>,
{
    pub fn new() -> Self {
        Self {
            history: [None; MAV_LENGTH],
            write_idx: 0,
            count: 0,
        }
    }

    pub fn push(&mut self, value: Option<T>) {
        self.history[self.write_idx] = value;
        self.write_idx = (self.write_idx + 1) % self.history.len();
        if self.count < self.history.len() {
            self.count += 1;
        }
    }

    pub fn evaluate(&self) -> Option<T> {
        if self.count == 0 {
            return None;
        }

        let mut sum = T::from(0);
        let mut valid_count = 0;

        for i in 0..self.count {
            if let Some(value) = self.history[i] {
                sum = sum + value;
                valid_count += 1;
            }
        }

        if valid_count > 0 {
            Some(sum / T::from(valid_count as u8))
        } else {
            None
        }
    }
}
