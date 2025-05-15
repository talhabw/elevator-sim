use crate::core::{Encoder, Motor};

pub struct SimulatedEncoder {
    position: f64,
}

impl SimulatedEncoder {
    pub fn new(initial_position: f64) -> Self {
        Self {
            position: initial_position,
        }
    }
}

impl Encoder for SimulatedEncoder {
    fn get_position(&self) -> f64 {
        self.position
    }

    fn set_position(&mut self, position: f64) {
        self.position = position;
    }
}

pub struct SimulatedMotor {
    voltage: f64,
}

impl Default for SimulatedMotor {
    fn default() -> Self {
        SimulatedMotor::new()
    }
}

impl SimulatedMotor {
    pub fn new() -> Self {
        Self { voltage: 0.0 }
    }

    pub fn get_voltage(&self) -> f64 {
        self.voltage
    }
}

impl Motor for SimulatedMotor {
    fn set_voltage(&mut self, voltage: f64) {
        self.voltage = voltage;
    }
}
