pub trait Encoder {
    fn get_position(&self) -> f64;
    fn set_position(&mut self, position: f64);
}

pub trait Motor {
    fn set_voltage(&mut self, voltage: f64);
}

pub trait Button {
    fn press(&mut self);
}
