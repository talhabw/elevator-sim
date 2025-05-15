use std::cell::RefCell;
use std::rc::Rc;

use crate::control::pid::PIDController;
use crate::core::{Encoder, Motor};

pub trait ElevatorController {
    fn set_target_floor(&mut self, floor: i8);
    fn tick(&mut self, delta_time: f64);
    fn get_current_floor(&self) -> Option<i8>;
    fn has_reached_target(&self) -> bool;
}

pub struct ElevatorPIDController<'a> {
    encoder: Rc<RefCell<dyn Encoder + 'a>>,
    motor: Rc<RefCell<dyn Motor + 'a>>,
    pid: PIDController,
    floor_height: f64,
    target_floor: i8,
    precision: f64,
}

impl<'a> ElevatorPIDController<'a> {
    pub fn new(
        encoder: Rc<RefCell<impl Encoder + 'a>>,
        motor: Rc<RefCell<impl Motor + 'a>>,
        kp: f64,
        ki: f64,
        kd: f64,
        floor_height: f64,
        precision: f64,
    ) -> Self {
        ElevatorPIDController {
            encoder,
            motor,
            pid: PIDController::new(kp, ki, kd),
            floor_height,
            precision,
            target_floor: 0,
        }
    }

    pub fn get_target_height(&self) -> f64 {
        self.target_floor as f64 * self.floor_height
    }

    pub fn get_current_height(&self) -> f64 {
        self.encoder.borrow().get_position()
    }
}

impl ElevatorController for ElevatorPIDController<'_> {
    fn set_target_floor(&mut self, floor: i8) {
        if self.target_floor != floor {
            self.target_floor = floor;
            self.pid.reset();
        }
    }

    fn tick(&mut self, dt: f64) {
        let current_pos = self.encoder.borrow().get_position();
        let target_pos = self.target_floor as f64 * self.floor_height;
        let error = target_pos - current_pos;

        let voltage = self.pid.update(error, dt);
        self.motor
            .borrow_mut()
            .set_voltage(voltage.clamp(-12.0, 12.0));
    }

    fn get_current_floor(&self) -> Option<i8> {
        let current_floor = self.encoder.borrow().get_position() / self.floor_height;
        let rounded = current_floor.round();

        if (current_floor - rounded).abs() <= self.precision {
            Some(rounded as i8)
        } else {
            None
        }
    }

    fn has_reached_target(&self) -> bool {
        let current = self.encoder.borrow().get_position();
        let target = self.target_floor as f64 * self.floor_height;

        (current - target).abs() < self.precision
    }
}
