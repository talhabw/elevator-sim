use std::{
    cell::RefCell,
    rc::Rc,
    thread,
    time::{Duration, Instant},
};

use elevator_sim::{
    Elevator, ElevatorController, ElevatorPIDController, ElevatorPhysics, Encoder,
    SimulatedEncoder, SimulatedMotor,
};

fn main() {
    println!("Elevator Simulation Starting...");
    let encoder = Rc::new(RefCell::new(SimulatedEncoder::new(0.0)));
    let motor = Rc::new(RefCell::new(SimulatedMotor::new()));

    let mut physics = ElevatorPhysics::new(1.0, encoder.borrow().get_position(), 0.0);

    let mut elevator = Elevator::new();

    let mut elevator_controller = ElevatorPIDController::new(
        Rc::clone(&encoder),
        Rc::clone(&motor),
        0.0,
        0.0,
        0.0,
        5.0,
        0.1,
    );

    let mut previous = Instant::now();
    loop {
        let now = Instant::now();
        let dt_secs = now.duration_since(previous).as_secs_f64();
        previous = now;

        if let Some(floor) = elevator_controller.get_current_floor() {
            let _ = elevator.notify_reached_floor(floor);
        }

        // State Loop - decide where to go -> outputs 'target_floor'
        elevator.state_loop(dt_secs);
        elevator_controller.set_target_floor(elevator.get_target_floor());

        // Control Loop - decide how to go -> outputs 'voltage'
        elevator_controller.tick(dt_secs);
        physics.set_voltage(motor.borrow().get_voltage());

        // Physics Loop - decide what happened -> outputs 'position'
        physics.update(dt_secs);
        encoder.borrow_mut().set_position(physics.get_position());

        thread::sleep(Duration::from_millis(20));
    }
}
