use std::{cell::RefCell, rc::Rc, sync::mpsc, thread, time::Duration};

use chrono::Local;
use elevator_sim::{
    Elevator, ElevatorController, ElevatorPIDController, ElevatorPhysics, ElevatorRequest,
    ElevatorState, Encoder, SimulatedEncoder, SimulatedMotor,
};
use fern::Dispatch;

const TIME_STEP: f32 = 1.0 / 60.0;

pub enum UserCommand {
    HallCall(ElevatorRequest),
    CarCall(i8),
    Quit,
    NoOp,
}

#[derive(Clone, Debug)]
pub struct DisplayData {
    pub logical_current_floor: i8,
    pub sensor_current_floor: Option<i8>,
    pub current_height: f64,
    pub target_height: f64,
    pub target_floor: i8,
    pub voltage: f64,
    pub velocity: f64,
    pub current_state: ElevatorState,
    pub sensor_at_target: bool,
    pub active_requests: Vec<ElevatorRequest>,
}

fn setup_logger() -> Result<(), Box<dyn std::error::Error>> {
    Dispatch::new()
        .format(|out, message, record| {
            out.finish(format_args!(
                "[{}][{}][{}] {}",
                Local::now().format("%Y-%m-%d %H:%M:%S"),
                record.level(),
                record.target(),
                message
            ))
        })
        .level(log::LevelFilter::Debug)
        .chain(std::fs::File::create("temp.log")?)
        .chain(std::io::stdout())
        .apply()?;

    Ok(())
}

fn main() {
    setup_logger().expect("failed");

    println!("elevator-sim.");
    let encoder = Rc::new(RefCell::new(SimulatedEncoder::new(0.0)));
    let motor = Rc::new(RefCell::new(SimulatedMotor::new()));

    let mut physics = ElevatorPhysics::new(100.0, 1.0, 3.0, 100.0, -9.81, 0.1);

    let mut elevator = Elevator::new();

    let mut elevator_controller = ElevatorPIDController::new(
        Rc::clone(&encoder),
        Rc::clone(&motor),
        1.0,
        0.0,
        25.0,
        5.0,
        0.1,
    );

    // Create channels for communication
    // User input from UI thread to main thread
    let (input_tx, input_rx) = mpsc::channel::<UserCommand>();
    // Display data from main thread to UI thread
    // let (output_tx, output_rx) = mpsc::channel::<DisplayData>();

    let time_step = Duration::from_secs_f64(TIME_STEP);
    loop {
        let dt = time_step.as_secs_f64();
        print!("\x1B[2J\x1B[1;1H");

    let time_step = Duration::from_secs_f32(TIME_STEP);
    let dt = time_step.as_secs_f64();
    loop {
        // Process user input from UI thread (non-blocking)
        match input_rx.try_recv() {
            Ok(UserCommand::HallCall(request)) => match elevator.hall_call(request) {
                Ok(_) => println!("hall call success: {:#?}.", request),
                Err(e) => println!("hall call error: {:?}. {:#?})", e, request),
            },
            Ok(UserCommand::CarCall(floor)) => match elevator.car_call(floor) {
                Ok(_) => println!("car call success: {}", floor),
                Err(e) => println!("car call error: {:?}. {}", e, floor),
            },
            Ok(UserCommand::Quit) => {
                println!("shutdown");
                break;
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                println!("thread disconnected");
                break;
            }
            _ => {}
        }

        if let Some(floor) = elevator_controller.get_current_floor() {
            let _ = elevator.notify_reached_floor(floor);
        }

        // State Loop - decide where to go -> outputs 'target_floor'
        elevator.state_loop(dt);
        elevator_controller.set_target_floor(elevator.get_target_floor());

        // Control Loop - decide how to go -> outputs 'voltage'
        elevator_controller.tick(dt);
        physics.set_voltage(motor.borrow().get_voltage() as f32);

        // Physics Loop - decide what happened -> outputs 'position'
        physics.update();
        encoder
            .borrow_mut()
            .set_position(physics.get_position() as f64);

        // if output_tx.send().is_err() {
        // break;
        // }

        thread::sleep(time_step);
    }
}
