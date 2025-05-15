use std::{
    cell::RefCell,
    rc::Rc,
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};

use chrono::Local;
use elevator_sim::{
    Elevator, ElevatorController, ElevatorDirection, ElevatorPIDController, ElevatorPhysics,
    ElevatorRequest, ElevatorState, Encoder, SimulatedEncoder, SimulatedMotor,
};
use fern::Dispatch;

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

    let mut physics = ElevatorPhysics::new(100.0, encoder.borrow().get_position(), 0.0);

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
    let (output_tx, output_rx) = mpsc::channel::<DisplayData>();

    let _ = input_tx.send(UserCommand::CarCall(5));
    let _ = input_tx.send(UserCommand::CarCall(7));
    let _ = input_tx.send(UserCommand::CarCall(12));
    let _ = input_tx.send(UserCommand::CarCall(15));

    let _ = input_tx.send(UserCommand::HallCall(ElevatorRequest::new(
        ElevatorDirection::DOWN,
        8,
    )));
    let _ = input_tx.send(UserCommand::HallCall(ElevatorRequest::new(
        ElevatorDirection::DOWN,
        12,
    )));

    let _ = input_tx.send(UserCommand::CarCall(3));
    let _ = input_tx.send(UserCommand::CarCall(0));

    let mut previous = Instant::now();
    loop {
        let now = Instant::now();
        let dt_secs = now.duration_since(previous).as_secs_f64() + 0.7;
        previous = now;
        print!("\x1B[2J\x1B[1;1H");

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
        elevator.state_loop(dt_secs);
        elevator_controller.set_target_floor(elevator.get_target_floor());

        // Control Loop - decide how to go -> outputs 'voltage'
        elevator_controller.tick(dt_secs);
        physics.set_voltage(motor.borrow().get_voltage());

        // Physics Loop - decide what happened -> outputs 'position'
        physics.update(dt_secs);
        encoder.borrow_mut().set_position(physics.get_position());

        // clone data to ui thread ???
        let display_data = DisplayData {
            logical_current_floor: elevator.get_current_floor(),
            sensor_current_floor: None,
            current_height: elevator_controller.get_current_height(),
            target_height: elevator_controller.get_target_height(),
            voltage: motor.borrow().get_voltage(),
            velocity: physics.get_velocity(),
            target_floor: elevator.get_target_floor(),
            current_state: elevator.get_state().clone(),
            sensor_at_target: elevator_controller.has_reached_target(),
            active_requests: elevator.get_all_requests().clone(),
        };
        println!("Time: {}", Local::now().format("%Y-%m-%d %H:%M:%S%.3f"));
        print!("{:#?}", display_data);

        if output_tx.send(display_data).is_err() {
            println!("[Main Sim] UI thread seems to have exited. Shutting down.");
            break;
        }

        thread::sleep(Duration::from_millis(20));
    }
}
