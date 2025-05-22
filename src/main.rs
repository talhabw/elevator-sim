use std::{cell::RefCell, rc::Rc, sync::mpsc, thread, time::Duration};

use chrono::Local;
use elevator_sim::{
    Elevator, ElevatorController, ElevatorDirection, ElevatorPIDController, ElevatorPhysics,
    ElevatorRequest, Encoder, SimulatedEncoder, SimulatedMotor, ui,
};
use fern::Dispatch;

const TIME_STEP: f32 = 1.0 / 60.0;

pub enum UserCommand {
    HallCall(ElevatorRequest),
    CarCall(i8),
    Quit,
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

    // get elevator calls using mpsc::channel
    let (input_tx, input_rx) = mpsc::channel::<UserCommand>();

    let input_thread = thread::spawn(move || {
        loop {
            let mut input = String::new();
            std::io::stdin().read_line(&mut input).unwrap();
            let parts: Vec<&str> = input.split_whitespace().collect();

            if parts.is_empty() {
                continue;
            }

            match parts[0] {
                "h" => {
                    if parts.len() != 3 {
                        println!("Usage: h <floor> <u|d>");
                        continue;
                    }
                    let floor: i8 = match parts[1].parse() {
                        Ok(f) => f,
                        Err(_) => continue,
                    };
                    let direction = match parts[2] {
                        "u" => ElevatorDirection::UP,
                        "d" => ElevatorDirection::DOWN,
                        _ => continue,
                    };
                    let request = ElevatorRequest::new(direction, floor);
                    input_tx.send(UserCommand::HallCall(request)).unwrap();
                }
                "c" => {
                    if parts.len() != 2 {
                        println!("Usage: c <floor>");
                        continue;
                    }
                    let floor: i8 = match parts[1].parse() {
                        Ok(f) => f,
                        Err(_) => continue,
                    };
                    input_tx.send(UserCommand::CarCall(floor)).unwrap();
                }
                "q" => {
                    input_tx.send(UserCommand::Quit).unwrap();
                    break;
                }
                _ => {
                    println!("Unknown command");
                }
            }
        }
    });

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

        if let Some(floor) = elevator_controller.get_current_floor() {
            let _ = elevator.notify_reached_floor(floor);
        }

        ui::log_to_terminal(&elevator, &elevator_controller, &physics, motor.borrow());
        thread::sleep(time_step);
    }

    input_thread.join().unwrap(); // Wait for the input thread to finish
}
