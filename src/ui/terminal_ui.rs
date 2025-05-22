use crate::{
    Elevator, ElevatorController, ElevatorPIDFFController, ElevatorPhysics, ElevatorRequest,
    ElevatorState, SimulatedMotor,
};
use std::cell::Ref;

pub struct DisplayData {
    pub logic_current_floor: i8,
    pub logic_target_floor: i8,
    pub elevator_state: String,
    pub requests: Vec<String>,
    pub waiting_time: f64,

    pub controller_estimated_current_floor: Option<i8>,
    pub current_height: f64,
    pub target_height: f64,

    pub position: f32,
    pub velocity: f32,
    pub motor_voltage: f32,
}

pub fn format_request(request: &ElevatorRequest) -> String {
    format!("Floor: {}, Dir: {:?}", request.floor, request.direction)
}

pub fn format_elevator_state(state: &ElevatorState) -> String {
    match state {
        ElevatorState::IDLE => "IDLE".to_string(),
        ElevatorState::MOVING(_) => "MOVING".to_string(),
        ElevatorState::WAITING(_, _) => "WAITING".to_string(),
    }
}

pub fn log_to_terminal(
    elevator: &Elevator,
    controller: &ElevatorPIDFFController,
    physics: &ElevatorPhysics,
    motor: Ref<SimulatedMotor>,
) {
    let display_data = DisplayData {
        logic_current_floor: elevator.get_current_floor(),
        logic_target_floor: elevator.get_target_floor(),
        elevator_state: format_elevator_state(elevator.get_state()),
        requests: elevator.get_all_requests().map(format_request).collect(),
        waiting_time: elevator.get_waiting_time(),
        controller_estimated_current_floor: controller.get_current_floor(),
        current_height: controller.get_current_height(),
        target_height: controller.get_target_height(),
        position: physics.get_position(),
        velocity: physics.get_velocity(),
        motor_voltage: motor.get_voltage() as f32,
    };

    print!("\x1B[2J\x1B[1;1H");

    println!("--- Elevator Logic ---");
    println!("State: {}", display_data.elevator_state);
    println!("Current Floor: {}", display_data.logic_current_floor);
    println!("Target Floor: {}", display_data.logic_target_floor);
    println!("Waiting Time: {:.2}s", display_data.waiting_time);
    println!("Requests:");
    if display_data.requests.is_empty() {
        println!("  No active requests");
    } else {
        for req_str in &display_data.requests {
            println!("  - {}", req_str);
        }
    }

    println!("\n--- Elevator Controller ---");
    println!(
        "Est. Current Floor: {}",
        match display_data.controller_estimated_current_floor {
            Some(f) => f.to_string(),
            None => "N/A (between floors)".to_string(),
        }
    );
    println!("Current Height: {:.2}m", display_data.current_height);
    println!("Target Height: {:.2}m", display_data.target_height);

    println!("\n--- Physics Engine ---");
    println!("Position: {:.2}m", display_data.position);
    println!("Velocity: {:.2}m/s", display_data.velocity);
    println!("Motor Voltage: {:.2}V", display_data.motor_voltage);
    println!("----------------------\n");
}
