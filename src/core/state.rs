use core::panic;
use std::{cmp::Ordering, collections::HashSet};

#[derive(PartialEq, Eq, Hash, Debug)]
pub enum ElevatorRequestErr {
    DUPLICATE,
    DENIED,
    CurrentFloor,
}

#[derive(PartialEq, Eq, Hash, Debug)]
pub enum ElevatorFloorReachErr {
    NotMoving,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
pub enum ElevatorDirection {
    UP,
    DOWN,
}

impl ElevatorDirection {
    fn opposite(&self) -> Self {
        match self {
            ElevatorDirection::UP => ElevatorDirection::DOWN,
            ElevatorDirection::DOWN => ElevatorDirection::UP,
        }
    }
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
pub enum ElevatorDoorsState {
    OPEN,
    CLOSED,
}

#[derive(PartialEq, Eq, Hash, Clone, Debug)]
pub enum ElevatorState {
    MOVING(ElevatorDirection),
    WAITING(ElevatorDirection, ElevatorDoorsState),
    IDLE,
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
pub struct ElevatorRequest {
    pub direction: ElevatorDirection,
    pub floor: i8,
}

impl ElevatorRequest {
    pub fn new(direction: ElevatorDirection, floor: i8) -> Self {
        ElevatorRequest { direction, floor }
    }

    fn recalculate_direction(&self, current_floor: i8) -> ElevatorDirection {
        match current_floor.cmp(&self.floor) {
            Ordering::Equal => self.direction,
            Ordering::Less => ElevatorDirection::UP,
            Ordering::Greater => ElevatorDirection::DOWN,
        }
    }
}

#[derive(PartialEq, Debug)]
pub struct Elevator {
    current_floor: i8,
    target_floor: i8,
    state: ElevatorState,
    request_buffer: HashSet<ElevatorRequest>,
    waiting_time: f64,
}

impl Default for Elevator {
    fn default() -> Self {
        Elevator::new()
    }
}

impl Elevator {
    pub fn new() -> Self {
        Self {
            current_floor: 0,
            target_floor: 0,
            state: ElevatorState::IDLE,
            request_buffer: HashSet::new(),
            waiting_time: 0.0,
        }
    }

    pub fn hall_call(&mut self, request: ElevatorRequest) -> Result<bool, ElevatorRequestErr> {
        match self.request_buffer.insert(request) {
            true => Ok(true),
            false => Err(ElevatorRequestErr::DUPLICATE),
            // _ => Err(ElevatorRequestErr::DENIED),
        }
    }

    pub fn car_call(&mut self, floor: i8) -> Result<bool, ElevatorRequestErr> {
        let request = ElevatorRequest {
            direction: match self.current_floor.cmp(&floor) {
                Ordering::Greater => ElevatorDirection::DOWN,
                Ordering::Less => ElevatorDirection::UP,
                Ordering::Equal => {
                    return Err(ElevatorRequestErr::CurrentFloor);
                }
            },
            floor,
        };

        match self.request_buffer.insert(request) {
            true => Ok(true),
            false => Err(ElevatorRequestErr::DUPLICATE),
            // _ => Err(ElevatorRequestErr::DENIED),
        }
    }

    fn get_target_on_the_way(
        &self,
        direction: ElevatorDirection,
        is_at_target: bool,
    ) -> Option<ElevatorRequest> {
        self.request_buffer
            .iter()
            .filter(|request| request.direction == direction)
            .filter(|request| match self.current_floor.cmp(&request.floor) {
                Ordering::Equal => true,
                Ordering::Less => {
                    direction == ElevatorDirection::UP
                        && (is_at_target || request.floor <= self.target_floor)
                }
                Ordering::Greater => {
                    direction == ElevatorDirection::DOWN
                        && (is_at_target || request.floor >= self.target_floor)
                }
            })
            .min_by_key(|request| (self.current_floor - request.floor).abs())
            .copied()
    }

    fn get_first_target_in_direction(
        &self,
        direction: ElevatorDirection,
    ) -> Option<ElevatorRequest> {
        let floors_in_direction = self
            .request_buffer
            .iter()
            .filter(|request| request.direction == direction);

        match direction {
            ElevatorDirection::UP => floors_in_direction
                .min_by_key(|request| request.floor)
                .copied(),
            ElevatorDirection::DOWN => floors_in_direction
                .max_by_key(|request| request.floor)
                .copied(),
        }
    }

    fn get_best_target_with_opposite_direction(
        &self,
        direction: ElevatorDirection,
    ) -> Option<ElevatorRequest> {
        let floors_in_direction = self
            .request_buffer
            .iter()
            .filter(|request| request.direction == direction.opposite());

        match direction {
            ElevatorDirection::UP => floors_in_direction
                .filter(|request| request.floor >= self.current_floor)
                .max_by_key(|request| request.floor)
                .copied(),
            ElevatorDirection::DOWN => floors_in_direction
                .filter(|request| request.floor <= self.current_floor)
                .min_by_key(|request| request.floor)
                .copied(),
        }
    }

    fn get_next_request_on_idle(&self) -> Option<ElevatorRequest> {
        self.get_first_target_in_direction(ElevatorDirection::UP)
            .or_else(|| self.get_first_target_in_direction(ElevatorDirection::DOWN))
    }

    fn get_next_request_while_moving(
        &self,
        direction: ElevatorDirection,
    ) -> Option<ElevatorRequest> {
        self.get_target_on_the_way(direction, false)
            .or_else(|| self.get_best_target_with_opposite_direction(direction))
    }

    fn get_next_request_after_waiting(
        &self,
        direction: ElevatorDirection,
    ) -> Option<ElevatorRequest> {
        self.get_target_on_the_way(direction, true)
            .or_else(|| self.get_first_target_in_direction(direction.opposite()))
            .or_else(|| self.get_first_target_in_direction(direction))
    }

    fn remove_finished_request(&mut self, direction: ElevatorDirection) {
        let _ = self
            .request_buffer
            .remove(&ElevatorRequest::new(direction, self.current_floor))
            || self.request_buffer.remove(&ElevatorRequest::new(
                direction.opposite(),
                self.current_floor,
            ));
    }

    pub fn state_loop(&mut self, dt_secs: f64) {
        match &self.state {
            ElevatorState::IDLE => {
                if let Some(request) = self.get_next_request_on_idle() {
                    self.target_floor = request.floor;

                    if self.current_floor == request.floor {
                        self.request_buffer.remove(&request);
                        self.state =
                            ElevatorState::WAITING(request.direction, ElevatorDoorsState::CLOSED);

                        return;
                    }

                    self.state =
                        ElevatorState::MOVING(request.recalculate_direction(self.current_floor));
                }
            }
            // todo, implement doors
            #[allow(unused_variables)]
            ElevatorState::WAITING(direction, doors) => {
                let direction = *direction;
                if self.waiting_time == 0.0 {
                    self.remove_finished_request(direction);
                }

                // todo better timer at one point, not a priority
                self.waiting_time += dt_secs;

                // After waiting period completes
                if self.waiting_time >= 5.0 {
                    self.waiting_time = 0.0;

                    if let Some(request) = self.get_next_request_after_waiting(direction) {
                        self.target_floor = request.floor;
                        self.state = ElevatorState::MOVING(
                            request.recalculate_direction(self.current_floor),
                        );
                    } else {
                        self.state = ElevatorState::IDLE;
                    }
                }
            }
            ElevatorState::MOVING(direction) => {
                if self.target_floor == self.current_floor {
                    // return self.state =
                    // ElevatorState::WAITING(*direction, ElevatorDoorsState::CLOSED);
                }

                if let Some(request) = self.get_next_request_while_moving(*direction) {
                    self.target_floor = request.floor;
                } else {
                    // this should never happen, because:
                    // the request that put the elevator in the moving mode should still be in the buffer.
                    panic!();
                    // todo: we can possibly put the state to idle instead of panicking.
                    // let's leave it for now so we can see if this ever happens.
                }
            }
        }
    }

    pub fn set_current_floor(&mut self, floor: i8) {
        self.current_floor = floor
    }

    pub fn get_target_floor(&self) -> i8 {
        self.target_floor
    }

    pub fn get_current_floor(&self) -> i8 {
        self.current_floor
    }

    pub fn get_state(&self) -> &ElevatorState {
        &self.state
    }

    pub fn get_all_requests(&self) -> Vec<ElevatorRequest> {
        self.request_buffer.iter().cloned().collect()
    }

    pub fn notify_reached_floor(&mut self, reached_floor: i8) -> Result<(), ElevatorFloorReachErr> {
        match self.state {
            ElevatorState::MOVING(direction) => {
                self.current_floor = reached_floor;

                if self.current_floor == self.target_floor {
                    self.state = ElevatorState::WAITING(direction, ElevatorDoorsState::CLOSED);
                }

                Ok(())
            }
            _ => Err(ElevatorFloorReachErr::NotMoving),
        }
    }
}

#[cfg(test)]
mod state_tests {
    use super::*;

    #[test]
    fn tbw_sceneario() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(8);

        let five_up_req = ElevatorRequest::new(ElevatorDirection::UP, 5);
        let three_up_req = ElevatorRequest::new(ElevatorDirection::UP, 3);

        assert_eq!(
            elevator.hall_call(five_up_req),
            Ok(true),
            "elevator got the hall call to five-up"
        );

        elevator.state_loop(5.1);

        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::DOWN),
            "elevator started to move down"
        );

        elevator.state_loop(5.1);
        elevator.set_current_floor(7);
        elevator.state_loop(5.1);

        assert_eq!(elevator.current_floor, 7, "elevator is at floor 7");
        assert_eq!(elevator.target_floor, 5, "elevator target = floor 5");

        assert_eq!(
            elevator.hall_call(three_up_req),
            Ok(true),
            "elevator got the hall call to three-up"
        );

        elevator.state_loop(5.1);

        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::DOWN),
            "elevator still moves down"
        );

        elevator.state_loop(5.1);

        assert_eq!(elevator.target_floor, 3, "elevator target = floor 3");

        assert_eq!(
            elevator.notify_reached_floor(3),
            Ok(()),
            "reached floor error"
        );

        elevator.state_loop(5.1);

        assert_eq!(elevator.target_floor, 5, "elevator target = floor 5");
    }

    // Utility function to simulate the elevator moving between floors
    fn simulate_movement(elevator: &mut Elevator, target_floor: i8) {
        while elevator.get_current_floor() != target_floor {
            let current = elevator.get_current_floor();
            let next = match elevator.state {
                ElevatorState::MOVING(direction) => match direction {
                    ElevatorDirection::UP => current + 1,
                    ElevatorDirection::DOWN => current - 1,
                },
                _ => current,
            };

            assert_eq!(elevator.notify_reached_floor(next), Ok(()), "NotMoving");

            elevator.state_loop(0.1); // Small time step for movement
        }
    }

    #[test]
    fn test_simple_up_request() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(0);

        // Hall call from floor 3 to go up
        let req = ElevatorRequest::new(ElevatorDirection::UP, 3);
        assert_eq!(
            elevator.hall_call(req),
            Ok(true),
            "Elevator should accept hall call"
        );

        // Initial state loop should transition to MOVING UP
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::UP),
            "Elevator should be moving up"
        );

        // Simulate movement to floor 3
        simulate_movement(&mut elevator, 3);

        // Should now be waiting at floor 3
        assert_eq!(
            elevator.state,
            ElevatorState::WAITING(ElevatorDirection::UP, ElevatorDoorsState::CLOSED),
            "Elevator should be waiting at floor 3"
        );

        // After waiting period, should return to IDLE
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::IDLE,
            "Elevator should return to IDLE"
        );
    }

    #[test]
    fn test_simple_down_request() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(5);

        // Hall call from floor 2 to go down
        let req = ElevatorRequest::new(ElevatorDirection::DOWN, 2);
        assert_eq!(
            elevator.hall_call(req),
            Ok(true),
            "Elevator should accept hall call"
        );

        // Initial state loop should transition to MOVING DOWN
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::DOWN),
            "Elevator should be moving down"
        );

        // Simulate movement to floor 2
        simulate_movement(&mut elevator, 2);

        // Should now be waiting at floor 2
        assert_eq!(
            elevator.state,
            ElevatorState::WAITING(ElevatorDirection::DOWN, ElevatorDoorsState::CLOSED),
            "Elevator should be waiting at floor 2"
        );

        // After waiting period, should return to IDLE
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::IDLE,
            "Elevator should return to IDLE"
        );
    }

    #[test]
    fn test_multiple_requests_same_direction() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(0);

        // Multiple hall calls going up
        let req1 = ElevatorRequest::new(ElevatorDirection::UP, 3);
        let req2 = ElevatorRequest::new(ElevatorDirection::UP, 5);
        let req3 = ElevatorRequest::new(ElevatorDirection::UP, 7);

        assert_eq!(elevator.hall_call(req1), Ok(true));
        assert_eq!(elevator.hall_call(req2), Ok(true));
        assert_eq!(elevator.hall_call(req3), Ok(true));

        // Start moving
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::UP),
            "Elevator should be moving up"
        );

        // Should stop at floor 3 first
        assert_eq!(elevator.get_target_floor(), 3);
        simulate_movement(&mut elevator, 3);

        // Wait at floor 3
        assert_eq!(
            elevator.state,
            ElevatorState::WAITING(ElevatorDirection::UP, ElevatorDoorsState::CLOSED)
        );

        // Next destination
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::UP),
            "Elevator should continue moving up"
        );
        assert_eq!(elevator.get_target_floor(), 5);

        // Continue to floor 5 and then 7
        simulate_movement(&mut elevator, 5);
        elevator.state_loop(5.1);
        simulate_movement(&mut elevator, 7);

        // After final floor, should return to IDLE
        elevator.state_loop(5.1);
        assert_eq!(elevator.state, ElevatorState::IDLE);
    }

    #[test]
    fn test_car_call_handling() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(3);

        // Car call to floor 7
        assert_eq!(elevator.car_call(7), Ok(true), "Should accept car call");

        // Should start moving up
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::UP),
            "Elevator should be moving up"
        );

        // Another car call while moving
        assert_eq!(
            elevator.car_call(10),
            Ok(true),
            "Should accept additional car call"
        );

        // Continue to floor 7
        simulate_movement(&mut elevator, 7);

        println!("{:#?}", elevator.state);

        // After brief wait, should continue to floor 10
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::UP),
            "Elevator should continue up to floor 10"
        );
        println!("{:#?}", elevator.state);
        println!("{:#?}", elevator.target_floor);

        // panics
        simulate_movement(&mut elevator, 10);

        // After reaching final destination, should return to IDLE
        elevator.state_loop(5.1);
        assert_eq!(elevator.state, ElevatorState::IDLE);
    }

    #[test]
    fn test_direction_change() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(5);

        // Requests in both directions
        let up_req = ElevatorRequest::new(ElevatorDirection::UP, 8);
        let down_req = ElevatorRequest::new(ElevatorDirection::DOWN, 2);

        assert_eq!(elevator.hall_call(up_req), Ok(true));
        assert_eq!(elevator.hall_call(down_req), Ok(true));

        // Should prioritize UP first (assumption based on implementation)
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::UP),
            "Elevator should move up first"
        );

        // Complete the UP request
        simulate_movement(&mut elevator, 8);
        elevator.state_loop(5.1);

        // Now should start moving DOWN for the second request
        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::DOWN),
            "Elevator should change direction to down"
        );

        // Complete the DOWN request
        simulate_movement(&mut elevator, 2);
        elevator.state_loop(5.1);

        // Finally return to IDLE
        assert_eq!(elevator.state, ElevatorState::IDLE);
    }

    #[test]
    fn test_request_pickup_en_route() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(1);

        // Initial request to floor 10
        let req1 = ElevatorRequest::new(ElevatorDirection::UP, 10);
        assert_eq!(elevator.hall_call(req1), Ok(true));

        // Start moving
        elevator.state_loop(5.1);
        assert_eq!(elevator.state, ElevatorState::MOVING(ElevatorDirection::UP));

        // Elevator reaches floor 3
        elevator.set_current_floor(3);

        // New request at floor 5 (en route)
        let req2 = ElevatorRequest::new(ElevatorDirection::UP, 5);
        assert_eq!(elevator.hall_call(req2), Ok(true));

        // Run state loop to update target
        elevator.state_loop(5.1);

        // Elevator should now target floor 5 first
        assert_eq!(
            elevator.get_target_floor(),
            5,
            "Elevator should update target to pickup request en route"
        );

        // Continue to floor 5
        simulate_movement(&mut elevator, 5);
        elevator.state_loop(5.1);

        // Then continue to floor 10
        assert_eq!(
            elevator.get_target_floor(),
            10,
            "Elevator should continue to original destination"
        );
    }

    #[test]
    fn test_current_floor_request() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(5);

        // Hall call at the current floor
        let req = ElevatorRequest::new(ElevatorDirection::UP, 5);
        assert_eq!(elevator.hall_call(req), Ok(true));

        // Run state loop
        elevator.state_loop(5.1);

        // Elevator should immediately enter waiting state without moving
        assert_eq!(
            elevator.state,
            ElevatorState::WAITING(ElevatorDirection::UP, ElevatorDoorsState::CLOSED),
            "Elevator should enter waiting state for request at current floor"
        );

        // After waiting, should return to IDLE
        elevator.state_loop(5.1);
        assert_eq!(elevator.state, ElevatorState::IDLE);
    }

    #[test]
    fn test_duplicate_request_handling() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(3);

        // Initial request
        let req = ElevatorRequest::new(ElevatorDirection::UP, 7);
        assert_eq!(elevator.hall_call(req), Ok(true));

        // Duplicate request
        assert_eq!(
            elevator.hall_call(req),
            Err(ElevatorRequestErr::DUPLICATE),
            "Elevator should reject duplicate request"
        );

        // Duplicate car call
        assert_eq!(
            elevator.car_call(7),
            Err(ElevatorRequestErr::DUPLICATE),
            "Elevator should reject duplicate car call"
        );
    }

    #[test]
    fn test_full_building_traverse() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(5);

        // Complex pattern of requests
        assert_eq!(
            elevator.hall_call(ElevatorRequest::new(ElevatorDirection::DOWN, 2)),
            Ok(true)
        );
        assert_eq!(
            elevator.hall_call(ElevatorRequest::new(ElevatorDirection::UP, 8)),
            Ok(true)
        );
        assert_eq!(
            elevator.hall_call(ElevatorRequest::new(ElevatorDirection::DOWN, 12)),
            Ok(true)
        );

        // First should go up to 8 (UP requests get priority in IDLE state)
        elevator.state_loop(5.1);
        assert_eq!(elevator.state, ElevatorState::MOVING(ElevatorDirection::UP));
        assert_eq!(elevator.get_target_floor(), 8);

        // Simulate moving to floor 8
        simulate_movement(&mut elevator, 8);

        println!("{:#?}", elevator.state);
        println!("{:#?}", elevator.current_floor);
        println!("{:#?}", elevator.target_floor);

        elevator.state_loop(5.1);

        // Now should check for requests in the UP direction beyond floor 8
        // Since there are none, it should transition to DOWN requests
        // The highest DOWN request is floor 12

        println!("{:#?}", elevator.state);
        println!("{:#?}", elevator.current_floor);
        println!("{:#?}", elevator.target_floor);

        assert_eq!(elevator.state, ElevatorState::MOVING(ElevatorDirection::UP));
        assert_eq!(elevator.get_target_floor(), 12);

        // Add a new request while moving
        assert_eq!(elevator.car_call(10), Ok(true));

        // Continue to floor 10
        simulate_movement(&mut elevator, 10);
        elevator.state_loop(5.1);

        // Next should be floor 12
        assert_eq!(elevator.get_target_floor(), 12);
        simulate_movement(&mut elevator, 12);
        elevator.state_loop(5.1);

        // Finally to floor 2
        assert_eq!(elevator.get_target_floor(), 2);
        simulate_movement(&mut elevator, 2);
        elevator.state_loop(5.1);

        // After all requests, should be IDLE
        assert_eq!(elevator.state, ElevatorState::IDLE);
    }

    #[test]
    fn test_car_call_to_current_floor() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(5);

        // Car call to current floor should be rejected
        assert_eq!(
            elevator.car_call(5),
            Err(ElevatorRequestErr::CurrentFloor),
            "Elevator should reject car call to current floor"
        );
    }

    #[test]
    fn test_waiting_with_new_requests() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(3);

        // Initial request
        assert_eq!(
            elevator.hall_call(ElevatorRequest::new(ElevatorDirection::UP, 3)),
            Ok(true)
        );

        // Start waiting
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::WAITING(ElevatorDirection::UP, ElevatorDoorsState::CLOSED)
        );

        // While waiting, get a new request
        assert_eq!(
            elevator.hall_call(ElevatorRequest::new(ElevatorDirection::UP, 7)),
            Ok(true)
        );

        // Partial wait time
        elevator.state_loop(3.0);

        // Should still be waiting
        assert!(matches!(elevator.state, ElevatorState::WAITING(_, _)));

        // Complete waiting time
        elevator.state_loop(2.1);

        // Should now move to the new request
        assert_eq!(elevator.state, ElevatorState::MOVING(ElevatorDirection::UP));
        assert_eq!(elevator.get_target_floor(), 7);
    }

    #[test]
    fn test_priority_for_same_direction_requests() {
        let mut elevator = Elevator::new();
        elevator.set_current_floor(5);

        // Requests in different directions
        assert_eq!(
            elevator.hall_call(ElevatorRequest::new(ElevatorDirection::UP, 8)),
            Ok(true)
        );
        assert_eq!(
            elevator.hall_call(ElevatorRequest::new(ElevatorDirection::DOWN, 7)),
            Ok(true)
        );

        // Should prioritize the UP direction first
        elevator.state_loop(5.1);
        assert_eq!(elevator.state, ElevatorState::MOVING(ElevatorDirection::UP));
        assert_eq!(elevator.get_target_floor(), 8);

        // After handling that, should pick up the DOWN request at 7
        simulate_movement(&mut elevator, 8);
        elevator.state_loop(5.1);
        assert_eq!(
            elevator.state,
            ElevatorState::MOVING(ElevatorDirection::DOWN)
        );
        assert_eq!(elevator.get_target_floor(), 7);
    }
}
