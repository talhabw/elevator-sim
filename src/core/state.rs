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
    NotTarget,
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

#[derive(PartialEq, Eq, Hash, Debug)]
pub enum ElevatorDoorsState {
    OPEN,
    CLOSED,
}

#[derive(PartialEq, Eq, Hash, Debug)]
pub enum ElevatorState {
    MOVING(ElevatorDirection),
    WAITING(ElevatorDirection, ElevatorDoorsState),
    IDLE,
}

impl ElevatorState {
    fn as_waiting(&self) -> Option<ElevatorState> {
        match self {
            ElevatorState::MOVING(direction) => Some(ElevatorState::WAITING(
                *direction,
                ElevatorDoorsState::CLOSED,
            )),
            _ => None,
        }
    }
}

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
pub struct ElevatorRequest {
    direction: ElevatorDirection,
    floor: i8,
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

    fn _get_target_on_the_way(
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
                        && (!is_at_target && request.floor <= self.target_floor)
                }
                Ordering::Greater => {
                    direction == ElevatorDirection::DOWN
                        && (!is_at_target && request.floor >= self.target_floor)
                }
            })
            .min_by_key(|request| (self.current_floor - request.floor).abs())
            .copied()
    }

    fn _get_first_target_in_direction(
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

    fn _get_best_target_with_opposite_direction(
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

    fn _get_next_request_on_idle(&self) -> Option<ElevatorRequest> {
        self._get_first_target_in_direction(ElevatorDirection::UP)
            .or_else(|| self._get_first_target_in_direction(ElevatorDirection::DOWN))
    }

    fn _get_next_request_while_moving(
        &self,
        direction: ElevatorDirection,
    ) -> Option<ElevatorRequest> {
        self._get_target_on_the_way(direction, false)
            .or_else(|| self._get_best_target_with_opposite_direction(direction))
    }

    fn _get_next_request_after_waiting(
        &self,
        direction: ElevatorDirection,
    ) -> Option<ElevatorRequest> {
        self._get_target_on_the_way(direction, true)
            .or_else(|| self._get_first_target_in_direction(direction.opposite()))
            .or_else(|| self._get_first_target_in_direction(direction))
    }

    fn _remove_finished_request(&mut self, direction: ElevatorDirection) {
        let _ = self
            .request_buffer
            .remove(&ElevatorRequest::new(direction, self.current_floor))
            || self.request_buffer.remove(&ElevatorRequest::new(
                direction.opposite(),
                self.current_floor,
            ));
    }

    pub fn _state_loop(&mut self, dt_secs: f64) {
        match &self.state {
            ElevatorState::IDLE => {
                if let Some(request) = self._get_next_request_on_idle() {
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
                let direction = direction.clone();
                self._remove_finished_request(direction);

                // todo better timer at one point, not a priority
                self.waiting_time += dt_secs;

                // After waiting period completes
                if self.waiting_time >= 5.0 {
                    self.waiting_time = 0.0;

                    if let Some(request) = self._get_next_request_after_waiting(direction) {
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

                if let Some(request) = self._get_next_request_while_moving(*direction) {
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

    pub fn notify_reached_target(&mut self, reached_floor: i8) -> Option<ElevatorFloorReachErr> {
        self.current_floor = reached_floor;

        if self.current_floor != self.target_floor {
            return Some(ElevatorFloorReachErr::NotTarget);
        }

        match self.state.as_waiting() {
            Some(new_state) => self.state = new_state,
            None => return Some(ElevatorFloorReachErr::NotMoving),
        }

        None
    }
}
