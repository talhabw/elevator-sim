pub struct ElevatorPhysics {
    // Physical parameters
    mass: f64,           // kg
    gravity: f64,        // m/s²
    motor_constant: f64, // N/V (force per volt)
    // friction_coefficient: f64, // N/(m/s)

    // State variables
    position: f64,     // m
    velocity: f64,     // m/s
    acceleration: f64, // m/s²

    // Input
    voltage: f64, // V
}

impl ElevatorPhysics {
    pub fn new(mass: f64, position: f64, voltage: f64) -> Self {
        Self {
            mass,
            gravity: 9.81,
            motor_constant: 10.0, // Adjust for realistic behavior
            // friction_coefficient: 50.0,
            position,
            velocity: 0.0,
            acceleration: 0.0,
            voltage,
        }
    }

    pub fn update(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }

        let voltage = self.voltage;
        let motor_force = voltage * self.motor_constant;
        let gravity_force = self.mass * self.gravity;

        let net_force = motor_force - gravity_force * 0.0;

        self.acceleration = net_force / self.mass;
        self.velocity += self.acceleration * dt;

        self.position += self.velocity * dt;
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        self.voltage = voltage;
    }

    pub fn get_position(&self) -> f64 {
        self.position
    }

    pub fn get_velocity(&self) -> f64 {
        self.velocity
    }
}
