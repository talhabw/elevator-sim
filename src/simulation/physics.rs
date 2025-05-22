use rapier2d::prelude::*;

pub struct ElevatorPhysics {
    // Rapier specific components
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity_vector: Vector<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    elevator_body_handle: RigidBodyHandle,

    // Config
    motor_constant: f32, // N/V (force per volt)

    // State variables
    voltage: f32, // V Current voltage to be applied
}

impl ElevatorPhysics {
    pub fn new(
        mass: f32,
        translation_x: f32,
        translation_y: f32,
        initial_y_position: f32,
        gravity_y: f32,
        motor_constant: f32,
    ) -> Self {
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        // ground
        let ground_collider = ColliderBuilder::cuboid(100.0, 0.1).build();
        collider_set.insert(ground_collider);

        let gravity_vector = vector![0.0, gravity_y];
        // todo: set the time step
        let integration_parameters = IntegrationParameters::default();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = DefaultBroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let impulse_joint_set = ImpulseJointSet::new();
        let multibody_joint_set = MultibodyJointSet::new();
        let ccd_solver = CCDSolver::new();
        let query_pipeline = QueryPipeline::new();

        // Create the elevator rigid body
        let elevator_rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, initial_y_position])
            .lock_rotations()
            .build();
        let elevator_collider = ColliderBuilder::cuboid(translation_x, translation_y)
            .mass(mass)
            .build();
        let elevator_body_handle = rigid_body_set.insert(elevator_rigid_body);
        collider_set.insert_with_parent(
            elevator_collider,
            elevator_body_handle,
            &mut rigid_body_set,
        );

        Self {
            rigid_body_set,
            collider_set,
            gravity_vector,
            integration_parameters,
            physics_pipeline,
            island_manager,
            broad_phase,
            narrow_phase,
            impulse_joint_set,
            multibody_joint_set,
            ccd_solver,
            query_pipeline,
            elevator_body_handle,
            motor_constant,
            voltage: 0.0,
        }
    }

    pub fn update(&mut self) {
        let motor_force_y = self.voltage * self.motor_constant;

        let elevator_body = self.get_mut_elevator_body();
        elevator_body.reset_forces(true);
        elevator_body.add_force(vector![0.0, motor_force_y], true);

        self.physics_pipeline.step(
            &self.gravity_vector,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &(),
            &(),
        );
    }

    pub fn set_voltage(&mut self, voltage: f32) {
        self.voltage = voltage;
    }

    pub fn get_position(&self) -> f32 {
        self.get_elevator_body().translation().y - 3.0
    }

    pub fn get_velocity(&self) -> f32 {
        self.get_elevator_body().linvel().y
    }

    fn get_elevator_body(&self) -> &RigidBody {
        self.rigid_body_set
            .get(self.elevator_body_handle)
            .expect("elevator body not found")
    }

    fn get_mut_elevator_body(&mut self) -> &mut RigidBody {
        self.rigid_body_set
            .get_mut(self.elevator_body_handle)
            .expect("elevator body not found")
    }
}
