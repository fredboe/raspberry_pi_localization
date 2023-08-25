use std::error::Error;

/// # Explanation
/// The MotorController trait is a trait that can be used to implement a struct that (like the name says)
/// controls a robot.
pub trait MotorController<ERR: Error> {
    fn set_speed(&mut self, motor_id: u8, speed: f32) -> Result<(), ERR>;

    fn set_direction(&mut self, motor_id: u8, direction: Directions) -> Result<(), ERR>;

    fn run(&mut self, motor_id: u8, direction: Directions, speed: f32) -> Result<(), ERR> {
        self.set_direction(motor_id, direction)?;
        self.set_speed(motor_id, speed)
    }
}

/// # Explanation
/// The directions enum consists of all the modes a motor can have (FORWARD, BACKWARD and BREAK).
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Directions {
    FORWARD,
    BACKWARD,
    BREAK,
}

impl From<f32> for Directions {
    /// # Explanation
    /// If value >= 0 then FORWARD else BACKWARD
    fn from(value: f32) -> Self {
        if value >= 0.0 {
            Directions::FORWARD
        } else {
            Directions::BACKWARD
        }
    }
}

/// # Explanation
/// The action enum contains all possible actions the robot can perform. Currently it is Idle (not moving)
/// and Drive(speed of the left motor, speed of the right motor).
#[derive(Debug)]
pub enum Action {
    Idle,
    Drive(f32, f32),
}

/// # Explanation
/// perform_action forwards the action to the MotorController. It requires that two motors are connected to the motor controller.
/// These motors must have the ids 0 and 2.
pub fn perform_action<ERR: Error, M: MotorController<ERR>>(
    action: Action,
    motor_controller: &mut M,
) -> Result<(), ERR> {
    log::info!("Robot: Perform the action {:?}", action);

    match action {
        Action::Idle => {
            motor_controller.run(0, Directions::BREAK, 0.0)?;
            motor_controller.run(2, Directions::BREAK, 0.0)
        }
        Action::Drive(motor_left, motor_right) => {
            let direction_left = Directions::from(motor_left);
            let speed_left = motor_left.abs().min(1.0);

            let direction_right = Directions::from(motor_right);
            let speed_right = motor_right.abs().min(1.0);

            motor_controller.run(0, direction_left, speed_left)?;
            motor_controller.run(2, direction_right, speed_right)
        }
    }
}
