use std::error::Error;

pub trait MotorController<ERR: Error> {
    fn set_speed(&mut self, motor_id: u8, speed: f32) -> Result<(), ERR>;

    fn set_direction(&mut self, motor_id: u8, direction: Directions) -> Result<(), ERR>;

    fn run(&mut self, motor_id: u8, direction: Directions, speed: f32) -> Result<(), ERR> {
        self.set_direction(motor_id, direction)?;
        self.set_speed(motor_id, speed)
    }
}

#[derive(Debug)]
pub enum Directions {
    FORWARD,
    BACKWARD,
    BREAK
}

impl From<f32> for Directions {
    fn from(value: f32) -> Self {
        if value >= 0.0 {
            Directions::FORWARD
        } else {
            Directions::BACKWARD
        }
    }
}

#[derive(Debug)]
pub enum Action {
    Idle,
    Drive(f32, f32)
}

pub fn perform_action<ERR: Error, M: MotorController<ERR>>(action: Action, motor_controller: &mut M) -> Result<(), ERR> {
    println!("Perform the action {:?}", action);

    match action {
        Action::Idle => {
            motor_controller.run(0, Directions::BREAK, 0.0)?;
            motor_controller.run(1, Directions::BREAK, 0.0)
        },
        Action::Drive(motor_left, motor_right) => {
            let direction_left = Directions::from(motor_left);
            let speed_left = motor_left.abs();

            let direction_right = Directions::from(motor_right);
            let speed_right = motor_right.abs();

            motor_controller.run(0, direction_left, speed_left)?;
            motor_controller.run(1, direction_right, speed_right)
        }
    }
}


