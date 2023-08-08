use crate::robot::Action;
use crate::user_input::UserInput;

pub trait Decider {
    fn decide(&mut self, user_input: &UserInput) -> Action;
}

pub struct AlwaysIdle;

#[allow(dead_code)]
impl AlwaysIdle {
    pub fn new() -> AlwaysIdle {
        AlwaysIdle
    }
}

impl Decider for AlwaysIdle {
    fn decide(&mut self, _: &UserInput) -> Action {
        Action::Idle
    }
}

pub struct FollowJoystick;

impl FollowJoystick {
    pub fn new() -> FollowJoystick {
        FollowJoystick
    }
}

impl Decider for FollowJoystick {
    fn decide(&mut self, user_input: &UserInput) -> Action {
        let result = if let Some(joystick_data) = user_input.joystick {
            let (jx, jy) = joystick_data;
            let motor_left = (jx + jy).min(1.0).max(-1.0);
            let motor_right = (jy - jx).min(1.0).max(-1.0);

            Action::Drive(motor_left, motor_right)
        } else {
            Action::Idle
        };
        result
    }
}
