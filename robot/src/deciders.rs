use crate::actions::Action;
use crate::user_input::UserInput;

/// # Explanation
/// The decider trait is used to decide the next action of the robot. It contains a function that
/// gets some data as input and returns an action.
pub trait Decider {
    fn decide(&mut self, user_input: &UserInput) -> Action;
}

/// # Explanation
/// The AlwaysIdle decider always returns the idle action (so the motors will not turn).
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

/// # Explanation
/// The FollowJoystick decider uses the joystick data to determine the motor speed. So it basically is
/// driving by joystick.
pub struct FollowJoystick;

impl FollowJoystick {
    pub fn new() -> FollowJoystick {
        FollowJoystick
    }
}

impl Decider for FollowJoystick {
    /// # Explanation
    /// This is a common algorithm to determine the motor speed from the joystick data.
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
