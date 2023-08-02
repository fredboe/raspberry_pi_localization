use gilrs::{Axis, Button, Gilrs};

/// The UserInput struct contains one Option for  the joystick data (x, y) and
/// one Option for a button that was pressed. It is the data holder for the user input.
#[derive(Debug, Default)]
pub struct UserInput {
    pub joystick: Option<(f32, f32)>,
    pub btn: Option<Button>,
}

impl UserInput {
    pub fn new(joystick: Option<(f32, f32)>, btn: Option<Button>) -> UserInput {
        UserInput { joystick, btn }
    }
}

/// # Explanation
/// The UserInputUnit basically is an iterator that when next is called returns the last joystick data
/// and the last button that was pressed.
pub struct UserInputUnit {
    gilrs: Gilrs,
}

impl UserInputUnit {
    pub fn new() -> Result<UserInputUnit, gilrs::Error> {
        let gilrs = Gilrs::new()?;
        Ok(UserInputUnit { gilrs })
    }
}

impl Iterator for UserInputUnit {
    type Item = UserInput;

    fn next(&mut self) -> Option<Self::Item> {
        // The event queue needs to be processed so that gilrs can cache the state of the gamepad
        while let Some(_) = self.gilrs.next_event() {}

        let gamepad = self.gilrs.gamepads().next().map(|(_, gamepad)| gamepad);

        let result = if let Some(gamepad) = gamepad {
            let joystick_data = (
                gamepad.value(Axis::LeftStickX),
                gamepad.value(Axis::LeftStickY),
            );
            let btn_pressed = [Button::South, Button::West, Button::East, Button::North]
                .into_iter()
                .filter(|&btn| gamepad.is_pressed(btn))
                .next();

            Some(UserInput::new(Some(joystick_data), btn_pressed))
        } else {
            None
        };
        result
    }
}
