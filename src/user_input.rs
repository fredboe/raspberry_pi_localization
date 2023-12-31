use gilrs::{Axis, Button, Gilrs};

/// The UserInput struct contains one option for the joystick data (jx, jy) and
/// one option for a button that was pressed. It is the data holder for the user input.
#[derive(Debug, Default)]
pub struct UserInput {
    pub joystick: Option<(f32, f32)>,
    pub btn: Option<Button>,
}

impl UserInput {
    pub fn new(joystick: Option<(f32, f32)>, btn: Option<Button>) -> UserInput {
        UserInput { joystick, btn }
    }

    pub fn is_pressed(&self, btn: Button) -> bool {
        self.btn.map(|user_btn| user_btn == btn).unwrap_or(false)
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
