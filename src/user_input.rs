use gilrs::{Button, Gilrs, Axis};

const GILRS_ERROR: &str = "There was an internal gilrs error. Without it the program cannot handle user input...";

#[derive(Debug, Default)]
pub struct UserInput {
    pub joystick: Option<(f32, f32)>,
    pub btn: Option<Button>
}

impl UserInput {
    pub fn new(joystick: Option<(f32, f32)>, btn: Option<Button>) -> UserInput {
        UserInput { joystick, btn }
    }
}


pub struct UserInputUnit {
    gilrs: Gilrs
}

impl UserInputUnit {
    pub fn new() -> UserInputUnit {
        let gilrs = Gilrs::new().expect(GILRS_ERROR);
        UserInputUnit { gilrs }
    }
}

impl Iterator for UserInputUnit {
    type Item = UserInput;

    fn next(&mut self) -> Option<Self::Item> {
        // The event queue needs to be processed so that gilrs can cache the state of the gamepad
        while let Some(_) = self.gilrs.next_event() {}

        let gamepad = self.gilrs.gamepads().next().map(|(_, gamepad)| gamepad);

        let result = if let Some(gamepad) = gamepad {
            let joystick_data = (gamepad.value(Axis::LeftStickX), gamepad.value(Axis::LeftStickY));
            let btn_pressed = [Button::South, Button::West, Button::East, Button::North].into_iter()
                .filter(|&btn| gamepad.is_pressed(btn)).next();

            Some(UserInput::new(Some(joystick_data), btn_pressed))
        } else {
            None
        };
        result
    }
}