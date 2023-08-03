#[cfg(test)]
mod tests {
    use raspberry_pi_localization::robot::{perform_action, Action, Directions, MotorController};
    use std::fmt::Error;

    struct MockMotorController {
        motors: [(Directions, f32); 4],
    }

    impl MotorController<Error> for MockMotorController {
        fn set_speed(&mut self, motor_id: u8, speed: f32) -> Result<(), Error> {
            let motor_id = motor_id as usize;
            let (old_direction, _) = self.motors[motor_id];
            self.motors[motor_id] = (old_direction, speed);
            Ok(())
        }

        fn set_direction(&mut self, motor_id: u8, direction: Directions) -> Result<(), Error> {
            let motor_id = motor_id as usize;
            let (_, old_speed) = self.motors[motor_id];
            self.motors[motor_id] = (direction, old_speed);
            Ok(())
        }
    }

    #[test]
    fn test_perform_action() {
        let mut motor_controller = MockMotorController {
            motors: [
                (Directions::FORWARD, 0.0),
                (Directions::FORWARD, 0.0),
                (Directions::FORWARD, 0.0),
                (Directions::FORWARD, 0.0),
            ],
        };

        perform_action(Action::Drive(1.0, 1.0), &mut motor_controller).unwrap();
        assert_eq!(motor_controller.motors[0], (Directions::FORWARD, 1.0));
        assert_eq!(motor_controller.motors[2], (Directions::FORWARD, 1.0));

        perform_action(Action::Idle, &mut motor_controller).unwrap();
        assert_eq!(motor_controller.motors[0], (Directions::BREAK, 0.0));
        assert_eq!(motor_controller.motors[2], (Directions::BREAK, 0.0));

        perform_action(Action::Drive(20.0, 20.0), &mut motor_controller).unwrap();
        assert_eq!(motor_controller.motors[0], (Directions::FORWARD, 1.0));
        assert_eq!(motor_controller.motors[2], (Directions::FORWARD, 1.0));

        perform_action(Action::Drive(0.5, -0.25), &mut motor_controller).unwrap();
        assert_eq!(motor_controller.motors[0], (Directions::FORWARD, 0.5));
        assert_eq!(motor_controller.motors[2], (Directions::BACKWARD, 0.25));
    }

    #[test]
    fn failure_test() {
        assert_eq!(0, 1);
    }
}
