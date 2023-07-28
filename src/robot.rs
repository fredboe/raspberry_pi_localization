#[derive(Debug)]
pub enum Action {
    Idle,
    Drive(f32, f32)
}

pub fn perform_action(action: Action) {
    println!("Perform the action {:?}", action);
}


