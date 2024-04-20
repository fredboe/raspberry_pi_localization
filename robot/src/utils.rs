use std::sync::mpsc;
use std::sync::mpsc::{Receiver, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// # Explanation
/// The game loop is an iterator that waits when the next function is called if the execution is faster
/// than the frame rate allows.
pub struct GameLoop {
    frame: u64,
    current_frame_start: Instant,
    duration_per_frame: Duration,
}

impl GameLoop {
    pub fn new(duration_per_frame: Duration) -> GameLoop {
        let current_frame_start = Instant::now();
        GameLoop {
            frame: 0,
            current_frame_start,
            duration_per_frame,
        }
    }

    pub fn from_fps(fps: usize) -> GameLoop {
        let duration_per_frame = Duration::from_secs_f32(1.0 / (fps as f32));
        Self::new(duration_per_frame)
    }
}

impl Iterator for GameLoop {
    type Item = u64;

    fn next(&mut self) -> Option<Self::Item> {
        let end_time = self.current_frame_start + self.duration_per_frame;
        let now = Instant::now();
        if now <= end_time {
            let sleep_time = end_time - Instant::now();
            std::thread::sleep(sleep_time);
        } else {
            log::warn!("The game loop is hanging behind by {:?}.", now - end_time);
        }

        let frame_number = self.frame;
        self.frame += 1;

        let next_frame_start_time = Instant::now();
        self.current_frame_start = next_frame_start_time;

        Some(frame_number)
    }
}

struct Stop;

/// # Explanation
/// The ParSampler struct can be used if an iterator should be called with a specific frame rate on a different thread.
/// I.e. for a sensor_utils struct that should sample the device with a specific frame rate.
///
/// The ParSampler keeps the values as a state so that always the last value the iterator returned can be accessed.
pub struct ParSampler<T> {
    stop_sender: Sender<Stop>,
    state_receiver: Receiver<Option<T>>,
    handle: Option<JoinHandle<()>>,
}

impl<T: Send + 'static> ParSampler<T> {
    pub fn new<IT: Iterator<Item = T> + Send + 'static>(
        sample_rate: usize,
        mut iterator: IT,
    ) -> Self {
        let (stop_sender, stop_receiver) = mpsc::channel();
        let (state_sender, state_receiver) = mpsc::channel();

        let handle = std::thread::spawn(move || {
            for _ in GameLoop::from_fps(sample_rate) {
                if stop_receiver.try_recv().is_ok() {
                    break;
                }

                let next_state = iterator.next();
                state_sender.send(next_state).unwrap_or(());
            }
        });

        ParSampler {
            stop_sender,
            state_receiver,
            handle: Some(handle),
        }
    }
}

impl<T: Clone> Iterator for ParSampler<T> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        let mut current_state = None;
        while let Ok(state) = self.state_receiver.try_recv() {
            if state.is_some() {
                current_state = state;
            }
        }

        current_state
    }
}

impl<T> Drop for ParSampler<T> {
    fn drop(&mut self) {
        const ERR_MSG: &str = "ParSampler: Could not terminate the worker thread.";
        self.stop_sender.send(Stop).expect(ERR_MSG);

        if let Some(handle) = self.handle.take() {
            handle.join().expect(ERR_MSG);
        }
    }
}
