use plotly::{Plot, Scatter};
use plotly::common::Mode;

use crate::state::Waypoint;

#[derive(Clone)]
pub struct Track<const D: usize> {
    waypoints: Vec<Waypoint<D>>,
}

impl<const D: usize> Track<D> {
    pub fn new(initial_waypoint: Waypoint<D>) -> Self {
        Self {
            waypoints: vec![initial_waypoint],
        }
    }

    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    pub fn add_waypoint(&mut self, waypoint: Waypoint<D>) {
        self.waypoints.push(waypoint);
    }

    pub fn get_latest_waypoint(&self) -> &Waypoint<D> {
        self.waypoints.last().unwrap() // waypoints cannot be empty
    }

    pub fn create_scatter(
        &self,
        name: &str,
        to_2d: impl Fn(&Waypoint<D>) -> (f64, f64),
    ) -> Box<Scatter<f64, f64>> {
        let (xs, ys): (Vec<f64>, Vec<f64>) = self
            .waypoints
            .iter()
            .map(|waypoint| to_2d(waypoint))
            .unzip();

        Scatter::new(xs, ys).mode(Mode::Lines).name(name)
    }

    pub fn plot(&self, name: &str, to_2d: impl Fn(&Waypoint<D>) -> (f64, f64)) {
        let mut plot = Plot::new();
        plot.add_trace(self.create_scatter(name, to_2d));
        plot.write_html(format!("{}.html", name));
    }
}

impl<const D: usize> IntoIterator for Track<D> {
    type Item = Waypoint<D>;
    type IntoIter = std::vec::IntoIter<Self::Item>;

    fn into_iter(self) -> Self::IntoIter {
        self.waypoints.into_iter()
    }
}
