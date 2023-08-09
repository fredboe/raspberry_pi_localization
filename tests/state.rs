#[cfg(test)]
mod tests {
    use raspberry_pi_localization::sensor::gps::Cartesian2D;
    use raspberry_pi_localization::state::{max_point_track, min_point_track, Cartesian2DTrack};

    #[test]
    fn test_min_max_point_on_track() {
        let mut track = Cartesian2DTrack::new();
        track.push(Cartesian2D(0.0, 0.0));
        track.push(Cartesian2D(-1.0, 0.0));
        track.push(Cartesian2D(0.0, 1.0));
        track.push(Cartesian2D(4.0, 5.0));
        track.push(Cartesian2D(0.0, -3.0));

        let min_point = min_point_track(&mut track).unwrap();
        assert_eq!(min_point.0, -1.0);
        assert_eq!(min_point.1, -3.0);

        let max_point = max_point_track(&mut track).unwrap();
        assert_eq!(max_point.0, 4.0);
        assert_eq!(max_point.1, 5.0);
    }
}
