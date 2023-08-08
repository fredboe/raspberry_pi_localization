use std::marker::PhantomData;

pub trait Sensor<T>: Iterator<Item = T> {
    fn attach<U, P>(self, preprocessor: P) -> PreprocessedSensor<T, U, Self, P>
    where
        P: Preprocessor<T, U>,
        Self: Sized,
    {
        PreprocessedSensor::new(self, preprocessor)
    }

    fn combine<T1, T2, S1, S2>(sensor1: S1, sensor2: S2) -> CombinedSensor<T1, T2, S1, S2>
    where
        S1: Sensor<T1>,
        S2: Sensor<T2>,
    {
        CombinedSensor::new(sensor1, sensor2)
    }
}

pub trait Preprocessor<X, Y> {
    fn run(&mut self, x: X) -> Y;
}

pub struct PreprocessedSensor<T, U, S, P>
where
    S: Sensor<T>,
    P: Preprocessor<T, U>,
{
    sensor: S,
    preprocessor: P,
    _phantom: PhantomData<(T, U)>,
}

impl<T, U, S, P> PreprocessedSensor<T, U, S, P>
where
    S: Sensor<T>,
    P: Preprocessor<T, U>,
{
    pub fn new(sensor: S, preprocessor: P) -> Self {
        PreprocessedSensor {
            sensor,
            preprocessor,
            _phantom: PhantomData::default(),
        }
    }
}

impl<T, U, S, P> Iterator for PreprocessedSensor<T, U, S, P>
where
    P: Preprocessor<T, U>,
    S: Sensor<T>,
{
    type Item = U;

    fn next(&mut self) -> Option<Self::Item> {
        self.sensor.next().map(|data| self.preprocessor.run(data))
    }
}

impl<T, U, S, P> Sensor<U> for PreprocessedSensor<T, U, S, P>
where
    S: Sensor<T>,
    P: Preprocessor<T, U>,
{
}

pub struct CombinedSensor<T, U, S1: Sensor<T>, S2: Sensor<U>> {
    sensor1: S1,
    sensor2: S2,
    _phantom: PhantomData<(T, U)>,
}

impl<T, U, S1: Sensor<T>, S2: Sensor<U>> CombinedSensor<T, U, S1, S2> {
    pub fn new(sensor1: S1, sensor2: S2) -> Self {
        CombinedSensor {
            sensor1,
            sensor2,
            _phantom: PhantomData::default(),
        }
    }
}

impl<T, U, S1: Sensor<T>, S2: Sensor<U>> Iterator for CombinedSensor<T, U, S1, S2> {
    type Item = (T, U);

    fn next(&mut self) -> Option<Self::Item> {
        self.sensor1
            .next()
            .and_then(|first| self.sensor2.next().map(|scd| (first, scd)))
    }
}

impl<T, U, S1: Sensor<T>, S2: Sensor<U>> Sensor<(T, U)> for CombinedSensor<T, U, S1, S2> {}
