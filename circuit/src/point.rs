use geo::Point as GeoPoint;
use ordered_float::NotNan;
use std::ops::{Deref, DerefMut};

#[derive(Clone, Copy, Debug)]
pub struct Point(GeoPoint<NotNan<f64>>);

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Point(GeoPoint::new(
            NotNan::new(x).unwrap(),
            NotNan::new(y).unwrap(),
        ))
    }
}

impl PartialEq for Point {
    fn eq(&self, other: &Point) -> bool {
        self.x() == other.x() && self.y() == other.y()
    }
}

impl Eq for Point {}

impl Deref for Point {
    type Target = GeoPoint<NotNan<f64>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Point {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Rotation(NotNan<f64>);

impl Rotation {
    pub fn new(degrees: f64) -> Self {
        Rotation(NotNan::new(degrees).unwrap())
    }
}

impl PartialEq for Rotation {
    fn eq(&self, other: &Rotation) -> bool {
        self.0 == other.0
    }
}

impl Eq for Rotation {}

impl Deref for Rotation {
    type Target = NotNan<f64>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Rotation {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
