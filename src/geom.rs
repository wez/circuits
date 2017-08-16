extern crate ncollide;
extern crate nalgebra as na;
use std::sync::Arc;

pub type Point = na::Point2<f64>;
pub type Location = na::Isometry2<f64>;
pub type Similarity = na::Similarity2<f64>;
pub type Polyline = ncollide::shape::Polyline2<f64>;
pub type ShapeHandle = ncollide::shape::ShapeHandle2<f64>;
pub type Vector = na::Vector2<f64>;

pub fn origin() -> Location {
    Location::new(Vector::new(0.0, 0.0), na::zero())
}

pub struct Shape {
    pub handle: ShapeHandle,
    pub location: Location,
}

impl Shape {
    // Make a polygon from the provided points.
    pub fn polygon(points: Vec<Point>, location: Location) -> Shape {
        let mut indices = Vec::new();
        for i in 0..points.len() - 1 {
            indices.push(na::Point2::new(i, i + 1));
        }
        indices.push(na::Point2::new(points.len() - 1, 0));

        Shape {
            handle: ShapeHandle::new(Polyline::new(Arc::new(points),
                                                   Arc::new(indices),
                                                   None,
                                                   None)),
            location: location,
        }

    }

    // returns the axis-aligned bounding-box
    pub fn aabb(&self) -> ncollide::bounding_volume::AABB<Point> {
        ncollide::bounding_volume::aabb(&*self.handle, &self.location)
    }
}

impl ::std::fmt::Debug for Shape {
    fn fmt(&self, fmt: &mut ::std::fmt::Formatter) -> ::std::result::Result<(), ::std::fmt::Error> {
        if let Some(poly) = self.handle.as_shape::<Polyline>() {
            write!(fmt,
                   "Polyline with {} vertices at {}",
                   poly.vertices().len(),
                   self.location)?;
        } else {
            write!(fmt, "Unhandled shape type at {}", self.location)?;
        }

        Ok(())
    }
}
