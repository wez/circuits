extern crate ncollide;
extern crate nalgebra as na;
use ncollide::transformation::ToPolyline;
use std::sync::Arc;
use geo;
use geo::convexhull::ConvexHull;

pub type Point = na::Point2<f64>;
pub type Location = na::Isometry2<f64>;
pub type Similarity = na::Similarity2<f64>;
pub type Polyline = ncollide::shape::Polyline2<f64>;
pub type ShapeHandle = ncollide::shape::ShapeHandle2<f64>;
pub type Vector = na::Vector2<f64>;
pub type Circle = ncollide::shape::Ball2<f64>;
pub type Capsule = ncollide::shape::Capsule<f64>;

pub fn origin() -> Location {
    Location::new(Vector::new(0.0, 0.0), na::zero())
}

/// Returns the rectilinear or manhattan distance between
/// two points.
pub fn manhattan_distance(a: &Point, b: &Point) -> f64 {
    (a.coords.x - b.coords.x).abs() + (a.coords.y - b.coords.y).abs()
}

pub struct Shape {
    pub handle: ShapeHandle,
    pub location: Location,
    pub width: Option<f64>,
}

// Convert geom::Point to geo::Point
fn geo_point(p: &Point) -> geo::Point<f64> {
    geo::Point::new(p.coords.x, p.coords.y)
}

impl Shape {
    // Make a polygon from the provided points.
    pub fn polygon(points: Vec<Point>, location: Location, width: Option<f64>) -> Shape {
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
            width: width,
        }
    }

    pub fn circle(radius: f64, location: Location) -> Shape {
        Shape {
            handle: ShapeHandle::new(Circle::new(radius)),
            location: location,
            width: None,
        }
    }

    pub fn line(a: &Point, b: &Point) -> Shape {
        Shape::polygon(vec![a.clone(), b.clone()], origin(), None)
    }

    pub fn capsule(radius: f64, a: &Point, b: &Point, location: Location) -> Shape {
        let half_height = na::distance(a, b) / 2.;

        let y = b.coords.y - a.coords.y;
        let x = b.coords.x - a.coords.x;
        use std::f64::consts::PI;
        let angle = y.atan2(x) + PI / 2.;

        let (mut points, _) = Capsule::new(half_height, radius)
            .to_polyline(16)
            .unwrap();

        let pt = points[0].clone();
        points.push(pt);

        Shape::polygon(points,
                       location *
                       Location::new(Vector::new(a.coords.x, a.coords.y + half_height), angle),
                       None)
    }

    // returns the axis-aligned bounding-box
    pub fn aabb(&self) -> ncollide::bounding_volume::AABB<Point> {
        ncollide::bounding_volume::aabb(&*self.handle, &self.location)
    }

    // returns the bounding sphere
    pub fn bounding_sphere(&self) -> ncollide::bounding_volume::BoundingSphere<Point> {
        ncollide::bounding_volume::bounding_sphere(&*self.handle, &self.location)
    }

    pub fn translate(&self, location: &Location) -> Shape {
        return Shape {
                   handle: self.handle.clone(),
                   location: location * self.location,
                   width: self.width,
               };
    }

    pub fn transform(&self, xlate: &Similarity) -> Shape {
        Shape::polygon(self.compute_points()
                           .iter()
                           .map(|pt| xlate * pt)
                           .collect(),
                       origin(),
                       if let Some(w) = self.width {
                           Some(w * xlate.scaling())
                       } else {
                           None
                       })
    }

    pub fn is_verticalish_line(&self) -> bool {
        if let Some(poly) = self.handle.as_shape::<Polyline>() {

            let vertices = &poly.vertices();
            let a = &vertices[0];
            let b = &vertices[1];

            let x = (a.coords.x - b.coords.x).abs();
            let y = (a.coords.y - b.coords.y).abs();

            y > x * 1.5
        } else {
            false
        }
    }

    pub fn compute_points(&self) -> Vec<Point> {
        if let Some(poly) = self.handle.as_shape::<Polyline>() {
            let mut points = Vec::new();

            for p in poly.vertices().iter() {
                points.push(self.location * p);
            }

            return points;
        }

        if let Some(circle) = self.handle.as_shape::<Circle>() {
            let mut poly = circle.to_polyline(16);
            poly.transform_by(&self.location);
            let (mut points, _) = poly.unwrap();
            // close the shape!
            let pt = points[0].clone();
            points.push(pt);
            return points;
        }
        panic!("unsupported shape type");
    }

    // Explicitly compute the convex hull.  The ncollide hull routines use
    // the implicit hull for collision detection purposes, so we use the geo
    // library to generate an explicit polygon for the hull.
    pub fn convex_hull(shapes: &Vec<Shape>) -> Shape {
        let polys: geo::MultiPolygon<_> = shapes
            .iter()
            .map(|shape| {
                     let ls: geo::LineString<_> =
                         shape.compute_points().iter().map(geo_point).collect();
                     geo::Polygon::new(ls, Vec::new())
                 })
            .collect();
        let hull = polys.convex_hull();
        let points = hull.exterior
            .into_iter()
            .map(|p| Point::new(p.x(), p.y()))
            .collect();
        Shape::polygon(points, origin(), None)
    }

    pub fn contact(&self, other: &Shape) -> Option<ncollide::query::Contact<Point>> {
        ncollide::query::contact(&self.location,
                                 &*self.handle,
                                 &other.location,
                                 &*other.handle,
                                 1.0)
    }

    pub fn intersects(&self, other: &Shape) -> bool {
        ncollide::query::proximity(&self.location,
                                   &*self.handle,
                                   &other.location,
                                   &*other.handle,
                                   0.0) == ncollide::query::Proximity::Intersecting
    }
}

impl Clone for Shape {
    fn clone(&self) -> Shape {
        self.translate(&origin())
    }
}

impl ::std::fmt::Debug for Shape {
    fn fmt(&self, fmt: &mut ::std::fmt::Formatter) -> ::std::result::Result<(), ::std::fmt::Error> {
        if let Some(poly) = self.handle.as_shape::<Polyline>() {
            write!(fmt,
                   "Polyline with {} vertices at {}",
                   poly.vertices().len(),
                   self.location)?;
        } else if let Some(circle) = self.handle.as_shape::<Circle>() {
            write!(fmt,
                   "Circle radius {} at {}",
                   circle.radius(),
                   self.location)?;
        } else {
            write!(fmt, "Unhandled shape type at {}", self.location)?;
        }

        Ok(())
    }
}
