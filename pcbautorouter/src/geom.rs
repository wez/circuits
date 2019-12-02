use crate::polyoffset::{buffer, JoinType};
use geo;
use geo::convexhull::ConvexHull;
use geo::simplify::Simplify;
use nalgebra as na;
use ncollide2d::bounding_volume::{aabb, bounding_sphere, BoundingSphere, AABB};
use ncollide2d::query::Proximity::Intersecting;
use ncollide2d::query::{contact, proximity, Contact, Proximity};
use ncollide2d::transformation::ToPolyline;
use ordered_float::OrderedFloat;
use petgraph::graphmap::UnGraphMap;
use spade::HasPosition;
use std::cmp::Ordering;
use std::fmt::{Debug, Error, Formatter};
use std::result::Result;

pub type Point = na::Point2<f64>;
pub type Location = na::Isometry2<f64>;
pub type Similarity = na::Similarity2<f64>;
pub type Polyline = ncollide2d::shape::Polyline<f64>;
pub type ShapeHandle = ncollide2d::shape::ShapeHandle<f64>;
pub type Vector = na::Vector2<f64>;
pub type Circle = ncollide2d::shape::Ball<f64>;
pub type Capsule = ncollide2d::shape::Capsule<f64>;

// Number of points to use when expanding polylines.
// More points means higher quality curves, but is more expensive
// when computing things later on.
const ARC_POINTS: u32 = 9;

pub fn origin() -> Location {
    Location::new(Vector::new(0.0, 0.0), na::zero())
}

/// Returns the rectilinear or manhattan distance between
/// two points.
pub fn manhattan_distance(a: &Point, b: &Point) -> f64 {
    (a.coords.x - b.coords.x).abs() + (a.coords.y - b.coords.y).abs()
}

#[derive(Debug, Eq, PartialEq, Hash, Clone, Copy, Ord, PartialOrd)]
pub struct OrderedPoint {
    pub x: OrderedFloat<f64>,
    pub y: OrderedFloat<f64>,
}

impl OrderedPoint {
    pub fn new(x: f64, y: f64) -> OrderedPoint {
        OrderedPoint {
            x: OrderedFloat(x),
            y: OrderedFloat(y),
        }
    }

    pub fn from_point(p: &Point) -> OrderedPoint {
        OrderedPoint {
            x: OrderedFloat(p.coords.x),
            y: OrderedFloat(p.coords.y),
        }
    }

    pub fn point(&self) -> Point {
        Point::new(self.x.into(), self.y.into())
    }
}

impl From<Point> for OrderedPoint {
    fn from(p: Point) -> Self {
        OrderedPoint {
            x: OrderedFloat(p.coords.x),
            y: OrderedFloat(p.coords.y),
        }
    }
}

impl From<OrderedPoint> for Point {
    fn from(p: OrderedPoint) -> Self {
        Point::new(p.x.into(), p.y.into())
    }
}

impl From<OrderedPoint> for geo::Point<f64> {
    fn from(p: OrderedPoint) -> Self {
        geo::Point::new(p.x.into(), p.y.into())
    }
}

impl HasPosition for OrderedPoint {
    type Point = [f64; 2];
    fn position(&self) -> [f64; 2] {
        [self.x.into(), self.y.into()]
    }
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

fn order_by_dist(a: &Point, b: &Point, relative: &Point) -> Ordering {
    let dist_a = OrderedFloat(na::distance(a, relative));
    let dist_b = OrderedFloat(na::distance(b, relative));

    dist_a.cmp(&dist_b)
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
            handle: ShapeHandle::new(Polyline::new(points, Some(indices))),
            location,
            width,
        }
    }

    pub fn circle(radius: f64, location: Location) -> Shape {
        Shape {
            handle: ShapeHandle::new(Circle::new(radius)),
            location,
            width: None,
        }
    }

    pub fn circle_from_point(p: &Point, radius: f64) -> Shape {
        Shape {
            handle: ShapeHandle::new(Circle::new(radius)),
            location: Location::new(Vector::new(p.coords.x, p.coords.y), na::zero()),
            width: None,
        }
    }

    pub fn line(a: &Point, b: &Point) -> Shape {
        Shape::polygon(vec![*a, *b], origin(), None)
    }

    pub fn capsule(radius: f64, a: &Point, b: &Point, location: Location) -> Shape {
        let half_height = na::distance(a, b) / 2.;

        let y = b.coords.y - a.coords.y;
        let x = b.coords.x - a.coords.x;
        use std::f64::consts::PI;
        let angle = y.atan2(x) + PI / 2.;

        let (mut points, _) = Capsule::new(half_height, radius)
            .to_polyline(ARC_POINTS)
            .unwrap();

        let pt = points[0];
        points.push(pt);

        Shape::polygon(
            points,
            location * Location::new(Vector::new(a.coords.x, a.coords.y + half_height), angle),
            None,
        )
    }

    // returns the axis-aligned bounding-box
    pub fn aabb(&self) -> AABB<f64> {
        aabb(&*self.handle, &self.location)
    }

    // returns the bounding sphere
    pub fn bounding_sphere(&self) -> BoundingSphere<f64> {
        bounding_sphere(&*self.handle, &self.location)
    }

    pub fn translate(&self, location: &Location) -> Shape {
        Shape {
            handle: self.handle.clone(),
            location: location * self.location,
            width: self.width,
        }
    }

    pub fn translate_by_point(&self, pt: &Point) -> Shape {
        self.translate(&Location::new(
            Vector::new(pt.coords.x, pt.coords.y),
            na::zero(),
        ))
    }

    pub fn transform(&self, xlate: &Similarity) -> Shape {
        Shape::polygon(
            self.compute_points().iter().map(|pt| xlate * pt).collect(),
            origin(),
            if let Some(w) = self.width {
                Some(w * xlate.scaling())
            } else {
                None
            },
        )
    }

    pub fn is_verticalish_line(&self) -> bool {
        if let Some(poly) = self.handle.as_shape::<Polyline>() {
            let vertices = &poly.points();
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

            for p in poly.points().iter() {
                points.push(self.location * p);
            }

            return points;
        }

        if let Some(circle) = self.handle.as_shape::<Circle>() {
            let mut poly = circle.to_polyline(ARC_POINTS);
            poly.transform_by(&self.location);
            let (mut points, _) = poly.unwrap();
            // close the shape!
            let pt = points[0];
            points.push(pt);
            return points;
        }
        panic!("unsupported shape type");
    }

    pub fn buffer(&self, delta: f64, join_type: JoinType) -> Shape {
        Shape::polygon(
            buffer(&self.compute_points(), delta, join_type),
            origin(),
            None,
        )
    }

    pub fn buffer_and_simplify(&self, delta: f64, join_type: JoinType, epsilon: f64) -> Shape {
        let ls: geo::LineString<_> = buffer(&self.compute_points(), delta, join_type)
            .iter()
            .map(geo_point)
            .collect();
        let simpler = ls.simplify(&epsilon);
        Shape::polygon(
            simpler
                .0
                .into_iter()
                .map(|p| Point::new(p.x, p.y))
                .collect(),
            origin(),
            None,
        )
    }

    // Explicitly compute the convex hull.  The ncollide hull routines use
    // the implicit hull for collision detection purposes, so we use the geo
    // library to generate an explicit polygon for the hull.
    pub fn convex_hull(shapes: &[Shape]) -> Shape {
        let polys: geo::MultiPolygon<_> = shapes
            .iter()
            .map(|shape| {
                let ls: geo::LineString<_> = shape.compute_points().iter().map(geo_point).collect();
                geo::Polygon::new(ls, Vec::new())
            })
            .collect();
        let hull = polys.convex_hull();
        let points = hull
            .exterior()
            .points_iter()
            .map(|p| Point::new(p.x(), p.y()))
            .collect();
        Shape::polygon(points, origin(), None)
    }

    pub fn contact(&self, other: &Shape, clearance: f64) -> Option<Contact<f64>> {
        contact(
            &self.location,
            &*self.handle,
            &other.location,
            &*other.handle,
            clearance,
        )
    }

    /// Computes the exterior points of the shape, then computes each
    /// possible point of contact between other and self.
    /// Returns (contacts, exterior_points)
    pub fn all_contacts(&self, other: &Shape, clearance: f64) -> (Vec<Point>, Vec<Point>) {
        let points = self.compute_points();
        let mut contacts = Vec::new();

        for i in 0..points.len() {
            let pt = &points[i];
            let prior = if i == 0 { points.len() - 1 } else { i - 1 };

            let prior_pt = &points[prior];

            let line = Shape::line(prior_pt, pt);

            if let Some(contact) = contact(
                &line.location,
                &*line.handle,
                &other.location,
                &*other.handle,
                clearance,
            ) {
                contacts.push(contact.world1);
            }
        }

        (contacts, points)
    }

    pub fn intersects(&self, other: &Shape) -> bool {
        proximity(
            &self.location,
            &*self.handle,
            &other.location,
            &*other.handle,
            0.0,
        ) == Intersecting
    }

    pub fn proximity(&self, other: &Shape, margin: f64) -> Proximity {
        proximity(
            &self.location,
            &*self.handle,
            &other.location,
            &*other.handle,
            margin,
        )
    }

    /// Given pair of points A and B, determine the detour graph.
    /// We do this by computing the points of intersection with self
    /// from a->b and from b->a.  We then connect these to the closest
    /// points on the polygon perimeter and return a graph with edges
    /// set to the distance between the points.
    pub fn detour_path(
        &self,
        a: &Point,
        b: &Point,
        clearance: f64,
    ) -> Option<UnGraphMap<OrderedPoint, f64>> {
        let line = Shape::line(a, b);

        let (contacts, points) = self.all_contacts(&line, clearance);

        if contacts.is_empty() {
            return None;
        }

        let mut graph = UnGraphMap::new();

        for i in 0..points.len() {
            let pt = &points[i];
            let prior = if i == 0 { points.len() - 1 } else { i - 1 };

            let prior_pt = &points[prior];
            graph.add_edge(
                OrderedPoint::from_point(prior_pt),
                OrderedPoint::from_point(pt),
                na::distance(prior_pt, pt),
            );
        }

        // Find the closest point of contact to a, b respectively
        let contact_a = contacts
            .iter()
            .min_by(|ca, cb| order_by_dist(&ca, &cb, &a))
            .unwrap();
        graph.add_edge(
            OrderedPoint::from_point(a),
            OrderedPoint::from_point(&contact_a),
            na::distance(a, contact_a),
        );

        let contact_b = contacts
            .iter()
            .min_by(|ca, cb| order_by_dist(&ca, &cb, &b))
            .unwrap();
        graph.add_edge(
            OrderedPoint::from_point(&contact_b),
            OrderedPoint::from_point(b),
            na::distance(contact_b, b),
        );

        // Find the closest vertex to contact_a, contact_b respectively.
        let close_a = points
            .iter()
            .min_by(|a, b| order_by_dist(&a, &b, &contact_a))
            .unwrap();
        graph.add_edge(
            OrderedPoint::from_point(&contact_a),
            OrderedPoint::from_point(close_a),
            na::distance(contact_a, &close_a),
        );

        let close_b = points
            .iter()
            .min_by(|a, b| order_by_dist(&a, &b, &contact_b))
            .unwrap();
        graph.add_edge(
            OrderedPoint::from_point(&contact_b),
            OrderedPoint::from_point(close_b),
            na::distance(contact_b, &close_b),
        );

        Some(graph)
    }
}

impl Clone for Shape {
    fn clone(&self) -> Shape {
        self.translate(&origin())
    }
}

impl Debug for Shape {
    fn fmt(&self, fmt: &mut Formatter) -> Result<(), Error> {
        if let Some(poly) = self.handle.as_shape::<Polyline>() {
            write!(
                fmt,
                "Polyline with {} vertices at {}",
                poly.points().len(),
                self.location
            )?;
        } else if let Some(circle) = self.handle.as_shape::<Circle>() {
            write!(
                fmt,
                "Circle radius {} at {}",
                circle.radius(),
                self.location
            )?;
        } else {
            write!(fmt, "Unhandled shape type at {}", self.location)?;
        }

        Ok(())
    }
}
