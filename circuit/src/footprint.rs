//! The default accessors in kicad_parse_gen are fairly simplistic and are
//! not sufficient for my needs, so this module provides a bit more glue
//! to make things do what I want.
use geo::prelude::*;
use geo::{Coordinate, Geometry, GeometryCollection, Line, LineString, Point as GeoPoint, Polygon};
use kicad_parse_gen::footprint::{
    At, Element, Flip, FpLine, FpPoly, Layer, LayerSide, LayerType, Module, Net as KicadNet, Pad,
    PadShape, Pts, Xy, XyType,
};

use point::{Point, Rotation};

pub trait HasLocation {
    fn get_location(&self) -> (Point, Rotation);
    fn set_location(&mut self, point: Point, rotation: Rotation);
}

pub trait LayerManipulation {
    fn flip_layer(&mut self);
}

pub trait AssignComponentNet {
    fn set_net(&mut self, pad_name: &str, pin_index: usize, net: &KicadNet);
}

pub trait ToGeom {
    fn to_geom(&self) -> Geometry<f64>;
}

fn is_physical_layer(layer: &Layer) -> bool {
    // TODO: actually check the layer
    true
}

impl ToGeom for Module {
    fn to_geom(&self) -> Geometry<f64> {
        let (point, rotation) = self.get_location();
        let collection: GeometryCollection<f64> = self
            .elements
            .iter()
            .filter_map(|element| -> Option<Geometry<f64>> {
                match element {
                    Element::FpPoly(poly) if is_physical_layer(&poly.layer) => Some(poly.to_geom()),
                    Element::FpLine(line) if is_physical_layer(&line.layer) => Some(line.to_geom()),
                    Element::Pad(pad) => Some(pad.to_geom()),
                    _ => None,
                }
            }).map(|geom| {
                geom.translate(*point.x(), *point.y())
                    //.rotate_around_point(**rotation, GeoPoint::new(0.0, 0.0))
                    .rotate_around_point(360.0-**rotation, GeoPoint::new(*point.x(), *point.y()))
            }).collect();
        Geometry::GeometryCollection(collection)
    }
}

fn xy_to_coord(xy: &Xy) -> Coordinate<f64> {
    Coordinate { x: xy.x, y: xy.y }
}

impl ToGeom for FpLine {
    fn to_geom(&self) -> Geometry<f64> {
        // TODO: consider self.width
        let start = xy_to_coord(&self.start);
        let end = xy_to_coord(&self.end);
        Geometry::Line(Line::new(start, end))
    }
}

impl ToGeom for FpPoly {
    fn to_geom(&self) -> Geometry<f64> {
        // TODO: consider self.width
        let points: LineString<f64> = self.pts.elements.iter().map(xy_to_coord).collect();
        Geometry::Polygon(Polygon::new(points, vec![]))
    }
}

impl ToGeom for Pad {
    fn to_geom(&self) -> Geometry<f64> {
        let size = xy_to_coord(&self.size);
        let points = match self.shape {
            // TODO: more faithfully describe the other shapes
            PadShape::Rect | PadShape::Circle | PadShape::Oval | PadShape::Trapezoid => {
                LineString(vec![
                    Coordinate {
                        x: -size.x / 2.0,
                        y: -size.y / 2.0,
                    },
                    Coordinate {
                        x: size.x / 2.0,
                        y: -size.y / 2.0,
                    },
                    Coordinate {
                        x: size.x / 2.0,
                        y: size.y / 2.0,
                    },
                    Coordinate {
                        x: -size.x / 2.0,
                        y: size.y / 2.0,
                    },
                ])
            }
        };
        let poly = Polygon::new(points, vec![])
            .translate(self.at.x, self.at.y)
            .rotate_around_point(360.0 - self.at.rot, GeoPoint::new(self.at.x, self.at.y));
        Geometry::Polygon(poly)
    }
}

impl AssignComponentNet for Module {
    fn set_net(&mut self, pad_name: &str, pin_index: usize, net: &KicadNet) {
        // This makes me a little sad; the symbols can define pin names
        // that are strings.  The footprints can define pad names that
        // are strings.  However, some footprints define their pad names
        // to be number strings that match up to the ordinal number of
        // the pin rather than the pin name.  This means that we need
        // to try to match names and number strings. This feels a bit
        // ambiguous to me; consider a component that has a mixture of
        // names and numbers for its pin labels (eg: a MCU defines a "VCC"
        // pad and a "12" pad).  If VCC happens to also be index 12 then it
        // it ambiguous which of these we should set.   I don't have a good
        // solution for this issue.  We take the approach of trying the name
        // string first and then falling back to the pin index if we didn't
        // find the name.
        let mut found_name = false;
        for element in &mut self.elements {
            if let Element::Pad(ref mut pad) = element {
                if pad.name == pad_name {
                    pad.set_net(net.clone());
                    found_name = true;
                }
            }
        }

        if !found_name {
            let num_name = format!("{}", pin_index);
            for element in &mut self.elements {
                if let Element::Pad(ref mut pad) = element {
                    if pad.name == num_name {
                        pad.set_net(net.clone());
                    }
                }
            }
        }
    }
}

impl LayerManipulation for Module {
    fn flip_layer(&mut self) {
        // Ensure that we have a location set.
        // In addition, we perform the rotation for ourselves,
        // skipping the default flip behavior, because the default
        // adjusts the position of the element.
        let (point, rotation) = self.get_location();
        self.set_location(point, Rotation::new(360.0 - **rotation));

        for element in &mut self.elements {
            match element {
                Element::At(_) => {
                    // Skip: we handled this already above
                }
                e @ _ => e.flip(),
            }
        }
    }
}

impl HasLocation for Module {
    fn get_location(&self) -> (Point, Rotation) {
        let (x, y) = self.at();
        let point = Point::new(x, y);
        let rotation = Rotation::new(self.get_rotation());

        (point, rotation)
    }

    fn set_location(&mut self, point: Point, rotation: Rotation) {
        let new_value = At {
            x: *point.x(),
            y: *point.y(),
            rot: **rotation,
        };
        for element in &mut self.elements {
            if let Element::At(ref mut at) = element {
                *at = new_value;
                return;
            }
        }

        self.elements.push(Element::At(new_value));
    }
}

pub fn polygon_to_fp_poly(poly: &Polygon<f64>, layer: &Layer) -> FpPoly {
    FpPoly {
        width: 0.15,
        layer: layer.clone(),
        pts: Pts {
            elements: poly
                .exterior
                .points_iter()
                .map(|p| Xy {
                    x: p.x(),
                    y: p.y(),
                    t: XyType::Xy,
                }).collect(),
        },
    }
}
