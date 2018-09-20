//! The default accessors in kicad_parse_gen are fairly simplistic and are
//! not sufficient for my needs, so this module provides a bit more glue
//! to make things do what I want.
use kicad_parse_gen::footprint::{At, Element, Flip, Module};

use point::{Point, Rotation};

pub trait HasLocation {
    fn get_location(&self) -> (Point, Rotation);
    fn set_location(&mut self, point: Point, rotation: Rotation);
}

pub trait LayerManipulation {
    fn flip_layer(&mut self);
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
