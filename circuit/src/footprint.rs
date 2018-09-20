//! The default accessors in kicad_parse_gen are fairly simplistic and are
//! not sufficient for my needs, so this module provides a bit more glue
//! to make things do what I want.
use kicad_parse_gen::footprint::{At, Element, Flip, Module, Net as KicadNet};

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
