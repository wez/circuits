use super::{Inst, Pin};
use kicad_parse_gen::footprint::Module;
use std::sync::Arc;

/// Defines a circuit component
#[derive(Clone, Debug)]
pub struct Component {
    pub name: String,
    pub description: Option<String>,
    pub pins: Vec<Pin>,
    pub footprint: Module,
}

impl Component {
    pub fn pin_idx_by_name(&self, name: &str) -> Option<usize> {
        for (idx, pin) in self.pins.iter().enumerate() {
            if pin.name == name {
                return Some(idx);
            }
        }
        None
    }
}

impl PartialEq for Component {
    fn eq(&self, other: &Component) -> bool {
        self.name == other.name
            && self.description == other.description
            && self.pins == other.pins
            && self.footprint.name == other.footprint.name
    }
}

impl Eq for Component {}

/// This trait allows defining convenience methods on Arc<Component> that
/// would otherwise logically be methods on the Component struct itself.
pub trait MakeInst {
    fn inst_with_name<S: Into<String>>(&self, name: S) -> Inst;
    fn inst_with_name_and_value<S: Into<String>>(&self, name: S, value: String) -> Inst;
}

impl MakeInst for Arc<Component> {
    fn inst_with_name<S: Into<String>>(&self, name: S) -> Inst {
        Inst::new(name, None, Arc::clone(self))
    }

    fn inst_with_name_and_value<S: Into<String>>(&self, name: S, value: String) -> Inst {
        Inst::new(name, Some(value), Arc::clone(self))
    }
}
