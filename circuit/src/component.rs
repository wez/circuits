use super::{Inst, Pin};
use kicad_parse_gen::footprint::Module;
use std::sync::Arc;

#[derive(Debug)]
struct Inner {
    name: String,
    description: Option<String>,
    pins: Vec<Pin>,
    footprint: Module,
}

impl Eq for Inner {}

impl PartialEq for Inner {
    fn eq(&self, other: &Inner) -> bool {
        self.name == other.name
            && self.description == other.description
            && self.pins == other.pins
            && self.footprint.name == other.footprint.name
    }
}

/// Defines a circuit component
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Component {
    inner: Arc<Inner>,
}

impl Component {
    pub fn new(name: &str, description: Option<String>, pins: Vec<Pin>, footprint: Module) -> Self {
        Self {
            inner: Arc::new(Inner {
                name: name.to_owned(),
                description,
                pins,
                footprint,
            }),
        }
    }

    pub fn pin_idx_by_name(&self, name: &str) -> Option<usize> {
        for (idx, pin) in self.pins().iter().enumerate() {
            if pin.name == name {
                return Some(idx);
            }
        }
        None
    }

    pub fn pins(&self) -> &[Pin] {
        &self.inner.pins
    }

    pub fn footprint(&self) -> &Module {
        &self.inner.footprint
    }

    pub fn name(&self) -> &str {
        self.inner.name.as_str()
    }

    pub fn description(&self) -> Option<&str> {
        self.inner.description.as_ref().map(String::as_str)
    }

    pub fn inst_with_name<S: Into<String>>(&self, name: S) -> Inst {
        Inst::new(name, None, self.clone())
    }

    pub fn inst_with_name_and_value<S: Into<String>>(&self, name: S, value: String) -> Inst {
        Inst::new(name, Some(value), self.clone())
    }
}
