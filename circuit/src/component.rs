use super::{Inst, Pin};
use kicad_parse_gen::footprint::Module;
use std::sync::Arc;

#[derive(Debug)]
struct Inner {
    name: String,
    description: Option<String>,
    pins: Vec<Pin>,
    footprint: Option<Module>,
}

impl Eq for Inner {}

fn is_same_optional_module(a: &Option<Module>, b: &Option<Module>) -> bool {
    if a.is_none() && b.is_none() {
        return true;
    }
    if a.is_some() && b.is_some() {
        let a = a.as_ref().unwrap();
        let b = b.as_ref().unwrap();
        a.name == b.name
    } else {
        false
    }
}

impl PartialEq for Inner {
    fn eq(&self, other: &Inner) -> bool {
        self.name == other.name
            && self.description == other.description
            && self.pins == other.pins
            && is_same_optional_module(&self.footprint, &other.footprint)
    }
}

/// Defines a circuit component
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Component {
    inner: Arc<Inner>,
}

impl Component {
    pub fn new(
        name: &str,
        description: Option<String>,
        pins: Vec<Pin>,
        footprint: Option<Module>,
    ) -> Self {
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

    pub fn footprint(&self) -> Option<&Module> {
        self.inner.footprint.as_ref()
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
