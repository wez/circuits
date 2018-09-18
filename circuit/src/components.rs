use kicad_parse_gen::error::KicadError;
use kicad_parse_gen::read_symbol_lib;
use kicad_parse_gen::symbol_lib::{Draw, PinType as KicadPinType, Symbol, SymbolLib};
use std::collections::HashMap;
use std::path::Path;
use std::sync::{Arc, Mutex};

use {Component, Pin, PinType};

#[derive(Default)]
struct SymbolLoader {
    by_lib: HashMap<String, SymbolLib>,
    by_id: HashMap<String, Arc<Component>>,
}

fn symbol_matches_name(sym: &Symbol, name: &str) -> bool {
    if sym.name == name {
        return true;
    }
    sym.aliases.iter().any(|n| n == name)
}

impl SymbolLoader {
    fn resolve_symbol(&mut self, library: &str, name: &str) -> Option<Symbol> {
        if let Some(lib) = self.by_lib.get(library) {
            return lib.find(|sym| symbol_matches_name(sym, name)).cloned();
        }

        match self.load_library(library) {
            Err(err) => {
                eprintln!("failed to load library {}: {}", library, err);
                return None;
            }
            Ok(_) => {}
        };

        if let Some(lib) = self.by_lib.get(library) {
            return lib.find(|sym| symbol_matches_name(sym, name)).cloned();
        }

        None
    }

    fn resolve_component(&mut self, library: &str, name: &str) -> Option<Arc<Component>> {
        let key = format!("{}:{}", library, name);
        if let Some(comp) = self.by_id.get(&key) {
            return Some(Arc::clone(comp));
        }

        let sym = self.resolve_symbol(library, name)?;

        let comp = Arc::new(convert_to_component(&sym));

        self.by_id.insert(key, Arc::clone(&comp));
        Some(comp)
    }

    fn load_library(&mut self, library: &str) -> Result<(), KicadError> {
        let path = format!("/usr/share/kicad/library/{}.lib", library);
        self.by_lib
            .insert(library.into(), read_symbol_lib(Path::new(&path))?);
        Ok(())
    }
}

fn convert_to_component(symbol: &Symbol) -> Component {
    let pins = symbol
        .draw
        .iter()
        .filter_map(|d| match d {
            Draw::Pin(pin) => Some(Pin {
                name: pin.name.clone(),
                description: Some(pin.number.clone()),
                pin_type: match pin.pin_type {
                    KicadPinType::Input => PinType::In,
                    KicadPinType::Output => PinType::Out,
                    KicadPinType::Bidi => PinType::InOut,
                    KicadPinType::PowerOutput => PinType::Power,
                    _ => PinType::Unknown,
                },
            }),
            _ => None,
        })
        .collect();

    Component {
        name: symbol.name.clone(),
        description: None,
        pins,
    }
}

lazy_static! {
    static ref LOADER: Mutex<SymbolLoader> = Mutex::new(SymbolLoader::default());
}

pub fn load_from_kicad(library: &str, symbol: &str) -> Option<Component> {
    let symbol = LOADER.lock().unwrap().resolve_symbol(library, symbol)?;
    Some(convert_to_component(&symbol))
}

pub fn diode() -> Component {
    Component {
        name: "DIODE".into(),
        description: None,
        pins: vec![
            Pin {
                name: "K".into(),
                description: Some("Cathode".into()),
                pin_type: PinType::In,
            },
            Pin {
                name: "A".into(),
                description: Some("Anode".into()),
                pin_type: PinType::In,
            },
        ],
    }
}

pub fn switch() -> Component {
    Component {
        name: "SW".into(),
        description: None,
        pins: vec![
            Pin {
                name: "1".into(),
                description: None,
                pin_type: PinType::Unknown,
            },
            Pin {
                name: "2".into(),
                description: None,
                pin_type: PinType::Unknown,
            },
        ],
    }
}
