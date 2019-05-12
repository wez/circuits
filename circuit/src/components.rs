use crate::{Component, Pin, PinType};
use failure::{format_err, Fallible};
use kicad_parse_gen::footprint::Module;
use kicad_parse_gen::symbol_lib::{Draw, PinType as KicadPinType, Symbol, SymbolLib};
use kicad_parse_gen::{read_module, read_symbol_lib};
use lazy_static::lazy_static;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

#[derive(Default)]
struct SymbolLoader {
    sym_by_lib: HashMap<String, SymbolLib>,
    mod_by_path: HashMap<String, Module>,
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
        if let Some(lib) = self.sym_by_lib.get(library) {
            return lib.find(|sym| symbol_matches_name(sym, name)).cloned();
        }

        if let Err(err) = self.load_library(library) {
            eprintln!("failed to load library {}: {}", library, err);
            return None;
        }

        if let Some(lib) = self.sym_by_lib.get(library) {
            return lib.find(|sym| symbol_matches_name(sym, name)).cloned();
        }

        None
    }

    fn resolve_module(&mut self, footprint: &str) -> Option<Module> {
        if let Some(module) = self.mod_by_path.get(footprint) {
            return Some(module.clone());
        }

        if let Err(err) = self.load_module(&footprint) {
            eprintln!("failed to load footprint {}: {}", footprint, err);
            return None;
        }

        self.mod_by_path.get(footprint).cloned()
    }

    fn resolve_component(
        &mut self,
        library: &str,
        name: &str,
        footprint: &str,
    ) -> Option<Arc<Component>> {
        let key = format!("{}:{}:{}", library, name, footprint);
        if let Some(comp) = self.by_id.get(&key) {
            return Some(Arc::clone(comp));
        }

        let module = self.resolve_module(footprint)?;

        let sym = self.resolve_symbol(library, name)?;

        let comp = Arc::new(convert_to_component(&sym, module));

        self.by_id.insert(key, Arc::clone(&comp));
        Some(comp)
    }

    fn load_library(&mut self, library: &str) -> Fallible<()> {
        let path = KICAD_INSTALLATION.join(format!("library/{}.lib", library));
        self.sym_by_lib.insert(
            library.into(),
            read_symbol_lib(Path::new(&path))
                .map_err(|e| format_err!("{}: {}", e, path.display()))?,
        );
        Ok(())
    }

    fn load_module(&mut self, footprint: &str) -> Fallible<()> {
        let path = PathBuf::from(footprint);
        let module = if path.is_absolute() {
            read_module(&path)?
        } else {
            let elements: Vec<&str> = footprint.splitn(2, ':').collect();
            let mut abs_path = KICAD_INSTALLATION.join("modules");
            abs_path.push(format!("{}.pretty", elements[0]));
            abs_path.push(format!("{}.kicad_mod", elements[1]));
            read_module(&abs_path).map_err(|e| format_err!("{}: {}", e, abs_path.display()))?
        };

        self.mod_by_path.insert(footprint.to_string(), module);
        Ok(())
    }
}

fn convert_to_component(symbol: &Symbol, module: Module) -> Component {
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
        footprint: module,
    }
}

pub fn find_kicad_install() -> PathBuf {
    let candidates = [
        "/Library/Application Support/kicad",
        "/usr/local/share/kicad",
        "/usr/share/kicad",
    ];
    for candidate in &candidates {
        let candidate = Path::new(candidate);
        if candidate.is_dir() {
            return candidate.to_path_buf();
        }
    }
    panic!("cannot find your kicad installation");
}

lazy_static! {
    static ref LOADER: Mutex<SymbolLoader> = Mutex::new(SymbolLoader::default());
    static ref KICAD_INSTALLATION: PathBuf = find_kicad_install();
}

pub fn load_from_kicad(library: &str, symbol: &str, footprint: &str) -> Option<Arc<Component>> {
    LOADER
        .lock()
        .unwrap()
        .resolve_component(library, symbol, footprint)
}

pub fn diode() -> Arc<Component> {
    load_from_kicad(
        "pspice",
        "DIODE",
        "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal",
    )
    .unwrap()
}

pub fn mx_switch() -> Arc<Component> {
    load_from_kicad(
        "Switch",
        "SW_Push",
        "Button_Switch_Keyboard:SW_Cherry_MX_1.00u_PCB",
    )
    .unwrap()
}
