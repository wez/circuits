use crate::{Component, Pin, PinType};
use failure::{format_err, Fallible};
use kicad_parse_gen::footprint::Module;
use kicad_parse_gen::symbol_lib::{self, Draw, PinType as KicadPinType, Symbol, SymbolLib};
use kicad_parse_gen::{read_module, read_symbol_lib};
use lazy_static::lazy_static;
use reqwest::{Client, Url};
use static_http_cache::Cache;
use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

lazy_static! {
    static ref CACHE: Mutex<Cache<Client>> = Mutex::new(init_cache().unwrap());
}

fn init_cache() -> Fallible<Cache<Client>> {
    let location = std::env::temp_dir().join("rust-circuits-crate-cache");
    let client = reqwest::Client::new();
    Cache::new(location, client)
        .map_err(|e| format_err!("Error initializing kicad download cache: {}", e))
}

fn cache_get_as_string(url: Url) -> Fallible<String> {
    let mut file = CACHE
        .lock()
        .unwrap()
        .get(url.clone())
        .map_err(|e| format_err!("get of {} failed: {}", url, e))?;
    let mut s = String::new();
    file.read_to_string(&mut s)?;
    Ok(s)
}

/// LibraryLocator specifies how to load a library.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum LibraryLocator {
    /// Resolve from the library shared on github via HTTP.
    /// It is recommended that rev specify a commit hash rather
    /// than a ref name (such as 'master') so that your code doesn't
    /// get broken when the library is updated.
    KiCadGitHub { name: String, rev: String },
    /// Resolve from the locally installed kicad library.
    /// NOT RECOMMENDED because there is a lot of variance in the
    /// version of the library installed.
    LocallyInstalledKiCad(String),
    /// Resolve from a local file.  This is suitable for eg:
    /// libraries that you have bundled with your code.
    LocalFile(PathBuf),
    /// The symbol library text is inlined as a string.
    InlineString(String),
}

impl LibraryLocator {
    /// Construct a symbol library that references github
    pub fn github(name: &str, rev: &str) -> Self {
        LibraryLocator::KiCadGitHub {
            name: name.to_owned(),
            rev: rev.to_owned(),
        }
    }

    /// Attempt to obtain and parse the symbol library
    pub fn load(&self) -> Fallible<SymbolLib> {
        match self {
            LibraryLocator::KiCadGitHub { name, rev } => {
                let url = format!(
                    "https://raw.githubusercontent.com/KiCad/kicad-symbols/{}/{}.lib",
                    rev, name
                );
                let symbol_file_contents = cache_get_as_string(Url::parse(&url)?)?;
                symbol_lib::parse_str(&symbol_file_contents)
                    .map_err(|e| format_err!("while parsing {} as a symbol library: {}", url, e))
            }
            LibraryLocator::InlineString(symbol_file_contents) => {
                symbol_lib::parse_str(&symbol_file_contents).map_err(|e| {
                    format_err!("while parsing inline data as a symbol library: {}", e)
                })
            }
            LibraryLocator::LocalFile(path) => read_symbol_lib(Path::new(&path))
                .map_err(|e| format_err!("{}: {}", e, path.display())),
            LibraryLocator::LocallyInstalledKiCad(name) => {
                let path = KICAD_INSTALLATION.join(format!("library/{}.lib", name));
                read_symbol_lib(Path::new(&path))
                    .map_err(|e| format_err!("{}: {}", e, path.display()))
            }
        }
    }
}

/// This is used as the key to cache the library+footprint -> Component
/// mapping
#[derive(Hash, Clone, PartialEq, Eq, Debug)]
struct IdKey {
    library: LibraryLocator,
    sym_name: String,
    footprint_name: String,
}

#[derive(Default)]
struct SymbolLoader {
    sym_by_lib: HashMap<LibraryLocator, SymbolLib>,
    mod_by_path: HashMap<String, Module>,
    by_id: HashMap<IdKey, Arc<Component>>,
}

fn symbol_matches_name(sym: &Symbol, name: &str) -> bool {
    if sym.name == name {
        return true;
    }
    sym.aliases.iter().any(|n| n == name)
}

impl SymbolLoader {
    fn resolve_symbol(&mut self, library: LibraryLocator, name: &str) -> Option<Symbol> {
        if let Some(lib) = self.sym_by_lib.get(&library) {
            return lib.find(|sym| symbol_matches_name(sym, name)).cloned();
        }

        match library.load() {
            Err(err) => {
                eprintln!("failed to load library {:?}: {}", library, err);
                return None;
            }
            Ok(lib) => self.sym_by_lib.insert(library.clone(), lib),
        };

        if let Some(lib) = self.sym_by_lib.get(&library) {
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
        library: LibraryLocator,
        name: &str,
        footprint: &str,
    ) -> Option<Arc<Component>> {
        let key = IdKey {
            library: library.clone(),
            sym_name: name.to_owned(),
            footprint_name: footprint.to_owned(),
        };
        if let Some(comp) = self.by_id.get(&key) {
            return Some(Arc::clone(comp));
        }

        let module = self.resolve_module(footprint)?;

        let sym = self.resolve_symbol(library, name)?;

        let comp = Arc::new(convert_to_component(&sym, module));

        self.by_id.insert(key, Arc::clone(&comp));
        Some(comp)
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

pub fn load_from_kicad(
    library: LibraryLocator,
    symbol: &str,
    footprint: &str,
) -> Option<Arc<Component>> {
    LOADER
        .lock()
        .unwrap()
        .resolve_component(library, symbol, footprint)
}

pub fn diode() -> Arc<Component> {
    load_from_kicad(
        LibraryLocator::github("pspice", "e88e195c388f5fdd98a5e6cb31ed3e38a42ad64d"),
        "DIODE",
        "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal",
    )
    .unwrap()
}

pub fn mx_switch() -> Arc<Component> {
    load_from_kicad(
        LibraryLocator::github("Switch", "e88e195c388f5fdd98a5e6cb31ed3e38a42ad64d"),
        "SW_Push",
        "Button_Switch_Keyboard:SW_Cherry_MX_1.00u_PCB",
    )
    .unwrap()
}
