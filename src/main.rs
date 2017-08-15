#[macro_use]
extern crate error_chain;
use std::process::exit;

#[macro_use]
extern crate nom;

extern crate itertools;

mod dsn;
mod geom;

use std::error::Error;
use self::dsn::Pcb;

fn go() -> Result<(), Box<Error>> {
    println!("Hello, world!");
    let pcb = Pcb::parse("left-pcb-no-fill.dsn")?;
    println!("made pcb: {:#?}", pcb);
    Ok(())
}

fn main() {
    match go() {
        Err(e) => {
            println!("Error: {}", e.to_string());
            exit(1);
        }
        _ => {}
    }
}
