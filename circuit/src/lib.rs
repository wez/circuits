mod circuit;
mod component;
pub(crate) mod geometry;
mod instance;
mod net;
mod pin;

pub mod footprint;
pub mod layout;
pub mod loader;
pub mod point;

pub use crate::circuit::*;
pub use component::*;
pub use instance::*;
pub use net::Net;
pub use pin::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::loader::*;

    #[test]
    fn one_connection() {
        let mut circuit = CircuitBuilder::default();
        let mut sw = mx_switch().inst_with_name("SW1");
        let mut diode = diode().inst_with_name("D1");
        sw.pin_ref("2").connect_pin(diode.pin_ref("A"));
        circuit.add_inst(sw);
        circuit.add_inst(diode);

        let circuit = circuit.build();
        let layout = circuit.to_pcb_layout();

        use kicad_parse_gen::write_layout;
        use std::path::PathBuf;
        write_layout(&layout, &PathBuf::from("/tmp/woot.kicad_pcb")).unwrap();

        assert_eq!(vec![Net::with_name("N$0")], circuit.nets);
    }

    #[test]
    fn redundant_nets() {
        let mut circuit = CircuitBuilder::default();
        let mut sw = mx_switch().inst_with_name("SW1");
        let mut diode = diode().inst_with_name("D1");

        let _ = Net::new();
        sw.pin_ref("2").connect_pin(diode.pin_ref("A"));
        sw.pin_ref("2").connect_net(Net::with_name("preferred"));
        sw.pin_ref("2").connect_net(Net::new());
        circuit.add_inst(sw);
        circuit.add_inst(diode);

        let circuit = circuit.build();

        assert_eq!(vec![Net::with_name("preferred")], circuit.nets);
    }
}
