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
    use crate::circuit::ErcItem;
    use crate::loader::*;

    fn setup_log() {
        let mut builder = env_logger::Builder::from_default_env();
        builder
            .is_test(true)
            //.filter(None, log::LevelFilter::Info)
            .try_init()
            .ok();
    }

    #[test]
    fn erc_power() {
        let mut circuit = CircuitBuilder::default();
        let mut vdc = vdc_supply().inst_with_name("VDC");
        let mut flag = pwr_flag().inst_with_name("P");
        vdc.pin_ref("+VDC").connect_pin(flag.pin_ref("pwr"));
        circuit.add_inst(vdc);
        circuit.add_inst(flag);
        let circuit = circuit.build();
        let problems = circuit.eletrical_rules_check();
        assert_eq!(problems, vec![]);
    }

    #[test]
    fn erc_power_not_connected() {
        let mut circuit = CircuitBuilder::default();
        let mut vdc = vdc_supply().inst_with_name("VDC");
        // The net is here purely for test determinism, otherwise
        // we end up with an unpredictable auto-generated net name
        vdc.pin_ref("+VDC").connect_net(Net::with_name("vdc"));
        circuit.add_inst(vdc);
        let circuit = circuit.build();
        let problems = circuit.eletrical_rules_check();
        assert_eq!(
            problems,
            vec![ErcItem::error(
                "Net `vdc`: drive `None` is below the minimum allowed drive \
                 `Power` for component `VDC` pin 0 PowerInput (Do you need \
                 to connect a pwr_flag() to this net?)"
            )]
        );
    }

    #[test]
    fn one_connection() {
        setup_log();
        let mut circuit = CircuitBuilder::default();
        let mut sw = mx_switch().inst_with_name("SW1");
        let mut diode = diode().inst_with_name("D1");
        sw.pin_ref("2").connect_pin(diode.pin_ref("A"));
        circuit.add_inst(sw);
        circuit.add_inst(diode);

        let circuit = circuit.build();
        assert!(circuit.eletrical_rules_check().is_empty());
        let layout = circuit.to_pcb_layout();

        use kicad_parse_gen::write_layout;
        use std::path::PathBuf;
        write_layout(&layout, &PathBuf::from("/tmp/woot.kicad_pcb")).unwrap();

        assert_eq!(vec![Net::with_name("N$0")], circuit.nets);
    }

    #[test]
    fn redundant_nets() {
        setup_log();
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
