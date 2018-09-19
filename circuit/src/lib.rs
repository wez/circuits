extern crate kicad_parse_gen;
extern crate petgraph;
#[macro_use]
extern crate lazy_static;

use kicad_parse_gen::footprint::Module;
use petgraph::prelude::*;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, ATOMIC_USIZE_INIT};
use std::sync::Arc;

pub mod components;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum PinType {
    Power,
    Ground,
    In,
    Out,
    InOut,
    Unknown,
}

/// Defines a connected set of terminals
#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub struct Net {
    pub name: String,
}

static NEXT_NET_ID: AtomicUsize = ATOMIC_USIZE_INIT;

impl Net {
    pub fn new() -> Self {
        let id = NEXT_NET_ID.fetch_add(1, ::std::sync::atomic::Ordering::Relaxed);
        Net {
            name: format!("N${}", id),
        }
    }

    pub fn with_name<S: Into<String>>(s: S) -> Self {
        Self { name: s.into() }
    }

    pub fn is_auto_net(&self) -> bool {
        self.name.starts_with("N$")
    }
}

impl PartialOrd for Net {
    fn partial_cmp(&self, other: &Net) -> Option<Ordering> {
        if self.is_auto_net() == other.is_auto_net() {
            self.name.partial_cmp(&other.name)
        } else if !self.is_auto_net() {
            Some(Ordering::Less)
        } else {
            Some(Ordering::Greater)
        }
    }
}

impl Ord for Net {
    fn cmp(&self, other: &Net) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

/// The definition of a connection point on a component
#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub struct Pin {
    pub name: String,
    pub pin_type: PinType,
    pub description: Option<String>,
}

/// Defines a circuit component
#[derive(Clone, Debug)]
pub struct Component {
    pub name: String,
    pub description: Option<String>,
    pub pins: Vec<Pin>,
    pub footprint: Option<Module>,
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
            && self.footprint.is_some() == other.footprint.is_some()
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

#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub struct PinAssignment {
    /// index into Component.pins
    pub pin_index: usize,
    /// The connected net
    pub net: Net,
}

/// An instance of a component in a circuit
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Inst {
    /// reference eg: R1
    pub name: String,
    /// Value of the component, eg: 220
    pub value: Option<String>,
    /// the component definition
    pub component: Arc<Component>,
    /// pin to net assignments
    assignments: Vec<PinAssignment>,
}

impl Inst {
    pub fn new<S: Into<String>>(name: S, value: Option<String>, component: Arc<Component>) -> Self {
        Self {
            name: name.into(),
            value,
            component,
            assignments: Vec::new(),
        }
    }

    pub fn pin_idx_by_name(&self, name: &str) -> Option<usize> {
        self.component.pin_idx_by_name(name)
    }

    pub fn pin_ref<'a>(&'a mut self, name: &str) -> PinRef<'a> {
        self.pin_idx_by_name(name)
            .map(move |idx| PinRef { inst: self, idx })
            .unwrap()
    }

    pub fn pin_idx_is_assigned(&self, idx: usize) -> bool {
        for ass in &self.assignments {
            if ass.pin_index == idx {
                return true;
            }
        }
        false
    }
}

#[derive(Debug)]
pub struct PinRef<'a> {
    inst: &'a mut Inst,
    idx: usize,
}

impl<'a> PinRef<'a> {
    pub fn connect_net(&mut self, net: Net) {
        self.inst.assignments.push(PinAssignment {
            pin_index: self.idx,
            net,
        });
    }

    pub fn connect_pin(&mut self, mut other: PinRef) {
        let net = Net::new();
        self.connect_net(net.clone());
        other.connect_net(net);
    }
}

#[derive(PartialEq, Eq, Debug, Default)]
pub struct CircuitBuilder {
    instances: Vec<Inst>,
}

/// For the set of instances provided, compute the set of connected
/// terminals and merge connected nets together.  The merge favors
/// explicitly named nets over auto-generated nets.
fn merge_connected_nets(mut instances: Vec<Inst>) -> (Vec<Inst>, Vec<Net>) {
    let mut net_by_comp = Vec::new();
    let mut g = UnGraphMap::new();
    #[derive(Clone, Copy, PartialOrd, Ord, Hash, PartialEq, Eq, Debug)]
    enum Node {
        InstNode { inst_idx: usize, pin_index: usize },
        NetNode(usize),
    }

    let components = {
        let mut net_to_id = HashMap::new();
        let mut id_to_net = HashMap::new();

        for (inst_idx, inst) in instances.iter().enumerate() {
            for ass in &inst.assignments {
                let next_id = net_to_id.len();
                let net_id = *net_to_id.entry(&ass.net).or_insert(next_id);
                id_to_net.entry(net_id).or_insert(ass.net.clone());

                let pin_index = ass.pin_index;

                g.add_edge(
                    Node::InstNode {
                        inst_idx,
                        pin_index,
                    },
                    Node::NetNode(net_id),
                    0,
                );
            }
        }

        // Compute the strongly connected components to obtain the merged set
        // of terminals and nets for each component
        let components = petgraph::algo::tarjan_scc(&g);
        let mut next_auto_id = 0;

        for comp in &components {
            let mut nets: Vec<&Net> = comp
                .iter()
                .filter_map(|node| match node {
                    Node::NetNode(net_id) => id_to_net.get(net_id),
                    _ => None,
                })
                .collect();

            // Pick the winning net
            nets.sort_unstable();

            // if the best net is an auto generated net, then generate a new
            // one starting from zero to keep the set of nets consistent and
            // well-defined in a built circuit
            let winner = if nets[0].is_auto_net() {
                let net = Net::with_name(format!("N${}", next_auto_id));
                next_auto_id += 1;
                net
            } else {
                nets[0].clone()
            };

            net_by_comp.push(winner);
        }

        components
    };

    // Now clear out any pin assignments from the instances, as we
    // are going to replace them all
    for inst in &mut instances {
        inst.assignments.clear();
    }

    // And finally, set up the assignments based on the components
    // that we collected
    for (idx, comp) in components.iter().enumerate() {
        let net = &net_by_comp[idx];
        for node in comp {
            match node {
                Node::InstNode {
                    inst_idx,
                    pin_index,
                } => {
                    if !instances[*inst_idx].pin_idx_is_assigned(*pin_index) {
                        instances[*inst_idx].assignments.push(PinAssignment {
                            pin_index: *pin_index,
                            net: (*net).clone(),
                        });
                    }
                }
                _ => {}
            }
        }
    }

    (instances, net_by_comp)
}

impl CircuitBuilder {
    pub fn add_inst(&mut self, inst: Inst) {
        self.instances.push(inst);
    }

    pub fn build(self) -> Circuit {
        let (instances, nets) = merge_connected_nets(self.instances);
        Circuit { instances, nets }
    }
}

#[derive(PartialEq, Eq, Debug, Default)]
pub struct Circuit {
    instances: Vec<Inst>,
    nets: Vec<Net>,
}

impl Circuit {}

#[cfg(test)]
mod tests {
    use super::*;
    use components::*;

    #[test]
    fn kicad_symbol() {
        let mut circuit = CircuitBuilder::default();
        let mut sw = load_from_kicad(
            "Switch",
            "SW_Push",
            "Button_Switch_Keyboard:SW_Cherry_MX1A_1.00u_PCB",
        ).unwrap()
            .inst_with_name("SW1");
        let mut diode = diode().inst_with_name("D1");
        sw.pin_ref("2").connect_pin(diode.pin_ref("A"));
        circuit.add_inst(sw);
        circuit.add_inst(diode);

        let circuit = circuit.build();

        assert_eq!(vec![Net::with_name("N$0")], circuit.nets);
    }

    #[test]
    fn one_connection() {
        let mut circuit = CircuitBuilder::default();
        let mut sw = switch().inst_with_name("SW1");
        let mut diode = diode().inst_with_name("D1");
        sw.pin_ref("2").connect_pin(diode.pin_ref("A"));
        circuit.add_inst(sw);
        circuit.add_inst(diode);

        let circuit = circuit.build();

        assert_eq!(vec![Net::with_name("N$0")], circuit.nets);
    }

    #[test]
    fn redundant_nets() {
        let mut circuit = CircuitBuilder::default();
        let mut sw = switch().inst_with_name("SW1");
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
