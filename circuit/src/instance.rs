use crate::footprint::{HasLocation, LayerManipulation, ToGeom};
use crate::geometry::convex_hull;
use crate::point::{Point, Rotation};
use crate::{Component, Net};
use geo::Polygon;
use kicad_parse_gen::footprint::Module;
use petgraph::prelude::*;
use std::collections::HashMap;

#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub struct PinAssignment {
    /// index into Component.pins
    pub pin_index: usize,
    /// The connected net
    pub net: Net,
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

/// An instance of a component in a circuit
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Inst {
    /// reference eg: R1
    pub name: String,
    /// Value of the component, eg: 220
    pub value: Option<String>,
    /// the component definition
    pub component: Component,
    /// Whether the footprint is placed on the opposite side of the board
    pub flipped: bool,
    /// The location of the centroid of the footprint
    pub coordinates: Point,
    /// Rotation applied to the footprint, measured in degrees
    pub rotation: Rotation,
    /// pin to net assignments
    assignments: Vec<PinAssignment>,
}

impl Inst {
    pub fn new<S: Into<String>>(name: S, value: Option<String>, component: Component) -> Self {
        Self {
            name: name.into(),
            value,
            component,
            assignments: Vec::new(),
            flipped: false,
            coordinates: Point::new(0.0, 0.0),
            rotation: Rotation::new(0.0),
        }
    }

    pub fn assignments(&self) -> &[PinAssignment] {
        &self.assignments
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

    /// Return the footprint for this instance with its location
    /// filled out.  This footprint doesn't have any pad assignment
    /// performed.  It is intended to be used to compute size and
    /// bounds only.
    pub fn make_footprint(&self) -> Module {
        let mut footprint = self.component.footprint().clone();
        footprint.set_location(self.coordinates, self.rotation);
        if self.flipped {
            footprint.flip_layer();
        }
        footprint
    }

    pub fn hull(&self) -> Polygon<f64> {
        convex_hull(self.make_footprint().to_geom())
    }

    /// For the set of instances provided, compute the set of connected
    /// terminals and merge connected nets together.  The merge favors
    /// explicitly named nets over auto-generated nets.
    pub fn merge_connected_nets(mut instances: Vec<Inst>) -> (Vec<Inst>, Vec<Net>) {
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
                    id_to_net.entry(net_id).or_insert_with(|| ass.net.clone());

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
                if let Node::InstNode {
                    inst_idx,
                    pin_index,
                } = node
                {
                    if !instances[*inst_idx].pin_idx_is_assigned(*pin_index) {
                        instances[*inst_idx].assignments.push(PinAssignment {
                            pin_index: *pin_index,
                            net: (*net).clone(),
                        });
                    }
                }
            }
        }

        (instances, net_by_comp)
    }
}
