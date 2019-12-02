use crate::dsn;
use crate::geom::{OrderedPoint, Point, Shape};
use crate::twonets;
use ordered_float::OrderedFloat;
use std::collections::{HashMap, HashSet};
use std::hash::{Hash, Hasher};
use std::sync::Arc;

pub type LayerSet = HashSet<u8>;

#[derive(Clone, Debug)]
pub struct Terminal {
    // name of this terminal (assuming it is a pin on a component)
    pub identifier: Option<String>,

    // name of the net to which this terminal is connected
    pub net_name: Option<String>,

    // which layers this terminal is connected to
    pub layers: LayerSet,

    // footprint of the terminal
    pub shape: Shape,

    // center point of the terminal
    pub point: Point,
}

pub type TwoNet = (Arc<Terminal>, Arc<Terminal>);

#[derive(Debug, Clone)]
pub struct Features {
    pub terminals_by_net: HashMap<String, Vec<Arc<Terminal>>>,
    pub obstacles: Vec<Arc<Terminal>>,
    pub all_pads: Vec<Arc<Terminal>>,
    pub twonets_by_net: HashMap<String, Vec<TwoNet>>,
    pub all_layers: LayerSet,
    pub paths_by_layer: HashMap<u8, Vec<(OrderedPoint, OrderedPoint)>>,
    pub cdt_edges: Vec<(Point, Point)>,
    pub via_shape: Shape,
}

impl Eq for Terminal {}

impl PartialEq for Terminal {
    // Note: we are not including the actual shape in equality tests because
    // the shapehandle does not define equality methods we can use.  We are
    // relying on the terminals not having the same point coordinates.
    fn eq(&self, other: &Terminal) -> bool {
        self.identifier == other.identifier
            && self.net_name == other.net_name
            && self.layers == other.layers
            && OrderedFloat(self.point.coords.x) == OrderedFloat(self.point.coords.y)
    }
}

impl Hash for Terminal {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.identifier.hash(state);
        self.net_name.hash(state);
        OrderedFloat(self.point.coords.x).hash(state);
        OrderedFloat(self.point.coords.y).hash(state);
        for l in self.layers.iter() {
            l.hash(state);
        }
    }
}

impl Terminal {
    fn merge(&mut self, other: &Self) {
        // TODO: this may only really be valid if the pads are identical,
        // which should be the case for our simple keyboard circuits
        self.layers = self.layers.union(&other.layers).cloned().collect();
    }
}

fn check_layer_contig(layers: &LayerSet) {
    let mut layer_vec = Vec::with_capacity(layers.len());
    for layer_idx in layers.iter() {
        layer_vec.push(*layer_idx);
    }
    layer_vec.sort();
    for (idx, layer_idx) in layer_vec.iter().enumerate() {
        if idx as u8 != *layer_idx {
            panic!(
                "layer indices must be contiguous and start at 0.  Have {:?}",
                layers
            );
        }
    }
}

impl Features {
    // Extract a list of terminals from a pcb
    pub fn from_pcb(pcb: &dsn::Pcb) -> Features {
        // First build the map of ident -> netname by inverting the
        // mapping described by the pcb structure
        let mut ident_to_net = HashMap::new();
        let mut by_net: HashMap<String, Vec<Arc<Terminal>>> = HashMap::new();

        let mut all_layers = LayerSet::new();
        let mut layer_name_to_index = HashMap::new();
        for layer in pcb.structure.layers.iter() {
            all_layers.insert(layer.index as u8);
            layer_name_to_index.insert(layer.name.clone(), layer.index as u8);
        }

        // We make the assumption that the layer indices are sequential,
        // contiguous and start with 0 elsewhere, so let's check that here.
        check_layer_contig(&all_layers);

        let via_shape = &pcb.pad_defs[&pcb.structure.via]
            .pads
            .values()
            .last()
            .expect("missing via pad shape")
            .shape;

        for (netname, net) in pcb.networks.iter() {
            by_net.insert(netname.clone(), Vec::new());

            for ident in net.pins.iter() {
                ident_to_net.insert(ident.clone(), netname.clone());
            }
        }

        // terminals with no net are obstacles, as are any explicit
        // keepout zones designated by the pcb.
        let mut obstacles = Vec::new();
        let mut boundary_obstacles = Vec::new();
        let mut all_pads = Vec::new();
        for shape in pcb.structure.keepout.iter() {
            boundary_obstacles.push(Arc::new(Terminal {
                identifier: None,
                net_name: None,
                layers: all_layers.clone(),
                shape: shape.shape.clone(),
                point: shape.shape.aabb().center(),
            }));
        }

        // Now build the terminals from the pins in the components
        for comp in pcb.components.iter() {
            let def = &pcb.component_defs[&comp.component_type];

            for out in def.keepout.iter() {
                let term = Arc::new(Terminal {
                    identifier: None,
                    net_name: None,
                    layers: all_layers.clone(),
                    shape: out.shape.translate(&comp.position),
                    point: out.shape.aabb().center(),
                });
                all_pads.push(Arc::clone(&term));
                obstacles.push(term);
            }

            for pin in def.pins.iter() {
                let pad_def = &pcb.pad_defs[&pin.pad_type];
                let mut terminal: Option<Terminal> = None;
                let identifier = format!("{}-{}", comp.instance_name, pin.pad_num);
                let net_name = ident_to_net.get(&identifier).cloned();
                let x = comp.position * pin.position;

                for (layer_name, pad_shape) in pad_def.pads.iter() {
                    let s = pad_shape.shape.translate(&x);

                    let mut layers = LayerSet::new();
                    layers.insert(layer_name_to_index[layer_name]);

                    let mut t = Terminal {
                        identifier: Some(identifier.clone()),
                        net_name: net_name.clone(),
                        layers,
                        point: s.aabb().center(),
                        shape: s,
                    };

                    if let Some(existing) = terminal {
                        t.merge(&existing);
                    }
                    terminal = Some(t);
                }

                if let Some(t) = terminal {
                    if let Some(net) = net_name {
                        if let Some(v) = by_net.get_mut(&net) {
                            let term = Arc::new(t);
                            all_pads.push(Arc::clone(&term));
                            v.push(term);
                        }
                    } else {
                        let term = Arc::new(t);
                        all_pads.push(Arc::clone(&term));
                        obstacles.push(term);
                    }
                }
            }
        }

        let mut twonets_by_net = HashMap::new();
        for (netname, mut terminals) in by_net.iter_mut() {
            let tnets = twonets::compute_2nets(
                netname,
                &mut terminals,
                &all_layers,
                &all_pads,
                pcb.structure.rule.clearance,
                &via_shape,
            );
            twonets_by_net.insert(netname.clone(), tnets);
        }

        // Merge all the obstacles together
        obstacles.extend(boundary_obstacles);

        Features {
            terminals_by_net: by_net,
            obstacles,
            all_pads,
            twonets_by_net,
            all_layers,
            paths_by_layer: HashMap::new(),
            cdt_edges: Vec::new(),
            via_shape: via_shape.clone(),
        }
    }
}
