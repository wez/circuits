use dsn;
use geom;
use std::collections::{HashMap, HashSet};

#[derive(Debug)]
pub struct Terminal {
    // name of this terminal (assuming it is a pin on a component)
    pub identifier: Option<String>,

    // name of the net to which this terminal is connected
    pub net_name: Option<String>,

    // which layers this terminal is connected to
    pub layers: HashSet<String>,

    // footprint of the terminal
    pub shape: geom::Shape,
}

#[derive(Debug)]
pub struct Features {
    pub terminals_by_net: HashMap<String, Vec<Terminal>>,
    pub obstacles: Vec<Terminal>,
}

impl Terminal {
    fn merge(&mut self, other: &Self) {
        // TODO: this may only really be valid if the pads are identical,
        // which should be the case for our simple keyboard circuits
        self.layers = self.layers.union(&other.layers).cloned().collect();
    }
}

impl Features {
    // Extract a list of terminals from a pcb
    pub fn from_pcb(pcb: &dsn::Pcb) -> Features {
        // First build the map of ident -> netname by inverting the
        // mapping described by the pcb structure
        let mut ident_to_net = HashMap::new();
        let mut by_net: HashMap<String, Vec<Terminal>> = HashMap::new();

        for (netname, net) in pcb.networks.iter() {
            by_net.insert(netname.clone(), Vec::new());

            for ident in net.pins.iter() {
                ident_to_net.insert(ident.clone(), netname.clone());
            }
        }

        // terminals with no net are obstacles, as are any explicit
        // keepout zones designated by the pcb.
        let mut obstacles: Vec<Terminal> = Vec::new();

        for shape in pcb.structure.keepout.iter() {
            obstacles.push(Terminal {
                               identifier: None,
                               net_name: None,
                               layers: HashSet::new(), // FIXME: all layers?
                               shape: shape.shape.clone(),
                           });
        }

        // Now build the terminals from the pins in the components
        for comp in pcb.components.iter() {
            let def = &pcb.component_defs[&comp.component_type];

            for out in def.keepout.iter() {
                obstacles.push(Terminal {
                                   identifier: None,
                                   net_name: None,
                                   layers: HashSet::new(), // FIXME: all layers?
                                   shape: out.shape.translate(&comp.position),
                               });
            }

            for pin in def.pins.iter() {
                let pad_def = &pcb.pad_defs[&pin.pad_type];
                let mut terminal: Option<Terminal> = None;
                let identifier = format!("{}-{}", comp.instance_name, pin.pad_num);
                let net_name = ident_to_net.get(&identifier).map(|x| x.clone());
                let x = comp.position * pin.position;

                for (layer_name, pad_shape) in pad_def.pads.iter() {
                    let s = pad_shape.shape.translate(&x);

                    let mut layers = HashSet::new();
                    layers.insert(layer_name.clone());

                    let mut t = Terminal {
                        identifier: Some(identifier.clone()),
                        net_name: net_name.clone(),
                        layers: layers,
                        shape: s,
                    };

                    if let Some(existing) = terminal {
                        t.merge(&existing);
                    }
                    terminal = Some(t);
                }

                if terminal.is_some() {
                    let t = terminal.unwrap();
                    if let Some(net) = net_name {
                        if let Some(v) = by_net.get_mut(&net) {
                            v.push(t);
                        }
                    } else {
                        obstacles.push(t);
                    }
                }
            }
        }

        Features {
            terminals_by_net: by_net,
            obstacles: obstacles,
        }
    }
}
