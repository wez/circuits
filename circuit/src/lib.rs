use geo::prelude::*;
use geo::{Geometry, GeometryCollection, MultiPolygon, Polygon, Rect};
use kicad_parse_gen::footprint::{
    Layer as FootprintLayer, LayerSide, LayerType as FPLayerType, Module, Net as KicadFootprintNet,
    NetName, Xy, XyType,
};
use kicad_parse_gen::layout::{
    Element, General, GrLine, Host, Layer, LayerType, Layout, Net as KicadNet, Setup,
};
use kicad_parse_gen::Adjust;
use petgraph::prelude::*;
use std::collections::HashMap;
use std::sync::Arc;

mod component;
mod net;
mod pin;

pub mod components;
pub mod footprint;
pub mod point;

pub use component::*;
pub use net::Net;
pub use pin::*;

use crate::footprint::{AssignComponentNet, HasLocation, LayerManipulation, ToGeom};
use crate::point::{Point, Rotation};

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
    pub fn new<S: Into<String>>(name: S, value: Option<String>, component: Arc<Component>) -> Self {
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
        let mut footprint = self.component.footprint.clone();
        footprint.set_location(self.coordinates, self.rotation);
        if self.flipped {
            footprint.flip_layer();
        }
        footprint
    }

    pub fn hull(&self) -> Polygon<f64> {
        convex_hull(self.make_footprint().to_geom())
    }
}

/// Convert a collection into a vector of its constituent polygons
fn iter_polygons(collection: GeometryCollection<f64>) -> Vec<Polygon<f64>> {
    let mut polys = vec![];
    for geom in collection {
        match geom {
            Geometry::GeometryCollection(collection) => {
                polys.append(&mut iter_polygons(collection));
            }
            Geometry::Line(line) => {
                polys.push(Polygon::new(vec![line.start, line.end].into(), vec![]))
            }
            Geometry::Polygon(poly) => polys.push(poly),
            _ => {
                println!("BOO: {:?}", geom);
            }
        }
    }
    polys
}

fn convex_hull_collection(collection: GeometryCollection<f64>) -> Polygon<f64> {
    let multi: MultiPolygon<f64> = iter_polygons(collection).into_iter().collect();
    multi.convex_hull()
}

fn convex_hull(geom: Geometry<f64>) -> Polygon<f64> {
    match geom {
        Geometry::GeometryCollection(collection) => convex_hull_collection(collection),
        _ => panic!("only GeometryCollection is supported"),
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

/// Configure the setup for the pcb to work within the constraints
/// of Seeed Fusion.
/// http://support.seeedstudio.com/knowledgebase/articles/447362-fusion-pcb-specification
pub fn apply_seeed_drc(layout: &mut Layout) {
    use kicad_parse_gen::layout::SetupElement;

    macro_rules! ele {
        ($item:ident, $value:expr) => {
            layout.setup.elements.push(SetupElement {
                name: stringify!($item).to_string(),
                value1: stringify!($value).to_string(),
                value2: None,
            });
        };

        ($item:ident, $value:expr, $value2:expr) => {
            layout.setup.elements.push(SetupElement {
                name: stringify!($item).to_string(),
                value1: stringify!($value).to_string(),
                value2: Some(stringify!($value2).to_string()),
            });
        };
    }

    ele!(last_trace_width, 0.153);
    ele!(trace_clearance, 0.153);
    ele!(zone_clearance, 0.153);
    ele!(zone_45_only, no);
    ele!(trace_min, 0.153);
    ele!(segment_width, 0.153);
    ele!(edge_width, 0.15);
    ele!(via_size, 0.6);
    ele!(via_drill, 0.3);
    ele!(via_min_size, 0.6);
    ele!(via_min_drill, 0.3);
    ele!(uvia_size, 0.3);
    ele!(uvia_drill, 0.1);
    ele!(uvias_allowed, no);
    ele!(uvia_min_size, 0.2);
    ele!(uvia_min_drill, 0.1);
    ele!(pcb_text_width, 0.3);
    ele!(pcb_text_size, 1.5, 1.5);
    ele!(mod_edge_width, 0.15);
    ele!(mod_text_size, 1, 1);
    ele!(mod_text_width, 0.15);
    ele!(pad_size, 1.524, 1.524);
    ele!(pad_drill, 0.762);
    ele!(pad_to_mask_clearance, 0.2);
    ele!(aux_axis_origin, 0, 0);

    macro_rules! plot {
        ($item:ident, $value:expr) => {
            layout.setup.pcbplotparams.push(SetupElement {
                name: stringify!($item).to_string(),
                value1: stringify!($value).to_string(),
                value2: None,
            });
        };

        ($item:ident, $value:expr, $value2:expr) => {
            layout.setup.pcbplotparams.push(SetupElement {
                name: stringify!($item).to_string(),
                value1: stringify!($value).to_string(),
                value2: Some(stringify!($value2).to_string()),
            });
        };
    }

    plot!(layerselection, 0x010f0_ffffffff);
    plot!(usegerberextensions, true);
    plot!(usegerberattributes, false);
    plot!(usegerberadvancedattributes, false);
    plot!(creategerberjobfile, false);
    plot!(excludeedgelayer, true);
    plot!(linewidth, 0.100000);
    plot!(plotframeref, false);
    plot!(viasonmask, false);
    plot!(mode, 1);
    plot!(useauxorigin, false);
    plot!(hpglpennumber, 1);
    plot!(hpglpenspeed, 20);
    plot!(hpglpendiameter, 15.000000);
    plot!(psnegative, false);
    plot!(psa4output, false);
    plot!(plotreference, true);
    plot!(plotvalue, true);
    plot!(plotinvisibletext, false);
    plot!(padsonsilk, false);
    plot!(subtractmaskfromsilk, false);
    plot!(outputformat, 1);
    plot!(mirror, false);
    plot!(drillshape, 0);
    plot!(scaleselection, 1);

    // Set up the netclass
    let nets = layout
        .elements
        .iter()
        .filter_map(|element| match element {
            Element::Net(net) if net.name.0 != "" => Some(net.name.clone()),
            _ => None,
        })
        .collect();

    use kicad_parse_gen::layout::NetClass;

    layout.elements.push(Element::NetClass(NetClass {
        name: "Default".into(),
        desc: "This is the default net class.".into(),
        clearance: 0.153,
        trace_width: 0.25,
        via_dia: 0.6,
        via_drill: 0.3,
        uvia_dia: 0.3,
        uvia_drill: 0.1,
        diff_pair_gap: None,
        diff_pair_width: None,
        nets,
    }));
}

impl Circuit {
    pub fn to_pcb_layout(&self) -> Layout {
        let mut elements = Vec::new();

        let mut net_to_idx = HashMap::new();

        // Reserve net id 0; kicad doesn't like it when a module
        // references id zero, so let's make sure that we don't use
        // it for anything in here.
        elements.push(Element::Net(KicadNet {
            num: 0,
            name: NetName("".into()),
        }));

        for (net_idx, net) in self.nets.iter().enumerate() {
            net_to_idx.insert(net.clone(), net_idx + 1);

            elements.push(Element::Net(KicadNet {
                num: 1 + net_idx as i64,
                name: NetName(net.name.clone()),
            }));
        }

        for inst in &self.instances {
            let mut footprint = inst.make_footprint();

            // assign pads to nets
            for ass in &inst.assignments {
                let net_idx = &net_to_idx[&ass.net];
                let net = KicadFootprintNet {
                    name: NetName(ass.net.name.clone()),
                    num: *net_idx as i64,
                };

                footprint.set_net(
                    &inst.component.pins[ass.pin_index].name,
                    ass.pin_index,
                    &net,
                );
            }

            elements.push(Element::Module(footprint));
        }

        let mut layout = Layout {
            host: Host {
                tool: "rust circuit crate".into(),
                build: "0.1.0".into(),
            },
            general: General {
                thickness: 1.6,
                drawings: 0,
                tracks: 0,
                zones: 0,
                modules: self.instances.len() as i64,
                nets: 1 + net_to_idx.len() as i64,
            },
            page: "USLetter".into(),
            setup: Setup {
                elements: vec![],
                pcbplotparams: vec![],
            },
            layers: vec![
                Layer {
                    num: 0,
                    layer: FootprintLayer {
                        side: LayerSide::Front,
                        t: FPLayerType::Cu,
                    },
                    layer_type: LayerType::Signal,
                    hide: false,
                },
                Layer {
                    num: 1,
                    layer: FootprintLayer {
                        side: LayerSide::Back,
                        t: FPLayerType::Cu,
                    },
                    layer_type: LayerType::Signal,
                    hide: false,
                },
                Layer {
                    num: 2,
                    layer: FootprintLayer {
                        side: LayerSide::Back,
                        t: FPLayerType::Adhes,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 3,
                    layer: FootprintLayer {
                        side: LayerSide::Front,
                        t: FPLayerType::Adhes,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 4,
                    layer: FootprintLayer {
                        side: LayerSide::Back,
                        t: FPLayerType::Paste,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 5,
                    layer: FootprintLayer {
                        side: LayerSide::Front,
                        t: FPLayerType::Paste,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 6,
                    layer: FootprintLayer {
                        side: LayerSide::Back,
                        t: FPLayerType::SilkS,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 7,
                    layer: FootprintLayer {
                        side: LayerSide::Front,
                        t: FPLayerType::SilkS,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 8,
                    layer: FootprintLayer {
                        side: LayerSide::Back,
                        t: FPLayerType::Mask,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 9,
                    layer: FootprintLayer {
                        side: LayerSide::Front,
                        t: FPLayerType::Mask,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 10,
                    layer: FootprintLayer {
                        side: LayerSide::Dwgs,
                        t: FPLayerType::User,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 11,
                    layer: FootprintLayer {
                        side: LayerSide::Cmts,
                        t: FPLayerType::User,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 12,
                    layer: FootprintLayer {
                        side: LayerSide::Eco1,
                        t: FPLayerType::User,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 13,
                    layer: FootprintLayer {
                        side: LayerSide::Eco2,
                        t: FPLayerType::User,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 14,
                    layer: FootprintLayer {
                        side: LayerSide::Edge,
                        t: FPLayerType::Cuts,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 15,
                    layer: FootprintLayer {
                        side: LayerSide::None,
                        t: FPLayerType::Margin,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 16,
                    layer: FootprintLayer {
                        side: LayerSide::Back,
                        t: FPLayerType::CrtYd,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 17,
                    layer: FootprintLayer {
                        side: LayerSide::Front,
                        t: FPLayerType::CrtYd,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 18,
                    layer: FootprintLayer {
                        side: LayerSide::Back,
                        t: FPLayerType::Fab,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
                Layer {
                    num: 19,
                    layer: FootprintLayer {
                        side: LayerSide::Front,
                        t: FPLayerType::Fab,
                    },
                    layer_type: LayerType::User,
                    hide: false,
                },
            ],
            elements,
            version: 2017_1130,
        };

        apply_seeed_drc(&mut layout);
        let hull = compute_hull(&layout);

        for line in hull.exterior().lines() {
            layout.elements.push(Element::GrLine(GrLine {
                start: Xy {
                    x: line.start.x,
                    y: line.start.y,
                    t: XyType::Start,
                },
                end: Xy {
                    x: line.end.x,
                    y: line.end.y,
                    t: XyType::End,
                },
                angle: 0.0,
                layer: FootprintLayer {
                    side: LayerSide::Edge,
                    t: FPLayerType::Cuts,
                },
                width: 0.15,
                tstamp: None,
            }));
        }

        // TODO: add area fills for GND and 3V3 nets

        adjust_to_fit_page(&mut layout);

        layout
    }
}

fn layout_to_geom_collection(layout: &Layout) -> GeometryCollection<f64> {
    layout
        .elements
        .iter()
        .filter_map(|ele| match ele {
            Element::Module(module) => Some(module.to_geom()),
            _ => None,
        })
        .collect()
}

fn compute_bounding_box(layout: &Layout) -> Rect<f64> {
    let collection = layout_to_geom_collection(layout);
    let multi: MultiPolygon<f64> = iter_polygons(collection).into_iter().collect();
    multi
        .bounding_rect()
        .expect("unable to compute bounding rect for layout")
}

/// Try to place the content tidily in the sheet.
fn adjust_to_fit_page(layout: &mut Layout) {
    // US Letter dimensions
    const LETTER_WIDTH: f64 = 280.0;
    const LETTER_HEIGHT: f64 = 218.0;

    let bounds = compute_bounding_box(layout);

    let width = bounds.max.x - bounds.min.x;
    let height = bounds.max.y - bounds.min.y;

    // Center it
    let x_off = (LETTER_WIDTH - width) / 2.0;
    let y_off = (LETTER_HEIGHT - height) / 2.0;

    layout.adjust(x_off - bounds.min.x, y_off - bounds.min.y);
}

fn compute_hull(layout: &Layout) -> Polygon<f64> {
    let collection = layout_to_geom_collection(layout);
    convex_hull_collection(collection)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::*;

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
