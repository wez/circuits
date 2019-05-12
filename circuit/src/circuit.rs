use crate::footprint::AssignComponentNet;
use crate::layout::*;
use crate::{Inst, Net};
use kicad_parse_gen::footprint::{
    Layer as FootprintLayer, LayerSide, LayerType as FPLayerType, Net as KicadFootprintNet,
    NetName, Xy, XyType,
};
use kicad_parse_gen::layout::{
    Element, General, GrLine, Host, Layer, LayerType, Layout, Net as KicadNet, Setup,
};
use std::collections::HashMap;

#[derive(PartialEq, Eq, Debug, Default)]
pub struct CircuitBuilder {
    instances: Vec<Inst>,
}

impl CircuitBuilder {
    pub fn add_inst(&mut self, inst: Inst) {
        self.instances.push(inst);
    }

    pub fn build(self) -> Circuit {
        let (instances, nets) = Inst::merge_connected_nets(self.instances);
        Circuit { instances, nets }
    }
}

#[derive(PartialEq, Eq, Debug, Default)]
pub struct Circuit {
    pub instances: Vec<Inst>,
    pub nets: Vec<Net>,
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
            for ass in inst.assignments() {
                let net_idx = &net_to_idx[&ass.net];
                let net = KicadFootprintNet {
                    name: NetName(ass.net.name.clone()),
                    num: *net_idx as i64,
                };

                footprint.set_net(
                    &inst.component.pins()[ass.pin_index].name,
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
