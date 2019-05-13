use crate::footprint::AssignComponentNet;
use crate::layout::*;
use crate::{Inst, Net, PinDrive, PinType};
use itertools::Itertools;
use kicad_parse_gen::footprint::{
    Layer as FootprintLayer, LayerSide, LayerType as FPLayerType, Net as KicadFootprintNet,
    NetName, Xy, XyType,
};
use kicad_parse_gen::layout::{
    Element, General, GrLine, Host, Layer, LayerType, Layout, Net as KicadNet, Setup,
};
use std::collections::{HashMap, HashSet};

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

#[derive(Debug, PartialEq, Eq)]
pub enum ErcItem {
    Warning(String),
    Error(String),
}

impl ErcItem {
    pub fn error(e: &str) -> Self {
        ErcItem::Error(e.to_owned())
    }
}

impl Circuit {
    pub fn eletrical_rules_check(&self) -> Vec<ErcItem> {
        let mut problems = vec![];

        #[derive(Debug, Clone)]
        struct InstPinAssignment {
            inst_idx: usize,
            pin_idx: usize,
            pin_type: PinType,
        }

        let mut assignments_by_net = HashMap::new();

        let not_connected_net = Net::new();

        for (inst_idx, inst) in self.instances.iter().enumerate() {
            let mut not_connected_pins = HashSet::new();
            for pin_idx in 0..inst.component.pins().len() {
                not_connected_pins.insert(pin_idx);
            }
            for pin_assign in inst.assignments() {
                let pin_type = inst.component.pins()[pin_assign.pin_index].pin_type;

                not_connected_pins.remove(&pin_assign.pin_index);

                assignments_by_net
                    .entry(&pin_assign.net)
                    .or_insert_with(|| vec![])
                    .push(InstPinAssignment {
                        inst_idx,
                        pin_idx: pin_assign.pin_index,
                        pin_type,
                    });
            }

            for pin_idx in not_connected_pins.into_iter() {
                let pin_type = inst.component.pins()[pin_idx].pin_type;
                assignments_by_net
                    .entry(&not_connected_net)
                    .or_insert_with(|| vec![])
                    .push(InstPinAssignment {
                        inst_idx,
                        pin_idx,
                        pin_type,
                    });
            }
        }

        for (net, assignments) in assignments_by_net.into_iter() {
            let net_drive = assignments
                .iter()
                .map(|a| a.pin_type.drive_strength())
                .max()
                .unwrap_or(PinDrive::None);

            // Exclude our special not_connected_net as the "assignments"
            // are not actually connected together
            if *net != not_connected_net {
                for pins in assignments.iter().combinations(2) {
                    let a = pins[0];
                    let b = pins[1];

                    for (output_pin, input_pin) in &[(&a, &b), (&b, &a)] {
                        let inst = &self.instances[input_pin.inst_idx];
                        let drive = output_pin.pin_type.drive_strength();
                        let min_drive = input_pin.pin_type.min_input_drive();
                        let max_drive = input_pin.pin_type.min_input_drive();
                        if drive < min_drive {
                            problems.push(ErcItem::Error(format!(
                                "Net `{}`: drive `{:?}` is below the minimum allowed drive \
                                 `{:?}` for component `{}` pin {} {:?}",
                                net,
                                drive,
                                min_drive,
                                inst.name,
                                input_pin.pin_idx,
                                input_pin.pin_type
                            )));
                        }
                        if drive > max_drive {
                            problems.push(ErcItem::Error(format!(
                                "Net `{}`: drive `{:?}` is above the maximum allowed drive \
                                 `{:?}` for component `{}` pin {} {:?}",
                                net,
                                drive,
                                max_drive,
                                inst.name,
                                input_pin.pin_idx,
                                input_pin.pin_type
                            )));
                        }
                    }

                    if a.pin_type.incompatible_pin_pairing(b.pin_type)
                        || b.pin_type.incompatible_pin_pairing(a.pin_type)
                    {
                        let a_inst = &self.instances[a.inst_idx];
                        let b_inst = &self.instances[b.inst_idx];
                        problems.push(ErcItem::Error(format!(
                            "Net `{}`: Incompatible pins are connected on the same net; \
                             component `{}` pin {} {:?} and component `{}` pin {} {:?}",
                            net,
                            a_inst.name,
                            a.pin_idx,
                            a.pin_type,
                            b_inst.name,
                            b.pin_idx,
                            b.pin_type
                        )));
                    }
                }
            }

            // Now make a pass to check the minimum drive.
            // This case is largely to catch eg: a missing
            // or unconnected power supply.
            for assignment in &assignments {
                let inst = &self.instances[assignment.inst_idx];
                let min_drive = assignment.pin_type.min_input_drive();
                if net_drive < min_drive {
                    problems.push(ErcItem::Error(format!(
                        "Net `{}`: drive `{:?}` is below the minimum allowed drive \
                         `{:?}` for component `{}` pin {} {:?}{}",
                        net,
                        net_drive,
                        min_drive,
                        inst.name,
                        assignment.pin_idx,
                        assignment.pin_type,
                        if min_drive == PinDrive::Power {
                            " (Do you need to connect a pwr_flag() to this net?)"
                        } else {
                            ""
                        }
                    )));
                }
            }
        }

        problems
    }

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
            if let Some(mut footprint) = inst.make_footprint() {
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
