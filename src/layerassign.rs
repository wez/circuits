use petgraph::graphmap::DiGraphMap;
use features::{LayerSet, Terminal};
use geom;
use std::collections::HashMap;
extern crate nalgebra as na;
use std::sync::Arc;

pub type TerminalId = usize;
const NUM_VIAS: usize = 3;

#[derive(Debug, Clone)]
pub struct TerminalInfo {
    configured_layers: LayerSet,
    // id we use for the graph key
    terminal_id: TerminalId,
}

#[derive(Debug, Eq, PartialEq, Hash, Clone, Copy, Ord, PartialOrd)]
pub struct TerminalOnLayer {
    terminal_id: TerminalId,
    layer: u8,
}

impl TerminalOnLayer {
    fn new(terminal_id: TerminalId, layer: usize) -> TerminalOnLayer {
        TerminalOnLayer {
            terminal_id: terminal_id,
            layer: layer as u8,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Hash, Clone, Copy, Ord, PartialOrd)]
pub enum Node {
    Source(TerminalId),
    Sink(TerminalId),
    Terminal(TerminalOnLayer),
    Via(TerminalOnLayer),
}

impl Node {
    fn src(terminal_id: TerminalId) -> Node {
        Node::Source(terminal_id)
    }

    fn sink(terminal_id: TerminalId) -> Node {
        Node::Sink(terminal_id)
    }

    fn terminal(terminal_id: TerminalId, layer: usize) -> Node {
        Node::Terminal(TerminalOnLayer::new(terminal_id, layer))
    }

    fn via(terminal_id: TerminalId, layer: usize) -> Node {
        Node::Via(TerminalOnLayer::new(terminal_id, layer))
    }
}

pub type AssignGraphMap = DiGraphMap<Node, f64>;

#[derive(Debug, Clone)]
struct TwoNet {
    graph: AssignGraphMap,
}

#[derive(Debug, Clone, Default)]
pub struct Configuration {
    // map constant terminal info to the mutable (within
    // this configuration instance) node info.
    terminals: HashMap<Arc<Terminal>, TerminalInfo>,
    terminal_by_id: HashMap<TerminalId, Arc<Terminal>>,
    two_nets: Vec<TwoNet>,
}

impl Configuration {
    // add or resolve a terminal to its TerminalId
    fn add_terminal(&mut self, terminal: &Arc<Terminal>) -> TerminalId {
        if let Some(info) = self.terminals.get(terminal) {
            return info.terminal_id;
        }

        let id = self.terminal_by_id.len();
        self.terminals
            .insert(Arc::clone(terminal),
                    TerminalInfo {
                        configured_layers: LayerSet::default(),
                        terminal_id: id,
                    });

        self.terminal_by_id.insert(id, Arc::clone(terminal));

        id
    }

    pub fn add_twonet(&mut self, a: &Arc<Terminal>, b: &Arc<Terminal>, all_layers: &LayerSet) {
        let mut g = AssignGraphMap::new();

        let a_id = self.add_terminal(a);
        let b_id = self.add_terminal(b);

        // We build a graph of potential paths:
        //
        //         /---- v1 --- v2 --- v3 ----\     layer 1
        //   source      |      |      |      sink
        //         \---- v1 --- v2 --- v3 ----/     layer 0

        let src_point = a.point;
        let sink_point = a.point;

        let x_delta = (sink_point.coords.x - src_point.coords.x).abs() / NUM_VIAS as f64;
        let y_delta = (sink_point.coords.y - src_point.coords.y).abs() / NUM_VIAS as f64;

        // Build up the via nodes on each layer.
        let mut via_terminals = Vec::new();
        let mut via_points = Vec::new();
        for i in 0..NUM_VIAS {
            let pt = geom::Point::new(src_point.coords.x + x_delta * i as f64,
                                      src_point.coords.y + y_delta * i as f64);
            via_points.push(pt.clone());

            let via_shape =
                geom::Shape::circle(200.0,
                                    geom::Location::new(geom::Vector::new(pt.coords.x,
                                                                          pt.coords.y),
                                                        0.0));
            let terminal = Arc::new(Terminal {
                                        identifier: None,
                                        net_name: a.net_name.clone(),
                                        layers: all_layers.clone(),
                                        shape: via_shape,
                                        point: pt.clone(),
                                    });
            let id = self.add_terminal(&terminal);

            via_terminals.push(id);

            for layer in 0..all_layers.len() {
                g.add_node(Node::via(id, layer));
            }
        }

        // Build up/down connectivity between the vias
        for i in 0..NUM_VIAS {
            for layer in 0..all_layers.len() - 1 {
                let via_terminal = via_terminals[i];
                g.add_edge(Node::via(via_terminal, layer),
                           Node::via(via_terminal, layer + 1),
                           0.0);
            }
        }

        // Build left/right connectivity between the vias
        for i in 1..NUM_VIAS {
            for layer in 0..all_layers.len() {
                let left_term = via_terminals[i - 1];
                let right_term = via_terminals[i];
                let cost = na::distance(&via_points[i - 1], &via_points[i]);

                g.add_edge(Node::via(left_term, layer),
                           Node::via(right_term, layer),
                           cost);
            }
        }

        // Build source/sink connectivity
        g.add_node(Node::src(a_id));
        g.add_node(Node::sink(b_id));

        for layer in 0..all_layers.len() {
            g.add_edge(Node::src(a_id), Node::terminal(a_id, layer), 0.0);

            g.add_edge(Node::terminal(a_id, layer),
                       Node::via(via_terminals[0], layer),
                       na::distance(&src_point, &via_points[0]));

            g.add_edge(Node::via(via_terminals[NUM_VIAS - 1], layer),
                       Node::terminal(b_id, layer),
                       na::distance(&sink_point, &via_points[NUM_VIAS - 1]));

            g.add_edge(Node::terminal(a_id, layer), Node::sink(b_id), 0.0);
        }

        self.two_nets.push(TwoNet { graph: g });
    }
}
