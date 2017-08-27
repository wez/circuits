use petgraph::graphmap::DiGraphMap;
use features::{LayerSet, Terminal};
use geom::{Location, Shape, Point, Vector, origin};
use geo::convexhull::ConvexHull;
use geo::intersects::Intersects;
use std::collections::{HashSet, HashMap};
use ncollide::broad_phase::BroadPhase;
use ncollide::broad_phase::DBVTBroadPhase;
use ncollide::bounding_volume::BoundingVolume;
extern crate ncollide;
use ncollide::query;
extern crate nalgebra as na;
use std::sync::Arc;
use std::{thread, time};
use dijkstra::shortest_path;

use progress::Progress;

pub type TerminalId = usize;
const NUM_VIAS: usize = 3;
const ALPHA: f64 = 0.1;

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

    fn as_terminal_on_layer(&self) -> Option<TerminalOnLayer> {
        match self {
            &Node::Terminal(tol) => Some(tol),
            &Node::Via(tol) => Some(tol),
            _ => None,
        }
    }
}

// a set of connected paths on a layer form a component.
// We use components to estimate the detour cost of a path
// intersecting a component.  The detour cost is based on
// the path around the convex hull of the paths.
#[derive(Debug, Clone)]
pub struct Component {
    pub terminals: HashSet<Arc<Terminal>>,
    pub paths: Vec<(Arc<Terminal>, Arc<Terminal>, Shape)>,
    pub hull: Shape,
}

pub struct ComponentPath {
    pub line: Shape,
    pub terminal: Arc<Terminal>,
}

pub type ComponentBroadPhase = DBVTBroadPhase<Point,
                                              ncollide::bounding_volume::AABB<Point>,
                                              ComponentPath>;

pub type ComponentIndex = usize;
pub struct ComponentList {
    pub components: Vec<Component>,
    pub terminal_to_component: HashMap<Arc<Terminal>, ComponentIndex>,
    pub broad_phase: ComponentBroadPhase,
    next_phase_id: usize,
}

fn same_object<T>(a: *const T, b: *const T) -> bool {
    a == b
}

impl ::std::fmt::Debug for ComponentList {
    fn fmt(&self, fmt: &mut ::std::fmt::Formatter) -> ::std::result::Result<(), ::std::fmt::Error> {
        write!(fmt,
               "ComponentList with {} components",
               self.components.len())?;
        Ok(())
    }
}

impl ComponentList {
    fn new() -> ComponentList {
        ComponentList {
            terminal_to_component: HashMap::new(),
            components: Vec::new(),
            broad_phase: ComponentBroadPhase::new(1.0, false),
            next_phase_id: 0,
        }

    }

    fn intersects(&self,
                  edge: &PathEdge)
                  -> Option<Vec<(Arc<Terminal>, ncollide::query::Contact<Point>)>> {
        if let Some((ref line, ref bv)) = edge.line {
            let mut candidates = Vec::new();
            self.broad_phase
                .interferences_with_bounding_volume(bv, &mut candidates);

            return Some(candidates
                            .into_iter()
                            .filter_map(|path| if let Some(contact) = path.line
                                               .contact(line) {
                                            Some((Arc::clone(&path.terminal), contact))
                                        } else {
                                            None
                                        })
                            .collect());
        }

        None
    }

    // Returns the largest of the two components first.
    // If both components are the same, returns (comp, None)
    // If either component is None, returns (comp, None)
    fn biggest_component(&self,
                         ta: &Arc<Terminal>,
                         tb: &Arc<Terminal>)
                         -> (Option<ComponentIndex>, Option<ComponentIndex>) {
        let a_idx = self.terminal_to_component.get(ta).map(|x| *x);
        let b_idx = self.terminal_to_component.get(tb).map(|x| *x);

        if a_idx.is_none() {
            (b_idx, None)
        } else if b_idx.is_none() {
            (a_idx, None)
        } else {
            let a_idx = a_idx.expect("we checked this above!?");
            let b_idx = b_idx.expect("we checked this above!?");

            if a_idx == b_idx {
                (Some(a_idx), None)
            } else {
                let ref a = self.components[a_idx];
                let ref b = self.components[b_idx];

                let a_len = a.terminals.len();
                let b_len = b.terminals.len();
                if a_len > b_len {
                    (Some(a_idx), Some(b_idx))
                } else if a_len < b_len {
                    (Some(b_idx), Some(a_idx))
                } else {
                    (Some(a_idx), Some(b_idx))
                }
            }
        }
    }


    fn add_path(&mut self, a: &Arc<Terminal>, b: &Arc<Terminal>) {
        // Add the path between two terminals.
        // First let's see if we're adding to an existing component,
        // or perhaps connecting two existing components together.

        let (target_idx, src_idx) = self.biggest_component(a, b);

        let line = Shape::line(&a.point, &b.point);

        self.broad_phase
            .deferred_add(self.next_phase_id,
                          line.aabb(),
                          ComponentPath {
                              line: line.clone(),
                              terminal: Arc::clone(a),
                          });
        self.next_phase_id = self.next_phase_id + 1;

        if let Some(target_idx) = target_idx {
            if let Some(src_idx) = src_idx {
                // Merge src_comp into target_comp
                let mut src_comp = self.components[src_idx].clone();
                let ref mut target_comp = self.components[target_idx];

                for t in src_comp.terminals.iter() {
                    self.terminal_to_component
                        .insert(Arc::clone(t), target_idx);
                }

                target_comp.terminals.extend(src_comp.terminals.drain());
                target_comp.paths.extend(src_comp.paths.drain(..));
            }

            // Add to target component
            self.terminal_to_component
                .insert(Arc::clone(a), target_idx);
            self.terminal_to_component
                .insert(Arc::clone(b), target_idx);

            let ref mut target_comp = self.components[target_idx];
            target_comp.terminals.insert(Arc::clone(a));
            target_comp.terminals.insert(Arc::clone(b));
            target_comp
                .paths
                .push((Arc::clone(a), Arc::clone(b), line.clone()));

            // and recompute the hull
            let lines: Vec<Shape> = target_comp
                .paths
                .iter()
                .map(|&(_, _, ref shape)| shape.clone())
                .collect();
            target_comp.hull = Shape::convex_hull(&lines);


        } else {
            // Create a new component
            let comp_idx = self.components.len();
            self.components
                .push(Component {
                          terminals: hashset!{Arc::clone(a), Arc::clone(b)},
                          paths: vec![(Arc::clone(a), Arc::clone(b), line.clone())],
                          hull: line,
                      });

            self.terminal_to_component
                .insert(Arc::clone(a), comp_idx);
            self.terminal_to_component
                .insert(Arc::clone(b), comp_idx);
        }
    }
}

#[derive(Debug, Clone)]
pub struct PathEdge {
    pub base_cost: f64,
    pub line: Option<(Shape, ncollide::bounding_volume::AABB<Point>)>,
}

impl PathEdge {
    fn from_points(a: &Point, b: &Point) -> PathEdge {
        let line = Shape::line(a, b);
        let bv = line.aabb();

        PathEdge {
            base_cost: na::distance(a, b),
            line: Some((line, bv)),
        }
    }

    fn zero() -> PathEdge {
        PathEdge {
            base_cost: 0.,
            line: None,
        }
    }
}



pub type AssignGraphMap = DiGraphMap<Node, PathEdge>;

#[derive(Debug, Clone)]
struct TwoNet {
    graph: AssignGraphMap,
    src: TerminalId,
    sink: TerminalId,
}
pub type TwoNetId = usize;

#[derive(Debug, Default)]
pub struct Configuration {
    // map constant terminal info to the mutable (within
    // this configuration instance) node info.
    terminals: HashMap<*const Terminal, TerminalInfo>,
    terminal_by_id: HashMap<TerminalId, Arc<Terminal>>,

    // Indices are referenced via TwoNetId
    two_nets: Vec<TwoNet>,
    all_layers: LayerSet,
    components: Vec<ComponentList>,

    assignment: Vec<(TwoNetId, Vec<Node>)>,
}

impl Configuration {
    pub fn new(all_layers: &LayerSet) -> Configuration {
        let mut cfg = Configuration::default();
        cfg.all_layers = all_layers.clone();
        for _ in 0..all_layers.len() {
            cfg.components.push(ComponentList::new());
        }
        cfg
    }

    // add or resolve a terminal to its TerminalId
    fn add_terminal(&mut self, terminal: &Arc<Terminal>) -> TerminalId {
        let raw: *const Terminal = &**terminal;
        use std::collections::hash_map::Entry::{Occupied, Vacant};
        match self.terminals.entry(raw) {
            Occupied(ent) => {
                return ent.get().terminal_id;
            }
            Vacant(ent) => {
                let id = self.terminal_by_id.len();
                ent.insert(TerminalInfo {
                               configured_layers: LayerSet::default(),
                               terminal_id: id,
                           });
                self.terminal_by_id.insert(id, Arc::clone(terminal));
                id
            }
        }
    }

    pub fn add_twonet(&mut self, a: &Arc<Terminal>, b: &Arc<Terminal>) {
        let mut g = AssignGraphMap::new();

        let a_id = self.add_terminal(a);
        let b_id = self.add_terminal(b);

        // We build a graph of potential paths:
        //
        //         /---- v1 --- v2 --- v3 ----\     layer 1
        //   source      |      |      |      sink
        //         \---- v1 --- v2 --- v3 ----/     layer 0

        let src_point = a.point;
        let sink_point = b.point;

        let x_delta = (sink_point.coords.x - src_point.coords.x).abs() / NUM_VIAS as f64;
        let y_delta = (sink_point.coords.y - src_point.coords.y).abs() / NUM_VIAS as f64;

        // Build up the via nodes on each layer.
        let mut via_terminals = Vec::new();
        let mut via_points = Vec::new();
        for i in 0..NUM_VIAS {
            let pt = Point::new(src_point.coords.x + x_delta * i as f64,
                                src_point.coords.y + y_delta * i as f64);
            via_points.push(pt.clone());

            let via_shape = Shape::circle(200.0,
                                          Location::new(Vector::new(pt.coords.x, pt.coords.y),
                                                        0.0));
            let terminal = Arc::new(Terminal {
                                        identifier: None,
                                        net_name: a.net_name.clone(),
                                        layers: self.all_layers.clone(),
                                        shape: via_shape,
                                        point: pt.clone(),
                                    });
            let id = self.add_terminal(&terminal);

            via_terminals.push(id);

            for layer in 0..self.all_layers.len() {
                g.add_node(Node::via(id, layer));
            }
        }

        // Build up/down connectivity between the vias
        for i in 0..NUM_VIAS {
            for layer in 0..self.all_layers.len() - 1 {
                let via_terminal = via_terminals[i];
                g.add_edge(Node::via(via_terminal, layer),
                           Node::via(via_terminal, layer + 1),
                           PathEdge::zero());
            }
        }

        // Build left/right connectivity between the vias
        for i in 1..NUM_VIAS {
            for layer in 0..self.all_layers.len() {
                let left_term = via_terminals[i - 1];
                let right_term = via_terminals[i];

                g.add_edge(Node::via(left_term, layer),
                           Node::via(right_term, layer),
                           PathEdge::from_points(&via_points[i - 1], &via_points[i]));
            }
        }

        // Build source/sink connectivity
        g.add_node(Node::src(a_id));
        g.add_node(Node::sink(b_id));

        for layer in 0..self.all_layers.len() {
            g.add_edge(Node::src(a_id),
                       Node::terminal(a_id, layer),
                       PathEdge::zero());

            g.add_edge(Node::terminal(a_id, layer),
                       Node::via(via_terminals[0], layer),
                       PathEdge::from_points(&src_point, &via_points[0]));

            g.add_edge(Node::via(via_terminals[NUM_VIAS - 1], layer),
                       Node::terminal(b_id, layer),
                       PathEdge::from_points(&sink_point, &via_points[NUM_VIAS - 1]));

            g.add_edge(Node::terminal(b_id, layer),
                       Node::sink(b_id),
                       PathEdge::zero());
        }

        self.two_nets
            .push(TwoNet {
                      graph: g,
                      src: a_id,
                      sink: b_id,
                  });
    }

    fn src_sink_cost(&self, src_sink: TerminalId, tol: &TerminalOnLayer) -> f64 {
        let term = &self.terminal_by_id[&tol.terminal_id];
        let raw: *const Terminal = &**term;
        let info = &self.terminals[&raw];

        if !term.layers.contains(&tol.layer) ||
           ((info.configured_layers.len() > 0 && !info.configured_layers.contains(&tol.layer))) {
            return ::std::f64::INFINITY;
        }
        return 0.0;
    }

    fn edge_cost(&self, a: &Node, b: &Node, edge: &PathEdge) -> f64 {
        if let &Node::Source(tid) = a {
            return self.src_sink_cost(tid, &b.as_terminal_on_layer().expect("b must be tol"));
        }
        if let &Node::Sink(tid) = b {
            return self.src_sink_cost(tid, &a.as_terminal_on_layer().expect("a must be tol"));
        }
        let a = a.as_terminal_on_layer().expect("a must be tol");
        let b = b.as_terminal_on_layer().expect("b must be tol");

        if a.layer == b.layer {
            // We may be colliding with something
            if let Some(contacts) = self.components[a.layer as usize].intersects(edge) {
                // TODO: compute detour cost
                (1.0 - ALPHA) * (edge.base_cost * 100.0)
            } else {
                (1.0 - ALPHA) * edge.base_cost
            }
        } else {
            // Moving between levels; impose a token cost to avoid
            // the algorithm from chosing to change levels for no reason.
            ALPHA
        }
    }

    fn assign_terminal_to_layer(&mut self, tol: &Option<TerminalOnLayer>) {
        if let &Some(tol) = tol {
            let t = &self.terminal_by_id[&tol.terminal_id];
            // We always populate self.terminals during initialization,
            // so this condition should always hold true.
            let raw: *const Terminal = &**t;
            if let Some(info) = self.terminals.get_mut(&raw) {
                info.configured_layers.insert(tol.layer);
            }
        }
    }

    pub fn initial_assignment(&mut self) {
        let pb = Progress::new("initial assignment", self.two_nets.len());

        let mut free_nets: HashSet<TwoNetId> = (0..self.two_nets.len()).collect();

        loop {
            if free_nets.len() == 0 {
                break;
            }

            pb.inc();
            let mut best = None;

            for i in free_nets.iter() {
                let netid: TwoNetId = *i;
                let ref twonet = self.two_nets[netid];

                if let Some((cost, path)) =
                    shortest_path(&twonet.graph,
                                  Node::src(twonet.src),
                                  Node::sink(twonet.sink),
                                  |(a, b, edge)| self.edge_cost(&a, &b, edge),
                                  best.as_ref().and_then(|&(cost, _, _)| Some(cost))) {
                    /*
                    println!("path:");
                    println!("cost: {}", cost);
                    println!("src: {:?}, sink: {:?}", twonet.src, twonet.sink);
                    println!("path: {:#?}", path);
                    println!("");
                    println!("");
                    */

                    if best.is_none() || cost < best.as_ref().map(|&(cost, _, _)| cost).unwrap() {
                        best = Some((cost, netid, path));
                    }
                }
            }

            let (cost, netid, path) = best.expect("must always be able to find the best");
            free_nets.remove(&netid);

            // Now register the pairs.
            // The path includes the src, sink nodes, so skip them.
            for i in 1..path.len() - 1 {
                let a = path[i].as_terminal_on_layer();
                let b = path[i + 1].as_terminal_on_layer();

                self.assign_terminal_to_layer(&a);
                self.assign_terminal_to_layer(&b);

                if let Some(a) = a {
                    if let Some(b) = b {
                        if a.layer == b.layer {
                            // We're forming a component; update info.
                            let layer = a.layer as usize;
                            self.components[layer].add_path(&self.terminal_by_id[&a.terminal_id],
                                                            &self.terminal_by_id[&b.terminal_id]);
                        }
                    }
                }
            }

            self.assignment.push((netid, path));
        }

    }
    //    thread::sleep(time::Duration::from_millis(10));
}
