use petgraph::graphmap::DiGraphMap;
use features::{LayerSet, Terminal};
use geom::{Shape, Point};
use std::collections::{HashSet, HashMap};
use ncollide::broad_phase::BroadPhase;
use ncollide::broad_phase::DBVTBroadPhase;
extern crate ncollide;
extern crate nalgebra as na;
use std::sync::Arc;
use dijkstra::shortest_path;
use ordered_float::OrderedFloat;
use std::rc::Rc;
use progress::Progress;
use cdt;

pub type TerminalId = usize;
const NUM_VIAS: usize = 5;
// Technically we should use the units and resolution from the pcb for this
const VIA_MAX_DIST: f64 = 5000.0;
const ALPHA: f64 = 0.1;

pub type CDTVertex = cdt::Vertex<TerminalId>;
pub type CDT = cdt::CDT<TerminalId>;
pub type CDTGraph = cdt::CDTUnGraph<TerminalId>;

// Use the raw pointer to the terminal when keying into hashes.
fn raw_terminal_ptr(t: &Arc<Terminal>) -> *const Terminal {
    &**t
}

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
    pub hull_perimeter: f64,
}

impl Component {
    fn compute_perimeter(&mut self) {
        let points = self.hull.compute_points();
        let mut cost = 0.0;
        for i in 0..points.len() - 1 {
            let a = &points[i];
            let b = &points[i + 1];
            cost += na::distance(a, b);
        }

        self.hull_perimeter = cost;
    }
}

#[derive(Clone)]
pub struct ComponentPath {
    pub line: Shape,
    pub a: Arc<Terminal>,
    pub b: Arc<Terminal>,
}


pub type ComponentBroadPhase = DBVTBroadPhase<Point, ncollide::bounding_volume::AABB<Point>, usize>;

pub type ComponentIndex = usize;
pub struct ComponentList {
    pub components: Vec<Component>,
    pub terminal_to_component: HashMap<*const Terminal, ComponentIndex>,
    broad_phase: ComponentBroadPhase,
    paths: Vec<ComponentPath>,
}

impl ::std::fmt::Debug for ComponentList {
    fn fmt(&self, fmt: &mut ::std::fmt::Formatter) -> ::std::result::Result<(), ::std::fmt::Error> {
        write!(fmt,
               "ComponentList with {} components",
               self.components.len())?;
        Ok(())
    }
}

impl Clone for ComponentList {
    fn clone(&self) -> ComponentList {
        let mut list = ComponentList {
            terminal_to_component: self.terminal_to_component.clone(),
            components: self.components.clone(),
            broad_phase: ComponentBroadPhase::new(1.0, false),
            paths: self.paths.clone(),
        };

        // Rebuild the broad phase
        for phase_id in 0..list.paths.len() {
            let path = &list.paths[phase_id];
            list.broad_phase
                .deferred_add(phase_id, path.line.aabb(), phase_id);
        }

        list.broad_phase
            .update(&mut |a, b| a != b, &mut |_, _, _| {});

        list
    }
}

impl ComponentList {
    fn new() -> ComponentList {
        ComponentList {
            terminal_to_component: HashMap::new(),
            components: Vec::new(),
            broad_phase: ComponentBroadPhase::new(1.0, false),
            paths: Vec::new(),
        }
    }

    fn intersects
        (&self,
         edge: &PathEdge)
         -> Option<Vec<(Arc<Terminal>, Arc<Terminal>, ncollide::query::Contact<Point>)>> {
        if let Some((ref line, ref bv)) = edge.line {
            let mut candidates = Vec::new();
            self.broad_phase
                .interferences_with_bounding_volume(bv, &mut candidates);

            if candidates.len() > 0 {

                return Some(candidates
                                .into_iter()
                                .filter_map(|path_index| {
                    let path = &self.paths[*path_index];
                    if let Some(contact) = path.line.contact(line) {
                        Some((Arc::clone(&path.a), Arc::clone(&path.b), contact))
                    } else {
                        None
                    }
                })
                                .collect());
            }
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
        let a_idx = self.terminal_to_component
            .get(&raw_terminal_ptr(ta))
            .map(|x| *x);
        let b_idx = self.terminal_to_component
            .get(&raw_terminal_ptr(tb))
            .map(|x| *x);

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

        let phase_id = self.paths.len();
        self.paths
            .push(ComponentPath {
                      line: line.clone(),
                      a: Arc::clone(a),
                      b: Arc::clone(b),
                  });

        self.broad_phase
            .deferred_add(phase_id, line.aabb(), phase_id);

        if let Some(target_idx) = target_idx {
            if let Some(src_idx) = src_idx {
                // Merge src_comp into target_comp
                let mut src_comp = self.components[src_idx].clone();
                let ref mut target_comp = self.components[target_idx];

                for t in src_comp.terminals.iter() {
                    self.terminal_to_component
                        .insert(raw_terminal_ptr(t), target_idx);
                }

                target_comp.terminals.extend(src_comp.terminals.drain());
                target_comp.paths.extend(src_comp.paths.drain(..));
            }

            // Add to target component
            self.terminal_to_component
                .insert(raw_terminal_ptr(a), target_idx);
            self.terminal_to_component
                .insert(raw_terminal_ptr(b), target_idx);

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
            target_comp.compute_perimeter();


        } else {
            // Create a new component
            let comp_idx = self.components.len();
            let mut comp = Component {
                terminals: hashset!{Arc::clone(a), Arc::clone(b)},
                paths: vec![(Arc::clone(a), Arc::clone(b), line.clone())],
                hull: line,
                hull_perimeter: 0.0,
            };
            comp.compute_perimeter();
            self.components.push(comp);

            self.terminal_to_component
                .insert(raw_terminal_ptr(a), comp_idx);
            self.terminal_to_component
                .insert(raw_terminal_ptr(b), comp_idx);
        }

        self.broad_phase
            .update(&mut |a, b| a != b, &mut |_, _, _| {});
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
    base_cost: f64,
}
pub type TwoNetId = usize;

#[derive(Debug, Default, Clone)]
pub struct Assignment {
    netid: TwoNetId,
    cost: f64,
    path: Vec<Node>,
    base_cost: f64,
}

#[derive(Debug, Default, Clone)]
pub struct SharedConfiguration {
    terminal_by_id: HashMap<TerminalId, Arc<Terminal>>,
    terminals: HashMap<*const Terminal, TerminalInfo>,

    pub cdt: CDTGraph,

    // Indices are referenced via TwoNetId
    two_nets: Vec<TwoNet>,
    all_layers: LayerSet,
}

impl SharedConfiguration {
    pub fn new(all_layers: &LayerSet) -> SharedConfiguration {
        let mut cfg = SharedConfiguration::default();
        cfg.all_layers = all_layers.clone();
        cfg
    }

    // add or resolve a terminal to its TerminalId
    pub fn add_terminal(&mut self, terminal: &Arc<Terminal>) -> TerminalId {
        let raw = raw_terminal_ptr(terminal);
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

    pub fn add_twonet(&mut self, a: &Arc<Terminal>, b: &Arc<Terminal>) -> (TerminalId, TerminalId) {
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

        let num_vias = {
            let total_distance = na::distance(&src_point, &sink_point);
            let distance = total_distance / NUM_VIAS as f64;
            if distance >= VIA_MAX_DIST {
                (total_distance / VIA_MAX_DIST) as usize
            } else {
                NUM_VIAS
            }
        };

        let x_delta = (sink_point.coords.x - src_point.coords.x) / num_vias as f64;
        let y_delta = (sink_point.coords.y - src_point.coords.y) / num_vias as f64;

        // Build up the via nodes on each layer.
        let mut via_terminals = Vec::new();
        let mut via_points = Vec::new();
        for i in 0..num_vias {
            let pt = Point::new(src_point.coords.x + x_delta * i as f64,
                                src_point.coords.y + y_delta * i as f64);
            via_points.push(pt.clone());

            let via_shape = Shape::circle_from_point(&pt, 200.0);
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
        for i in 0..num_vias {
            for layer in 0..self.all_layers.len() - 1 {
                let via_terminal = via_terminals[i];
                g.add_edge(Node::via(via_terminal, layer),
                           Node::via(via_terminal, layer + 1),
                           PathEdge::zero());
            }
        }

        // Build left/right connectivity between the vias
        for i in 1..num_vias {
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

            g.add_edge(Node::via(via_terminals[num_vias - 1], layer),
                       Node::terminal(b_id, layer),
                       PathEdge::from_points(&via_points[num_vias - 1], &sink_point));

            g.add_edge(Node::terminal(b_id, layer),
                       Node::sink(b_id),
                       PathEdge::zero());
        }

        self.two_nets
            .push(TwoNet {
                      graph: g,
                      src: a_id,
                      sink: b_id,
                      base_cost: na::distance(&src_point, &sink_point),
                  });
        (a_id, b_id)
    }
}

#[derive(Debug, Default, Clone)]
pub struct Configuration {
    // map constant terminal info to the mutable (within
    // this configuration instance) node info.
    terminals: HashMap<*const Terminal, TerminalInfo>,
    components: Vec<ComponentList>,
    shared: Rc<SharedConfiguration>,

    pub assignment: Vec<Assignment>,
    pub overall_cost: f64,
}

impl Configuration {
    pub fn new(shared: &Rc<SharedConfiguration>) -> Configuration {
        let terminals = shared.terminals.clone();
        let mut cfg = Configuration {
            terminals: terminals,
            components: Vec::new(),
            shared: Rc::clone(shared),
            assignment: Vec::new(),
            overall_cost: 0.0,
        };

        for _ in 0..shared.all_layers.len() {
            cfg.components.push(ComponentList::new());
        }

        cfg
    }

    fn src_sink_cost(&self, _: TerminalId, tol: &TerminalOnLayer) -> f64 {
        let term = &self.shared.terminal_by_id[&tol.terminal_id];
        let raw = raw_terminal_ptr(term);
        let info = &self.terminals[&raw];

        if !term.layers.contains(&tol.layer) {
            // Not connected to this layer
            return ::std::f64::INFINITY;

        }

        if info.configured_layers.len() > 0 && !info.configured_layers.contains(&tol.layer) {
            // TODO: re-examine the intent of this check.  Right now it
            // prevents us from using the alternate layer in most cases.
            // return ::std::f64::INFINITY;
        }
        return 0.0;
    }

    fn detour_cost(&self,
                   a: &Arc<Terminal>,
                   b: &Arc<Terminal>,
                   comp_list: &ComponentList,
                   contacts: &Vec<(Arc<Terminal>,
                                   Arc<Terminal>,
                                   ncollide::query::Contact<Point>)>)
                   -> f64 {
        let mut cost = 0.0;
        for &(ref terminal_a, ref terminal_b, _) in contacts.iter() {

            // Skip self intersection
            if raw_terminal_ptr(&a) == raw_terminal_ptr(&terminal_a) ||
               raw_terminal_ptr(&b) == raw_terminal_ptr(&terminal_a) ||
               raw_terminal_ptr(&a) == raw_terminal_ptr(&terminal_b) ||
               raw_terminal_ptr(&b) == raw_terminal_ptr(&terminal_b) {
                continue;
            }

            let component_idx = comp_list.terminal_to_component[&raw_terminal_ptr(terminal_a)];
            let comp = &comp_list.components[component_idx];

            // TODO: we can use the contact point to find the shortest path
            // around the perimeter.  For now we just take the perimeter
            // as an estimation.

            cost += comp.hull_perimeter / 2.0;
        }

        cost
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

            // Bias vertical lines to one layer, horizontal to the other
            let layer_bias = match (a.layer, edge.line.as_ref().unwrap().0.is_verticalish_line()) {
                // vertical lines are more expensive on layer 0
                (0, true) => 1.1,
                // horizontal lines are more expensive on layer 1
                (1, false) => 1.1,
                _ => 1.0,
            };

            // We may be colliding with something
            let comp_list = &self.components[a.layer as usize];
            let mut detour_cost = 0.0;

            if edge.base_cost > 0.0 {
                let term_a = &self.shared.terminal_by_id[&a.terminal_id];
                let term_b = &self.shared.terminal_by_id[&b.terminal_id];

                if let Some(contacts) = comp_list.intersects(edge) {
                    detour_cost = self.detour_cost(&term_a, &term_b, comp_list, &contacts);
                }
            }

            (1.0 - ALPHA) * (edge.base_cost + detour_cost) * layer_bias
        } else {
            // Moving between levels; impose a token cost to avoid
            // the algorithm from chosing to change levels for no reason.
            ALPHA
        }
    }

    fn assign_terminal_to_layer(&mut self, tol: &Option<TerminalOnLayer>) {
        if let &Some(tol) = tol {
            let t = &self.shared.terminal_by_id[&tol.terminal_id];
            // We always populate self.terminals during initialization,
            // so this condition should always hold true.
            let raw = raw_terminal_ptr(t);
            if let Some(info) = self.terminals.get_mut(&raw) {
                info.configured_layers.insert(tol.layer);
            } else {
                panic!("wat!?");
            }
        }
    }

    fn path_for_net(&self, netid: TwoNetId, cutoff: Option<f64>) -> Option<(f64, Vec<Node>)> {
        let ref twonet = self.shared.two_nets[netid];

        shortest_path(&twonet.graph,
                      Node::src(twonet.src),
                      Node::sink(twonet.sink),
                      |(a, b, edge)| self.edge_cost(&a, &b, edge),
                      cutoff)
    }

    fn assign_path(&mut self, assignment: Assignment) {
        // Now register the pairs.
        // The path includes the src, sink nodes, so skip them.
        for i in 1..assignment.path.len() - 1 {
            let a = assignment.path[i].as_terminal_on_layer();
            let b = assignment.path[i + 1].as_terminal_on_layer();

            self.assign_terminal_to_layer(&a);
            self.assign_terminal_to_layer(&b);

            if let Some(a) = a {
                if let Some(b) = b {
                    if a.layer == b.layer {
                        // We're forming a component; update info.
                        let layer = a.layer as usize;
                        self.components[layer].add_path(&self.shared.terminal_by_id
                                                             [&a.terminal_id],
                                                        &self.shared.terminal_by_id
                                                             [&b.terminal_id]);
                    }
                }
            }
        }

        self.overall_cost += assignment.cost;
        self.assignment.push(assignment);
    }

    /// Initial assignment is done by finding the cheapest path and assigning that
    /// first, then repeating to find the next cheapest path and so on.
    pub fn initial_assignment(&mut self) {
        let pb = Progress::new("initial assignment", self.shared.two_nets.len());

        let mut free_nets: HashSet<TwoNetId> = (0..self.shared.two_nets.len()).collect();

        loop {
            if free_nets.len() == 0 {
                break;
            }

            pb.inc();
            let mut best = None;
            for i in free_nets.iter() {
                let netid: TwoNetId = *i;
                let ref twonet = self.shared.two_nets[netid];
                let cutoff = best.as_ref().and_then(|&(_, cost, _, _)| Some(cost));

                if let Some((cost, path)) = self.path_for_net(netid, cutoff) {
                    if best.is_none() ||
                       cost < best.as_ref().map(|&(_, cost, _, _)| cost).unwrap() {
                        best = Some((netid, cost, path, twonet.base_cost));
                    }
                }
            }

            let (netid, cost, path, base_cost) =
                best.expect("must always be able to find the best");
            free_nets.remove(&netid);

            self.assign_path(Assignment {
                                 netid: netid,
                                 cost: cost,
                                 path: path,
                                 base_cost: base_cost,
                             });

        }
    }

    pub fn improve_one(&self) -> Option<Configuration> {
        // Do we have any paths that we think could be improved?
        // Comparing the base cost of the path with the final cost helps us
        // decide if there is potential for an improvement.

        let mut needs_improvement: Vec<(_, _, _)> = self.assignment
            .iter()
            .enumerate()
            .filter_map(|(idx, ref assignment)| if assignment.cost >
                                                   (1.0 - ALPHA) * assignment.base_cost {
                            // Track the difference between the base and the actual cost;
                            // we will try to improve the largest of these first
                            Some((OrderedFloat(assignment.cost - assignment.base_cost),
                                  idx,
                                  assignment.base_cost))
                        } else {
                            None
                        })
            .collect();

        // Sort by decreasing cost delta
        needs_improvement.sort_by(|&(a, _, _), &(b, _, _)| b.cmp(&a));
        let pb = Progress::new("improving", needs_improvement.len());

        for &(_, worst_idx, _) in pb.wrap_iter(needs_improvement.iter()) {
            // make a new Configuration that we can build up in a different order
            let mut cfg = Configuration::new(&self.shared);

            // Now, take the worst_idx first and assignment, stitching in the
            // paths in the prior assignment order, taking care not to assign
            // worst_idx twice.
            let mut complete = true;
            for idx in [worst_idx]
                    .iter()
                    .map(|x| *x)
                    .chain((0..self.assignment.len()).filter(|x| *x != worst_idx)) {
                let assignment = &self.assignment[idx];

                // let the path finding algo bail out early if this path puts
                // us over the target cost.
                let cutoff = Some(self.overall_cost - cfg.overall_cost);

                if let Some((cost, path)) = cfg.path_for_net(assignment.netid, cutoff) {
                    cfg.assign_path(Assignment {
                                        netid: assignment.netid,
                                        cost: cost,
                                        path: path,
                                        base_cost: assignment.base_cost,
                                    });
                } else {
                    complete = false;
                    break;
                }
            }

            if complete && cfg.overall_cost < self.overall_cost {
                return Some(cfg);
            }
        }
        None
    }

    pub fn extract_paths(&self) -> HashMap<u8, Vec<(Arc<Terminal>, Arc<Terminal>)>> {
        let mut paths = HashMap::<u8, Vec<(Arc<Terminal>, Arc<Terminal>)>>::new();

        for layer_id in self.shared.all_layers.iter() {
            paths.insert(*layer_id, Vec::new());
        }

        for ref assignment in self.assignment.iter() {
            // Skip source and sink nodes
            for i in 1..assignment.path.len() - 2 {
                let a = assignment.path[i];
                let b = assignment.path[i + 1];

                match (a.as_terminal_on_layer(), b.as_terminal_on_layer()) {
                    (Some(a), Some(b)) => {
                        if a.layer == b.layer {
                            let ta = &self.shared.terminal_by_id[&a.terminal_id];
                            let tb = &self.shared.terminal_by_id[&b.terminal_id];

                            if let Some(paths) = paths.get_mut(&b.layer) {
                                paths.push((Arc::clone(ta), Arc::clone(tb)));
                            }
                        }
                    }
                    _ => {}
                }
            }
        }

        paths
    }
}
