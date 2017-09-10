// Find a path for a set of terminals on a single layer
use std::collections::{HashSet, HashMap};
use features::Terminal;
use std::rc::Rc;
use std::sync::Arc;
use geom::{OrderedPoint, Point, Shape};
use dijkstra::shortest_path;
use ncollide::broad_phase::BroadPhase;
use ncollide::broad_phase::DBVTBroadPhase;
use ncollide::bounding_volume::AABB;
use progress::Progress;
use petgraph::graphmap::UnGraphMap;
use std::f64::INFINITY;
use std::collections::hash_map::Entry::{Occupied, Vacant};

type Path = (OrderedPoint, OrderedPoint);
type PadBroadPhase = DBVTBroadPhase<Point, AABB<Point>, usize>;

type TerminalBroadPhase = DBVTBroadPhase<Point, AABB<Point>, usize>;

pub type CDTGraph = UnGraphMap<OrderedPoint, f64>;

#[derive(Debug, Clone)]
pub struct TerminalInfo {
    // Terminals for paths that are routed around this terminal;
    // the rubber band sketch snaps to this point.
    // The TerminalId is the id of the peer terminal.
    pub attached: Vec<OrderedPoint>,
    // Terminals for paths that are directly incident to this
    // terminal.  They have electrical connectivity to it.
    // The TerminalId is the id of the peer terminal.
    pub incident: Vec<OrderedPoint>,
}

/// Given an ordering of twonet paths, re-order the paths such that
/// the result is planar.  This works by considering whether each
/// twonet is "open" or "closed".  A "closed" twonet is one that closes
/// a path between two terminals when considering the twonets preceeding
/// it.  Otherwise (eg: it is blocked by those before it) it is "open".
/// The open nets are brought to the front of the ordering while
/// maintaining the relative ordering of the open and closed sets respectively.
fn planarity_enforcement_operator(twonets: &Vec<Path>) -> Vec<Path> {
    type Broad = DBVTBroadPhase<Point, AABB<Point>, usize>;
    let mut broad = Broad::new(0.0, false);
    let mut lines = Vec::new();

    let mut open = Vec::new();
    let mut closed = Vec::new();

    for path in twonets.iter() {
        let line = Shape::line(&path.0.point(), &path.1.point());
        let idx = lines.len();
        let bv = line.aabb();
        let mut is_open = false;

        {
            let mut candidates = Vec::new();
            broad.interferences_with_bounding_volume(&bv, &mut candidates);

            for lineidx in candidates.iter().map(|x| **x) {
                let other_path = &twonets[lineidx];
                if other_path.0 == path.0 || other_path.0 == path.1 ||
                    other_path.1 == path.0 || other_path.1 == path.1 {
                    // We're expanding this twonet into a larger component
                    continue;
                }
                if let Some(_) = line.contact(&lines[lineidx], 0.0) {
                    // Blocked by a preceeding item
                    is_open = true;
                    break;
                }
            }
        }

        broad.deferred_add(idx, bv, idx);
        lines.push(line);
        broad.update(&mut |a, b| a != b, &mut |_, _, _| {});

        if is_open {
            open.push(*path);
        } else {
            closed.push(*path);
        }
    }

    open.append(&mut closed);
    open
}

struct Assignment {
    assignment: HashMap<OrderedPoint, TerminalInfo>,
    broad_phase: TerminalBroadPhase,
    paths: Vec<Path>,
    path_lines: Vec<Shape>,
}

impl Assignment {
    fn new(clearance: f64) -> Assignment {
        Assignment {
            assignment: HashMap::new(),
            broad_phase: TerminalBroadPhase::new(clearance, false),
            paths: Vec::new(),
            path_lines: Vec::new(),
        }
    }
}

pub struct PathConfiguration {
    cdt: Rc<CDTGraph>,
    twonets: Vec<Path>,
    assignment: Assignment,
    broad_all_pads: PadBroadPhase,
    all_pads: Vec<Arc<Terminal>>,
    clearance: f64,
}

impl PathConfiguration {
    pub fn new(clearance: f64,
               cdt: Rc<CDTGraph>,
               paths: &Vec<Path>,
               all_pads: &Vec<Arc<Terminal>>)
               -> PathConfiguration {

        let mut pads = PadBroadPhase::new(clearance, false);

        for (idx, term) in all_pads.iter().enumerate() {
            pads.deferred_add(idx, term.shape.aabb(), idx);
        }
        pads.update(&mut |a, b| a != b, &mut |_, _, _| {});

        PathConfiguration {
            cdt: cdt,
            twonets: paths.clone(),
            assignment: Assignment::new(clearance),
            all_pads: all_pads.clone(),
            broad_all_pads: pads,
            clearance: clearance,
        }
    }

    fn edge_cost(&self, a: &OrderedPoint, b: &OrderedPoint, edge_weight: &f64) -> f64 {
        let base_cost = *edge_weight;

        // If this path crosses any pads we disallow it, unless it is intentionally
        // trying to be directly incident to the pad
        let line = Shape::line(&a.point(), &b.point());
        let bv = line.aabb();

        let mut candidates = Vec::new();
        self.broad_all_pads
            .interferences_with_bounding_volume(&bv, &mut candidates);

        for padidx in candidates.iter().map(|x| **x) {
            let term = &self.all_pads[padidx];
            // Allow direct connections to the terminal
            if term.point == a.point() || term.point == b.point() {
                continue;
            }
            if let Some(_) = term.shape.contact(&line, self.clearance) {
                // Otherwise we cannot connect here
                return INFINITY;
            }
        }

        let mut candidates = Vec::new();
        self.assignment
            .broad_phase
            .interferences_with_bounding_volume(&bv, &mut candidates);

        for edge_idx in candidates.iter().map(|x| **x) {
            // Allow direct connections to the terminal
            let edge_line = &self.assignment.path_lines[edge_idx];
            let path = &self.assignment.paths[edge_idx];
            if path.0 == *a || path.0 == *b || path.1 == *a || path.1 == *b {
                continue;
            }
            if let Some(_) = edge_line.contact(&line, self.clearance) {
                // Otherwise we cannot connect here
                return INFINITY;
            }
        }

        base_cost
    }

    fn compute_path(&self,
                    twonet_path: &Path,
                    cutoff: Option<f64>)
                    -> Option<(f64, Vec<OrderedPoint>)> {
        shortest_path(&*self.cdt,
                      twonet_path.0,
                      twonet_path.1,
                      |(a, b, edge)| self.edge_cost(&a, &b, edge),
                      cutoff)
    }

    fn a_to_b(&mut self, a: OrderedPoint, b: OrderedPoint, is_incident: bool) {
        match self.assignment.assignment.entry(a) {
            Occupied(mut ent) => {
                if is_incident {
                    ent.get_mut().incident.push(b);
                } else {
                    ent.get_mut().attached.push(b);
                }
            }
            Vacant(ent) => {
                let mut info = TerminalInfo {
                    attached: Vec::new(),
                    incident: Vec::new(),
                };
                if is_incident {
                    info.incident.push(b);
                } else {
                    info.attached.push(b);
                }
                ent.insert(info);
            }
        }
    }

    fn add_assignment(&mut self, full_path: Vec<OrderedPoint>) {
        for i in 0..full_path.len() - 1 {
            let a = full_path[i];
            let b = full_path[i + 1];
            let path = (a, b);
            let line = Shape::line(&a.point(), &b.point());

            let edge_idx = self.assignment.paths.len();
            self.assignment
                .broad_phase
                .deferred_add(self.assignment.paths.len(), line.aabb(), edge_idx);
            self.assignment.paths.push(path);
            self.assignment.path_lines.push(line);
            self.a_to_b(a, b, i == 0);
            self.a_to_b(b, a, i + 1 == full_path.len() - 1);
        }
        self.assignment
            .broad_phase
            .update(&mut |a, b| a != b, &mut |_, _, _| {});
    }

    pub fn initial_assignment(&mut self) {
        let pb = Progress::new("initial path", self.twonets.len());
        let mut free_nets: HashSet<Path> = self.twonets.iter().map(|x| *x).collect();

        loop {
            if free_nets.len() == 0 {
                break;
            }

            pb.inc();

            let mut best = None;
            for path_ref in free_nets.iter() {
                let twonet_path = *path_ref;
                let cutoff = best.as_ref().and_then(|&(_, cost, _)| Some(cost));

                if let Some((cost, path)) = self.compute_path(&twonet_path, cutoff) {
                    if best.is_none() || cost < best.as_ref().map(|&(_, cost, _)| cost).unwrap() {
                        best = Some((twonet_path, cost, path));
                    }
                }
            }

            let (twonet_path, _, path) = best.expect("must always be able to find the best");
            free_nets.remove(&twonet_path);
            self.add_assignment(path);
        }
    }

    pub fn get_paths(&self) -> &Vec<Path> {
        &self.assignment.paths
    }
}
