// Find a path for a set of terminals on a single layer
use super::layerassign::InterferenceHandler;
use crate::dijkstra::shortest_path;
use crate::features::Terminal;
use crate::geom::{OrderedPoint, Shape};
use crate::progress::Progress;
use nalgebra as na;
use ncollide2d::bounding_volume::AABB;
use ncollide2d::broad_phase::BroadPhase;
use ncollide2d::broad_phase::DBVTBroadPhase;
use petgraph::graphmap::UnGraphMap;
use std::collections::hash_map::Entry::{Occupied, Vacant};
use std::collections::{HashMap, HashSet};
use std::f64::INFINITY;
use std::rc::Rc;
use std::sync::Arc;

type Path = (OrderedPoint, OrderedPoint);
type PadBroadPhase = DBVTBroadPhase<f64, AABB<f64>, usize>;

type TerminalBroadPhase = DBVTBroadPhase<f64, AABB<f64>, usize>;

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
            broad_phase: TerminalBroadPhase::new(clearance),
            paths: Vec::new(),
            path_lines: Vec::new(),
        }
    }
}

#[derive(Clone)]
struct Component {
    twonets: Vec<Path>,
}

struct ComponentWithHull {
    twonets: Vec<Path>,
    lines: Vec<Shape>,
    bvs: Vec<AABB<f64>>,
    hull: Shape,
    broad: TerminalBroadPhase,
}

#[derive(Default)]
struct ComponentGrouper {
    components: Vec<Component>,
    point_to_comp: HashMap<OrderedPoint, usize>,
}

impl ComponentWithHull {
    fn new(comp: &Component) -> ComponentWithHull {
        let lines: Vec<_> = comp
            .twonets
            .iter()
            .map(|&(a, b)| Shape::line(&a.point(), &b.point()))
            .collect();
        let hull = Shape::convex_hull(&lines);

        let mut broad = TerminalBroadPhase::new(0.0);
        let mut bvs = Vec::new();

        for (idx, line) in lines.iter().enumerate() {
            let bv = line.aabb();
            broad.create_proxy(bv.clone(), idx);
            bvs.push(bv);
        }
        broad.update(&mut InterferenceHandler {});

        ComponentWithHull {
            twonets: comp.twonets.clone(),
            lines,
            hull,
            bvs,
            broad,
        }
    }
}

impl ComponentGrouper {
    fn biggest_component(&self, path: &Path) -> (Option<usize>, Option<usize>) {
        let a_idx = self.point_to_comp.get(&path.0).copied();
        let b_idx = self.point_to_comp.get(&path.1).copied();

        if a_idx.is_none() {
            return (b_idx, None);
        }

        if b_idx.is_none() {
            return (a_idx, None);
        }

        let a_idx = a_idx.expect("checked above");
        let b_idx = b_idx.expect("checked above");

        if a_idx == b_idx {
            return (Some(a_idx), None);
        }

        let a = &self.components[a_idx];
        let b = &self.components[b_idx];

        let a_len = a.twonets.len();
        let b_len = b.twonets.len();

        if a_len > b_len {
            return (Some(a_idx), Some(b_idx));
        }

        if a_len < b_len {
            return (Some(b_idx), Some(a_idx));
        }

        (Some(a_idx), Some(b_idx))
    }

    fn add_path(&mut self, path: &Path) {
        let (target_idx, src_idx) = self.biggest_component(path);

        if let Some(target_idx) = target_idx {
            if let Some(src_idx) = src_idx {
                // Update mapping
                for path in self.components[src_idx].twonets.iter() {
                    self.point_to_comp.insert(path.0, target_idx);
                    self.point_to_comp.insert(path.1, target_idx);
                }

                // Merge src -> target
                if src_idx < target_idx {
                    let (s1, s2) = self.components.split_at_mut(target_idx);
                    s2[0].twonets.append(&mut s1[src_idx].twonets);
                } else {
                    let (s1, s2) = self.components.split_at_mut(src_idx);
                    s1[target_idx].twonets.append(&mut s2[0].twonets);
                }
            }

            // Add to target component
            self.point_to_comp.insert(path.0, target_idx);
            self.point_to_comp.insert(path.1, target_idx);

            let target_comp = &mut self.components[target_idx];
            target_comp.twonets.push(*path);
        } else {
            // Create a new component

            let idx = self.components.len();
            let comp = Component {
                twonets: vec![*path],
            };
            self.components.push(comp);
            self.point_to_comp.insert(path.0, idx);
            self.point_to_comp.insert(path.1, idx);
        }
    }

    fn add_paths(&mut self, paths: &[Path]) {
        for path in paths.iter() {
            self.add_path(path);
        }
    }

    // Remove any empty (merged) components
    fn minimize(&self) -> Vec<ComponentWithHull> {
        self.components
            .iter()
            .filter(|comp| !comp.twonets.is_empty())
            .map(|comp| ComponentWithHull::new(comp))
            .collect()
    }
}

pub struct PathConfiguration {
    cdt: Rc<CDTGraph>,
    assignment: Assignment,
    broad_all_pads: PadBroadPhase,
    all_pads: Vec<Arc<Terminal>>,
    clearance: f64,
    components: Vec<ComponentWithHull>,
}

impl PathConfiguration {
    pub fn new(
        clearance: f64,
        cdt: Rc<CDTGraph>,
        paths: &[Path],
        all_pads: &[Arc<Terminal>],
    ) -> PathConfiguration {
        let mut pads = PadBroadPhase::new(clearance);

        for (idx, term) in all_pads.iter().enumerate() {
            pads.create_proxy(term.shape.aabb(), idx);
        }
        pads.update(&mut InterferenceHandler {});

        let mut components = ComponentGrouper::default();
        components.add_paths(paths);

        PathConfiguration {
            cdt,
            assignment: Assignment::new(clearance),
            all_pads: all_pads.to_vec(),
            broad_all_pads: pads,
            clearance,
            components: components.minimize(),
        }
    }

    fn edge_cost(&self, a: &OrderedPoint, b: &OrderedPoint, edge_weight: f64) -> f64 {
        let base_cost = edge_weight;

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
            if term.shape.contact(&line, self.clearance).is_some() {
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
            if edge_line.contact(&line, self.clearance).is_some() {
                // Otherwise we cannot connect here
                return INFINITY;
            }
        }

        base_cost
    }

    fn compute_path(
        &self,
        twonet_path: &Path,
        cutoff: Option<f64>,
    ) -> Option<(f64, Vec<OrderedPoint>)> {
        shortest_path(
            &*self.cdt,
            twonet_path.0,
            twonet_path.1,
            |(a, b, &edge)| self.edge_cost(&a, &b, edge),
            cutoff,
        )
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
                .create_proxy(line.aabb(), edge_idx);
            self.assignment.paths.push(path);
            self.assignment.path_lines.push(line);
            self.a_to_b(a, b, i == 0);
            self.a_to_b(b, a, i + 1 == full_path.len() - 1);
        }
        self.assignment
            .broad_phase
            .update(&mut InterferenceHandler {});
    }

    /// This method computes the order in which we will route the paths in
    /// the final topology.  The ordering is important to ensure that the
    /// result is planar (eg: no paths will cross) and to minimize the
    /// length of the wiring.
    /// The twonet ordering problem is comprised of the following steps:
    /// 1. Grouping the twonets into components (handled in our constructor)
    /// 2. Building an n-by-n matrix (where n = number of components)
    /// 3. The values in the matrix are the detour cost of the pair
    /// 4. A given component when paired with itself has a 0 detour cost
    /// 5. Minimize the lower triangle.
    /// Note that thesis describes a concept of closed and open twonets;
    /// those are used when binning the problem into smaller portions.
    /// Since we are not binning we don't need to perform the extra
    /// planar enforcement operator step.
    pub fn compute_twonet_order(&mut self) -> Vec<Path> {
        let num_comps = self.components.len();
        let pb = Progress::new("component ordering", num_comps);

        let mut input_matrix: Vec<Vec<f64>> = Vec::with_capacity(num_comps);

        for i in 0..num_comps {
            let mut row = Vec::with_capacity(num_comps);
            for j in 0..num_comps {
                if i == j {
                    row.push(0.0);
                } else {
                    row.push(self.compute_detour_cost(i, j));
                }
            }
            input_matrix.push(row);
        }

        let mut output_matrix = input_matrix.clone();

        let mut component_order: Vec<usize> = Vec::with_capacity(num_comps);
        let mut selected = HashSet::new();

        for _ in (0..num_comps).rev() {
            pb.inc();
            let mut best_cost = INFINITY;
            let mut best_comp = 0;

            #[allow(clippy::needless_range_loop)]
            for j in 0..num_comps {
                if selected.contains(&j) {
                    continue;
                }

                let cost = output_matrix[j].iter().sum();
                if cost < best_cost {
                    best_comp = j;
                    best_cost = cost;
                }
            }

            selected.insert(best_comp);
            component_order.push(best_comp);
            for i in 0..num_comps {
                output_matrix[best_comp][i] = 0.0;
                output_matrix[i][best_comp] = 0.0;
            }
        }

        // component_order now holds the components in the reverse
        // order they should be routed, so walk over it backwards
        // to yield the final ordering.
        component_order
            .iter()
            .rev()
            .flat_map(|i| self.components[*i].twonets.iter())
            .copied()
            .collect()
    }

    fn compute_detour_cost(&self, component_i: usize, component_j: usize) -> f64 {
        let mut detour_cost = 0.0;
        let mut base_cost = 0.0;

        let component_i = &self.components[component_i];
        let component_j = &self.components[component_j];

        // The first component is easy: there are no conflicts so we just
        // accumulate the base cost
        for idx in 0..component_i.twonets.len() {
            let path = component_i.twonets[idx];
            base_cost += na::distance(&path.0.point(), &path.1.point());
        }

        // The second component may conflict with the first
        for idx in 0..component_j.twonets.len() {
            let path = component_j.twonets[idx];
            let line = &component_j.lines[idx];
            let bv = &component_j.bvs[idx];
            let a = path.0.point();
            let b = path.1.point();

            base_cost += na::distance(&path.0.point(), &path.1.point());

            {
                // Does this line collide with anything previously routed?
                let mut candidates = Vec::new();
                component_i
                    .broad
                    .interferences_with_bounding_volume(&bv, &mut candidates);

                for i_idx in candidates.iter().map(|x| *x) {
                    if line.contact(&component_i.lines[*i_idx], 0.0).is_some() {
                        // We have a collision so we need to detour around the hull

                        if let Some(detour) = component_i.hull.detour_path(&a, &b, 0.0) {
                            let (cost, _) =
                                shortest_path(&detour, path.0, path.1, |(_, _, cost)| *cost, None)
                                    .expect("must be a path");
                            detour_cost += cost;
                        }
                    }
                }
            }

            {
                // Route around pads
                let mut candidates = Vec::new();
                self.broad_all_pads
                    .interferences_with_bounding_volume(&bv, &mut candidates);

                for padidx in candidates.iter().map(|x| **x) {
                    let term = &self.all_pads[padidx];
                    // Allow direct connections to the terminal
                    if term.point == a || term.point == b {
                        continue;
                    }
                    if term.shape.contact(&line, self.clearance).is_some() {
                        if let Some(detour) = term.shape.detour_path(&a, &b, 0.0) {
                            let (cost, _) =
                                shortest_path(&detour, path.0, path.1, |(_, _, cost)| *cost, None)
                                    .expect("must be a path");
                            detour_cost += cost;
                        }
                    }
                }
            }
        }

        detour_cost + base_cost
    }

    pub fn compute_path_from_order(&mut self, twonets: &[Path]) {
        for twonet_path in twonets.iter() {
            if let Some((_, path)) = self.compute_path(&twonet_path, None) {
                self.add_assignment(path);
            }
        }
    }

    pub fn get_paths(&self) -> &Vec<Path> {
        &self.assignment.paths
    }
}
