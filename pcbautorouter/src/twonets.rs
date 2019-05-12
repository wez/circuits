use crate::features::{LayerSet, Terminal};
use crate::geom;
use itertools::Itertools;
use ncollide2d::query::Proximity::Disjoint;
use petgraph::algo::min_spanning_tree;
use petgraph::data::FromElements;
use petgraph::graph::UnGraph;
use std::sync::Arc;

// TODO: consider http://vlsicad.ucsd.edu/Publications/Conferences/142/c142.ps

type MSTGraph = UnGraph<usize, f64>;

fn compute_mst(points: &Vec<geom::Point>) -> MSTGraph {
    let mut g = MSTGraph::new_undirected();

    // Add nodes to the graph and record the node indices;
    // indices maps the point index to a node index.
    let mut indices = Vec::with_capacity(points.len());
    for (idx, _) in points.iter().enumerate() {
        indices.push(g.add_node(idx));
    }

    for (a, b) in indices.iter().tuple_combinations() {
        let a_point = points[g[*a]];
        let b_point = points[g[*b]];
        g.add_edge(*a, *b, geom::manhattan_distance(&a_point, &b_point));
    }

    let mst = MSTGraph::from_elements(min_spanning_tree(&g));
    mst
}

fn mst_cost(g: &MSTGraph) -> f64 {
    let mut cost = 0.0;
    for edge in g.edge_indices() {
        if let Some(c) = g.edge_weight(edge) {
            cost += *c;
        }
    }
    cost
}

fn hanan_grid(points: &Vec<geom::Point>) -> Vec<geom::Point> {
    let mut hanan = Vec::new();

    for (i_point, j_point) in points.iter().tuple_combinations() {
        hanan.push(geom::Point::new(i_point.coords.x, j_point.coords.y));
        hanan.push(geom::Point::new(j_point.coords.x, i_point.coords.y));
    }

    hanan
}

fn intersects_obstacle(
    p: &geom::Point,
    via_shape: &geom::Shape,
    clearance: f64,
    obstacles: &Vec<Arc<Terminal>>,
) -> bool {
    let via = via_shape.translate_by_point(&p);
    obstacles
        .iter()
        .any(|t| t.shape.proximity(&via, clearance) != Disjoint)
}

/// Computes the set of 2nets for a given set of terminals.
/// This may introduce steiner points and hence may mutate the
/// input list of terminals.
/// The return value is a list of tuples holding the indices
/// in `terminals` of connected pairs.
pub fn compute_2nets(
    net_name: &String,
    terminals: &mut Vec<Arc<Terminal>>,
    all_layers: &LayerSet,
    all_pads: &Vec<Arc<Terminal>>,
    clearance: f64,
    via_shape: &geom::Shape,
) -> Vec<(Arc<Terminal>, Arc<Terminal>)> {
    let mut terminal_points = Vec::new();
    for t in terminals.iter() {
        terminal_points.push(t.shape.aabb().center());
    }

    // Use the minimum rectilinear spanning tree as a starting point
    let mut mst = compute_mst(&terminal_points);

    // Compute hanan grid, but avoid obstacles, including our own pads
    let h_grid: Vec<geom::Point> = hanan_grid(&terminal_points)
        .into_iter()
        .filter(|p| !intersects_obstacle(p, &via_shape, clearance, &all_pads))
        .collect();

    loop {
        let base_cost = mst_cost(&mst);

        let mut best: Option<(f64, Vec<geom::Point>, MSTGraph)> = None;

        // Now compute candidate locations for steiner points and evaluate
        // whether adding a candidate would reduce the cost of the mst.
        // We use the hanan grid for the candidate locations.
        let mut candidate_points = Vec::with_capacity(terminal_points.len() + 1);
        for hanan_pt in h_grid.iter() {
            candidate_points.clear();
            candidate_points.extend_from_slice(&terminal_points[..]);
            candidate_points.push(*hanan_pt);

            let g = compute_mst(&candidate_points);
            let cost = mst_cost(&g);

            if cost < base_cost {
                match best {
                    None => {
                        best = Some((cost, candidate_points.clone(), g));
                    }
                    Some((best_cost, _, _)) => {
                        if cost < best_cost {
                            best = Some((cost, candidate_points.clone(), g));
                        }
                    }
                }
            }
        }

        if best.is_none() {
            // Nothing changed; not further improvements are possible
            break;
        }

        // We found a better, lower cost arrangement.
        let (_, candidate_points, g) = best.unwrap();

        // prune out points that are no longer used
        let mut points = Vec::with_capacity(terminals.len() * 2);

        // We always want the input terminals
        points.extend_from_slice(&terminal_points[0..terminals.len()]);

        // We only want to preserve useful steiner points.  Those
        // are points that have a number of edges > 2.
        for a in g.node_indices() {
            let idx = g[a];
            if idx >= terminals.len() && g.edges(a).count() > 2 {
                points.push(candidate_points[idx].clone());
            }
        }

        let same = terminal_points == points;
        terminal_points = points;
        mst = compute_mst(&terminal_points);
        if same {
            // If we didn't actually mutate anything, we've run
            // out of things to do, so terminate the loop.
            break;
        }
    }

    // Add any steiner points that we created back into the set
    // of terminals.
    for pt in terminal_points.iter().skip(terminals.len()) {
        terminals.push(Arc::new(Terminal {
            identifier: None,
            net_name: Some(net_name.clone()),
            layers: all_layers.clone(),
            point: *pt,
            shape: via_shape.translate_by_point(&pt),
        }));
    }

    let mut twonets = Vec::new();

    for edge in mst.edge_indices() {
        if let Some((a, b)) = mst.edge_endpoints(edge) {
            twonets.push((
                Arc::clone(&terminals[mst[a]]),
                Arc::clone(&terminals[mst[b]]),
            ));
        }
    }

    twonets
}
