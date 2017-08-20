use petgraph::algo::min_spanning_tree;
use petgraph::graph::UnGraph;
use petgraph::data::FromElements;
use features::Terminal;
use itertools::Itertools;
use geom;
use std::collections::{HashMap, HashSet};
extern crate nalgebra as na;

type MSTGraph = UnGraph<usize, f64>;

fn compute_mst(points: &Vec<geom::Point>) -> MSTGraph {
    let mut g = MSTGraph::new_undirected();
    let mut indices = Vec::new();
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

    for (i, i_point) in points.iter().enumerate() {
        for (j, j_point) in points.iter().enumerate() {
            if i != j {
                hanan.push(geom::Point::new(i_point.coords.x, j_point.coords.y));
                hanan.push(geom::Point::new(j_point.coords.x, i_point.coords.y));
            }
        }
    }

    hanan
}

/// Computes the set of 2nets for a given set of terminals.
/// This may introduce steiner points and hence may mutate the
/// input list of terminals.
/// The return value is a list of tuples holding the indices
/// in `terminals` of connected pairs.
pub fn compute_2nets(net_name: &String, terminals: &mut Vec<Terminal>) -> Vec<(usize, usize)> {
    let mut terminal_points = Vec::new();
    for t in terminals.iter() {
        terminal_points.push(t.shape.aabb().center());
    }

    // Use the minimum rectilinear spanning tree as a starting point
    let mut mst = compute_mst(&terminal_points);
    loop {
        let base_cost = mst_cost(&mst);

        let mut best: Option<(f64, Vec<geom::Point>, MSTGraph)> = None;

        // Now compute candidate locations for steiner points and evaluate
        // whether adding a candidate would reduce the cost of the mst.
        // We use the hanan grid for the candidate locations.
        for hanan_pt in hanan_grid(&terminal_points) {
            let mut candidate_points = Vec::with_capacity(terminal_points.len() + 1);
            for p in terminal_points.iter() {
                candidate_points.push(p.clone());
            }
            candidate_points.push(hanan_pt);

            let g = compute_mst(&candidate_points);
            let cost = mst_cost(&g);

            if cost < base_cost {
                match best {
                    None => {
                        best = Some((cost, candidate_points, g));
                    }
                    Some((best_cost, _, _)) => {
                        if cost < best_cost {
                            best = Some((cost, candidate_points, g));
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
        let mut points = Vec::new();

        // We always want the input terminals
        for (idx, _) in terminals.iter().enumerate() {
            if idx >= terminals.len() {
                break;
            }
            points.push(terminal_points[idx].clone());
        }

        // We only want to preserve useful steiner points.  Those
        // are points that have a number of edges > 2.
        // This map is used to count the edge degree.
        let mut extras: HashMap<usize, usize> = HashMap::new();
        let bump = |extras: &mut HashMap<usize, usize>, idx: usize| if idx >= terminals.len() {
            let degree = extras.get(&idx).unwrap_or(&0) + 1;
            extras.insert(idx, degree);
        };

        for edge in g.edge_indices() {
            if let Some((a, b)) = g.edge_endpoints(edge) {
                bump(&mut extras, g[a]);
                bump(&mut extras, g[b]);
            }
        }

        for (idx, degree) in extras {
            if degree > 2 {
                // Keep the useful steiner points
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

    let mut twonets = Vec::new();

    for edge in mst.edge_indices() {
        if let Some((a, b)) = mst.edge_endpoints(edge) {
            twonets.push((mst[a], mst[b]));
        }
    }

    // Add any steiner points that we created back into the set
    // of terminals.
    for pt in terminal_points.iter().skip(terminals.len()) {
        terminals
            .push(Terminal {
                      identifier: None,
                      net_name: Some(net_name.clone()),
                      layers: HashSet::new(),
                      shape:
                          geom::Shape::circle(200.0,
                                              geom::Location::new(geom::Vector::new(pt.coords.x,
                                                                                    pt.coords.y),
                                                                  0.0)),
                  });
    }

    twonets
}
