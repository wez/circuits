use petgraph::algo::min_spanning_tree;
use petgraph::graph::UnGraph;
use petgraph::data::FromElements;
use features::Terminal;
use itertools::Itertools;
extern crate nalgebra as na;

/// Computes the set of 2nets for a given set of terminals.
/// This may introduce steiner points and hence may mutate the
/// input list of terminals.
/// The return value is a list of tuples holding the indices
/// in `terminals` of connected pairs.
pub fn compute_2nets(terminals: &mut Vec<Terminal>) -> Vec<(usize, usize)> {
    let mut g = UnGraph::<usize, f64>::new_undirected();
    let mut indices = Vec::new();

    for (idx, _) in terminals.iter().enumerate() {
        indices.push(g.add_node(idx));
    }

    for (a, b) in indices.iter().tuple_combinations() {
        let a_point = terminals[g[*a]].shape.aabb().center();
        let b_point = terminals[g[*b]].shape.aabb().center();
        g.add_edge(*a, *b, na::distance(&a_point, &b_point));
    }

    let mut twonets = Vec::new();

    let mst = UnGraph::<usize, f64>::from_elements(min_spanning_tree(&g));

    for edge in mst.edge_indices() {
        if let Some((a, b)) = mst.edge_endpoints(edge) {
            twonets.push((mst[a], mst[b]));
        }
    }

    twonets
}
