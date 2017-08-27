// This is here because petgraph doesn't offer a function to return the
// computed shortest path.  See https://github.com/bluss/petgraph/issues/140
// which raised the request to do so.

use std::collections::{HashMap, BinaryHeap};
use std::collections::hash_map::Entry::{Occupied, Vacant};
use std::hash::Hash;
use petgraph::visit::{Visitable, IntoEdges, EdgeRef};
use petgraph::algo::Measure;
use std::cmp::Ordering;

#[derive(Copy, Clone, Debug)]
pub struct MinScored<K, T>(pub K, pub T, pub usize);

impl<K: PartialOrd, T> PartialEq for MinScored<K, T> {
    #[inline]
    fn eq(&self, other: &MinScored<K, T>) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl<K: PartialOrd, T> Eq for MinScored<K, T> {}

impl<K: PartialOrd, T> PartialOrd for MinScored<K, T> {
    #[inline]
    fn partial_cmp(&self, other: &MinScored<K, T>) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K: PartialOrd, T> Ord for MinScored<K, T> {
    #[inline]
    fn cmp(&self, other: &MinScored<K, T>) -> Ordering {
        let a = &self.0;
        let b = &other.0;
        if a == b {
            // Ordering::Equal
            self.2.cmp(&other.2)
        } else if a < b {
            Ordering::Greater
        } else if a > b {
            Ordering::Less
        } else if a != a && b != b {
            // these are the NaN cases
            // Ordering::Equal
            self.2.cmp(&other.2)
        } else if a != a {
            // Order NaN less, so that it is last in the MinScore order
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

pub fn shortest_path<G, F, K>(graph: G,
                              start: G::NodeId,
                              goal: G::NodeId,
                              mut edge_cost: F,
                              cutoff: Option<K>)
                              -> Option<(K, Vec<G::NodeId>)>
    where G: IntoEdges + Visitable,
          G::NodeId: Eq + Hash,
          F: FnMut(G::EdgeRef) -> K,
          K: Measure + Copy
{
    let zero_score = K::default();
    let mut visit_next = BinaryHeap::new();
    let mut counter = 0;

    // backwards linkage through nodes
    let mut pred = HashMap::new();
    pred.insert(start, start);

    // node -> final distance
    let mut dist = HashMap::new();
    let mut seen = HashMap::new();

    seen.insert(start, zero_score);

    visit_next.push(MinScored(zero_score, start, counter));
    counter += 1;

    while let Some(MinScored(node_score, node, _)) = visit_next.pop() {
        match dist.entry(node) {
            Occupied(_) => {
                // Already searched this node
                continue;
            }
            Vacant(ent) => {
                *ent.insert(node_score);
            }
        }
        if goal == node {
            break;
        }

        for edge in graph.edges(node) {
            let next = edge.target();

            let cost = edge_cost(edge);
            let total_dist = dist[&node] + cost;

            if let Some(cutoff) = cutoff {
                if total_dist > cutoff {
                    continue;
                }
            }

            if !dist.contains_key(&next) {
                match seen.entry(next) {
                    Occupied(ent) => {
                        if total_dist < *ent.get() {
                            *ent.into_mut() = total_dist;
                            pred.insert(next, node);
                            visit_next.push(MinScored(total_dist, next, counter));
                            counter += 1;
                        }
                    }
                    Vacant(ent) => {
                        *ent.insert(total_dist);
                        pred.insert(next, node);
                        visit_next.push(MinScored(total_dist, next, counter));
                        counter += 1;
                    }
                }
            }
        }
    }

    if !pred.contains_key(&goal) {
        None
    } else {
        // Now convert the maps into the cost and the path
        let mut n = goal;
        let mut cost = zero_score;
        let mut path = Vec::new();
        loop {
            cost = cost + dist[&n];
            path.push(n);

            if n == start {
                break;
            }

            n = pred[&n];
        }
        path.reverse();
        Some((cost, path))
    }
}
