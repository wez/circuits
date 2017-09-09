use spade::delaunay::{ConstrainedDelaunayTriangulation, EdgeHandle};
use spade::kernels::FloatKernel;
use spade::HasPosition;
use petgraph::graphmap::{UnGraphMap, NodeTrait};
extern crate nalgebra as na;
use spade::delaunay::Subdivision;

use geom::{Point, OrderedPoint};

#[derive(Debug, Eq, PartialEq, Hash, Clone, Copy, Ord, PartialOrd)]
pub struct Vertex<Value> {
    pub p: OrderedPoint,
    pub value: Value,
}

impl<Value> Vertex<Value> {
    pub fn new(point: &Point, value: Value) -> Vertex<Value> {
        Vertex {
            p: OrderedPoint::from_point(point),
            value: value,
        }
    }

    pub fn point(&self) -> Point {
        self.p.point()
    }
}

impl<Value> HasPosition for Vertex<Value> {
    type Point = [f64; 2];
    fn position(&self) -> [f64; 2] {
        [self.p.x.into(), self.p.y.into()]
    }
}

pub type CDT<Value> = ConstrainedDelaunayTriangulation<Vertex<Value>, FloatKernel>;
pub type CDTUnGraph<Value> = UnGraphMap<Vertex<Value>, f64>;

pub fn cdt_to_graph<Value, F>(cdt: &CDT<Value>, mut edge_cost: F) -> CDTUnGraph<Value>
    where Value: NodeTrait,
          F: FnMut(&EdgeHandle<Vertex<Value>>) -> Option<f64>
{
    let mut g = CDTUnGraph::<Value>::new();

    for edge in cdt.edges() {
        if let Some(cost) = edge_cost(&edge) {
            let from = &*edge.from();
            let to = &*edge.to();

            g.add_node(*from);
            g.add_node(*to);
            g.add_edge(*from, *to, cost);
        }
    }

    g
}
