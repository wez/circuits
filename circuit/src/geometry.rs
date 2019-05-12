use geo::prelude::*;
use geo::{Geometry, GeometryCollection, MultiPolygon, Polygon};

/// Convert a collection into a vector of its constituent polygons
pub fn iter_polygons(collection: GeometryCollection<f64>) -> Vec<Polygon<f64>> {
    let mut polys = vec![];
    for geom in collection {
        match geom {
            Geometry::GeometryCollection(collection) => {
                polys.append(&mut iter_polygons(collection));
            }
            Geometry::Line(line) => {
                polys.push(Polygon::new(vec![line.start, line.end].into(), vec![]))
            }
            Geometry::Polygon(poly) => polys.push(poly),
            _ => {
                println!("BOO: {:?}", geom);
            }
        }
    }
    polys
}

pub fn convex_hull_collection(collection: GeometryCollection<f64>) -> Polygon<f64> {
    let multi: MultiPolygon<f64> = iter_polygons(collection).into_iter().collect();
    multi.convex_hull()
}

pub fn convex_hull(geom: Geometry<f64>) -> Polygon<f64> {
    match geom {
        Geometry::GeometryCollection(collection) => convex_hull_collection(collection),
        _ => panic!("only GeometryCollection is supported"),
    }
}
