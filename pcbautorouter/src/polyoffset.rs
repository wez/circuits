// This is derived from the Clipper library which is
// Copyright 2000-2017 Angus Johnson
// and was made available under the Boost Software License v1
// http://www.boost.org/LICENSE_1_0.txt.

use itertools::Itertools;
use nalgebra as na;

type Point = na::Point2<f64>;

/// Describes how to join points when buffering
pub enum JoinType {
    /// Convex edges will be squared at 1x delta
    Square,
    /// Allow edges to come to point, unless they exceed the specified
    /// mitre limit (which is expressed as multiples of delta), in which
    /// case the edges are Square'd off.
    Miter(f64),
    /// The edge is rounded off with an approximate arc.  The parameter
    /// specifies the arc tolerance, which should be a reasonable fraction
    /// of the delta to be used in buffering.  Smaller values of the tolerance
    /// parameter result in smoother arcs but yield more vertices in the
    /// output polygon.
    Round(f64),
}

// Holds pre-computed parameters to be used by Round joins
struct RoundParams {
    steps_per_rad: f64,
    m_cos: f64,
    m_sin: f64,
}

impl RoundParams {
    fn new(delta: f64, tolerance: f64) -> RoundParams {
        const TOLERANCE: f64 = 0.25;
        use std::f64::consts::PI;

        let tolerance = if tolerance <= 0.0 {
            TOLERANCE
        } else {
            tolerance.min(delta.abs() * TOLERANCE)
        };

        let steps = (PI / (1.0 - tolerance / delta.abs()).acos()).min(delta.abs() * PI);
        let (mut m_sin, m_cos) = (2.0 * PI / steps).sin_cos();
        let steps_per_rad = steps / (2.0 * PI);

        if delta < 0.0 {
            m_sin = -m_sin;
        }
        RoundParams {
            steps_per_rad,
            m_cos,
            m_sin,
        }
    }
}

// Internal equivalent to JoinType.  Holds pre-computed values
// derived from JoinType + delta parameter.
enum AdjustedJoinType {
    Square,
    Miter(f64),
    Round(RoundParams),
}

impl AdjustedJoinType {
    fn new(delta: f64, join_type: JoinType) -> AdjustedJoinType {
        match join_type {
            JoinType::Miter(limit) => {
                let limit = limit.max(2.0);
                let adjusted = 2.0 / (limit * limit);
                AdjustedJoinType::Miter(adjusted)
            }
            JoinType::Square => AdjustedJoinType::Square,
            JoinType::Round(tolerance) => {
                AdjustedJoinType::Round(RoundParams::new(delta, tolerance))
            }
        }
    }
}

fn area(points: &[Point]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;

    for i in 0..points.len() {
        let j = if i == 0 { points.len() - 1 } else { i - 1 };
        area +=
            (points[j].coords.x + points[i].coords.x) * (points[j].coords.y - points[i].coords.y);
    }

    area * -0.5
}

fn orientation(points: &[Point]) -> bool {
    area(points) >= 0.0
}

fn unit_normal(a: &Point, b: &Point) -> Point {
    if a == b {
        return Point::new(0.0, 0.0);
    }

    let dx = b.coords.x - a.coords.x;
    let dy = b.coords.y - a.coords.y;
    let f = 1.0 / (dx * dx + dy * dy).sqrt();

    Point::new(dy * f, dx * f * -1.0)
}

fn compute_norms(points: &[Point]) -> Vec<Point> {
    let mut norms = Vec::new();

    for i in 0..points.len() - 1 {
        norms.push(unit_normal(&points[i], &points[i + 1]));
    }
    norms.push(unit_normal(&points[points.len() - 1], &points[0]));

    norms
}

fn cross2d(j: &Point, k: &Point) -> f64 {
    (k.coords.x * j.coords.y) - (j.coords.x * k.coords.y)
}

fn dot2d(j: &Point, k: &Point) -> f64 {
    k.coords.x * j.coords.x + j.coords.y * k.coords.y
}

fn square(
    delta: f64,
    j: usize,
    k: usize,
    points: &[Point],
    norms: &[Point],
    sin_a: f64,
    output: &mut Vec<Point>,
) {
    let dx = (sin_a.atan2(dot2d(&norms[j], &norms[k])) / 4.0).tan();
    let src = &points[j];
    output.push(Point::new(
        src.coords.x + delta * (norms[k].coords.x - norms[k].coords.y * dx),
        src.coords.y + delta * (norms[k].coords.y + norms[k].coords.x * dx),
    ));
    output.push(Point::new(
        src.coords.x + delta * (norms[j].coords.x - norms[j].coords.y * dx),
        src.coords.y + delta * (norms[j].coords.y + norms[j].coords.x * dx),
    ));
}

fn mitre(
    delta: f64,
    j: usize,
    k: usize,
    r: f64,
    points: &[Point],
    norms: &[Point],
    output: &mut Vec<Point>,
) {
    let q = delta / r;
    output.push(Point::new(
        points[j].coords.x + (norms[k].x + norms[j].x) * q,
        points[j].coords.y * (norms[k].y + norms[j].y) * q,
    ));
}

#[allow(clippy::too_many_arguments, clippy::many_single_char_names)]
fn round(
    delta: f64,
    j: usize,
    k: usize,
    points: &[Point],
    norms: &[Point],
    sin_a: f64,
    params: &RoundParams,
    output: &mut Vec<Point>,
) {
    let a = sin_a.atan2(dot2d(&norms[j], &norms[k]));
    let steps = (params.steps_per_rad * a.abs()).round().max(1.0) as usize;

    let mut x = norms[k].coords.x;
    let mut y = norms[k].coords.y;

    let src = &points[j];

    for _ in 0..steps {
        output.push(Point::new(
            src.coords.x + x * delta,
            src.coords.y + y * delta,
        ));

        let x2 = x;
        x = x * params.m_cos - params.m_sin * y;
        y = x2 * params.m_sin + y * params.m_cos;
    }

    output.push(Point::new(
        src.coords.x + norms[j].x * delta,
        src.coords.y + norms[j].y * delta,
    ));
}

/// Performs a buffering operation on the points of a polygon.
/// The polygon is assumed to be closed (eg: the last point is connected to
/// the first point).  The last point may optionally be the same as the first
/// point in the input slice but it is not required to be physically present.
/// Positive delta values increase the size of the polygon whereas negative
/// delta values shrink it.  The join parameter describes how edges are
/// handled in the resultant polygon.
/// Returns a vector holding the points of the buffered polygon.
pub fn buffer(input_points: &[Point], delta: f64, join: JoinType) -> Vec<Point> {
    let join_type = AdjustedJoinType::new(delta, join);

    let mut point_slice = input_points;

    // polygon paths are typically closed, with the final element
    // being the same as the first element.  Remove that final
    // element.
    while point_slice.len() > 1 && point_slice[0] == point_slice[point_slice.len() - 1] {
        let new_len = point_slice.len() - 1;
        point_slice = &point_slice[0..new_len];
    }

    // Remove sequences of duplicate points
    let mut points: Vec<Point> = point_slice.iter().copied().dedup().collect();

    // Fixup orientation of the points if they are counter to what we
    // require for this algorithm to work.
    if !orientation(&points) {
        points.reverse();
    }

    let norms = compute_norms(&points);

    let mut output = Vec::new();

    let mut k = points.len() - 1;

    for j in 0..points.len() {
        let mut sin_a = cross2d(&norms[j], &norms[k]);
        if (sin_a * delta).abs() < 1.0 {
            let cos_a = dot2d(&norms[j], &norms[k]);
            if cos_a > 0.0 {
                // angle -> 0 degrees
                output.push(Point::new(
                    points[j].coords.x + norms[k].x * delta,
                    points[j].coords.y + norms[k].y * delta,
                ));
                continue;
            }
        // angle -> 180 degrees
        } else if sin_a > 1.0 {
            sin_a = 1.0;
        } else if sin_a < -1.0 {
            sin_a = -1.0;
        }

        if sin_a * delta < 0.0 {
            let point = Point::new(
                points[j].coords.x + norms[k].x * delta,
                points[j].coords.y + norms[k].y * delta,
            );
            output.push(point.clone());
            output.push(points[j].clone());
            output.push(point);
        } else {
            match join_type {
                AdjustedJoinType::Miter(limit) => {
                    let r = 1.0 + dot2d(&norms[j], &norms[k]);
                    if r >= limit {
                        mitre(delta, j, k, r, &points, &norms, &mut output);
                    } else {
                        square(delta, j, k, &points, &norms, sin_a, &mut output);
                    }
                }
                AdjustedJoinType::Square => {
                    square(delta, j, k, &points, &norms, sin_a, &mut output);
                }
                AdjustedJoinType::Round(ref params) => {
                    round(delta, j, k, &points, &norms, sin_a, &params, &mut output);
                }
            }
        }

        k = j;
    }

    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn as_raw_points(points: &Vec<Point>) -> Vec<[f64; 2]> {
        points.iter().map(|p| [p.coords.x, p.coords.y]).collect()
    }

    fn from_raw_points(points: &[[f64; 2]]) -> Vec<Point> {
        points.iter().map(|p| Point::new(p[0], p[1])).collect()
    }

    #[test]
    fn buffer_square() {
        let input_square = from_raw_points(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
        let output_square = buffer(&input_square, 1.0, JoinType::Miter(1.0));
        let expected_square = vec![[-1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [-1.0, 1.0]];
        assert_eq!(expected_square, as_raw_points(&output_square));
    }
}
