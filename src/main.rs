#[macro_use]
extern crate error_chain;
#[macro_use]
extern crate maplit;
use std::process::exit;

#[macro_use]
extern crate nom;

extern crate clap;
extern crate geo;
extern crate indicatif;
extern crate itertools;
extern crate ncollide;
extern crate petgraph;
extern crate ordered_float;
extern crate piston_window;
extern crate spade;
extern crate nalgebra as na;

mod dijkstra;
mod dsn;
mod features;
mod gui;

#[allow(dead_code)]
mod geom;
mod layerassign;
mod layerpath;
mod twonets;

#[allow(dead_code)]
mod polyoffset;

mod progress;
use progress::Progress;
use polyoffset::JoinType;

use std::sync::mpsc;
use std::thread;
use clap::{App, Arg};
use std::error::Error;
use self::dsn::Pcb;
use std::rc::Rc;
use std::sync::Arc;

use features::Terminal;
use ncollide::query::Proximity::Disjoint;
use layerpath::{PathConfiguration, CDTGraph};
use spade::delaunay::Subdivision;
use geom::OrderedPoint;
use gui::run_gui;

pub enum ProgressUpdate {
    Feature(features::Features),
    Done(),
}

/// Helper for waking up the GUI thread as we generate
/// new information.
struct Notify {
    tx: mpsc::Sender<ProgressUpdate>,
}

impl Notify {
    fn send(&self, update: ProgressUpdate) {
        self.tx.send(update).unwrap();
    }
}

use spade::delaunay::{ConstrainedDelaunayTriangulation, EdgeHandle};
use spade::kernels::FloatKernel;
type CDT = ConstrainedDelaunayTriangulation<OrderedPoint, FloatKernel>;

fn cdt_add_obstacle(cdt: &mut CDT, shape: &geom::Shape, clearance: f64) {

    // Add the polygon that describes the terminal boundaries

    let points = shape
        .buffer_and_simplify(clearance,
                             JoinType::Round(clearance.abs() / 4.0),
                             clearance.abs() / 4.0)
        .compute_points();

    for i in 0..points.len() - 1 {
        let a = &points[i];
        let b = &points[i + 1];

        if a == b {
            continue;
        }
        cdt.add_new_constraint_edge(OrderedPoint::from_point(a), OrderedPoint::from_point(&b));
    }
}

fn cdt_add_pad(cdt: &mut CDT, shape: &geom::Shape, terminal_point: &geom::Point, clearance: f64) {
    // Add the polygon that describes the terminal boundaries

    let points = shape
        .buffer_and_simplify(clearance,
                             JoinType::Round(clearance.abs() / 4.0),
                             clearance.abs() / 4.0)
        .compute_points();

    for pt in points.iter() {
        cdt.insert(OrderedPoint::from_point(&pt));

        // Make sure that the terminal center is reachable from
        // the pad outline
        cdt.add_new_constraint_edge(OrderedPoint::from_point(&pt),
                                    OrderedPoint::from_point(&terminal_point));
    }
}

fn cdt_to_graph<F>(graph: &mut CDTGraph, cdt: &CDT, mut edge_cost: F)
    where F: FnMut(&EdgeHandle<OrderedPoint>) -> Option<f64>
{
    for edge in cdt.edges() {
        if let Some(cost) = edge_cost(&edge) {
            let from = &*edge.from();
            let to = &*edge.to();
            graph.add_edge(*from, *to, cost);
        }
    }
}


/// This is where we initiate the heavy lifting.
/// We do this separately from the UI thread so that we can incrementally
/// update the UI as we compute the data.
fn compute_thread(pcb: &Pcb, notifier: Notify) {
    let mut features = features::Features::from_pcb(&pcb);
    notifier.send(ProgressUpdate::Feature(features.clone()));

    let clearance = pcb.structure.rule.clearance;
    let mut cfg = layerassign::SharedConfiguration::new(&features.all_layers, clearance);
    let mut cdt = CDT::new();
    let mut cdt_graph = CDTGraph::new();

    {
        let pb = Progress::new("building layer assignment graphs",
                               features.twonets_by_net.len());

        for (_, twonets) in pb.wrap_iter(features.twonets_by_net.iter()) {
            for &(ref a, ref b) in twonets {
                cfg.add_twonet(a, b, &features.via_shape);
                cdt.insert(OrderedPoint::from_point(&a.point));
                cdt.insert(OrderedPoint::from_point(&b.point));
                cdt_add_pad(&mut cdt, &a.shape, &a.point, clearance);
                cdt_add_pad(&mut cdt, &b.shape, &b.point, clearance);
            }
        }
    }

    {
        let pb = Progress::spinner("triangulating stage 1");

        // Now add constraints to the CDT for each of the obstacles
        for shape in pcb.structure.boundary.iter() {
            pb.inc();
            let term = Arc::new(Terminal {
                                    identifier: None,
                                    net_name: None,
                                    layers: features.all_layers.clone(),
                                    shape: shape.shape.clone(),
                                    point: shape.shape.aabb().center(),
                                });
            cfg.add_terminal(&term);
            // Negative clearance so that we fit inside the boundary
            cdt_add_obstacle(&mut cdt, &term.shape, -clearance);
        }
        for obs in features.obstacles.iter() {
            pb.inc();
            cfg.add_terminal(&obs);
            // Positive clearance so that we go around the obstacle
            cdt_add_obstacle(&mut cdt, &obs.shape, clearance);
        }
    }

    let mut cfg = layerassign::Configuration::new(&Rc::new(cfg));

    cfg.initial_assignment();
    println!("initial cfg cost is {} for {} paths",
             cfg.overall_cost,
             cfg.assignment.len());

    features.paths_by_layer = cfg.extract_paths();
    notifier.send(ProgressUpdate::Feature(features.clone()));

    loop {
        if let Some(improved) = cfg.improve_one() {
            cfg = improved;
            println!("improved cfg cost is {}", cfg.overall_cost);
            features.paths_by_layer = cfg.extract_paths();
            notifier.send(ProgressUpdate::Feature(features.clone()));
        } else {
            break;
        }
    }

    {
        let pb = Progress::spinner("triangulating stage 2");

        // Add in points for the resolved twonet paths
        for (_, twonets) in features.paths_by_layer.iter() {
            for &(ref a, ref b) in twonets.iter() {
                cdt.insert(*a);
                cdt.insert(*b);
            }
        }

        {
            let edge_cost = |edge: &EdgeHandle<OrderedPoint>| {
                let from = &*edge.from();
                let to = &*edge.to();

                let a = from.point();
                let b = to.point();
                let line = geom::Shape::line(&a, &b);

                // Elide lines that cross the obstacles.  The CDT seems to
                // falsely include these.  Whether I'm misunderstanding how
                // to use it or not, it's worth processing the graph to ensure
                // that they are not included in the result.
                for obs in features.obstacles.iter() {
                    if line.proximity(&obs.shape, clearance / 2.0) != Disjoint {
                        // return None;
                    }
                }

                Some(na::distance(&a, &b))
            };

            cdt_to_graph(&mut cdt_graph, &cdt, edge_cost);
        }

        for (a, b, _) in cdt_graph.all_edges() {
            pb.inc();
            features.cdt_edges.push((a.point(), b.point()));
        }
        notifier.send(ProgressUpdate::Feature(features.clone()));
    }


    let cdt_graph = Rc::new(cdt_graph);
    let paths_by_layer = features.paths_by_layer.clone();

    for (layer, twonets) in paths_by_layer.iter() {
        println!("Starting path assignment for layer {} with {} twonets",
                 layer,
                 twonets.len());

        let mut path_config = PathConfiguration::new(clearance / 100.0,
                                                     Rc::clone(&cdt_graph),
                                                     &twonets,
                                                     &features.all_pads);
        path_config.initial_assignment();
        if let Some(p) = features.paths_by_layer.get_mut(layer) {
            *p = path_config.get_paths().clone();
        }
        notifier.send(ProgressUpdate::Feature(features.clone()));
    }

    println!("All Done");

    // Send the Done notification.  This sets the GUI to lazy mode
    // which effectively prevents it from being woken up by us in
    // the future, but turns the CPU utilization way down.
    notifier.send(ProgressUpdate::Done());
}

fn go() -> Result<(), Box<Error>> {
    let args = App::new("pcbautorouter")
        .author("Wez Furlong")
        .version("0.0.1")
        .about("Solves pcb track routing")
        .arg(Arg::with_name("dsn")
                 .help("path to specctra design file")
                 .long("dsn")
                 .takes_value(true)
                 .required(true))
        .get_matches();

    let pcb = Pcb::parse(args.value_of("dsn").unwrap())?;

    let (tx, rx) = mpsc::channel();

    {
        let pcb_copy = pcb.clone();
        thread::spawn(move || compute_thread(&pcb_copy, Notify { tx: tx }));
    }

    run_gui(&pcb, rx);
    Ok(())
}

fn main() {
    match go() {
        Err(e) => {
            println!("Error: {}", e.to_string());
            exit(1);
        }
        _ => {}
    }
}
