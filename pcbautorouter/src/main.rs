use std::process::exit;

#[macro_use]
extern crate nom;

use nalgebra as na;

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
use crate::polyoffset::JoinType;
use crate::progress::Progress;

use self::dsn::Pcb;
use clap::{App, Arg};
use std::error::Error;
use std::rc::Rc;
use std::sync::mpsc;
use std::sync::Arc;
use std::thread;

use crate::features::Terminal;
use crate::geom::OrderedPoint;
use crate::gui::run_gui;
use crate::layerpath::{CDTGraph, PathConfiguration};

#[allow(clippy::large_enum_variant)]
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

use spade::delaunay::{CdtEdge, ConstrainedDelaunayTriangulation, EdgeHandle};
use spade::kernels::FloatKernel;
type CDT = ConstrainedDelaunayTriangulation<OrderedPoint, FloatKernel>;

fn cdt_add_obstacle(cdt: &mut CDT, shape: &geom::Shape, clearance: f64) {
    // Add the polygon that describes the terminal boundaries

    let points = shape
        .buffer_and_simplify(
            clearance,
            JoinType::Round(clearance.abs() / 4.0),
            clearance.abs() / 4.0,
        )
        .compute_points();

    for i in 0..points.len() - 1 {
        let a = &points[i];
        let b = &points[i + 1];

        if a == b {
            continue;
        }
        cdt.add_constraint_edge(OrderedPoint::from_point(a), OrderedPoint::from_point(&b));
    }
}

fn cdt_add_pad(cdt: &mut CDT, shape: &geom::Shape, terminal_point: &geom::Point, clearance: f64) {
    // Add the polygon that describes the terminal boundaries

    let points = shape
        .buffer_and_simplify(
            clearance,
            JoinType::Round(clearance.abs() / 4.0),
            clearance.abs() / 4.0,
        )
        .compute_points();

    for pt in points.iter() {
        cdt.insert(OrderedPoint::from_point(&pt));

        // Make sure that the terminal center is reachable from
        // the pad outline
        cdt.add_constraint_edge(
            OrderedPoint::from_point(&pt),
            OrderedPoint::from_point(&terminal_point),
        );
    }
}

fn cdt_to_graph<F>(graph: &mut CDTGraph, cdt: &CDT, mut edge_cost: F)
where
    F: FnMut(&EdgeHandle<OrderedPoint, CdtEdge>) -> Option<f64>,
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
        let pb = Progress::new(
            "building layer assignment graphs",
            features.twonets_by_net.len(),
        );

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
    println!(
        "initial cfg cost is {} for {} paths",
        cfg.overall_cost,
        cfg.assignment.len()
    );

    features.paths_by_layer = cfg.extract_paths();
    notifier.send(ProgressUpdate::Feature(features.clone()));

    let mut improvement_pass = 1;

    while let Some(improved) = cfg.improve_one(improvement_pass) {
        cfg = improved;
        features.paths_by_layer = cfg.extract_paths();
        notifier.send(ProgressUpdate::Feature(features.clone()));
        improvement_pass += 1;
    }
    println!(
        "improved cfg cost is {} for {} paths in {} passes",
        cfg.overall_cost,
        cfg.assignment.len(),
        improvement_pass
    );

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
            let edge_cost = |edge: &EdgeHandle<OrderedPoint, CdtEdge>| {
                let from = &*edge.from();
                let to = &*edge.to();
                let a = from.point();
                let b = to.point();
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
        println!(
            "Starting path assignment for layer {} with {} twonets",
            layer,
            twonets.len()
        );

        let mut path_config = PathConfiguration::new(
            clearance / 100.0,
            Rc::clone(&cdt_graph),
            &twonets,
            &features.all_pads,
        );
        let order = path_config.compute_twonet_order();
        path_config.compute_path_from_order(&order);
        // path_config.initial_assignment();
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

fn go() -> Result<(), Box<dyn Error>> {
    let args = App::new("pcbautorouter")
        .author("Wez Furlong")
        .version("0.0.1")
        .about("Solves pcb track routing")
        .arg(
            Arg::with_name("dsn")
                .help("path to specctra design file")
                .long("dsn")
                .takes_value(true)
                .required(true),
        )
        .get_matches();

    let pcb = Pcb::parse(args.value_of("dsn").unwrap())?;

    let (tx, rx) = mpsc::channel();

    {
        let pcb_copy = pcb.clone();
        thread::spawn(move || compute_thread(&pcb_copy, Notify { tx }));
    }

    run_gui(&pcb, rx);
    Ok(())
}

fn main() {
    if let Err(e) = go() {
        println!("Error: {}", e.to_string());
        exit(1);
    }
}
