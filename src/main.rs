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

mod dijkstra;
mod dsn;
mod features;
mod geom;
mod layerassign;
mod twonets;
mod progress;
use progress::Progress;

use std::sync::mpsc;
use std::thread;
use clap::{App, Arg};
use std::error::Error;
use self::dsn::Pcb;
use piston_window::types::Color;

/// Holds pre-transformed lines ready to be rendered.
/// The PCB can have a lot of discrete lines (especially with
/// circular pads) which takes some time to transform and build
/// up all the lines.  This caches those lines for a given size
/// of window.  It does still take some cycles to render it to
/// the raw context though.
struct LinesToRender {
    height: u32,
    width: u32,
    lines: Vec<(Color, [f64; 4])>,
}

impl LinesToRender {
    fn reset(&mut self) {
        self.height = 0;
        self.width = 0;
        self.lines.clear();
    }

    fn reset_if_changed(&mut self, height: u32, width: u32) -> bool {
        if self.height != height || self.width != width {
            self.height = height;
            self.width = width;
            self.lines.clear();
            true
        } else {
            false
        }
    }

    fn polygon(&mut self, color: Color, shape: &geom::Shape) {
        let poly = shape
            .handle
            .as_shape::<geom::Polyline>()
            .expect("transform always yields a polygon!?");

        let points = poly.vertices();

        for i in 0..points.len() - 1 {
            let a = &points[i];
            let b = &points[i + 1];
            self.lines
                .push((color, [a.coords.x, a.coords.y, b.coords.x, b.coords.y]));
        }
    }
}

enum ProgressUpdate {
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

/// This is where we initiate the heavy lifting.
/// We do this separately from the UI thread so that we can incrementally
/// update the UI as we compute the data.
fn compute_thread(pcb: &Pcb, notifier: Notify) {
    let features = features::Features::from_pcb(&pcb);
    notifier.send(ProgressUpdate::Feature(features.clone()));

    let mut cfg = layerassign::Configuration::new(&features.all_layers);

    {
        let pb = Progress::new("building layer assignment graphs",
                               features.twonets_by_net.len());

        for (_, twonets) in pb.wrap_iter(features.twonets_by_net.iter()) {
            for &(ref a, ref b) in twonets {
                cfg.add_twonet(a, b);
            }
        }
    }

    cfg.initial_assignment();


    // Send the Done notification.  This sets the GUI to lazy mode
    // which effectively prevents it from being woken up by us in
    // the future, but turns the CPU utilization way down.
    notifier.send(ProgressUpdate::Done());
}

fn draw_gui(window: &mut piston_window::PistonWindow,
            e: &piston_window::Event,
            pcb: &Pcb,
            features: &Option<features::Features>,
            lines: &mut LinesToRender) {
    use piston_window::*;
    use piston_window::types::Color;

    const DARK_CHARCOAL: Color = [46.0 / 255.0, 52.0 / 255.0, 54.0 / 255.0, 1.0];
    const LIGHT_BLUE: Color = [114.0 / 255.0, 159.0 / 255.0, 207.0 / 255.0, 1.0];
    const DARK_GREEN: Color = [78.0 / 255.0, 154.0 / 255.0, 6.0 / 255.0, 1.0];
    const RED: Color = [204.0 / 255.0, 0.0, 0.0, 1.0];
    const YELLOW: Color = [237.0 / 255.0, 212.0 / 255.0, 0.0, 1.0];
    const LIGHT_PURPLE: Color = [173.0 / 255.0, 127.0 / 255.0, 168.0 / 255.0, 1.0];
    let size = window.size();

    window.draw_2d(e, |c, g| {
        // pixel padding in the window, so that there is a gap between
        // the window border and the pcb boundary
        let padding = 40f64;
        let t = c.transform
            .flip_v()
            .trans(padding / 2.0, -(size.height as f64) + padding / 2.0);

        if lines.reset_if_changed(size.height, size.width) {
            // First, compute the overall bounds of the shapes we want to render.
            let bounds = {
                let mut bounds = None;
                for shape in pcb.structure.boundary.iter() {
                    use ncollide::bounding_volume::BoundingVolume;
                    let b = shape.shape.aabb();
                    if bounds.is_some() {
                        bounds = Some(b.merged(&bounds.unwrap()));
                    } else {
                        bounds = Some(b);
                    }
                }
                bounds.unwrap()
            };
            // That gives us the overall width and height
            // in the pcb coordinate system.
            let p_width = bounds.maxs().coords.x - bounds.mins().coords.x;
            let p_height = bounds.maxs().coords.y - bounds.mins().coords.y;
            let x_offset = -bounds.mins().coords.x;
            let y_offset = -bounds.mins().coords.y;

            // Figure out how much we want to scale; we want to fix the zoom
            // to show 100% of the circuit at all times, so we sanity check
            // against both the width and the height.
            let mut factor = (size.height as f64 - padding) / p_height;
            if p_width * factor > size.width as f64 {
                factor = (size.width as f64 - padding) / p_width;
            }
            let scale = geom::Similarity::new(geom::Vector::new(0.0, 0.0), 0.0, factor) *
                        geom::Similarity::new(geom::Vector::new(x_offset, y_offset), 0.0, 1.0);

            // boundary in light blue
            for shape in pcb.structure.boundary.iter() {
                lines.polygon(LIGHT_BLUE, &shape.shape.transform(&scale));
            }

            // components
            for comp in pcb.components.iter() {
                let def = &pcb.component_defs[&comp.component_type];

                for outline in def.outlines.iter() {
                    let s = outline
                        .shape
                        .translate(&comp.position)
                        .transform(&scale);
                    lines.polygon(DARK_GREEN, &s);
                }
            }

            if let Some(ref features) = *features {
                for obs in features.obstacles.iter() {
                    lines.polygon(RED, &obs.shape.transform(&scale));
                }

                for (_, terminals) in features.terminals_by_net.iter() {
                    for terminal in terminals.iter() {
                        lines.polygon(YELLOW, &terminal.shape.transform(&scale));
                    }
                }

                for (_, twonets) in features.twonets_by_net.iter() {
                    for &(ref a, ref b) in twonets.iter() {
                        let points = vec![a.shape.aabb().center(), b.shape.aabb().center()];
                        let poly = geom::Shape::polygon(points, geom::origin(), None)
                            .transform(&scale);
                        lines.polygon(LIGHT_PURPLE, &poly);
                    }
                }
            }
        }

        clear(DARK_CHARCOAL, g);
        for &(color, line) in lines.lines.iter() {
            piston_window::line(color, 0.5, line, t, g);
        }
    });
}

fn run_gui(pcb: &Pcb, rx: mpsc::Receiver<ProgressUpdate>) {
    use piston_window::*;
    let mut features: Option<features::Features> = None;
    let mut lines = LinesToRender {
        height: 0,
        width: 0,
        lines: Vec::new(),
    };

    let mut window: PistonWindow = WindowSettings::new("PCB Autorouter", [700; 2])
        .exit_on_esc(true)
        .build()
        .unwrap();

    // We don't need to be so aggressive with our frame rate, so
    // dial down the update and frame rates.
    window.set_ups(0);
    window.set_max_fps(1);
    window.set_lazy(false);

    while let Some(e) = window.next() {
        match e {
            Event::Loop(l) => {
                match l {
                    Loop::Render(_) => {
                        draw_gui(&mut window, &e, &pcb, &features, &mut lines);
                    }
                    Loop::Idle(_) => {
                        if let Ok(progress) = rx.try_recv() {
                            match progress {
                                ProgressUpdate::Feature(f) => {
                                    features = Some(f);
                                    lines.reset();
                                }
                                ProgressUpdate::Done() => {
                                    window.set_lazy(true);
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }
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
        thread::spawn(move || { compute_thread(&pcb_copy, Notify { tx: tx }); });
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
