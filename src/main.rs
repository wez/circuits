#[macro_use]
extern crate error_chain;
use std::process::exit;

#[macro_use]
extern crate nom;

extern crate clap;
extern crate conrod;
extern crate itertools;
extern crate ncollide;
extern crate petgraph;
extern crate ordered_float;

mod dsn;
mod features;
mod geom;
mod layerassign;
mod twonets;

use conrod::backend::glium::glium::{self, Surface};
use std::sync::mpsc;
use std::thread;
use clap::{App, Arg};

const WIDTH: u32 = 700;
const HEIGHT: u32 = 700;

use std::error::Error;
use self::dsn::Pcb;

enum ProgressUpdate {
    Feature(features::Features),
}

fn build_ui(pcb: &Pcb,
            features: &Option<features::Features>,
            prior: Option<&conrod::Ui>)
            -> conrod::Ui {
    let dim = if let Some(prior) = prior {
        [prior.win_w, prior.win_h]
    } else {
        [WIDTH as f64, HEIGHT as f64]
    };
    let mut ui = conrod::UiBuilder::new(dim).build();

    // Generate the widget identifiers.
    let mut shape_ids = Vec::new();
    {
        let mut ids = ui.widget_id_generator();
        shape_ids.push(ids.next()); // canvas
        shape_ids.push(ids.next()); // grid

        for _ in pcb.structure.boundary.iter() {
            shape_ids.push(ids.next());
        }
        for comp in pcb.components.iter() {
            let def = &pcb.component_defs[&comp.component_type];

            for _ in def.outlines.iter() {
                shape_ids.push(ids.next());
            }

            if let Some(ref features) = *features {
                for _ in features.obstacles.iter() {
                    shape_ids.push(ids.next());
                }
                for (_, terminals) in features.terminals_by_net.iter() {
                    for _ in terminals.iter() {
                        shape_ids.push(ids.next());
                    }
                }
                for (_, twonets) in features.twonets_by_net.iter() {
                    for _ in twonets.iter() {
                        shape_ids.push(ids.next());
                    }
                }
            }
        }
    }

    let canvas = shape_ids[0];
    let grid = shape_ids[1];
    let first_shape_id = 2;

    {
        use conrod::{color, widget, Colorable, Positionable, Sizeable, Widget};

        let ui = &mut ui.set_widgets();

        widget::Canvas::new()
            .color(color::DARK_CHARCOAL)
            .set(canvas, ui);

        let min_x = 0.0;
        let max_x = std::f64::consts::PI * 2.0;
        let min_y = -1.0;
        let max_y = 1.0;

        let quarter_lines = widget::grid::Lines::step(0.5_f64).thickness(2.0);
        let sixteenth_lines = widget::grid::Lines::step(0.125_f64).thickness(1.0);
        let lines = &[quarter_lines.x(),
                      quarter_lines.y(),
                      sixteenth_lines.x(),
                      sixteenth_lines.y()];

        widget::Grid::new(min_x, max_x, min_y, max_y, lines.iter().cloned())
            .color(color::rgb(0.1, 0.12, 0.15))
            .wh_of(canvas)
            .middle_of(canvas)
            .set(grid, ui);

        // We want to scale it to fit available space.
        // There's probably a better way to avoid all this math, but
        // conrod docs are pretty thin.  Something to investigate later.

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

        // pixel padding in the window, so that there is a gap between
        // the window border and the pcb boundary
        let padding = 20;

        // Figure out how much we want to scale; we want to fix the zoom
        // to show 100% of the circuit at all times, so we sanity check
        // against both the width and the height.
        let dim = ui.window_dim();
        let mut factor = (dim[1] - padding as f64) / p_height;
        if p_width * factor > dim[0] {
            factor = (dim[0] - padding as f64) / p_width;
        }

        // Compensate for scaled width and also for the origin of the
        // PointPath being in the center of its rectangle
        let x_offset = (-p_width / 2.0) - bounds.mins().coords.x;
        let y_offset = (-p_height / 2.0) - bounds.mins().coords.y;

        // Apply the scaling Similarity transform first, followed by
        // the translation transform for the offsets we computed.
        let scale = geom::Similarity::new(geom::Vector::new(0.0, 0.0), 0.0, factor) *
                    geom::Similarity::new(geom::Vector::new(x_offset, y_offset), 0.0, 1.0);

        let mut i = first_shape_id;
        let mut render_shape = |shape: &geom::Shape, color: conrod::Color| {
            let xlate = scale * shape.location;

            if let Some(poly) = shape.handle.as_shape::<geom::Polyline>() {
                let mut points = Vec::new();

                for p in poly.vertices().iter() {
                    let tp = xlate * p;
                    points.push([tp.coords.x, tp.coords.y]);
                }

                let style = match shape.width {
                    Some(width) => {
                        // This doesn't render exactly how I'd like; what
                        // we really want here is to render the minkowski
                        // sum of a circle with the specified width walked
                        // along the described path, because that is how
                        // the pad is going to be drilled out on the pcb
                        widget::primitive::line::Style::new().thickness(width * xlate.scaling())
                    }
                    None => widget::primitive::line::Style::new(),
                };

                widget::PointPath::styled(points, style)
                    .color(color)
                    .middle_of(canvas)
                    .set(shape_ids[i], ui);
                i = i + 1;
            } else if let Some(circle) = shape.handle.as_shape::<geom::Circle>() {
                let p = xlate * geom::Point::new(0.0, 0.0);
                let r = xlate.scaling() * circle.radius();

                widget::Circle::outline(r)
                    .color(color)
                    .xy([p.coords.x, p.coords.y])
                    .set(shape_ids[i], ui);
                i = i + 1;
            }
        };

        // boundary in light blue
        for shape in pcb.structure.boundary.iter() {
            render_shape(&shape.shape, color::LIGHT_BLUE);
        }

        // components
        for comp in pcb.components.iter() {
            let def = &pcb.component_defs[&comp.component_type];

            for outline in def.outlines.iter() {
                let s = outline.shape.translate(&comp.position);
                render_shape(&s, color::DARK_GREEN);
            }
        }

        if let Some(ref features) = *features {

            for obs in features.obstacles.iter() {
                render_shape(&obs.shape, color::RED);
            }

            for (_, terminals) in features.terminals_by_net.iter() {
                for t in terminals.iter() {
                    render_shape(&t.shape, color::YELLOW);
                }
            }

            for (_, twonets) in features.twonets_by_net.iter() {
                for &(ref a, ref b) in twonets.iter() {
                    let points = vec![a.shape.aabb().center(), b.shape.aabb().center()];
                    let poly = geom::Shape::polygon(points, geom::origin(), None);
                    render_shape(&poly, color::LIGHT_PURPLE);
                }
            }
        }
    }
    ui
}

fn gui_loop(pcb: &Pcb,
            rx: mpsc::Receiver<ProgressUpdate>,
            events_loop: &mut conrod::glium::glutin::EventsLoop) {

    let mut features: Option<features::Features> = None;

    let window = glium::glutin::WindowBuilder::new()
        .with_title("PCB Autorouter")
        .with_dimensions(WIDTH, HEIGHT);
    let context = glium::glutin::ContextBuilder::new()
        .with_vsync(true)
        .with_multisampling(4);
    let display = glium::Display::new(window, context, &events_loop).unwrap();

    let mut renderer = conrod::backend::glium::Renderer::new(&display).unwrap();
    let mut ui = build_ui(&pcb, &features, None);
    let image_map = conrod::image::Map::<glium::texture::Texture2d>::new();

    events_loop.run_forever(|event| {
        let mut woke = false;
        match event.clone() {
            glium::glutin::Event::Awakened => {
                woke = true;
                if let Ok(progress) = rx.try_recv() {
                    match progress {
                        ProgressUpdate::Feature(f) => {
                            println!("updated features");
                            features = Some(f);
                        }
                    }
                }
                ()
            }
            glium::glutin::Event::WindowEvent { event, .. } => {
                match event {
                    // Break from the loop upon `Escape` or closed window.
                    glium::glutin::WindowEvent::Closed |
                    glium::glutin::WindowEvent::KeyboardInput {
                        input: glium::glutin::KeyboardInput {
                            virtual_keycode: Some(glium::glutin::VirtualKeyCode::Escape), ..
                        },
                        ..
                    } => return glium::glutin::ControlFlow::Break,

                    _ => (),
                }
            }
            _ => (),
        }

        // Use the `winit` backend feature to convert the winit event to a conrod one.
        match conrod::backend::winit::convert_event(event, &display) {
            None => {
                if !woke {
                    return glium::glutin::ControlFlow::Continue;
                }
            }
            Some(input) => {
                ui.handle_event(input);
            }
        }
        ui = build_ui(&pcb, &features, Some(&ui));

        if let Some(primitives) = ui.draw_if_changed() {
            renderer.fill(&display, primitives, &image_map);
            let mut target = display.draw();
            target.clear_color(0.0, 0.0, 0.0, 1.0);
            renderer.draw(&display, &mut target, &image_map).unwrap();
            target.finish().unwrap();
        }

        glium::glutin::ControlFlow::Continue
    });
}

/// Helper for waking up the GUI thread as we generate
/// new information.
struct Notify {
    tx: mpsc::Sender<ProgressUpdate>,
    proxy: conrod::glium::glutin::EventsLoopProxy,
}

impl Notify {
    fn send(&self, update: ProgressUpdate) {
        self.tx.send(update).unwrap();
        self.proxy.wakeup().unwrap();
    }
}

/// This is where we initiate the heavy lifting.
/// We do this separately from the UI thread so that we can incrementally
/// update the UI as we compute the data.
fn compute_thread(pcb: &Pcb, notifier: Notify) {
    let features = features::Features::from_pcb(&pcb);
    notifier.send(ProgressUpdate::Feature(features.clone()));
    println!("building layer assignment graphs");

    let mut cfg = layerassign::Configuration::default();
    for (_, twonets) in features.twonets_by_net.iter() {
        for &(ref a, ref b) in twonets {
            cfg.add_twonet(a, b, &features.all_layers);
        }
    }

    println!("graphs built");
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
    let mut events_loop = glium::glutin::EventsLoop::new();
    let proxy = events_loop.create_proxy();

    {
        let pcb_copy = pcb.clone();
        thread::spawn(move || {
                          compute_thread(&pcb_copy,
                                         Notify {
                                             tx: tx,
                                             proxy: proxy,
                                         });
                      });
    }

    gui_loop(&pcb, rx, &mut events_loop);

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
