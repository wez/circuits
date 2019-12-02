use piston_window::types::{Color, Matrix2d};
use piston_window::{line, Event, Graphics, PistonWindow};
use std::sync::mpsc;
use std::time::Duration;

use crate::dsn::Pcb;
use crate::features::Features;
use crate::geom::{origin, Polyline, Shape, Similarity, Vector};
use crate::ProgressUpdate;
use ncollide2d::bounding_volume::BoundingVolume;

#[allow(dead_code)]
const DARK_CHARCOAL: Color = [46.0 / 255.0, 52.0 / 255.0, 54.0 / 255.0, 1.0];
#[allow(dead_code)]
const LIGHT_BLUE: Color = [114.0 / 255.0, 159.0 / 255.0, 207.0 / 255.0, 1.0];
#[allow(dead_code)]
const DARK_GREEN: Color = [78.0 / 255.0, 154.0 / 255.0, 6.0 / 255.0, 1.0];
#[allow(dead_code)]
const RED: Color = [204.0 / 255.0, 0.0, 0.0, 1.0];
#[allow(dead_code)]
const YELLOW: Color = [237.0 / 255.0, 212.0 / 255.0, 0.0, 1.0];
#[allow(dead_code)]
const LIGHT_PURPLE: Color = [173.0 / 255.0, 127.0 / 255.0, 168.0 / 255.0, 1.0];
#[allow(dead_code)]
const PURPLE: Color = [117.0 / 255.0, 80.0 / 255.0, 123.0 / 255.0, 1.0];
#[allow(dead_code)]
const DARK_PURPLE: Color = [92.0 / 255.0, 53.0 / 255.0, 102.0 / 255.0, 1.0];
#[allow(dead_code)]
const LIGHT_RED: Color = [239.0 / 255.0, 41.0 / 255.0, 41.0 / 255.0, 1.0];
#[allow(dead_code)]
const ORANGE: Color = [245.0 / 255.0, 121.0 / 255.0, 0.0, 1.0];
#[allow(dead_code)]
const BROWN: Color = [193.0 / 255.0, 125.0 / 255.0, 17.0 / 255.0, 1.0];
#[allow(dead_code)]
const BLUE: Color = [52.0 / 255.0, 101.0 / 255.0, 164.0 / 255.0, 1.0];
#[allow(dead_code)]
const LAYER_COLORS: [Color; 2] = [ORANGE, LIGHT_BLUE];

#[derive(Default)]
struct RenderState {
    height: u32,
    width: u32,
}

impl RenderState {
    fn reset(&mut self) {
        self.height = 0;
        self.width = 0;
    }

    fn reset_if_changed(&mut self, height: u32, width: u32) -> bool {
        if self.height != height || self.width != width {
            self.height = height;
            self.width = width;
            true
        } else {
            false
        }
    }
}

fn draw_polygon<G>(color: Color, shape: &Shape, radius: f64, transform: Matrix2d, g: &mut G)
where
    G: Graphics,
{
    let poly = shape
        .handle
        .as_shape::<Polyline>()
        .expect("transform always yields a polygon!?");

    let points = poly.points();

    for i in 0..points.len() - 1 {
        let a = &points[i];
        let b = &points[i + 1];
        line(
            color,
            radius,
            [a.coords.x, a.coords.y, b.coords.x, b.coords.y],
            transform,
            g,
        );
    }
}

fn draw_gui(
    window: &mut PistonWindow,
    e: &Event,
    pcb: &Pcb,
    features: &Option<Features>,
    state: &mut RenderState,
) {
    use piston_window::*;

    let size = window.size();

    // We do manual swapping so that we can avoid rendering if nothing
    // actually changed.
    let mut need_swap = false;

    window.draw_2d(e, |c, g| {
        // pixel padding in the window, so that there is a gap between
        // the window border and the pcb boundary
        let padding = 10f64;
        let t = c
            .transform
            .flip_v()
            .trans(padding / 2.0, -(size.height as f64) + padding / 2.0);

        if state.reset_if_changed(size.height as u32, size.width as u32) {
            clear(DARK_CHARCOAL, g);

            // First, compute the overall bounds of the shapes we want to render.
            let bounds = {
                let mut bounds = None;
                for shape in pcb.structure.boundary.iter() {
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
            let scale = Similarity::new(Vector::new(0.0, 0.0), 0.0, factor)
                * Similarity::new(Vector::new(x_offset, y_offset), 0.0, 1.0);

            // boundary in light blue
            for shape in pcb.structure.boundary.iter() {
                draw_polygon(LIGHT_BLUE, &shape.shape.transform(&scale), 0.5, t, g);
            }

            // components
            for comp in pcb.components.iter() {
                let def = &pcb.component_defs[&comp.component_type];

                for outline in def.outlines.iter() {
                    let s = outline.shape.translate(&comp.position).transform(&scale);
                    draw_polygon(DARK_GREEN, &s, 0.5, t, g);
                }
            }

            if let Some(ref features) = *features {
                // Render paths from the CDTGraph
                for &(ref a, ref b) in features.cdt_edges.iter() {
                    let line = Shape::line(a, b).transform(&scale);
                    draw_polygon([0.0, 0.0, 0.0, 0.3], &line, 0.6, t, g);
                }

                for obs in features.obstacles.iter() {
                    draw_polygon(RED, &obs.shape.transform(&scale), 0.5, t, g);
                }

                for (_, terminals) in features.terminals_by_net.iter() {
                    for terminal in terminals.iter() {
                        draw_polygon(YELLOW, &terminal.shape.transform(&scale), 0.5, t, g);
                    }
                }

                for (_, twonets) in features.twonets_by_net.iter() {
                    for &(ref a, ref b) in twonets.iter() {
                        let points = vec![a.shape.aabb().center(), b.shape.aabb().center()];
                        let poly = Shape::polygon(points, origin(), None).transform(&scale);
                        draw_polygon(LIGHT_PURPLE, &poly, 0.5, t, g);
                    }
                }

                for (layer_id, paths) in features.paths_by_layer.iter() {
                    for &(ref a, ref b) in paths.iter() {
                        draw_polygon(
                            LAYER_COLORS[*layer_id as usize],
                            &Shape::line(&a.point(), &b.point()).transform(&scale),
                            0.7,
                            t,
                            g,
                        );
                    }
                }
            }

            need_swap = true;
        }
    });

    if need_swap {
        window.swap_buffers();
    }
}

pub fn run_gui(pcb: &Pcb, rx: mpsc::Receiver<ProgressUpdate>) {
    use piston_window::*;
    let mut features: Option<Features> = None;
    let mut state = RenderState::default();

    let mut window: PistonWindow = WindowSettings::new("PCB Autorouter", [600, 600])
        .exit_on_esc(true)
        .build()
        .unwrap();

    // We don't need to be so aggressive with our frame rate, so
    // dial down the update and frame rates.
    window.set_ups(5);
    window.set_max_fps(5);
    window.set_lazy(false);
    // We'll manually swap the buffers only when we know that the content
    // has changed.
    window.set_swap_buffers(false);

    while let Some(e) = window.next() {
        if let Event::Loop(l) = e {
            match l {
                Loop::Render(_) => {
                    draw_gui(&mut window, &e, &pcb, &features, &mut state);
                }
                Loop::Idle(args) => {
                    let mut need_update = false;
                    let seconds = args.dt.trunc() as u64;
                    let nanos = args.dt.fract() as u32 * 1_000_000_000;
                    while let Ok(progress) = rx.recv_timeout(Duration::new(seconds, nanos)) {
                        need_update = true;
                        match progress {
                            ProgressUpdate::Feature(f) => {
                                features = Some(f);
                            }
                            ProgressUpdate::Done() => {}
                        }
                    }
                    if need_update {
                        state.reset();
                    }
                }
                _ => {}
            }
        }
    }
}
