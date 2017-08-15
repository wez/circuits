#[macro_use]
extern crate error_chain;
use std::process::exit;

#[macro_use]
extern crate nom;

extern crate itertools;

#[macro_use]
extern crate conrod;

mod dsn;
mod geom;


use std::error::Error;
use self::dsn::Pcb;

fn go() -> Result<(), Box<Error>> {
    println!("Hello, world!");
    let pcb = Pcb::parse("left-pcb-no-fill.dsn")?;
    println!("made pcb: {:#?}", pcb);

    use conrod;
    use conrod::backend::glium::glium::{self, Surface};
    const WIDTH: u32 = 700;
    const HEIGHT: u32 = 700;

    let mut events_loop = glium::glutin::EventsLoop::new();
    let window = glium::glutin::WindowBuilder::new()
        .with_title("PCB Autorouter")
        .with_dimensions(WIDTH, HEIGHT);
    let context = glium::glutin::ContextBuilder::new()
        .with_vsync(true)
        .with_multisampling(4);
    let display = glium::Display::new(window, context, &events_loop).unwrap();

    let mut ui = conrod::UiBuilder::new([WIDTH as f64, HEIGHT as f64]).build();

    // Generate the widget identifiers.
    widget_ids!(struct Ids {canvas, plot, grid, poly });
    let ids = Ids::new(ui.widget_id_generator());
    let mut renderer = conrod::backend::glium::Renderer::new(&display).unwrap();

    // The image map describing each of our widget->image mappings (in our case, none).
    let image_map = conrod::image::Map::<glium::texture::Texture2d>::new();

    events_loop.run_forever(|event| {

        match event.clone() {
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
        let input = match conrod::backend::winit::convert_event(event, &display) {
            None => return glium::glutin::ControlFlow::Continue,
            Some(input) => input,
        };

        ui.handle_event(input);

        {
            use conrod::{color, widget, Colorable, Positionable, Sizeable, Widget};

            let ui = &mut ui.set_widgets();

            widget::Canvas::new()
                .color(color::DARK_CHARCOAL)
                .set(ids.canvas, ui);

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
                .wh_of(ids.canvas)
                .middle_of(ids.canvas)
                .set(ids.grid, ui);

            for shape in pcb.structure.boundary.iter() {
                if let Some(poly) = shape.shape.handle.as_shape::<geom::Polyline>() {
                    // We want to scale it to fit available space.
                    // There's probably a better way to avoid all this math, but
                    // conrod docs are pretty thin.  Something to investigate later.
                    let bounds = shape.shape.aabb();
                    let p_width = bounds.maxs().coords.x - bounds.mins().coords.x;
                    let p_height = bounds.maxs().coords.y - bounds.mins().coords.y;

                    let mut points = Vec::new();

                    // Compensate for scaled width and also for the origin of the
                    // PointPath being in the center of its rectangle
                    let x_offset = (-p_width / 2.0) - bounds.mins().coords.x;
                    let y_offset = (-p_height / 2.0) - bounds.mins().coords.y;

                    // pixel padding in the window, so that there is a gap between
                    // the window border and the pcb boundary
                    let padding = 20;

                    let factor = (HEIGHT - padding) as f64;

                    let scale = |p: &geom::Point| {
                        let x: f64 = (p.coords.x + x_offset) * factor / p_width;
                        let y: f64 = (p.coords.y + y_offset) * factor / p_height;
                        [x, y]
                    };

                    for p in poly.vertices().iter() {
                        points.push(scale(p));
                    }

                    widget::PointPath::new(points)
                        .color(color::LIGHT_BLUE)
                        .wh_of(ids.canvas)
                        .middle_of(ids.canvas)
                        .set(ids.poly, ui);
                }
            }
        }

        if let Some(primitives) = ui.draw_if_changed() {
            renderer.fill(&display, primitives, &image_map);
            let mut target = display.draw();
            target.clear_color(0.0, 0.0, 0.0, 1.0);
            renderer.draw(&display, &mut target, &image_map).unwrap();
            target.finish().unwrap();
        }

        glium::glutin::ControlFlow::Continue
    });

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
