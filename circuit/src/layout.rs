//! Functions that operate on a Layout (a kicad pcb file)
use crate::footprint::ToGeom;
use crate::geometry;
use geo::prelude::*;
use geo::{GeometryCollection, MultiPolygon, Polygon, Rect};
use kicad_parse_gen::layout::{Element, Layout};
use kicad_parse_gen::Adjust;

/// Configure the setup for the pcb to work within the constraints
/// of Seeed Fusion.
/// http://support.seeedstudio.com/knowledgebase/articles/447362-fusion-pcb-specification
pub fn apply_seeed_drc(layout: &mut Layout) {
    use kicad_parse_gen::layout::SetupElement;

    macro_rules! ele {
        ($item:ident, $value:expr) => {
            layout.setup.elements.push(SetupElement {
                name: stringify!($item).to_string(),
                value1: stringify!($value).to_string(),
                value2: None,
            });
        };

        ($item:ident, $value:expr, $value2:expr) => {
            layout.setup.elements.push(SetupElement {
                name: stringify!($item).to_string(),
                value1: stringify!($value).to_string(),
                value2: Some(stringify!($value2).to_string()),
            });
        };
    }

    ele!(last_trace_width, 0.153);
    ele!(trace_clearance, 0.153);
    ele!(zone_clearance, 0.153);
    ele!(zone_45_only, no);
    ele!(trace_min, 0.153);
    ele!(segment_width, 0.153);
    ele!(edge_width, 0.15);
    ele!(via_size, 0.6);
    ele!(via_drill, 0.3);
    ele!(via_min_size, 0.6);
    ele!(via_min_drill, 0.3);
    ele!(uvia_size, 0.3);
    ele!(uvia_drill, 0.1);
    ele!(uvias_allowed, no);
    ele!(uvia_min_size, 0.2);
    ele!(uvia_min_drill, 0.1);
    ele!(pcb_text_width, 0.3);
    ele!(pcb_text_size, 1.5, 1.5);
    ele!(mod_edge_width, 0.15);
    ele!(mod_text_size, 1, 1);
    ele!(mod_text_width, 0.15);
    ele!(pad_size, 1.524, 1.524);
    ele!(pad_drill, 0.762);
    ele!(pad_to_mask_clearance, 0.2);
    ele!(aux_axis_origin, 0, 0);

    macro_rules! plot {
        ($item:ident, $value:expr) => {
            layout.setup.pcbplotparams.push(SetupElement {
                name: stringify!($item).to_string(),
                value1: stringify!($value).to_string(),
                value2: None,
            });
        };

        ($item:ident, $value:expr, $value2:expr) => {
            layout.setup.pcbplotparams.push(SetupElement {
                name: stringify!($item).to_string(),
                value1: stringify!($value).to_string(),
                value2: Some(stringify!($value2).to_string()),
            });
        };
    }

    plot!(layerselection, 0x010f0_ffffffff);
    plot!(usegerberextensions, true);
    plot!(usegerberattributes, false);
    plot!(usegerberadvancedattributes, false);
    plot!(creategerberjobfile, false);
    plot!(excludeedgelayer, true);
    plot!(linewidth, 0.100000);
    plot!(plotframeref, false);
    plot!(viasonmask, false);
    plot!(mode, 1);
    plot!(useauxorigin, false);
    plot!(hpglpennumber, 1);
    plot!(hpglpenspeed, 20);
    plot!(hpglpendiameter, 15.000000);
    plot!(psnegative, false);
    plot!(psa4output, false);
    plot!(plotreference, true);
    plot!(plotvalue, true);
    plot!(plotinvisibletext, false);
    plot!(padsonsilk, false);
    plot!(subtractmaskfromsilk, false);
    plot!(outputformat, 1);
    plot!(mirror, false);
    plot!(drillshape, 0);
    plot!(scaleselection, 1);

    // Set up the netclass
    let nets = layout
        .elements
        .iter()
        .filter_map(|element| match element {
            Element::Net(net) if net.name.0 != "" => Some(net.name.clone()),
            _ => None,
        })
        .collect();

    use kicad_parse_gen::layout::NetClass;

    layout.elements.push(Element::NetClass(NetClass {
        name: "Default".into(),
        desc: "This is the default net class.".into(),
        clearance: 0.153,
        trace_width: 0.25,
        via_dia: 0.6,
        via_drill: 0.3,
        uvia_dia: 0.3,
        uvia_drill: 0.1,
        diff_pair_gap: None,
        diff_pair_width: None,
        nets,
    }));
}

pub fn layout_to_geom_collection(layout: &Layout) -> GeometryCollection<f64> {
    layout
        .elements
        .iter()
        .filter_map(|ele| match ele {
            Element::Module(module) => Some(module.to_geom()),
            _ => None,
        })
        .collect()
}

pub fn compute_bounding_box(layout: &Layout) -> Rect<f64> {
    let collection = layout_to_geom_collection(layout);
    let multi: MultiPolygon<f64> = geometry::iter_polygons(collection).into_iter().collect();
    multi
        .bounding_rect()
        .expect("unable to compute bounding rect for layout")
}

/// Try to place the content tidily in the sheet.
pub fn adjust_to_fit_page(layout: &mut Layout) {
    // US Letter dimensions
    const LETTER_WIDTH: f64 = 280.0;
    const LETTER_HEIGHT: f64 = 218.0;

    let bounds = compute_bounding_box(layout);

    let width = bounds.max.x - bounds.min.x;
    let height = bounds.max.y - bounds.min.y;

    // Center it
    let x_off = (LETTER_WIDTH - width) / 2.0;
    let y_off = (LETTER_HEIGHT - height) / 2.0;

    layout.adjust(x_off - bounds.min.x, y_off - bounds.min.y);
}

pub fn compute_hull(layout: &Layout) -> Polygon<f64> {
    let collection = layout_to_geom_collection(layout);
    geometry::convex_hull_collection(collection)
}
