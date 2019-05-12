// https://github.com/steveklabnik/rustdoc/issues/96
#![allow(unused_doc_comments)]

use crate::geom::{origin, Location, Point, Shape, Vector};
use failure::Fallible;
use failure_derive::*;
use itertools::Itertools;
use std::collections::{HashMap, HashSet};
use std::default::Default;
use std::fs::File;
use std::io::prelude::*;
use std::str;

#[derive(Clone, Debug)]
pub enum Value {
    Literal(String),
    Quoted(String),
    Integer(i64),
    Float(f64),
    TaggedList(String, Vec<Value>),
}

#[derive(Debug, Fail)]
pub enum ErrorKind {
    #[fail(display = "No string representation for {:?}", 0)]
    NoStringRep(Value),
    #[fail(display = "NoIntRep")]
    NoIntRep,
    #[fail(display = "NoFloatRep")]
    NoFloatRep,
    #[fail(display = "unhandled shape type {}", 0)]
    UnhandledShapeType(String),
    #[fail(display = "expected ({} ...) but got {:?}", a, value)]
    UnexpectedTag { a: &'static str, value: Value },
    #[fail(display = "expected (tag ...) but got {:?}", 0)]
    ExpectedTaggedList(Value),
    #[fail(display = "expected (tag value) but got {:?}", 0)]
    ExpectedTaggedValue(Value),
    #[fail(display = "unexpected value {:?}", 0)]
    UnexpectedValue(Value),
    #[fail(display = "parse error {}", 0)]
    Nom(String),
}

impl Value {
    pub fn as_string(&self) -> Fallible<&String> {
        match self {
            &Value::Quoted(ref s) => Ok(s),
            &Value::Literal(ref s) => Ok(s),
            _ => Err(ErrorKind::NoStringRep(self.clone()).into()),
        }
    }

    pub fn as_i64(&self) -> Fallible<i64> {
        match self {
            &Value::Integer(ref s) => Ok(*s),
            _ => Err(ErrorKind::NoIntRep.into()),
        }
    }

    pub fn as_f64(&self) -> Fallible<f64> {
        match self {
            &Value::Float(ref s) => Ok(*s),
            &Value::Integer(ref s) => Ok(*s as f64),
            _ => Err(ErrorKind::NoFloatRep.into()),
        }
    }

    pub fn as_tagged_value(&self) -> Fallible<(&String, Value)> {
        match self {
            &Value::TaggedList(ref tag, ref list) => {
                if list.len() != 1 {
                    Err(ErrorKind::ExpectedTaggedValue(self.clone()).into())
                } else {
                    Ok((tag, list[0].clone()))
                }
            }
            _ => Err(ErrorKind::ExpectedTaggedValue(self.clone()).into()),
        }
    }

    pub fn as_tagged_list(&self) -> Fallible<(&String, Vec<Value>)> {
        match self {
            &Value::TaggedList(ref tag, ref list) => Ok((tag, list.clone())),
            _ => Err(ErrorKind::ExpectedTaggedList(self.clone()).into()),
        }
    }

    pub fn as_tagged_list_with_name(&self, name: &'static str) -> Fallible<Vec<Value>> {
        match self {
            &Value::TaggedList(ref tag, ref list) => {
                if tag == name {
                    Ok(list.clone())
                } else {
                    Err(ErrorKind::UnexpectedTag {
                        a: name,
                        value: self.clone(),
                    }
                    .into())
                }
            }
            _ => Err(ErrorKind::UnexpectedTag {
                a: name,
                value: self.clone(),
            }
            .into()),
        }
    }
}

named!(string_quote_parser<&[u8], Value>, do_parse!(
    tag!("(string_quote \")") >>
    (Value::TaggedList(
            "string_quote".to_owned(),
            vec![Value::Literal("\"".to_owned())]
       ))
));

named!(quoted_string_parser<&[u8], Value>, do_parse!(
    char!('"') >>
    s: map_res!(take_until!("\""), str::from_utf8) >>
    char!('"') >>
    (Value::Quoted(s.to_owned()))
));

fn literal_or_numeric(s: &str) -> Value {
    match s.parse::<i64>() {
        Ok(i) => {
            return Value::Integer(i);
        }
        _ => {}
    }
    match s.parse::<f64>() {
        Ok(i) => {
            return Value::Float(i);
        }
        _ => {}
    }
    Value::Literal(s.to_owned())
}

named!(literal_parser<&[u8], Value>,
    do_parse!(
    opt!(nom::multispace) >>
    s: map_res!(take_until_either!(" ()\n\t"), str::from_utf8) >>
    opt!(nom::multispace) >>
    (literal_or_numeric(s))
));

named!(tagged_list_parser<&[u8], Value>, do_parse!(
    opt!(nom::multispace) >>
    char!('(') >>
    t: map_res!(take_until_either!(" ()\n\t"), str::from_utf8) >>
    opt!(nom::multispace) >>
    args: many0!(alt!(
                string_quote_parser |
                quoted_string_parser |
                literal_parser |
                tagged_list_parser
                )) >>
    opt!(nom::multispace) >>
    char!(')') >>
    (Value::TaggedList(t.to_owned(), args))
));

fn parse_value(bytes: &[u8]) -> Fallible<Value> {
    let res = tagged_list_parser(bytes);
    match res.clone() {
        nom::IResult::Done(_, output) => Ok(output),
        nom::IResult::Incomplete(needed) => {
            Err(ErrorKind::Nom(format!("need {:?} more bytes", needed)).into())
        }
        nom::IResult::Error(_) => {
            use nom::{prepare_errors, print_offsets};
            if let Some(v) = prepare_errors(bytes, res) {
                Err(ErrorKind::Nom(format!("\n{}", print_offsets(bytes, 0, &v))).into())
            } else {
                Err(ErrorKind::Nom(format!("sadness")).into())
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct DsnShape {
    pub layer: String,
    pub shape: Shape,
}

#[derive(Default, Debug, Clone)]
pub struct Parser {
    pub string_quote: String,
    pub space_in_quoted_tokens: String,
    pub host_cad: String,
    pub host_version: String,
}

#[derive(Default, Debug, Clone)]
pub struct Layer {
    pub name: String,
    pub layer_type: String,
    pub index: i64,
}

#[derive(Default, Debug, Clone)]
pub struct Rule {
    pub width: f64,
    pub clearance: f64,
}

#[derive(Default, Debug, Clone)]
pub struct Structure {
    pub layers: Vec<Layer>,
    pub boundary: Vec<DsnShape>,
    pub keepout: Vec<DsnShape>,
    pub via: String,
    pub rule: Rule,
}

#[derive(Debug, Clone)]
pub struct Component {
    pub component_type: String,
    pub instance_name: String,
    pub position: Location,
}

#[derive(Default, Debug, Clone)]
pub struct PadStack {
    pub pad_type: String,
    pub pads: HashMap<String, DsnShape>,
    pub attach: bool,
}

#[derive(Debug, Clone)]
pub struct Pin {
    pub pad_type: String,
    pub pad_num: i64,
    pub position: Location,
}

#[derive(Default, Debug, Clone)]
pub struct ComponentDef {
    pub component_type: String,
    pub outlines: Vec<DsnShape>,
    pub pins: Vec<Pin>,
    pub keepout: Vec<DsnShape>,
}

#[derive(Default, Debug, Clone)]
pub struct Net {
    pub pins: Vec<String>,
}

#[derive(Default, Debug, Clone)]
pub struct NetClass {
    pub class_name: String,
    pub nets: HashSet<String>,
}

#[derive(Default, Debug, Clone)]
pub struct Pcb {
    pub file_name: String,
    pub parser: Parser,
    pub structure: Structure,
    pub unit: String,
    pub components: Vec<Component>,
    pub component_defs: HashMap<String, ComponentDef>,
    pub pad_defs: HashMap<String, PadStack>,
    pub networks: HashMap<String, Net>,
    pub net_classes: HashMap<String, NetClass>,
}

impl DsnShape {
    //! parses a tagged list into a shape instance
    fn parse(tag: &String, list: &Vec<Value>, drill: bool) -> Fallible<DsnShape> {
        match tag.as_ref() {
            "path" | "polygon" => DsnShape::parse_path(list, drill),
            "circle" => DsnShape::parse_circle(list),
            "rect" => DsnShape::parse_rect(list),
            _ => Err(ErrorKind::UnhandledShapeType(tag.to_string()).into()),
        }
    }

    fn parse_circle(list: &Vec<Value>) -> Fallible<DsnShape> {
        let layer = list[0].as_string()?;

        let diameter = list[1].as_f64()?;
        let (x, y) = {
            if list.len() > 2 {
                (list[2].as_f64()?, list[3].as_f64()?)
            } else {
                (0.0, 0.0)
            }
        };

        Ok(DsnShape {
            layer: layer.clone(),
            shape: Shape::circle(diameter / 2.0, Location::new(Vector::new(x, y), 0.0)),
        })
    }

    fn parse_path(list: &Vec<Value>, drill: bool) -> Fallible<DsnShape> {
        let layer = list[0].as_string()?;

        let mut points = Vec::new();
        let aperture = list[1].as_f64()? / 2.0;
        let width = if aperture == 0.0 {
            None
        } else {
            Some(aperture)
        };

        // skip 2 because 0 is the layer that we parsed and index 1 is
        // the aperture width which we extract above.
        for (x, y) in list.iter().skip(2).tuples() {
            points.push(Point::new(x.as_f64()?, y.as_f64()?));
        }

        if drill && points.len() == 2 {
            if let Some(width) = width {
                return Ok(DsnShape {
                    layer: layer.clone(),
                    shape: Shape::capsule(width, &points[0], &points[1], origin()),
                });
            }
        }

        Ok(DsnShape {
            layer: layer.clone(),
            shape: Shape::polygon(points, origin(), width),
        })
    }

    fn parse_rect(list: &Vec<Value>) -> Fallible<DsnShape> {
        let layer = list[0].as_string()?;

        let (bottom_left_x, bottom_left_y) = (list[1].as_f64()?, list[2].as_f64()?);
        let (top_right_x, top_right_y) = (list[3].as_f64()?, list[4].as_f64()?);

        let points = vec![
            Point::new(bottom_left_x, bottom_left_y),
            Point::new(bottom_left_x, top_right_y),
            Point::new(top_right_x, top_right_y),
            Point::new(top_right_x, bottom_left_y),
            Point::new(bottom_left_x, bottom_left_y),
        ];

        Ok(DsnShape {
            layer: layer.clone(),
            shape: Shape::polygon(points, origin(), None),
        })
    }
}

impl Parser {
    //! parses a value list into an existing Parser object
    fn new(list: &Vec<Value>) -> Fallible<Parser> {
        let mut p = Parser::default();

        for ele in list.iter() {
            let (k, v) = ele.as_tagged_value()?;
            match k.as_ref() {
                "string_quote" => {
                    p.string_quote = v.as_string()?.clone();
                }
                "space_in_quoted_tokens" => {
                    p.space_in_quoted_tokens = v.as_string()?.clone();
                }
                "host_cad" => {
                    p.host_cad = v.as_string()?.clone();
                }
                "host_version" => {
                    p.host_version = v.as_string()?.clone();
                }
                _ => {
                    return Err(ErrorKind::UnexpectedValue(ele.clone()).into());
                }
            }
        }

        Ok(p)
    }
}

impl Layer {
    //! parses a value list into a new Layer object
    fn new(list: &Vec<Value>) -> Fallible<Layer> {
        let mut l = Layer::default();

        l.name = list[0].as_string()?.clone();
        for ele in list.iter().skip(1) {
            let (k, v) = ele.as_tagged_value()?;
            match k.as_ref() {
                "type" => {
                    l.layer_type = v.as_string()?.clone();
                }
                "property" => {
                    let (name, val) = v.as_tagged_value()?;
                    match name.as_ref() {
                        "index" => {
                            l.index = val.as_i64()?;
                        }
                        _ => {
                            return Err(ErrorKind::UnexpectedValue(ele.clone()).into());
                        }
                    }
                }
                _ => {
                    return Err(ErrorKind::UnexpectedValue(ele.clone()).into());
                }
            }
        }

        Ok(l)
    }
}

impl Rule {
    fn new(list: &Vec<Value>) -> Fallible<Rule> {
        let mut rule = Rule::default();

        for ele in list.iter() {
            let (tag, list) = ele.as_tagged_list()?;
            match tag.as_ref() {
                "width" => {
                    rule.width = list[0].as_f64()?;
                }
                "clearance" => {
                    // we ignore the variants that specify clearance for
                    // different types.
                    if list.len() == 1 {
                        rule.clearance = list[0].as_f64()?;
                    }
                }
                _ => {
                    return Err(ErrorKind::UnexpectedValue(ele.clone()).into());
                }
            }
        }

        Ok(rule)
    }
}

impl Structure {
    //! parses a value list and populates an existing Structure instance
    fn new(list: &Vec<Value>) -> Fallible<Structure> {
        let mut s = Structure::default();
        for ele in list.iter() {
            let (k, list) = ele.as_tagged_list()?;
            match k.as_ref() {
                "layer" => {
                    s.layers.push(Layer::new(&list)?);
                }
                "keepout" => {
                    let (tag, list) = list[1].as_tagged_list()?;
                    s.keepout.push(DsnShape::parse(tag, &list, false)?);
                }
                "boundary" => {
                    let (tag, list) = list[0].as_tagged_list()?;
                    s.boundary.push(DsnShape::parse(tag, &list, false)?);
                }
                "via" => {
                    s.via = list[0].as_string()?.clone();
                }
                "rule" => {
                    s.rule = Rule::new(&list)?;
                }
                _ => {
                    return Err(ErrorKind::UnexpectedValue(ele.clone()).into());
                }
            }
        }
        Ok(s)
    }
}

impl Pcb {
    //! parses the contents of filename into a Pcb object
    // the file is assumed to be a spectra dsn file, and was implemented
    // based on a dsn file produced by a recent kicad version.
    pub fn parse(filename: &str) -> Fallible<Pcb> {
        let mut f = File::open(filename)?;
        let mut buffer = String::new();

        f.read_to_string(&mut buffer)?;

        let v = parse_value(buffer.as_bytes())?;
        let mut pcb = Pcb::default();

        let list = v.as_tagged_list_with_name("pcb")?;
        for ele in list.iter().skip(1) {
            let (tagname, list) = ele.as_tagged_list()?;
            match tagname.as_ref() {
                "parser" => {
                    pcb.parser = Parser::new(&list)?;
                }
                "structure" => {
                    pcb.structure = Structure::new(&list)?;
                }
                "placement" => {
                    pcb.component_list(&list)?;
                }
                "library" => {
                    pcb.component_def_list(&list)?;
                }
                "network" => {
                    pcb.network_list(&list)?;
                }
                "unit" => {
                    pcb.unit = list[0].as_string()?.clone();
                }
                "resolution" | "wiring" => {
                    // ignored
                }
                _ => {
                    return Err(ErrorKind::UnexpectedValue(ele.clone()).into());
                }
            }
        }
        Ok(pcb)
    }

    fn network_list(&mut self, list: &Vec<Value>) -> Fallible<()> {
        for ele in list.iter() {
            let (tagname, list) = ele.as_tagged_list()?;
            match tagname.as_ref() {
                "net" => {
                    let netname = list[0].as_string()?;
                    let pinlist = list[1].as_tagged_list_with_name("pins")?;
                    let mut net = Net::default();
                    for pin in pinlist.iter() {
                        net.pins.push(pin.as_string()?.clone());
                    }
                    self.networks.insert(netname.clone(), net);
                }
                "class" => {
                    let mut class = NetClass::default();
                    class.class_name = list[0].as_string()?.clone();
                    for ele in list.iter().skip(1) {
                        match ele {
                            &Value::Literal(ref s) => {
                                class.nets.insert(s.clone());
                            }
                            _ => {}
                        }
                    }
                    self.net_classes.insert(class.class_name.clone(), class);
                }
                _ => return Err(ErrorKind::UnexpectedValue(ele.clone()).into()),
            }
        }
        Ok(())
    }

    fn component_list(&mut self, list: &Vec<Value>) -> Fallible<()> {
        for ele in list.iter() {
            let list = ele.as_tagged_list_with_name("component")?;
            let component_type = list[0].as_string()?;
            for place in list.iter().skip(1) {
                let list = place.as_tagged_list_with_name("place")?;
                let name = list[0].as_string()?;
                let x = list[1].as_f64()?;
                let y = list[2].as_f64()?;
                // let side = list[3].as_string()?;
                let rotation = list[4].as_f64()?;
                self.components.push(Component {
                    component_type: component_type.clone(),
                    instance_name: name.clone(),
                    position: Location::new(Vector::new(x, y), rotation.to_radians()),
                })
            }
        }
        Ok(())
    }

    fn padstack(&mut self, list: &Vec<Value>) -> Fallible<()> {
        let mut def = PadStack::default();
        def.pad_type = list[0].as_string()?.clone();

        for ele in list.iter().skip(1) {
            let (tag, list) = ele.as_tagged_list()?;
            match tag.as_ref() {
                "shape" => {
                    let (t, l) = list[0].as_tagged_list()?;
                    let shape = DsnShape::parse(t, &l, true)?;
                    def.pads.insert(shape.layer.clone(), shape);
                }
                "attach" => {
                    let v = list[0].as_string()?;
                    if v == "on" {
                        def.attach = true;
                    }
                }
                _ => {
                    return Err(ErrorKind::UnexpectedValue(ele.clone()).into());
                }
            }
        }
        self.pad_defs.insert(def.pad_type.clone(), def);
        Ok(())
    }

    fn image(&mut self, list: &Vec<Value>) -> Fallible<()> {
        let mut def = ComponentDef::default();
        def.component_type = list[0].as_string()?.clone();

        for ele in list.iter().skip(1) {
            let (tagname, list) = ele.as_tagged_list()?;
            match tagname.as_ref() {
                "outline" => {
                    let (t, l) = list[0].as_tagged_list()?;
                    def.outlines.push(DsnShape::parse(t, &l, false)?);
                }
                "pin" => {
                    def.pins.push(self.parse_pin(&list)?);
                }
                "keepout" => {
                    let (t, l) = list[1].as_tagged_list()?;
                    def.keepout.push(DsnShape::parse(t, &l, false)?);
                }
                _ => {
                    return Err(ErrorKind::UnexpectedValue(ele.clone()).into());
                }
            }
        }

        self.component_defs.insert(def.component_type.clone(), def);
        Ok(())
    }

    fn parse_pin(&self, list: &Vec<Value>) -> Fallible<Pin> {
        if list.len() == 4 {
            Ok(Pin {
                pad_type: list[0].as_string()?.clone(),
                pad_num: list[1].as_i64()?,
                position: Location::new(Vector::new(list[2].as_f64()?, list[3].as_f64()?), 0.0),
            })
        } else {
            let rot = list[1].as_tagged_list_with_name("rotate")?;
            let degrees = rot[0].as_f64()?;
            Ok(Pin {
                pad_type: list[0].as_string()?.clone(),
                pad_num: list[2].as_i64()?,
                position: Location::new(
                    Vector::new(list[3].as_f64()?, list[4].as_f64()?),
                    degrees.to_radians(),
                ),
            })
        }
    }

    fn component_def_list(&mut self, list: &Vec<Value>) -> Fallible<()> {
        for image in list.iter() {
            let (tagname, list) = image.as_tagged_list()?;
            match tagname.as_ref() {
                "image" => {
                    self.image(&list)?;
                }
                "padstack" => {
                    self.padstack(&list)?;
                }
                _ => {
                    return Err(ErrorKind::UnexpectedValue(image.clone()).into());
                }
            }
        }
        Ok(())
    }
}
