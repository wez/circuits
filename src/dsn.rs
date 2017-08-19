use std::fs::File;
use std::io::prelude::*;
use std::default::Default;
extern crate error_chain;
use std::str;
extern crate nom;
use geom;
use itertools::Itertools;
use std::collections::{HashMap, HashSet};

#[derive(Clone, Debug)]
pub enum Value {
    Literal(String),
    Quoted(String),
    Integer(i64),
    Float(f64),
    TaggedList(String, Vec<Value>),
    TaggedValue(String, Box<Value>),
}

error_chain! {
    foreign_links {
        Io(::std::io::Error);
    }
    errors {
        NoStringRep(value: Value) {
            description("no string representation")
            display("no string representation for {:?}", value)
        }
        NoIntRep
        NoFloatRep
        UnhandledShapeType(a: String) {
            description("no parser for a shape type")
            display("unhandled shape type {}", a)
        }
        UnexpectedTag(a: &'static str, value: Value) {
            description("unexpected tag")
            display("expected ({} ...) but got {:?}", a, value)
        }
        ExpectedTaggedList(value: Value) {
            description("expected tagged list")
            display("expected (tag ...) but got {:?}", value)
        }
        UnexpectedValue(value: Value) {
            description("unexpected value")
            display("unexpected value {:?}", value)
        }
        Nom(e: String) {
            description("parse error")
            display("parse error {}", e)
        }
        PinError
    }
}

impl Value {
    pub fn as_string(&self) -> Result<&String> {
        match self {
            &Value::Quoted(ref s) => Ok(s),
            &Value::Literal(ref s) => Ok(s),
            _ => Err(ErrorKind::NoStringRep(self.clone()).into()),
        }
    }

    pub fn as_i64(&self) -> Result<i64> {
        match self {
            &Value::Integer(ref s) => Ok(*s),
            _ => Err(ErrorKind::NoIntRep.into()),
        }
    }

    pub fn as_f64(&self) -> Result<f64> {
        match self {
            &Value::Float(ref s) => Ok(*s),
            &Value::Integer(ref s) => Ok(*s as f64),
            _ => Err(ErrorKind::NoFloatRep.into()),
        }
    }

    pub fn as_tagged_list(&self) -> Result<(&String, Vec<Value>)> {
        match self {
            &Value::TaggedList(ref tag, ref list) => Ok((tag, list.clone())),
            &Value::TaggedValue(ref tag, ref value) => {
                let v = (**value).clone();
                Ok((tag, vec![v]))
            }
            _ => Err(ErrorKind::ExpectedTaggedList(self.clone()).into()),
        }
    }

    pub fn as_tagged_list_with_name(&self, name: &'static str) -> Result<Vec<Value>> {
        match self {
            &Value::TaggedList(ref tag, ref list) => {
                if tag == name {
                    Ok(list.clone())
                } else {
                    Err(ErrorKind::UnexpectedTag(name, self.clone()).into())
                }
            }
            &Value::TaggedValue(ref tag, ref value) => {
                if tag == name {
                    let v = (**value).clone();
                    Ok(vec![v])
                } else {
                    Err(ErrorKind::UnexpectedTag(name, self.clone()).into())
                }
            }
            _ => Err(ErrorKind::UnexpectedTag(name, self.clone()).into()),
        }
    }
}


named!(string_quote_parser<&[u8], Value>, do_parse!(
    tag!("(string_quote \")") >>
    (Value::TaggedValue(
            "string_quote".to_owned(),
            Box::new(Value::Literal("\"".to_owned()))
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

fn parse_tagged(t: &str, mut args: Vec<Value>) -> Value {
    match args.len() {
        1 => Value::TaggedValue(t.to_owned(), Box::new(args.remove(0))),
        _ => Value::TaggedList(t.to_owned(), args),
    }
}

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
    (parse_tagged(t, args))
));

fn parse_value(bytes: &[u8]) -> Result<Value> {
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

#[derive(Debug)]
pub struct DsnShape {
    pub layer: String,
    pub shape: geom::Shape,
}


#[derive(Default, Debug)]
pub struct Parser {
    pub string_quote: String,
    pub space_in_quoted_tokens: String,
    pub host_cad: String,
    pub host_version: String,
}

#[derive(Default, Debug)]
pub struct Layer {
    pub name: String,
    pub layer_type: String,
    pub index: i64,
}

#[derive(Default, Debug)]
pub struct Structure {
    pub layers: Vec<Layer>,
    pub boundary: Vec<DsnShape>,
    pub keepout: Vec<DsnShape>,
}

#[derive(Debug)]
pub struct Component {
    pub component_type: String,
    pub instance_name: String,
    pub position: geom::Location,
}

#[derive(Default, Debug)]
pub struct PadStack {
    pub pad_type: String,
    pub pads: HashMap<String, DsnShape>,
    pub attach: bool,
}

#[derive(Debug)]
pub struct Pin {
    pub pad_type: String,
    pub pad_num: i64,
    pub position: geom::Location,
}

#[derive(Default, Debug)]
pub struct ComponentDef {
    pub component_type: String,
    pub outlines: Vec<DsnShape>,
    pub pins: Vec<Pin>,
    pub keepout: Vec<DsnShape>,
}

#[derive(Default, Debug)]
pub struct Net {
    pub pins: Vec<String>,
}

#[derive(Default, Debug)]
pub struct NetClass {
    pub class_name: String,
    pub nets: HashSet<String>,
}

#[derive(Default, Debug)]
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
    fn parse(tag: &String, list: &Vec<Value>) -> Result<DsnShape> {
        match tag.as_ref() {
            "path" | "polygon" => DsnShape::parse_path(list),
            "circle" => DsnShape::parse_circle(list),
            "rect" => DsnShape::parse_rect(list),
            _ => Err(ErrorKind::UnhandledShapeType(tag.to_string()).into()),
        }
    }

    fn parse_circle(list: &Vec<Value>) -> Result<DsnShape> {
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
               shape: geom::Shape::circle(diameter / 2.0,
                                          geom::Location::new(geom::Vector::new(x, y), 0.0)),
           })
    }

    fn parse_path(list: &Vec<Value>) -> Result<DsnShape> {
        let layer = list[0].as_string()?;

        let mut points = Vec::new();
        // skip 2 because 0 is the layer that we parsed and index 1 is
        // the aperture width.
        let aperture = list[1].as_f64()? / 2.0;
        let width = if aperture == 0.0 {
            None
        } else {
            Some(aperture)
        };

        for (x, y) in list.iter().skip(2).tuples() {
            points.push(geom::Point::new(x.as_f64()?, y.as_f64()?));
        }

        Ok(DsnShape {
               layer: layer.clone(),
               shape: geom::Shape::polygon(points, geom::origin(), width),
           })
    }

    fn parse_rect(list: &Vec<Value>) -> Result<DsnShape> {
        let layer = list[0].as_string()?;


        let (bottom_left_x, bottom_left_y) = (list[1].as_f64()?, list[2].as_f64()?);
        let (top_right_x, top_right_y) = (list[3].as_f64()?, list[4].as_f64()?);

        let points = vec![geom::Point::new(bottom_left_x, bottom_left_y),
                          geom::Point::new(bottom_left_x, top_right_y),
                          geom::Point::new(top_right_x, top_right_y),
                          geom::Point::new(top_right_x, bottom_left_y),
                          geom::Point::new(bottom_left_x, bottom_left_y)];

        Ok(DsnShape {
               layer: layer.clone(),
               shape: geom::Shape::polygon(points, geom::origin(), None),
           })
    }
}

impl Parser {
    //! parses a value list into an existing Parser object
    fn from_list(&mut self, list: &Vec<Value>) -> Result<()> {
        for ele in list.iter() {
            match ele {
                &Value::TaggedValue(ref k, ref v) => {
                    match k.as_ref() {
                        "string_quote" => {
                            self.string_quote = v.as_string()?.clone();
                        }
                        "space_in_quoted_tokens" => {
                            self.space_in_quoted_tokens = v.as_string()?.clone();
                        }
                        "host_cad" => {
                            self.host_cad = v.as_string()?.clone();
                        }
                        "host_version" => {
                            self.host_version = v.as_string()?.clone();
                        }
                        _ => {
                            println!("unhandled key Parser::{}", k);
                        }
                    }
                }
                _ => {
                    println!("unhandled Parser::{:?}", ele);
                }
            }
        }

        Ok(())
    }
}

impl Layer {
    //! parses a value list into a new Layer object
    fn from_list(list: &Vec<Value>) -> Result<Layer> {
        let mut l = Layer::default();

        l.name = list[0].as_string()?.clone();
        for ele in list.iter().skip(1) {
            match ele {
                &Value::TaggedValue(ref k, ref v) => {
                    match k.as_ref() {
                        "type" => {
                            l.layer_type = v.as_string()?.clone();
                        }
                        "property" => {
                            match **v {
                                Value::TaggedValue(ref name, ref val) => {
                                    if name == "index" {
                                        l.index = val.as_i64()?;
                                    }
                                }
                                _ => {
                                    println!("unhandled key Layer::property::{:?}", v);
                                }
                            }
                        }
                        _ => {
                            println!("unhandled key Layer::{}", k);
                        }
                    }
                }
                _ => {
                    println!("unhandled Layer::{:?}", ele);
                }
            }
        }

        Ok(l)
    }
}

impl Structure {
    //! parses a value list and populates an existing Structure instance
    fn from_list(&mut self, list: &Vec<Value>) -> Result<()> {
        for ele in list.iter() {
            match ele {
                &Value::TaggedList(ref k, ref list) => {
                    match k.as_ref() {
                        "layer" => {
                            self.layers.push(Layer::from_list(list)?);
                        }
                        "keepout" => {
                            match &list[1] {
                                &Value::TaggedList(ref tag, ref list) => {
                                    self.keepout.push(DsnShape::parse(tag, list)?);
                                }
                                _ => {
                                    println!("unhandled Structure::keepout{:?}", list);
                                }
                            }
                        }
                        _ => {
                            println!("unhandled key Structure::{}", k);
                        }
                    }
                }
                &Value::TaggedValue(ref tag, ref val) => {
                    match tag.as_ref() {
                        "boundary" => {
                            match &**val {
                                &Value::TaggedList(ref tag, ref list) => {
                                    self.boundary.push(DsnShape::parse(tag, list)?);
                                }
                                _ => {
                                    println!("unhandled ele in Structure::boundary {:?}", ele);
                                }
                            }
                        }
                        _ => {
                            println!("unhandled TaggedValue Structure::{} {:?}", tag, val);
                        }
                    }
                }
                _ => {
                    println!("unhandled Structure::{:?}", ele);
                }
            }
        }
        Ok(())
    }
}


impl Pcb {
    //! parses the contents of filename into a Pcb object
    // the file is assumed to be a spectra dsn file, and was implemented
    // based on a dsn file produced by a recent kicad version.
    pub fn parse(filename: &str) -> Result<Pcb> {
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
                    pcb.parser.from_list(&list)?;
                }
                "structure" => {
                    pcb.structure.from_list(&list)?;
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
                _ => {
                    println!("unhandled key Pcb::{:?}", ele);
                }
            }
        }
        Ok(pcb)
    }

    fn network_list(&mut self, list: &Vec<Value>) -> Result<()> {
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

    fn component_list(&mut self, list: &Vec<Value>) -> Result<()> {
        for ele in list.iter() {
            let list = ele.as_tagged_list_with_name("component")?;
            let component_type = list[0].as_string()?;
            for place in list.iter().skip(1) {
                let list = place.as_tagged_list_with_name("place")?;
                let name = list[0].as_string()?;
                let x = list[1].as_f64()?;
                let y = list[2].as_f64()?;
                //let side = list[3].as_string()?;
                let rotation = list[4].as_f64()?;
                self.components
                    .push(Component {
                              component_type: component_type.clone(),
                              instance_name: name.clone(),
                              position: geom::Location::new(geom::Vector::new(x, y),
                                                            rotation.to_radians()),
                          })
            }
        }
        Ok(())
    }

    fn padstack(&mut self, list: &Vec<Value>) -> Result<()> {
        let mut def = PadStack::default();
        def.pad_type = list[0].as_string()?.clone();

        for ele in list.iter().skip(1) {
            let (tag, list) = ele.as_tagged_list()?;
            match tag.as_ref() {
                "shape" => {
                    let (t, l) = list[0].as_tagged_list()?;
                    let shape = DsnShape::parse(t, &l)?;
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

    fn image(&mut self, list: &Vec<Value>) -> Result<()> {
        let mut def = ComponentDef::default();
        def.component_type = list[0].as_string()?.clone();

        for ele in list.iter().skip(1) {
            let (tagname, list) = ele.as_tagged_list()?;
            match tagname.as_ref() {
                "outline" => {
                    let (t, l) = list[0].as_tagged_list()?;
                    def.outlines.push(DsnShape::parse(t, &l)?);
                }
                "pin" => {
                    def.pins.push(self.parse_pin(&list)?);
                }
                "keepout" => {
                    let (t, l) = list[1].as_tagged_list()?;
                    def.keepout.push(DsnShape::parse(t, &l)?);
                }
                _ => {
                    return Err(ErrorKind::UnexpectedValue(ele.clone()).into());
                }
            }
        }

        self.component_defs
            .insert(def.component_type.clone(), def);
        Ok(())
    }

    fn parse_pin(&self, list: &Vec<Value>) -> Result<Pin> {
        if list.len() == 4 {
            Ok(Pin {
                   pad_type: list[0].as_string()?.clone(),
                   pad_num: list[1].as_i64()?,
                   position: geom::Location::new(geom::Vector::new(list[2].as_f64()?,
                                                                   list[3].as_f64()?),
                                                 0.0),
               })
        } else {
            match &list[1] {
                &Value::TaggedValue(_, ref rot) => {
                    let degrees = rot.as_f64()?;
                    Ok(Pin {
                           pad_type: list[0].as_string()?.clone(),
                           pad_num: list[2].as_i64()?,
                           position: geom::Location::new(geom::Vector::new(list[3].as_f64()?,
                                                                           list[4].as_f64()?),
                                                         degrees.to_radians()),
                       })
                }
                _ => Err(ErrorKind::PinError.into()),
            }
        }
    }

    fn component_def_list(&mut self, list: &Vec<Value>) -> Result<()> {
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
