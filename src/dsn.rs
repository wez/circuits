use std::fs::File;
use std::io::prelude::*;
use std::default::Default;
extern crate error_chain;
use std::str;
extern crate nom;
use geom;
use itertools::Itertools;
use std::collections::HashMap;

#[derive(Clone, Debug)]
enum Value {
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
        NoStringRep
        NoIntRep
        NoFloatRep
        UnhandledShapeType(a: String) {
            description("no parser for a shape type")
            display("unhandled shape type {}", a)
        }
        ParseError(a: &'static str) {
            description(".dsn file parse error")
            display("parse error: {}", a)
        }
        Nom(e: String) {
            description("parse error")
            display("parse error {}", e)
        }
    }
}

impl Value {
    pub fn as_string(&self) -> Result<&String> {
        match self {
            &Value::Quoted(ref s) => Ok(s),
            &Value::Literal(ref s) => Ok(s),
            _ => Err(ErrorKind::NoStringRep.into()),
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
pub struct Pcb {
    pub file_name: String,
    pub parser: Parser,
    pub structure: Structure,
    pub unit: String,
    pub components: Vec<Component>,
    pub component_defs: HashMap<String, ComponentDef>,
}

impl DsnShape {
    //! parses a tagged list into a shape instance
    fn parse(tag: &String, list: &Vec<Value>) -> Result<DsnShape> {
        match tag.as_ref() {
            "path" | "polygon" => DsnShape::parse_path(list),
            "circle" => DsnShape::parse_circle(list),
            _ => Err(ErrorKind::UnhandledShapeType(tag.to_string()).into()),
        }
    }

    fn parse_circle(list: &Vec<Value>) -> Result<DsnShape> {
        let layer = list[0].as_string()?;

        let radius = list[1].as_f64()?;
        let (x, y) = {
            if list.len() > 2 {
                (list[2].as_f64()?, list[3].as_f64()?)
            } else {
                (0.0, 0.0)
            }
        };

        Ok(DsnShape {
               layer: layer.clone(),
               shape: geom::Shape::circle(radius / 2.0,
                                          geom::Location::new(geom::Vector::new(x, y), 0.0)),
           })
    }

    fn parse_path(list: &Vec<Value>) -> Result<DsnShape> {
        let layer = list[0].as_string()?;

        let mut points = Vec::new();
        // skip 2 because 0 is the layer that we parsed and index 1 is
        // the aperture width.
        for (x, y) in list.iter().skip(2).tuples() {
            points.push(geom::Point::new(x.as_f64()?, y.as_f64()?));
        }

        Ok(DsnShape {
               layer: layer.clone(),
               shape: geom::Shape::polygon(points, geom::origin()),
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

        match &v {
            &Value::TaggedList(_, ref list) => {
                for ele in list.iter().skip(1) {
                    match ele {
                        &Value::TaggedList(ref k, ref list) => {
                            match k.as_ref() {
                                "parser" => {
                                    pcb.parser.from_list(list)?;
                                }
                                "structure" => {
                                    pcb.structure.from_list(list)?;
                                }
                                "placement" => {
                                    pcb.component_list(list)?;
                                }
                                "library" => {
                                    pcb.component_def_list(list)?;
                                }
                                _ => {
                                    println!("unhandled key Pcb::{}", k);
                                }
                            }
                        }
                        &Value::TaggedValue(ref k, ref v) => {
                            match k.as_ref() {
                                "unit" => {
                                    pcb.unit = v.as_string()?.clone();
                                }
                                _ => {
                                    println!("unhandled key Pcb::{}", k);
                                }
                            }
                        }
                        _ => {
                            println!("unhanded Pcb: {:?}", ele);
                        }
                    }
                }
                Ok(pcb)
            }
            _ => Err(ErrorKind::ParseError("expected (pcb ...) but got something else").into()),
        }
    }

    fn component_list(&mut self, list: &Vec<Value>) -> Result<()> {
        for ele in list.iter() {
            match ele {
                &Value::TaggedList(_, ref list) => {
                    let component_type = list[0].as_string()?;
                    println!("component_type {}", component_type);
                    for place in list.iter().skip(1) {
                        match place {
                            &Value::TaggedList(_, ref list) => {
                                let name = list[0].as_string()?;
                                let x = list[1].as_f64()?;
                                let y = list[2].as_f64()?;
                                //let side = list[3].as_string()?;
                                let rotation = list[4].as_f64()?;
                                self.components
                                    .push(Component {
                                              component_type: component_type.clone(),
                                              instance_name: name.clone(),
                                              position: geom::Location::new(geom::Vector::new(x,
                                                                                              y),
                                                                            rotation.to_radians()),
                                          })
                            }
                            _ => {}
                        }
                    }
                }
                _ => {
                    println!("unhandled component list entry: {:?}", ele);
                }
            }
        }
        Ok(())
    }

    fn image(&mut self, list: &Vec<Value>) -> Result<()> {
        let mut def = ComponentDef::default();

        def.component_type = list[0].as_string()?.clone();
        println!("component_def {}", def.component_type);

        for ele in list.iter().skip(1) {
            match ele {
                &Value::TaggedValue(ref tag, ref val) => {
                    match tag.as_ref() {
                        "outline" => {
                            match &**val {
                                &Value::TaggedList(ref t, ref l) => {
                                    println!("doing an outline! {} {:?}", t, l);
                                    def.outlines.push(DsnShape::parse(t, l)?);
                                }
                                _ => {
                                    println!("unhandled outline {:?}", val);
                                }
                            }
                        }
                        _ => {
                            println!("unahdnled outline! {} {:?}", tag, val);
                        }
                    }
                }
                &Value::TaggedList(ref tag, ref list) => {
                    match tag.as_ref() {
                        "pin" => {}
                        "keepout" => {
                            match &list[1] {
                                &Value::TaggedList(ref tag, ref list) => {
                                    def.keepout.push(DsnShape::parse(tag, list)?);
                                }
                                _ => {
                                    println!("unhandled image::keepout{:?}", list);
                                }
                            }
                        }
                        _ => {
                            println!("unhandled tag in library.image: {:?}", ele);
                        }
                    }
                }
                _ => {}
            }
        }

        self.component_defs
            .insert(def.component_type.clone(), def);
        Ok(())
    }

    fn component_def_list(&mut self, list: &Vec<Value>) -> Result<()> {
        for image in list.iter() {
            match image {
                &Value::TaggedList(ref tag, ref list) => {
                    match tag.as_ref() {
                        "image" => {
                            self.image(list)?;
                        }
                        _ => {
                            println!("unhandled {:?}", image);
                        }
                    }
                }
                _ => {
                    println!("unhandled library entry: {:?}", image);
                }
            }
        }
        Ok(())
    }
}
