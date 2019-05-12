#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum PinType {
    Power,
    Ground,
    In,
    Out,
    InOut,
    Unknown,
}

/// The definition of a connection point on a component
#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub struct Pin {
    pub name: String,
    pub pin_type: PinType,
    pub description: Option<String>,
}
