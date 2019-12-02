#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash, Ord, PartialOrd)]
pub enum PinDrive {
    /// Not connected at all, so no drive
    NotConnected = 0,
    /// No drive capability; eg: an input pin
    None = 1,
    /// Small drive capability, less than pull-up or pull-down
    Passive = 2,
    /// Pull up or pull-down drive
    PullUpOrDown = 3,
    /// Can pull high (open emitter) or low (open collector)
    OneSide = 4,
    /// Can pull high or low and be in a high-impedance state
    TriState = 5,
    /// Can actively drive high or low
    PushPull = 6,
    /// A power supply or ground line
    Power = 7,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash, Ord, PartialOrd)]
pub enum PinType {
    PowerInput,
    PowerOutput,
    In,
    Out,
    InOut,
    NotConnected,
    Unknown,
    TriState,
    Passive,
    OpenCollector,
    OpenEmitter,
}

impl PinType {
    /// Returns the drive strength associated with this type of pin
    pub fn drive_strength(self) -> PinDrive {
        match self {
            PinType::PowerInput => PinDrive::None,
            PinType::PowerOutput => PinDrive::Power,
            PinType::In => PinDrive::None,
            PinType::Out => PinDrive::PushPull,
            PinType::InOut => PinDrive::TriState,
            PinType::NotConnected => PinDrive::NotConnected,
            PinType::Unknown => PinDrive::None,
            PinType::TriState => PinDrive::TriState,
            PinType::Passive => PinDrive::Passive,
            PinType::OpenCollector => PinDrive::OneSide,
            PinType::OpenEmitter => PinDrive::OneSide,
        }
    }

    /// Returns the minimum amount of drive that the pin must receive
    /// in order to function
    pub fn min_input_drive(self) -> PinDrive {
        match self {
            PinType::PowerInput => PinDrive::Power,
            PinType::PowerOutput => PinDrive::None,
            PinType::In => PinDrive::Passive,
            PinType::Out => PinDrive::None,
            PinType::InOut => PinDrive::None,
            PinType::NotConnected => PinDrive::NotConnected,
            PinType::Unknown => PinDrive::None,
            PinType::TriState => PinDrive::None,
            PinType::Passive => PinDrive::None,
            PinType::OpenCollector => PinDrive::None,
            PinType::OpenEmitter => PinDrive::None,
        }
    }

    /// Returns the maximum amount of drive that the pin can receive
    /// and still function
    pub fn max_input_drive(self) -> PinDrive {
        match self {
            PinType::PowerInput => PinDrive::Power,
            PinType::PowerOutput => PinDrive::Passive,
            PinType::In => PinDrive::Power,
            PinType::Out => PinDrive::Passive,
            PinType::InOut => PinDrive::Power,
            PinType::NotConnected => PinDrive::NotConnected,
            PinType::Unknown => PinDrive::Power,
            PinType::TriState => PinDrive::TriState,
            PinType::Passive => PinDrive::Power,
            PinType::OpenCollector => PinDrive::TriState,
            PinType::OpenEmitter => PinDrive::TriState,
        }
    }

    pub fn incompatible_pin_pairing(self, other: PinType) -> bool {
        // These might be redundant with the drive strength checks;
        // happy to remove them if that proves to be true
        match (self, other) {
            (PinType::Out, PinType::Out) => true,
            (PinType::PowerOutput, PinType::TriState) => true,
            (PinType::PowerOutput, PinType::Out) => true,
            (PinType::PowerOutput, PinType::PowerOutput) => true,
            (PinType::OpenCollector, PinType::Out) => true,
            (PinType::OpenCollector, PinType::TriState) => true,
            (PinType::OpenCollector, PinType::PowerOutput) => true,
            (PinType::OpenEmitter, PinType::Out) => true,
            (PinType::OpenEmitter, PinType::PowerOutput) => true,
            (PinType::NotConnected, _) => true,
            _ => false,
        }
    }
}

/// The definition of a connection point on a component
#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub struct Pin {
    pub name: String,
    pub pin_type: PinType,
    pub description: Option<String>,
}
