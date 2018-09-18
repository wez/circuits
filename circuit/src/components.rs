use {Component, Pin, PinType};

pub fn diode() -> Component {
    Component {
        name: "DIODE".into(),
        description: None,
        pins: vec![
            Pin {
                name: "K".into(),
                description: Some("Cathode".into()),
                pin_type: PinType::In,
            },
            Pin {
                name: "A".into(),
                description: Some("Anode".into()),
                pin_type: PinType::In,
            },
        ],
    }
}

pub fn switch() -> Component {
    Component {
        name: "SW".into(),
        description: None,
        pins: vec![
            Pin {
                name: "1".into(),
                description: None,
                pin_type: PinType::Unknown,
            },
            Pin {
                name: "2".into(),
                description: None,
                pin_type: PinType::Unknown,
            },
        ],
    }
}
