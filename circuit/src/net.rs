use std::cmp::Ordering;
use std::sync::atomic::AtomicUsize;

/// Defines a connected set of terminals
#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub struct Net {
    pub name: String,
}

static NEXT_NET_ID: AtomicUsize = AtomicUsize::new(0);

impl Net {
    pub fn new() -> Self {
        let id = NEXT_NET_ID.fetch_add(1, ::std::sync::atomic::Ordering::Relaxed);
        Net {
            name: format!("N${}", id),
        }
    }

    pub fn with_name<S: Into<String>>(s: S) -> Self {
        Self { name: s.into() }
    }

    pub fn is_auto_net(&self) -> bool {
        self.name.starts_with("N$")
    }
}

impl Default for Net {
    fn default() -> Self {
        Net::new()
    }
}

impl PartialOrd for Net {
    fn partial_cmp(&self, other: &Net) -> Option<Ordering> {
        if self.is_auto_net() == other.is_auto_net() {
            self.name.partial_cmp(&other.name)
        } else if !self.is_auto_net() {
            Some(Ordering::Less)
        } else {
            Some(Ordering::Greater)
        }
    }
}

impl Ord for Net {
    fn cmp(&self, other: &Net) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}
