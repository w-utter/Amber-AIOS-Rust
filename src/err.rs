pub enum Err {
    Serde(serde_json::Error),
    Io(nix::Error),
    Binary(binary::Error),
    UnexpectedReturn,
}

impl From<serde_json::Error> for Err {
    fn from(err: serde_json::Error) -> Err {
        Err::Serde(err)
    }
}

impl From<nix::Error> for Err {
    fn from(err: nix::Error) -> Err {
        Err::Io(err)
    }
}

impl From<binary::Error> for Err {
    fn from(err: binary::Error) -> Err {
        Err::Binary(err)
    }
}

#[derive(Debug)]
pub struct Expected<T> {
    pub expected: T,
    pub got: T,
}

impl<T> Expected<T> {
    pub(crate) fn new(got: T, expected: T) -> Self {
        Self { expected, got }
    }

    pub fn format_expected(&self) -> String
    where
        T: std::fmt::Debug,
    {
        format!("expected `{:?}`, received `{:?}`.", self.expected, self.got)
    }
}

pub(crate) mod binary {
    use super::Expected;
    #[derive(Debug)]
    pub enum Error {
        Failed,
        BadSize(Expected<usize>),
        BadMsgId(Expected<u8>),
    }

    impl std::fmt::Display for Error {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            match self {
                Self::Failed => write!(f, "Unknown motor error, status: `FAILED!`")?,
                Self::BadSize(s) => write!(f, "Unexpected message size: {}", s.format_expected())?,
                Self::BadMsgId(m) => write!(f, "Unexpected message id: {}", m.format_expected())?,
            }

            <Self as std::fmt::Debug>::fmt(self, f)
        }
    }

    impl std::error::Error for Error {}
}
