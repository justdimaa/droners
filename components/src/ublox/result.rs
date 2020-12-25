use core::fmt;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    NotEnoughData,
    SerialError, // todo: impl serial error
    ParserError(&'static str),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use Error::*;

        match &self {
            NotEnoughData => write!(f, "Not enough data in buffer to parse nmea sentence."),
            SerialError => write!(f, "A serial error occured."),
            ParserError(e) => fmt::Display::fmt(&e, f),
        }
    }
}
