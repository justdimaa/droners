use core::marker::PhantomData;

use embedded_hal::serial;
pub use nmea0183 as parser;
pub mod result;

use result::Error;
use result::Result;

pub struct Ublox<SERIAL>
where
    SERIAL: serial::Read<u8> + serial::Write<u8>,
{
    parser: parser::Parser,
    _serial: PhantomData<SERIAL>,
}

impl<SERIAL> Ublox<SERIAL>
where
    SERIAL: serial::Read<u8> + serial::Write<u8>,
{
    pub fn new() -> Self {
        Self {
            parser: parser::Parser::new(),
            _serial: PhantomData::default(),
        }
    }

    pub fn read(&mut self, serial: &mut SERIAL) -> Result<parser::ParseResult> {
        match nb::block!(serial.read()) {
            Ok(v) => match self.parser.parse_from_byte(v) {
                Some(v) => match v {
                    Ok(v) => Ok(v),
                    Err(e) => Err(Error::ParserError(e)),
                },
                None => Err(Error::NotEnoughData),
            },
            Err(_) => Err(Error::SerialError),
        }
    }
}
