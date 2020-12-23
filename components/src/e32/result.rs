use core::fmt;

use stm32f4xx_hal::serial;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    InvalidBaudRate,
    ParseConfigurationError(ParseConfigurationError),
    SerialError(serial::Error),
    SerdeDeserializeError(serde_json_core::de::Error),
    SerdeSerializeError(serde_json_core::ser::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use Error::*;

        match self {
            InvalidBaudRate => write!(
                f,
                "Could not configure module because of an invalid baud rate."
            ),
            ParseConfigurationError(e) => fmt::Display::fmt(&e, f),
            SerialError(e) => fmt::Debug::fmt(&e, f),
            SerdeDeserializeError(e) => fmt::Display::fmt(&e, f),
            SerdeSerializeError(e) => fmt::Display::fmt(&e, f),
        }
    }
}

#[derive(Debug)]
pub enum ParseConfigurationError {
    CouldNotParse,
    InvalidHead(u8),
    InvalidUartParity(u8),
    InvalidUartBaudRate(u8),
    InvalidAirDataRate(u8),
    InvalidTransmission(u8),
    InvalidWirelessWakeUpTime(u8),
    InvalidIoDriveMode(u8),
    InvalidTxPower(u8),
}

impl fmt::Display for ParseConfigurationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use ParseConfigurationError::*;

        match *self {
            CouldNotParse => write!(f, "Could not parse configuration."),
            InvalidHead(v) => write!(f, "Invalid head {}", v),
            InvalidUartParity(v) => write!(f, "Invalid uart parity {:#010b}", v),
            InvalidUartBaudRate(v) => write!(f, "Invalid uart baud rate {:#010b}", v),
            InvalidAirDataRate(v) => write!(f, "Invalid air data rate {:#010b}", v),
            InvalidTransmission(v) => write!(f, "Invalid transmission {:#010b}", v),
            InvalidWirelessWakeUpTime(v) => write!(f, "Invalid wireless wake up time {:#010b}", v),
            InvalidIoDriveMode(v) => write!(f, "Invalid io drive mode {:#010b}", v),
            InvalidTxPower(v) => write!(f, "Invalid tx power {:#010b}", v),
        }
    }
}
