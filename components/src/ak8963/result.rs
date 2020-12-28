use core::fmt;

pub type Result<T, I2cError> = core::result::Result<T, Error<I2cError>>;

#[derive(Debug)]
pub enum Error<I2cError> {
    InvalidDevice(u8),
    DataNotReady,
    Overflow,
    I2cError(I2cError),
}

impl<I2cError> fmt::Display for Error<I2cError>
where
    I2cError: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use Error::*;

        match self {
            InvalidDevice(v) => write!(f, "Invalid device with id {}", v),
            DataNotReady => write!(f, "Cannot fetch data because the device is not ready yet."),
            Overflow => write!(f, "Cannot fetch data on an overflow."),
            I2cError(e) => fmt::Debug::fmt(&e, f),
        }
    }
}
