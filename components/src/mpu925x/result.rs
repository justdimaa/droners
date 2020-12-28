use core::fmt;

use crate::{ak8963, mpu6500};

pub type Result<T, I2cError> = core::result::Result<T, Error<I2cError>>;

#[derive(Debug)]
pub enum Error<I2cError> {
    NotAvailable,
    Ak8963Error(ak8963::result::Error<I2cError>),
    Mpu6500Error(mpu6500::result::Error<I2cError>),
    AhrsUpdateAccelerometer,
    AhrsUpdateMagnetometer,
}

impl<I2cError> fmt::Display for Error<I2cError>
where
    I2cError: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use Error::*;

        match self {
            NotAvailable => write!(f, "Device is not ready for reading data."),
            Ak8963Error(e) => fmt::Display::fmt(&e, f),
            Mpu6500Error(e) => fmt::Display::fmt(&e, f),
            AhrsUpdateAccelerometer => write!(f, "Accelerometer norm divided by zero."),
            AhrsUpdateMagnetometer => write!(f, "Magnetometer norm divided by zero."),
        }
    }
}
