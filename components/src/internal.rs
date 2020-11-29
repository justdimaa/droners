use core::marker::PhantomData;

use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
use stm32f4xx_hal::{i2c, pac};

const FC_ADDR: u8 = 0x22;
const FC_CMD_THROTTLE: u8 = 0x01;

#[derive(Debug)]
pub struct Navigation {}

impl Navigation {
    pub fn new() -> Self {
        Self {}
    }
}

#[derive(Debug)]
pub struct FlightControl<I2C> {
    _i2c: PhantomData<I2C>,
}

impl FlightControl<pac::I2C1> {
    pub fn new() -> Self {
        Self {
            _i2c: PhantomData::default(),
        }
    }

    pub fn set_throttle<PINS>(
        &self,
        i2c: &mut i2c::I2c<pac::I2C1, PINS>,
        esc_id: u8,
        value: u16,
    ) -> Result<(), i2c::Error> {
        let value_le = value.to_le_bytes();
        i2c.write(
            FC_ADDR,
            &[FC_CMD_THROTTLE, esc_id, value_le[0], value_le[1]],
        )
    }
}
