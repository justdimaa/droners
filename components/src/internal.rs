use core::marker::PhantomData;

use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write};
use heapless::consts::*;
use serde::{Deserialize, Serialize};
use stm32f4xx_hal::{i2c, pac};

pub const NAV_ADDR: u8 = 0x21;
pub const FC_ADDR: u8 = 0x22;

pub type I2cSize = U64;

#[derive(Debug, Serialize, Deserialize)]
pub enum Command {
    Throttle { esc_id: u8, value: u16 },
}

impl Command {
    pub fn encode(&self) -> heapless::Vec<u8, I2cSize> {
        let mut buf = heapless::Vec::<_, I2cSize>::new();
        serde_json_core::to_slice(self, &mut buf[..]).unwrap();
        buf
    }

    pub fn decode(buf: heapless::Vec<u8, I2cSize>) -> Option<Command> {
        let cmd = serde_json_core::from_slice::<Command>(&buf[..]);

        match cmd {
            Ok((cmd, _)) => Some(cmd),
            Err(_) => None,
        }
    }
}

#[derive(Debug)]
pub struct Navigation<I2C> {
    _i2c: PhantomData<I2C>,
}

impl Navigation<pac::I2C1> {
    pub fn new() -> Self {
        Self {
            _i2c: PhantomData::default(),
        }
    }

    pub fn read<PINS>(
        &self,
        i2c: &mut i2c::I2c<pac::I2C1, PINS>,
    ) -> Result<Option<Command>, i2c::Error> {
        let mut buf = heapless::Vec::<_, I2cSize>::new();
        i2c.read(NAV_ADDR, &mut buf[..])?;
        Ok(Command::decode(buf))
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

    pub fn send<PINS>(
        &self,
        i2c: &mut i2c::I2c<pac::I2C1, PINS>,
        cmd: Command,
    ) -> Result<(), i2c::Error> {
        let buf = cmd.encode();
        i2c.write(FC_ADDR, &buf[..])?;
        Ok(())
    }
}
