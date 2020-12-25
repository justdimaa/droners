use core::{convert::TryFrom, marker::PhantomData};

use embedded_hal::{
    digital::v2::OutputPin,
    prelude::{_embedded_hal_serial_Read, _embedded_hal_serial_Write},
};
use heapless::consts::*;
use stm32f4xx_hal::{pac, serial};

use self::{
    command::Command,
    config::ParameterSettings,
    op::{OperationCode, OperationMode},
    result::{Error, Result},
};

pub mod command;
pub mod config;
pub mod op;
pub mod result;

pub type SerialSize = U64;

#[derive(Debug)]
pub struct E32<USART> {
    air_data_rate: u8,
    uart_data_rate: u16,
    parity_bit: u8,
    speed: u8,
    _usart: PhantomData<USART>,
}

impl E32<pac::USART1> {
    pub fn new() -> Self {
        Self {
            air_data_rate: 0,
            uart_data_rate: 0,
            parity_bit: 0,
            speed: 0,
            _usart: PhantomData::default(),
        }
    }

    pub fn read_cfg<PINS, M0: OutputPin, M1: OutputPin>(
        &mut self,
        serial: &mut serial::Serial<pac::USART1, PINS>,
        m0_pin: &mut M0,
        m1_pin: &mut M1,
    ) -> Result<()> {
        self.set_mode(OperationMode::Sleep, m0_pin, m1_pin)?;
        self.write_op(serial, OperationCode::ReadCfg)?;

        Ok(())
    }

    pub fn read_cfg_callback<PINS>(
        &mut self,
        serial: &mut serial::Serial<pac::USART1, PINS>,
    ) -> Result<ParameterSettings> {
        let mut buf = [0; 6];
        let mut i = 0;

        while serial.is_rxne() && i != buf.len() {
            match nb::block!(serial.read()) {
                Ok(v) => {
                    buf[i] = v;
                    i += 1;
                }
                Err(e) => return Err(Error::SerialError(e)),
            };
        }

        ParameterSettings::try_from(buf)
    }

    pub fn write_cfg<PINS, M0: OutputPin, M1: OutputPin>(
        &mut self,
        _serial: &mut serial::Serial<pac::USART1, PINS>,
        _cfg: ParameterSettings,
        m0_pin: &mut M0,
        m1_pin: &mut M1,
    ) -> Result<()> {
        self.set_mode(OperationMode::Normal, m0_pin, m1_pin)?;

        Ok(())
    }

    fn set_mode<M0: OutputPin, M1: OutputPin>(
        &self,
        op_mode: OperationMode,
        m0_pin: &mut M0,
        m1_pin: &mut M1,
    ) -> Result<()> {
        if self.uart_data_rate != 9600 {
            return Err(Error::InvalidBaudRate);
        }

        match op_mode {
            OperationMode::Normal => {
                m0_pin.set_low().ok();
                m1_pin.set_low().ok();
            }
            OperationMode::WakeUp => {
                m0_pin.set_high().ok();
                m1_pin.set_low().ok();
            }
            OperationMode::PowerSaving => {
                m0_pin.set_low().ok();
                m1_pin.set_high().ok();
            }
            OperationMode::Sleep => {
                m0_pin.set_high().ok();
                m1_pin.set_high().ok();
            }
        }

        Ok(())
    }

    fn write_op<PINS>(
        &self,
        serial: &mut serial::Serial<pac::USART1, PINS>,
        op_code: OperationCode,
    ) -> Result<()> {
        let op_code = op_code as u8;

        for _ in 0..3 {
            match nb::block!(serial.write(op_code)) {
                Ok(()) => {}
                Err(e) => return Err(Error::SerialError(e)),
            }
        }

        Ok(())
    }

    pub fn read<PINS>(
        &mut self,
        serial: &mut serial::Serial<pac::USART1, PINS>,
    ) -> Result<Option<Command>> {
        let mut buf = heapless::Vec::<_, SerialSize>::new();

        while serial.is_rxne() {
            if buf.capacity() > buf.len() {
                break;
            }

            match nb::block!(serial.read()) {
                Ok(v) => buf.push(v).unwrap(),
                Err(e) => return Err(Error::SerialError(e)),
            }

            if let Some(cmd) = Command::decode(&buf) {
                return Ok(Some(cmd));
            }
        }

        Ok(None)
    }

    pub fn write<PINS>(
        &mut self,
        serial: &mut serial::Serial<pac::USART1, PINS>,
        cmd: &Command,
        addr: u16,
        channel: u8,
    ) -> Result<()> {
        let mut buf = heapless::Vec::<_, SerialSize>::new();
        buf[0..1].copy_from_slice(&addr.to_be_bytes());
        buf[2] = channel;
        cmd.encode(&mut buf[3..]);

        for v in &buf {
            match nb::block!(serial.write(*v)) {
                Ok(()) => {}
                Err(e) => return Err(Error::SerialError(e)),
            }
        }

        Ok(())
    }
}
