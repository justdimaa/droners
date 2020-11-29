use core::marker::PhantomData;

use embedded_hal::prelude::{_embedded_hal_serial_Read, _embedded_hal_serial_Write};
use stm32f4xx_hal::{nb, pac, serial};

#[derive(Debug)]
pub struct E32<USART> {
    air_data_rate: u8,
    uart_data_rate: u8,
    parity_bit: u8,
    speed: u8,
    _usart: PhantomData<USART>,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct ControllerMessage {
    pub left_thumb_x: u16,
    pub left_thumb_y: u16,
    pub left_trigger: u8,
    pub right_thumb_x: u16,
    pub right_thumb_y: u16,
    pub right_trigger: u8,
    pub buttons: u16,
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

    pub fn set_air_data_rate(&mut self, value: u8) {
        self.air_data_rate = value;
        self.build_speed_byte();
    }

    pub fn get_air_data_rate(&self) -> u8 {
        self.air_data_rate
    }

    pub fn set_parity_bit(&mut self, value: u8) {
        self.parity_bit = value;
        self.build_speed_byte();
    }

    pub fn get_parity_bit(&self) -> u8 {
        self.parity_bit
    }

    fn build_speed_byte(&mut self) {
        self.speed = 0;
        self.speed = (self.parity_bit << 6) | (self.uart_data_rate << 3) | (self.air_data_rate);
    }

    fn save_parameters<PINS>(
        &mut self,
        serial: &mut serial::Serial<pac::USART1, PINS>,
    ) -> nb::Result<(), serial::Error> {
        serial.flush()?;

        Ok(())
    }

    pub fn read<PINS>(
        &mut self,
        serial: &mut serial::Serial<pac::USART1, PINS>,
    ) -> nb::Result<Option<ControllerMessage>, serial::Error> {
        let msg_csum = u16::from_le_bytes([serial.read()?, serial.read()?]);

        let mut msg_buf = [0; core::mem::size_of::<ControllerMessage>()];
        let mut buf_offset = 0;

        while serial.is_rxne() {
            if buf_offset < core::mem::size_of::<ControllerMessage>() {
                msg_buf[buf_offset] = serial.read()?;
                buf_offset += 1;
            } else {
                break;
            }
        }

        let csum = crc::crc16::checksum_x25(&msg_buf);

        if msg_csum != csum {
            return Ok(None);
        }

        let msg: *const ControllerMessage = unsafe { core::mem::transmute(msg_buf.as_ptr()) };
        let msg = unsafe { *msg };

        Ok(Some(msg))
    }

    pub fn write<PINS>(
        &mut self,
        serial: &mut serial::Serial<pac::USART1, PINS>,
        msg: &ControllerMessage,
    ) -> nb::Result<(), serial::Error> {
        let msg_buf = unsafe {
            core::slice::from_raw_parts(
                (msg as *const ControllerMessage) as *const u8,
                core::mem::size_of::<ControllerMessage>(),
            )
        };

        let csum = crc::crc16::checksum_x25(msg_buf);
        let csum = csum.to_le_bytes();

        serial.write(csum[0])?;
        serial.write(csum[1])?;

        let mut buf_offset = 0;

        while buf_offset < msg_buf.len() {
            serial.write(msg_buf[buf_offset])?;
            buf_offset += 1;
        }

        Ok(())
    }

    // fn decode() {}

    // fn encode() -> [u8] {}
}
