use core::marker::PhantomData;

use embedded_hal::prelude::_embedded_hal_serial_Read;
use stm32f4xx_hal::{pac, serial};

macro_rules! hal {
    {$($USARTX:ty), *} => ($(

        impl Neo6m<$USARTX> {
            pub fn new() -> Self {
                Self {
                    nmea_parser: nmea0183::Parser::new(),
                    _usart: PhantomData::default(),
                }
            }

            pub fn read<PINS>(&mut self, serial: &mut serial::Serial<$USARTX, PINS>) -> Option<nmea0183::ParseResult> {
                while serial.is_rxne() {
                    match serial.read() {
                        Ok(b) => {
                            return match self.nmea_parser.parse_from_byte(b) {
                                Some(v) => match v {
                                    Ok(v) => Some(v),
                                    Err(_) => None,
                                },
                                None => None,
                            }
                        }
                        Err(_) => {}
                    }
                }

                None
            }
        }
    )*)
}

pub struct Neo6m<USART> {
    nmea_parser: nmea0183::Parser,
    _usart: PhantomData<USART>,
}

hal!(pac::USART1, pac::USART2, pac::USART6);
