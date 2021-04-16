use core::convert::TryFrom;

use super::result::{Error, ParseConfigurationError, Result};

#[derive(Debug)]
pub struct ParameterSettings {
    save: bool,
    addr: u16,
    sped: Sped,
    chan: u8,
    option: TransmissionOption,
}

impl From<ParameterSettings> for [u8; 6] {
    fn from(cfg: ParameterSettings) -> Self {
        let save = match cfg.save {
            true => 0xC0,
            false => 0xC2,
        };
        let addr = cfg.addr.to_be_bytes();
        let sped: u8 = cfg.sped.into();
        let chan = cfg.chan;
        let option: u8 = cfg.option.into();

        [save, addr[0], addr[1], sped, chan, option]
    }
}

impl TryFrom<[u8; 6]> for ParameterSettings {
    type Error = Error;

    fn try_from(value: [u8; 6]) -> Result<Self> {
        let save = match value[0] {
            0xC0 => true,
            0xC2 => false,
            _ => {
                return Err(Error::ParseConfigurationError(
                    ParseConfigurationError::InvalidHead(value[0]),
                ))
            }
        };

        let addr = u16::from_be_bytes([value[1], value[2]]);
        let sped = Sped::try_from(value[3])?;
        let chan = value[4];
        let option = TransmissionOption::try_from(value[5])?;

        Ok(Self {
            save,
            addr,
            sped,
            chan,
            option,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Sped {
    pub uart_parity: UartParity,
    pub uart_baud_rate: UartBaudRate,
    pub air_data_rate: AirDataRate,
}

impl From<Sped> for u8 {
    fn from(sped: Sped) -> Self {
        let uart_parity = sped.uart_parity as u8;
        let uart_baud_rate = sped.uart_baud_rate as u8;
        let air_data_rate = sped.air_data_rate as u8;

        ((uart_parity & 0b10) << 7)
            | ((uart_parity & 0b01) << 6)
            | ((uart_baud_rate & 0b100) << 5)
            | ((uart_baud_rate & 0b010) << 4)
            | ((uart_baud_rate & 0b001) << 3)
            | ((air_data_rate & 0b100) << 2)
            | ((air_data_rate & 0b010) << 1)
            | ((air_data_rate & 0b001) << 0)
    }
}

impl TryFrom<u8> for Sped {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        let uart_parity = ((value & 0b10000000) >> 6) | ((value & 0b01000000) >> 6);
        let uart_baud_rate =
            ((value & 0b00100000) >> 3) | ((value & 0b00010000) >> 3) | ((value & 0b00001000) >> 3);
        let air_data_rate =
            ((value & 0b00000100) >> 0) | ((value & 0b00000010) >> 0) | ((value & 0b00000001) >> 0);

        let uart_parity = UartParity::try_from(uart_parity)?;
        let uart_baud_rate = UartBaudRate::try_from(uart_baud_rate)?;
        let air_data_rate = AirDataRate::try_from(air_data_rate)?;

        Ok(Self {
            uart_parity,
            uart_baud_rate,
            air_data_rate,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TransmissionOption {
    pub transmission: Transmission,
    pub io_drive_mode: IoDriveMode,
    pub wireless_wakeup_time: WirelessWakeUpTime,
    pub fec: bool,
    pub tx_power: TransmissionPower,
}

impl From<TransmissionOption> for u8 {
    fn from(option: TransmissionOption) -> Self {
        let transmission = option.transmission as u8;
        let io_drive_mode = option.io_drive_mode as u8;
        let wireless_wakeup_time = option.wireless_wakeup_time as u8;
        let fec = option.fec as u8;
        let tx_power = option.tx_power as u8;

        ((transmission & 0b1) << 7)
            | ((io_drive_mode & 0b1) << 6)
            | ((wireless_wakeup_time & 0b100) << 5)
            | ((wireless_wakeup_time & 0b010) << 4)
            | ((wireless_wakeup_time & 0b001) << 3)
            | ((fec & 0b1) << 2)
            | ((tx_power & 0b10) << 1)
            | ((tx_power & 0b01) << 0)
    }
}

impl TryFrom<u8> for TransmissionOption {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        let transmission = (value & 0b10000000) >> 7;
        let io_drive_mode = (value & 0b01000000) >> 6;
        let wireless_wakeup_time =
            ((value & 0b00100000) >> 3) | ((value & 0b00010000) >> 3) | ((value & 0b00001000) >> 3);
        let fec = (value & 0b00000010) >> 1;
        let tx_power = (value & 0b00000001) >> 0;

        let transmission = Transmission::try_from(transmission)?;
        let io_drive_mode = IoDriveMode::try_from(io_drive_mode)?;
        let wireless_wakeup_time = WirelessWakeUpTime::try_from(wireless_wakeup_time)?;
        let fec = fec != 0;
        let tx_power = TransmissionPower::try_from(tx_power)?;

        Ok(Self {
            transmission,
            io_drive_mode,
            wireless_wakeup_time,
            fec,
            tx_power,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub enum UartParity {
    N8_1 = 0b00,
    O8_1 = 0b01,
    E8_1 = 0b10,
    _N8_1 = 0b11,
}

impl Default for UartParity {
    fn default() -> Self {
        Self::N8_1
    }
}

impl TryFrom<u8> for UartParity {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0b00 => Ok(Self::N8_1),
            0b01 => Ok(Self::O8_1),
            0b10 => Ok(Self::E8_1),
            0b11 => Ok(Self::_N8_1),
            _ => Err(Error::ParseConfigurationError(
                ParseConfigurationError::InvalidUartParity(value),
            )),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum UartBaudRate {
    Bps1200 = 0b000,
    Bps2400 = 0b001,
    Bps4800 = 0b010,
    Bps9600 = 0b011,
    Bps19200 = 0b100,
    Bps38400 = 0b101,
    Bps57600 = 0b110,
    Bps115200 = 0b111,
}

impl Default for UartBaudRate {
    fn default() -> Self {
        Self::Bps9600
    }
}

impl TryFrom<u8> for UartBaudRate {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0b000 => Ok(Self::Bps1200),
            0b001 => Ok(Self::Bps2400),
            0b010 => Ok(Self::Bps4800),
            0b011 => Ok(Self::Bps9600),
            0b100 => Ok(Self::Bps19200),
            0b101 => Ok(Self::Bps38400),
            0b110 => Ok(Self::Bps57600),
            0b111 => Ok(Self::Bps115200),
            _ => Err(Error::ParseConfigurationError(
                ParseConfigurationError::InvalidUartBaudRate(value),
            )),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum AirDataRate {
    Kbps0_3 = 0b000,
    Kbps1_2 = 0b001,
    Kbps2_4 = 0b010,
    Kbps4_8 = 0b011,
    Kbps9_6 = 0b100,
    Kbps19_2 = 0b101,
    _Kbps19_2 = 0b110,
    Reserved = 0b111,
}

impl Default for AirDataRate {
    fn default() -> Self {
        Self::Kbps2_4
    }
}

impl TryFrom<u8> for AirDataRate {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0b000 => Ok(Self::Kbps0_3),
            0b001 => Ok(Self::Kbps1_2),
            0b010 => Ok(Self::Kbps2_4),
            0b011 => Ok(Self::Kbps4_8),
            0b100 => Ok(Self::Kbps9_6),
            0b101 => Ok(Self::Kbps19_2),
            0b110 => Ok(Self::_Kbps19_2),
            0b111 => Ok(Self::Reserved),
            _ => Err(Error::ParseConfigurationError(
                ParseConfigurationError::InvalidAirDataRate(value),
            )),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Transmission {
    Transparent = 0b0,
    Fixed = 0b1,
}

impl TryFrom<u8> for Transmission {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0b0 => Ok(Self::Transparent),
            0b1 => Ok(Self::Fixed),
            _ => Err(Error::ParseConfigurationError(
                ParseConfigurationError::InvalidTransmission(value),
            )),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum IoDriveMode {
    OpenCollector = 0b0,
    PullUp = 0b1,
}

impl TryFrom<u8> for IoDriveMode {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0b0 => Ok(Self::OpenCollector),
            0b1 => Ok(Self::PullUp),
            _ => Err(Error::ParseConfigurationError(
                ParseConfigurationError::InvalidIoDriveMode(value),
            )),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum WirelessWakeUpTime {
    Ms250 = 0b000,
    Ms500 = 0b001,
    Ms750 = 0b010,
    Ms1000 = 0b011,
    Ms1250 = 0b100,
    Ms1500 = 0b101,
    Ms1750 = 0b110,
    Ms2000 = 0b111,
}

impl Default for WirelessWakeUpTime {
    fn default() -> Self {
        Self::Ms250
    }
}

impl TryFrom<u8> for WirelessWakeUpTime {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0b000 => Ok(Self::Ms250),
            0b001 => Ok(Self::Ms500),
            0b010 => Ok(Self::Ms750),
            0b011 => Ok(Self::Ms1000),
            0b100 => Ok(Self::Ms1250),
            0b101 => Ok(Self::Ms1500),
            0b110 => Ok(Self::Ms1750),
            0b111 => Ok(Self::Ms2000),
            _ => Err(Error::ParseConfigurationError(
                ParseConfigurationError::InvalidWirelessWakeUpTime(value),
            )),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum TransmissionPower {
    Dbm30 = 0b00,
    Dbm27 = 0b01,
    Dbm24 = 0b10,
    Dbm21 = 0b11,
}

impl TryFrom<u8> for TransmissionPower {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0b00 => Ok(Self::Dbm30),
            0b01 => Ok(Self::Dbm27),
            0b10 => Ok(Self::Dbm24),
            0b11 => Ok(Self::Dbm21),
            _ => Err(Error::ParseConfigurationError(
                ParseConfigurationError::InvalidTxPower(value),
            )),
        }
    }
}

impl Default for TransmissionPower {
    fn default() -> Self {
        Self::Dbm30
    }
}
