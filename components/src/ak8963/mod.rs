use core::marker::PhantomData;

use embedded_hal::blocking::{delay, i2c};
use nalgebra::Vector3;

use self::{
    config::Config,
    register::Register,
    result::{Error, Result},
};

pub mod config;
mod register;
pub mod result;

pub const I2C_ADDR: u8 = 0x0C;
pub const DEV_ID: u8 = 0x48;

pub struct Ak8963<I2C>
where
    I2C: i2c::Read + i2c::Write,
{
    addr: u8,
    cfg: Config,
    pub(crate) magnetic_field: Vector3<f32>,
    mag_resolution: f32,
    mag_bias_factory: Vector3<f32>,
    mag_bias: Vector3<f32>,
    mag_scale: Vector3<f32>,
    _i2c: PhantomData<I2C>,
}

impl<I2C, I2cError> Ak8963<I2C>
where
    I2C: i2c::Read<Error = I2cError> + i2c::Write<Error = I2cError>,
{
    pub fn new<DELAY>(addr: u8, delay: &mut DELAY, i2c: &mut I2C) -> Result<Self, I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        Self::with_configuration(addr, i2c, delay, Config::default())
    }

    pub fn with_configuration<DELAY>(
        addr: u8,
        i2c: &mut I2C,
        delay: &mut DELAY,
        cfg: Config,
    ) -> Result<Self, I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        let dev_id = Self::who_am_i(addr, i2c)?;

        if dev_id != DEV_ID {
            return Err(Error::InvalidDevice(dev_id));
        }

        let mut ak = Self {
            addr,
            magnetic_field: Vector3::default(),
            mag_scale: Vector3::new(1.0, 1.0, 1.0),
            mag_bias: Vector3::default(),
            mag_bias_factory: Vector3::default(),
            mag_resolution: cfg.output_bits.get_resolution(),
            cfg,
            _i2c: PhantomData::default(),
        };

        ak.init(i2c, delay)?;

        Ok(ak)
    }

    fn init<DELAY>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<(), I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        Self::write_register(self.addr, i2c, Register::CNTL, 0x00)?; // shutdown
        delay.delay_ms(10);

        Self::write_register(self.addr, i2c, Register::CNTL, 0x0F)?; // enter fuse rom access mode
        delay.delay_ms(10);

        let mut raw = [0; 3];
        Self::read_register(self.addr, i2c, Register::ASAX, &mut raw)?; // read axis calibration values

        self.mag_bias_factory = Vector3::new(
            (raw[0] - 128) as f32 / 256.0 + 1.0,
            (raw[1] - 128) as f32 / 256.0 + 1.0,
            (raw[2] - 128) as f32 / 256.0 + 1.0,
        );

        Self::write_register(self.addr, i2c, Register::CNTL, 0x00)?; // shutdown
        delay.delay_ms(10);

        Self::write_register(
            self.addr,
            i2c,
            Register::CNTL,
            (self.cfg.output_bits as u8) << 4 | (self.cfg.measurement_mode as u8),
        )?; // set data resolution and sample odr
        delay.delay_ms(10);

        Ok(())
    }

    pub fn read(&mut self, i2c: &mut I2C) -> Result<(), I2cError> {
        let raw = Self::read_raw(self.addr, i2c)?;

        let bias_to_current_bias =
            self.mag_resolution / config::OutputBits::Bits16.get_resolution();

        self.magnetic_field = Vector3::new(
            (raw.x as f32 * self.mag_resolution * self.mag_bias_factory.x
                - self.mag_bias.x * bias_to_current_bias)
                * self.mag_scale.x,
            (raw.y as f32 * self.mag_resolution * self.mag_bias_factory.y
                - self.mag_bias.y * bias_to_current_bias)
                * self.mag_scale.y,
            (raw.z as f32 * self.mag_resolution * self.mag_bias_factory.z
                - self.mag_bias.z * bias_to_current_bias)
                * self.mag_scale.z,
        );

        Ok(())
    }

    pub fn calibrate<DELAY>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<(), I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        let output_bits = self.cfg.output_bits;

        self.cfg.output_bits = config::OutputBits::Bits16;
        self.init(i2c, delay)?;

        delay.delay_ms(4000);

        let sample_count = match self.cfg.measurement_mode {
            config::MeasurementMode::ContinuousMeasurement8Hz => 128,
            config::MeasurementMode::ContinuousMeasurement100Hz => 1500,
            _ => 0,
        };

        let mut max = Vector3::new(i16::MIN, i16::MIN, i16::MIN);
        let mut min = Vector3::new(i16::MAX, i16::MAX, i16::MAX);

        for _ in 0..sample_count {
            let raw = Self::read_raw(self.addr, i2c)?;

            max.x = i16::max(max.x, raw.x);
            min.x = i16::min(min.x, raw.x);

            max.y = i16::max(max.y, raw.y);
            min.y = i16::min(min.y, raw.y);

            max.z = i16::max(max.z, raw.z);
            min.z = i16::min(min.z, raw.z);

            match self.cfg.measurement_mode {
                config::MeasurementMode::ContinuousMeasurement8Hz => delay.delay_ms(135),
                config::MeasurementMode::ContinuousMeasurement100Hz => delay.delay_ms(12),
                _ => {}
            }
        }

        let bias = (max + min) / 2;
        let scale = (max - min) / 2;

        let bias_resolution = config::OutputBits::Bits16.get_resolution();
        self.mag_bias = Vector3::new(
            bias.x as f32 * bias_resolution * self.mag_bias_factory.x,
            bias.y as f32 * bias_resolution * self.mag_bias_factory.y,
            bias.z as f32 * bias_resolution * self.mag_bias_factory.z,
        );

        let avg_rad = (scale.x + scale.y + scale.z) as f32 / 3.0;
        self.mag_scale = Vector3::new(
            avg_rad / scale.x as f32,
            avg_rad / scale.y as f32,
            avg_rad / scale.z as f32,
        );

        self.cfg.output_bits = output_bits;
        self.init(i2c, delay)?;

        Ok(())
    }

    fn read_raw(addr: u8, i2c: &mut I2C) -> Result<Vector3<i16>, I2cError> {
        let mut buf = [0; 1];
        Self::read_register(addr, i2c, Register::ST1, &mut buf)?;

        if (buf[0] & 0x01) == 0 {
            return Err(Error::DataNotReady);
        }

        let mut buf = [0; 7];
        Self::read_register(addr, i2c, Register::HXL, &mut buf)?;

        if (buf[6] & 0x08) != 0 {
            return Err(Error::Overflow);
        }

        Ok(Vector3::new(
            i16::from_be_bytes([buf[1], buf[0]]),
            i16::from_be_bytes([buf[3], buf[2]]),
            i16::from_be_bytes([buf[5], buf[4]]),
        ))
    }

    fn who_am_i(addr: u8, i2c: &mut I2C) -> Result<u8, I2cError> {
        let mut buf = [0; 1];
        Self::read_register(addr, i2c, Register::WIA, &mut buf)?;
        Ok(buf[0])
    }

    fn read_register(
        addr: u8,
        i2c: &mut I2C,
        reg: Register,
        buf: &mut [u8],
    ) -> Result<(), I2cError> {
        match i2c.write(addr, &[reg as u8]) {
            Ok(()) => {}
            Err(e) => return Err(Error::I2cError(e)),
        }

        match i2c.read(addr, buf) {
            Ok(()) => Ok(()),
            Err(e) => Err(Error::I2cError(e)),
        }
    }

    fn write_register(addr: u8, i2c: &mut I2C, reg: Register, cmd: u8) -> Result<(), I2cError> {
        match i2c.write(addr, &[reg as u8, cmd]) {
            Ok(()) => Ok(()),
            Err(e) => Err(Error::I2cError(e)),
        }
    }
}
