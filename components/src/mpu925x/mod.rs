use core::marker::PhantomData;

use config::Config;
use embedded_hal::blocking::{delay, i2c};
use nalgebra::Quaternion;

use self::result::{Error, Result};
use crate::{ak8963, mpu6500};

pub type Madgwick = ahrs::Madgwick<f32>;
pub type Mahony = ahrs::Mahony<f32>;

pub mod config;
pub mod result;

const MADGWICK_BETA: f32 = 0.60459978807; // sqrt(3.0 / 4.0) * PI * (40.0 / 180.0)

const MAHONY_KP: f32 = 10.0;
const MAHONY_KI: f32 = 0.0;

pub struct Mpu925x<I2C, AHRS>
where
    I2C: i2c::Read + i2c::Write,
    AHRS: ahrs::Ahrs<f32>,
{
    ak: ak8963::Ak8963<I2C>,
    mpu: mpu6500::Mpu6500<I2C>,
    ahrs: AHRS,
    _i2c: PhantomData<I2C>,
}

impl<I2C, AHRS, I2cError> Mpu925x<I2C, AHRS>
where
    I2C: i2c::Read<Error = I2cError> + i2c::Write<Error = I2cError>,
    AHRS: ahrs::Ahrs<f32>,
{
    fn with_configuration_ahrs<DELAY>(
        ak_addr: u8,
        mpu_addr: u8,
        i2c: &mut I2C,
        delay: &mut DELAY,
        cfg: Config,
        ahrs: AHRS,
    ) -> Result<Self, I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        let mpu = match mpu6500::Mpu6500::with_configuration(mpu_addr, i2c, delay, cfg.mpu) {
            Ok(v) => v,
            Err(e) => return Err(Error::Mpu6500Error(e)),
        };

        let ak = match ak8963::Ak8963::with_configuration(ak_addr, i2c, delay, cfg.ak) {
            Ok(v) => v,
            Err(e) => return Err(Error::Ak8963Error(e)),
        };

        Ok(Self {
            ak,
            mpu,
            ahrs,
            _i2c: PhantomData::default(),
        })
    }

    pub fn read(&mut self, i2c: &mut I2C) -> Result<(), I2cError> {
        if let Err(e) = self.mpu.read_imu(i2c) {
            return Err(Error::Mpu6500Error(e));
        }

        if let Err(e) = self.ak.read(i2c) {
            return Err(Error::Ak8963Error(e));
        }

        match self.ahrs.update(
            &self.mpu.angular_velocity,
            &self.mpu.acceleration,
            &self.ak.magnetic_field,
        ) {
            Ok(_) => {}
            Err(e) => {
                return match e {
                    "Accelerometer norm divided by zero." => Err(Error::AhrsUpdateAccelerometer),
                    "Magnetometer norm divided by zero." => Err(Error::AhrsUpdateMagnetometer),
                    _ => unreachable!(),
                }
            }
        }

        Ok(())
    }
}

impl<I2C, I2cError> Mpu925x<I2C, Madgwick>
where
    I2C: i2c::Read<Error = I2cError> + i2c::Write<Error = I2cError>,
{
    pub fn new<DELAY>(
        ak_addr: u8,
        mpu_addr: u8,
        i2c: &mut I2C,
        delay: &mut DELAY,
    ) -> Result<Self, I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        Self::with_configuration(ak_addr, mpu_addr, i2c, delay, config::Config::default())
    }

    pub fn with_configuration<DELAY>(
        ak_addr: u8,
        mpu_addr: u8,
        i2c: &mut I2C,
        delay: &mut DELAY,
        cfg: Config,
    ) -> Result<Self, I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        let sample_rate_freq = cfg.mpu.fifo_sample_rate.get_freq();
        let sample_period = sample_rate_freq as f32 / 1000.0;

        Self::with_configuration_ahrs(
            ak_addr,
            mpu_addr,
            i2c,
            delay,
            cfg,
            ahrs::Madgwick::new(sample_period, MADGWICK_BETA),
        )
    }

    pub fn calibrate_mpu<DELAY>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<(), I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        match self.mpu.calibrate(i2c, delay) {
            Ok(()) => Ok(()),
            Err(e) => Err(Error::Mpu6500Error(e)),
        }
    }

    pub fn calibrate_ak<DELAY>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<(), I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        match self.ak.calibrate(i2c, delay) {
            Ok(()) => Ok(()),
            Err(e) => Err(Error::Ak8963Error(e)),
        }
    }

    pub fn rotation(&self) -> Quaternion<f32> {
        self.ahrs.quat
    }
}

impl<I2C, I2cError> Mpu925x<I2C, Mahony>
where
    I2C: i2c::Read<Error = I2cError> + i2c::Write<Error = I2cError>,
{
    pub fn new<DELAY>(
        ak_addr: u8,
        mpu_addr: u8,
        i2c: &mut I2C,
        delay: &mut DELAY,
    ) -> Result<Self, I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        Self::with_configuration(ak_addr, mpu_addr, i2c, delay, config::Config::default())
    }

    pub fn with_configuration<DELAY>(
        ak_addr: u8,
        mpu_addr: u8,
        i2c: &mut I2C,
        delay: &mut DELAY,
        cfg: Config,
    ) -> Result<Self, I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        let sample_rate_freq = cfg.mpu.fifo_sample_rate.get_freq();
        let sample_period = sample_rate_freq as f32 / 1000.0;

        Self::with_configuration_ahrs(
            ak_addr,
            mpu_addr,
            i2c,
            delay,
            cfg,
            ahrs::Mahony::new(sample_period, MAHONY_KP, MAHONY_KI),
        )
    }

    pub fn rotation(&self) -> Quaternion<f32> {
        self.ahrs.quat
    }
}
