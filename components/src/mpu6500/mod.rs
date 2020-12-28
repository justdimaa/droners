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

pub const I2C_ADDR_AL: u8 = 0x68;
pub const I2C_ADDR_AH: u8 = 0x69;

pub const DEV_ID_MPU6500: u8 = 0x70;
pub const DEV_ID_MPU9250: u8 = 0x71;
pub const DEV_ID_MPU9255: u8 = 0x73;

// const CALIB_GYRO_SENSITIVITY: u16 = 131; // LSB/deg/s
const CALIB_ACCEL_SENSITIVITY: u16 = 16384; // LSB/g

const DEG_TO_RAD: f32 = 0.01745329252;

pub struct Mpu6500<I2C>
where
    I2C: i2c::Read + i2c::Write,
{
    addr: u8,
    cfg: Config,
    pub(crate) acceleration: Vector3<f32>,
    pub(crate) angular_velocity: Vector3<f32>,
    pub(crate) temperature: f32,
    accel_bias: Vector3<f32>,
    gyro_bias: Vector3<f32>,
    accel_resolution: f32,
    gyro_resolution: f32,
    _i2c: PhantomData<I2C>,
}

impl<I2C, I2cError> Mpu6500<I2C>
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

        if dev_id != DEV_ID_MPU6500 && dev_id != DEV_ID_MPU9250 && dev_id != DEV_ID_MPU9255 {
            return Err(Error::InvalidDevice(dev_id));
        }

        let mut mpu = Self {
            addr,
            acceleration: Vector3::default(),
            angular_velocity: Vector3::default(),
            temperature: 0.0,
            accel_bias: Vector3::default(),
            gyro_bias: Vector3::default(),
            accel_resolution: cfg.accel_fs_sel.get_resolution(),
            gyro_resolution: cfg.gyro_fs_sel.get_resolution(),
            cfg,
            _i2c: PhantomData::default(),
        };

        mpu.init(i2c, delay)?;

        Ok(mpu)
    }

    fn init<DELAY>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<(), I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        Self::pwr_mgmt_1(self.addr, i2c, false, false, false, false, false, 0x00)?;
        delay.delay_ms(100);

        Self::pwr_mgmt_1(self.addr, i2c, false, false, false, false, false, 0x01)?;
        delay.delay_ms(200);

        Self::config(self.addr, i2c, false, self.cfg.gyro_dlpf_cfg)?;
        Self::sample_rate_div(self.addr, i2c, self.cfg.fifo_sample_rate)?;

        Self::gyro_cfg(self.addr, i2c, self.cfg.gyro_fs_sel, self.cfg.gyro_fchoice)?;

        Self::accel_cfg(self.addr, i2c, self.cfg.accel_fs_sel)?;
        Self::accel_cfg_2(
            self.addr,
            i2c,
            (self.cfg.accel_fchoice & 0x01) == 1,
            self.cfg.accel_dlpf_cfg,
        )?;
        Self::int_pin_bypass_enable_interrupt_cfg(
            self.addr, i2c, false, false, false, false, false, false, true,
        )?;
        Self::enable_interrupt(self.addr, i2c, false, false, false, true)?;
        delay.delay_ms(100);

        Ok(())
    }

    pub(crate) fn read_imu(&mut self, i2c: &mut I2C) -> Result<(), I2cError> {
        let mut buf = [0; 14];
        Self::read_register(self.addr, i2c, Register::ACCEL_XOUT_H, &mut buf)?;

        self.acceleration = Vector3::new(
            i16::from_be_bytes([buf[0], buf[1]]) as f32 * self.accel_resolution,
            i16::from_be_bytes([buf[2], buf[3]]) as f32 * self.accel_resolution,
            i16::from_be_bytes([buf[4], buf[5]]) as f32 * self.accel_resolution,
        );

        self.temperature = i16::from_be_bytes([buf[6], buf[7]]) as f32 / 333.87 + 21.0;

        self.angular_velocity = Vector3::new(
            i16::from_be_bytes([buf[8], buf[9]]) as f32 * self.gyro_resolution * DEG_TO_RAD,
            i16::from_be_bytes([buf[10], buf[11]]) as f32 * self.gyro_resolution * DEG_TO_RAD,
            i16::from_be_bytes([buf[12], buf[13]]) as f32 * self.gyro_resolution * DEG_TO_RAD,
        );

        Ok(())
    }

    pub fn read(&mut self, i2c: &mut I2C) -> Result<(), I2cError> {
        self.read_imu(i2c)?;

        Ok(())
    }

    pub fn calibrate<DELAY>(&mut self, i2c: &mut I2C, delay: &mut DELAY) -> Result<(), I2cError>
    where
        DELAY: delay::DelayMs<u16>,
    {
        Self::pwr_mgmt_1(self.addr, i2c, false, false, false, false, false, 0x01)?;
        Self::pwr_mgmt_2(self.addr, i2c, false, false, false, false, false, false)?;
        delay.delay_ms(200);

        Self::enable_interrupt(self.addr, i2c, false, false, false, false)?;
        Self::fifo_enable(
            self.addr, i2c, false, false, false, false, false, false, false, false,
        )?;
        Self::pwr_mgmt_1(self.addr, i2c, false, false, false, false, false, 0x00)?;
        Self::i2c_mst_ctrl(self.addr, i2c, false, false, false, false, 0x00)?;
        Self::user_ctrl(self.addr, i2c, false, false, false, false, false, false)?;
        Self::user_ctrl(self.addr, i2c, false, false, true, true, false, false)?;
        delay.delay_ms(15);

        Self::config(self.addr, i2c, false, config::GyroDlpfCfg::Dlpf184Hz)?;
        Self::sample_rate_div(self.addr, i2c, config::FifoSampleRate::Smpl1000Hz)?;
        Self::gyro_cfg(self.addr, i2c, config::GyroFullScaleSelect::Dps250, 0x00)?;
        Self::accel_cfg(self.addr, i2c, config::AccelFullScaleSelect::G2)?;

        Self::user_ctrl(self.addr, i2c, true, false, false, false, false, false)?;
        Self::fifo_enable(
            self.addr, i2c, false, true, true, true, true, false, false, false,
        )?;
        delay.delay_ms(40);

        Self::fifo_enable(
            self.addr, i2c, false, false, false, false, false, false, false, false,
        )?;

        let fifo_count = Self::fifo_count_h(self.addr, i2c)?;
        let packet_count = fifo_count / 12;

        for _ in 0..packet_count {
            let (accel, gyro) = Self::fifo_read(self.addr, i2c)?;

            self.accel_bias.x += accel.x as f32;
            self.accel_bias.y += accel.y as f32;
            self.accel_bias.z += accel.z as f32;

            self.gyro_bias.x += gyro.x as f32;
            self.gyro_bias.y += gyro.y as f32;
            self.gyro_bias.z += gyro.z as f32;
        }

        if packet_count != 0 {
            self.accel_bias /= packet_count as f32;
            self.gyro_bias /= packet_count as f32;

            if self.accel_bias.z > 0.0 {
                self.accel_bias.z -= CALIB_ACCEL_SENSITIVITY as f32;
            } else {
                self.accel_bias.z += CALIB_ACCEL_SENSITIVITY as f32;
            }
        }

        let mut accel_offset = Self::read_accel_offset(self.addr, i2c)?;
        let mut mask_bit = Vector3::new(1, 1, 1);

        // x

        if accel_offset.x % 2 != 0 {
            mask_bit.x = 0;
        }

        accel_offset.x -= (self.accel_bias.x as i16) >> 3;

        if mask_bit.x != 0 {
            accel_offset.x &= !mask_bit.x;
        } else {
            accel_offset.x |= 0b1;
        }

        // y

        if accel_offset.y % 2 != 0 {
            mask_bit.y = 0;
        }

        accel_offset.y -= (self.accel_bias.y as i16) >> 3;

        if mask_bit.y != 0 {
            accel_offset.y &= !mask_bit.y;
        } else {
            accel_offset.y |= 0b1;
        }

        // z

        if accel_offset.z % 2 != 0 {
            mask_bit.z = 0;
        }

        accel_offset.z -= (self.accel_bias.z as i16) >> 3;

        if mask_bit.z != 0 {
            accel_offset.z &= !mask_bit.z;
        } else {
            accel_offset.z |= 0b1;
        }

        let gyro_offset = Vector3::new(
            -self.gyro_bias.x as i16 / 4,
            -self.gyro_bias.y as i16 / 4,
            -self.gyro_bias.z as i16 / 4,
        );

        Self::write_accel_offset(self.addr, i2c, accel_offset)?;
        Self::write_gyro_offset(self.addr, i2c, gyro_offset)?;

        delay.delay_ms(100);

        self.init(i2c, delay)?;

        Ok(())
    }

    pub fn get_acceleration(&self) -> Vector3<f32> {
        self.acceleration
    }

    pub fn get_angular_velocity(&self) -> Vector3<f32> {
        self.angular_velocity
    }

    /// Registers 19 to 24 – Gyro Offset Registers
    fn write_gyro_offset(
        addr: u8,
        i2c: &mut I2C,
        gyro_offset: Vector3<i16>,
    ) -> Result<(), I2cError> {
        let x = gyro_offset.x.to_be_bytes();
        Self::write_register(addr, i2c, Register::XG_OFFSET_H, x[0])?;
        Self::write_register(addr, i2c, Register::XG_OFFSET_L, x[1])?;

        let y = gyro_offset.y.to_be_bytes();
        Self::write_register(addr, i2c, Register::YG_OFFSET_H, y[0])?;
        Self::write_register(addr, i2c, Register::YG_OFFSET_L, y[1])?;

        let z = gyro_offset.z.to_be_bytes();
        Self::write_register(addr, i2c, Register::ZG_OFFSET_H, z[0])?;
        Self::write_register(addr, i2c, Register::ZG_OFFSET_L, z[1])?;

        Ok(())
    }

    /// Register 25 – Sample Rate Divider
    fn sample_rate_div(
        addr: u8,
        i2c: &mut I2C,
        smplrt_div: config::FifoSampleRate,
    ) -> Result<(), I2cError> {
        Self::write_register(addr, i2c, Register::SMPLRT_DIV, smplrt_div as u8)
    }

    /// Register 26 – Configuration
    fn config(
        addr: u8,
        i2c: &mut I2C,
        fifo_mode: bool,
        dlpf_cfg: config::GyroDlpfCfg,
    ) -> Result<(), I2cError> {
        Self::write_register(
            addr,
            i2c,
            Register::CONFIG,
            (fifo_mode as u8) << 6 | (dlpf_cfg as u8) << 0,
        )
    }

    /// Register 27 – Gyroscope Configuration
    fn gyro_cfg(
        addr: u8,
        i2c: &mut I2C,
        gyro_fs_sel: config::GyroFullScaleSelect,
        fchoice_b: u8,
    ) -> Result<(), I2cError> {
        let mut c = [0; 1];
        Self::read_register(addr, i2c, Register::GYRO_CONFIG, &mut c)?;

        Self::write_register(
            addr,
            i2c,
            Register::GYRO_CONFIG,
            c[0] & 0b11100111 | (gyro_fs_sel as u8) << 3 | (fchoice_b & 0b11) << 0,
        )
    }

    /// Register 28 – Accelerometer Configuration
    fn accel_cfg(
        addr: u8,
        i2c: &mut I2C,
        accel_fs_sel: config::AccelFullScaleSelect,
    ) -> Result<(), I2cError> {
        let mut c = [0; 1];
        Self::read_register(addr, i2c, Register::ACCEL_CONFIG_2, &mut c)?;

        Self::write_register(
            addr,
            i2c,
            Register::ACCEL_CONFIG,
            c[0] & 0b11100111 | (accel_fs_sel as u8) << 3,
        )
    }

    /// Register 29 – Accelerometer Configuration 2
    fn accel_cfg_2(
        addr: u8,
        i2c: &mut I2C,
        accel_fchoice_b: bool,
        a_dlpf_cfg: config::AccelDlpfCfg,
    ) -> Result<(), I2cError> {
        let mut c = [0; 1];
        Self::read_register(addr, i2c, Register::ACCEL_CONFIG_2, &mut c)?;

        Self::write_register(
            addr,
            i2c,
            Register::ACCEL_CONFIG_2,
            c[0] & 0b11110000 | (accel_fchoice_b as u8) << 3 | (a_dlpf_cfg as u8) << 0,
        )
    }

    /// Register 35 – FIFO Enable
    fn fifo_enable(
        addr: u8,
        i2c: &mut I2C,
        temp_out: bool,
        gyro_xout: bool,
        gyro_yout: bool,
        gyro_zout: bool,
        accel: bool,
        slv_2: bool,
        slv_1: bool,
        slv_0: bool,
    ) -> Result<(), I2cError> {
        Self::write_register(
            addr,
            i2c,
            Register::FIFO_EN,
            (temp_out as u8) << 7
                | (gyro_xout as u8) << 6
                | (gyro_yout as u8) << 5
                | (gyro_zout as u8) << 4
                | (accel as u8) << 3
                | (slv_2 as u8) << 2
                | (slv_1 as u8) << 1
                | (slv_0 as u8) << 0,
        )
    }

    /// Register 36 – I2C Master Control
    fn i2c_mst_ctrl(
        addr: u8,
        i2c: &mut I2C,
        mult_mst_en: bool,
        wait_for_es: bool,
        slv_3_fifo_en: bool,
        i2c_mst_p_nsr: bool,
        i2c_mst_clk: u8,
    ) -> Result<(), I2cError> {
        Self::write_register(
            addr,
            i2c,
            Register::I2C_MST_CTRL,
            (mult_mst_en as u8) << 7
                | (wait_for_es as u8) << 6
                | (slv_3_fifo_en as u8) << 5
                | (i2c_mst_p_nsr as u8) << 4
                | (i2c_mst_clk & 0b1111) << 0,
        )
    }

    /// Register 55 – INT Pin / Bypass Enable Configuration
    fn int_pin_bypass_enable_interrupt_cfg(
        addr: u8,
        i2c: &mut I2C,
        actl: bool,
        open: bool,
        latch_int_en: bool,
        int_anyrd_2clear: bool,
        actl_fsync: bool,
        fsync_int_mode_en: bool,
        bypass_en: bool,
    ) -> Result<(), I2cError> {
        Self::write_register(
            addr,
            i2c,
            Register::INT_PIN_CFG,
            (actl as u8) << 7
                | (open as u8) << 6
                | (latch_int_en as u8) << 5
                | (int_anyrd_2clear as u8) << 4
                | (actl_fsync as u8) << 3
                | (fsync_int_mode_en as u8) << 2
                | (bypass_en as u8) << 1,
        )
    }

    /// Register 56 – Interrupt Enable
    fn enable_interrupt(
        addr: u8,
        i2c: &mut I2C,
        wom_en: bool,
        fifo_overflow_en: bool,
        fsync_int_en: bool,
        raw_rdy_en: bool,
    ) -> Result<(), I2cError> {
        Self::write_register(
            addr,
            i2c,
            Register::INT_ENABLE,
            (wom_en as u8) << 6
                | (fifo_overflow_en as u8) << 4
                | (fsync_int_en as u8) << 3
                | (raw_rdy_en as u8) << 0,
        )
    }

    /// Register 106 – User Control
    fn user_ctrl(
        addr: u8,
        i2c: &mut I2C,
        fifo_en: bool,
        i2c_mst_en: bool,
        i2c_if_dis: bool,
        fifo_rst: bool,
        i2c_mst_rst: bool,
        sig_cond_rst: bool,
    ) -> Result<(), I2cError> {
        Self::write_register(
            addr,
            i2c,
            Register::USER_CTRL,
            (fifo_en as u8) << 6
                | (i2c_mst_en as u8) << 5
                | (i2c_if_dis as u8) << 4
                | (fifo_rst as u8) << 2
                | (i2c_mst_rst as u8) << 1
                | (sig_cond_rst as u8) << 0,
        )
    }

    /// Register 107 – Power Management 1
    fn pwr_mgmt_1(
        addr: u8,
        i2c: &mut I2C,
        h_reset: bool,
        sleep: bool,
        cycle: bool,
        gyro_standby: bool,
        pd_ptat: bool,
        clksel: u8,
    ) -> Result<(), I2cError> {
        Self::write_register(
            addr,
            i2c,
            Register::PWR_MGMT_1,
            (h_reset as u8) << 7
                | (sleep as u8) << 6
                | (cycle as u8) << 5
                | (gyro_standby as u8) << 4
                | (pd_ptat as u8) << 3
                | (clksel & 0b111) << 0,
        )
    }

    // Register 108 – Power Management 2
    fn pwr_mgmt_2(
        addr: u8,
        i2c: &mut I2C,
        disable_xa: bool,
        disable_ya: bool,
        disable_za: bool,
        disable_xg: bool,
        disable_yg: bool,
        disable_zg: bool,
    ) -> Result<(), I2cError> {
        Self::write_register(
            addr,
            i2c,
            Register::PWR_MGMT_2,
            (disable_xa as u8) << 5
                | (disable_ya as u8) << 4
                | (disable_za as u8) << 3
                | (disable_xg as u8) << 2
                | (disable_yg as u8) << 1
                | (disable_zg as u8) << 0,
        )
    }

    /// Register 114 – FIFO Count High
    fn fifo_count_h(addr: u8, i2c: &mut I2C) -> Result<u16, I2cError> {
        let mut buf = [0; 2];
        Self::read_register(addr, i2c, Register::FIFO_COUNTH, &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Register 116 – FIFO Read Write
    fn fifo_read(addr: u8, i2c: &mut I2C) -> Result<(Vector3<i16>, Vector3<i16>), I2cError> {
        let mut buf = [0; 12];
        Self::read_register(addr, i2c, Register::FIFO_R_W, &mut buf)?;
        Ok((
            Vector3::new(
                i16::from_be_bytes([buf[0], buf[1]]),
                i16::from_be_bytes([buf[2], buf[3]]),
                i16::from_be_bytes([buf[4], buf[5]]),
            ),
            Vector3::new(
                i16::from_be_bytes([buf[6], buf[7]]),
                i16::from_be_bytes([buf[8], buf[9]]),
                i16::from_be_bytes([buf[10], buf[11]]),
            ),
        ))
    }

    /// Register 117 – Who Am I
    fn who_am_i(addr: u8, i2c: &mut I2C) -> Result<u8, I2cError> {
        let mut buf = [0; 1];
        Self::read_register(addr, i2c, Register::WHO_AM_I, &mut buf)?;
        Ok(buf[0])
    }

    /// Registers 119, 120, 122, 123, 125, 126 – Accelerometer Offset Registers
    fn read_accel_offset(addr: u8, i2c: &mut I2C) -> Result<Vector3<i16>, I2cError> {
        let mut buf = [0; 2];
        Self::read_register(addr, i2c, Register::XA_OFFSET_H, &mut buf)?;
        let x = i16::from_be_bytes(buf);

        let mut buf = [0; 2];
        Self::read_register(addr, i2c, Register::YA_OFFSET_H, &mut buf)?;
        let y = i16::from_be_bytes(buf);

        let mut buf = [0; 2];
        Self::read_register(addr, i2c, Register::ZA_OFFSET_H, &mut buf)?;
        let z = i16::from_be_bytes(buf);

        Ok(Vector3::new(x, y, z))
    }

    /// Registers 119, 120, 122, 123, 125, 126 – Accelerometer Offset Registers
    fn write_accel_offset(
        addr: u8,
        i2c: &mut I2C,
        accel_offset: Vector3<i16>,
    ) -> Result<(), I2cError> {
        let x = accel_offset.x.to_be_bytes();
        Self::write_register(addr, i2c, Register::XA_OFFSET_H, x[0])?;
        Self::write_register(addr, i2c, Register::XA_OFFSET_L, x[1])?;

        let y = accel_offset.y.to_be_bytes();
        Self::write_register(addr, i2c, Register::YA_OFFSET_H, y[0])?;
        Self::write_register(addr, i2c, Register::YA_OFFSET_L, y[1])?;

        let z = accel_offset.z.to_be_bytes();
        Self::write_register(addr, i2c, Register::ZA_OFFSET_H, z[0])?;
        Self::write_register(addr, i2c, Register::ZA_OFFSET_L, z[1])?;

        Ok(())
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
