#[derive(Debug, Clone)]
pub struct Config {
    pub(crate) accel_fs_sel: AccelFullScaleSelect,
    pub(crate) gyro_fs_sel: GyroFullScaleSelect,
    pub(crate) fifo_sample_rate: FifoSampleRate,
    pub(crate) gyro_fchoice: u8,
    pub(crate) gyro_dlpf_cfg: GyroDlpfCfg,
    pub(crate) accel_fchoice: u8,
    pub(crate) accel_dlpf_cfg: AccelDlpfCfg,
}

impl Config {
    pub fn accel_fs_sel(mut self, accel_fs_sel: AccelFullScaleSelect) -> Self {
        self.accel_fs_sel = accel_fs_sel;
        self
    }

    pub fn gyro_fs_sel(mut self, gyro_fs_sel: GyroFullScaleSelect) -> Self {
        self.gyro_fs_sel = gyro_fs_sel;
        self
    }

    pub fn fifo_sample_rate(mut self, fifo_sample_rate: FifoSampleRate) -> Self {
        self.fifo_sample_rate = fifo_sample_rate;
        self
    }

    pub fn gyro_fchoice(mut self, gyro_fchoice: u8) -> Self {
        self.gyro_fchoice = gyro_fchoice;
        self
    }

    pub fn gyro_dlpf_cfg(mut self, gyro_dlpf_cfg: GyroDlpfCfg) -> Self {
        self.gyro_dlpf_cfg = gyro_dlpf_cfg;
        self
    }

    pub fn accel_fchoice(mut self, accel_fchoice: u8) -> Self {
        self.accel_fchoice = accel_fchoice;
        self
    }

    pub fn accel_dlpf_cfg(mut self, accel_dlpf_cfg: AccelDlpfCfg) -> Self {
        self.accel_dlpf_cfg = accel_dlpf_cfg;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            accel_fs_sel: AccelFullScaleSelect::G16,
            gyro_fs_sel: GyroFullScaleSelect::Dps2000,
            fifo_sample_rate: FifoSampleRate::Smpl200Hz,
            gyro_fchoice: 0x03,
            gyro_dlpf_cfg: GyroDlpfCfg::Dlpf41Hz,
            accel_fchoice: 0x01,
            accel_dlpf_cfg: AccelDlpfCfg::Dlpf45Hz,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum AccelFullScaleSelect {
    G2,
    G4,
    G8,
    G16,
}

impl AccelFullScaleSelect {
    pub fn get_resolution(&self) -> f32 {
        match *self {
            AccelFullScaleSelect::G2 => 2.0 / 32768.0,
            AccelFullScaleSelect::G4 => 4.0 / 32768.0,
            AccelFullScaleSelect::G8 => 8.0 / 32768.0,
            AccelFullScaleSelect::G16 => 16.0 / 32768.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum GyroFullScaleSelect {
    Dps250 = 0b00,
    Dps500 = 0b01,
    Dps1000 = 0b10,
    Dps2000 = 0b11,
}

impl GyroFullScaleSelect {
    pub fn get_resolution(&self) -> f32 {
        match *self {
            GyroFullScaleSelect::Dps250 => 250.0 / 32768.0,
            GyroFullScaleSelect::Dps500 => 500.0 / 32768.0,
            GyroFullScaleSelect::Dps1000 => 1000.0 / 32768.0,
            GyroFullScaleSelect::Dps2000 => 2000.0 / 32768.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum FifoSampleRate {
    Smpl1000Hz = 0x00,
    Smpl500Hz = 0x01,
    Smpl333Hz = 0x02,
    Smpl250Hz = 0x03,
    Smpl200Hz = 0x04,
    Smpl167Hz = 0x05,
    Smpl143Hz = 0x06,
    Smpl125Hz = 0x07,
}

impl FifoSampleRate {
    pub fn get_freq(&self) -> u32 {
        match *self {
            FifoSampleRate::Smpl1000Hz => 1000,
            FifoSampleRate::Smpl500Hz => 500,
            FifoSampleRate::Smpl333Hz => 333,
            FifoSampleRate::Smpl250Hz => 250,
            FifoSampleRate::Smpl200Hz => 200,
            FifoSampleRate::Smpl167Hz => 167,
            FifoSampleRate::Smpl143Hz => 143,
            FifoSampleRate::Smpl125Hz => 125,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum GyroDlpfCfg {
    Dlpf250Hz = 0x00,
    Dlpf184Hz = 0x01,
    Dlpf92Hz = 0x02,
    Dlpf41Hz = 0x03,
    Dlpf20Hz = 0x04,
    Dlpf10Hz = 0x05,
    Dlpf5Hz = 0x06,
    Dlpf3600Hz = 0x07,
}

#[derive(Debug, Clone, Copy)]
pub enum AccelDlpfCfg {
    Dlpf218Hz0 = 0x00,
    Dlpf218Hz1 = 0x01,
    Dlpf99Hz = 0x02,
    Dlpf45Hz = 0x03,
    Dlpf21Hz = 0x04,
    Dlpf10Hz = 0x05,
    Dlpf5Hz = 0x06,
    Dlpf420Hz = 0x07,
}
