use crate::{ak8963, mpu6500};

#[derive(Debug, Clone)]
pub struct Config {
    pub(crate) ak: ak8963::config::Config,
    pub(crate) mpu: mpu6500::config::Config,
}

impl Config {
    pub fn ak(mut self, ak: ak8963::config::Config) -> Self {
        self.ak = ak;
        self
    }

    pub fn mpu(mut self, mpu: mpu6500::config::Config) -> Self {
        self.mpu = mpu;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            ak: Default::default(),
            mpu: Default::default(),
        }
    }
}
