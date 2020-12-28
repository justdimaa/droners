#[derive(Debug, Clone)]
pub struct Config {
    pub(crate) output_bits: OutputBits,
    pub(crate) measurement_mode: MeasurementMode,
}

impl Config {
    pub fn output_bits(mut self, output_bits: OutputBits) -> Self {
        self.output_bits = output_bits;
        self
    }

    pub fn measurement_mode(mut self, measurement_mode: MeasurementMode) -> Self {
        self.measurement_mode = measurement_mode;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            output_bits: OutputBits::Bits16,
            measurement_mode: MeasurementMode::ContinuousMeasurement100Hz,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum OutputBits {
    Bits14,
    Bits16,
}

impl OutputBits {
    pub fn get_resolution(&self) -> f32 {
        match *self {
            OutputBits::Bits14 => 10.0 * 4912.0 / 8190.0,
            OutputBits::Bits16 => 10.0 * 4912.0 / 32760.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum MeasurementMode {
    SingleMeasurement = 0b0001,
    ContinuousMeasurement8Hz = 0b0010,
    ContinuousMeasurement100Hz = 0b0110,
    ExternalTriggerMeasurement = 0b0100,
}
