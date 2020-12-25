use droners_components::e32;
use stm32f4xx_hal::{adc, gpio, pac, serial};

pub type Adc1 = adc::Adc<pac::ADC1>;

pub type Serial1 = serial::Serial<
    pac::USART1,
    (
        gpio::gpioa::PA9<gpio::Alternate<gpio::AF7>>,
        gpio::gpioa::PA10<gpio::Alternate<gpio::AF7>>,
    ),
>;

pub type LeftThumbStickX = gpio::gpioa::PA0<gpio::Analog>;
pub type LeftThumbStickY = gpio::gpioa::PA1<gpio::Analog>;

pub type RightThumbStickX = gpio::gpioa::PA2<gpio::Analog>;
pub type RightThumbStickY = gpio::gpioa::PA3<gpio::Analog>;

pub type Navigation = e32::E32<pac::USART1>;
