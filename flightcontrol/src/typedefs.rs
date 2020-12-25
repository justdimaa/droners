use droners_components::{e32, esc, ublox};
use stm32f4xx_hal::{dma, gpio, pac, serial, timer};

pub type Timer2 = timer::Timer<pac::TIM2>;

pub type Serial1 = serial::Serial<
    pac::USART1,
    (
        gpio::gpioa::PA9<gpio::Alternate<gpio::AF7>>,
        gpio::gpioa::PA10<gpio::Alternate<gpio::AF7>>,
    ),
>;

pub type Serial2 = serial::Serial<
    pac::USART2,
    (
        gpio::gpioa::PA2<gpio::Alternate<gpio::AF7>>,
        gpio::gpioa::PA3<gpio::Alternate<gpio::AF7>>,
    ),
>;

// TIM3_CH1
pub type DmaTransfer4 = dma::Transfer<
    dma::Stream4<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR1<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; esc::DMA_BUFFER_LEN],
>;

// TIM3_CH2
pub type DmaTransfer5 = dma::Transfer<
    dma::Stream5<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR2<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; esc::DMA_BUFFER_LEN],
>;

// TIM3_CH3
pub type DmaTransfer7 = dma::Transfer<
    dma::Stream7<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR3<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; esc::DMA_BUFFER_LEN],
>;

// TIM3_CH4
pub type DmaTransfer2 = dma::Transfer<
    dma::Stream2<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR4<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; esc::DMA_BUFFER_LEN],
>;

pub type Esc1 = esc::EscChannels<DmaTransfer4>;
pub type Esc2 = esc::EscChannels<DmaTransfer5>;
pub type Esc3 = esc::EscChannels<DmaTransfer7>;
pub type Esc4 = esc::EscChannels<DmaTransfer2>;

pub type ControllerAux = gpio::gpioa::PA8<gpio::Input<gpio::PullUp>>;
pub type ControllerM0 = gpio::gpiob::PB14<gpio::Output<gpio::OpenDrain>>;
pub type ControllerM1 = gpio::gpiob::PB15<gpio::Output<gpio::OpenDrain>>;
pub type Controller = e32::E32<pac::USART1>;

pub type GpsModule = ublox::Ublox<Serial2>;
