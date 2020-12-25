#![no_main]
#![no_std]
#![deny(warnings)]

extern crate cortex_m_semihosting;
extern crate panic_halt;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use droners_components::e32;
    use stm32f4xx_hal::{adc, gpio, pac, prelude::*, serial, stm32};

    type Adc1 = adc::Adc<pac::ADC1>;

    type Serial1 = serial::Serial<
        pac::USART1,
        (
            gpio::gpioa::PA9<gpio::Alternate<gpio::AF7>>,
            gpio::gpioa::PA10<gpio::Alternate<gpio::AF7>>,
        ),
    >;

    type LeftThumbStickX = gpio::gpioa::PA0<gpio::Analog>;
    type LeftThumbStickY = gpio::gpioa::PA1<gpio::Analog>;

    type RightThumbStickX = gpio::gpioa::PA2<gpio::Analog>;
    type RightThumbStickY = gpio::gpioa::PA3<gpio::Analog>;

    type Navigation = e32::E32<pac::USART1>;

    #[resources]
    struct Resources {
        adc1: Adc1,
        serial1: Serial1,
        left_thumb_stick_x: LeftThumbStickX,
        left_thumb_stick_y: LeftThumbStickY,
        right_thumb_stick_x: RightThumbStickX,
        right_thumb_stick_y: RightThumbStickY,
        nav: Navigation,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let _core: cortex_m::Peripherals = cx.core;
        let device: stm32::Peripherals = cx.device;

        let rcc = device.RCC.constrain();

        let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(48.mhz()).freeze();

        let gpioa = device.GPIOA.split();
        let pa0 = gpioa.pa0.into_analog();
        let pa1 = gpioa.pa1.into_analog();
        let pa2 = gpioa.pa2.into_analog();
        let pa3 = gpioa.pa3.into_analog();

        let mut adc1 = adc::Adc::adc1(
            device.ADC1,
            true,
            adc::config::AdcConfig::default()
                .end_of_conversion_interrupt(adc::config::Eoc::Conversion)
                .scan(adc::config::Scan::Enabled)
                .clock(adc::config::Clock::Pclk2_div_8),
        );
        adc1.configure_channel(
            &pa0,
            adc::config::Sequence::One,
            adc::config::SampleTime::Cycles_112,
        );
        adc1.configure_channel(
            &pa1,
            adc::config::Sequence::Two,
            adc::config::SampleTime::Cycles_112,
        );
        adc1.configure_channel(
            &pa2,
            adc::config::Sequence::Three,
            adc::config::SampleTime::Cycles_112,
        );
        adc1.configure_channel(
            &pa3,
            adc::config::Sequence::Four,
            adc::config::SampleTime::Cycles_112,
        );
        adc1.start_conversion();

        let serial1 = serial::Serial::usart1(
            device.USART1,
            (
                gpioa.pa9.into_alternate_af7(),
                gpioa.pa10.into_alternate_af7(),
            ),
            serial::config::Config::default()
                .baudrate(9600.bps())
                .parity_even()
                .stopbits(serial::config::StopBits::STOP1)
                .wordlength_8(),
            clocks,
        )
        .unwrap();

        let nav = e32::E32::new();

        init::LateResources {
            adc1,
            serial1,
            left_thumb_stick_x: pa0,
            left_thumb_stick_y: pa1,
            right_thumb_stick_x: pa2,
            right_thumb_stick_y: pa3,
            nav,
        }
    }
}
