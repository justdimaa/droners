#![no_main]
#![no_std]

#[macro_use]
extern crate cortex_m_semihosting;
extern crate panic_halt;

use aeroflight_components::{internal, neo6m, sx1278};
use stm32f4xx_hal::{gpio, i2c, pac, prelude::*, pwm, serial, stm32};

type I2c1 = i2c::I2c<
    pac::I2C1,
    (
        gpio::gpiob::PB6<gpio::AlternateOD<gpio::AF4>>,
        gpio::gpiob::PB7<gpio::AlternateOD<gpio::AF4>>,
    ),
>;

type Serial1 = serial::Serial<
    pac::USART1,
    (
        gpio::gpioa::PA9<gpio::Alternate<gpio::AF7>>,
        gpio::gpioa::PA10<gpio::Alternate<gpio::AF7>>,
    ),
>;

type Serial2 = serial::Serial<
    pac::USART2,
    (
        gpio::gpioa::PA2<gpio::Alternate<gpio::AF7>>,
        gpio::gpioa::PA3<gpio::Alternate<gpio::AF7>>,
    ),
>;

type Controller = sx1278::E32<pac::USART1>;
type GpsModule = neo6m::Neo6m<pac::USART2>;
type FlightControl = internal::FlightControl<pac::I2C1>;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        i2c1: I2c1,
        serial1: Serial1,
        serial2: Serial2,
        controller: Controller,
        gps: GpsModule,
        fc: FlightControl,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let core: cortex_m::Peripherals = cx.core;
        let device: stm32::Peripherals = cx.device;

        let rcc = device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .sysclk(84.mhz())
            .pclk2(28.mhz())
            .pclk1(28.mhz())
            .freeze();

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();

        let i2c1 = i2c::I2c::i2c1(
            device.I2C1,
            (
                gpiob.pb6.into_alternate_af4_open_drain(),
                gpiob.pb7.into_alternate_af4_open_drain(),
            ),
            400.khz(),
            clocks,
        );

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

        let serial2 = serial::Serial::usart2(
            device.USART2,
            (
                gpioa.pa2.into_alternate_af7(),
                gpioa.pa3.into_alternate_af7(),
            ),
            serial::config::Config::default()
                .baudrate(9600.bps())
                .parity_even()
                .stopbits(serial::config::StopBits::STOP1)
                .wordlength_8(),
            clocks,
        )
        .unwrap();

        let controller = sx1278::E32::<pac::USART1>::new();
        let gps = neo6m::Neo6m::<pac::USART2>::new();
        let fc = internal::FlightControl::<pac::I2C1>::new();

        init::LateResources {
            i2c1,
            serial1,
            serial2,
            controller,
            gps,
            fc,
        }
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = USART1, resources = [serial1, controller, i2c1, fc])]
    fn usart1(cx: usart1::Context) {
        let serial1: &mut Serial1 = cx.resources.serial1;
        let controller: &mut Controller = cx.resources.controller;
        let i2c1: &mut I2c1 = cx.resources.i2c1;
        let fc: &mut FlightControl = cx.resources.fc;

        while serial1.is_rxne() {
            match controller.read(serial1) {
                Ok(v) => {
                    match v {
                        Some(v) => {
                            let throttle = v.right_trigger as u16 * 256;
                            if let Err(_) = fc.set_throttle(i2c1, 0, throttle) {
                                hprintln!("failed to send throttle {}", throttle).unwrap();
                            }
                        }
                        None => {}
                    }

                    break;
                }
                Err(_) => {}
            }
        }
    }

    #[task(binds = USART2, resources = [serial2, gps])]
    fn usart2(cx: usart2::Context) {
        let serial2: &mut Serial2 = cx.resources.serial2;
        let gps: &mut GpsModule = cx.resources.gps;

        if let Some(v) = gps.read(serial2) {}
    }
};
