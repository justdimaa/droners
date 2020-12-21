#![no_main]
#![no_std]

#[macro_use]
extern crate cortex_m_semihosting;
extern crate panic_halt;

use core::mem;

use aeroflight_components::{esc, internal};
use esc::DSHOT_600_MHZ;
use stm32f4xx_hal::{
    dma::{self, traits::Stream},
    gpio, i2c, pac,
    prelude::*,
    stm32, time,
};

// TIM3_CH1
type DmaTransfer4 = dma::Transfer<
    dma::Stream4<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR1<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; esc::DMA_BUFFER_LEN],
>;

// TIM3_CH2
type DmaTransfer5 = dma::Transfer<
    dma::Stream5<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR2<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; esc::DMA_BUFFER_LEN],
>;

// TIM3_CH3
type DmaTransfer7 = dma::Transfer<
    dma::Stream7<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR3<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; esc::DMA_BUFFER_LEN],
>;

// TIM3_CH4
type DmaTransfer2 = dma::Transfer<
    dma::Stream2<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR4<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; esc::DMA_BUFFER_LEN],
>;

type I2c1 = i2c::I2c<
    pac::I2C1,
    (
        gpio::gpiob::PB6<gpio::AlternateOD<gpio::AF4>>,
        gpio::gpiob::PB7<gpio::AlternateOD<gpio::AF4>>,
    ),
>;

type Navigation = internal::Navigation<pac::I2C1>;

type Esc1 = esc::EscChannels<DmaTransfer4>;
type Esc2 = esc::EscChannels<DmaTransfer5>;
type Esc3 = esc::EscChannels<DmaTransfer7>;
type Esc4 = esc::EscChannels<DmaTransfer2>;

fn get_dshot_dma_cfg() -> dma::config::DmaConfig {
    dma::config::DmaConfig::default()
        .transfer_complete_interrupt(true)
        .transfer_error_interrupt(false)
        .half_transfer_interrupt(false)
        .fifo_enable(true)
        .fifo_threshold(dma::config::FifoThreshold::QuarterFull)
        .peripheral_burst(dma::config::BurstMode::NoBurst)
        .peripheral_increment(false)
        .memory_burst(dma::config::BurstMode::NoBurst)
        .memory_increment(true)
        .priority(dma::config::Priority::High)
}

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        i2c1: I2c1,
        dma_transfer4: DmaTransfer4,
        dma_transfer5: DmaTransfer5,
        dma_transfer7: DmaTransfer7,
        dma_transfer2: DmaTransfer2,
        nav: Navigation,
        esc1: Esc1,
        esc2: Esc2,
        esc3: Esc3,
        esc4: Esc4,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let core: cortex_m::Peripherals = cx.core;
        let device: stm32::Peripherals = cx.device;

        let rcc = device.RCC.constrain();

        let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(48.mhz()).freeze();

        let gpiob = device.GPIOB.split();
        gpiob.pb4.into_alternate_af2();
        gpiob.pb5.into_alternate_af2();
        gpiob.pb0.into_alternate_af2();
        gpiob.pb1.into_alternate_af2();

        let tim3 = device.TIM3;

        let dma1_streams = dma::StreamsTuple::new(device.DMA1);

        let ccr1_tim3 = dma::traits::CCR1::<pac::TIM3>(unsafe { mem::transmute_copy(&tim3) });
        let ccr2_tim3 = dma::traits::CCR2::<pac::TIM3>(unsafe { mem::transmute_copy(&tim3) });
        let ccr3_tim3 = dma::traits::CCR3::<pac::TIM3>(unsafe { mem::transmute_copy(&tim3) });
        let ccr4_tim3 = dma::traits::CCR4::<pac::TIM3>(unsafe { mem::transmute_copy(&tim3) });

        let dma_cfg = get_dshot_dma_cfg();

        let dma_transfer4: DmaTransfer4 = dma::Transfer::init(
            dma1_streams.4,
            ccr1_tim3,
            cortex_m::singleton!(: [u16; esc::DMA_BUFFER_LEN] = [0; esc::DMA_BUFFER_LEN]).unwrap(),
            None,
            dma_cfg,
        );

        let dma_transfer5: DmaTransfer5 = dma::Transfer::init(
            dma1_streams.5,
            ccr2_tim3,
            cortex_m::singleton!(: [u16; esc::DMA_BUFFER_LEN] = [0; esc::DMA_BUFFER_LEN]).unwrap(),
            None,
            dma_cfg,
        );

        let dma_transfer7: DmaTransfer7 = dma::Transfer::init(
            dma1_streams.7,
            ccr3_tim3,
            cortex_m::singleton!(: [u16; esc::DMA_BUFFER_LEN] = [0; esc::DMA_BUFFER_LEN]).unwrap(),
            None,
            dma_cfg,
        );

        let dma_transfer2: DmaTransfer2 = dma::Transfer::init(
            dma1_streams.2,
            ccr4_tim3,
            cortex_m::singleton!(: [u16; esc::DMA_BUFFER_LEN] = [0; esc::DMA_BUFFER_LEN]).unwrap(),
            None,
            dma_cfg,
        );

        let i2c1 = i2c::I2c::i2c1(
            device.I2C1,
            (
                gpiob.pb6.into_alternate_af4_open_drain(),
                gpiob.pb7.into_alternate_af4_open_drain(),
            ),
            400.khz(),
            clocks,
        );

        let nav = internal::Navigation::new();

        let (esc1, esc2, esc3, esc4): (Esc1, Esc2, Esc3, Esc4) =
            esc::tim3(tim3, clocks, DSHOT_600_MHZ.mhz());

        init::LateResources {
            i2c1,
            dma_transfer4,
            dma_transfer5,
            dma_transfer7,
            dma_transfer2,
            nav,
            esc1,
            esc2,
            esc3,
            esc4,
        }
    }

    #[idle(spawn = [escs])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cx.spawn.escs().unwrap();
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = I2C1_EV, resources = [i2c1, nav, esc1, esc2, esc3, esc4])]
    fn i2c1_ev(mut cx: i2c1_ev::Context) {
        let i2c1: &mut I2c1 = cx.resources.i2c1;
        let nav: &mut Navigation = cx.resources.nav;
        let esc1 = &mut cx.resources.esc1;
        let esc2 = &mut cx.resources.esc2;
        let esc3 = &mut cx.resources.esc3;
        let esc4 = &mut cx.resources.esc4;

        if let Ok(cmd) = nav.read(i2c1) {
            if let Some(cmd) = cmd {
                match cmd {
                    internal::Command::ThrottleBulk { values } => {
                        if let Some(throttle) = values[0] {
                            esc1.lock(|esc: &mut Esc1| esc.set_throttle(throttle));
                        }

                        if let Some(throttle) = values[1] {
                            esc2.lock(|esc: &mut Esc2| esc.set_throttle(throttle));
                        }

                        if let Some(throttle) = values[2] {
                            esc3.lock(|esc: &mut Esc3| esc.set_throttle(throttle));
                        }

                        if let Some(throttle) = values[3] {
                            esc4.lock(|esc: &mut Esc4| esc.set_throttle(throttle));
                        }
                    }
                }
            }
        }
    }

    #[task(resources = [dma_transfer4, dma_transfer5, dma_transfer7, dma_transfer2, esc1, esc2, esc3, esc4])]
    fn escs(mut cx: escs::Context) {
        let dma_transfer4 = &mut cx.resources.dma_transfer4;
        let dma_transfer5 = &mut cx.resources.dma_transfer5;
        let dma_transfer7 = &mut cx.resources.dma_transfer7;
        let dma_transfer2 = &mut cx.resources.dma_transfer2;
        let esc1 = &mut cx.resources.esc1;
        let esc2 = &mut cx.resources.esc2;
        let esc3 = &mut cx.resources.esc3;
        let esc4 = &mut cx.resources.esc4;

        dma_transfer4.lock(|transfer: &mut DmaTransfer4| {
            esc1.lock(|esc: &mut Esc1| {
                esc.start(transfer);
            });
        });

        dma_transfer5.lock(|transfer: &mut DmaTransfer5| {
            esc2.lock(|esc: &mut Esc2| {
                esc.start(transfer);
            });
        });

        dma_transfer7.lock(|transfer: &mut DmaTransfer7| {
            esc3.lock(|esc: &mut Esc3| {
                esc.start(transfer);
            });
        });

        dma_transfer2.lock(|transfer: &mut DmaTransfer2| {
            esc4.lock(|esc: &mut Esc4| {
                esc.start(transfer);
            });
        });

        unsafe {
            let tim3 = pac::TIM3::ptr();
            (*tim3).cnt.modify(|_, w| w.bits(0));
            (*tim3).dier.modify(|_, w| w.cc1de().enabled());
            (*tim3).dier.modify(|_, w| w.cc2de().enabled());
            (*tim3).dier.modify(|_, w| w.cc3de().enabled());
            (*tim3).dier.modify(|_, w| w.cc4de().enabled());
        }
    }

    #[task(binds = DMA1_STREAM4, priority = 2, resources = [dma_transfer4, esc1])]
    fn dma1_stream4(cx: dma1_stream4::Context) {
        let dma_transfer: &mut DmaTransfer4 = cx.resources.dma_transfer4;
        let esc: &mut Esc1 = cx.resources.esc1;

        if !dma::Stream4::<pac::DMA1>::get_transfer_complete_flag() {
            return;
        }

        dma_transfer.clear_transfer_complete_interrupt();
        esc.pause(dma_transfer);
    }

    #[task(binds = DMA1_STREAM5, priority = 2, resources = [dma_transfer5, esc2])]
    fn dma1_stream5(cx: dma1_stream5::Context) {
        let dma_transfer: &mut DmaTransfer5 = cx.resources.dma_transfer5;
        let esc: &mut Esc2 = cx.resources.esc2;

        if !dma::Stream5::<pac::DMA1>::get_transfer_complete_flag() {
            return;
        }

        dma_transfer.clear_transfer_complete_interrupt();
        esc.pause(dma_transfer);
    }

    #[task(binds = DMA1_STREAM7, priority = 2, resources = [dma_transfer7, esc3])]
    fn dma1_stream7(cx: dma1_stream7::Context) {
        let dma_transfer: &mut DmaTransfer7 = cx.resources.dma_transfer7;
        let esc: &mut Esc3 = cx.resources.esc3;

        if !dma::Stream7::<pac::DMA1>::get_transfer_complete_flag() {
            return;
        }

        dma_transfer.clear_transfer_complete_interrupt();
        esc.pause(dma_transfer);
    }

    #[task(binds = DMA1_STREAM2, priority = 2, resources = [dma_transfer2, esc4])]
    fn dma1_stream2(cx: dma1_stream2::Context) {
        let dma_transfer: &mut DmaTransfer2 = cx.resources.dma_transfer2;
        let esc: &mut Esc4 = cx.resources.esc4;

        if !dma::Stream2::<pac::DMA1>::get_transfer_complete_flag() {
            return;
        }

        dma_transfer.clear_transfer_complete_interrupt();
        esc.pause(dma_transfer);
    }

    extern "C" {
        fn EXTI2();
    }
};
