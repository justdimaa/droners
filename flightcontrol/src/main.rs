#![no_main]
#![no_std]

use stm32f4xx_hal::dma;

#[macro_use]
extern crate cortex_m_semihosting;
extern crate panic_halt;

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

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use core::mem;

    use droners_components::{esc, internal};
    use esc::DSHOT_600_MHZ;
    use stm32f4xx_hal::{
        dma::{self, traits::Stream},
        gpio, i2c, pac,
        prelude::*,
        stm32, timer,
    };

    use crate::get_dshot_dma_cfg;

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

    type Timer2 = timer::Timer<pac::TIM2>;

    type Navigation = internal::Navigation<pac::I2C1>;

    type Esc1 = esc::EscChannels<DmaTransfer4>;
    type Esc2 = esc::EscChannels<DmaTransfer5>;
    type Esc3 = esc::EscChannels<DmaTransfer7>;
    type Esc4 = esc::EscChannels<DmaTransfer2>;

    #[resources]
    struct Resources {
        tim2: Timer2,
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

        let mut tim2 = timer::Timer::tim2(device.TIM2, 750.hz(), clocks);
        tim2.listen(timer::Event::TimeOut);

        init::LateResources {
            tim2,
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

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = TIM2, priority = 2, resources = [tim2, dma_transfer4, dma_transfer5, dma_transfer7, dma_transfer2, esc1, esc2, esc3, esc4])]
    fn tim2(mut cx: tim2::Context) {
        let tim = &mut cx.resources.tim2;
        let transfer4 = &mut cx.resources.dma_transfer4;
        let transfer5 = &mut cx.resources.dma_transfer5;
        let transfer7 = &mut cx.resources.dma_transfer7;
        let transfer2 = &mut cx.resources.dma_transfer2;
        let esc1 = &mut cx.resources.esc1;
        let esc2 = &mut cx.resources.esc2;
        let esc3 = &mut cx.resources.esc3;
        let esc4 = &mut cx.resources.esc4;

        tim.lock(|tim: &mut Timer2| {
            tim.clear_interrupt(timer::Event::TimeOut);
        });

        (transfer4, esc1).lock(|transfer: &mut DmaTransfer4, esc: &mut Esc1| {
            esc.start(transfer);
        });

        (transfer5, esc2).lock(|transfer: &mut DmaTransfer5, esc: &mut Esc2| {
            esc.start(transfer);
        });

        (transfer7, esc3).lock(|transfer: &mut DmaTransfer7, esc: &mut Esc3| {
            esc.start(transfer);
        });

        (transfer2, esc4).lock(|transfer: &mut DmaTransfer2, esc: &mut Esc4| {
            esc.start(transfer);
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

    #[task(binds = I2C1_EV, resources = [i2c1, nav, esc1, esc2, esc3, esc4])]
    fn i2c1_ev(mut cx: i2c1_ev::Context) {
        let i2c = &mut cx.resources.i2c1;
        let nav = &mut cx.resources.nav;

        let cmd = (i2c, nav).lock(|i2c: &mut I2c1, nav: &mut Navigation| nav.read(i2c));

        if let Ok(cmd) = cmd {
            if let Some(cmd) = cmd {
                match cmd {
                    internal::Command::ThrottleBulk { values } => {
                        if let Some(throttle) = values[0] {
                            let esc = &mut cx.resources.esc1;
                            esc.lock(|esc: &mut Esc1| esc.set_throttle(throttle));
                        }

                        if let Some(throttle) = values[1] {
                            let esc = &mut cx.resources.esc2;
                            esc.lock(|esc: &mut Esc2| esc.set_throttle(throttle));
                        }

                        if let Some(throttle) = values[2] {
                            let esc = &mut cx.resources.esc3;
                            esc.lock(|esc: &mut Esc3| esc.set_throttle(throttle));
                        }

                        if let Some(throttle) = values[3] {
                            let esc = &mut cx.resources.esc4;
                            esc.lock(|esc: &mut Esc4| esc.set_throttle(throttle));
                        }
                    }
                }
            }
        }
    }

    #[task(binds = DMA1_STREAM4, priority = 3, resources = [dma_transfer4, esc1])]
    fn dma1_stream4(mut cx: dma1_stream4::Context) {
        let transfer = &mut cx.resources.dma_transfer4;
        let esc = &mut cx.resources.esc1;

        if !dma::Stream4::<pac::DMA1>::get_transfer_complete_flag() {
            return;
        }

        transfer.lock(|transfer| {
            transfer.clear_transfer_complete_interrupt();

            esc.lock(|esc| {
                esc.pause(transfer);
            })
        });
    }

    #[task(binds = DMA1_STREAM5, priority = 3, resources = [dma_transfer5, esc2])]
    fn dma1_stream5(mut cx: dma1_stream5::Context) {
        let transfer = &mut cx.resources.dma_transfer5;
        let esc = &mut cx.resources.esc2;

        if !dma::Stream5::<pac::DMA1>::get_transfer_complete_flag() {
            return;
        }

        transfer.lock(|transfer| {
            transfer.clear_transfer_complete_interrupt();

            esc.lock(|esc| {
                esc.pause(transfer);
            })
        });
    }

    #[task(binds = DMA1_STREAM7, priority = 3, resources = [dma_transfer7, esc3])]
    fn dma1_stream7(mut cx: dma1_stream7::Context) {
        let transfer = &mut cx.resources.dma_transfer7;
        let esc = &mut cx.resources.esc3;

        if !dma::Stream7::<pac::DMA1>::get_transfer_complete_flag() {
            return;
        }

        transfer.lock(|transfer| {
            transfer.clear_transfer_complete_interrupt();

            esc.lock(|esc| {
                esc.pause(transfer);
            })
        });
    }

    #[task(binds = DMA1_STREAM2, priority = 3, resources = [dma_transfer2, esc4])]
    fn dma1_stream2(mut cx: dma1_stream2::Context) {
        let transfer = &mut cx.resources.dma_transfer2;
        let esc = &mut cx.resources.esc4;

        if !dma::Stream2::<pac::DMA1>::get_transfer_complete_flag() {
            return;
        }

        transfer.lock(|transfer| {
            transfer.clear_transfer_complete_interrupt();

            esc.lock(|esc| {
                esc.pause(transfer);
            })
        });
    }
}
