#![no_main]
#![no_std]
#![deny(warnings)]

use stm32f4xx_hal::dma;

extern crate cortex_m_semihosting;
extern crate panic_halt;

mod esc;
mod typedefs;

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

    use embedded_sensors::{ak8963, mpu6500, mpu925x, ublox};
    use stm32f4xx_hal::{
        delay,
        dma::{self, traits::Stream},
        gpio, i2c, pac,
        prelude::*,
        serial, stm32, timer,
    };

    use crate::esc;
    use crate::get_dshot_dma_cfg;
    use crate::typedefs::*;

    #[resources]
    struct Resources {
        delay: delay::Delay,

        tim2: Timer2,

        i2c1: I2c1,

        serial1: Serial1,
        serial2: Serial2,

        dma_transfer4: DmaTransfer4,
        dma_transfer5: DmaTransfer5,
        dma_transfer7: DmaTransfer7,
        dma_transfer2: DmaTransfer2,

        esc1: Esc1,
        esc2: Esc2,
        esc3: Esc3,
        esc4: Esc4,

        gps: Gps,

        mpu_aux: MpuAux,
        mpu: Mpu,

        key: Key,
    }

    #[init]
    fn init(cx: init::Context) -> (init::LateResources, init::Monotonics) {
        let core: cortex_m::Peripherals = cx.core;
        let mut device: stm32::Peripherals = cx.device;

        let rcc = device.RCC.constrain();

        let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(48.mhz()).freeze();

        let mut syscfg = device.SYSCFG.constrain();

        let mut delay = delay::Delay::new(core.SYST, clocks);

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();

        gpiob.pb4.into_alternate_af2();
        gpiob.pb5.into_alternate_af2();
        gpiob.pb0.into_alternate_af2();
        gpiob.pb1.into_alternate_af2();

        let mut tim2 = timer::Timer::tim2(device.TIM2, 750.hz(), clocks);
        tim2.listen(timer::Event::TimeOut);

        let mut i2c1 = i2c::I2c::new(
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

        let dma1_streams = dma::StreamsTuple::new(device.DMA1);

        let ccr1_tim3 =
            dma::traits::CCR1::<pac::TIM3>(unsafe { mem::transmute_copy(&device.TIM3) });
        let ccr2_tim3 =
            dma::traits::CCR2::<pac::TIM3>(unsafe { mem::transmute_copy(&device.TIM3) });
        let ccr3_tim3 =
            dma::traits::CCR3::<pac::TIM3>(unsafe { mem::transmute_copy(&device.TIM3) });
        let ccr4_tim3 =
            dma::traits::CCR4::<pac::TIM3>(unsafe { mem::transmute_copy(&device.TIM3) });

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

        let (esc1, esc2, esc3, esc4): (Esc1, Esc2, Esc3, Esc4) =
            esc::tim3(device.TIM3, clocks, esc::DSHOT_600_MHZ.mhz());

        let mut mpu_aux = gpiob.pb2.into_floating_input();
        mpu_aux.make_interrupt_source(&mut syscfg);
        mpu_aux.enable_interrupt(&mut device.EXTI);
        mpu_aux.trigger_on_edge(&mut device.EXTI, gpio::Edge::RISING);

        let mpu = mpu925x::Mpu925x::<I2c1, mpu925x::Madgwick>::with_configuration(
            0x0C,
            0x68,
            &mut i2c1,
            &mut delay,
            mpu925x::config::Config::default()
                .ak(ak8963::config::Config::default())
                .mpu(mpu6500::config::Config::default()),
        );

        if let Err(e) = &mpu {
            cortex_m_semihosting::heprintln!("{:?}", e).ok();
            panic!("could not initialize mpu.");
        }

        let mpu = mpu.unwrap();

        let gps = ublox::Ublox::<Serial2>::new();

        let mut key = gpioa.pa0.into_pull_up_input();
        key.make_interrupt_source(&mut syscfg);
        key.enable_interrupt(&mut device.EXTI);
        key.trigger_on_edge(&mut device.EXTI, gpio::Edge::FALLING);

        (
            init::LateResources {
                delay,
                tim2,

                i2c1,

                serial1,
                serial2,

                dma_transfer4,
                dma_transfer5,
                dma_transfer7,
                dma_transfer2,

                esc1,
                esc2,
                esc3,
                esc4,

                mpu_aux,
                mpu,

                gps,

                key,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = EXTI0, resources = [delay, i2c1, mpu, key])]
    fn exti0(mut cx: exti0::Context) {
        let delay = &mut cx.resources.delay;
        let i2c1 = &mut cx.resources.i2c1;
        let mpu = &mut cx.resources.mpu;
        let key = &mut cx.resources.key;

        key.lock(|key: &mut Key| {
            if !key.check_interrupt() {
                return;
            }

            key.clear_interrupt_pending_bit();

            delay.lock(|delay: &mut delay::Delay| {
                (i2c1, mpu).lock(|i2c: &mut I2c1, mpu: &mut Mpu| {
                    mpu.calibrate_mpu(i2c, delay).ok();
                    mpu.calibrate_ak(i2c, delay).ok();
                });
            });
        });
    }

    #[task(binds = EXTI2, resources = [i2c1, mpu_aux, mpu])]
    fn exti2(mut cx: exti2::Context) {
        let i2c1 = &mut cx.resources.i2c1;
        let mpu_aux = &mut cx.resources.mpu_aux;
        let mpu = &mut cx.resources.mpu;

        mpu_aux.lock(|mpu_aux: &mut MpuAux| {
            if !mpu_aux.check_interrupt() {
                return;
            }

            mpu_aux.clear_interrupt_pending_bit();

            (i2c1, mpu).lock(|i2c: &mut I2c1, mpu: &mut Mpu| {
                mpu.read(i2c).ok();
            })
        });
    }

    #[task(binds = USART2, resources = [serial2, gps])]
    fn usart2(mut cx: usart2::Context) {
        let serial = &mut cx.resources.serial2;
        let gps = &mut cx.resources.gps;

        serial.lock(|serial: &mut Serial2| {
            if serial.is_rxne() {
                gps.lock(|gps: &mut Gps| {
                    let _ = gps.read(serial);
                })
            }
        });
    }

    #[task(binds = TIM2, priority = 2, resources = [tim2, dma_transfer4, dma_transfer5, dma_transfer7, dma_transfer2, esc1, esc2, esc3, esc4, mpu])]
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

        let mpu = &mut cx.resources.mpu;

        let _rotation = mpu.lock(|mpu: &mut Mpu| mpu.rotation());

        // todo: impl pid controllers

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
