#![no_main]
#![no_std]

#[macro_use]
extern crate cortex_m_semihosting;
extern crate panic_halt;

use core::ops::Deref;

use stm32f4xx_hal::{
    delay,
    dma::{self, traits::Stream},
    gpio, pac,
    prelude::*,
    pwm, spi, stm32, time, timer,
};

// TIM3_CH1
type DmaTransfer4 = dma::Transfer<
    dma::Stream4<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR1<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; DMA_BUFFER_LEN],
>;

// TIM3_CH2
type DmaTransfer5 = dma::Transfer<
    dma::Stream5<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR1<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; DMA_BUFFER_LEN],
>;

// TIM3_CH3
type DmaTransfer7 = dma::Transfer<
    dma::Stream7<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR1<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; DMA_BUFFER_LEN],
>;

// TIM3_CH4
type DmaTransfer2 = dma::Transfer<
    dma::Stream2<pac::DMA1>,
    dma::Channel5,
    dma::traits::CCR1<pac::TIM3>,
    dma::MemoryToPeripheral,
    &'static mut [u16; DMA_BUFFER_LEN],
>;

const DMA_BUFFER_LEN: usize = DSHOT_BUFFER_LEN + 2;
const DSHOT_BUFFER_LEN: usize = 16;
const DSHOT_BIT_0: u16 = 7;
const DSHOT_BIT_1: u16 = 14;
const DSHOT_600_MHZ: u32 = 12;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        delay: delay::Delay,
        dma_transfer4: DmaTransfer4,
        dma_buffer4: Option<&'static mut [u16; DMA_BUFFER_LEN]>,
        tim2: timer::Timer<pac::TIM2>,
        dshot_step: u32,
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

        unsafe {
            let rcc = &(*pac::RCC::ptr());
            rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());
            rcc.apb1rstr.modify(|_, w| w.tim3rst().set_bit());
            rcc.apb1rstr.modify(|_, w| w.tim3rst().clear_bit());
        }

        let tim3 = device.TIM3;

        tim3.ccmr1_output()
            .modify(|_, w| w.oc1pe().set_bit().oc1m().pwm_mode1()); // enable ch1
        tim3.ccmr1_output()
            .modify(|_, w| w.oc2pe().set_bit().oc2m().pwm_mode1()); // enable ch2
        tim3.cr1.modify(|_, w| w.arpe().set_bit());

        let freq: time::Hertz = DSHOT_600_MHZ.mhz().into();
        let clk = clocks.pclk1().0 * if clocks.ppre1() == 1 { 1 } else { 2 };
        tim3.psc.write(|w| w.psc().bits((clk / freq.0 - 1) as u16));
        tim3.arr.write(|w| unsafe { w.bits(DMA_BUFFER_LEN as u32) });

        tim3.cr1.modify(|_, w| w.urs().set_bit());
        tim3.egr.write(|w| w.ug().set_bit());
        tim3.cr1.modify(|_, w| w.urs().clear_bit());

        tim3.cr1.write(|w| {
            w.cms()
                .bits(0b00)
                .dir()
                .clear_bit()
                .opm()
                .clear_bit()
                .cen()
                .set_bit()
        });

        tim3.ccer.modify(|_, w| w.cc1e().set_bit());

        let dma1_streams = dma::StreamsTuple::new(device.DMA1);
        let ccr1_tim3 = dma::traits::CCR1::<pac::TIM3>(tim3);

        let dma_transfer4: DmaTransfer4 = dma::Transfer::init(
            dma1_streams.4,
            ccr1_tim3,
            cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]).unwrap(),
            None,
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
                .priority(dma::config::Priority::High),
        );

        let delay = delay::Delay::new(core.SYST, clocks);

        let mut tim2 = timer::Timer::tim2(device.TIM2, 1.hz(), clocks);
        tim2.listen(timer::Event::TimeOut);

        init::LateResources {
            delay,
            dma_transfer4,
            dma_buffer4: Some(
                cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]).unwrap(),
            ),
            tim2,
            dshot_step: 0,
        }
    }

    #[idle(resources = [dma_transfer4])]
    fn idle(mut cx: idle::Context) -> ! {
        hprintln!("idle").unwrap();

        cx.resources
            .dma_transfer4
            .lock(move |shared: &mut DmaTransfer4| {
                shared.start(|tim3| {
                    tim3.dier.modify(|_, w| w.cc1de().enabled());
                });
            });

        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = TIM2, resources = [dshot_step])]
    fn tim2(cx: tim2::Context) {
        hprintln!("tim2").unwrap();

        let dshot_step: &mut u32 = cx.resources.dshot_step;

        match *dshot_step {
            0..=100 => *dshot_step += 1,
            _ => {},
        }
    }

    #[task(binds = DMA1_STREAM4, resources = [dma_transfer4, dma_buffer4, dshot_step])]
    fn dma1_stream4(cx: dma1_stream4::Context) {
        let dma_transfer: &mut DmaTransfer4 = cx.resources.dma_transfer4;
        let dma_buffer = cx.resources.dma_buffer4.take().unwrap();
        let dshot_step: &mut u32 = cx.resources.dshot_step;

        let mut packet = match *dshot_step {
            0..=40 => encode_dshot_packet(0, false),
            41..=100 => encode_dshot_packet(2047, false),
            _ => encode_dshot_packet(0, false),
        };

        for n in 0..DSHOT_BUFFER_LEN {
            dma_buffer[n] = match packet & 0x8000 {
                0 => DSHOT_BIT_0,
                _ => DSHOT_BIT_1,
            };

            packet <<= 1;
        }

        let buf = dma_transfer.next_transfer(dma_buffer).unwrap();
        *cx.resources.dma_buffer4 = Some(buf.0);
    }
};

fn encode_dshot_packet(mut packet: u16, enable_tel: bool) -> u16 {
    packet = (packet << 1) | enable_tel as u16;
    let csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xf;
    (packet << 4) | csum
}
