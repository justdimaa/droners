use core::marker::PhantomData;

use stm32f4xx_hal::{
    bb, dma,
    pac::{self, TIM2, TIM3, TIM4, TIM5},
    rcc::Clocks,
    time::Hertz,
};

pub const DSHOT_150_MHZ: u32 = 3;
pub const DSHOT_300_MHZ: u32 = 6;
pub const DSHOT_600_MHZ: u32 = 12;
pub const DSHOT_1200_MHZ: u32 = 24;

pub const DMA_BUFFER_LEN: usize = DSHOT_BUFFER_LEN + 2;
const DSHOT_BUFFER_LEN: usize = 16;
const DSHOT_BIT_0: u16 = 7;
const DSHOT_BIT_1: u16 = 14;

#[derive(Debug)]
pub struct EscChannels<TRANSFER> {
    throttle: u16,
    buffer: Option<&'static mut [u16; DMA_BUFFER_LEN]>,
    _transfer: PhantomData<TRANSFER>,
}

impl<TRANSFER> EscChannels<TRANSFER> {
    pub fn get_throttle(&self) -> u16 {
        self.throttle
    }

    pub fn set_throttle(&mut self, throttle: u16) {
        self.throttle = throttle;
    }
}

macro_rules! esc_all_channels {
    ($($TIMX:ident: ($timx:ident, $apbenr:ident, $apbrstr:ident, $bit:expr, $pclk:ident, $ppre:ident),)+) => {
        $(
            pub fn $timx<T: Into<Hertz>, TRANSFER1, TRANSFER2, TRANSFER3, TRANSFER4>(tim: $TIMX, clocks: Clocks, freq: T)
            -> (
                EscChannels<TRANSFER1>,
                EscChannels<TRANSFER2>,
                EscChannels<TRANSFER3>,
                EscChannels<TRANSFER4>,
            ) {
                unsafe {
                    let rcc = &(*pac::RCC::ptr());
                    bb::set(&rcc.$apbenr, $bit);
                    bb::set(&rcc.$apbrstr, $bit);
                    bb::clear(&rcc.$apbrstr, $bit);
                }

                tim.ccmr1_output()
                    .modify(|_, w| w.oc1pe().set_bit().oc1m().pwm_mode1());
                tim.ccmr1_output()
                    .modify(|_, w| w.oc2pe().set_bit().oc2m().pwm_mode1());
                tim.ccmr2_output()
                    .modify(|_, w| w.oc3pe().set_bit().oc3m().pwm_mode1());
                tim.ccmr2_output()
                    .modify(|_, w| w.oc4pe().set_bit().oc4m().pwm_mode1());

                tim.cr1.modify(|_, w| w.arpe().set_bit());

                let clk = clocks.$pclk().0 * if clocks.$ppre() == 1 { 1 } else { 2 };
                tim.psc
                    .write(|w| w.psc().bits((clk / freq.into().0 - 1) as u16));
                tim.arr.write(|w| unsafe { w.bits(DMA_BUFFER_LEN as u32) });

                tim.cr1.modify(|_, w| w.urs().set_bit());
                tim.egr.write(|w| w.ug().set_bit());
                tim.cr1.modify(|_, w| w.urs().clear_bit());

                tim.cr1.write(|w| {
                    w.cms()
                        .bits(0b00)
                        .dir()
                        .clear_bit()
                        .opm()
                        .clear_bit()
                        .cen()
                        .set_bit()
                });

                tim.ccer.modify(|_, w| w.cc1e().set_bit());
                tim.ccer.modify(|_, w| w.cc2e().set_bit());
                tim.ccer.modify(|_, w| w.cc3e().set_bit());
                tim.ccer.modify(|_, w| w.cc4e().set_bit());

                (
                    EscChannels {
                        throttle: 0,
                        buffer: Some(cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]).unwrap()),
                        _transfer: PhantomData::default(),
                    },
                    EscChannels {
                        throttle: 0,
                        buffer: Some(cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]).unwrap()),
                        _transfer: PhantomData::default(),
                    },
                    EscChannels {
                        throttle: 0,
                        buffer: Some(cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]).unwrap()),
                        _transfer: PhantomData::default(),
                    },
                    EscChannels {
                        throttle: 0,
                        buffer: Some(cortex_m::singleton!(: [u16; DMA_BUFFER_LEN] = [0; DMA_BUFFER_LEN]).unwrap()),
                        _transfer: PhantomData::default(),
                    },
                )
            }
        )+
    };
}

esc_all_channels!(
    TIM2: (tim2, apb1enr, apb1rstr, 0u8, pclk1, ppre1),
    TIM3: (tim3, apb1enr, apb1rstr, 1u8, pclk1, ppre1),
    TIM4: (tim4, apb1enr, apb1rstr, 2u8, pclk1, ppre1),
    TIM5: (tim5, apb1enr, apb1rstr, 3u8, pclk1, ppre1),
);

macro_rules! esc_transfers {
    {$(($STREAM:ty, $CHANNEL:ty, $PERIPHERAL:ty, $ccde:ident)),+ $(,)*} =>  {
        $(
            impl EscChannels<dma::Transfer<$STREAM, $CHANNEL, $PERIPHERAL, dma::MemoryToPeripheral, &'static mut [u16; DMA_BUFFER_LEN]>> {
                pub fn start(
                    &mut self,
                    transfer: &mut dma::Transfer<
                        $STREAM, $CHANNEL, $PERIPHERAL, dma::MemoryToPeripheral, &'static mut [u16; DMA_BUFFER_LEN]
                    >,
                ) {
                    let buffer = self.buffer.take().unwrap();
                    let mut packet = encode_dshot_packet(self.throttle, false);

                    for n in 0..DSHOT_BUFFER_LEN {
                        buffer[n] = match packet & 0x8000 {
                            0 => DSHOT_BIT_0,
                            _ => DSHOT_BIT_1,
                        };

                        packet <<= 1;
                    }

                    let buffer = transfer.next_transfer(buffer).unwrap();
                    self.buffer = Some(buffer.0);
                }

                pub fn pause(&mut self, transfer: &mut dma::Transfer<
                    $STREAM, $CHANNEL, $PERIPHERAL, dma::MemoryToPeripheral, &'static mut [u16; DMA_BUFFER_LEN]
                >) {
                    transfer.pause(|tim| {
                        tim.dier.modify(|_, w| w.$ccde().disabled());
                    });
                }
            }
        )+
    };
}

esc_transfers!(
    (
        dma::Stream4<pac::DMA1>,
        dma::Channel5,
        dma::traits::CCR1<pac::TIM3>,
        cc1de
    ),
    (
        dma::Stream5<pac::DMA1>,
        dma::Channel5,
        dma::traits::CCR2<pac::TIM3>,
        cc2de
    ),
    (
        dma::Stream7<pac::DMA1>,
        dma::Channel5,
        dma::traits::CCR3<pac::TIM3>,
        cc3de
    ),
    (
        dma::Stream2<pac::DMA1>,
        dma::Channel5,
        dma::traits::CCR4<pac::TIM3>,
        cc4de
    ),
    (
        dma::Stream0<pac::DMA1>,
        dma::Channel2,
        dma::traits::CCR1<pac::TIM4>,
        cc1de
    ),
    (
        dma::Stream3<pac::DMA1>,
        dma::Channel2,
        dma::traits::CCR2<pac::TIM4>,
        cc2de
    ),
);

fn encode_dshot_packet(mut packet: u16, enable_tel: bool) -> u16 {
    packet = (packet << 1) | enable_tel as u16;
    let csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xf;
    (packet << 4) | csum
}
