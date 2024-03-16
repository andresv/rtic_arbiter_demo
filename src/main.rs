#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use rtic_arbiter_demo as _; // global logger + panicking-behavior + memory layout

use embassy_stm32::i2c::I2c;
use embassy_stm32::rcc::low_level::RccPeripheral;
use embassy_stm32::time::Hertz;
use embassy_stm32::{
    bind_interrupts, i2c,
    peripherals::{DMA1_CH0, DMA1_CH6, I2C1, TIM2},
};
use rtic::app;
pub use rtic_monotonics::stm32::Tim2 as Mono;
use rtic_monotonics::{stm32::ExtU64, Monotonic};
use rtic_sync::arbiter::{i2c::ArbiterDevice, Arbiter};

pub type Instant = <Mono as Monotonic>::Instant;
pub type Duration = <Mono as Monotonic>::Duration;

#[app(device = embassy_stm32, peripherals = true, dispatchers = [TIM7])]
mod app {
    use super::*;
    use core::mem::MaybeUninit;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init(local = [
        i2c_arbiter: MaybeUninit<Arbiter<I2c<'static, I2C1, DMA1_CH6, DMA1_CH0>>> = MaybeUninit::uninit(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let p = embassy_stm32::init(Default::default());

        // Setup Monotonic clock
        let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
        Mono::start(TIM2::frequency().0, token);

        let i2c = I2c::new(
            p.I2C1,
            p.PB8,
            p.PB7,
            Irqs,
            p.DMA1_CH6,
            p.DMA1_CH0,
            Hertz(100_000),
            Default::default(),
        );

        let i2c_arbiter = cx.local.i2c_arbiter.write(Arbiter::new(i2c));
        // Share I2C bus with both ENS160 drivers.
        let ens160_52 = ens160::Ens160::new(ArbiterDevice::new(i2c_arbiter), 0x52);
        let ens160_53 = ens160::Ens160::new(ArbiterDevice::new(i2c_arbiter), 0x53);

        // Spawn tasks that are reading ENS160 sensors.
        // Notice that ENS160 did not take in just a I2C bus, but a ArbiterDevice<I2c>.
        // ArbiterDevice makes sure that only one task can access I2C at the time.
        // So when one task is reading data from ENS160 then other waits until I2C is free again
        // and then it can use I2C bus to read data.
        read_ens160_52::spawn(ens160_52).ok();
        read_ens160_53::spawn(ens160_53).ok();

        (Shared {}, Local {})
    }

    #[task()]
    async fn read_ens160_52(
        _cx: read_ens160_52::Context,
        mut ens160: ens160::Ens160<ArbiterDevice<'static, i2c::I2c<'static, I2C1, DMA1_CH6, DMA1_CH0>>>,
    ) {
        ens160.reset().await.unwrap();
        Mono::delay(250.millis()).await;
        ens160.operational().await.unwrap();
        Mono::delay(50.millis()).await;

        loop {
            if let Ok(status) = ens160.status().await {
                if status.data_is_ready() {
                    let tvoc = ens160.tvoc().await.unwrap();
                    let eco2 = ens160.eco2().await.unwrap();
                    defmt::info!("ens160 0x52: tvoc {} eco2 {}", tvoc, eco2);
                }
            }
        }
    }

    #[task()]
    async fn read_ens160_53(
        _cx: read_ens160_53::Context,
        mut ens160: ens160::Ens160<ArbiterDevice<'static, i2c::I2c<'static, I2C1, DMA1_CH6, DMA1_CH0>>>,
    ) {
        ens160.reset().await.unwrap();
        Mono::delay(250.millis()).await;
        ens160.operational().await.unwrap();
        Mono::delay(50.millis()).await;

        loop {
            if let Ok(status) = ens160.status().await {
                if status.data_is_ready() {
                    let tvoc = ens160.tvoc().await.unwrap();
                    let eco2 = ens160.eco2().await.unwrap();
                    defmt::info!("ens160 0x53: tvoc {} eco2 {}", tvoc, eco2);
                }
            }
        }
    }
}

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
});
