use attiny_hal::pac::TC1;
use attiny_hal::port::mode::{Floating, Input, Output};
use attiny_hal::port::{Pin, PB3, PB4};
use attiny_hal::prelude::_embedded_hal_serial_Read;
use bitbang_hal::serial::{Error, Serial};
use core::convert::Infallible;
use embedded_hal::timer::{CountDown, Periodic};
use nb::Error::WouldBlock;
use void::Void;

pub struct SoftwareUart<const BAUD_RATE: u32, const CPU_FREQUENCY: u32> {
    serial: Serial<Pin<Output, PB4>, Pin<Input<Floating>, PB3>, Timer<BAUD_RATE, CPU_FREQUENCY>>,
}

pub struct Timer<const BAUD_RATE: u32, const CPU_FREQUENCY: u32> {
    inner: TC1,
}

#[derive(Clone, Copy, Debug)]
pub enum Prescaler {
    Direct = 1,
    Div8 = 8,
    Div64 = 64,
    Div256 = 256,
    Div1024 = 1024,
}

impl<const BAUD_RATE: u32, const CPU_FREQUENCY: u32> Timer<BAUD_RATE, CPU_FREQUENCY> {
    const fn assert_valid_params() {
        assert!(BAUD_RATE > 0, "BAUD_RATE must be greater than 0");
        assert!(CPU_FREQUENCY > 0, "CPU_FREQUENCY must be greater than 0");
    }

    pub const fn new(timer: TC1) -> Self {
        Self::assert_valid_params();
        Self { inner: timer }
    }

    const fn calculate_prescaler() -> (Prescaler, u8) {
        const PRESCALERS: [(Prescaler, u32); 5] = [
            (Prescaler::Direct, 1),
            (Prescaler::Div8, 8),
            (Prescaler::Div64, 64),
            (Prescaler::Div256, 256),
            (Prescaler::Div1024, 1024),
        ];

        let mut i = 0;
        while i < PRESCALERS.len() {
            let (prescaler, prescaler_value) = PRESCALERS[i];
            let ticks_per_bit = (CPU_FREQUENCY / (BAUD_RATE * prescaler_value)) as u16;
            if ticks_per_bit > 0 && ticks_per_bit <= 255 {
                return (prescaler, ticks_per_bit as u8);
            }
            i += 1;
        }

        panic!("No suitable prescaler found for the given BAUD_RATE and CPU_FREQUENCY");
    }

    const PRESCALER: Prescaler = Self::calculate_prescaler().0;
    const TICKS_PER_BIT: u8 = Self::calculate_prescaler().1;
}

impl<const BAUD_RATE: u32, const CPU_FREQUENCY: u32> CountDown for Timer<BAUD_RATE, CPU_FREQUENCY> {
    type Time = ();

    fn start<T>(&mut self, _duration: T)
    where
        T: Into<()>,
    {
        // Set up timer with appropriate prescaling
        self.inner.tccr1.write(|w| match Self::PRESCALER {
            Prescaler::Direct => w.cs1().direct(),
            Prescaler::Div8 => w.cs1().prescale_8(),
            Prescaler::Div64 => w.cs1().prescale_64(),
            Prescaler::Div256 => w.cs1().prescale_256(),
            Prescaler::Div1024 => w.cs1().prescale_1024(),
        });
        // Reset timer count
        self.inner.tcnt1.write(|w| unsafe { w.bits(0) });
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        let current_count = self.inner.tcnt1.read().bits();
        if current_count < Self::TICKS_PER_BIT {
            Err(WouldBlock)
        } else {
            // Reset timer count
            self.inner.tcnt1.write(|w| unsafe { w.bits(0) });
            Ok(())
        }
    }
}

impl<const BAUD_RATE: u32, const CPU_FREQUENCY: u32> Periodic for Timer<BAUD_RATE, CPU_FREQUENCY> {}

impl<const BAUD_RATE: u32, const CPU_FREQUENCY: u32> SoftwareUart<BAUD_RATE, CPU_FREQUENCY> {
    pub fn new(tc1: TC1, rx: Pin<Input<Floating>, PB3>, tx: Pin<Output, PB4>) -> Self {
        let timer = Timer::new(tc1);
        let serial = Serial::new(tx, rx, timer);

        SoftwareUart { serial }
    }

    pub fn receive(&mut self) -> nb::Result<u8, Error<Infallible>> {
        self.serial.read()
    }
}
