use attiny_hal::clock::Clock;
use attiny_hal::delay::Delay;
use attiny_hal::pac::TC1;
use attiny_hal::port::mode::{Floating, Input, Output};
use attiny_hal::port::{Pin, PB3, PB4};
use attiny_hal::prelude::_embedded_hal_timer_CountDown;
use attiny_hal::simple_pwm::{Timer0Pwm, Timer1Pwm};
use avr_device::attiny85::{TC0, USI};
use bitbang_hal::serial::{Error, Serial};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::{CountDown, Periodic};
use nb::Error::WouldBlock;
use void::Void;

pub struct SoftwareUart {
    serial: Serial<Pin<Output, PB4>, Pin<Input<Floating>, PB3>, Timer>,
}

struct Timer {
    inner: TC1,
    frequency: u32,
    skip: u8,
    long: bool,
}

impl Timer {
    pub fn new(timer: TC1) -> Self {
        Self {
            inner: timer,
            frequency: 9600,
            skip: 2,
            long: true,
        }
    }
}

impl CountDown for Timer {
    type Time = bool;

    fn start<T: Into<bool>>(&mut self, long: T) {
        self.inner.tccr1.write(|w| w.cs1().direct());
        self.inner.tcnt1.write(|w| unsafe { w.bits(0) });
        self.skip = 2;
        self.long = long.into();
    }

    fn wait(&mut self) -> nb::Result<(), void::Void> {
        let mut bits = self.inner.tcnt1.read().bits();
        if self.skip == 0 {
            bits += 1;
            self.skip = 2;
        } else {
            self.skip -= 1;
        }
        if (self.long && bits < 1667) || (!self.long && bits < 833) {
            Err(nb::Error::WouldBlock)
        } else {
            self.inner.tcnt1.write(|w| unsafe { w.bits(0) });
            Ok(())
        }
    }
}

impl Periodic for Timer {}

impl SoftwareUart {
    pub fn new(
        tc1: attiny_hal::pac::TC1,
        rx: Pin<Input<Floating>, PB3>,
        tx: Pin<Output, PB4>,
    ) -> Self {
        let timer = Timer::new(tc1);
        let serial = Serial::new(tx, rx, timer);

        SoftwareUart { serial }
    }

    pub fn receive(&mut self) {
        self.serial.read()
    }
}
