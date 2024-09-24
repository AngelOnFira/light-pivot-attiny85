use attiny_hal::clock::Clock;
use attiny_hal::delay::Delay;
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
    serial: Serial<Pin<Output, PB4>, Pin<Input, PB3>, Timer>,
}

pub struct Timer {
    timer: Timer1Pwm,
}

impl CountDown for Timer {
    type Time = u32;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        self.timer.
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        todo!()
    }
}

impl Periodic for Timer {}

impl SoftwareUart {
    pub fn new(
        tc1: attiny_hal::pac::TC1,
        rx: Pin<Input<Floating>, PB3>,
        tx: Pin<Output, PB4>,
    ) -> Self {
        let mut timer = Timer {
            timer: Timer1Pwm::new(tc1, attiny_hal::simple_pwm::Prescaler::Prescale1024),
        };
        let serial = Serial::new(tx, rx, timer);

        SoftwareUart { serial }
    }

    pub fn receive(&mut self) {
        self.serial.read()
    }
}
