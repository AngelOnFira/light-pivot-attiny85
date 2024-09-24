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
    serial: Serial<Pin<Output, PB4>, Pin<Input, PB3>, Timer<attiny_hal::clock::MHz16>>,
}

pub struct Timer<SPEED> {
    delay: Delay<SPEED>,
    duration: u32,
    started: bool,
}

impl<SPEED> Timer<SPEED> {
    pub fn new(delay: Delay<SPEED>) -> Self {
        Timer {
            delay,
            duration: 0,
            started: false,
        }
    }
}

impl<SPEED> CountDown for Timer<SPEED> {
    type Time = u32;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        self.duration = count.into();
        self.started = true;
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.started {
            self.delay.delay_us(self.duration);
            self.started = false;
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }
}

impl<SPEED> Periodic for Timer<SPEED> {}

impl SoftwareUart {
    pub fn new(
        tc1: attiny_hal::pac::TC1,
        rx: Pin<Input<Floating>, PB3>,
        tx: Pin<Output, PB4>,
    ) -> Self {
        // let delay = Delay::<C>::new();
        // let timer = Timer::new(delay);
        let mut timer = Timer1Pwm::new(tc1, attiny_hal::simple_pwm::Prescaler::Prescale1024);
        let serial = Serial::new(tx, rx, timer).unwrap();

        SoftwareUart { serial }
    }

    pub fn receive(&mut self) -> Result<u8, Error> {
        self.serial.read()
    }
}
