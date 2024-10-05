use attiny_hal::pac::TC1;
use attiny_hal::port::mode::{Floating, Input, Output};
use attiny_hal::port::{Pin, PB3, PB4};
use attiny_hal::prelude::_embedded_hal_serial_Read;
// use attiny_hal::prelude::_embedded_hal_serial_Write;
use avr_device::asm::nop;
use bitbang_hal::serial::{Error, Nop, Reset, Serial};
use core::convert::Infallible;
use embedded_hal::timer::{CountDown, Periodic};
use nb::Error::WouldBlock;
use void::Void;

pub struct SoftwareUart {
    serial: Serial<Pin<Output, PB4>, Pin<Input<Floating>, PB3>, Timer>,
}

pub struct Timer {
    inner: TC1,
}

impl Timer {
    pub fn new(timer: TC1) -> Self {
        Self { inner: timer }
    }
}

impl CountDown for Timer {
    type Time = u8;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        let ticks = count.into();
        self.inner
            .tccr1
            .write(|w| w.cs1().prescale_8().ctc1().clear_bit());
        // Set compare value
        self.inner.ocr1a.write(|w| w.bits(ticks));
        // Clear counter
        self.inner.tcnt1.write(|w| w.bits(0));
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.inner.tcnt1.read().bits() < self.inner.ocr1a.read().bits() {
            Err(WouldBlock)
        } else {
            // Reset the counter
            self.inner.tcnt1.write(|w| w.bits(0));
            Ok(())
        }
    }
}

impl Periodic for Timer {}

impl Reset for Timer {
    fn reset(&mut self) {
        self.inner.tcnt1.write(|w| w.bits(0));
    }
}

impl Nop for Timer {
    fn nop(&mut self) {
        nop();
    }
}

impl SoftwareUart {
    pub fn new(tc1: TC1, rx: Pin<Input<Floating>, PB3>, tx: Pin<Output, PB4>) -> Self {
        let mut timer = Timer::new(tc1);
        timer.start(104u8);
        let serial = Serial::new(tx, rx, timer);

        SoftwareUart { serial }
    }

    pub fn receive(&mut self) -> nb::Result<u8, Error<Infallible>> {
        self.serial.read()
    }

    // pub fn send(&mut self, data: u8) -> nb::Result<(), Error<Infallible>> {
    //     self.serial.write(data)
    // }
}
