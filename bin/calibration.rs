#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use attiny_hal::port::mode::Output;
use attiny_hal::port::Pin;
use attiny_hal::port::PB4;
use avr_device::interrupt::{self, Mutex};
use core::cell::Cell;
use panic_halt as _;

static PIN: Mutex<Cell<Option<Pin<Output, PB4>>>> = Mutex::new(Cell::new(None));
const OSCCAL_ADJUSTMENT: i16 = -8;

#[avr_device::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins = attiny_hal::pins!(dp);

    // Adjust OSCCAL
    let current_value = dp.CPU.osccal.read().bits();
    let new_value = (current_value as i16 + OSCCAL_ADJUSTMENT).clamp(0, 255) as u8;
    dp.CPU.osccal.write(|w| w.bits(new_value));

    loop {
        // avr_device::asm::sleep();
    }

    // Adjust OSCCAL
    let current_value = dp.CPU.osccal.read().bits();
    let new_value = (current_value as i16 + OSCCAL_ADJUSTMENT).clamp(0, 255) as u8;
    dp.CPU.osccal.write(|w| w.bits(new_value));

    // Set up Timer0 for PWM
    dp.TC0.tccr0a.write(|w| w.wgm0().ctc());
    dp.TC0.tccr0b.write(|w| w.cs0().prescale_8());
    dp.TC0.ocr0a.write(|w| w.bits(99));
    dp.TC0.timsk.write(|w| w.ocie0a().set_bit());

    let pin = pins.pb4.into_output();
    interrupt::free(|cs| PIN.borrow(cs).set(Some(pin)));

    // Enable global interrupts
    unsafe { interrupt::enable() };

    loop {
        avr_device::asm::sleep();
    }
}

#[avr_device::interrupt(attiny85)]
fn TIMER0_COMPA() {
    interrupt::free(|cs| {
        if let Some(mut pin) = PIN.borrow(cs).take() {
            // Toggle PB4
            pin.toggle();

            // Store pin back in the global variable
            PIN.borrow(cs).set(Some(pin));
        }
    });
}
