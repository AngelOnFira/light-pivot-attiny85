#![no_main]
#![no_std]
#![feature(abi_avr_interrupt)]

use attiny_hal::port::mode::{Output, PwmOutput};
use attiny_hal::port::Pin;
use attiny_hal::prelude::*;
use avr_device::{
    attiny85::Peripherals,
    interrupt::{self, free, Mutex},
};
use core::cell::RefCell;

mod buffer;
mod pwm;
mod uart;

use buffer::Buffer;
use pwm::Pwm;
use uart::Uart;

static UART: Mutex<RefCell<Option<Uart>>> = Mutex::new(RefCell::new(None));
static BUFFER: Mutex<RefCell<Buffer>> = Mutex::new(RefCell::new(Buffer::new()));

#[avr_device::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins: attiny_hal::Pins = attiny_hal::pins!(dp);
    // Set up UART
    let uart = Uart::new(pins.pb3.into_floating_input(), pins.pb4.into_output());
    free(|cs| UART.borrow(cs).replace(Some(uart)));

    // Set up PWM for servos and light
    let mut pwm = Pwm::new(dp.TC0, dp.TC1);
    let base_servo = pwm.get_output(&pins.pb0.into_output());
    let tilt_servo = pwm.get_output(&pins.pb1.into_output());
    let light = pwm.get_output(&pins.pb2.into_output());

    unsafe { avr_device::interrupt::enable() };

    loop {
        free(|cs| {
            if let Some(byte) = BUFFER.borrow(cs).borrow_mut().pop() {
                // Process received byte
                match byte {
                    b'0'..=b'9' => {
                        let value = (byte - b'0') as u8;
                        base_servo.set_duty(value * 25);
                    }
                    b'A'..=b'J' => {
                        let value = (byte - b'A') as u8;
                        tilt_servo.set_duty(value * 25);
                    }
                    b'K'..=b'T' => {
                        let value = (byte - b'K') as u8;
                        light.set_duty(value * 25);
                    }
                    _ => {}
                }
            }
        });

        // Sleep to save power
        avr_device::asm::sleep();
    }
}

#[avr_device::interrupt(attiny85)]
fn USI_START() {
    free(|cs| {
        if let Some(uart) = UART.borrow(cs).borrow_mut().as_mut() {
            if let Some(byte) = uart.receive() {
                BUFFER.borrow(cs).borrow_mut().push(byte);
            }
        }
    });
}
