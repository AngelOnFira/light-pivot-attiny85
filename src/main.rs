#![no_main]
#![no_std]
#![feature(abi_avr_interrupt)]

use avr_device::{
    attiny85::Peripherals,
    interrupt::{self, free, Mutex},
};
use core::cell::RefCell;
use attiny_hal::port::Pin;
use attiny_hal::port::mode::{Output, PwmOutput};
use attiny_hal::prelude::*;

mod uart;
mod buffer;
mod pwm;

use uart::Uart;
use buffer::Buffer;
use pwm::Pwm;

static UART: Mutex<RefCell<Option<Uart>>> = Mutex::new(RefCell::new(None));
static BUFFER: Mutex<RefCell<Buffer>> = Mutex::new(RefCell::new(Buffer::new()));

#[avr_device::entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let pins = attiny_hal::Pins::new(dp.PORTB);

    // Set up UART
    let uart = Uart::new(pins.pb3, pins.pb4);
    free(|cs| UART.borrow(cs).replace(Some(uart)));

    // Set up PWM for servos and light
    let mut pwm = Pwm::new(dp.TC0, dp.TC1);
    let base_servo = pwm.get_output(&pins.pb0);
    let tilt_servo = pwm.get_output(&pins.pb1);
    let light = pwm.get_output(&pins.pb2);

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
