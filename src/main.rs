#![no_main]
#![no_std]
#![feature(abi_avr_interrupt)]

use attiny_hal::prelude::*;
use attiny_hal::simple_pwm::IntoPwmPin;
use attiny_hal::simple_pwm::{Prescaler, Timer0Pwm};
use avr_device::interrupt::{free, Mutex};
use core::cell::RefCell;
use panic_halt as _;

mod buffer;
// mod pwm;
mod uart;

use buffer::Buffer;
// use pwm::Pwm;
use uart::SoftwareUart;

// Define constants for baud rate and CPU frequency
const BAUD_RATE: u32 = 9600;
const CPU_FREQUENCY: u32 = 8_000_000; // Adjust this to match your ATtiny85's clock speed
const DEVICE_ID: u8 = 0x01;
static UART: Mutex<RefCell<Option<SoftwareUart<BAUD_RATE, CPU_FREQUENCY>>>> =
    Mutex::new(RefCell::new(None));
static BUFFER: Mutex<RefCell<Buffer>> = Mutex::new(RefCell::new(Buffer::new()));

#[avr_device::entry]
fn main() -> ! {
    let dp = attiny_hal::Peripherals::take().unwrap();
    let pins: attiny_hal::Pins = attiny_hal::pins!(dp);

    // Starting and initializing the timer with prescaling 64.
    // it gives one clock count every 4 µs.
    // since the clock register size is 8 bits, the timer is full every
    // 1/(16e6/64)*2^8 ≈ 10 ms
    //

    // We need to work with 50 Hz.
    // 8_000_000 / 128

    let tc0 = dp.TC0;
    tc0.tccr0a.write(|w| w.wgm0().ctc());
    tc0.ocr0a.write(|w| w.bits(77));
    let timer0 = Timer0Pwm::new(tc0, Prescaler::Prescale1024);

    // Set up UART with the new constant generic parameters
    let uart: SoftwareUart<BAUD_RATE, CPU_FREQUENCY> = SoftwareUart::new(
        dp.TC1,
        pins.pb3.into_floating_input(),
        pins.pb4.into_output(),
    );
    free(|cs| UART.borrow(cs).replace(Some(uart)));

    // Set up PWM for servos and light
    // let mut pwm = Pwm::new(dp.TC0, dp.TC1);
    let mut base_servo = pins.pb0.into_output().into_pwm(&timer0);
    let mut tilt_servo = pins.pb1.into_output().into_pwm(&timer0);
    let _light = pins.pb2.into_output();

    unsafe { avr_device::interrupt::enable() };

    loop {
        free(|cs| {
            let mut buffer = BUFFER.borrow(cs).borrow_mut();
            if buffer.len() >= 4 {
                let id_and_light = buffer.pop().unwrap();
                let rotation = buffer.pop().unwrap();
                let tilt = buffer.pop().unwrap();
                let _checksum = buffer.pop().unwrap(); // Assuming a 4th byte for checksum

                let id = (id_and_light & 0xF0) >> 4;
                let _light_state = id_and_light & 0x0F;

                // Check if this message is for this device
                if id == DEVICE_ID {
                    // Process the message
                    base_servo.set_duty(rotation);
                    tilt_servo.set_duty(tilt);
                    // Handle light state
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
            if let Ok(byte) = uart.receive() {
                BUFFER.borrow(cs).borrow_mut().push(byte);
            }
        }
    });
}
